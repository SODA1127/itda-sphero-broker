"""로봇 입출력 — LED·매트릭스, 주행 명령, 위치 센서(locator) 읽기·좌표 변환, 시뮬레이션 자세"""

import math
import time

from .config import DRIVE_MPS, SPEED_REF, _ROBOT_KEYS
from .state import log_event, rt, state


def _set_led(bolt, r: int, g: int, b: int):
    """전면+후면 RGB LED 설정 (multi_led_control API)"""
    if bolt is None:
        return
    try:
        from spherov2.toy.bolt import BOLT as _B
        bolt.multi_led_control.set_leds({
            _B.LEDs.FRONT_RED: r, _B.LEDs.FRONT_GREEN: g, _B.LEDs.FRONT_BLUE: b,
            _B.LEDs.BACK_RED:  r, _B.LEDs.BACK_GREEN:  g, _B.LEDs.BACK_BLUE:  b,
        })
    except Exception as e:
        log_event(f"  [LED 오류] {e}")

def _set_matrix(bolt, r: int, g: int, b: int, char: str = ""):
    """8×8 매트릭스 — char 있으면 문자, 없으면 단색 채우기"""
    if bolt is None:
        return
    try:
        if char:
            bolt.set_compressed_frame_player_single_character(r, g, b, char)
        else:
            bolt.set_compressed_frame_player_one_color(r, g, b)
    except Exception as e:
        log_event(f"  [Matrix 오류] {e}")

def _scroll_connected(bolt, floor_num: str, r: int, g: int, b: int):
    """연결 시 'Floor N' 스크롤 후 숫자 고정 표시 (별도 스레드에서 실행)"""
    text = f"Floor {floor_num}"
    for ch in text:
        _set_matrix(bolt, r, g, b, ch)
        time.sleep(0.35)
    # 스크롤 완료 → 숫자 고정
    _set_matrix(bolt, r, g, b, floor_num)

def _mps(key, speed):
    """속도 값 → 실제 m/s 추정 (R2는 거리 보정만큼 덜 감)"""
    v = DRIVE_MPS * speed / SPEED_REF
    if key == "rear":
        v /= state["r2_dist_pct"] / 100
    return v

# BOLT가 알려 주는 자기 위치(cm, x = 오른쪽 + / y = 앞 +)를 보며 목표 지점까지 간다.
# 시간으로만 가던 방식과 달리 미끄러짐·트롤리 무게·출발/정지 관성이 쌓이지 않고, 복귀는 좌표로 돌아간다.
LOC_INTERVAL_MS    = 100    # 위치 스트리밍 주기 (ms)

LOC_MAX_AGE        = 1.0    # 이보다 오래된 위치는 못 믿음 (초)

LOC_ROT_SNAP_DEG   = 15.0   # 좌표축 보정이 이보다 작으면 0° — 트롤리에 끌린 출발 흔들림을 보정값으로 오인하지 않게

_loc      = {k: None for k in _ROBOT_KEYS}                     # 원시 위치 (x, y cm, 받은 시각)

_loc_rot  = {k: None for k in _ROBOT_KEYS}                     # 위치 좌표축 → 헤딩 좌표 회전(도), 첫 구간에서 추정

_sim_pose = {k: [0.0, 0.0, 0, 0, time.time()] for k in _ROBOT_KEYS}  # SIM: x, y, 헤딩, 속도, 시각

_org_off  = {k: (0.0, 0.0) for k in _ROBOT_KEYS}               # 원점 리셋 뒤 '지금 자리' 좌표 (헤딩 좌표 cm) — 보통 (0, 0)

def _bolt_of(key):
    return {"floor2": rt.bolt_2f, "rear": rt.bolt_r2, "floor1": rt.bolt_1f}[key]

def _is_real(bolt):
    return bolt is not None and hasattr(bolt, "reset_locator_x_and_y")

def _vec(h):
    return (math.sin(math.radians(h)), math.cos(math.radians(h)))

def _angle_of(dx, dy):
    return math.degrees(math.atan2(dx, dy)) % 360

def _snap_rot(rot):
    """위치 좌표축은 원래 헤딩과 같음 — 작은 차이는 측정 흔들림으로 보고 0°"""
    return 0.0 if abs(rot) < LOC_ROT_SNAP_DEG else rot

def _norm180(a):
    return ((a + 180) % 360) - 180

def _unrot(x, y, rot):
    """위치 좌표축이 헤딩 좌표보다 rot도 돌아가 있을 때 헤딩 좌표로 되돌림"""
    r = math.radians(rot)
    return (x * math.cos(r) - y * math.sin(r), y * math.cos(r) + x * math.sin(r))

def _make_loc_listener(key):
    def _on(data):
        loc = data.get("locator") if isinstance(data, dict) else None
        if loc and "x" in loc and "y" in loc:
            _loc[key] = (float(loc["x"]), float(loc["y"]), time.time())
    return _on

def _sim_advance(key):
    p = _sim_pose[key]
    now = time.time()
    if p[3] != 0:                       # 음수 = 후진
        v = _mps(key, p[3]) * 100
        dx, dy = _vec(p[2])
        p[0] += dx * v * (now - p[4])
        p[1] += dy * v * (now - p[4])
    p[4] = now

def _drive(key, bolt, heading, speed):
    if not _is_real(bolt):
        _sim_advance(key)
        _sim_pose[key][2], _sim_pose[key][3] = heading % 360, speed
    if bolt is not None:
        bolt.drive_control.roll_start(int(heading) % 360, int(speed))

def _drive_reverse(key, bolt, heading, speed):
    """앞을 heading으로 둔 채 뒤로 — spherov2 roll_start(음수)는 180°를 더해 돌아선 뒤 원래 앞쪽으로 가 버림"""
    if not _is_real(bolt):
        _sim_advance(key)
        _sim_pose[key][2], _sim_pose[key][3] = heading % 360, -speed
    if _is_real(bolt):
        from spherov2.commands.drive import DriveFlags
        bolt.drive_with_heading(int(speed), int(heading) % 360, DriveFlags.BACKWARD)

def _halt(key, bolt, heading):
    if not _is_real(bolt):
        _sim_advance(key)
        _sim_pose[key][3] = 0
    if bolt is not None:
        try:
            bolt.drive_control.roll_stop(int(heading) % 360)
        except Exception:
            pass

def _raw(key):
    loc = _loc[key]
    if loc is None or time.time() - loc[2] > LOC_MAX_AGE:
        return None
    return (loc[0], loc[1])

def _pos(key, bolt):
    """헤딩 좌표계 현재 위치 (cm). 아직 못 믿으면 None"""
    if not _is_real(bolt):
        _sim_advance(key)
        return (_sim_pose[key][0], _sim_pose[key][1])
    raw = _raw(key)
    if raw is None or _loc_rot[key] is None:
        return None
    return _raw_to_pos(key, raw, _loc_rot[key])

def _raw_to_pos(key, raw, rot):
    """원시 위치 → 헤딩 좌표 (축 회전·R2 거리 보정·원점 좌표 반영)"""
    x, y = _unrot(raw[0], raw[1], rot)
    if key == "rear":
        # R2는 트롤리 때문에 바퀴가 미끄러져 위치 센서가 실제보다 많이 센다 — 'R2 거리 보정 %'만큼 줄여 실제 거리로
        k = 100.0 / state["r2_dist_pct"]
        x, y = x * k, y * k
    ox, oy = _org_off[key]
    return (x + ox, y + oy)

def _nav_available(key, bolt):
    return (not _is_real(bolt)) or _raw(key) is not None
