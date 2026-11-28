"""방향 보정 — 수동 에임 LED, 나침반 자동 에임(셀피 정면 = 0°)"""

import threading
import time

from .config import _AIM_LABEL
from .hw import _idle_later, _set_led
from .nav import _reset_origin
from .state import _state_lock, log_event, rt, state


_aim_heading = {"floor2": 0, "rear": 0, "floor1": 0}

# 자동 에임 마지막 회전 — 한 번에 보내면 BOLT가 최고 속도로 확 도므로 조금씩 나눠 보냄
AIM_TURN_STEP_DEG = 10      # 한 번에 돌리는 각도
AIM_TURN_STEP_SEC = 0.08    # 다음 각도를 보내기까지 간격 (10° / 0.08초 ≈ 초당 125°)


def _slow_turn(bolt, from_h, to_h):
    """from_h → to_h로 가까운 쪽으로 조금씩 제자리 회전 (속도 0)"""
    diff = ((to_h - from_h + 540) % 360) - 180
    steps = max(1, int(abs(diff) // AIM_TURN_STEP_DEG) + 1)
    for i in range(1, steps + 1):
        bolt.drive_control.roll_start(int(round(from_h + diff * i / steps)) % 360, 0)
        time.sleep(AIM_TURN_STEP_SEC)

def _set_aim_led(bolt):
    """방향 보정용: 앞 LED 끄고 꼬리 LED만 파랗게 — 꼬리 반대쪽이 로봇 정면"""
    if bolt is None:
        return
    try:
        from spherov2.toy.bolt import BOLT as _B
        L = _B.LEDs
        bolt.multi_led_control.set_leds({
            L.FRONT_RED: 0, L.FRONT_GREEN: 0, L.FRONT_BLUE: 0,
            L.BACK_RED: 0, L.BACK_GREEN: 0, L.BACK_BLUE: 255,
        })
    except Exception as e:
        log_event(f"  [LED 오류] {e}")

PHONE_HEADING_MAX_AGE = 5.0    # 초 — 이보다 오래된 폰 방위는 쓰지 않음

PHONE_MIN_QUALITY     = 0.5    # 폰이 이보다 눕혀져 있으면 방위가 부정확

NORTH_TIMEOUT_SEC     = 20.0

# BOLT 북쪽 yaw와 폰 방위의 기준 차이 보정 — 0이면 로봇 정면이 셀피 방향 (현장 확인). 어긋나면 이 값만 조정
AIM_AZIMUTH_OFFSET    = 0

def _set_aim_status(key, status, msg):
    with _state_lock:
        state["aim"][key] = {"status": status, "msg": msg}

def _auto_aim(key, azimuth):
    """나침반 자동 에임: 북쪽 찾기 → (북쪽 yaw + 셀피 방위)로 제자리 회전 → 그 방향을 0°로 저장"""
    label = _AIM_LABEL[key]
    bolt = {"floor2": rt.bolt_2f, "rear": rt.bolt_r2, "floor1": rt.bolt_1f}[key]
    try:
        _set_aim_status(key, "running", "북쪽 찾는 중…")
        _set_aim_led(bolt)
        if bolt is not None and hasattr(bolt, "magnetometer_calibrate_to_north"):
            got, evt = {}, threading.Event()
            def _on_north(yaw):
                got["yaw"] = yaw
                evt.set()
            bolt.add_magnetometer_north_yaw_notify_listener(_on_north)
            try:
                bolt.magnetometer_calibrate_to_north()
                if not evt.wait(NORTH_TIMEOUT_SEC):
                    raise TimeoutError(f"북쪽 찾기 {NORTH_TIMEOUT_SEC:.0f}초 초과")
            finally:
                bolt.remove_magnetometer_north_yaw_notify_listener(_on_north)
            north = int(got["yaw"]) % 360
        elif state["sim_mode"]:
            time.sleep(2.0)
            north = 0
        else:
            raise RuntimeError("나침반 미지원 로봇")
        trim = state["aim_trim"][key]
        target = int(round(north + azimuth + AIM_AZIMUTH_OFFSET + trim)) % 360
        _set_aim_status(key, "running", f"셀피 방위 {azimuth:.0f}°로 회전 중…")
        if bolt is not None:
            _slow_turn(bolt, 0, target)            # 지금 정면(0°)에서 목표까지 천천히
            time.sleep(0.8)                        # 마지막 각도에 자리 잡을 때까지
            bolt.drive_control.roll_stop(target)
            time.sleep(0.5)
            if hasattr(bolt.drive_control, "reset_heading"):
                bolt.drive_control.reset_heading()
        _aim_heading[key] = 0
        _reset_origin(key, bolt)
        _set_led(bolt, 0, 255, 0)
        _idle_later(key)                     # ⓪ 완료 표시 잠깐 → 평소 대기(꺼짐)
        msg = f"완료 · 북 {north}° + 셀피 {azimuth:.0f}°" + (f" + 보정 {trim:+d}°" if trim else "")
        _set_aim_status(key, "done", msg)
        log_event(f"🧭 {label} 나침반 자동 에임 {msg} → 지금 정면 = 0°")
    except Exception as e:
        _set_aim_status(key, "error", f"실패: {e}")
        log_event(f"🧭 {label} 나침반 자동 에임 실패: {e}")
