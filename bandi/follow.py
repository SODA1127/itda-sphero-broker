"""R2 따라가기 — R1 발자국 추종, 모서리 정지, IR 끊김·경로 이탈 감시"""

import math
import os
import tempfile
import time

from .config import FOLLOW_END_GAP_M, TARGET_MODES, _build_2f_paths
from .hw import _angle_of, _drive, _halt, _is_real, _loc_rot, _norm180, _pos, _raw, _set_led, _set_matrix
from .manual import _enter_manual
from .nav import (
    NAV_MIN_SPEED,
    NAV_SLOW_CM,
    _calibrate_axes,
    _common,
    _run_path,
    _run_path_time,
    _trail_lock,
    _wpts,
)
from .state import (
    _active,
    _mark_motion_end,
    _save_settings,
    _state_lock,
    _uturn_learned,
    log_event,
    raise_exception,
    rt,
    state,
)


# 공통 좌표 = R2 출발점 기준 (R1·R2는 같은 방향으로 에임, R1은 간격만큼 앞에 놓음).
# R1이 실제로 지나간 자리를 발자국으로 남기고, R2는 경로 거리로 항상 간격만큼 뒤 지점을 쫓는다.
FOLLOW_CRUMB_CM     = 3.0    # R1 발자국 간격

FOLLOW_LOOKAHEAD_CM = 30.0   # R2가 바라보는 앞쪽 거리 — 클수록 작은 옆 오차에 덜 꺾임 (모서리는 따로 멈춰 돎)
FOLLOW_DEADBAND_CM  = 4.0    # 경로선에서 이 안쪽으로 벗어난 건 무시하고 경로 방향으로 감 (출발 직후 갑자기 꺾이지 않게)

FOLLOW_TOL_CM       = 4.0    # 목표 간격까지 이만큼 남으면 멈춤
FOLLOW_CORNER_REACH_CM = 5.0    # 모서리에서 멈춘 뒤 이보다 많이 남았으면 마저 전진하고 돎
FOLLOW_CORNER_REACH_SEC = 4.0   # 마저 전진 제한 시간
FOLLOW_CORNER_SETTLE_MIN = 1.0  # 모서리 정지 후 꺾기 전 정지에 더해 최소 이만큼 더 멈춤 — 트롤리 밀림이 끝난 뒤 돌게
FOLLOW_CORNER_SETTLE_MAX = 3.0  # 아직 밀리고 있어도 이만큼 넘으면 진행
FOLLOW_CORNER_STILL_CM  = 1.0   # 0.4초 동안 이보다 덜 움직이면 밀림이 끝난 것으로 봄
FOLLOW_CORNER_AFTER_SEC = 0.8   # 제자리 회전 뒤 트롤리 흔들림이 멎을 때까지 기다림
FOLLOW_MIN_SPEED_RATIO = 0.75   # 감속해도 R2 속도의 이 비율 아래로는 안 내림 — 트롤리를 끌면 저속에서 바퀴가 섬

FOLLOW_STALL_SEC    = 6.0    # 남은 거리가 있는데 이만큼 전혀 못 나가면 R1과 끊긴 것으로 보고 수동 조작
FOLLOW_LAG_CM       = 100.0  # 따라가야 할 지점보다 이만큼 넘게 뒤처지고
FOLLOW_LAG_SEC      = 4.0    # 그 상태가 이만큼 이어지면 R1과 끊긴 것으로 보고 수동 조작

def _crumb_project(crumbs, p, idx):
    """발자국 경로에서 p와 가장 가까운 점의 경로 거리 — 뒤로는 조금만 허용"""
    best = (None, idx)
    for i in range(max(0, idx - 2), min(len(crumbs) - 1, idx + 60)):
        ax, ay, sa = crumbs[i]
        bx, by, sb = crumbs[i + 1]
        vx, vy = bx - ax, by - ay
        l2 = vx * vx + vy * vy
        t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * vx + (p[1] - ay) * vy) / l2))
        qx, qy = ax + vx * t, ay + vy * t
        d = math.hypot(p[0] - qx, p[1] - qy)
        if best[0] is None or d < best[0][0]:
            best = ((d, sa + (sb - sa) * t), i)
    return (best[0][1] if best[0] else crumbs[idx][2]), best[1]

def _crumb_point(crumbs, s_cm):
    for i in range(len(crumbs) - 1):
        ax, ay, sa = crumbs[i]
        bx, by, sb = crumbs[i + 1]
        if s_cm <= sb or i == len(crumbs) - 2:
            t = 0.0 if sb == sa else max(0.0, min(1.0, (s_cm - sa) / (sb - sa)))
            return (ax + (bx - ax) * t, ay + (by - ay) * t)
    return crumbs[-1][:2]

def _simplify(points, tol=6.0):
    """더글러스-포커 — 촘촘한 실제 궤적을 꺾임점만 남김 (복귀 때 멈춤 최소화)"""
    if len(points) < 3:
        return list(points)
    (ax, ay), (bx, by) = points[0], points[-1]
    vx, vy = bx - ax, by - ay
    l = math.hypot(vx, vy) or 1e-9
    far, idx = 0.0, 0
    for i, (x, y) in enumerate(points[1:-1], 1):
        d = abs((x - ax) * vy - (y - ay) * vx) / l
        if d > far:
            far, idx = d, i
    if far <= tol:
        return [points[0], points[-1]]
    return _simplify(points[:idx + 1], tol)[:-1] + _simplify(points[idx:], tol)

FOLLOW_CORNER_DEG     = 50.0   # R1 발자국이 이만큼 꺾이면 모서리로 봄

# 모서리 정지 거리 = 학습한 미끄러짐(_uturn_learned["corner_slide"]) — 모서리마다 실측해 갱신

FOLLOW_TRACE = os.path.join(tempfile.gettempdir(), "bandi_follow_trace.csv")

def _trace(line):
    """R2 따라가기 진단 기록 — None이면 새로 시작"""
    try:
        with open(FOLLOW_TRACE, "w" if line is None else "a", encoding="utf-8") as f:
            if line is not None:
                f.write(line + "\n")
    except OSError:
        pass

def _crumb_corner(crumbs, s_from, s_to):
    """s_from~s_to 안에서 발자국이 가장 크게 꺾이는 지점 → (경로 거리, 꺾인 뒤 방향) 또는 None"""
    end = crumbs[-1][2]
    best, s = None, s_from
    while s <= min(s_to, end - 15):
        a = _crumb_point(crumbs, max(0.0, s - 15))
        b = _crumb_point(crumbs, s)
        c = _crumb_point(crumbs, s + 15)
        if math.hypot(b[0] - a[0], b[1] - a[1]) > 5 and math.hypot(c[0] - b[0], c[1] - b[1]) > 5:
            h_out = _angle_of(c[0] - b[0], c[1] - b[1])
            turn = abs(_norm180(h_out - _angle_of(b[0] - a[0], b[1] - a[1])))
            if turn >= FOLLOW_CORNER_DEG and (best is None or turn > best[0]):
                best = (turn, s, h_out)
            elif best is not None:
                break                       # 모서리 지나감 — 가장 크게 꺾인 지점 확정
        s += 3
    return None if best is None else (best[1], best[2])

# R1이 IR을 송출하고 R2가 수신기로 읽는다. 따라가다 IR_LOST_SEC 동안 R1이 안 잡히거나
# R2가 R1 발자국에서 OFFPATH_CM 넘게 벗어나면 자동 주행을 멈추고 대시보드 수동 조작으로 넘긴다.
IR_FAR_CH, IR_NEAR_CH = 1, 0   # R1 IR 송출 채널 (원거리 1~3m / 근거리 1m 미만)

IR_POLL_SEC    = 0.3           # R2 IR 수신기 확인 주기

IR_LOST_SEC    = 2.0           # 이만큼 R1 IR이 안 잡히면 끊김

IR_ARM_SEC     = 5.0           # 따라가기 시작 후 이 안에 한 번도 안 잡히면 IR 감지 끔 (센서 미지원·거리 초과)

OFFPATH_CM     = 40.0          # R1 발자국에서 이만큼 벗어나면 경로 이탈

OFFPATH_SEC    = 1.0           # 이탈이 이만큼 이어지면 전환

def _ir_cmd(fn, bolt, *args):
    """BOLT IR 명령 — spherov2가 BOLT용 프로세서를 안 붙여 둬서 직접 지정"""
    from spherov2.controls.v2 import Processors
    return fn(bolt, *args, proc=Processors.SECONDARY)

def _ir_begin():
    """R1 IR 송출 시작 — 실제 R1·R2가 모두 있을 때만"""
    if not (_is_real(rt.bolt_2f) and _is_real(rt.bolt_r2)):
        return False
    try:
        from spherov2.commands.sensor import Sensor
        _ir_cmd(Sensor.start_robot_to_robot_infrared_broadcasting, rt.bolt_2f, IR_FAR_CH, IR_NEAR_CH)
        return True
    except Exception as e:
        log_event(f"  📡 R1 IR 송출 실패 — IR 끊김 감지 꺼짐 ({e})")
        return False

def _ir_end():
    if _is_real(rt.bolt_2f):
        try:
            from spherov2.commands.sensor import Sensor
            _ir_cmd(Sensor.stop_robot_to_robot_infrared_broadcasting, rt.bolt_2f)
        except Exception:
            pass

def _ir_seen():
    """R2 IR 수신기 중 하나라도 R1 신호를 보면 True, 읽기 실패면 None"""
    try:
        from spherov2.commands.sensor import Sensor
        r = _ir_cmd(Sensor.get_bot_to_bot_infrared_readings, rt.bolt_r2)
        return any(bool(v) for v in r)
    except Exception:
        return None

def _set_ir(status):
    with _state_lock:
        state["ir"] = status

def _run_r2_follow(gen, fallback_path=None):
    """R2가 R1 발자국을 간격만큼 뒤에서 따라감. R1이 끝나면 도착 간격까지 다가가 멈춤"""
    key, bolt, label = "rear", rt.bolt_r2, "R2"
    cancelled = lambda: gen != rt.run_gen or state["manual"]["on"]
    if state["manual"]["on"]:
        log_event(f"  🎮 {label} 수동 조작 중 — 자동 따라가기 생략")
        return False
    gap = state["follow_gap"] * 100
    end_gap = min(FOLLOW_END_GAP_M, state["follow_gap"]) * 100
    speed = min(255, int(state["speeds"][key] * TARGET_MODES[state["target"]]["mps"]))
    min_spd = max(NAV_MIN_SPEED, int(speed * FOLLOW_MIN_SPEED_RATIO))
    # R2가 출발하는 순간 R1이 실제로 있던 자리 — R2는 먼저 이 점으로 간 뒤 R1이 지나간 길을 따라간다
    # (R1이 직전 복귀 오차 등으로 '정면 간격만큼 앞'에 정확히 있지 않아도 흐름대로 따라가게)
    r1_now = _pos("floor2", rt.bolt_2f) or _raw("floor2")
    p0 = _pos(key, bolt)
    if p0 is None and _is_real(bolt) and _raw(key) is not None and _loc_rot[key] is None:
        p0 = _calibrate_axes(key, bolt, 0, speed, cancelled)    # 0° = R1 출발점 쪽 (경로 위)
    if p0 is None:
        log_event(f"  ⏱ {label} 위치 센서 없음 — 계획 경로로 주행")
        # 위치 센서가 없으면 시간 기반으로 — 테스트 경로면 테스트 경로를, 아니면 2층 대피 경로를
        path_r2 = fallback_path or _build_2f_paths(state["follow_gap"])[1]
        return _run_path_time(bolt, path_r2, key, label, (255, 180, 0), (255, 0, 0), gen)
    r1_start = _common("floor2", r1_now) if r1_now is not None else (p0[0], p0[1] + gap)
    d0 = math.hypot(r1_start[0] - p0[0], r1_start[1] - p0[1]) or gap
    crumbs = [(p0[0], p0[1], 0.0), (r1_start[0], r1_start[1], d0)]   # R2 출발점 → R1이 출발 전 있던 자리
    with _trail_lock:
        if not _wpts[key]:
            _wpts[key].append(p0)
    track = [p0]
    with _state_lock:
        state[key].update(moving=True, done=False, step=1, progress_m=0.0)
    _set_led(bolt, 255, 180, 0)
    _set_matrix(bolt, 255, 180, 0, "^")
    log_event(f"  📍 {label} R1 좌표 따라가기 — 간격 {gap:g}cm, 도착 간격 {end_gap:g}cm · "
              f"먼저 R1 출발 자리 ({r1_start[0]:.0f}, {r1_start[1]:.0f})로 {d0:.0f}cm")
    _trace(None)
    _trace("t,R1x,R1y,R1s,R2x,R2y,R2s,target_s,heading,moving,leader_done")
    r2_s, idx, moving, last_s, last_t, cur_h = 0.0, 0, False, 0.0, time.time(), 0
    ok, leader_seen, t_start = False, False, time.time()
    corner_done_s = -1e9            # 이미 멈춰 돈 모서리 (다시 잡지 않게)
    ir_on = _ir_begin()
    ir_t0, ir_last, ir_poll = time.time(), None, 0.0
    prog_s, prog_t, lag_since = 0.0, time.time(), None
    off_since = None
    _set_ir("wait" if ir_on else "off")

    def feed():
        lp = _common("floor2", _pos("floor2", rt.bolt_2f))
        if lp is not None and state["floor2"]["moving"]:
            lx, ly, ls = crumbs[-1]
            d = math.hypot(lp[0] - lx, lp[1] - ly)
            if d >= FOLLOW_CRUMB_CM:
                crumbs.append((lp[0], lp[1], ls + d))

    while True:
        if cancelled():
            break
        feed()
        if state["floor2"]["moving"]:
            leader_seen = True
        # R1이 실제로 출발한 걸 본 뒤에 멈추면 '끝'. 3초 안에 출발 안 하면 R1 없이 끝난 것으로
        leader_done = (leader_seen and not state["floor2"]["moving"]) or (not leader_seen and time.time() - t_start > 3)
        target_s = crumbs[-1][2] - (end_gap if leader_done else gap)
        p = _pos(key, bolt)
        if p is None:
            log_event(f"⚠ {label} 위치 신호 끊김 — 따라가기 중단")
            raise_exception("E4", f"{label} 위치 센서 끊김")
            break
        r2_s, idx = _crumb_project(crumbs, p, idx)
        # 경로 이탈 1: R1 발자국에서 벗어남 (위치)
        qx, qy = _crumb_point(crumbs, r2_s)
        off = math.hypot(p[0] - qx, p[1] - qy)
        if off > OFFPATH_CM:
            off_since = off_since or time.time()
            if time.time() - off_since > OFFPATH_SEC:
                state["manual"]["heading"] = cur_h
                _enter_manual(f"경로 이탈 {off:.0f}cm", auto=True)
                break
        else:
            off_since = None
        # 경로 이탈 2: R1 IR 끊김
        if ir_on and time.time() - ir_poll >= IR_POLL_SEC:
            ir_poll = time.time()
            seen = _ir_seen()
            if seen:
                ir_last = ir_poll
                _set_ir("ok")
            if ir_last is None and ir_poll - ir_t0 > IR_ARM_SEC:
                ir_on = False
                _set_ir("off")
                log_event(f"  📡 {label} IR로 R1이 한 번도 안 잡힘 — IR 감지 끄고 위치 이탈만 감시")
            elif ir_last is not None and ir_poll - ir_last > IR_LOST_SEC:
                _set_ir("lost")
                state["manual"]["heading"] = cur_h
                _enter_manual(f"IR {IR_LOST_SEC:g}초 끊김 — R1 인식 안 됨", auto=True)
                break
        _trace(f"{time.time() - t_start:.2f},{crumbs[-1][0]:.0f},{crumbs[-1][1]:.0f},{crumbs[-1][2]:.0f},"
               f"{p[0]:.0f},{p[1]:.0f},{r2_s:.0f},{target_s:.0f},{cur_h},{int(moving)},{int(leader_done)}")
        with _state_lock:
            state[key]["progress_m"] = round(r2_s / 100, 2)
            state["floor2"]["progress_m"] = round(crumbs[-1][2] / 100, 2)
        if math.hypot(p[0] - track[-1][0], p[1] - track[-1][1]) >= 5:
            track.append(p)
        remaining = target_s - r2_s
        if remaining > last_s + 1 or r2_s > last_s + 1:
            last_s, last_t = r2_s, time.time()
        if remaining <= FOLLOW_TOL_CM:
            if moving:
                _halt(key, bolt, cur_h)
                moving = False
            if leader_done:
                ok = True
                break
            time.sleep(0.05)
            continue
        # R1과 끊김 1: 따라가야 할 지점보다 크게 뒤처짐 (막힘·바퀴 헛돎·사람이 붙잡음)
        now = time.time()
        if r2_s > prog_s + 3:
            prog_s, prog_t = r2_s, now
        if remaining > FOLLOW_LAG_CM:
            lag_since = lag_since or now
            if now - lag_since > FOLLOW_LAG_SEC:
                state["manual"]["heading"] = cur_h
                _enter_manual(f"R1과 간격 벌어짐 — {remaining:.0f}cm 뒤처짐", auto=True)
                break
        else:
            lag_since = None
        # R1과 끊김 2: 남은 거리가 있는데 전혀 못 나감
        if now - prog_t > FOLLOW_STALL_SEC:
            state["manual"]["heading"] = cur_h
            _enter_manual(f"R2 {FOLLOW_STALL_SEC:g}초 동안 못 나감 (남은 {remaining:.0f}cm)", auto=True)
            break
        # 모서리: R1처럼 앞에서 감속 → 멈춤 → 제자리에서 돌고 출발 (트롤리 관성으로 지나치지 않게)
        corner = _crumb_corner(crumbs, max(r2_s, corner_done_s + 20), r2_s + 40)
        if corner is not None and corner[0] > target_s:
            corner = None                       # 목표 지점 너머 모서리 — 지금은 상관없음
        if corner is not None:
            cs, new_h = corner
            to_c = cs - r2_s
            if to_c <= _uturn_learned["corner_slide"]:
                _halt(key, bolt, cur_h)
                moving = False
                stop_p = p
                log_event(f"  ↪ {label} 모서리 {to_c:.0f}cm 앞 정지 → {new_h:.0f}° 방향으로 회전")
                t_end = time.time() + state["turn_pause"]
                while time.time() < t_end and not cancelled():
                    feed()
                    time.sleep(0.05)
                # 트롤리 밀림이 끝날 때까지 더 멈춤 — 밀리는 중에 돌면 회전하며 더 밀려 모서리를 지나침
                t_settle = time.time()
                hist = []
                while not cancelled():
                    now2 = time.time()
                    q = _pos(key, bolt)
                    if q is not None:
                        hist.append((now2, q))
                        hist = [h for h in hist if now2 - h[0] <= 0.4]
                    waited = now2 - t_settle
                    still = (len(hist) >= 3 and hist[-1][0] - hist[0][0] >= 0.3 and
                             math.hypot(hist[-1][1][0] - hist[0][1][0], hist[-1][1][1] - hist[0][1][1]) < FOLLOW_CORNER_STILL_CM)
                    if waited >= FOLLOW_CORNER_SETTLE_MAX or (waited >= FOLLOW_CORNER_SETTLE_MIN and (still or q is None)):
                        break
                    feed()
                    time.sleep(0.05)
                log_event(f"  ⏸ {label} 모서리에서 {state['turn_pause'] + time.time() - t_settle:.1f}초 멈춤 (트롤리 밀림 끝난 뒤 회전)")
                # 멈춘 뒤 실제로 미끄러진 거리 → 다음 모서리 정지 거리에 반영
                after_p = _pos(key, bolt)
                if after_p is not None and _is_real(bolt):      # 시뮬레이션은 미끄러지지 않으므로 학습 안 함
                    ax, ay = _crumb_point(crumbs, max(0.0, cs - 20))
                    bx, by = _crumb_point(crumbs, cs)
                    seg = math.hypot(bx - ax, by - ay) or 1.0
                    slide = ((after_p[0] - stop_p[0]) * (bx - ax) + (after_p[1] - stop_p[1]) * (by - ay)) / seg
                    old = _uturn_learned["corner_slide"]
                    _uturn_learned["corner_slide"] = round(min(60.0, max(3.0, (old + max(0.0, slide)) / 2)), 1)
                    _save_settings()
                    log_event(f"  📏 {label} 모서리 정지 후 미끄러짐 {slide:.0f}cm → 다음 정지 거리 "
                              f"{_uturn_learned['corner_slide']:.0f}cm")
                    # 아직 모서리(R1이 꺾은 지점) 전이면 그 지점까지 마저 간 뒤 돈다 — 일찍 꺾지 않게
                    ux, uy = (bx - ax) / seg, (by - ay) / seg
                    left = (bx - after_p[0]) * ux + (by - after_p[1]) * uy
                    if left > FOLLOW_CORNER_REACH_CM:
                        log_event(f"  ↪ {label} 모서리까지 {left:.0f}cm 남음 — 마저 전진 후 회전")
                        t_end = time.time() + FOLLOW_CORNER_REACH_SEC
                        while time.time() < t_end and not cancelled():
                            q = _pos(key, bolt)
                            if q is None:
                                break
                            left = (bx - q[0]) * ux + (by - q[1]) * uy
                            if left <= FOLLOW_CORNER_REACH_CM:
                                break
                            _drive(key, bolt, round(_angle_of(bx - q[0], by - q[1])) % 360, min(min_spd, speed))
                            feed()
                            time.sleep(0.05)
                        _halt(key, bolt, cur_h)
                        t_end = time.time() + 0.3
                        while time.time() < t_end and not cancelled():
                            feed()
                            time.sleep(0.05)
                cur_h = round(new_h) % 360
                _drive(key, bolt, cur_h, 0)          # 제자리 회전
                t_end = time.time() + 0.6 + FOLLOW_CORNER_AFTER_SEC
                while time.time() < t_end and not cancelled():
                    feed()
                    time.sleep(0.05)
                corner_done_s = cs
                last_t = prog_t = time.time()      # 모서리에서 멈춘 시간은 정체로 세지 않음
                continue
            cx, cy = _crumb_point(crumbs, cs)
            cur_h = round(_angle_of(cx - p[0], cy - p[1]))
            spd = max(min(min_spd, speed), min(speed, int(speed * to_c / NAV_SLOW_CM)))
            _drive(key, bolt, cur_h, spd)
            moving = True
            time.sleep(0.05)
            continue
        tx, ty = _crumb_point(crumbs, min(r2_s + FOLLOW_LOOKAHEAD_CM, target_s))
        if off < FOLLOW_DEADBAND_CM:               # 경로선 위 — 경로 방향 그대로
            cur_h = round(_angle_of(tx - qx, ty - qy)) if math.hypot(tx - qx, ty - qy) > 1 else cur_h
        else:                                      # 벗어남 — 앞쪽 지점을 겨눠 돌아옴
            cur_h = round(_angle_of(tx - p[0], ty - p[1]))
        spd = speed if remaining > NAV_SLOW_CM else max(min(min_spd, speed), int(speed * remaining / NAV_SLOW_CM))
        _drive(key, bolt, cur_h, spd)
        moving = True
        time.sleep(0.05)
    _halt(key, bolt, cur_h)
    p = _pos(key, bolt)
    if p is not None:
        track.append(p)
    with _trail_lock:
        _wpts[key] = [_wpts[key][0]] + _simplify(track)[1:] if _wpts[key] else _simplify(track)
    with _state_lock:
        state[key].update(moving=False, step=0, done=ok, progress_m=None)
        state["floor2"]["progress_m"] = None
    _mark_motion_end()
    if ok:
        _set_led(bolt, 255, 0, 0)
        log_event(f"✅ {label} R1 뒤 {end_gap:g}cm 도착 (따라간 거리 {r2_s / 100:.2f}m)")
    elif state["manual"]["on"]:
        log_event(f"⏸ {label} 자동 따라가기 멈춤 — 수동 조작으로 넘김")
    elif cancelled():
        log_event(f"⏹ {label} 따라가기 중단 (상황 해제)")
    _ir_end()
    if state.get("ir") != "lost":
        _set_ir("off")
    return ok

def _run_r2_delayed(gen, delay, path):
    """R2는 delay초 뒤 출발 — 대기 중 상황 해제되면 출발하지 않음"""
    if delay > 0:
        log_event(f"  R2 {delay:g}초 뒤 출발")
        end = time.time() + delay
        while time.time() < end:
            if gen != rt.run_gen:
                return
            time.sleep(0.05)
    if _active("floor2"):
        return _run_r2_follow(gen)
    _run_path(rt.bolt_r2, path, "rear", "R2",
              move_color=(255, 180, 0), done_color=(255, 0, 0), gen=gen)
