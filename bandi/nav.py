"""위치 기반 주행 — 경로 구간 주행(위치/시간 기반), 좌표축 보정, 평시 복귀(U턴 포함)"""

import math
import threading
import time

from .config import DRIVE_MPS, FOLLOW_END_GAP_M, SPEED_REF, TARGET_MODES, _AIM_LABEL, _ARROW, _ROBOT_KEYS, _path_meters
from .hw import (
    _angle_of,
    _bolt_of,
    _drive,
    _halt,
    _is_real,
    _loc_rot,
    _mps,
    _nav_available,
    _norm180,
    _org_off,
    _pos,
    _raw,
    _raw_to_pos,
    _scroll_connected,
    _set_led,
    _set_matrix,
    _sim_pose,
    _snap_rot,
    _vec,
)
from .state import (
    _active,
    _mark_motion_end,
    _save_settings,
    _state_lock,
    _test_running,
    _uturn_learned,
    log_event,
    raise_exception,
    rt,
    state,
)


# 로봇별 실제 주행 기록 [(헤딩, 속도, 초)] — 평시 복귀 때 역순·반대 방향으로 되짚는다
_trail = {"floor2": [], "rear": [], "floor1": []}

_trail_lock = threading.Lock()

def _clear_trail(key=None):
    with _trail_lock:
        for k in ([key] if key else list(_trail)):
            _trail[k] = []

R2_UTURN_SPEED = 80    # R2 복귀 끝 U턴 속도 — 트롤리를 끌고 돌 만큼

R2_UTURN_SEC   = 2.0   # R2 U턴 시간 — 짧을수록 작은 반원 (트롤리가 덜 돌면 늘림)

R2_UTURN_STEPS = 12    # 반원을 나누는 회전 단계 수 (180° / 12 = 15°씩)

R2_UTURN_COMP  = 1.0   # U턴 옆걸음 보정 배율 — 복귀 뒤 R2가 출발점 옆으로 비켜서면 조정 (U턴 방향 쪽이면 올림)

def _plan_return(key, trail):
    """복귀 계획: 기록을 역순·반대 방향으로 되짚되, R2는 끝의 반원 U턴이 옆으로 옮겨 놓는 거리만큼
    그 방향의 되짚기 구간을 미리 줄이거나 늘려 U턴 뒤 출발점에 서게 한다.
    반환: ([(헤딩, 속도, 초, 원래 기록 초)], U턴 방향 +1 시계 / -1 반시계 / None)"""
    legs = [((h + 180) % 360, spd, secs, secs) for h, spd, secs in reversed(trail)]
    if key != "rear" or not legs:
        return legs, None
    start_h = legs[-1][0]
    shift = 2 * _mps(key, R2_UTURN_SPEED) * R2_UTURN_SEC / math.pi * R2_UTURN_COMP   # 반원 지름
    best = None
    for sign in (-1, +1):
        d_dir = (start_h + 90 * sign) % 360          # U턴이 로봇을 옮겨 놓는 방향
        for i, (h, spd, secs, _) in enumerate(legs):
            v = _mps(key, spd)
            if h == d_dir and secs * v > shift + 0.1:
                cand = (0, sign, i, -shift / v)      # 그 방향 구간을 줄임 (지나온 곳 안에서 해결 — 우선)
            elif h == (d_dir + 180) % 360:
                cand = (1, sign, i, shift / v)       # 반대 방향 구간을 늘림
            else:
                continue
            if best is None or cand[0] < best[0]:
                best = cand
    if best is None:
        return legs, +1
    _, sign, i, dt = best
    h, spd, secs, orig = legs[i]
    legs[i] = (h, spd, round(secs + dt, 2), orig)
    return legs, sign

def _u_turn(bolt, from_heading, sign, gen):
    """천천히 움직이며 반원을 그려 방향을 180° 바꾼다 — 제자리 회전과 달리 트롤리도 함께 돈다"""
    step_sec = R2_UTURN_SEC / R2_UTURN_STEPS
    for i in range(1, R2_UTURN_STEPS + 1):
        if gen != rt.run_gen:
            break
        h = int(round(from_heading + sign * 180 * (i - 0.5) / R2_UTURN_STEPS)) % 360   # 구간 중간 각 → 대칭 반원 (Sphero는 정수 각만)
        if bolt is not None:
            bolt.drive_control.roll_start(h, R2_UTURN_SPEED)
        time.sleep(step_sec)
    if bolt is not None:
        bolt.drive_control.roll_stop(int(round(from_heading + 180)) % 360)

def _return_home_time(key, gen):
    """기록한 주행을 거꾸로 되짚어 출발점으로 돌아가 원래 방향(0°)으로 선다"""
    label = _AIM_LABEL[key]
    bolt = {"floor2": rt.bolt_2f, "rear": rt.bolt_r2, "floor1": rt.bolt_1f}[key]
    for _ in range(40):                      # 중단 중인 주행이 멈추고 기록을 마칠 때까지
        if not state[key]["moving"]:
            break
        time.sleep(0.05)
    with _trail_lock:
        trail = list(_trail[key])
    if sum(sec for _, _, sec in trail) <= 0:
        return
    legs, uturn = _plan_return(key, trail)
    total = sum(secs for _, _, secs, _ in legs)
    with _state_lock:
        state[key]["returning"] = True
        state[key]["return_secs"] = round(total + 0.5 * len(legs) + (R2_UTURN_SEC if uturn else 0), 1)
    _set_led(bolt, 255, 255, 255)
    log_event(f"↩ {label} 출발점으로 복귀 ({total:.1f}초 주행 되짚기"
              + (f", U턴 {'오른쪽' if uturn > 0 else '왼쪽'} 반원 옆걸음 보정" if uturn else "") + ")")
    try:
        for back, speed, secs, orig in legs:
            _set_matrix(bolt, 255, 255, 255, "^")
            if bolt is not None and secs > 0:
                bolt.drive_control.roll_start(back, speed)
            t0 = time.time()
            while time.time() - t0 < secs and gen == rt.run_gen:
                time.sleep(0.05)
            done = min(time.time() - t0, secs)
            if bolt is not None:
                bolt.drive_control.roll_stop(back)
            with _trail_lock:
                if _trail[key]:
                    if done >= secs - 0.01:
                        _trail[key].pop()
                    else:                    # 복귀 중 새 재난 → 남은 만큼 기록에 남김
                        h, spd, _ = _trail[key][-1]
                        left = orig * (1 - done / secs) if secs else orig
                        _trail[key][-1] = (h, spd, round(left, 2))
            if gen != rt.run_gen:
                log_event(f"↩ {label} 복귀 중단 (새 신호)")
                return
            time.sleep(0.5)
        if uturn:                            # 트롤리: 거꾸로 끌려왔으므로 움직이며 반바퀴 돌아 원래 방향으로
            _u_turn(bolt, legs[-1][0], uturn, gen)
        elif bolt is not None:               # 원래 방향으로 제자리 회전
            bolt.drive_control.roll_start(0, 0)
            time.sleep(1.0)
            bolt.drive_control.roll_stop(0)
        _set_led(bolt, 0, 255, 0)
        log_event(f"✅ {label} 출발점 복귀 완료")
    except Exception as e:
        log_event(f"↩ {label} 복귀 실패: {e}")
    finally:
        with _state_lock:
            state[key]["returning"] = False
        _mark_motion_end()

NAV_STOP_CM        = 4.0    # 목표까지 이만큼 남으면 정지 — 굴러가는 관성 고려

NAV_SLOW_CM        = 30.0   # 이 거리 안에서 감속

NAV_MIN_SPEED      = 50     # 감속 하한 속도 값 — 이보다 낮으면 Sphero가 거의 안 움직임 (설정 속도가 더 낮으면 그 값)

NAV_MAX_STEER_DEG  = 25     # 구간 방향에서 벗어나 보정할 수 있는 최대 각

NAV_CALIB_CM       = 15.0   # 첫 구간에서 위치 좌표축 회전을 추정할 최소 이동 거리

NAV_TIMEOUT_FACTOR = 3.0    # 예상 시간 × 이 값 + 3초가 지나도 도착 못 하면 다음 구간으로

NAV_HOME_TOL_CM    = 8.0    # 복귀 때 이 안이면 도착으로 봄

_wpts     = {k: [] for k in _ROBOT_KEYS}                       # 실제로 지난 지점 (헤딩 좌표 cm), [0] = 출발점

def _reset_origin(key, bolt):
    """에임·연결 직후 — 지금 자리를 위치 원점으로"""
    if _is_real(bolt):
        try:
            bolt.reset_locator_x_and_y()
        except Exception as e:
            log_event(f"  [위치 원점 리셋 오류] {e}")
    _loc_rot[key] = None
    _org_off[key] = (0.0, 0.0)
    _sim_pose[key][:4] = [0.0, 0.0, 0, 0]
    with _trail_lock:
        _wpts[key] = []
        _trail[key] = []

def _nav_to(key, bolt, ctx, target_of, heading, speed, cancelled, calib=True, hold=None):
    """ctx["start"] 기준 목표점까지 heading 방향 구간을 위치를 보며 주행.
    target_of(start) → 목표 (헤딩 좌표 cm). 반환 'ok' | 'cancel' | 'timeout' | 'noloc'"""
    u = _vec(heading)
    calib_from, deadline = None, None
    hist = []                                     # (시각, 구간 방향 위치) — 지금 속도 추정
    while True:
        if cancelled():
            return "cancel"
        if hold is not None and hold():           # 앞 로봇과 너무 가까움 → 멈춰 기다림
            _halt(key, bolt, heading)
            if deadline is not None:
                deadline += 0.05
            time.sleep(0.05)
            continue
        p = _pos(key, bolt)
        if p is None:
            raw = _raw(key)
            if raw is None:
                return "noloc"
            if _loc_rot[key] is None and calib:       # 축 회전 추정: 구간 방향으로 직진하며 실제 이동 방향을 잰다
                if calib_from is None:
                    calib_from = raw
                    deadline = time.time() + max(4.0, 0.3 / max(_mps(key, speed), 0.05) * 3)
                dx, dy = raw[0] - calib_from[0], raw[1] - calib_from[1]
                if math.hypot(dx, dy) >= NAV_CALIB_CM:
                    rot = _norm180(_angle_of(dx, dy) - heading)
                    rot = _snap_rot(rot)
                    _loc_rot[key] = rot
                    log_event(f"  📐 {_AIM_LABEL[key]} 위치 좌표축 보정 {rot:+.0f}°")
                    if ctx.get("start") is None and ctx.get("raw0") is not None:
                        ctx["start"] = _raw_to_pos(key, ctx["raw0"], rot)
                    deadline = None              # 보정용 제한 시간 → 구간 제한 시간으로 다시 계산
                    continue
                if time.time() > deadline:
                    return "noloc"
                _drive(key, bolt, heading, speed)
                time.sleep(0.05)
                continue
            return "noloc"
        if ctx.get("start") is None:
            ctx["start"] = p
        tx, ty = target_of(ctx["start"])
        rx, ry = tx - p[0], ty - p[1]
        along = rx * u[0] + ry * u[1]
        if deadline is None:
            exp = max(along, 0) / 100 / max(_mps(key, speed), 0.05)
            deadline = time.time() + exp * NAV_TIMEOUT_FACTOR + 3.0
        now = time.time()
        hist.append((now, p[0] * u[0] + p[1] * u[1]))
        while len(hist) > 2 and now - hist[0][0] > COAST_SPEED_WIN:
            hist.pop(0)
        dt = hist[-1][0] - hist[0][0]
        v = max(0.0, (hist[-1][1] - hist[0][1]) / dt) if dt >= 0.15 else 0.0    # cm/s
        coast = _coast_cm(key, v)        # 멈춘 뒤 미끄러질 만큼 일찍 멈춤 (R1·R3) — 빠를수록 많이
        if along <= NAV_STOP_CM + coast:
            ctx["stop"] = (p, v)
            return "ok"
        if time.time() > deadline:
            return "timeout"
        dist = math.hypot(rx, ry)
        dev = 0.0 if dist < 15 else max(-NAV_MAX_STEER_DEG, min(NAV_MAX_STEER_DEG, _norm180(_angle_of(rx, ry) - heading)))
        eff = along - coast
        spd = speed if eff > NAV_SLOW_CM else max(min(NAV_MIN_SPEED, speed), int(speed * eff / NAV_SLOW_CM))
        _drive(key, bolt, round(heading + dev), spd)
        time.sleep(0.05)

REST_SETTLE_SEC = 1.0          # 멈춘 뒤 미끄러짐이 끝날 때까지 기다리는 시간
COAST_LEARN_MIN_PAUSE = 0.8   # 꺾기 전 정지가 이보다 길어야 미끄러짐이 끝난 위치로 학습


COAST_REF_CMS   = DRIVE_MPS * 100   # 학습값 coast_*는 이 속도(cm/s, 기준 속도 100)에서 멈췄을 때 미끄러지는 거리
COAST_SPEED_WIN = 0.4               # 지금 속도를 이 시간(초) 동안의 이동으로 추정
COAST_LEARN_MIN_V = 25.0            # 이보다 느리게 멈추면 미끄러짐이 작아 학습에 안 씀 (cm/s)


def _coast_cm(key, v):
    """속도 v(cm/s)로 멈출 때 미끄러질 거리 — 운동 에너지처럼 속도 제곱에 비례.
    감속해 천천히 멈춘 구간을 빠르게 멈춘 구간만큼 일찍 세우면 못 미친다 (실측: 같은 학습값에 24cm vs 3cm)"""
    ref = _uturn_learned.get(f"coast_{key}", 0.0)
    return ref * min(2.0, (v / COAST_REF_CMS) ** 2) if ref else 0.0


def _learn_coast(key, label, goal, pos, pause, stop=None):
    """멈춘 뒤 실제로 미끄러진 거리를 그때 속도로 나눠 기준 속도 미끄러짐을 학습 (R1·R3)"""
    if key not in ("floor2", "floor1") or goal is None or pos is None or pause < COAST_LEARN_MIN_PAUSE:
        return
    (gx, gy), (ux, uy) = goal
    over = (pos[0] - gx) * ux + (pos[1] - gy) * uy        # +: 목표를 지나침, -: 못 미침
    if stop is None or stop[1] < COAST_LEARN_MIN_V:
        if abs(over) >= 3:
            log_event(f"  📏 {label} 구간 끝 목표를 {over:+.0f}cm {'지나침' if over > 0 else '못 미침'} (천천히 멈춤 — 학습 안 함)")
        return
    (sx, sy), v = stop
    slide = max(0.0, (pos[0] - sx) * ux + (pos[1] - sy) * uy)
    implied = slide / (v / COAST_REF_CMS) ** 2
    name = f"coast_{key}"
    old = _uturn_learned[name]
    _uturn_learned[name] = round(min(50.0, max(0.0, (old + implied) / 2)), 1)
    _save_settings()
    log_event(f"  📏 {label} {v:.0f}cm/s에서 멈춰 {slide:.0f}cm 미끄러짐 (목표 {over:+.0f}cm) → "
              f"기준 속도 미끄러짐 {_uturn_learned[name]:.0f}cm (지금 속도면 {_coast_cm(key, v):.0f}cm 일찍 멈춤)")


def _run_path_nav(bolt, path, state_key, label, move_color, done_color, gen=None):
    """위치 기반 경로 실행 — 구간 끝 지점을 좌표로 정해 놓고 도착할 때까지 보정하며 간다"""
    def cancelled():
        return gen is not None and gen != rt.run_gen

    def mark(p):
        with _trail_lock:
            if not _wpts[state_key] and ctx.get("start") is not None:
                _wpts[state_key].append(ctx["start"])     # 축 보정 뒤에야 출발점이 정해진 경우
            if p is not None:
                _wpts[state_key].append(p)

    def finish(ok):
        _halt(state_key, bolt, prev_heading or 0)
        if ok and _is_real(bolt):
            time.sleep(REST_SETTLE_SEC)        # 미끄러짐이 끝난 실제 위치를 기록 (평시 복귀 지점)
            _learn_coast(state_key, label, last_goal, _pos(state_key, bolt), REST_SETTLE_SEC, last_stop)
        mark(_pos(state_key, bolt))
        with _state_lock:
            state[state_key]["moving"] = False
            state[state_key]["step"]   = 0
            if ok:
                state[state_key]["done"] = True
        _mark_motion_end()
        if not ok:
            log_event(f"⏹ {label} 주행 중단 (상황 해제)")
        return ok

    with _state_lock:
        state[state_key]["moving"] = True
        state[state_key]["done"]   = False
        state[state_key]["step"]   = 0
    _set_led(bolt, *move_color)
    factor = TARGET_MODES[state["target"]]["mps"]
    speed = min(255, int(state["speeds"][state_key] * factor))
    turn_pause = state["turn_pause"]
    meters = _path_meters(path)
    offs = [(0.0, 0.0)]
    for (spd, h, dur, ch), m in zip(path, meters):
        if m > 0:
            dx, dy = _vec(h)
            offs.append((offs[-1][0] + dx * m * 100, offs[-1][1] + dy * m * 100))
    ctx = {"start": _pos(state_key, bolt), "raw0": _raw(state_key) if _is_real(bolt) else None}
    with _trail_lock:
        if not _wpts[state_key] and ctx["start"] is not None:
            _wpts[state_key].append(ctx["start"])
    prev_heading, leg, last_goal, last_stop = None, 0, None, None
    log_event(f"  📍 {label} 위치 기반 주행")

    for i, (spd, heading, dur, char) in enumerate(path):
        if cancelled():
            return finish(False)
        m = meters[i]
        if m > 0 and prev_heading is not None and heading != prev_heading:
            _halt(state_key, bolt, prev_heading)
            if turn_pause > 0:
                turn = (heading - prev_heading) % 360
                _set_matrix(bolt, *move_color, _ARROW.get(turn, "^"))
                log_event(f"  ↪ {label} {'좌' if turn == 270 else '우' if turn == 90 else ''}회전 전 {turn_pause:g}초 정지")
                end = time.time() + turn_pause
                while time.time() < end and not cancelled():
                    time.sleep(0.05)
                if cancelled():
                    return finish(False)
                if _is_real(bolt):            # 시뮬레이션 로봇은 미끄러지지 않으므로 학습 안 함
                    _learn_coast(state_key, label, last_goal, _pos(state_key, bolt), turn_pause, last_stop)
            elif _is_real(bolt):
                time.sleep(REST_SETTLE_SEC)
            # 꺾은 지점 = 멈춰서 미끄러짐이 끝난 위치 — 정지 명령 순간 위치는 미끄러짐만큼 앞이라 복귀 때 대각선이 됨
            mark(_pos(state_key, bolt))
        with _state_lock:
            state[state_key]["step"] = i + 1
        _set_matrix(bolt, *move_color, "^" if m > 0 else char)
        if m <= 0:
            log_event(f"  → {label} 정지 (도착)  [{char}]")
            continue
        prev_heading = heading
        log_event(f"  → {label} step {i+1}/{len(path)}: 속도={speed}  방향={heading}°  {m:g}m  [{char}]")
        if i == 0 and state_key in ("floor2", "rear") and state["latency"] is None and not _test_running["on"]:
            lat = round(time.time() - rt.trigger_ts, 2)
            with _state_lock:
                state["latency"] = lat
            log_event(f"⏱ 트리거→선두 출발 {lat:.1f}s (목표 ≤ 2.0s)")
        k = leg + 1
        t0 = time.time()
        res = _nav_to(state_key, bolt, ctx, lambda st, k=k: (st[0] + offs[k][0], st[1] + offs[k][1]),
                      heading, speed, cancelled)
        if ctx.get("start") is not None:
            last_goal = ((ctx["start"][0] + offs[k][0], ctx["start"][1] + offs[k][1]), _vec(heading))
        last_stop = ctx.pop("stop", None) if res == "ok" else None
        with _trail_lock:
            _trail[state_key].append((heading, speed, round(time.time() - t0, 2)))
        leg += 1
        if res == "cancel":
            return finish(False)
        if res == "noloc":
            log_event(f"⚠ {label} 위치 신호 끊김 — 남은 구간은 시간 기반")
            raise_exception("E4", f"{label} 위치 센서 끊김")
            if not _run_path_time(bolt, path[i + 1:], state_key, label, move_color, done_color, gen):
                return False
            return True
        if res == "timeout":
            log_event(f"⚠ {label} {m:g}m 구간 시간 초과 — 다음 구간으로")
        p = _pos(state_key, bolt)
        if p is not None and ctx.get("start") is not None:
            tx, ty = ctx["start"][0] + offs[k][0], ctx["start"][1] + offs[k][1]
            log_event(f"    {label} 구간 도착 오차 {math.hypot(tx - p[0], ty - p[1]):.0f}cm")

    if cancelled():
        return finish(False)
    _set_led(bolt, *done_color)
    floor_num = "1" if state_key == "floor1" else "2"
    threading.Thread(target=_scroll_connected, args=(bolt, floor_num, *done_color), daemon=True).start()
    finish(True)
    log_event(f"✅ {label} 경로 완료")
    return True

def _run_path(bolt, path, state_key, label, move_color, done_color, gen=None):
    """위치 센서가 있으면 위치 기반, 없으면 시간 기반"""
    if _nav_available(state_key, bolt):
        return _run_path_nav(bolt, path, state_key, label, move_color, done_color, gen)
    log_event(f"  ⏱ {label} 위치 센서 없음 — 시간 기반 주행")
    return _run_path_time(bolt, path, state_key, label, move_color, done_color, gen)

def _return_home(key, gen):
    """위치를 알면 지나온 지점을 거꾸로 거쳐 출발 좌표로, 모르면 시간 되짚기"""
    try:
        _return_home_inner(key, gen)
    finally:
        with _state_lock:
            state[key]["returning"] = False

def _return_home_inner(key, gen):
    bolt = _bolt_of(key)
    for _ in range(40):
        if not state[key]["moving"]:
            break
        time.sleep(0.05)
    with _trail_lock:
        wpts = list(_wpts[key])
    if len(wpts) >= 1 and _pos(key, bolt) is not None:
        return _return_home_nav(key, gen)
    return _return_home_time(key, gen)

R2_ALIGN_WAIT_SEC = 10.0   # R2가 U턴 전에 R1 복귀가 끝나기를 기다리는 최대 시간
R2_ALIGN_MAX_CM   = 30.0   # R1에 맞춰 좌우로 옮기는 최대 거리


def _align_r2_to_r1(label, path_pts, uturn, r1_home, cancelled, bolt, cur_h):
    """U턴 직전: R1 복귀가 끝나기를 기다렸다가, R1이 출발점에서 좌우로 벗어난 만큼 U턴 지점을 옮김
    → R2가 R1 바로 뒤(같은 줄)에 서게. 반환: 옮긴 다음 목표(U턴 30cm 앞 지점)"""
    if cur_h is not None:
        _halt("rear", bolt, cur_h)
    with _state_lock:
        state["rear"]["align_wait"] = True
    try:
        t_end = time.time() + R2_ALIGN_WAIT_SEC
        while state["floor2"].get("returning") and time.time() < t_end and not cancelled():
            time.sleep(0.1)
    finally:
        with _state_lock:
            state["rear"]["align_wait"] = False
    r1 = _common("floor2", _pos("floor2", rt.bolt_2f))
    if r1 is None or state["floor2"].get("returning"):
        return path_pts[-2]
    lx, ly = _vec(uturn[3] + 90)                       # 다가오는 방향의 오른쪽
    off = (r1[0] - r1_home[0]) * lx + (r1[1] - r1_home[1]) * ly
    off = max(-R2_ALIGN_MAX_CM, min(R2_ALIGN_MAX_CM, off))
    if abs(off) >= 3:
        for j in (-2, -1):
            x, y = path_pts[j]
            path_pts[j] = (x + lx * off, y + ly * off)
        # 다가오는 방향(back)의 오른쪽 = R2가 U턴 뒤 바라보는 방향의 왼쪽 → 사람이 보기엔 R1 쪽으로 맞춤
        log_event(f"  ↔ {label} R1이 선 자리에 맞춰 U턴 지점을 {abs(off):.0f}cm 옮김")
    return path_pts[-2]


def _return_home_nav(key, gen):
    label = _AIM_LABEL[key]
    bolt = _bolt_of(key)
    cancelled = lambda: gen != rt.run_gen
    with _trail_lock:
        wpts = list(_wpts[key])
    home = wpts[0]
    p = _pos(key, bolt)
    if p is None or math.hypot(home[0] - p[0], home[1] - p[1]) < NAV_HOME_TOL_CM:
        with _trail_lock:
            _wpts[key] = [home] if p is None else []
            _trail[key] = []
        return
    speed = min(255, state["speeds"][key])
    path_pts = [w for w in reversed(wpts[1:])] + [home]
    hold = None
    if key == "floor2" and _active("rear"):
        keep = min(FOLLOW_END_GAP_M, state["follow_gap"]) * 100 + 10
        def hold():
            if not state["rear"].get("returning") or state["rear"].get("align_wait"):
                return False                     # R2가 R1 복귀 끝나기를 기다리는 중이면 R1은 계속 감
            a = _common("floor2", _pos("floor2", rt.bolt_2f))
            b2 = _pos("rear", rt.bolt_r2)
            return a is not None and b2 is not None and math.hypot(a[0] - b2[0], a[1] - b2[1]) < keep
    # R2: 마지막에 반원 U턴 → U턴이 옆으로 옮길 만큼 미리 비켜선 지점까지만 간다
    uturn = None
    if key == "rear":
        last_from = path_pts[-2] if len(path_pts) >= 2 else p
        back = _angle_of(home[0] - last_from[0], home[1] - last_from[1])
        shift, along = _uturn_learned["side"], _uturn_learned["along"]
        bx, by = _vec(back)
        best = None
        for sign in (-1, +1):
            dvx, dvy = _vec(back + 90 * sign)
            pre = (home[0] - dvx * shift - bx * along, home[1] - dvy * shift - by * along)
            score = math.hypot(pre[0] - last_from[0], pre[1] - last_from[1])
            if best is None or score < best[0]:
                best = (score, sign, pre)
        uturn = (best[1], best[2], shift, back)
        lead_in = (best[2][0] - bx * 30, best[2][1] - by * 30)     # U턴 지점 30cm 앞
        path_pts[-1:] = [lead_in, best[2]]
        # R1 출발점(공통 좌표) — 복귀 끝에 R1이 실제로 선 자리와 비교해 R2를 R1 바로 뒤로 맞춤
        with _trail_lock:
            r1_home = _common("floor2", _wpts["floor2"][0]) if _active("floor2") and _wpts["floor2"] else None
    with _state_lock:
        state[key]["returning"] = True
        state[key]["return_secs"] = round(sum(
            math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip([p] + path_pts[:-1], path_pts)) / 100
            / max(_mps(key, speed), 0.05) + 0.5 * len(path_pts) + (R2_UTURN_SEC if uturn else 1), 1)
    _set_led(bolt, 255, 255, 255)
    log_event(f"↩ {label} 출발 좌표로 복귀 (위치 기반, 지점 {len(path_pts)}개"
              + (f", U턴 {'오른쪽' if uturn[0] > 0 else '왼쪽'} 옆 {uturn[2]:.0f}cm·앞뒤 {-_uturn_learned['along']:+.0f}cm 보정" if uturn else "") + ")")
    try:
        cur_h = None
        for i_t, tgt in enumerate(path_pts):
            if uturn and r1_home is not None and i_t == len(path_pts) - 2:
                tgt = _align_r2_to_r1(label, path_pts, uturn, r1_home, cancelled, bolt, cur_h)
                uturn = (uturn[0], path_pts[-1], uturn[2], uturn[3])
            p = _pos(key, bolt)
            if p is None:
                log_event(f"⚠ {label} 위치 신호 끊김 — 복귀 중단")
                return
            if math.hypot(tgt[0] - p[0], tgt[1] - p[1]) < NAV_HOME_TOL_CM:
                continue
            h = _angle_of(tgt[0] - p[0], tgt[1] - p[1])
            if cur_h is not None:
                _halt(key, bolt, cur_h)
                time.sleep(0.5)
            _set_matrix(bolt, 255, 255, 255, "^")
            res = _nav_to(key, bolt, {"start": (0.0, 0.0)}, lambda st, t=tgt: t, h, speed, cancelled,
                          calib=False, hold=hold)
            cur_h = h
            if res == "cancel":
                _halt(key, bolt, h)
                log_event(f"↩ {label} 복귀 중단 (새 신호)")
                return
            if res != "ok":
                log_event(f"⚠ {label} 복귀 지점 도착 실패 ({res})")
        _halt(key, bolt, cur_h or 0)
        if uturn:
            before = _pos(key, bolt)
            sign, start_h = uturn[0], uturn[3]
            _u_turn(bolt, start_h, sign, gen)
            if not _is_real(bolt):     # SIM: U턴을 반원 근사로 반영
                dvx, dvy = _vec(start_h + 90 * sign)
                _sim_pose[key][0] += dvx * uturn[2]
                _sim_pose[key][1] += dvy * uturn[2]
            time.sleep(0.3)
            after = _pos(key, bolt)
            if before is not None and after is not None:
                # 계획한 U턴 지점 → U턴 뒤 위치: 멈춘 뒤 미끄러짐까지 포함한 실제 옮김
                pre = uturn[1]
                dx, dy = after[0] - pre[0], after[1] - pre[1]
                dvx, dvy = _vec(start_h + 90 * sign)
                bx, by = _vec(start_h)
                side_m, along_m = dx * dvx + dy * dvy, dx * bx + dy * by
                if 0 < side_m < 200 and abs(along_m) < 150:
                    _uturn_learned["side"] = round((_uturn_learned["side"] + side_m) / 2, 1)
                    _uturn_learned["along"] = round((_uturn_learned["along"] + along_m) / 2, 1)
                    _save_settings()
                    log_event(f"  📏 R2 U턴 실측 옆 {side_m:.0f}cm · 앞뒤 {-along_m:+.0f}cm"
                              f" → 다음 보정 옆 {_uturn_learned['side']:.0f}cm · 앞뒤 {-_uturn_learned['along']:+.0f}cm")
        elif bolt is not None:
            bolt.drive_control.roll_start(0, 0)
            time.sleep(1.0)
            bolt.drive_control.roll_stop(0)
        p = _pos(key, bolt)
        err = math.hypot(home[0] - p[0], home[1] - p[1]) if p else None
        with _trail_lock:
            _wpts[key] = []
            _trail[key] = []
        _set_led(bolt, 0, 255, 0)
        log_event(f"✅ {label} 출발 좌표 복귀 완료" + (f" (오차 {err:.0f}cm)" if err is not None else ""))
    except Exception as e:
        log_event(f"↩ {label} 복귀 실패: {e}")
    finally:
        with _state_lock:
            state[key]["returning"] = False
        _mark_motion_end()

def _common(key, p):
    """각 로봇 좌표 → R2 출발점 기준 공통 좌표"""
    if p is None:
        return None
    if key == "floor2":
        return (p[0], p[1] + state["follow_gap"] * 100)
    return p

def _calibrate_axes(key, bolt, heading, speed, cancelled):
    """위치 좌표축 회전을 모를 때 heading 방향으로 15cm 곧게 가며 추정. 반환: 출발 위치(헤딩 좌표) 또는 None"""
    raw0 = _raw(key)
    if raw0 is None:
        return None
    t0 = time.time()
    deadline = t0 + max(4.0, 0.3 / max(_mps(key, speed), 0.05) * 3)
    while not cancelled() and time.time() < deadline:
        raw = _raw(key)
        if raw is None:
            break
        dx, dy = raw[0] - raw0[0], raw[1] - raw0[1]
        if math.hypot(dx, dy) >= NAV_CALIB_CM:
            _loc_rot[key] = _snap_rot(_norm180(_angle_of(dx, dy) - heading))
            log_event(f"  📐 {_AIM_LABEL[key]} 위치 좌표축 보정 {_loc_rot[key]:+.0f}°")
            return _raw_to_pos(key, raw0, _loc_rot[key])
        _drive(key, bolt, heading, speed)
        time.sleep(0.05)
    _halt(key, bolt, heading)
    if not cancelled():
        raw = _raw(key)
        moved = math.hypot(raw[0] - raw0[0], raw[1] - raw0[1]) if raw else None
        log_event(f"  ⚠ {_AIM_LABEL[key]} 좌표축 보정 실패 — "
                  + ("위치 신호 끊김" if raw is None
                     else f"{time.time() - t0:.0f}초 동안 위치 변화 {moved:.0f}cm (바퀴가 헛돌거나 위치 값이 안 바뀜)"))
    return None

def _run_path_time(bolt, path, state_key, label, move_color, done_color, gen=None):
    """범용 경로 실행 — 2F/1F 공통. 상황 해제(세대 변경) 시 멈추고 False 반환"""
    def cancelled():
        return gen is not None and gen != rt.run_gen

    def abort():
        if bolt is not None:
            try:
                bolt.drive_control.roll_stop(0)
            except Exception:
                pass
        with _state_lock:
            state[state_key]["moving"] = False
            state[state_key]["step"]   = 0
        _mark_motion_end()
        log_event(f"⏹ {label} 주행 중단 (상황 해제)")
        return False

    with _state_lock:
        state[state_key]["moving"] = True
        state[state_key]["done"]   = False
        state[state_key]["step"]   = 0

    _set_led(bolt, *move_color)
    factor = TARGET_MODES[state["target"]]["mps"]
    with _state_lock:
        my_speed = state["speeds"][state_key]
    time_scale = SPEED_REF / my_speed        # 각자 자기 속도로 환산 → 속도를 바꿔도 정해진 거리만 간다
    if state_key == "rear":
        time_scale *= state["r2_dist_pct"] / 100
    turn_pause = state["turn_pause"]
    prev_heading = None

    for i, (speed, heading, duration, char) in enumerate(path):
        if cancelled():
            return abort()
        if duration > 0 and prev_heading is not None and heading != prev_heading and turn_pause > 0:
            turn = (heading - prev_heading) % 360
            if bolt is not None:
                try:
                    bolt.drive_control.roll_stop(prev_heading)
                except Exception:
                    pass
            _set_matrix(bolt, *move_color, _ARROW.get(turn, "^"))   # 몸체 기준: 왼쪽 < / 오른쪽 >
            log_event(f"  ↪ {label} {'좌' if turn == 270 else '우' if turn == 90 else ''}회전 전 {turn_pause:g}초 정지")
            end = time.time() + turn_pause
            while time.time() < end and not cancelled():
                time.sleep(0.05)
            if cancelled():
                return abort()
        with _state_lock:
            state[state_key]["step"] = i + 1

        # 매트릭스는 몸체와 함께 돌아가므로 주행 중엔 항상 전방 화살표, 정지 시 지정 문자
        _set_matrix(bolt, *move_color, "^" if duration > 0 else char)

        if duration > 0:
            prev_heading = heading
            speed    = min(255, int(my_speed * factor))
            duration = round(duration * time_scale / factor, 2)
            log_event(f"  → {label} step {i+1}/{len(path)}: "
                      f"속도={speed}  방향={heading}°  {duration}s  [{char}]")
            if bolt is not None:
                try:
                    bolt.drive_control.roll_start(heading, speed)
                except Exception as e:
                    log_event(f"  [이동 오류] {e}")
                    raise_exception("E4", f"{label} 명령 실패 — 남은 경로 계속 진행")
            if i == 0 and state_key in ("floor2", "rear") and state["latency"] is None and not _test_running["on"]:
                lat = round(time.time() - rt.trigger_ts, 2)
                with _state_lock:
                    state["latency"] = lat
                log_event(f"⏱ 트리거→선두 출발 {lat:.1f}s (목표 ≤ 2.0s)")
            t0 = time.time()
            end = t0 + duration
            while time.time() < end and not cancelled():
                time.sleep(0.05)
            with _trail_lock:
                _trail[state_key].append((heading, speed, round(min(time.time() - t0, duration), 2)))
        else:
            log_event(f"  → {label} 정지 (핸드오프 포인트)  [{char}]")
            if bolt is not None:
                try:
                    bolt.drive_control.roll_stop(heading)
                except Exception:
                    pass

    if cancelled():
        return abort()
    _set_led(bolt, *done_color)
    floor_num = "1" if state_key == "floor1" else "2"
    threading.Thread(target=_scroll_connected, args=(bolt, floor_num, *done_color), daemon=True).start()

    with _state_lock:
        state[state_key]["moving"] = False
        state[state_key]["done"]   = True
        state[state_key]["step"]   = 0
    _mark_motion_end()

    log_event(f"✅ {label} 경로 완료")
    return True
