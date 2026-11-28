"""수납함 출동 — 수납함 칸에서 ㄱ자로 나와 R1 앞·R2 뒤로 한 줄 배치 → 대피자가 R1·R2 사이에 3초 머물면 경로 안내 시작

수납함(정면에서 보면 왼쪽부터 R4 · R2 · R1)에서 로봇은 바깥을 보고 서 있다.
  R1: 0.8m 직진 → (로봇 기준) 왼쪽으로 꺾어 1.4m → 정지
  R2: 0.8m 직진 → 왼쪽으로 꺾어 R1 뒤 (간격 + 0.3m)가 될 때까지 → 정지 (R1 위치를 보며 멈춤)
배치가 끝나면 두 로봇이 보는 방향을 새 0°로 잡고 위치 원점도 다시 잡는다 → 기존 경로·R2 따라가기를 그대로 쓴다.
따라가기는 'R1이 R2보다 간격만큼 앞'을 기준으로 하므로, 더 벌어진 만큼 R2 출발 좌표를 그만큼 뒤로 둔다.
"""

import math
import threading
import time

from .camera import _follow, _follow_lock, person_still_secs
from .config import (
    DEFAULT_SPEEDS,
    DEPLOY_EXTRA_GAP_M,
    DEPLOY_OUT_M,
    DEPLOY_R1_SIDE_M,
    DEPLOY_R2_SIDE_M,
    DEPLOY_START_TTS,
    DEPLOY_STILL_SEC,
    DEPLOY_THEN_TEST,
    DEPLOY_TURN_WAIT_SEC,
    TARGET_MODES,
    _ARROW,
    _route_to_path,
)
from .hw import _halt, _is_real, _org_off, _pos, _sim_pose, _raw, _set_led, _set_matrix
from .manual import _queue_tts
from .nav import REST_SETTLE_SEC, _nav_to, _reset_origin, _run_path, _run_path_time, _trail, _trail_lock, _wpts
from .state import _active, _mark_motion_end, _state_lock, _test_running, log_event, raise_exception, rt, state

DEPLOY_COLOR      = (255, 160, 0)   # 출동 배치 중 LED (주황)
WAIT_COLOR        = (255, 255, 0)   # 대피자 기다리는 중 LED (노랑)
DEPLOY_LEFT       = 270             # 로봇 기준 왼쪽
HOLD_MARGIN_CM    = 10.0            # R2는 R1과 (간격 − 이 값)보다 가까워지면 멈춰 기다림
FINAL_FIX_MIN_CM  = 10.0            # R1이 선 뒤 간격 차이가 이보다 작으면 R2를 더 움직이지 않음
R2_EARLY_CM       = 10.0            # R2는 간격보다 이만큼 일찍 멈춤 — 멈춘 뒤 트롤리에 밀려 더 감 (실측: 덜 가야 함)
R1_STOP_WAIT_SEC  = 15.0            # R2가 R1이 멈추기를 기다리는 최대 시간
CAMERA_WARN_SEC   = 5.0             # 이만큼 카메라 감지 값이 없으면 안내 로그


def _set_stage(stage, still=0.0):
    with _state_lock:
        state["deploy"] = {"stage": stage, "still": round(still, 1)}


def _r1_side_m(slot_cm, gap_cm):
    """R1이 꺾은 뒤 갈 거리 — R2가 (칸 간격만큼 옆에서) 꺾어 R1 뒤 간격에 서려면 R1이 그만큼 멀리 있어야 함.
    예) 간격 4m+0.3m, 칸 35cm → R1 4.4m (R2는 0.45m 가서 섬). 간격이 작으면 기본 1.4m"""
    return round(max(DEPLOY_R1_SIDE_M, (gap_cm - slot_cm) / 100 + DEPLOY_R2_SIDE_M), 2)


def _r1_path(side_m):
    return _route_to_path([(0, DEPLOY_OUT_M), (DEPLOY_LEFT, side_m)], "v", DEFAULT_SPEEDS["floor2"])


def _deploy_r2(gen, slot_cm, gap_cm, r1_on, out, side_m):
    """R2: 0.8m 직진 → 왼쪽으로 꺾어 R1 뒤 간격까지. 반환 True = 배치 완료"""
    key, bolt, label = "rear", rt.bolt_r2, "R2(출동)"
    out_cm, side_cm = DEPLOY_OUT_M * 100, side_m * 100

    def cancelled():
        return gen != rt.run_gen

    def finish(ok, heading):
        _halt(key, bolt, heading)
        if ok and _is_real(bolt):
            time.sleep(REST_SETTLE_SEC)
        p = _pos(key, bolt)
        with _trail_lock:
            if p is not None:
                _wpts[key].append(p)
        with _state_lock:
            state[key]["moving"] = False
            state[key]["step"] = 0
        _mark_motion_end()
        if not ok:
            log_event(f"⏹ {label} 주행 중단 (상황 해제)")
        return ok

    with _state_lock:
        state[key]["moving"] = True
        state[key]["done"] = False
        state[key]["step"] = 1
    _set_led(bolt, *DEPLOY_COLOR)
    speed = min(255, int(state["speeds"][key] * TARGET_MODES[state["target"]]["mps"]))
    ctx = {"start": _pos(key, bolt), "raw0": _raw(key) if _is_real(bolt) else None}
    with _trail_lock:
        if not _wpts[key] and ctx["start"] is not None:
            _wpts[key].append(ctx["start"])
    log_event(f"  → {label} step 1/2: 직진 {DEPLOY_OUT_M:g}m (수납함에서 나옴)")
    _set_matrix(bolt, *DEPLOY_COLOR, "^")
    t0 = time.time()
    res = _nav_to(key, bolt, ctx, lambda st: (st[0], st[1] + out_cm), 0, speed, cancelled)
    with _trail_lock:
        _trail[key].append((0, speed, round(time.time() - t0, 2)))
        if not _wpts[key] and ctx.get("start") is not None:
            _wpts[key].append(ctx["start"])          # 축 보정 뒤에야 출발점이 정해진 경우
    if res == "cancel":
        return finish(False, 0)
    if res == "noloc" or ctx.get("start") is None:
        log_event(f"⚠ {label} 위치 신호 끊김 — 시간 기반으로 배치 (간격은 계산값)")
        raise_exception("E4", f"{label} 위치 센서 끊김")
        with _state_lock:
            state[key]["moving"] = False
        rest = max(0.1, (slot_cm + side_cm - gap_cm) / 100)
        return _run_path_time(bolt, _route_to_path([(DEPLOY_LEFT, rest)], "v", DEFAULT_SPEEDS[key]),
                              key, label, DEPLOY_COLOR, DEPLOY_COLOR, gen)
    if res == "timeout":
        log_event(f"⚠ {label} {DEPLOY_OUT_M:g}m 구간 시간 초과 — 다음 구간으로")

    # 꺾기 전 정지 — 멈춰 미끄러짐이 끝난 뒤 DEPLOY_TURN_WAIT_SEC 더 기다렸다 꺾음
    _halt(key, bolt, 0)
    pause = DEPLOY_TURN_WAIT_SEC + (REST_SETTLE_SEC if _is_real(bolt) else 0)
    _set_matrix(bolt, *DEPLOY_COLOR, _ARROW[DEPLOY_LEFT])
    log_event(f"  ↪ {label} 좌회전 전 {pause:g}초 정지 (멈춘 뒤 {DEPLOY_TURN_WAIT_SEC:g}초)")
    end = time.time() + pause
    while time.time() < end and not cancelled():
        time.sleep(0.05)
    if cancelled():
        return finish(False, 0)
    corner = _pos(key, bolt)
    with _trail_lock:
        if corner is not None:
            _wpts[key].append(corner)

    start = ctx["start"]
    stop_gap = gap_cm + R2_EARLY_CM                                       # 이 간격에서 멈추면 밀려서 간격쯤에 섬
    final = (start[0] - slot_cm - side_cm + stop_gap, start[1] + out_cm)   # R1이 계획대로 섰을 때 R2 멈출 자리

    def r1_here():
        """R1 위치를 R2 좌표로 — R1 출발점 = R2 출발점에서 왼쪽으로 칸 간격"""
        if not r1_on:
            return None
        with _trail_lock:
            w = _wpts["floor2"][0] if _wpts["floor2"] else None
        p = _pos("floor2", rt.bolt_2f)
        if w is None or p is None:
            return None
        return (start[0] - slot_cm + p[0] - w[0], start[1] + p[1] - w[1])

    def target(st):
        r = r1_here()
        if r is not None and not state["floor2"]["moving"]:
            return (r[0] + stop_gap, r[1])         # R1이 선 자리 기준 — 같은 줄, 간격만큼 뒤
        return final

    def hold():
        r, me = r1_here(), _pos(key, bolt)
        if r is None:
            return r1_on and state["floor2"]["moving"]     # R1 위치를 모르는데 R1이 움직이는 중 → 부딪히지 않게 기다림
        return me is not None and math.hypot(r[0] - me[0], r[1] - me[1]) < gap_cm - HOLD_MARGIN_CM

    with _state_lock:
        state[key]["step"] = 2
    _set_matrix(bolt, *DEPLOY_COLOR, "^")
    log_event(f"  → {label} step 2/2: 좌회전 → R1 뒤 {gap_cm:g}cm까지 (예상 {(slot_cm + side_cm - stop_gap) / 100:.2f}m, "
              f"밀림 고려 {R2_EARLY_CM:g}cm 일찍 멈춤)")
    t0 = time.time()
    res = _nav_to(key, bolt, {"start": start}, target, DEPLOY_LEFT, speed, cancelled, calib=False, hold=hold)
    if res == "ok" and r1_on and state["floor2"]["moving"]:
        # R1보다 먼저 계획 자리에 옴 → R1이 서면 실제 자리에 맞춰 조금 더
        _halt(key, bolt, DEPLOY_LEFT)
        t_end = time.time() + R1_STOP_WAIT_SEC
        while state["floor2"]["moving"] and time.time() < t_end and not cancelled():
            time.sleep(0.1)
        r, me = r1_here(), _pos(key, bolt)
        if r is not None and me is not None and not cancelled():
            gap_err = math.hypot(r[0] - me[0], r[1] - me[1]) - stop_gap
            if gap_err > FINAL_FIX_MIN_CM:
                log_event(f"  ↔ {label} R1이 선 자리에 맞춰 {gap_err:.0f}cm 더 감")
                res = _nav_to(key, bolt, {"start": start}, target, DEPLOY_LEFT, speed, cancelled,
                              calib=False, hold=hold)
    with _trail_lock:
        _trail[key].append((DEPLOY_LEFT, speed, round(time.time() - t0, 2)))
    if res == "cancel":
        return finish(False, DEPLOY_LEFT)
    if res in ("timeout", "noloc"):
        log_event(f"⚠ {label} 배치 구간 {'시간 초과' if res == 'timeout' else '위치 신호 끊김'}")
    finish(True, DEPLOY_LEFT)
    r, me = r1_here(), _pos(key, bolt)
    if r is not None and me is not None:
        d = math.hypot(r[0] - me[0], r[1] - me[1])
        out["gap_cm"] = d
        log_event(f"✅ {label} 배치 완료 — R1과 간격 {d:.0f}cm (목표 {gap_cm:g}cm, 옆 어긋남 {me[1] - r[1]:+.0f}cm)")
    else:
        log_event(f"✅ {label} 배치 완료")
    return True


def _deploy(gen):
    """1단계 — 출동 배치. 반환 True = 두 로봇(연결된 것) 배치 완료"""
    r1_on, r2_on = _active("floor2"), _active("rear")
    slot, follow_gap = state["deploy_slot"], state["follow_gap"] * 100
    gap = follow_gap + DEPLOY_EXTRA_GAP_M * 100          # 대기 때 R1–R2 거리 (대피자가 설 자리)
    _set_stage("deploying")
    side_m = _r1_side_m(slot, gap)                      # 간격이 크면 R1이 더 멀리 나가야 R2가 뒤에 설 수 있음
    log_event(f"📦 수납함 출동 — R1 직진 {DEPLOY_OUT_M:g}m → 좌회전 {side_m:g}m · "
              f"R2 직진 {DEPLOY_OUT_M:g}m → 좌회전 R1 뒤 {gap:g}cm (간격 {follow_gap:g} + {DEPLOY_EXTRA_GAP_M * 100:g}cm, "
              f"칸 간격 {slot:g}cm)")
    result = {}

    def run_r1():
        # 나와서 멈추고 미끄러짐이 끝난 뒤 1초 더 기다렸다 꺾음 (실제 로봇만 미끄러짐 대기)
        wait = DEPLOY_TURN_WAIT_SEC + (REST_SETTLE_SEC if _is_real(rt.bolt_2f) else 0)
        result["floor2"] = _run_path(rt.bolt_2f, _r1_path(side_m), "floor2", "R1(출동)", turn_pause=wait,
                                     move_color=DEPLOY_COLOR, done_color=DEPLOY_COLOR, gen=gen)

    def run_r2():
        try:
            result["rear"] = _deploy_r2(gen, slot, gap, r1_on, result, side_m)
        except Exception as e:
            log_event(f"⚠ R2(출동) 오류: {e}")
            result["rear"] = False

    threads = []
    if r1_on:
        threads.append(threading.Thread(target=run_r1, daemon=True))
    if r2_on:
        threads.append(threading.Thread(target=run_r2, daemon=True))
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    with _state_lock:
        for k in ("floor2", "rear"):
            state[k]["done"] = False
    if gen != rt.run_gen or not all(v for k, v in result.items() if k != "gap_cm"):
        return False
    # 실제 R1–R2 거리 (R2 좌표 기준) — 새 원점에서 R2를 '간격보다 더 뒤'에 둘 만큼
    measured = result.get("gap_cm") if r1_on and r2_on else None
    # 지금 보는 방향 = 새 0°, 지금 자리 = 새 출발점 → 기존 경로·R2 따라가기·평시 복귀(배치 자리로)를 그대로 쓴다
    for k, bolt in (("floor2", rt.bolt_2f), ("rear", rt.bolt_r2)):
        if not _active(k):
            continue
        if bolt is not None and hasattr(bolt.drive_control, "reset_heading"):
            try:
                bolt.drive_control.reset_heading()
            except Exception as e:
                log_event(f"  [방향 0° 리셋 오류] {e}")
        _reset_origin(k, bolt)
    back = (measured if measured is not None else gap) - follow_gap
    if r1_on and r2_on and abs(back) >= 1:
        _org_off["rear"] = (0.0, -back)
        if not _is_real(rt.bolt_r2):
            _sim_pose["rear"][1] = -back
    log_event("📦 배치 완료 — 지금 보는 방향을 0°, 지금 자리를 출발점으로 다시 잡음"
              + (f" (R2는 따라가기 기준보다 {back:.0f}cm 뒤에서 출발)" if r1_on and r2_on and abs(back) >= 1 else ""))
    return True


def _wait_person(gen):
    """2단계 — R2 카메라에 대피자가 다가옴·정지·멀어짐 상태로 DEPLOY_STILL_SEC 동안 머물면 True"""
    with _follow_lock:
        _follow.update(active=True, person=False, seen=False, lost_since=None, lost_alert_ts=None,
                       still_since=None, still_bad_since=None)
    _set_led(rt.bolt_2f, *WAIT_COLOR)
    _set_led(rt.bolt_r2, *WAIT_COLOR)
    _set_stage("waiting")
    log_event(f"👀 대피자 기다림 — R1·R2 사이(카메라 다가옴·정지·멀어짐)에 {DEPLOY_STILL_SEC:g}초 머물면 경로 안내 시작 (R2 카메라 감지 ON)")
    t0, warned, seen = time.time(), False, False
    while gen == rt.run_gen:
        now = time.time()
        still = person_still_secs(now)
        _set_stage("waiting", still)
        with _follow_lock:
            last_ts, person = _follow["last_ts"], _follow["person"]
        if person and not seen:
            seen = True
            log_event("🙋 R2 카메라: 대피자 보임 — R1·R2 사이에 머물기를 기다림")
        if not warned and now - max(last_ts, t0) > CAMERA_WARN_SEC:
            warned = True
            log_event("⚠ R2 카메라 감지 값이 안 옴 — 폰 앱의 감지를 켜 두세요")
        if still >= DEPLOY_STILL_SEC:
            log_event(f"🙋 대피자가 R1·R2 사이에 {still:.1f}초 머묾 → 경로 안내 시작")
            return True
        time.sleep(0.1)
    return False


def run_deploy_test(gen):
    """수납함 출동 테스트 — 배치 → 대피자 확인(3초 머묾) → ㄱ자 테스트 경로 (R3는 빠짐)"""
    from .scenario import _run_test_path, _test_paths      # scenario ↔ deploy 순환 import 방지
    try:
        if not _deploy(gen):
            if gen == rt.run_gen:
                log_event("⚠ 수납함 출동 배치 실패 — '상황 해제'로 수납함 앞 출발점 복귀")
            return
        if not DEPLOY_THEN_TEST:                            # 출동만 — 다음은 재난 트리거로
            log_event("📦 출동 완료 — 이 자리에서 재난 트리거(화재·지진·정전)를 누르면 고른 2층 경로로 출발")
            return
        if not _wait_person(gen):
            return
        _queue_tts(DEPLOY_START_TTS)
        _set_stage("guiding")
        paths = _test_paths(state["follow_gap"])
        keys = [k for k in ("floor2", "rear") if _active(k)]
        with _state_lock:
            _test_running["on"] = True
        log_event(f"🧪 경로 안내 시작 — ㄱ자 테스트 경로 ({', '.join('R1' if k == 'floor2' else 'R2' for k in keys)})")
        threads = [threading.Thread(target=_run_test_path, args=(k, gen, paths[k]), daemon=True) for k in keys]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
    except Exception as e:
        log_event(f"⚠ 수납함 출동 오류: {e}")
    finally:
        _set_stage("")
        with _state_lock:
            if not any(state[k]["moving"] for k in ("floor2", "rear", "floor1")):
                _test_running["on"] = False
