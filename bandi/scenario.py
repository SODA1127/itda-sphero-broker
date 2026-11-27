"""대피 시나리오 — 재난 신호 처리, 2층→1층 인수인계, 테스트 경로"""

import threading
import time

from .config import (
    DEFAULT_SPEEDS,
    DISASTERS,
    EVAC_PATH_1F,
    QUAKE_SETTLE_SEC,
    SIGNAL_COOLDOWN,
    TARGET_MODES,
    TEST_ROUTE_M,
    _AIM_LABEL,
    _build_2f_paths,
    _build_follow_paths,
    _path_meters,
    _route_to_path,
)
from .follow import _run_r2_delayed, _run_r2_follow
from .hw import _scroll_connected, _set_led
from .nav import _run_path, _trail, _trail_lock
from .state import (
    _active,
    _set_phase,
    _state_lock,
    _test_running,
    _trigger_lock,
    log_event,
    raise_exception,
    rt,
    state,
)


def trigger_signal(source: str = "unknown", disaster: str = None):
    """재난 신호 수신 → 2F(R1·R2) 유도 → 계단 인수인계 → 1F(R3) 유도"""
    with _trigger_lock:
        if rt.triggered:
            log_event(f"⚠ 중복 신호 무시 ({source})")
            return
        rt.triggered = True
        rt.run_gen += 1
        gen = rt.run_gen

    disaster = disaster or "fire"
    rt.trigger_ts = time.time()
    with _state_lock:
        state["floor2"]["signal"] = True
        state["trigger_count"] += 1
        state["disaster"] = disaster
        state["latency"]  = None
        state["exceptions"] = {}
        for k in ("floor2", "floor1", "rear"):
            state[k]["done"] = False
        mode = TARGET_MODES[state["target"]]

    d = DISASTERS[disaster]
    with _trail_lock:
        away = [_AIM_LABEL[k] for k, t in _trail.items() if t]
    if away:
        log_event(f"⚠ {', '.join(away)} 출발점이 아님 — 지금 위치에서 경로 시작 (먼저 '상황 해제'로 복귀 권장)")
    log_event(f"🚨 {d['icon']} {d['label']} 신호 수신 [{source}] — {d['src']}")
    log_event(f"  안내 대상: {mode['icon']} {mode['label']} · {mode['mps']} m/s · "
              f"LED {mode['led']} · 음성 {mode['voice']}")
    _set_phase("armed")

    # 2F LED: 빨강 (위험 알림) → 2F 경로 실행 → 완료 후 1F 자동 출발
    _set_led(rt.bolt_2f, 255, 0, 0)
    _set_led(rt.bolt_r2, 255, 0, 0)
    t = threading.Thread(target=_run_2f_then_1f, args=(gen,), daemon=True)
    t.start()

    # cooldown 후 신호 리셋
    threading.Thread(target=_reset_signal_after, args=(SIGNAL_COOLDOWN, gen), daemon=True).start()

def _reset_signal_after(seconds: float, gen: int):
    time.sleep(seconds)
    if gen != rt.run_gen:
        return
    with _state_lock:
        state["floor2"]["signal"] = False
    with _trigger_lock:
        rt.triggered = False
    _set_led(rt.bolt_2f, 0, 255, 0)
    _set_led(rt.bolt_r2, 0, 255, 0)
    threading.Thread(target=_scroll_connected, args=(rt.bolt_2f, "2", 0, 255, 0), daemon=True).start()
    log_event("✅ 신호 리셋 — 대기 상태 복귀")

def _run_2f_then_1f(gen):
    """2F 경로(R1 선두 + R2 후방 추종) 완료 → 1F 자동 출발 (연결된 쪽만 실행)"""
    def live():
        return gen == rt.run_gen

    if state["disaster"] == "quake":
        log_event("  진동 중 — 음성 안내만, 진동 종료 대기")
        while live() and time.time() - max(rt.last_shake_ts, rt.trigger_ts) < QUAKE_SETTLE_SEC:
            time.sleep(0.1)
        if not live():
            return
        log_event(f"  진동 종료 {QUAKE_SETTLE_SEC:.0f}초 경과 → 유도 시작")

    r1_on = _active("floor2")
    r2_on = _active("rear")
    path_r1, path_r2 = _build_2f_paths(state["follow_gap"])
    with _state_lock:
        state["floor2"]["total_steps"] = len(path_r1)
        state["rear"]["total_steps"]   = len(path_r2)
    log_event(f"  R1·R2 간격 {state['follow_gap']*100:g}cm — R1 {sum(_path_meters(path_r1)):.1f}m · R2 {sum(_path_meters(path_r2)):.1f}m")
    if not r1_on and r2_on:
        raise_exception("E5", "R1 미연결 → R2가 R1 경로를 대신 선도")
    if not _active("floor1"):
        raise_exception("E5", "R3 미연결 → 1층 인수인계 불가")
    _set_phase("guiding")
    if r2_on:
        threading.Thread(target=_run_r2_delayed, args=(gen, state["r2_delay"], path_r2), daemon=True).start()
    if r1_on or r2_on:
        if not r1_on:
            log_event("ℹ️  R1 미연결 — R2만 R1 경로를 따라 주행")
        if not _run_path(rt.bolt_2f, path_r1, "floor2", "2F",
                         move_color=(255, 80, 0), done_color=(255, 0, 0), gen=gen):
            return
        log_event("🏁 2F 계단 앞 도달 → 중계기가 R3에 출발 명령")
    else:
        log_event("ℹ️  2F 미연결 — 1F 직접 출발")
    if not live():
        return
    _set_phase("handoff")
    if _active("floor1"):
        if not _run_path(rt.bolt_1f, EVAC_PATH_1F, "floor1", "1F",
                         move_color=(0, 100, 255), done_color=(0, 255, 0), gen=gen):
            return
        log_event("🚪 R3 비상구 도달")
    _set_phase("complete")

def _test_paths(gap):
    """테스트 경로 — R1·R2는 실제 2층과 같은 따라가기 구조, R3는 혼자 ㄱ자"""
    route = [(TEST_ROUTE_M[0][0], gap + TEST_ROUTE_M[0][1])] + TEST_ROUTE_M[1:]   # R2 출발점 기준
    r1, r2 = _build_follow_paths(route, gap)
    return {"floor2": r1, "rear": r2, "floor1": _route_to_path(TEST_ROUTE_M, "v", DEFAULT_SPEEDS["floor1"])}

def _run_test_path(key, gen, path):
    label = _AIM_LABEL[key]
    bolt = {"floor2": rt.bolt_2f, "rear": rt.bolt_r2, "floor1": rt.bolt_1f}[key]
    try:
        if key == "rear" and _active("floor2"):
            _run_r2_follow(gen)
            return
        _run_path(bolt, path, key, f"{label}(테스트)", move_color=(0, 200, 255), done_color=(0, 255, 0), gen=gen)
    finally:
        with _state_lock:
            state[key]["done"] = False      # 실제 경로 '도착'으로 표시되지 않게
            if not any(state[k]["moving"] for k in ("floor2", "rear", "floor1")):
                _test_running["on"] = False
