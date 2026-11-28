"""공유 상태 — 대시보드 state, 런타임 값(rt), 이벤트 로그·예외 표시, 설정 저장"""

from datetime import datetime
import json
import logging
import socket
import threading
import time

from .config import (
    DEFAULT_R2_DELAY_SEC,
    DEFAULT_R2_DIST_PCT,
    DEFAULT_TURN_PAUSE_SEC,
    EVAC_PATH_1F,
    EVAC_PATH_2F,
    EVAC_PATH_R2,
    EXCEPTIONS,
    EXCEPTION_SHOW_SEC,
    FOLLOW_GAP_M,
    GAP_MAX_M,
    GAP_MIN_M,
    SPEED_FILE,
    SPHERO_1F_NAME,
    SPHERO_2F_NAME,
    SPHERO_R2_NAME,
    _load_aim_trim,
    _load_setting,
    _load_speeds,
)


class _Runtime:
    """여러 모듈이 다시 대입하는 실행 중 값 — 모듈마다 복사본이 생기지 않게 한 객체에 모음"""
    bolt_2f = None          # 연결된 BOLT (없으면 None)
    bolt_1f = None
    bolt_r2 = None
    triggered = False       # 재난 신호 처리 중
    trigger_ts = 0.0
    run_gen = 0             # 트리거·리셋마다 증가 — 이전 세대 주행 스레드는 스스로 멈춘다
    last_shake_ts = 0.0
    last_motion_end = 0.0   # 로봇이 마지막으로 멈춘 시각 — 직후엔 흔들기를 지진으로 보지 않음
    latest_frame = None     # R2 카메라 최신 JPEG (bytes)
    frame_ts = 0.0


rt = _Runtime()

state = {
    "floor2": {"connected": False, "signal": False, "battery": 0,
               "moving": False, "step": 0, "total_steps": len(EVAC_PATH_2F),
               "accel": {"x": 0.0, "y": 0.0, "z": 0.0}},
    "floor1": {"connected": False, "moving": False, "step": 0,
               "total_steps": len(EVAC_PATH_1F), "battery": 0},
    "rear":   {"connected": False, "moving": False, "step": 0,
               "total_steps": len(EVAC_PATH_R2), "battery": 0},
    "trigger_count": 0,
    "phase": "idle",            # idle → armed → guiding → handoff → complete
    "disaster": None,           # fire / quake / black
    "target": "normal",
    "exceptions": {},           # "E4": "triggered" | "resolved"
    "estop": False,             # ⑬ 비상 정지 — 모든 로봇 그 자리 정지, 매트릭스 빨강 전체
    "matrix": {},               # 로봇별 지금 매트릭스 화면 이름 (bandi/matrix.py PATTERNS)
    "manual": {"on": False, "reason": "", "heading": 0, "cmd": None, "since": 0.0},   # R2 수동 조작(원격)
    "ir": "off",                # R2가 R1 IR 보는지: off | wait | ok | lost
    "latency": None,            # 트리거 → 선두 출발 (초)
    "log": [],
    "sim_mode": False,
    "names": {"floor2": SPHERO_2F_NAME, "floor1": SPHERO_1F_NAME, "rear": SPHERO_R2_NAME},
    "connecting": False,
    "speeds": _load_speeds(),
    "r2_delay": _load_setting("r2_delay", DEFAULT_R2_DELAY_SEC, 0, 30),
    "turn_pause": _load_setting("turn_pause", DEFAULT_TURN_PAUSE_SEC, 0, 10),
    "aim_trim": _load_aim_trim(),
    "r2_dist_pct": _load_setting("r2_dist_pct", DEFAULT_R2_DIST_PCT, 50, 200),
    "follow_gap": _load_setting("follow_gap", FOLLOW_GAP_M, GAP_MIN_M, GAP_MAX_M),
    # 앱이 보낸 셀피 카메라 정면 방위 (자북 기준 시계 방향 도). quality: 1 = 폰 세움, 0 = 눕힘
    "phone": {"azimuth": None, "quality": 0.0, "accuracy": None, "ts": 0.0},
    # 나침반 자동 에임 진행 상태 (로봇별)
    "aim": {"floor2": {"status": "", "msg": ""}, "rear": {"status": "", "msg": ""},
            "floor1": {"status": "", "msg": ""}},
}

_state_lock = threading.Lock()

def log_event(msg: str):
    ts = datetime.now().strftime("%H:%M:%S")
    entry = {"ts": ts, "msg": msg}
    with _state_lock:
        state["log"].insert(0, entry)
        state["log"] = state["log"][:100]
    logging.info(msg)

_trigger_lock    = threading.Lock()

_broker_stop_evt = threading.Event()

def _robots_in_motion() -> bool:
    return any(state[k]["moving"] or state[k].get("returning") for k in ("floor2", "rear", "floor1"))

def _mark_motion_end():
    rt.last_motion_end = time.time()

def _lan_ip() -> str:
    """폰 앱이 접속할 이 맥의 사설망 IP (UDP connect는 패킷을 보내지 않음)"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()

def _active(key: str) -> bool:
    """실제 연결됐거나 SIM 모드면 주행 대상"""
    return state[key]["connected"] or state["sim_mode"]

def _set_phase(phase: str):
    with _state_lock:
        state["phase"] = phase
    log_event(f"📍 시스템 상태 → {phase.upper()}")

def raise_exception(eid: str, detail: str = ""):
    """예외 E1–E5 표시 — EXCEPTION_SHOW_SEC 동안 발생 상태 후 처리됨으로 바뀜"""
    with _state_lock:
        state["exceptions"][eid] = "triggered"
    log_event(f"⚠ {eid} {EXCEPTIONS[eid]}" + (f" ({detail})" if detail else ""))

    def _resolve():
        time.sleep(EXCEPTION_SHOW_SEC)
        with _state_lock:
            if state["exceptions"].get(eid) == "triggered":
                state["exceptions"][eid] = "resolved"
    threading.Thread(target=_resolve, daemon=True).start()

# R2 U턴이 실제로 옮겨 놓는 거리 — side: U턴 방향 옆으로, along: 다가온 방향으로 (앞으로 밀리면 음수)
# 복귀할 때마다 실측해 반씩 섞고 설정 파일에 저장 (재시작해도 유지)
_uturn_learned = {"side": _load_setting("uturn_side", 35.0, 0, 200),
                  "along": _load_setting("uturn_along", -25.0, -150, 150)}

_test_running = {"on": False}

def _save_settings():
    with _state_lock:
        data = {**state["speeds"], "r2_delay": state["r2_delay"],
                "turn_pause": state["turn_pause"], "aim_trim": state["aim_trim"],
                "r2_dist_pct": state["r2_dist_pct"], "follow_gap": state["follow_gap"],
                "uturn_side": _uturn_learned["side"], "uturn_along": _uturn_learned["along"]}
    try:
        with open(SPEED_FILE, "w", encoding="utf-8") as f:
            json.dump(data, f)
    except OSError as e:
        log_event(f"⚠ 설정 저장 실패: {e}")
