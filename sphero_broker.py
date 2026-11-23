"""
ISE103 재난 대피 안내 로봇 — Sphero BOLT PoC 브로커
======================================================
2F Sphero(R1 선두) 가 신호를 보내면 → 브로커가 받아서 → 1F Sphero(R3 대기)를 이동시킨다.

실행:
    pip install spherov2 flask              # 최초 1회
    python3.9 sphero_broker.py             # 실제 BLE 연결  (macOS: python3.9 권장)
    python3.9 sphero_broker.py --sim       # 시뮬레이션(Sphero 없이 대시보드 테스트)

대시보드: http://localhost:5001
트리거:   2F Sphero 세게 흔들기 (가속도 2.5G↑) 또는 대시보드 버튼
"""

import argparse
import os
import tempfile
import threading
import time
import json
import logging
import base64
import queue as _queue
import math
import socket
from datetime import datetime
from flask import Flask, jsonify, render_template_string, request as flask_request
from flask_sock import Sock

# 동네 프로젝트 등에서 사용하는 TypeSafe AI API 키
JEV_KEY       = os.environ.get("JEV_API_KEY", "").strip()
ANTHROPIC_KEY = os.environ.get("ANTHROPIC_API_KEY", "").strip()
_vision_busy  = threading.Lock()   # Vision 호출 중복 방지

logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
try:    # 재시작해도 남는 로그 — 문제 진단용
    _fh = logging.FileHandler(os.path.join(tempfile.gettempdir(), "bandi_broker.log"), encoding="utf-8")
    _fh.setFormatter(logging.Formatter('%(asctime)s %(message)s'))
    _fh.addFilter(lambda r: r.name != "werkzeug")     # HTTP 요청 줄은 빼고
    logging.getLogger().addHandler(_fh)
except OSError:
    pass

# ── 설정 ───────────────────────────────────────────────────────────────────────
SPHERO_2F_NAME  = "SB-25D4"   # 2층 Sphero BLE 이름 (R1 선두)
SPHERO_1F_NAME  = "SB-D721"   # 1층 Sphero BLE 이름 (R3 대기)
SPHERO_R2_NAME  = "SB-3601"   # 2층 후방 Sphero BLE 이름 (R2, 스마트폰 트롤리)
FLASK_PORT      = 5001
FLOORPLAN_PDF   = os.path.join(os.path.dirname(__file__), "팔달관_도면.pdf")
SHAKE_THRESHOLD = 2.5          # G  — 이 이상이면 흔들기 신호로 판정
SIGNAL_COOLDOWN = 10           # 초  — 한 번 신호 후 재차단 시간

# ── 대피 경로 (팔달관 도면 실측 기준) ────────────────────────────────────────
# 구간 = (헤딩, 거리 m). 헤딩 0° = R2 스마트폰 셀피 카메라 정면 = 경로 첫 구간 방향.
# 90 = 우회전, 270 = 좌회전. 출발 전 대시보드 "방향 보정"으로 각 로봇 정면을 셀피 정면에 맞춰 0°로 저장.
# 속도 기본값 (0~255) — 대시보드 "주행 속도"에서 바꾸면 speed_settings.json에 저장돼 재시작 후에도 유지
DEFAULT_SPEEDS = {
    "floor2": 100,     # R1 (2층 선두)
    "rear":   150,     # R2 (2층 후방) — 스마트폰을 실어 무거우므로 더 세게. 실제 속도가 R1과 같아야 간격 유지
    "floor1": 100,     # R3 (1층)
}
DEFAULT_TURN_PAUSE_SEC = 1.0 # 꺾기 전 정지 시간 — 멈춘 동안 매트릭스에 꺾는 방향 화살표 (0~10초)
DEFAULT_R2_DIST_PCT = 100     # R2 거리 보정(%) — 폰 무게로 덜 가면 올림. 구간 시간에 곱해 같은 비율로 더 감 (50~200)
DEFAULT_R2_DELAY_SEC = 0.0   # R2(후방) 출발 지연 — 0이면 R1과 동시 출발해 R1이 지나간 경로를 4m 뒤에서 따라감 (0~30초)
SPEED_REF      = 100   # DRIVE_MPS를 잰 기준 속도 값
DRIVE_MPS      = 0.8   # R1이 SPEED_REF로 1초에 가는 거리(m) — 현장 바닥에서 실측 후 수정
                       # 측정: 3초 직진시켜 이동 거리 ÷ 3. 구간 시간 = 거리 ÷ (이 값 × 속도/SPEED_REF)
FOLLOW_GAP_M   = 4.0   # R1(전방)–R2(후방) 간격 기본값 — 대시보드에서 바꿈. R2는 경로 출발점, R1은 이만큼 앞에서 출발
FOLLOW_END_GAP_M = 1.0 # 도착 때 R2가 R1 뒤에 남기는 간격 — R2는 출발 4m 차이를 따라잡아 R1 바로 뒤까지 간다

# 2층 (현재 시연용): 중앙 복도 모서리(보이드 아래) 출발 → 직진 → 좌회전 → 계단(EXIT). 모든 재난 유형 공통
# 도면 전체 경로로 되돌릴 때: (0,0.9) (270,5.7) (0,5.7) (270,9.0) + 대시보드 ROUTE_2F 꺾임점 5개
ROUTE_2F_M = [
    (0,   5.7),        # ③ 19칸 — 직진 (중앙 복도)
    (270, 9.0),        # ④ 30칸 — 좌회전, 계단 앞 핸드오프
]
ROUTE_1F_M = [         # 1층: 137호 옆 계단(출발) → 로비 → 주출입구(EXIT)
    (0,   7.5),        # ① 25칸
    (90,  7.5),        # ② 25칸 — 우회전
]

_ARROW = {0: "^", 90: ">", 180: "v", 270: "<"}

def _slice_route(route, start_m, end_m):
    """경로 중 start_m ~ end_m 구간만 (헤딩, m) 목록으로 잘라 낸다 (꺾이는 지점은 그대로)"""
    out, pos = [], 0.0
    for h, m in route:
        a, b = max(pos, start_m), min(pos + m, end_m)
        if b - a > 1e-6:
            out.append((h, round(b - a, 2)))
        pos += m
    return out

def _route_to_path(route, end_char, speed):
    """(헤딩, m) 구간 → (속도, 헤딩, 지속 초, 매트릭스 문자) 주행 단계. 시간은 R1 기준 속도로 환산"""
    path = [(speed, h, round(m / DRIVE_MPS, 2), _ARROW.get(h % 360, "^")) for h, m in route]
    return path + [(0, route[-1][0], 0.0, end_char)]

def _path_meters(path):
    return [round(dur * DRIVE_MPS, 2) if spd > 0 else 0.0 for spd, _, dur, _ in path]

ROUTE_2F_LEN = sum(m for _, m in ROUTE_2F_M)

# 주행 단계: (속도, 헤딩, 지속 초, 매트릭스 표시 문자)
# R1·R2는 같은 경로 위를 달려 같은 지점에서 꺾는다. R1은 간격만큼 앞에서, R2는 출발점에서 출발해 R1 정지점 1m 뒤까지 간다
# 지속 시간은 SPEED_REF 기준값 — 실제 주행 때 설정 속도에 맞춰 _run_path가 늘리거나 줄인다
def _build_follow_paths(route, gap):
    """R2 출발점 기준 경로(route)에서 R1은 gap만큼 앞에서, R2는 출발점에서 같은 경로를 따라간다.
    R2는 R1 정지점 min(1m, gap) 뒤까지 — R1이 꺾은 지점에서 똑같이 꺾는다"""
    length = sum(m for _, m in route)
    end_gap = min(FOLLOW_END_GAP_M, gap)
    r1 = _route_to_path(_slice_route(route, gap, length), "v", DEFAULT_SPEEDS["floor2"])
    r2 = _route_to_path(_slice_route(route, 0, length - end_gap), "v", DEFAULT_SPEEDS["rear"])
    return r1, r2

def _build_2f_paths(gap):
    """간격(m)에 맞춘 2층 R1·R2 주행 단계"""
    return _build_follow_paths(ROUTE_2F_M, gap)

def _path_json(path):
    return [{"step": i + 1, "char": ch, "speed": spd, "heading": hdg, "duration": dur,
             "meters": round(dur * DRIVE_MPS, 2) if spd > 0 else 0}
            for i, (spd, hdg, dur, ch) in enumerate(path)]

GAP_MIN_M = 0.1                            # 10cm — 짧은 테스트용
GAP_MAX_M = round(ROUTE_2F_LEN - 0.5, 1)   # R1이 최소 0.5m는 가야 함
EVAC_PATH_2F, EVAC_PATH_R2 = _build_2f_paths(FOLLOW_GAP_M)
EVAC_PATH_1F = _route_to_path(ROUTE_1F_M, "!", DEFAULT_SPEEDS["floor1"])    # 마지막: 정지 (비상구 도달)

SPEED_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "speed_settings.json")

def _load_speeds():
    speeds = dict(DEFAULT_SPEEDS)
    try:
        with open(SPEED_FILE, encoding="utf-8") as f:
            saved = json.load(f)
        for k in speeds:
            if isinstance(saved.get(k), int) and 1 <= saved[k] <= 255:
                speeds[k] = saved[k]
    except (OSError, ValueError):
        pass
    return speeds

def _load_aim_trim():
    """로봇별 자동 에임 보정값(도) — 자동 에임 뒤 수동 미세 조정한 만큼 누적"""
    trim = {"floor2": 0, "rear": 0, "floor1": 0}
    try:
        with open(SPEED_FILE, encoding="utf-8") as f:
            saved = json.load(f).get("aim_trim") or {}
        for k in trim:
            if isinstance(saved.get(k), (int, float)):
                trim[k] = int(saved[k])
    except (OSError, ValueError, AttributeError):
        pass
    return trim

def _load_setting(key, default, lo, hi):
    try:
        with open(SPEED_FILE, encoding="utf-8") as f:
            v = json.load(f).get(key)
        if isinstance(v, (int, float)) and lo <= v <= hi:
            return float(v)
    except (OSError, ValueError, AttributeError):
        pass
    return default

# 안내 대상 모드 — mps 비율로 Sphero 속도를 낮추고 지속 시간을 늘려 같은 거리를 간다
TARGET_MODES = {
    "normal": {"label": "일반",     "icon": "🚶", "mps": 1.0, "led": "표준 화살표",     "voice": "기본 안내 1회"},
    "blind":  {"label": "시각장애", "icon": "🦯", "mps": 0.6, "led": "고휘도 단색 점멸", "voice": "상황·방향 반복 안내"},
    "elder":  {"label": "노인",     "icon": "🧓", "mps": 0.6, "led": "대형 화살표",     "voice": "주기적으로 또렷하게"},
    "child":  {"label": "어린이",   "icon": "🧒", "mps": 0.8, "led": "친근한 캐릭터",   "voice": "쉽고 짧은 문구"},
}
DISASTERS = {
    "fire":  {"label": "화재", "icon": "🔥", "src": "서버 신호",
              "note": "발화실 번호를 받는 즉시 시작 (시연: 대시보드 입력으로 수신반 신호 대체)"},
    "quake": {"label": "지진", "icon": "🌍", "src": "가속도계 감지",
              "note": "흔들리는 동안 음성 안내만 — 진동이 멈추고 2초 뒤 유도 시작"},
    "black": {"label": "정전", "icon": "💡", "src": "조도 센서 감지",
              "note": "조도 10 lux 미만 3초 지속 시 시작 — 네트워크 없이 저장 경로로 자립 유도"},
}
EXCEPTIONS = {
    "E1": "센서 오인식 — 3초 미만 조도 하강 무시",
    "E2": "경로 이탈 — IR 2초 끊기면 전 로봇 일시정지",
    "E3": "장애물 충돌 — 0.3m 후진 후 회피 재시도",
    "E4": "통신 단절 — 저장 경로로 계속 유도",
    "E5": "로봇 고장 — 예비 로봇이 역할 넘겨받아 투입",
}
QUAKE_SETTLE_SEC = 2.0
EXCEPTION_SHOW_SEC = 4.0

# ── 공유 상태 ─────────────────────────────────────────────────────────────────
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

# ── 신호 처리 ─────────────────────────────────────────────────────────────────
_bolt_2f   = None
_bolt_1f   = None
_bolt_r2   = None
_triggered = False
_trigger_lock    = threading.Lock()
_broker_stop_evt = threading.Event()
_trigger_ts      = 0.0
_last_shake_ts   = 0.0
_last_motion_end = 0.0  # 로봇이 마지막으로 멈춘 시각 — 직후엔 흔들기를 지진으로 보지 않음
SHAKE_QUIET_SEC  = 3.0  # 주행·복귀가 끝나고 이 시간 동안은 흔들기 무시

def _robots_in_motion() -> bool:
    return any(state[k]["moving"] or state[k].get("returning") for k in ("floor2", "rear", "floor1"))

def _mark_motion_end():
    global _last_motion_end
    _last_motion_end = time.time()
_run_gen         = 0    # 트리거·리셋마다 증가 — 이전 세대 주행 스레드는 스스로 멈춘다

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

def trigger_signal(source: str = "unknown", disaster: str = None):
    """재난 신호 수신 → 2F(R1·R2) 유도 → 계단 인수인계 → 1F(R3) 유도"""
    global _triggered, _trigger_ts, _run_gen
    with _trigger_lock:
        if _triggered:
            log_event(f"⚠ 중복 신호 무시 ({source})")
            return
        _triggered = True
        _run_gen += 1
        gen = _run_gen

    disaster = disaster or "fire"
    _trigger_ts = time.time()
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
    _set_led(_bolt_2f, 255, 0, 0)
    _set_led(_bolt_r2, 255, 0, 0)
    t = threading.Thread(target=_run_2f_then_1f, args=(gen,), daemon=True)
    t.start()

    # cooldown 후 신호 리셋
    threading.Thread(target=_reset_signal_after, args=(SIGNAL_COOLDOWN, gen), daemon=True).start()

def _reset_signal_after(seconds: float, gen: int):
    global _triggered
    time.sleep(seconds)
    if gen != _run_gen:
        return
    with _state_lock:
        state["floor2"]["signal"] = False
    with _trigger_lock:
        _triggered = False
    _set_led(_bolt_2f, 0, 255, 0)
    _set_led(_bolt_r2, 0, 255, 0)
    threading.Thread(target=_scroll_connected, args=(_bolt_2f, "2", 0, 255, 0), daemon=True).start()
    log_event("✅ 신호 리셋 — 대기 상태 복귀")

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

def _run_2f_then_1f(gen):
    """2F 경로(R1 선두 + R2 후방 추종) 완료 → 1F 자동 출발 (연결된 쪽만 실행)"""
    def live():
        return gen == _run_gen

    if state["disaster"] == "quake":
        log_event("  진동 중 — 음성 안내만, 진동 종료 대기")
        while live() and time.time() - max(_last_shake_ts, _trigger_ts) < QUAKE_SETTLE_SEC:
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
        if not _run_path(_bolt_2f, path_r1, "floor2", "2F",
                         move_color=(255, 80, 0), done_color=(255, 0, 0), gen=gen):
            return
        log_event("🏁 2F 계단 앞 도달 → 중계기가 R3에 출발 명령")
    else:
        log_event("ℹ️  2F 미연결 — 1F 직접 출발")
    if not live():
        return
    _set_phase("handoff")
    if _active("floor1"):
        if not _run_path(_bolt_1f, EVAC_PATH_1F, "floor1", "1F",
                         move_color=(0, 100, 255), done_color=(0, 255, 0), gen=gen):
            return
        log_event("🚪 R3 비상구 도달")
    _set_phase("complete")

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

def _mps(key, speed):
    """속도 값 → 실제 m/s 추정 (R2는 거리 보정만큼 덜 감)"""
    v = DRIVE_MPS * speed / SPEED_REF
    if key == "rear":
        v /= state["r2_dist_pct"] / 100
    return v

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
        if gen != _run_gen:
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
    bolt = {"floor2": _bolt_2f, "rear": _bolt_r2, "floor1": _bolt_1f}[key]
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
            while time.time() - t0 < secs and gen == _run_gen:
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
            if gen != _run_gen:
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

# ── 위치 센서(locator) 기반 주행 ─────────────────────────────────────────────
# BOLT가 알려 주는 자기 위치(cm, x = 오른쪽 + / y = 앞 +)를 보며 목표 지점까지 간다.
# 시간으로만 가던 방식과 달리 미끄러짐·트롤리 무게·출발/정지 관성이 쌓이지 않고, 복귀는 좌표로 돌아간다.
LOC_INTERVAL_MS    = 100    # 위치 스트리밍 주기 (ms)
LOC_MAX_AGE        = 1.0    # 이보다 오래된 위치는 못 믿음 (초)
NAV_STOP_CM        = 4.0    # 목표까지 이만큼 남으면 정지 — 굴러가는 관성 고려
NAV_SLOW_CM        = 30.0   # 이 거리 안에서 감속
NAV_MIN_SPEED      = 50     # 감속 하한 속도 값 — 이보다 낮으면 Sphero가 거의 안 움직임 (설정 속도가 더 낮으면 그 값)
NAV_MAX_STEER_DEG  = 25     # 구간 방향에서 벗어나 보정할 수 있는 최대 각
LOC_ROT_SNAP_DEG   = 15.0   # 좌표축 보정이 이보다 작으면 0° — 트롤리에 끌린 출발 흔들림을 보정값으로 오인하지 않게
NAV_CALIB_CM       = 15.0   # 첫 구간에서 위치 좌표축 회전을 추정할 최소 이동 거리
NAV_TIMEOUT_FACTOR = 3.0    # 예상 시간 × 이 값 + 3초가 지나도 도착 못 하면 다음 구간으로
NAV_HOME_TOL_CM    = 8.0    # 복귀 때 이 안이면 도착으로 봄

_ROBOT_KEYS = ("floor2", "rear", "floor1")
_loc      = {k: None for k in _ROBOT_KEYS}                     # 원시 위치 (x, y cm, 받은 시각)
_loc_rot  = {k: None for k in _ROBOT_KEYS}                     # 위치 좌표축 → 헤딩 좌표 회전(도), 첫 구간에서 추정
_sim_pose = {k: [0.0, 0.0, 0, 0, time.time()] for k in _ROBOT_KEYS}  # SIM: x, y, 헤딩, 속도, 시각
_wpts     = {k: [] for k in _ROBOT_KEYS}                       # 실제로 지난 지점 (헤딩 좌표 cm), [0] = 출발점
# R2 U턴이 실제로 옮겨 놓는 거리 — side: U턴 방향 옆으로, along: 다가온 방향으로 (앞으로 밀리면 음수)
# 복귀할 때마다 실측해 반씩 섞고 설정 파일에 저장 (재시작해도 유지)
_uturn_learned = {"side": _load_setting("uturn_side", 35.0, 0, 200),
                  "along": _load_setting("uturn_along", -25.0, -150, 150)}

def _bolt_of(key):
    return {"floor2": _bolt_2f, "rear": _bolt_r2, "floor1": _bolt_1f}[key]

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
    if p[3] > 0:
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
    return _unrot(raw[0], raw[1], _loc_rot[key])

def _nav_available(key, bolt):
    return (not _is_real(bolt)) or _raw(key) is not None

def _reset_origin(key, bolt):
    """에임·연결 직후 — 지금 자리를 위치 원점으로"""
    if _is_real(bolt):
        try:
            bolt.reset_locator_x_and_y()
        except Exception as e:
            log_event(f"  [위치 원점 리셋 오류] {e}")
    _loc_rot[key] = None
    _sim_pose[key][:4] = [0.0, 0.0, 0, 0]
    with _trail_lock:
        _wpts[key] = []
        _trail[key] = []

def _nav_to(key, bolt, ctx, target_of, heading, speed, cancelled, calib=True, hold=None):
    """ctx["start"] 기준 목표점까지 heading 방향 구간을 위치를 보며 주행.
    target_of(start) → 목표 (헤딩 좌표 cm). 반환 'ok' | 'cancel' | 'timeout' | 'noloc'"""
    u = _vec(heading)
    calib_from, deadline = None, None
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
                        ctx["start"] = _unrot(ctx["raw0"][0], ctx["raw0"][1], rot)
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
        if along <= NAV_STOP_CM:
            return "ok"
        if time.time() > deadline:
            return "timeout"
        dist = math.hypot(rx, ry)
        dev = 0.0 if dist < 15 else max(-NAV_MAX_STEER_DEG, min(NAV_MAX_STEER_DEG, _norm180(_angle_of(rx, ry) - heading)))
        spd = speed if along > NAV_SLOW_CM else max(min(NAV_MIN_SPEED, speed), int(speed * along / NAV_SLOW_CM))
        _drive(key, bolt, round(heading + dev), spd)
        time.sleep(0.05)

def _run_path_nav(bolt, path, state_key, label, move_color, done_color, gen=None):
    """위치 기반 경로 실행 — 구간 끝 지점을 좌표로 정해 놓고 도착할 때까지 보정하며 간다"""
    def cancelled():
        return gen is not None and gen != _run_gen

    def mark(p):
        with _trail_lock:
            if not _wpts[state_key] and ctx.get("start") is not None:
                _wpts[state_key].append(ctx["start"])     # 축 보정 뒤에야 출발점이 정해진 경우
            if p is not None:
                _wpts[state_key].append(p)

    def finish(ok):
        mark(_pos(state_key, bolt))
        _halt(state_key, bolt, prev_heading or 0)
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
    prev_heading, leg = None, 0
    log_event(f"  📍 {label} 위치 기반 주행")

    for i, (spd, heading, dur, char) in enumerate(path):
        if cancelled():
            return finish(False)
        m = meters[i]
        if m > 0 and prev_heading is not None and heading != prev_heading:
            mark(_pos(state_key, bolt))
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
        with _state_lock:
            state[state_key]["step"] = i + 1
        _set_matrix(bolt, *move_color, "^" if m > 0 else char)
        if m <= 0:
            log_event(f"  → {label} 정지 (도착)  [{char}]")
            continue
        prev_heading = heading
        log_event(f"  → {label} step {i+1}/{len(path)}: 속도={speed}  방향={heading}°  {m:g}m  [{char}]")
        if i == 0 and state_key in ("floor2", "rear") and state["latency"] is None and not _test_running["on"]:
            lat = round(time.time() - _trigger_ts, 2)
            with _state_lock:
                state["latency"] = lat
            log_event(f"⏱ 트리거→선두 출발 {lat:.1f}s (목표 ≤ 2.0s)")
        k = leg + 1
        t0 = time.time()
        res = _nav_to(state_key, bolt, ctx, lambda st, k=k: (st[0] + offs[k][0], st[1] + offs[k][1]),
                      heading, speed, cancelled)
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

def _return_home_nav(key, gen):
    label = _AIM_LABEL[key]
    bolt = _bolt_of(key)
    cancelled = lambda: gen != _run_gen
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
            if not state["rear"].get("returning"):
                return False
            a = _common("floor2", _pos("floor2", _bolt_2f))
            b2 = _pos("rear", _bolt_r2)
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
        for tgt in path_pts:
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

# ── R2: R1 좌표 따라가기 ────────────────────────────────────────────────────
# 공통 좌표 = R2 출발점 기준 (R1·R2는 같은 방향으로 에임, R1은 간격만큼 앞에 놓음).
# R1이 실제로 지나간 자리를 발자국으로 남기고, R2는 경로 거리로 항상 간격만큼 뒤 지점을 쫓는다.
FOLLOW_CRUMB_CM     = 3.0    # R1 발자국 간격
FOLLOW_LOOKAHEAD_CM = 15.0   # R2가 바라보는 앞쪽 거리 — 작을수록 모서리를 정확히, 클수록 부드럽게
FOLLOW_TOL_CM       = 4.0    # 목표 간격까지 이만큼 남으면 멈춤
FOLLOW_STALL_SEC    = 15.0   # R1이 끝났는데 이만큼 진척이 없으면 종료

def _common(key, p):
    """각 로봇 좌표 → R2 출발점 기준 공통 좌표"""
    if p is None:
        return None
    if key == "floor2":
        return (p[0], p[1] + state["follow_gap"] * 100)
    return p

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

def _calibrate_axes(key, bolt, heading, speed, cancelled):
    """위치 좌표축 회전을 모를 때 heading 방향으로 15cm 곧게 가며 추정. 반환: 출발 위치(헤딩 좌표) 또는 None"""
    raw0 = _raw(key)
    if raw0 is None:
        return None
    deadline = time.time() + max(4.0, 0.3 / max(_mps(key, speed), 0.05) * 3)
    while not cancelled() and time.time() < deadline:
        raw = _raw(key)
        if raw is None:
            break
        dx, dy = raw[0] - raw0[0], raw[1] - raw0[1]
        if math.hypot(dx, dy) >= NAV_CALIB_CM:
            _loc_rot[key] = _snap_rot(_norm180(_angle_of(dx, dy) - heading))
            log_event(f"  📐 {_AIM_LABEL[key]} 위치 좌표축 보정 {_loc_rot[key]:+.0f}°")
            return _unrot(raw0[0], raw0[1], _loc_rot[key])
        _drive(key, bolt, heading, speed)
        time.sleep(0.05)
    _halt(key, bolt, heading)
    return None

FOLLOW_CORNER_DEG     = 50.0   # R1 발자국이 이만큼 꺾이면 모서리로 봄
FOLLOW_CORNER_STOP_CM = 12.0   # 모서리 이만큼 앞에서 멈춤 — 멈춘 뒤 트롤리에 밀려 미끄러지는 거리만큼

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

def _run_r2_follow(gen):
    """R2가 R1 발자국을 간격만큼 뒤에서 따라감. R1이 끝나면 도착 간격까지 다가가 멈춤"""
    key, bolt, label = "rear", _bolt_r2, "R2"
    cancelled = lambda: gen != _run_gen
    gap = state["follow_gap"] * 100
    end_gap = min(FOLLOW_END_GAP_M, state["follow_gap"]) * 100
    speed = min(255, int(state["speeds"][key] * TARGET_MODES[state["target"]]["mps"]))
    p0 = _pos(key, bolt)
    if p0 is None and _is_real(bolt) and _raw(key) is not None and _loc_rot[key] is None:
        p0 = _calibrate_axes(key, bolt, 0, speed, cancelled)    # 0° = R1 출발점 쪽 (경로 위)
    if p0 is None:
        log_event(f"  ⏱ {label} 위치 센서 없음 — 계획 경로로 주행")
        _, path_r2 = _build_2f_paths(state["follow_gap"])
        return _run_path_time(bolt, path_r2, key, label, (255, 180, 0), (255, 0, 0), gen)
    crumbs = [(p0[0], p0[1], 0.0), (p0[0], p0[1] + gap, gap)]      # R2 출발점 → R1 출발점
    with _trail_lock:
        if not _wpts[key]:
            _wpts[key].append(p0)
    track = [p0]
    with _state_lock:
        state[key].update(moving=True, done=False, step=1, progress_m=0.0)
    _set_led(bolt, 255, 180, 0)
    _set_matrix(bolt, 255, 180, 0, "^")
    log_event(f"  📍 {label} R1 좌표 따라가기 — 간격 {gap:g}cm, 도착 간격 {end_gap:g}cm")
    _trace(None)
    _trace("t,R1x,R1y,R1s,R2x,R2y,R2s,target_s,heading,moving,leader_done")
    r2_s, idx, moving, last_s, last_t, cur_h = 0.0, 0, False, 0.0, time.time(), 0
    ok, leader_seen, t_start = False, False, time.time()
    corner_done_s = -1e9            # 이미 멈춰 돈 모서리 (다시 잡지 않게)

    def feed():
        lp = _common("floor2", _pos("floor2", _bolt_2f))
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
        if leader_done and time.time() - last_t > FOLLOW_STALL_SEC:
            log_event(f"⚠ {label} 진척 없음 — 따라가기 종료 (남은 {remaining:.0f}cm)")
            break
        # 모서리: R1처럼 앞에서 감속 → 멈춤 → 제자리에서 돌고 출발 (트롤리 관성으로 지나치지 않게)
        corner = _crumb_corner(crumbs, max(r2_s, corner_done_s + 20), r2_s + 40)
        if corner is not None and corner[0] > target_s:
            corner = None                       # 목표 지점 너머 모서리 — 지금은 상관없음
        if corner is not None:
            cs, new_h = corner
            to_c = cs - r2_s
            if to_c <= FOLLOW_CORNER_STOP_CM:
                _halt(key, bolt, cur_h)
                moving = False
                log_event(f"  ↪ {label} 모서리 앞 정지 → {new_h:.0f}° 방향으로 회전")
                t_end = time.time() + state["turn_pause"]
                while time.time() < t_end and not cancelled():
                    feed()
                    time.sleep(0.05)
                cur_h = round(new_h) % 360
                _drive(key, bolt, cur_h, 0)          # 제자리 회전
                t_end = time.time() + 0.6
                while time.time() < t_end and not cancelled():
                    feed()
                    time.sleep(0.05)
                corner_done_s = cs
                last_t = time.time()
                continue
            cx, cy = _crumb_point(crumbs, cs)
            cur_h = round(_angle_of(cx - p[0], cy - p[1]))
            spd = max(min(NAV_MIN_SPEED, speed), min(speed, int(speed * to_c / NAV_SLOW_CM)))
            _drive(key, bolt, cur_h, spd)
            moving = True
            time.sleep(0.05)
            continue
        tx, ty = _crumb_point(crumbs, min(r2_s + FOLLOW_LOOKAHEAD_CM, target_s))
        cur_h = round(_angle_of(tx - p[0], ty - p[1]))
        spd = speed if remaining > NAV_SLOW_CM else max(min(NAV_MIN_SPEED, speed), int(speed * remaining / NAV_SLOW_CM))
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
    elif cancelled():
        log_event(f"⏹ {label} 따라가기 중단 (상황 해제)")
    return ok

def _run_r2_delayed(gen, delay, path):
    """R2는 delay초 뒤 출발 — 대기 중 상황 해제되면 출발하지 않음"""
    if delay > 0:
        log_event(f"  R2 {delay:g}초 뒤 출발")
        end = time.time() + delay
        while time.time() < end:
            if gen != _run_gen:
                return
            time.sleep(0.05)
    if _active("floor2"):
        return _run_r2_follow(gen)
    _run_path(_bolt_r2, path, "rear", "R2",
              move_color=(255, 180, 0), done_color=(255, 0, 0), gen=gen)

def _run_path_time(bolt, path, state_key, label, move_color, done_color, gen=None):
    """범용 경로 실행 — 2F/1F 공통. 상황 해제(세대 변경) 시 멈추고 False 반환"""
    def cancelled():
        return gen is not None and gen != _run_gen

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
                lat = round(time.time() - _trigger_ts, 2)
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

# ── 2F 가속도 센서 콜백 ────────────────────────────────────────────────────────
def _on_2f_sensor(data: dict):
    """sensor_control 콜백 — data = {'accelerometer': {'x':..,'y':..,'z':..}}"""
    try:
        acc = data.get('accelerometer', {})
        ax = float(acc.get('x', 0) or 0)
        ay = float(acc.get('y', 0) or 0)
        az = float(acc.get('z', 0) or 0)

        with _state_lock:
            state["floor2"]["accel"] = {
                "x": round(ax, 2), "y": round(ay, 2), "z": round(az, 2)}

        magnitude = (ax**2 + ay**2 + az**2) ** 0.5
        if magnitude > SHAKE_THRESHOLD:
            if _robots_in_motion() or time.time() - _last_motion_end < SHAKE_QUIET_SEC:
                return   # 로봇 자신의 주행·회전·충돌 — 지진 아님
            global _last_shake_ts
            _last_shake_ts = time.time()
            trigger_signal(f"2F-shake({magnitude:.1f}G)", "quake")
    except Exception:
        pass

# ── Sphero 연결 ───────────────────────────────────────────────────────────────
def _connect_sphero(name: str, label: str):
    """BLE 스캔 후 연결. 실패 시 None 반환."""
    try:
        from spherov2 import scanner
        log_event(f"🔍 {label} Sphero 스캔 중: {name} … (최대 30초)")
        toy = scanner.find_toy(toy_name=name)
        log_event(f"✅ {label} Sphero 연결: {name}")
        return toy
    except Exception as e:
        msg = str(e) or "BLE 스캔 타임아웃 또는 기기 미발견"
        log_event(f"❌ {label} Sphero 연결 실패: {msg}")
        log_event(f"  💡 Sphero Edu 앱에서 {name} 연결 해제 후 재시도하세요")
        return None

def _broker_main(sim: bool):
    """브로커 메인 루프 (별도 데몬 스레드)"""
    global _bolt_2f, _bolt_1f, _bolt_r2

    if sim:
        log_event("🟡 시뮬레이션 모드 — Sphero 없이 대시보드 동작")

        class _Dummy:
            class _DummyDrive:
                def roll_start(self, *a, **kw): pass
                def roll_stop(self, *a, **kw):  pass
            class _DummyLed:
                def set_leds(self, *a, **kw): pass
            class _DummySensor:
                def enable(self, *a, **kw): pass
                def set_interval(self, *a, **kw): pass
                def add_sensor_data_listener(self, *a, **kw): pass
            def wake(self, *a, **kw): pass
            def get_battery_percentage(self): return 0
            def set_compressed_frame_player_one_color(self, *a, **kw): pass
            def set_compressed_frame_player_single_character(self, *a, **kw): pass
            def add_sensor_streaming_data_notify_listener(self, *a, **kw): pass
            def remove_sensor_streaming_data_notify_listener(self, *a, **kw): pass
            @property
            def drive_control(self):     return self._DummyDrive()
            @property
            def multi_led_control(self): return self._DummyLed()
            @property
            def sensor_control(self):    return self._DummySensor()

        global _bolt_2f, _bolt_1f, _bolt_r2
        _bolt_2f = _Dummy()
        _bolt_1f = _Dummy()
        _bolt_r2 = _Dummy()
        # SIM 모드: 실제 BLE 연결 아님 → connected는 False 유지
        while True:
            time.sleep(30)
        return

    with _state_lock:
        name_2f = state["names"]["floor2"]
        name_1f = state["names"]["floor1"]
        name_r2 = state["names"]["rear"]

    if name_r2 and name_r2 in (name_2f, name_1f):
        log_event(f"⚠ R2 이름({name_r2})이 다른 슬롯과 같음 — R2 연결 스킵")
        name_r2 = ""

    toy_2f = _connect_sphero(name_2f, "2층(R1)") if name_2f else None
    toy_1f = _connect_sphero(name_1f, "1층(R3)") if name_1f else None
    toy_r2 = _connect_sphero(name_r2, "2층 후방(R2)") if name_r2 else None

    with _state_lock:
        state["floor2"]["connected"] = bool(toy_2f)
        state["floor1"]["connected"] = bool(toy_1f)
        state["rear"]["connected"]   = bool(toy_r2)
        state["connecting"] = False
        state["aim"] = {k: {"status": "", "msg": ""} for k in ("floor2", "rear", "floor1")}
    _clear_trail()

    connected = [f for f, t in [("2F", toy_2f), ("1F", toy_1f), ("R2", toy_r2)] if t]
    skipped   = [f for f, n in [("2F", name_2f), ("1F", name_1f), ("R2", name_r2)] if not n]
    if skipped:
        log_event(f"ℹ️  {', '.join(skipped)} 이름 없음 — 스킵")
    if not connected:
        log_event("⚠ 연결된 Sphero가 없습니다")
    else:
        log_event(f"🟢 연결 완료: {', '.join(connected)}")

    # BOLT 컨텍스트 매니저: 연결된 것만 열기
    # (toy가 None이면 더미 클래스로 대체)
    class _Dummy:
        """Sphero 없을 때 오류 없이 넘어가는 더미"""
        class _DummyDrive:
            def roll_start(self, *a, **kw): pass
            def roll_stop(self, *a, **kw):  pass
        class _DummyLed:
            def set_leds(self, *a, **kw): pass
        class _DummySensor:
            def enable(self, *a, **kw): pass
            def set_interval(self, *a, **kw): pass
            def add_sensor_data_listener(self, *a, **kw): pass
        def __enter__(self):   return self
        def __exit__(self, *_): pass
        def wake(self, *a, **kw): pass
        def get_battery_percentage(self): return 0
        def set_compressed_frame_player_one_color(self, *a, **kw): pass
        def set_compressed_frame_player_single_character(self, *a, **kw): pass
        def add_sensor_streaming_data_notify_listener(self, *a, **kw): pass
        def remove_sensor_streaming_data_notify_listener(self, *a, **kw): pass
        @property
        def drive_control(self):  return self._DummyDrive()
        @property
        def multi_led_control(self): return self._DummyLed()
        @property
        def sensor_control(self): return self._DummySensor()

    # find_toy()가 이미 BOLT 인스턴스를 반환 — 그대로 컨텍스트 매니저로 사용
    ctx_2f = toy_2f if toy_2f else _Dummy()
    ctx_1f = toy_1f if toy_1f else _Dummy()
    ctx_r2 = toy_r2 if toy_r2 else _Dummy()

    with ctx_2f as b2, ctx_1f as b1, ctx_r2 as br:
        _bolt_2f, _bolt_1f, _bolt_r2 = b2, b1, br

        # wake → 초기 LED: 녹색 + 매트릭스 "Floor N" 스크롤
        for bolt, floor, num in [(b2, "2F", "2"), (b1, "1F", "1"), (br, "R2", "2")]:
            try:
                bolt.wake()
            except Exception:
                pass
            _set_led(bolt, 0, 255, 0)
            threading.Thread(
                target=_scroll_connected, args=(bolt, num, 0, 255, 0), daemon=True
            ).start()
            log_event(f"💡 {floor} 매트릭스 'Floor {num}' 스크롤 시작")

        # 센서 스트리밍: R1 가속도(흔들기 = 지진) + 세 로봇 위치(locator)
        for bolt, key in ((b2, "floor2"), (br, "rear"), (b1, "floor1")):
            if not _is_real(bolt):
                continue
            try:
                if hasattr(bolt, "set_locator_flags"):
                    bolt.set_locator_flags(False)
                bolt.sensor_control.set_interval(LOC_INTERVAL_MS)
                if key == "floor2":
                    bolt.sensor_control.enable('accelerometer', 'locator')
                    bolt.sensor_control.add_sensor_data_listener(_on_2f_sensor)
                else:
                    bolt.sensor_control.enable('locator')
                bolt.sensor_control.add_sensor_data_listener(_make_loc_listener(key))
                _reset_origin(key, bolt)
                log_event(f"📡 {_AIM_LABEL[key]} 위치 센서 스트리밍 시작" + (" + 가속도(흔들면 지진)" if key == "floor2" else ""))
            except Exception as e:
                log_event(f"⚠ {_AIM_LABEL[key]} 센서 스트리밍 불가: {e} — 시간 기반으로 주행")

        log_event("🟢 브로커 준비 완료 — 신호 대기 중")

        # 하트비트: 30초마다 배터리 조회 (stop event로 중단 가능)
        _broker_stop_evt.clear()
        while not _broker_stop_evt.wait(30):
            try:
                bat2 = b2.get_battery_percentage()
                bat1 = b1.get_battery_percentage()
                batr = br.get_battery_percentage()
                with _state_lock:
                    state["floor2"]["battery"] = bat2
                    state["floor1"]["battery"] = bat1
                    state["rear"]["battery"]   = batr
                log_event(f"🔋 배터리 — 2F: {bat2}%  1F: {bat1}%  R2: {batr}%")
            except Exception:
                pass

    # with 블록 종료 → BLE 연결 해제됨
    _bolt_2f = None
    _bolt_1f = None
    _bolt_r2 = None
    with _state_lock:
        state["floor2"]["connected"] = False
        state["floor1"]["connected"] = False
        state["rear"]["connected"]   = False
    log_event("🔌 Sphero 연결 해제 완료")

# ── Flask 대시보드 ─────────────────────────────────────────────────────────────
app = Flask(__name__)
sock = Sock(app)

@app.route("/")
def dashboard():
    return render_template_string(_DASHBOARD_HTML)

@app.route("/floorplan")
def floorplan():
    from flask import send_file, Response
    try:
        return send_file(FLOORPLAN_PDF, mimetype="application/pdf")
    except FileNotFoundError:
        return Response("팔달관_도면.pdf 파일을 찾을 수 없습니다.", status=404)

BRAND_FILES = {"logo": "bandi_logo.png", "symbol": "bandi_symbol.png", "favicon": "favicon.png"}

@app.route("/brand/<name>")
def brand_img(name):
    from flask import send_file, Response
    if name not in BRAND_FILES:
        return Response("not found", status=404)
    try:
        return send_file(os.path.join(os.path.dirname(__file__), "brand", BRAND_FILES[name]),
                         mimetype="image/png", max_age=86400)
    except FileNotFoundError:
        return Response(f"brand/{BRAND_FILES[name]} 파일을 찾을 수 없습니다.", status=404)

@app.route("/floorplan-img/<floor>")
def floorplan_img(floor):
    from flask import send_file, Response
    if floor not in ("1f", "2f"):
        return Response("not found", status=404)
    try:
        return send_file(os.path.join(os.path.dirname(__file__), f"floorplan_{floor}.png"),
                         mimetype="image/png")
    except FileNotFoundError:
        return Response(f"floorplan_{floor}.png 파일을 찾을 수 없습니다.", status=404)

@app.route("/api/status")
def api_status():
    with _state_lock:
        return jsonify({
            "floor2":        state["floor2"],
            "floor1":        state["floor1"],
            "rear":          state["rear"],
            "trigger_count": state["trigger_count"],
            "phase":         state["phase"],
            "disaster":      state["disaster"],
            "target":        state["target"],
            "exceptions":    state["exceptions"],
            "latency":       state["latency"],
            "log":           state["log"][:30],
            "sim_mode":      state["sim_mode"],
            "names":         state["names"],
            "connecting":    state["connecting"],
            "server_ip":     _lan_ip(),
            "phone":         {**state["phone"],
                              "age": round(time.time() - state["phone"]["ts"], 1) if state["phone"]["ts"] else None},
            "aim":           state["aim"],
            "speeds":        state["speeds"],
            "r2_delay":      state["r2_delay"],
            "turn_pause":    state["turn_pause"],
            "aim_trim":      state["aim_trim"],
            "r2_dist_pct":   state["r2_dist_pct"],
            "follow_gap":    state["follow_gap"],
            "test_running":  _test_running["on"],
            "positions":     {k: (lambda p: {"x": round(p[0]), "y": round(p[1])} if p else None)(_pos(k, _bolt_of(k)))
                              for k in _ROBOT_KEYS},
            "paths":         dict(zip(("floor2", "rear"),
                                      map(_path_json, _build_2f_paths(state["follow_gap"])))),
        })

@app.route("/api/connect", methods=["POST"])
def api_connect():
    from flask import request as req
    data = req.get_json(silent=True) or {}
    name_2f = (data.get("name_2f") or "").strip()
    name_1f = (data.get("name_1f") or "").strip()
    name_r2 = (data.get("name_r2") or "").strip()
    if not name_2f and not name_1f and not name_r2:
        return jsonify({"ok": False, "error": "이름을 최소 하나 입력하세요"}), 400

    with _state_lock:
        if state["connecting"]:
            return jsonify({"ok": False, "error": "이미 연결 시도 중입니다"}), 409
        state["names"]["floor2"] = name_2f   # 비워도 클리어
        state["names"]["floor1"] = name_1f   # 비워도 클리어
        state["names"]["rear"]   = name_r2   # 비워도 클리어
        state["floor2"]["connected"] = False
        state["floor1"]["connected"] = False
        state["rear"]["connected"]   = False
        state["connecting"] = True

    global _bolt_2f, _bolt_1f, _bolt_r2
    _bolt_2f = None
    _bolt_1f = None
    _bolt_r2 = None

    log_event(f"🔌 재연결 요청 — 2F: {name_2f}  1F: {name_1f}  R2: {name_r2}")
    t = threading.Thread(target=_broker_main, args=(False,), daemon=True)
    t.start()
    return jsonify({"ok": True})

@app.route("/api/trigger", methods=["POST"])
def api_trigger():
    trigger_signal("대시보드-수동")
    return jsonify({"ok": True})

TEST_ROUTE_M = [(0, 1.0), (270, 1.5)]   # 테스트: R1 기준 직진 1m → 좌회전 1.5m (ㄱ자). R2는 간격만큼 뒤에서 같은 경로
_test_running = {"on": False}

def _test_paths(gap):
    """테스트 경로 — R1·R2는 실제 2층과 같은 따라가기 구조, R3는 혼자 ㄱ자"""
    route = [(TEST_ROUTE_M[0][0], gap + TEST_ROUTE_M[0][1])] + TEST_ROUTE_M[1:]   # R2 출발점 기준
    r1, r2 = _build_follow_paths(route, gap)
    return {"floor2": r1, "rear": r2, "floor1": _route_to_path(TEST_ROUTE_M, "v", DEFAULT_SPEEDS["floor1"])}

def _run_test_path(key, gen, path):
    label = _AIM_LABEL[key]
    bolt = {"floor2": _bolt_2f, "rear": _bolt_r2, "floor1": _bolt_1f}[key]
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

@app.route("/api/test_route", methods=["POST"])
def api_test_route():
    """테스트 경로 — 연결된 로봇마다 제자리에서 직진 1m → 좌회전 1.5m. 상황 해제로 출발점 복귀"""
    global _run_gen
    keys = [k for k in ("floor2", "rear", "floor1") if _active(k)]
    if not keys:
        return jsonify({"ok": False, "error": "연결된 로봇 없음"}), 400
    if _robots_in_motion():
        return jsonify({"ok": False, "error": "주행·복귀 중 — 끝난 뒤 실행"}), 409
    with _trigger_lock:
        _run_gen += 1
        gen = _run_gen
    with _state_lock:
        _test_running["on"] = True
    names = ", ".join(_AIM_LABEL[k] for k in keys)
    paths = _test_paths(state["follow_gap"])
    desc = lambda k: " → ".join(f"{'직진' if h == 0 else '좌회전' if h == 270 else '우회전'} {m:g}m"
                                for (h, m) in [(p[1], mm) for p, mm in zip(paths[k], _path_meters(paths[k])) if mm])
    for k in keys:
        if k == "rear" and "floor2" in keys:
            log_event(f"  R2(테스트): R1 좌표를 {state['follow_gap'] * 100:g}cm 뒤에서 따라감")
        else:
            log_event(f"  {_AIM_LABEL[k]}(테스트): {desc(k) or '이동 없음'}")
    log_event(f"🧪 테스트 경로 시작 ({names}) — 직진 {TEST_ROUTE_M[0][1]:g}m → 좌회전 {TEST_ROUTE_M[1][1]:g}m")
    for k in keys:
        threading.Thread(target=_run_test_path, args=(k, gen, paths[k]), daemon=True).start()
    return jsonify({"ok": True, "robots": names})

@app.route("/api/disaster", methods=["POST"])
def api_disaster():
    kind = (flask_request.get_json(silent=True) or {}).get("type")
    if kind == "clear":
        api_reset()
        log_event("🟢 상황 해제 — 전 로봇 평시 복귀")
        gen = _run_gen
        if _active("rear"):
            with _state_lock:
                state["rear"]["returning"] = True      # R1이 R2보다 먼저 움직이지 않게
        for k in ("floor2", "rear", "floor1"):
            if _active(k):
                threading.Thread(target=_return_home, args=(k, gen), daemon=True).start()
        return jsonify({"ok": True})
    if kind not in DISASTERS:
        return jsonify({"ok": False, "error": "unknown type"}), 400
    trigger_signal("대시보드", kind)
    return jsonify({"ok": True})

@app.route("/api/target", methods=["POST"])
def api_target():
    mode = (flask_request.get_json(silent=True) or {}).get("mode")
    if mode not in TARGET_MODES:
        return jsonify({"ok": False, "error": "unknown mode"}), 400
    with _state_lock:
        state["target"] = mode
    m = TARGET_MODES[mode]
    log_event(f"🎯 안내 대상 → {m['icon']} {m['label']} ({m['mps']} m/s · {m['led']}) — 다음 주행부터 적용")
    return jsonify({"ok": True})

@app.route("/api/exception", methods=["POST"])
def api_exception():
    eid = (flask_request.get_json(silent=True) or {}).get("id")
    if eid not in EXCEPTIONS:
        return jsonify({"ok": False, "error": "unknown id"}), 400
    raise_exception(eid, "대시보드 수동 발생")
    return jsonify({"ok": True})

@app.route("/api/reset", methods=["POST"])
def api_reset():
    global _triggered, _run_gen
    with _trigger_lock:
        _run_gen += 1
    with _state_lock:
        state["floor2"]["signal"] = False
        state["floor1"]["moving"] = False
        state["floor1"]["step"]   = 0
        state["rear"]["moving"]   = False
        state["rear"]["step"]     = 0
        for k in ("floor2", "floor1", "rear"):
            state[k]["done"] = False
        state["phase"]      = "idle"
        state["disaster"]   = None
        state["exceptions"] = {}
        state["latency"]    = None
    with _trigger_lock:
        _triggered = False
    _set_led(_bolt_2f, 0, 255, 0)
    _set_led(_bolt_1f, 0, 255, 0)
    _set_led(_bolt_r2, 0, 255, 0)
    log_event("🔄 수동 리셋")
    return jsonify({"ok": True})

# ── R2 카메라 모니터링 (주행은 R1 경로 추종, 카메라는 관찰·기록만) ─────────────
_follow = {
    "active": False,
    "direction": "stop",
    "fallen": False,
    "fallen_since": None,   # 넘어짐 확정 시각 (epoch 초)
    "aspect": None,         # 앱이 보낸 사람 박스 가로/세로 (화면 픽셀 기준)
    "tilt": None,           # 앱이 보낸 상체 기울기 (도, 0 = 서 있음)
    "last_ts": 0.0,
}
_follow_lock = threading.Lock()
_FOLLOW_INTERVAL = 0.12  # 초 — 앱 100ms 주기에 맞춰 단축

# 카메라 프리뷰 프레임 (최신 JPEG 1장 메모리 보관)
_latest_frame = None  # bytes or None
_frame_lock = threading.Lock()
_frame_ts: float = 0.0

# MJPEG 스트리밍 클라이언트 큐 목록
_mjpeg_clients: list = []
_mjpeg_clients_lock = threading.Lock()

def _jev_direction(jpeg_bytes: bytes) -> str:
    """api.typesafe.ai (인증 필요) 로 사람 위치 → 방향 판단"""
    if not JEV_KEY:
        return "stop"
    import requests as _req
    img_b64 = base64.b64encode(jpeg_bytes).decode()
    payload = {
        "model": "imajev-4b",
        "images": [f"data:image/jpeg;base64,{img_b64}"],
        "state": "로봇의 전면 카메라 프레임(세로 방향 사진). 사람 추종 로봇이 다음에 어느 방향으로 움직여야 하는지 판단해주세요.",
        "questions": {
            "direction": {
                "type": "choice",
                "instructions": "사진 속 사람을 따라가려는 로봇이 어느 방향으로 움직여야 하나요?",
                "criteria": {
                    "forward": "사람이 화면 중앙(가로 1/3~2/3)에 있음",
                    "left":    "사람이 화면 왼쪽 1/3에 있음",
                    "right":   "사람이 화면 오른쪽 1/3에 있음",
                    "stop":    "사람이 없거나 너무 가까워 화면 절반 이상 차지",
                },
            }
        },
    }
    try:
        resp = _req.post(
            "https://api.typesafe.ai/v1/systemone",
            headers={"Authorization": f"Bearer {JEV_KEY}"},
            json=payload,
            timeout=5,
        )
        if resp.status_code != 200:
            log_event(f"⚠ Jev Vision HTTP {resp.status_code}: {resp.text[:80]}")
            return "stop"
        result    = resp.json()
        ans       = result.get("answers", {}).get("direction", {})
        direction = ans.get("choice", "stop")
        if ans.get("abstained", False):
            direction = "stop"
        return direction if direction in ("forward", "left", "right", "stop") else "stop"
    except Exception as e:
        log_event(f"⚠ Jev Vision 오류: {e}")
        return "stop"


def _claude_direction(jpeg_bytes: bytes) -> str:
    """Claude Haiku Vision으로 사람 위치 → 방향 판단"""
    if not ANTHROPIC_KEY:
        return "stop"
    import anthropic, base64 as _b64
    img_b64 = _b64.b64encode(jpeg_bytes).decode()
    try:
        # OAuth 토큰(sk-ant-oat01-)은 auth_token, API 키(sk-ant-api03-)는 api_key
        if ANTHROPIC_KEY.startswith("sk-ant-oat"):
            client = anthropic.Anthropic(auth_token=ANTHROPIC_KEY)
        else:
            client = anthropic.Anthropic(api_key=ANTHROPIC_KEY)
        resp = client.messages.create(
            model="claude-haiku-5-5",
            max_tokens=10,
            messages=[{
                "role": "user",
                "content": [
                    {
                        "type": "image",
                        "source": {"type": "base64", "media_type": "image/jpeg", "data": img_b64},
                    },
                    {
                        "type": "text",
                        "text": (
                            "로봇이 사람을 따라갑니다. 사진 속 사람 위치를 보고 "
                            "다음 중 하나만 답하세요(다른 말 금지):\n"
                            "forward - 사람이 화면 중앙(좌우 1/3~2/3)\n"
                            "left    - 사람이 화면 왼쪽 1/3\n"
                            "right   - 사람이 화면 오른쪽 1/3\n"
                            "stop    - 사람 없음 또는 너무 가까움\n"
                            "답:"
                        ),
                    },
                ],
            }],
        )
        text = next((b.text for b in resp.content if hasattr(b, "text")), "stop")
        direction = text.strip().lower().split()[0]
        return direction if direction in ("forward", "left", "right", "stop") else "stop"
    except Exception as e:
        log_event(f"⚠ Claude Vision 오류: {e}")
        return "stop"


def _judge_and_move(jpeg: bytes):
    """Jev Vision 방향 판단 → Sphero 이동 (백그라운드 스레드 전용)"""
    if not _vision_busy.acquire(blocking=False):
        return   # 이전 판단이 아직 진행 중 → 스킵
    try:
        direction = _jev_direction(jpeg)
        with _follow_lock:
            _follow["direction"] = direction
        log_event(f"👁 R2 카메라(Jev) → {direction}")
    finally:
        _vision_busy.release()

@app.route("/api/follow/toggle", methods=["POST"])
def api_follow_toggle():
    with _follow_lock:
        _follow["active"] = not _follow["active"]
        active = _follow["active"]
        if not active:
            _follow.update(fallen=False, fallen_since=None, direction="stop")
    log_event(f"{'▶' if active else '⏹'} R2 카메라 감지 {'ON' if active else 'OFF'}")
    return jsonify({"ok": True, "active": active})

@app.route("/api/follow/frame", methods=["POST"])
def api_follow_frame():
    """Android 앱에서 JPEG 프레임 수신 → AI 판단 → Sphero 이동"""
    with _follow_lock:
        active = _follow["active"]

    if not active:
        return jsonify({"direction": "stop", "active": False})

    # 최소 간격 쓰로틀
    now = time.time()
    if now - _follow["last_ts"] < _FOLLOW_INTERVAL:
        return jsonify({"direction": _follow["direction"], "skipped": True})
    _follow["last_ts"] = now

    jpeg_bytes = flask_request.data
    if not jpeg_bytes:
        return jsonify({"ok": False, "error": "no image data"}), 400

    direction = _jev_direction(jpeg_bytes)

    with _follow_lock:
        _follow["direction"] = direction

    log_event(f"👁 R2 카메라 → {direction}")
    return jsonify({"direction": direction, "active": True})

@app.route("/api/follow/calibrate", methods=["POST"])
def api_follow_calibrate():
    """R2 방향 맞추기: R2가 지금 보는 방향을 heading 0°로 (R1과 같은 방향을 보게 둔 뒤 누른다)"""
    if not state["rear"]["connected"]:
        return jsonify({"ok": False, "error": "R2 미연결"}), 400
    try:
        _bolt_r2.drive_control.reset_heading()
        log_event("🧭 R2 heading 0° 리셋 (지금 정면 = 셀피 정면 기준)")
        return jsonify({"ok": True})
    except Exception as e:
        log_event(f"🧭 캘리브레이션 실패: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500

_AIM_LABEL = {"floor2": "R1", "rear": "R2", "floor1": "R3"}
_aim_heading = {"floor2": 0, "rear": 0, "floor1": 0}

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

@app.route("/api/aim", methods=["POST"])
def api_aim():
    """방향 보정 — 제자리 회전으로 로봇 정면을 R2 셀피 정면 방향에 맞춘 뒤 그 방향을 0°로 저장"""
    data = flask_request.get_json(silent=True) or {}
    key, action = data.get("robot"), data.get("action")
    if key not in _aim_heading:
        return jsonify({"ok": False, "error": "unknown robot"}), 400
    label = _AIM_LABEL[key]
    if action == "clear_trim":
        with _state_lock:
            state["aim_trim"][key] = 0
        _save_settings()
        log_event(f"🧭 {label} 자동 에임 보정값 지움")
        return jsonify({"ok": True, "heading": _aim_heading[key]})
    if not _active(key):
        return jsonify({"ok": False, "error": f"{label} 미연결"}), 400
    if state[key]["moving"] or state[key].get("returning"):
        return jsonify({"ok": False, "error": f"{label} 주행·복귀 중 — 끝난 뒤 보정"}), 409
    bolt = {"floor2": _bolt_2f, "rear": _bolt_r2, "floor1": _bolt_1f}[key]
    try:
        if action == "start":
            _aim_heading[key] = 0
            _set_aim_led(bolt)
            if bolt is not None:
                bolt.drive_control.roll_start(0, 0)
            log_event(f"🧭 {label} 방향 보정 시작 — 꼬리 LED(파랑)를 셀피 정면의 반대쪽으로")
        elif action == "rotate":
            _aim_heading[key] = ((_aim_heading[key] + int(data.get("delta", 0)) + 180) % 360) - 180
            if bolt is not None:
                bolt.drive_control.roll_start(_aim_heading[key] % 360, 0)
        elif action == "set":
            if bolt is not None and hasattr(bolt.drive_control, "reset_heading"):
                bolt.drive_control.reset_heading()
            offset = _aim_heading[key]
            _aim_heading[key] = 0
            _reset_origin(key, bolt)
            _set_led(bolt, 0, 255, 0)
            if state["aim"][key]["status"] == "done" and offset:
                # 자동 에임 결과에서 손으로 더 돌린 만큼 → 다음 자동 에임에 미리 더함
                trim = ((state["aim_trim"][key] + offset + 180) % 360) - 180
                with _state_lock:
                    state["aim_trim"][key] = trim
                _save_settings()
                _set_aim_status(key, "done", f"미세 조정 {offset:+d}° → 보정값 {trim:+d}° 저장")
                log_event(f"🧭 {label} 미세 조정 {offset:+d}° — 보정값 {trim:+d}° 기억 (다음 자동 에임에 적용)")
            else:
                log_event(f"🧭 {label} 방향 보정 완료 — 지금 정면 = 0° (셀피 정면 기준)")
        elif action == "clear_trim":
            with _state_lock:
                state["aim_trim"][key] = 0
            _save_settings()
            log_event(f"🧭 {label} 자동 에임 보정값 지움")
        else:
            return jsonify({"ok": False, "error": "unknown action"}), 400
    except Exception as e:
        log_event(f"🧭 {label} 방향 보정 실패: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500
    return jsonify({"ok": True, "heading": _aim_heading[key]})

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

@app.route("/api/speeds", methods=["POST"])
def api_speeds():
    """주행 속도 설정 (0~255) — 다음 주행부터 적용, 파일에 저장해 재시작 후에도 유지"""
    data = flask_request.get_json(silent=True) or {}
    new = dict(state["speeds"])
    for k in DEFAULT_SPEEDS:
        if k in data:
            try:
                v = int(data[k])
            except (TypeError, ValueError):
                return jsonify({"ok": False, "error": f"{_AIM_LABEL[k]} 속도가 숫자가 아님"}), 400
            if not 1 <= v <= 255:
                return jsonify({"ok": False, "error": f"{_AIM_LABEL[k]} 속도는 1~255"}), 400
            new[k] = v
    gap = state["follow_gap"]
    if "follow_gap" in data:
        try:
            gap = round(float(data["follow_gap"]), 2)
        except (TypeError, ValueError):
            return jsonify({"ok": False, "error": "간격이 숫자가 아님"}), 400
        if not GAP_MIN_M <= gap <= GAP_MAX_M:
            return jsonify({"ok": False, "error": f"간격은 {GAP_MIN_M*100:g}~{GAP_MAX_M*100:g}cm"}), 400
    dist_pct = state["r2_dist_pct"]
    if "r2_dist_pct" in data:
        try:
            dist_pct = round(float(data["r2_dist_pct"]))
        except (TypeError, ValueError):
            return jsonify({"ok": False, "error": "R2 거리 보정이 숫자가 아님"}), 400
        if not 50 <= dist_pct <= 200:
            return jsonify({"ok": False, "error": "R2 거리 보정은 50~200%"}), 400
    pause = state["turn_pause"]
    if "turn_pause" in data:
        try:
            pause = round(float(data["turn_pause"]), 1)
        except (TypeError, ValueError):
            return jsonify({"ok": False, "error": "꺾기 정지 시간이 숫자가 아님"}), 400
        if not 0 <= pause <= 10:
            return jsonify({"ok": False, "error": "꺾기 정지 시간은 0~10초"}), 400
    delay = state["r2_delay"]
    if "r2_delay" in data:
        try:
            delay = round(float(data["r2_delay"]), 1)
        except (TypeError, ValueError):
            return jsonify({"ok": False, "error": "R2 출발 지연이 숫자가 아님"}), 400
        if not 0 <= delay <= 30:
            return jsonify({"ok": False, "error": "R2 출발 지연은 0~30초"}), 400
    with _state_lock:
        state["speeds"] = new
        state["r2_delay"] = delay
        state["turn_pause"] = pause
        state["r2_dist_pct"] = dist_pct
        state["follow_gap"] = gap
    _save_settings()
    log_event(f"⚙ 주행 속도 — R1 {new['floor2']} · R2 {new['rear']} (출발 {delay:g}초 지연, 거리 {dist_pct:g}%) · R3 {new['floor1']} · 꺾기 전 {pause:g}초 정지 · 간격 {gap*100:g}cm (다음 주행부터)")
    return jsonify({"ok": True, "speeds": new, "r2_delay": delay, "turn_pause": pause, "r2_dist_pct": dist_pct,
                    "follow_gap": gap})

@app.route("/api/phone/heading", methods=["POST"])
def api_phone_heading():
    """앱이 0.5초마다 보내는 셀피 카메라 정면 방위"""
    data = flask_request.get_json(silent=True) or {}
    try:
        az = float(data["azimuth"]) % 360
        quality = max(0.0, min(1.0, float(data.get("quality", 0))))
    except (KeyError, TypeError, ValueError):
        return jsonify({"ok": False, "error": "azimuth 필요"}), 400
    acc = data.get("accuracy")
    with _state_lock:
        state["phone"] = {"azimuth": round(az, 1), "quality": round(quality, 2),
                          "accuracy": acc if isinstance(acc, int) else None, "ts": time.time()}
    return jsonify({"ok": True})

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
    bolt = {"floor2": _bolt_2f, "rear": _bolt_r2, "floor1": _bolt_1f}[key]
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
            bolt.drive_control.roll_start(target, 0)
            time.sleep(1.5)
            bolt.drive_control.roll_stop(target)
            time.sleep(0.5)
            if hasattr(bolt.drive_control, "reset_heading"):
                bolt.drive_control.reset_heading()
        _aim_heading[key] = 0
        _reset_origin(key, bolt)
        _set_led(bolt, 0, 255, 0)
        msg = f"완료 · 북 {north}° + 셀피 {azimuth:.0f}°" + (f" + 보정 {trim:+d}°" if trim else "")
        _set_aim_status(key, "done", msg)
        log_event(f"🧭 {label} 나침반 자동 에임 {msg} → 지금 정면 = 0°")
    except Exception as e:
        _set_aim_status(key, "error", f"실패: {e}")
        log_event(f"🧭 {label} 나침반 자동 에임 실패: {e}")

@app.route("/api/aim/auto", methods=["POST"])
def api_aim_auto():
    """연결된 로봇을 폰 셀피 정면 방위에 맞춰 동시에 자동 에임"""
    data = flask_request.get_json(silent=True) or {}
    with _state_lock:
        ph = dict(state["phone"])
    if ph["azimuth"] is None or time.time() - ph["ts"] > PHONE_HEADING_MAX_AGE:
        return jsonify({"ok": False, "error": "폰 방위 수신 없음 — 앱을 켜 두세요"}), 400
    if ph["quality"] < PHONE_MIN_QUALITY:
        return jsonify({"ok": False, "error": "폰이 너무 눕혀져 있음 — 세워서 셀피가 진행 방향을 보게"}), 400
    keys = [k for k in (data.get("robots") or list(_AIM_LABEL)) if k in _AIM_LABEL]
    started, skipped = [], []
    for k in keys:
        if not _active(k) or state[k]["moving"] or state["aim"][k]["status"] == "running":
            skipped.append(_AIM_LABEL[k])
            continue
        threading.Thread(target=_auto_aim, args=(k, ph["azimuth"]), daemon=True).start()
        started.append(_AIM_LABEL[k])
    if not started:
        return jsonify({"ok": False, "error": "에임할 로봇 없음 (미연결·주행 중)"}), 400
    log_event(f"🧭 나침반 자동 에임 시작: {', '.join(started)} (셀피 방위 {ph['azimuth']:.0f}°)")
    return jsonify({"ok": True, "started": started, "skipped": skipped, "azimuth": ph["azimuth"]})

@app.route("/api/follow/direction", methods=["POST"])
def api_follow_direction():
    """R2 스마트폰이 감지한 대피자 움직임 수신 → 기록·대시보드 표시 (주행에는 쓰지 않음)"""
    with _follow_lock:
        active = _follow["active"]

    if not active:
        return jsonify({"direction": "stop", "active": False})

    data = flask_request.get_json(silent=True) or {}
    direction = data.get("direction", "stop")
    if direction not in ("forward", "left", "right", "stop", "back"):
        direction = "stop"
    ratio = max(0.0, min(1.0, float(data.get("ratio", 0.5))))
    fallen = bool(data.get("fallen", False))

    def _num(key):
        try:
            v = data.get(key)
            return None if v is None else round(float(v), 2)
        except (TypeError, ValueError):
            return None
    aspect, tilt = _num("aspect"), _num("tilt")

    now = time.time()
    with _follow_lock:
        prev_dir, prev_fallen = _follow["direction"], _follow["fallen"]
        _follow["direction"] = direction
        _follow["fallen"] = fallen
        _follow["aspect"] = aspect
        _follow["tilt"] = tilt
        _follow["last_ts"] = now
        if fallen and not prev_fallen:
            _follow["fallen_since"] = now
        elif not fallen:
            _follow["fallen_since"] = None

    if fallen and not prev_fallen:
        metrics = f"w/h={aspect}" + (f", 기울기 {tilt:.0f}°" if tilt is not None else "")
        log_event(f"🚨 R2 카메라: 대피자 넘어짐 감지 (3초 이상 · {metrics})")
    elif prev_fallen and not fallen:
        log_event("✅ R2 카메라: 넘어짐 해제")
    # 방향 변화는 초당 여러 번이라 로그에 남기지 않음 (주행 로그가 밀려남) — 대시보드 카메라 카드에 실시간 표시
    return jsonify({"direction": direction, "active": True})

@app.route("/api/follow/preview", methods=["POST"])
def api_follow_preview():
    """Android 앱에서 JPEG 수신 → 대시보드 저장 + (JEV_KEY 있으면) 방향 판단"""
    global _latest_frame, _frame_ts
    jpeg = flask_request.data
    if not jpeg:
        return "", 204

    with _frame_lock:
        _latest_frame = jpeg
        _frame_ts = time.time()

    # MJPEG 클라이언트들에게 프레임 push
    with _mjpeg_clients_lock:
        for q in _mjpeg_clients:
            if q.full():
                try: q.get_nowait()
                except: pass
            try: q.put_nowait(jpeg)
            except: pass

    return "", 204

@app.route("/api/follow/snapshot")
def api_follow_snapshot():
    """최신 카메라 프레임을 JPEG로 반환 — 대시보드 폴링용"""
    with _frame_lock:
        frame = _latest_frame
        ts    = _frame_ts
    if not frame:
        return "", 204
    from flask import Response
    resp = Response(frame, mimetype="image/jpeg")
    resp.headers["Cache-Control"] = "no-store"
    resp.headers["X-Frame-Ts"] = str(ts)
    return resp

@app.route("/api/follow/stream")
def api_follow_stream():
    """MJPEG 스트리밍 endpoint — 브라우저 img src 직접 연결용"""
    from flask import Response, stream_with_context
    q = _queue.Queue(maxsize=2)
    with _mjpeg_clients_lock:
        _mjpeg_clients.append(q)
    def generate():
        try:
            # 연결 직후 최신 프레임 즉시 전송
            with _frame_lock:
                first = _latest_frame
            if first:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + first + b'\r\n')
            while True:
                try:
                    frame = q.get(timeout=5)
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                except _queue.Empty:
                    yield b'--frame\r\n\r\n'  # keepalive
        finally:
            with _mjpeg_clients_lock:
                try: _mjpeg_clients.remove(q)
                except: pass
    return Response(stream_with_context(generate()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@sock.route("/api/follow/ws")
def follow_ws(ws):
    """WebSocket 프리뷰 스트리밍 — 브라우저와 1:1 binary 연결"""
    q = _queue.Queue(maxsize=2)
    with _mjpeg_clients_lock:
        _mjpeg_clients.append(q)
    try:
        # 연결 직후 최신 프레임 즉시 전송
        with _frame_lock:
            first = _latest_frame
        if first:
            ws.send(first)
        while True:
            try:
                frame = q.get(timeout=10)
                ws.send(frame)
            except _queue.Empty:
                pass  # 타임아웃 무시, 다음 프레임 대기
    except Exception:
        pass
    finally:
        with _mjpeg_clients_lock:
            try: _mjpeg_clients.remove(q)
            except: pass

@app.route("/api/follow/status")
def api_follow_status():
    now = time.time()
    with _follow_lock:
        d = dict(_follow)
    d["fallen_secs"] = round(now - d["fallen_since"], 1) if d["fallen_since"] else 0
    d["stale_secs"]  = round(now - d["last_ts"], 1) if d["last_ts"] else None
    with _frame_lock:
        d["preview_ts"] = _frame_ts
        d["has_preview"] = _latest_frame is not None
    return jsonify(d)

@app.route("/api/disconnect", methods=["POST"])
def api_disconnect():
    _broker_stop_evt.set()   # 하트비트 루프 종료 → with 블록 탈출 → BLE 해제
    log_event("🔌 연결 해제 요청")
    return jsonify({"ok": True})

@app.route("/api/restart", methods=["POST"])
def api_restart():
    import threading, os, sys, subprocess, shlex
    def _do():
        import time; time.sleep(0.3)
        cmd = ' '.join(shlex.quote(a) for a in [sys.executable] + sys.argv)
        subprocess.Popen(['bash', '-c', f'sleep 1 && {cmd}'],
                         cwd=os.getcwd(), start_new_session=True)
        time.sleep(0.1)
        os._exit(0)
    threading.Thread(target=_do, daemon=True).start()
    return jsonify({"ok": True})

# ── 대시보드 HTML ──────────────────────────────────────────────────────────────
_DASHBOARD_HTML = r"""<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>BANDI — 재난 대피 안내 브로커</title>
<link rel="icon" type="image/png" href="/brand/favicon">
<link rel="apple-touch-icon" href="/brand/symbol">
<style>
:root {
  --bg:   #0f1117;
  --card: #1a1d27;
  --bdr:  #2c3144;
  --txt:  #e2e8f0;
  --dim:  #94a3b8;
  --grn:  #22c55e;
  --red:  #ef4444;
  --yel:  #f59e0b;
  --blu:  #3b82f6;
  --teal: #14b8a6;
}
* { box-sizing: border-box; margin: 0; padding: 0; }
body { background: var(--bg); color: var(--txt); font: 14px/1.6 'Pretendard', 'Noto Sans KR', system-ui, sans-serif; min-height: 100vh; }

header { padding: 14px 24px; border-bottom: 1px solid var(--bdr); display: flex; align-items: center; gap: 12px; }
header h1 { font-size: 18px; font-weight: 700; }
.brand-logo { height: 34px; width: auto; display: block; }
.brand-sub { font-size: 12px; color: var(--dim); border-left: 1px solid var(--bdr); padding-left: 12px; line-height: 1.35; }
.brand-sub b { color: var(--txt); font-weight: 600; }
@media (max-width: 600px) { .brand-sub { display: none; } }
.sim-badge { background: #78350f; color: #fde68a; font-size: 11px; padding: 2px 8px; border-radius: 4px; display: none; }

.grid { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; padding: 2px 20px 24px; }
@media (max-width: 700px) { .grid { grid-template-columns: 1fr; } }

.card { background: var(--card); border: 1px solid var(--bdr); border-radius: 12px; padding: 13px 15px; }
.card h2 { font-size: 11px; font-weight: 600; color: var(--ink3); text-transform: uppercase; letter-spacing: .09em; margin-bottom: 10px; }

.status-row { display: flex; align-items: center; gap: 10px; margin-bottom: 10px; }
.dot { width: 10px; height: 10px; border-radius: 50%; background: var(--red); flex-shrink: 0; }
.dot.on { background: var(--grn); box-shadow: 0 0 8px var(--grn); }
.dot.warn { background: var(--yel); box-shadow: 0 0 8px var(--yel); }

.big-label { font-size: 28px; font-weight: 700; line-height: 1; }
.sub-label  { font-size: 12px; color: var(--dim); margin-top: 4px; }

.bat-bar { height: 6px; background: var(--bdr); border-radius: 3px; margin-top: 10px; }
.bat-bar-inner { height: 100%; border-radius: 3px; background: var(--grn); transition: width .6s; }

.accel-row { display: grid; grid-template-columns: repeat(3,1fr); gap: 6px; margin-top: 10px; }
.accel-box { background: #0f1117; border-radius: 6px; padding: 6px 8px; text-align: center; }
.accel-box .axis { font-size: 11px; color: var(--dim); }
.accel-box .val  { font-size: 15px; font-weight: 600; }

.path-steps { margin-top: 10px; display: flex; gap: 6px; flex-wrap: wrap; }
.step-box { background: var(--bdr); border-radius: 6px; padding: 4px 10px; font-size: 12px; color: var(--dim); }
.step-box.active { background: var(--blu); color: #fff; font-weight: 700; }
.step-box.done   { background: var(--grn); color: #fff; }

.progress-bar { height: 8px; background: var(--bdr); border-radius: 4px; margin-top: 12px; }
.progress-inner { height: 100%; border-radius: 4px; background: var(--blu); transition: width .4s; }

/* Signal pill */
.signal-pill { display: inline-block; padding: 4px 14px; border-radius: 20px;
  font-size: 12px; font-weight: 700; background: var(--bdr); color: var(--dim); }
.signal-pill.active { background: #7f1d1d; color: var(--red);
  animation: pulse 1s infinite; }
@keyframes pulse { 0%,100% { opacity: 1; } 50% { opacity: .5; } }

/* Buttons */
.btn-row { display: flex; gap: 8px; margin-top: 12px; flex-wrap: wrap; }
.btn { padding: 6px 14px; border-radius: 7px; border: none; cursor: pointer;
  font-size: 13px; font-weight: 600; transition: opacity .15s; }
.btn:hover { opacity: .85; }
.btn-trigger { background: var(--red);   color: #fff; }
.btn-reset   { background: var(--bdr);   color: var(--txt); }

/* Log */
.log-wrap { grid-column: 1/-1; }
.log-list { list-style: none; max-height: 220px; overflow-y: auto;
  font-size: 12px; font-family: 'JetBrains Mono', monospace; }
.log-list li { padding: 4px 0; border-bottom: 1px solid var(--bdr); display: flex; gap: 10px; }
.log-ts { color: var(--dim); flex-shrink: 0; }

/* Count badge */
.count-badge { background: var(--red); color: #fff; font-size: 11px;
  font-weight: 700; padding: 2px 7px; border-radius: 10px; }

/* Connect form */
.connect-wrap { grid-column: 1/-1; }
.input-row { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 12px; margin-bottom: 12px; }
@media (max-width: 600px) { .input-row { grid-template-columns: 1fr; } }
.field label { display: block; font-size: 11px; color: var(--dim); margin-bottom: 5px; letter-spacing: .05em; text-transform: uppercase; }
.field input { width: 100%; background: #0f1117; border: 1px solid var(--bdr); border-radius: 8px;
  color: var(--txt); font: 15px/1 'JetBrains Mono', monospace; padding: 10px 12px;
  outline: none; transition: border-color .15s; }
.field input:focus { border-color: var(--blu); }
.field input.ok    { border-color: var(--grn); }
.field input.err   { border-color: var(--red); }
.btn-connect { background: var(--blu); color: #fff; }
.btn-connect:disabled { opacity: .4; cursor: not-allowed; }
.btn-disconnect { background: #3f1515; color: var(--red); border: 1px solid var(--red); }
.btn-disconnect:disabled { opacity: .4; cursor: not-allowed; }
.btn-restart { background: #1a1a2e; color: #aaa; border: 1px solid #444; }
.connect-hint { font-size: 12px; color: var(--dim); margin-top: 8px; }

/* Follow Mode + Log 2컬럼 */
.follow-log-row { grid-column: 1/-1; display: flex; gap: 16px; align-items: flex-start; }
.follow-log-row > .card { flex: 1 1 0; min-width: 0; }
@media (max-width: 900px) { .follow-log-row { flex-direction: column; } }
.follow-inner { display: grid; grid-template-columns: auto 1fr; gap: 20px; align-items: start; }
@media (max-width: 700px) { .follow-inner { grid-template-columns: 1fr; } }
.follow-preview-wrap { position: relative; width: 180px; height: 320px; background: #0a0d13;
  border: 1px solid var(--bdr); border-radius: 10px; overflow: hidden; flex-shrink: 0; }
.follow-preview { width: 100%; height: 100%; object-fit: cover; display: block; }
.follow-preview-placeholder { position: absolute; inset: 0; display: flex; flex-direction: column;
  align-items: center; justify-content: center; gap: 6px; color: var(--dim); font-size: 12px; }
.follow-preview-placeholder svg { opacity: .3; }
.follow-dir { font-size: 36px; font-weight: 800; line-height: 1; margin-top: 6px; }
.follow-info { font-size: 12px; color: var(--dim); margin-top: 4px; }
.btn-follow-on  { background: #166534; color: #86efac; border: 1px solid #22c55e; }
.btn-follow-off { background: #7f1d1d; color: #fca5a5; border: 1px solid #ef4444; }
.preview-age { position: absolute; bottom: 4px; right: 6px; font-size: 10px;
  color: rgba(255,255,255,.6); background: rgba(0,0,0,.5); padding: 1px 5px; border-radius: 3px; }
#floorplanWrap { background: #fff; }
.fp-stage { position: relative; background: #fff; }
.fp-stage img { width: 100%; display: block; }
.robot { position: absolute; transform: translate(-50%, -50%); z-index: 5; pointer-events: none;
  transition: left 1.4s linear, top 1.4s linear; }
.robot .rb-dot { width: 22px; height: 22px; border-radius: 50%; border: 3px solid #fff;
  background: #9ca3af; box-shadow: 0 1px 4px rgba(0,0,0,.45); display: flex; align-items: center;
  justify-content: center; font-size: 9px; font-weight: 800; color: #fff; }
.robot .rb-tag { position: absolute; left: 50%; bottom: 28px; transform: translateX(-50%);
  white-space: nowrap; font-size: 11px; font-weight: 700; padding: 2px 7px; border-radius: 10px;
  background: rgba(17,24,39,.85); color: #fff; }
.robot.rb-rear .rb-tag { bottom: auto; top: 28px; }
.robot.idle   .rb-dot { background: #22c55e; }
.robot.moving .rb-dot { background: #f97316; animation: rbPulse 1s infinite; }
.robot.done   .rb-dot { background: #2563eb; }
@keyframes rbPulse { 0% { box-shadow: 0 0 0 0 rgba(249,115,22,.7); }
  100% { box-shadow: 0 0 0 14px rgba(249,115,22,0); } }
.fp-legend { display: flex; gap: 14px; flex-wrap: wrap; padding: 8px 20px; font-size: 12px;
  color: var(--dim); border-top: 1px solid var(--bdr); }
.fp-legend i { display: inline-block; width: 10px; height: 10px; border-radius: 50%;
  margin-right: 5px; vertical-align: -1px; }

/* ── 재난 대응 (시연 페이지 UI 기준) ── */
:root { --panel2: #1c232c; --wire: #3b4654; --ink2: #9aa7b4; --ink3: #6b7785;
  --ok: #34d399; --bad: #f87171; --fire: #ff6b35; --quake: #a78bfa; --black: #fbbf24; --blue2: #5b9bd5; }
.ops { display: grid; grid-template-columns: 270px minmax(400px, 1fr) 300px; gap: 14px;
  padding: 14px 20px 0; align-items: start; }
@media (max-width: 1100px) { .ops { grid-template-columns: 1fr; } }
.ops .card { margin-bottom: 12px; }
.state-pill { margin-left: auto; font-size: 11px; font-weight: 700; padding: 3px 10px; border-radius: 20px;
  background: var(--panel2); border: 1px solid var(--bdr); color: var(--ink2); letter-spacing: .06em; transition: .3s; }
.state-pill.armed    { border-color: var(--black); color: var(--black); background: rgba(251,191,36,.12); }
.state-pill.guiding  { border-color: var(--ok);    color: var(--ok);    background: rgba(52,211,153,.12); }
.state-pill.handoff  { border-color: var(--blue2); color: var(--blue2); background: rgba(91,155,213,.12); }
.state-pill.complete { border-color: var(--ok);    color: var(--ok);    background: rgba(52,211,153,.18); }
.hdr-btn { appearance: none; border: 1px solid var(--bdr); background: var(--panel2); color: var(--ink2);
  border-radius: 7px; padding: 3px 10px; font: inherit; font-size: 11px; font-weight: 600; cursor: pointer; transition: .15s; }
.hdr-btn:hover { border-color: var(--bad); color: var(--bad); }

.timeline { display: flex; align-items: center; margin: 4px 0 8px; }
.tnode { display: flex; flex-direction: column; align-items: center; gap: 3px; flex: 1; }
.tnode .tc { width: 36px; height: 36px; border-radius: 50%; background: var(--panel2); border: 2px solid var(--wire);
  display: grid; place-items: center; font-size: 7.5px; font-weight: 700; color: var(--ink3); letter-spacing: .02em; transition: .3s; }
.tnode .tl { font-size: 9.5px; color: var(--ink3); font-weight: 600; text-align: center; line-height: 1.2; }
.tnode.active .tc { border-color: var(--ok); background: rgba(52,211,153,.15); color: var(--ok); }
.tnode.done .tc   { border-color: var(--blue2); background: rgba(91,155,213,.12); color: var(--blue2); }
.tarrow { color: var(--wire); font-size: 11px; margin: 0 1px; padding-bottom: 16px; transition: .3s; flex: none; }
.tarrow.on { color: var(--ok); }
.meta-row { display: flex; gap: 14px; flex-wrap: wrap; font-size: 11px; color: var(--ink3); }
.meta-row b { color: var(--txt); font-weight: 600; }
.note { margin-top: 8px; padding: 8px 10px; border-radius: 7px; background: var(--panel2);
  border-left: 2px solid var(--bdr); font-size: 11.5px; color: var(--ink2); line-height: 1.5; }
.note b { color: var(--txt); }

.dz-btns { display: grid; grid-template-columns: 1fr 1fr; gap: 7px; }
.dz { appearance: none; border: 1px solid var(--bdr); background: var(--panel2); color: var(--txt);
  border-radius: 9px; padding: 10px 7px; font: inherit; font-weight: 600; cursor: pointer; text-align: center;
  transition: .15s; display: flex; flex-direction: column; align-items: center; gap: 3px; }
.dz .ico { font-size: 18px; line-height: 1; }
.dz small { font-weight: 400; color: var(--ink3); font-size: 10.5px; }
.dz:hover { border-color: var(--ink3); transform: translateY(-1px); }
.dz.fire.on  { border-color: var(--fire);  background: rgba(255,107,53,.14);  color: var(--fire); }
.dz.quake.on { border-color: var(--quake); background: rgba(167,139,250,.14); color: var(--quake); }
.dz.black.on { border-color: var(--black); background: rgba(251,191,36,.14);  color: var(--black); }
.dz-clear { grid-column: 1/-1; background: transparent; font-size: 12.5px; }
.dz-clear:hover { border-color: var(--ok); color: var(--ok); }
.dz-test { border-style: dashed; }
.dz-test:hover { border-color: var(--blue2); color: var(--blue2); }

.seg { display: grid; grid-template-columns: 1fr 1fr; gap: 5px; }
.seg button { appearance: none; border: 1px solid var(--bdr); background: var(--panel2); color: var(--ink2);
  border-radius: 7px; padding: 7px 4px; font: inherit; font-size: 11.5px; cursor: pointer; transition: .15s;
  display: flex; flex-direction: column; align-items: center; gap: 1px; }
.seg button .ico { font-size: 15px; }
.seg button .spd { font-size: 9.5px; color: var(--ink3); }
.seg button.on { border-color: var(--ok); color: var(--txt); background: rgba(52,211,153,.12); }

.bots { display: grid; gap: 6px; }
.bot { display: flex; align-items: center; gap: 9px; padding: 8px 10px; background: var(--panel2);
  border: 1px solid var(--bdr); border-radius: 8px; transition: .2s; }
.rball { width: 32px; height: 32px; border-radius: 7px; background: #111; flex: none; display: grid;
  place-items: center; font-size: 12px; font-weight: 800; color: #333; transition: .3s; }
.bot .n { font-weight: 600; font-size: 12px; }
.bot .n span { font-weight: 400; color: var(--ink3); font-size: 10.5px; }
.bot .s { font-size: 10.5px; color: var(--ink3); }
.bot .st { margin-left: auto; font-size: 10px; padding: 2px 7px; border-radius: 4px; background: #1a2030;
  color: var(--ink3); font-weight: 700; letter-spacing: .04em; min-width: 60px; text-align: center; flex: none; }
.bot.armed .st    { background: rgba(251,191,36,.14); color: var(--black); }
.bot.guiding .st  { background: rgba(52,211,153,.15); color: var(--ok); }
.bot.handoff .st  { background: rgba(91,155,213,.14); color: var(--blue2); }
.bot.complete .st { background: rgba(52,211,153,.2);  color: var(--ok); }

.cnode { display: flex; gap: 10px; align-items: flex-start; padding: 8px 9px; border-radius: 8px;
  border: 1px solid transparent; transition: .2s; }
.cnode.active { background: var(--panel2); border-color: var(--bdr); }
.cnode .cdot { width: 9px; height: 9px; border-radius: 50%; background: var(--wire); margin-top: 5px; flex: none; transition: .2s; }
.cnode.active .cdot { background: var(--ok); box-shadow: 0 0 0 4px rgba(52,211,153,.16); }
.cnode.cut .cdot { background: var(--bad); }
.cnode .t { font-weight: 600; font-size: 12.5px; }
.cnode .d { font-size: 11px; color: var(--ink3); margin-top: 1px; }
.cnode.cut .t { color: var(--bad); text-decoration: line-through; text-decoration-thickness: 1px; }
.clink { width: 1px; height: 10px; background: var(--wire); margin-left: 13px; }
.clink.on  { background: var(--ok); }
.clink.cut { background: repeating-linear-gradient(180deg, var(--bad) 0 3px, transparent 3px 6px); }

.excepts { display: grid; gap: 4px; }
.ex { appearance: none; width: 100%; display: flex; align-items: center; gap: 8px; padding: 5px 8px; border-radius: 6px;
  background: var(--panel2); border: 1px solid var(--bdr); font: inherit; font-size: 11px; color: var(--txt);
  text-align: left; cursor: pointer; transition: .3s; }
.ex:hover { border-color: var(--ink3); }
.ex .eid { width: 22px; height: 22px; border-radius: 50%; background: #1a2030; border: 1px solid var(--wire);
  display: grid; place-items: center; font-size: 9px; font-weight: 700; color: var(--ink3); flex: none; }
.ex .etxt { color: var(--ink3); flex: 1; }
.ex .est { font-size: 10px; font-weight: 700; flex: none; }
.ex.triggered { border-color: var(--bad); background: rgba(248,113,113,.1); }
.ex.triggered .eid { border-color: var(--bad); color: var(--bad); background: rgba(248,113,113,.15); }
.ex.triggered .etxt { color: var(--txt); }
.ex.triggered .est { color: var(--bad); }
.ex.resolved { border-color: var(--ok); background: rgba(52,211,153,.08); }
.ex.resolved .eid { border-color: var(--ok); color: var(--ok); }
.ex.resolved .est { color: var(--ok); }

/* 방향 보정 */
.aim-rows { display: grid; gap: 8px; }
.aim-row { display: flex; align-items: center; gap: 6px; flex-wrap: wrap; }
.aim-row b { width: 26px; }
.aim-row .aim-desc { font-size: 12px; color: var(--dim); flex: 1 1 220px; }
.aim-row .btn { padding: 5px 10px; font-size: 12px; }
.aim-auto { display: flex; align-items: center; justify-content: space-between; gap: 10px; flex-wrap: wrap;
  padding: 8px 10px; margin-bottom: 10px; border: 1px solid var(--bdr); border-radius: 8px; background: var(--bg); }
.aim-auto b { font: 15px 'JetBrains Mono', monospace; }
.aim-st { font-weight: 700; margin-left: 6px; }
.aim-st.running { color: var(--yel); } .aim-st.done { color: var(--grn); } .aim-st.error { color: var(--red); }
.aim-trim { min-width: 64px; text-align: center; font: 12px 'JetBrains Mono', monospace; color: var(--yel); }
.aim-deg { min-width: 44px; text-align: center; font: 13px 'JetBrains Mono', monospace; color: var(--txt); }

/* 넘어짐 경보 */
.fall-alert { display: none; position: sticky; top: 0; z-index: 50; align-items: center; gap: 12px;
  padding: 10px 24px; background: #b91c1c; color: #fff; font-size: 14px; cursor: pointer;
  box-shadow: 0 4px 14px rgba(185,28,28,.45); animation: fallPulse 1s ease-in-out infinite; }
.fall-alert.on { display: flex; }
.fall-alert .fa-ico { font-size: 20px; line-height: 1; }
.fall-alert .fa-meta { margin-left: auto; font-size: 12px; opacity: .85; font-family: 'JetBrains Mono', monospace; }
@keyframes fallPulse { 0%, 100% { background: #b91c1c; } 50% { background: #dc2626; } }
.fall-badge { display: none; position: absolute; top: 8px; left: 8px; z-index: 2; background: #dc2626; color: #fff;
  font-size: 12px; font-weight: 800; padding: 3px 9px; border-radius: 6px; animation: pulse 1s infinite; }
.follow-preview-wrap.fallen { border: 3px solid #dc2626; box-shadow: 0 0 0 4px rgba(220,38,38,.25); }
.follow-preview-wrap.fallen .fall-badge { display: block; }
.bot.fall { border-color: var(--bad); background: rgba(248,113,113,.1); }
.bot.fall .st { background: rgba(248,113,113,.2); color: var(--bad); }
.robot.fallen .rb-dot { background: #dc2626; animation: rbPulseRed 1s infinite; }
.robot.fallen .rb-tag { background: #b91c1c; }
@keyframes rbPulseRed { 0% { box-shadow: 0 0 0 0 rgba(220,38,38,.75); } 100% { box-shadow: 0 0 0 16px rgba(220,38,38,0); } }

footer { padding: 0 20px 24px; color: var(--ink3); font-size: 11px; line-height: 1.75; }
footer b { color: var(--ink2); }
kbd { background: var(--panel2); border: 1px solid var(--bdr); border-radius: 4px; padding: 1px 5px; font-size: 10.5px; }
</style>
</head>
<body>

<header>
  <img class="brand-logo" src="/brand/logo" alt="BANDI">
  <span class="brand-sub"><b>재난 대피 안내 로봇 · 중계기 대시보드</b><br>4조 잇다(ITDA) · 팔달관</span>
  <span class="sim-badge" id="simBadge">SIM</span>
  <span class="state-pill" id="statePill" title="시스템 상태">IDLE</span>
  <button class="hdr-btn" onclick="restartBroker()">↺ 브로커 재시작</button>
</header>

<div class="fall-alert" id="fallAlert" role="alert" onclick="document.getElementById('followWrapCard').scrollIntoView({behavior:'smooth'})">
  <span class="fa-ico">⚠</span>
  <span><b>대피자 넘어짐 감지</b> — R2 카메라 · <span id="fallSecs">0</span>초째 · 즉시 확인 필요</span>
  <span class="fa-meta" id="fallMeta"></span>
</div>

<div class="ops">
  <div>
    <!-- 재난 트리거 -->
    <div class="card">
      <h2>재난 트리거</h2>
      <div class="dz-btns">
        <button class="dz fire"  id="dzfire"  onclick="sendDisaster('fire')"><span class="ico">🔥</span>화재<small>서버 신호</small></button>
        <button class="dz quake" id="dzquake" onclick="sendDisaster('quake')"><span class="ico">🌍</span>지진<small>가속도계 감지</small></button>
        <button class="dz black" id="dzblack" onclick="sendDisaster('black')"><span class="ico">💡</span>정전<small>조도 센서 감지</small></button>
        <button class="dz dz-clear" onclick="sendDisaster('clear')">상황 해제 · 평시 복귀</button>
        <button class="dz dz-clear dz-test" onclick="testRoute()">🧪 테스트 경로 — ㄱ자 (직진 1m → 좌회전 1.5m)</button>
      </div>
      <div class="note" id="triggerNote"></div>
    </div>
    <!-- 안내 대상 모드 -->
    <div class="card">
      <h2>안내 대상 모드</h2>
      <div class="seg" id="seg"></div>
      <div class="note" id="targetNote"></div>
    </div>
    <!-- 로봇 상태 -->
    <div class="card">
      <h2>로봇 상태 R1–R4</h2>
      <div class="bots" id="bots"></div>
    </div>
  </div>
  <div>
    <!-- 대피 흐름 타임라인 -->
    <div class="card">
      <h2>팔달관 대피 흐름 — 2층 복도 → 계단 → 1층 로비 → 비상구</h2>
      <div class="timeline">
        <div class="tnode" id="tn0"><div class="tc">IDLE</div><div class="tl">대기</div></div>
        <div class="tarrow" id="ta0">›</div>
        <div class="tnode" id="tn1"><div class="tc">ARMED</div><div class="tl">감지</div></div>
        <div class="tarrow" id="ta1">›</div>
        <div class="tnode" id="tn2"><div class="tc">GUIDING</div><div class="tl">유도</div></div>
        <div class="tarrow" id="ta2">›</div>
        <div class="tnode" id="tn3"><div class="tc">HANDOFF</div><div class="tl">인수인계</div></div>
        <div class="tarrow" id="ta3">›</div>
        <div class="tnode" id="tn4"><div class="tc">COMPLETE</div><div class="tl">완료</div></div>
      </div>
      <div class="meta-row">
        <span>재난 <b id="metaDisaster">없음</b></span>
        <span>안내 대상 <b id="metaTarget">—</b></span>
        <span>트리거→선두 출발 <b id="metaLatency">—</b> (목표 ≤ 2.0s)</span>
      </div>
    </div>
    <!-- 팔달관 평면도 -->
    <div class="card" style="padding:0; overflow:hidden;">
      <div style="display:flex; align-items:center; justify-content:space-between; padding:12px 20px; border-bottom:1px solid var(--bdr); flex-wrap:wrap; gap:8px;">
        <h2 style="margin:0;">🗺 팔달관 대피 경로 도면</h2>
        <div style="display:flex; gap:6px; align-items:center; flex-wrap:wrap;">
          <button class="btn" id="fpTab2f" style="padding:4px 10px; font-size:11px; background:var(--blu); color:#fff;"
            onclick="floorplanTab('2f', true)">2층 · R1 R2</button>
          <button class="btn" id="fpTab1f" style="padding:4px 10px; font-size:11px; background:var(--bdr); color:var(--txt);"
            onclick="floorplanTab('1f', true)">1층 · R3</button>
          <button class="btn" style="padding:4px 10px; font-size:11px; background:var(--bdr); color:var(--txt);"
            onclick="window.open('/floorplan','_blank')">↗ PDF</button>
          <button class="btn" style="padding:4px 10px; font-size:11px; background:var(--bdr); color:var(--txt);"
            id="btnFloorplanToggle" onclick="toggleFloorplan()">▲ 접기</button>
        </div>
      </div>
      <div id="floorplanWrap">
        <div class="fp-stage" id="fpStage2f">
          <img src="/floorplan-img/2f" alt="팔달관 2층 도면">
          <div class="robot" id="robotR1"><div class="rb-dot">R1</div><div class="rb-tag">R1 선두</div></div>
          <div class="robot rb-rear" id="robotR2"><div class="rb-dot">R2</div><div class="rb-tag">R2 후방</div></div>
        </div>
        <div class="fp-stage" id="fpStage1f" style="display:none;">
          <img src="/floorplan-img/1f" alt="팔달관 1층 도면">
          <div class="robot" id="robotR3"><div class="rb-dot">R3</div><div class="rb-tag">R3 1층</div></div>
        </div>
        <div class="fp-legend">
          <span><i style="background:#9ca3af"></i>미연결</span>
          <span><i style="background:#22c55e"></i>출발점 대기</span>
          <span><i style="background:#f97316"></i>경로 주행 중</span>
          <span><i style="background:#2563eb"></i>도착 (핸드오프·비상구)</span>
          <span>위치는 경로 단계 기준 추정값</span>
        </div>
      </div>
    </div>
  </div>
  <div>
    <!-- 신호 전달 경로 -->
    <div class="card">
      <h2>신호 전달 경로</h2>
      <div class="cnode" id="cn0"><span class="cdot"></span><div><div class="t">재난 트리거</div><div class="d">화재: 서버 신호 / 지진·정전: 로봇 센서</div></div></div>
      <div class="clink" id="cl0"></div>
      <div class="cnode" id="cn1"><span class="cdot"></span><div><div class="t">중계기 (이 브로커)</div><div class="d">상태 수집 · 판단 · 명령 · 대시보드</div></div></div>
      <div class="clink" id="cl1"></div>
      <div class="cnode" id="cn2"><span class="cdot"></span><div><div class="t">BLE 게이트웨이</div><div class="d">랩탑 → Sphero BOLT</div></div></div>
      <div class="clink" id="cl2"></div>
      <div class="cnode" id="cn3"><span class="cdot"></span><div><div class="t">R1 · R2 · R3</div><div class="d">LED 화살표 · 대피 유도</div></div></div>
      <div class="note" id="pathNote"></div>
    </div>
    <!-- 예외 처리 -->
    <div class="card">
      <h2>예외 처리 E1–E5</h2>
      <div class="excepts" id="excepts"></div>
      <div class="note">E4·E5는 주행 중 자동 표시. 항목을 누르면 시연용으로 수동 발생 (표시·로그만, 로봇 동작은 바뀌지 않음).</div>
    </div>
  </div>
</div>

<div class="grid">

  <!-- 2F Sphero 카드 -->
  <div class="card">
    <h2>2층 Sphero (R1 선두)</h2>
    <div class="status-row">
      <div class="dot" id="dot2f"></div>
      <span id="conn2f" style="font-weight:600;">연결 중…</span>
      <span class="signal-pill" id="signalPill">대기</span>
    </div>
    <div class="bat-bar"><div class="bat-bar-inner" id="bat2f" style="width:0%"></div></div>
    <div class="sub-label" id="bat2fTxt">배터리 —</div>
    <div class="accel-row">
      <div class="accel-box"><div class="axis">X</div><div class="val" id="ax">0.00</div></div>
      <div class="accel-box"><div class="axis">Y</div><div class="val" id="ay">0.00</div></div>
      <div class="accel-box"><div class="axis">Z</div><div class="val" id="az">0.00</div></div>
    </div>
    <div class="sub-label" style="margin-top:6px;">가속도계 (G) — 흔들기 임계: """ + str(SHAKE_THRESHOLD) + r"""G</div>

    <div class="path-steps" id="pathSteps2f"></div>
    <div class="progress-bar"><div class="progress-inner" id="pathProgress2f" style="width:0%"></div></div>
    <div class="sub-label" id="pathLabel2f" style="margin-top:6px;">대기 중</div>

    <div class="btn-row">
      <button class="btn btn-trigger" onclick="triggerSignal()">🚨 신호 발생 (수동)</button>
    </div>
  </div>

  <!-- 1F Sphero 카드 -->
  <div class="card">
    <h2>1층 Sphero (R3 대기)</h2>
    <div class="status-row">
      <div class="dot" id="dot1f"></div>
      <span id="conn1f" style="font-weight:600;">연결 중…</span>
      <span id="movingBadge" style="font-size:12px; color:var(--dim);"></span>
    </div>
    <div class="bat-bar"><div class="bat-bar-inner" id="bat1f" style="width:0%"></div></div>
    <div class="sub-label" id="bat1fTxt">배터리 —</div>

    <div class="path-steps" id="pathSteps"></div>
    <div class="progress-bar"><div class="progress-inner" id="pathProgress" style="width:0%"></div></div>
    <div class="sub-label" id="pathLabel" style="margin-top:6px;">대기 중</div>

    <div class="btn-row">
      <button class="btn btn-reset" onclick="resetBroker()">🔄 리셋</button>
    </div>
  </div>

  <!-- Sphero 연결 설정 -->
  <div class="card connect-wrap">
    <h2>Sphero 연결 설정</h2>
    <div class="input-row">
      <div class="field">
        <label>2층 Sphero 이름 (R1 선두)</label>
        <input id="inp2f" type="text" placeholder="SB-XXXX" autocomplete="off" spellcheck="false">
      </div>
      <div class="field">
        <label>1층 Sphero 이름 (R3 대기)</label>
        <input id="inp1f" type="text" placeholder="SB-YYYY" autocomplete="off" spellcheck="false">
      </div>
      <div class="field">
        <label>2층 후방 Sphero 이름 (R2 트롤리)</label>
        <input id="inpR2" type="text" placeholder="SB-ZZZZ" autocomplete="off" spellcheck="false">
      </div>
    </div>
    <div class="btn-row">
      <button class="btn btn-connect" id="btnConnect" onclick="connectSphero()">🔌 연결 시작</button>
      <button class="btn btn-disconnect" id="btnDisconnect" onclick="disconnectSphero()">⏏ 연결 해제</button>
    </div>
    <p class="connect-hint" id="connectHint">Sphero Edu 앱에서 각 볼트의 이름을 확인 후 입력하세요.</p>
  </div>

  <!-- 주행 속도 -->
  <div class="card connect-wrap">
    <h2>주행 속도 (1~255) — 다음 주행부터 적용 · 재시작해도 유지</h2>
    <div class="input-row">
      <div class="field"><label>R1 앞 (2층 선두)</label><input id="spd_floor2" type="number" min="1" max="255"></div>
      <div class="field"><label>R2 뒤 (2층 후방 · 폰 탑재)</label><input id="spd_rear" type="number" min="1" max="255"></div>
      <div class="field"><label>R3 (1층)</label><input id="spd_floor1" type="number" min="1" max="255"></div>
      <div class="field"><label>R2 출발 지연 (초)</label><input id="spd_r2delay" type="number" min="0" max="30" step="0.5"></div>
      <div class="field"><label>꺾기 전 정지 (초)</label><input id="spd_turnpause" type="number" min="0" max="10" step="0.5"></div>
      <div class="field"><label>R2 거리 보정 (%) — 덜 가면 올림</label><input id="spd_r2dist" type="number" min="50" max="200" step="1"></div>
      <div class="field"><label>전방–후방 간격 (cm) — R1이 R2보다 앞에서 출발</label><input id="spd_gap" type="number" min="10" max="1400" step="10"></div>
    </div>
    <div class="btn-row" style="margin-top:0;">
      <button class="btn btn-connect" onclick="saveSpeeds()">💾 속도 저장</button>
      <button class="btn btn-reset" onclick="resetSpeeds()">기본값 (100 · 150 · 100 · 0초 · 1초 · 100% · 400cm)</button>
    </div>
    <p class="connect-hint" id="speedHint">각 로봇은 자기 속도로 구간 시간을 계산해 정해진 거리만큼만 간다. R2가 R1보다 빠르면 간격이 줄고 느리면 벌어진다 — 간격 유지는 실제 속도를 비슷하게.</p>
  </div>

  <!-- 방향 보정 -->
  <div class="card connect-wrap">
    <h2>방향 보정 — 기준: R2 스마트폰 셀피 정면 = 0°</h2>
    <div class="aim-auto">
      <span>📱 폰 셀피 정면 방위 <b id="phoneAz">—</b> <span id="phoneAzNote" class="aim-desc"></span></span>
      <button class="btn btn-connect" onclick="autoAim()">🧭 나침반 자동 에임 (연결된 로봇 전부)</button>
    </div>
    <div class="aim-rows">
""" + "".join(
    f'''      <div class="aim-row">
        <b>{lbl}</b><span class="aim-desc">{desc} <span class="aim-st" id="aimSt_{key}"></span></span>
        <button class="btn btn-reset" onclick="aim(\'{key}\',\'start\')">보정 시작</button>
        <button class="btn btn-reset" onclick="aim(\'{key}\',\'rotate\',-15)">⟲ 15°</button>
        <button class="btn btn-reset" onclick="aim(\'{key}\',\'rotate\',-5)">⟲ 5°</button>
        <button class="btn btn-reset" onclick="aim(\'{key}\',\'rotate\',-1)">⟲ 1°</button>
        <span class="aim-deg" id="aimDeg_{key}">—</span>
        <button class="btn btn-reset" onclick="aim(\'{key}\',\'rotate\',1)">⟳ 1°</button>
        <button class="btn btn-reset" onclick="aim(\'{key}\',\'rotate\',5)">⟳ 5°</button>
        <button class="btn btn-reset" onclick="aim(\'{key}\',\'rotate\',15)">⟳ 15°</button>
        <button class="btn btn-connect" onclick="aim(\'{key}\',\'set\')">✔ 0°로 저장</button>
        <span class="aim-trim" id="aimTrim_{key}" title="자동 에임 때 더하는 보정값">보정 0°</span>
        <button class="btn btn-reset" onclick="aim(\'{key}\',\'clear_trim\')" title="보정값 지우기">✕</button>
      </div>
''' for key, lbl, desc in [
        ("rear",   "R2", "경로 출발점 · 셀피 카메라가 진행 방향(첫 구간)을 보게"),
        ("floor2", "R1", "R2에서 경로 따라 간격(기본 400cm)만큼 앞 · 셀피 정면과 같은 방향"),
        ("floor1", "R3", "1층 출발점 · 1층 첫 구간(로비 쪽) 방향"),
    ]) + r"""    </div>
    <p class="connect-hint" id="aimHint">자동 에임 뒤 살짝 틀어졌으면: 그 로봇만 보정 시작 → ⟲/⟳ 1°·5°로 맞추고 0°로 저장 (나침반 재측정 없음). 수동: 보정 시작 → 꼬리 LED(파랑)가 켜짐. 화살표로 돌려 <b>꼬리 LED가 셀피 정면의 정반대</b>를 향하게(= 로봇 정면이 셀피 정면) 맞춘 뒤 0°로 저장. 경로의 모든 회전은 이 방향 기준.</p>
  </div>

  <!-- Follow Mode 카드 -->
  <div class="follow-log-row">
  <div class="card follow-wrap" id="followWrapCard">
    <h2>R2 후방 (R1 경로 추종 · 카메라 감지)</h2>
    <div class="conn-row" style="display:flex; align-items:center; gap:8px; margin-bottom:6px;">
      <div class="dot" id="dotR2"></div>
      <span id="connR2" style="font-weight:600;">연결 안됨</span>
      <span class="sub-label" id="batR2Txt">배터리 —</span>
    </div>
    <div class="sub-label" style="margin-bottom:8px;">
      앱 연결 주소
      <code id="appUrl" title="눌러서 복사" onclick="copyAppUrl()"
        style="cursor:pointer; color:var(--txt); background:var(--bg); border:1px solid var(--bdr);
               border-radius:5px; padding:1px 7px; font:12px 'JetBrains Mono', monospace;">—</code>
      <span id="appUrlCopied" style="color:var(--grn); display:none;">복사됨</span>
    </div>
    <div class="path-steps" id="pathStepsR2"></div>
    <div class="sub-label" id="pathLabelR2" style="margin:6px 0 14px;">대기 중</div>
    <div class="follow-inner">
      <div class="follow-preview-wrap" id="followPreviewWrap">
        <span class="fall-badge">⚠ 넘어짐</span>
        <img class="follow-preview" id="followPreview" alt="카메라 프리뷰">
        <div class="follow-preview-placeholder" id="followPlaceholder">
          <svg width="40" height="40" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5">
            <path d="M15 10l4.553-2.069A1 1 0 0121 8.869v6.262a1 1 0 01-1.447.894L15 14M3 8a2 2 0 012-2h10a2 2 0 012 2v8a2 2 0 01-2 2H5a2 2 0 01-2-2V8z"/>
          </svg>
          <span>앱 카메라 대기</span>
        </div>
        <span class="preview-age" id="previewAge" style="display:none;"></span>
      </div>
      <div>
        <div class="follow-dir" id="followDir">■</div>
        <div class="follow-info" id="followInfo">대기 중</div>
        <div class="follow-info" id="followMetrics" title="넘어짐 판정: w/h > 1.0 또는 기울기 > 60° 가 3초 지속">w/h — · 기울기 —</div>
        <div class="btn-row" style="margin-top:14px;">
          <button class="btn btn-follow-on" id="btnFollowToggle" onclick="toggleFollow()">▶ 감지 ON</button>
        </div>
        <div class="sub-label" style="margin-top:10px;">카메라 감지는 기록·표시만 — 주행은 R1 경로를 따라감</div>
      </div>
    </div>
  </div>

  <!-- 이벤트 로그 -->
  <div class="card log-wrap">
    <h2>이벤트 로그 &nbsp;<span class="count-badge" id="triggerCount">0</span></h2>
    <ul class="log-list" id="logList"></ul>
  </div>
  </div><!-- /follow-log-row -->


</div>

<footer>
  <b>동작 흐름</b> — 트리거 발생 → 대피 유도 → 층간 인수인계 → 비상구 도달.
  &nbsp;· <b>목표</b>: 트리거→선두 출발 ≤ 2.0초 · 비상구 유도 완료율 ≥ 80% · 모든 대피자 안내 ≥ 80%.
  &nbsp;· 모든 모드는 1.0 m/s 이하 (보행 속도 1.2–1.4 m/s보다 느려야 로봇이 앞서지 않음).
  &nbsp;· 키보드: <kbd>1</kbd> 화재 <kbd>2</kbd> 지진 <kbd>3</kbd> 정전 <kbd>0</kbd> 해제
</footer>

<script>
let PATH_2F = """ + json.dumps([
    {"step": i+1, "char": char, "speed": spd, "heading": hdg, "duration": dur, "meters": round(dur * DRIVE_MPS, 2) if spd > 0 else 0}
    for i, (spd, hdg, dur, char) in enumerate(EVAC_PATH_2F)
]) + r""";
const PATH_1F = """ + json.dumps([
    {"step": i+1, "char": char, "speed": spd, "heading": hdg, "duration": dur, "meters": round(dur * DRIVE_MPS, 2) if spd > 0 else 0}
    for i, (spd, hdg, dur, char) in enumerate(EVAC_PATH_1F)
]) + r""";
let PATH_R2 = """ + json.dumps([
    {"step": i+1, "char": char, "speed": spd, "heading": hdg, "duration": dur, "meters": round(dur * DRIVE_MPS, 2) if spd > 0 else 0}
    for i, (spd, hdg, dur, char) in enumerate(EVAC_PATH_R2)
]) + r""";
const TARGET_MODES = """ + json.dumps(TARGET_MODES, ensure_ascii=False, indent=1) + r""";
const DISASTERS    = """ + json.dumps(DISASTERS, ensure_ascii=False, indent=1) + r""";
const EXCEPTIONS   = """ + json.dumps(EXCEPTIONS, ensure_ascii=False, indent=1) + r""";
const PHASES = ['idle', 'armed', 'guiding', 'handoff', 'complete'];
const PATH_NOTES = {
  fire:  '<b>네트워크 경유</b> — 화재는 로봇이 직접 감지할 수 없음. 외부 신호(수신반 대역)를 중계기가 받아 전달.',
  quake: '<b>다중 교차검증</b> — 1대만 흔들리면 외부 접촉, 여러 대가 동시에 흔들리면 지진 확정. (PoC: R1 가속도 ' + """ + str(SHAKE_THRESHOLD) + r""" + 'G↑)',
  black: '<b>자립 감지</b> — 정전 시 공유기도 꺼짐. 로봇이 광센서로 스스로 판정하고 저장 경로로 유도. (PoC: 브로커가 저장 경로를 그대로 실행)'
};
const ROBOTS = [
  { id: 'R1', key: 'floor2', name: 'R1 선두',    role: 'LED 화살표 방향 표시',      color: '#ff6b35' },
  { id: 'R2', key: 'rear',   name: 'R2 후방',    role: '이탈 감지 · 스마트폰 탑재', color: '#fbbf24' },
  { id: 'R3', key: 'floor1', name: 'R3 1층 대기', role: '서버 신호로 층간 인수인계', color: '#34d399' },
  { id: 'R4', key: null,     name: 'R4 예비',    role: '고장 시 R1·R2 역할 대체',   color: '#9aa7b4' }
];

function postJSON(url, body) {
  return fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) })
    .then(() => update()).catch(console.error);
}
function sendDisaster(type) { postJSON('/api/disaster', { type }); }
function testRoute() {
  fetch('/api/test_route', { method: 'POST' })
    .then(r => r.json())
    .then(j => {
      const note = document.getElementById('triggerNote');
      note.innerHTML = j.ok
        ? `<b>🧪 테스트 경로</b> — ${j.robots} R1 직진 1m → 좌회전 1.5m, R2는 간격만큼 뒤에서 R1이 꺾은 곳에서 꺾음 (이벤트 로그에 구간 표시). 끝나면 '상황 해제'로 출발점 복귀`
        : '⚠ ' + j.error;
      update();
    })
    .catch(console.error);
}
function setTarget(mode)    { postJSON('/api/target', { mode }); }
function fireException(id)  { postJSON('/api/exception', { id }); }

document.getElementById('seg').innerHTML = Object.entries(TARGET_MODES).map(([k, m]) =>
  `<button id="seg_${k}" onclick="setTarget('${k}')"><span class="ico">${m.icon}</span>${m.label}` +
  `<span class="spd">${m.mps} m/s · ${m.led}</span></button>`).join('');
document.getElementById('excepts').innerHTML = Object.entries(EXCEPTIONS).map(([k, t]) =>
  `<button class="ex" id="ex_${k}" onclick="fireException('${k}')"><span class="eid">${k}</span>` +
  `<span class="etxt">${t}</span><span class="est"></span></button>`).join('');

document.addEventListener('keydown', e => {
  if (['INPUT', 'TEXTAREA'].includes(e.target.tagName) || e.metaKey || e.ctrlKey || e.altKey) return;
  const m = { '1': 'fire', '2': 'quake', '3': 'black', '0': 'clear' }[e.key];
  if (m) sendDisaster(m);
});

function robotPhase(r, d) {
  const ph = d.phase;
  if (!r.key) return d.exceptions.E5 === 'triggered' ? '투입 요청' : '예비 대기';
  const s = d[r.key];
  if (s.returning) return '복귀 중';
  if (r.key === 'floor1') {
    if (ph === 'idle') return 'IDLE';
    if (ph === 'armed' || ph === 'guiding') return 'ARMED';
    if (ph === 'handoff') return s.moving ? 'GUIDING' : 'HANDOFF';
    return 'COMPLETE';
  }
  if (ph === 'idle' || ph === 'armed') return ph.toUpperCase();
  if (ph === 'guiding') return s.moving || !s.done ? 'GUIDING' : 'HANDOFF';
  return ph === 'handoff' ? 'HANDOFF' : 'COMPLETE';
}

function renderFlow(d) {
  const ph = d.phase || 'idle', idx = PHASES.indexOf(ph);
  const pill = document.getElementById('statePill');
  pill.className = 'state-pill ' + ph;
  pill.textContent = ph.toUpperCase();
  PHASES.forEach((_, i) => {
    document.getElementById('tn' + i).className = 'tnode' + (i === idx ? ' active' : i < idx ? ' done' : '');
    if (i < 4) document.getElementById('ta' + i).className = 'tarrow' + (i < idx ? ' on' : '');
  });

  const dz = d.disaster ? DISASTERS[d.disaster] : null;
  const tm = TARGET_MODES[d.target];
  document.getElementById('metaDisaster').textContent = dz ? `${dz.icon} ${dz.label}` : '없음';
  document.getElementById('metaTarget').textContent   = `${tm.icon} ${tm.label} · ${tm.mps} m/s`;
  document.getElementById('metaLatency').textContent  = d.latency == null ? '—'
    : d.latency.toFixed(1) + 's' + (d.disaster === 'quake' ? ' (진동 종료 대기 포함)' : '');

  ['fire', 'quake', 'black'].forEach(k =>
    document.getElementById('dz' + k).classList.toggle('on', d.disaster === k));
  document.getElementById('triggerNote').innerHTML = dz
    ? `<b>${dz.label}</b> — ${dz.note}`
    : '평시 — 로봇 연결 상태(하트비트) 확인 중. 버튼이나 키보드 1·2·3으로 재난 상황을 발생시키세요.';

  Object.keys(TARGET_MODES).forEach(k =>
    document.getElementById('seg_' + k).classList.toggle('on', d.target === k));
  document.getElementById('targetNote').innerHTML =
    `<b>${tm.label}</b> — ${tm.led}, ${tm.mps} m/s. 음성: ${tm.voice}. ` +
    `Sphero 속도 ×${tm.mps}, 구간 시간 ÷${tm.mps}로 같은 거리 유지 (다음 주행부터 적용).`;

  document.getElementById('bots').innerHTML = ROBOTS.map(r => {
    const fall = r.id === 'R2' && followFallen;
    const st = fall ? '대피자 넘어짐' : robotPhase(r, d);
    const online = r.key && (d[r.key].connected || d.sim_mode);
    const conn = !r.key ? '하드웨어 없음' : d[r.key].connected ? '연결됨' : d.sim_mode ? 'SIM' : '미연결';
    const lit = online && st !== 'IDLE';
    const ballStyle = lit ? `color:${r.color};box-shadow:inset 0 0 0 1px ${r.color}` : online ? 'color:#6b7785' : '';
    return `<div class="bot ${fall ? 'fall' : st.toLowerCase()}">
      <div class="rball" style="${ballStyle}">${r.id}</div>
      <div style="min-width:0"><div class="n">${r.name} <span>· ${conn}${(r.key && d.positions && d.positions[r.key]) ? ` · 📍 ${d.positions[r.key].x}, ${d.positions[r.key].y}cm` : ''}</span></div><div class="s">${r.role}</div></div>
      <div class="st">${st}</div></div>`;
  }).join('');

  const anyConn = d.sim_mode || d.floor2.connected || d.floor1.connected || d.rear.connected;
  const running = ['guiding', 'handoff', 'complete'].includes(ph);
  let cs = ['', '', '', ''];
  if (d.disaster === 'black') cs = ['cut', 'cut', 'cut', running ? 'on' : ''];
  else if (d.disaster) cs = ['on', 'on', anyConn ? 'on' : 'cut', running || d.disaster === 'quake' ? 'on' : ''];
  cs.forEach((c, i) => {
    document.getElementById('cn' + i).className = 'cnode' + (c === 'on' ? ' active' : c === 'cut' ? ' cut' : '');
    if (i < 3) document.getElementById('cl' + i).className = 'clink' +
      (cs[i] === 'cut' || cs[i + 1] === 'cut' ? ' cut' : cs[i] === 'on' && cs[i + 1] === 'on' ? ' on' : '');
  });
  document.getElementById('pathNote').innerHTML = d.disaster
    ? PATH_NOTES[d.disaster] : '평시 — 신호 없음. 연결 상태로 로봇 고장 감지.';

  Object.keys(EXCEPTIONS).forEach(k => {
    const v = d.exceptions[k] || '';
    const el = document.getElementById('ex_' + k);
    el.className = 'ex ' + v;
    el.querySelector('.est').textContent = v === 'triggered' ? '발생' : v === 'resolved' ? '처리됨' : '';
  });
}

const ACC_LABEL = ['불안정 — 폰으로 8자를 그려 보정', '정확도 낮음', '정확도 보통', '정확도 높음'];
function renderAim(d) {
  const ph = d.phone || {};
  const fresh = ph.azimuth != null && ph.age != null && ph.age < 5;
  document.getElementById('phoneAz').textContent = fresh ? Math.round(ph.azimuth) + '°' : '—';
  document.getElementById('phoneAzNote').textContent =
    !fresh ? '앱에서 방위 수신 없음' :
    ph.quality < 0.5 ? '폰을 세워 주세요 (셀피가 진행 방향을 보게)' :
    (ACC_LABEL[ph.accuracy] || '');
  Object.entries(d.aim_trim || {}).forEach(([k, t]) => {
    const el = document.getElementById('aimTrim_' + k);
    if (el) el.textContent = '보정 ' + (t > 0 ? '+' : '') + t + '°';
  });
  Object.entries(d.aim || {}).forEach(([k, a]) => {
    const el = document.getElementById('aimSt_' + k);
    if (!el) return;
    el.textContent = a.msg || '';
    el.className = 'aim-st ' + (a.status || '');
    if (a.status === 'done') document.getElementById('aimDeg_' + k).textContent = '0° ✔';
  });
}
function autoAim() {
  const hint = document.getElementById('aimHint');
  fetch('/api/aim/auto', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' })
    .then(r => r.json())
    .then(j => {
      hint.style.color = j.ok ? '' : 'var(--red)';
      hint.textContent = j.ok
        ? `🧭 자동 에임 시작: ${j.started.join(', ')} — 로봇이 제자리에서 돌며 북쪽을 찾습니다 (셀피 방위 ${Math.round(j.azimuth)}°)`
          + (j.skipped.length ? ` · 건너뜀: ${j.skipped.join(', ')}` : '')
        : '⚠ ' + j.error;
    })
    .catch(e => { hint.textContent = '⚠ 요청 실패: ' + e; hint.style.color = 'var(--red)'; });
}

const SPEED_REF = """ + str(SPEED_REF) + r""";
const DEFAULT_SPEEDS = """ + json.dumps(DEFAULT_SPEEDS) + r""";
let speeds = Object.assign({}, DEFAULT_SPEEDS);
const SPEED_KEYS = ['floor2', 'rear', 'floor1'];
const DEFAULT_R2_DELAY = """ + str(DEFAULT_R2_DELAY_SEC) + r""";
const DEFAULT_TURN_PAUSE = """ + str(DEFAULT_TURN_PAUSE_SEC) + r""";
const DEFAULT_R2_DIST = """ + str(DEFAULT_R2_DIST_PCT) + r""";
const DEFAULT_GAP = """ + str(FOLLOW_GAP_M) + r""";
function renderSpeeds(d) {
  if (!d.speeds) return;
  speeds = d.speeds;
  SPEED_KEYS.forEach(k => {
    const el = document.getElementById('spd_' + k);
    if (!el.dataset.touched) el.value = d.speeds[k];
  });
  const dl = document.getElementById('spd_r2delay');
  if (!dl.dataset.touched && d.r2_delay != null) dl.value = d.r2_delay;
  const tp = document.getElementById('spd_turnpause');
  if (!tp.dataset.touched && d.turn_pause != null) tp.value = d.turn_pause;
  const rd = document.getElementById('spd_r2dist');
  if (!rd.dataset.touched && d.r2_dist_pct != null) rd.value = d.r2_dist_pct;
  const gp = document.getElementById('spd_gap');
  if (!gp.dataset.touched && d.follow_gap != null) gp.value = Math.round(d.follow_gap * 100);
}
function postSpeeds(body) {
  const hint = document.getElementById('speedHint');
  fetch('/api/speeds', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) })
    .then(r => r.json())
    .then(j => {
      if (!j.ok) { hint.textContent = '⚠ ' + j.error; hint.style.color = 'var(--red)'; return; }
      SPEED_KEYS.forEach(k => { const el = document.getElementById('spd_' + k); delete el.dataset.touched; el.value = j.speeds[k]; });
      const dl = document.getElementById('spd_r2delay'); delete dl.dataset.touched; dl.value = j.r2_delay;
      const tp = document.getElementById('spd_turnpause'); delete tp.dataset.touched; tp.value = j.turn_pause;
      const rd = document.getElementById('spd_r2dist'); delete rd.dataset.touched; rd.value = j.r2_dist_pct;
      const gp = document.getElementById('spd_gap'); delete gp.dataset.touched; gp.value = Math.round(j.follow_gap * 100);
      const t2 = (PATH_2F.reduce((a, p) => a + p.duration, 0) * SPEED_REF / j.speeds.floor2).toFixed(1);
      const t1 = (PATH_1F.reduce((a, p) => a + p.duration, 0) * SPEED_REF / j.speeds.floor1).toFixed(1);
      hint.style.color = '';
      hint.textContent = `✔ 저장됨 — R1 ${j.speeds.floor2} · R2 ${j.speeds.rear} (${j.r2_delay}초 뒤 출발, 거리 ${j.r2_dist_pct}%) · R3 ${j.speeds.floor1} · 꺾기 전 ${j.turn_pause}초 정지 · 간격 ${Math.round(j.follow_gap * 100)}cm · 예상 2층 ${t2}초 / 1층 ${t1}초 (일반 모드)`;
      update();
    })
    .catch(e => { hint.textContent = '⚠ 요청 실패: ' + e; hint.style.color = 'var(--red)'; });
}
function saveSpeeds() {
  const body = {};
  SPEED_KEYS.forEach(k => { body[k] = parseInt(document.getElementById('spd_' + k).value, 10); });
  body.r2_delay = parseFloat(document.getElementById('spd_r2delay').value);
  body.turn_pause = parseFloat(document.getElementById('spd_turnpause').value);
  body.r2_dist_pct = parseFloat(document.getElementById('spd_r2dist').value);
  body.follow_gap = parseFloat(document.getElementById('spd_gap').value) / 100;   // cm → m
  postSpeeds(body);
}
function resetSpeeds() { postSpeeds(Object.assign({}, DEFAULT_SPEEDS, { r2_delay: DEFAULT_R2_DELAY, turn_pause: DEFAULT_TURN_PAUSE, r2_dist_pct: DEFAULT_R2_DIST, follow_gap: DEFAULT_GAP })); }
SPEED_KEYS.concat(['r2delay', 'turnpause', 'r2dist', 'gap']).forEach(k => document.getElementById('spd_' + k)
  .addEventListener('input', e => { e.target.dataset.touched = '1'; }));

function aim(robot, action, delta) {
  const hint = document.getElementById('aimHint');
  fetch('/api/aim', { method: 'POST', headers: { 'Content-Type': 'application/json' },
                      body: JSON.stringify({ robot, action, delta: delta || 0 }) })
    .then(r => r.json())
    .then(j => {
      if (!j.ok) { hint.textContent = '⚠ ' + j.error; hint.style.color = 'var(--red)'; return; }
      hint.style.color = '';
      document.getElementById('aimDeg_' + robot).textContent = action === 'set' ? '0° ✔' : j.heading + '°';
    })
    .catch(e => { hint.textContent = '⚠ 요청 실패: ' + e; hint.style.color = 'var(--red)'; });
}

function copyAppUrl() {
  const url = document.getElementById('appUrl').textContent;
  const done = () => {
    const el = document.getElementById('appUrlCopied');
    el.style.display = 'inline';
    setTimeout(() => { el.style.display = 'none'; }, 1500);
  };
  if (navigator.clipboard && window.isSecureContext) {
    navigator.clipboard.writeText(url).then(done).catch(() => {});
  } else {
    const ta = document.createElement('textarea');
    ta.value = url; document.body.appendChild(ta); ta.select();
    try { document.execCommand('copy'); done(); } catch (e) {}
    ta.remove();
  }
}

function fallMetricText(f) {
  return `w/h ${f.aspect == null ? '—' : f.aspect.toFixed(2)} · 기울기 ${f.tilt == null ? '—' : Math.round(f.tilt) + '°'}`;
}
function renderFall(f) {
  const was = followFallen;
  followFallen = !!(f.active && f.fallen);
  document.getElementById('fallAlert').classList.toggle('on', followFallen);
  document.getElementById('followPreviewWrap').classList.toggle('fallen', followFallen);
  document.getElementById('followMetrics').textContent = f.active ? fallMetricText(f) : '';
  if (followFallen) {
    document.getElementById('fallSecs').textContent = Math.floor(f.fallen_secs || 0);
    const stale = f.stale_secs != null && f.stale_secs > 3;
    document.getElementById('fallMeta').textContent =
      stale ? `카메라 신호 ${Math.floor(f.stale_secs)}초 끊김` : fallMetricText(f);
  }
  if (was !== followFallen) update();
}

function toggleFloorplan() {
  const wrap = document.getElementById('floorplanWrap');
  const btn  = document.getElementById('btnFloorplanToggle');
  const open = wrap.style.display === 'none';
  wrap.style.display = open ? 'block' : 'none';
  btn.textContent = open ? '▲ 접기' : '▼ 펼치기';
}
let fpManual = false, fpFloor = '2f';
function floorplanTab(floor, manual) {
  if (manual) fpManual = true;
  fpFloor = floor;
  ['2f','1f'].forEach(f => {
    document.getElementById('fpStage' + f).style.display = f === floor ? 'block' : 'none';
    const t = document.getElementById('fpTab' + f);
    t.style.background = f === floor ? 'var(--blu)' : 'var(--bdr)';
    t.style.color      = f === floor ? '#fff' : 'var(--txt)';
  });
  const wrap = document.getElementById('floorplanWrap');
  if (wrap.style.display === 'none') toggleFloorplan();
}

// 도면 이미지 기준 % 좌표 (도면 PNG의 파란 스피로 경로 꺾임점)
const ROUTE_2F = [[64.1,60.8],[54.6,60.8],[54.6,89.6]];   // ROUTE_2F_M 구간과 1:1 (전체: [66.1,44.7],[64.1,45.3] 앞에 추가)
const ROUTE_1F = [[67.5,45.0],[56.1,45.0],[56.1,18.5]];
const ROUTE_2F_LEN_M = """ + json.dumps([m for _, m in ROUTE_2F_M]) + r""";
const ROUTE_1F_LEN_M = """ + json.dumps([m for _, m in ROUTE_1F_M]) + r""";
let FOLLOW_GAP_M = """ + str(FOLLOW_GAP_M) + r""";
// 경로 출발점에서 m미터 지점의 도면 좌표 — 구간(꺾임점) 단위로 도면 실측 거리를 따라감
function pointAtMeters(route, lens, m) {
  m = Math.max(0, m);
  for (let i = 0; i < lens.length; i++) {
    if (m <= lens[i] || i === lens.length - 1) {
      const r = lens[i] ? Math.min(1, m / lens[i]) : 0;
      return [route[i][0] + (route[i+1][0] - route[i][0]) * r,
              route[i][1] + (route[i+1][1] - route[i][1]) * r];
    }
    m -= lens[i];
  }
}
let modeFactor = 1;
// offsetM: 이 로봇이 경로 출발점에서 몇 m 앞에서 출발하는지 (R1 = 4m, R2·R3 = 0)
let testRunning = false;
function placeRobot(id, s, route, lens, path, offsetM, timeScale) {
  const el = document.getElementById(id);
  let m = offsetM, glide = 1.4, cls = 'idle', word = '대기';
  if (!s.connected && !simMode && !s.moving) { cls = ''; word = '미연결'; }
  if (s.moving && s.progress_m != null) {
    m = s.progress_m; glide = 0.3; cls = 'moving'; word = '따라가기';
    if (id === 'robotR1') word = '주행';
  } else if (testRunning && s.moving) {
    cls = 'moving'; word = '테스트';
  } else if (s.returning) {
    glide = s.return_secs || 1.4; cls = 'moving'; word = '복귀';
  } else if (s.moving) {
    const k = s.step || 0;
    m = offsetM + path.slice(0, k).reduce((a, p) => a + (p.meters || 0), 0);   // 현재 구간 끝 지점
    const cur = path[k - 1];
    if (cur && cur.duration > 0) glide = cur.duration * timeScale / modeFactor;   // 실제 주행 시간만큼 이동
    cls = 'moving'; word = '주행';
  } else if (s.done) {
    m = offsetM + path.reduce((a, p) => a + (p.meters || 0), 0);
    cls = 'done'; word = '도착';
  }
  const [x, y] = pointAtMeters(route, lens, m);
  if (el.dataset.m !== String(m)) {
    el.style.transition = `left ${glide}s linear, top ${glide}s linear`;
    el.dataset.m = String(m);
  }
  el.style.left = x + '%';
  el.style.top  = y + '%';
  el.className  = 'robot ' + cls + (id === 'robotR2' ? ' rb-rear' : '');
  el.querySelector('.rb-tag').textContent = el.querySelector('.rb-dot').textContent + ' ' + word;
}
let simMode = false;
let followFallen = false;
function updateFloorplan(d) {
  simMode = !!d.sim_mode;
  modeFactor = (TARGET_MODES[d.target] || {}).mps || 1;
  const sp = d.speeds || DEFAULT_SPEEDS;
  placeRobot('robotR1', d.floor2, ROUTE_2F, ROUTE_2F_LEN_M, PATH_2F, FOLLOW_GAP_M, SPEED_REF / sp.floor2);
  // 따라가기 중 R1 진행 거리는 R2 출발점 기준 — offset 없이 그대로
  placeRobot('robotR2', d.rear,   ROUTE_2F, ROUTE_2F_LEN_M, PATH_R2, 0, SPEED_REF / sp.rear * (d.r2_dist_pct || 100) / 100);
  placeRobot('robotR3', d.floor1, ROUTE_1F, ROUTE_1F_LEN_M, PATH_1F, 0, SPEED_REF / sp.floor1);
  if (followFallen) {
    const r2 = document.getElementById('robotR2');
    r2.classList.add('fallen');
    r2.querySelector('.rb-tag').textContent = 'R2 ⚠ 넘어짐 감지';
  }
  // 주행 중인 층을 자동으로 보여줌 (사용자가 탭을 누르면 자동 전환 중단)
  if (!fpManual) {
    if (d.floor1.moving && fpFloor !== '1f') floorplanTab('1f', false);
    else if ((d.floor2.moving || d.rear.moving) && fpFloor !== '2f') floorplanTab('2f', false);
  }
}

function triggerSignal() {
  fetch('/api/trigger', { method: 'POST' }).catch(console.error);
}
function resetBroker() {
  fetch('/api/reset', { method: 'POST' }).catch(console.error);
}

function restartBroker() {
  if (!confirm('브로커를 재시작합니다. 잠시 후 페이지를 새로고침하세요.')) return;
  fetch('/api/restart', { method: 'POST' })
    .finally(() => setTimeout(() => location.reload(), 1500));
}

function disconnectSphero() {
  const hint = document.getElementById('connectHint');
  hint.textContent = '⏏ 연결 해제 중…';
  hint.style.color = 'var(--yel)';
  fetch('/api/disconnect', { method: 'POST' })
    .then(r => r.json())
    .then(() => {
      hint.textContent = '✅ 연결 해제됨 — 다시 연결하려면 연결 시작을 누르세요';
      hint.style.color = 'var(--dim)';
      document.getElementById('btnConnect').disabled = false;
    })
    .catch(e => {
      hint.textContent = '❌ 해제 실패: ' + e;
      hint.style.color = 'var(--red)';
    });
}

function connectSphero() {
  const n2f = document.getElementById('inp2f').value.trim();
  const n1f = document.getElementById('inp1f').value.trim();
  const nR2 = document.getElementById('inpR2').value.trim();
  const hint = document.getElementById('connectHint');
  const btn  = document.getElementById('btnConnect');

  if (!n2f && !n1f && !nR2) {
    hint.textContent = '⚠ 이름을 최소 하나 입력하세요.';
    hint.style.color = 'var(--red)';
    return;
  }

  btn.disabled = true;
  hint.textContent = '🔍 BLE 스캔 중… (10~30초 소요)';
  hint.style.color = 'var(--yel)';

  fetch('/api/connect', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name_2f: n2f, name_1f: n1f, name_r2: nR2 })
  })
  .then(r => r.json())
  .then(d => {
    if (d.ok) {
      hint.textContent = '✅ 연결 시도 중 — 로그 확인';
      hint.style.color = 'var(--grn)';
    } else {
      hint.textContent = '❌ ' + (d.error || '오류');
      hint.style.color = 'var(--red)';
      btn.disabled = false;
    }
  })
  .catch(e => {
    hint.textContent = '❌ 요청 실패: ' + e;
    hint.style.color = 'var(--red)';
    btn.disabled = false;
  });
}

function update() {
  fetch('/api/status')
    .then(r => r.json())
    .then(d => {
      if (d.paths) { PATH_2F = d.paths.floor2; PATH_R2 = d.paths.rear; }
      if (d.follow_gap != null) FOLLOW_GAP_M = d.follow_gap;
      testRunning = !!d.test_running;
      // sim badge
      if (d.sim_mode) document.getElementById('simBadge').style.display = 'inline-block';

      // 2F
      const c2 = d.floor2.connected;
      document.getElementById('dot2f').className   = 'dot' + (c2 ? ' on' : '');
      document.getElementById('conn2f').textContent = c2 ? '연결됨' : '연결 안됨';
      document.getElementById('bat2f').style.width  = d.floor2.battery + '%';
      document.getElementById('bat2fTxt').textContent = `배터리 ${d.floor2.battery}%`;
      document.getElementById('ax').textContent = (d.floor2.accel?.x ?? 0).toFixed(2);
      document.getElementById('ay').textContent = (d.floor2.accel?.y ?? 0).toFixed(2);
      document.getElementById('az').textContent = (d.floor2.accel?.z ?? 0).toFixed(2);

      const pill = document.getElementById('signalPill');
      if (d.floor2.signal) {
        pill.textContent = '신호!'; pill.className = 'signal-pill active';
      } else {
        pill.textContent = '대기';  pill.className = 'signal-pill';
      }

      // 1F
      const c1 = d.floor1.connected;
      document.getElementById('dot1f').className   = 'dot' + (c1 ? ' on' : '');
      document.getElementById('conn1f').textContent = c1 ? '연결됨' : '연결 안됨';
      document.getElementById('bat1f').style.width  = d.floor1.battery + '%';
      document.getElementById('bat1fTxt').textContent = `배터리 ${d.floor1.battery}%`;

      const mov = d.floor1.moving;
      document.getElementById('movingBadge').textContent = mov ? '🏃 이동 중' : '';

      // 2F path steps
      function renderSteps(containerId, path, curStep, timeScale) {
        const el = document.getElementById(containerId);
        el.innerHTML = '';
        path.forEach(p => {
          const sp = document.createElement('div');
          sp.className = 'step-box' + (curStep === p.step ? ' active' : curStep > p.step ? ' done' : '');
          sp.textContent = `${p.char} ${p.duration > 0 ? (p.duration * timeScale).toFixed(1) + 's' : '정지'}`;
          sp.title = `step${p.step}: ${p.meters}m 방향${p.heading}°`;
          el.appendChild(sp);
        });
      }
      const ts2 = SPEED_REF / (d.speeds ? d.speeds.floor2 : SPEED_REF);
      const ts1 = SPEED_REF / (d.speeds ? d.speeds.floor1 : SPEED_REF);
      renderSteps('pathSteps2f', PATH_2F, d.floor2.step || 0, ts2);
      const pct2f = d.floor2.total_steps > 0
        ? Math.round((d.floor2.step || 0) / d.floor2.total_steps * 100) : 0;
      document.getElementById('pathProgress2f').style.width = pct2f + '%';
      document.getElementById('pathLabel2f').textContent = d.floor2.moving
        ? `경로 실행 중: ${d.floor2.step} / ${d.floor2.total_steps}`
        : ((d.floor2.step || 0) === 0 ? '대기 중' : '완료');

      // 1F path steps
      renderSteps('pathSteps', PATH_1F, d.floor1.step, ts1);
      const pct = d.floor1.total_steps > 0
        ? Math.round(d.floor1.step / d.floor1.total_steps * 100) : 0;
      document.getElementById('pathProgress').style.width = pct + '%';
      document.getElementById('pathLabel').textContent = mov
        ? `경로 실행 중: ${d.floor1.step} / ${d.floor1.total_steps}`
        : (d.floor1.step === 0 ? '대기 중' : '완료');

      // R2 후방
      const r2 = d.rear;
      document.getElementById('dotR2').className   = 'dot' + (r2.connected ? ' on' : '');
      document.getElementById('connR2').textContent = r2.connected ? '연결됨' : '연결 안됨';
      document.getElementById('batR2Txt').textContent = `배터리 ${r2.battery}%`;
      const tsR2 = SPEED_REF / (d.speeds ? d.speeds.rear : SPEED_REF) * (d.r2_dist_pct || 100) / 100;
      renderSteps('pathStepsR2', PATH_R2, r2.step || 0, tsR2);
      document.getElementById('pathLabelR2').textContent = r2.moving
        ? `R1 경로 추종 중: ${r2.step} / ${r2.total_steps}`
        : ((r2.step || 0) === 0 ? '대기 중' : '완료');

      updateFloorplan(d);
      renderFlow(d);
      renderAim(d);
      renderSpeeds(d);
      document.getElementById('appUrl').textContent =
        `http://${d.server_ip}:${location.port || (location.protocol === 'https:' ? 443 : 80)}`;

      // counter
      document.getElementById('triggerCount').textContent = d.trigger_count;

      // connect form: 현재 이름 채우기 (한 번만)
      const inp2f = document.getElementById('inp2f');
      const inp1f = document.getElementById('inp1f');
      if (!inp2f.dataset.touched) inp2f.value = d.names?.floor2 || '';
      if (!inp1f.dataset.touched) inp1f.value = d.names?.floor1 || '';
      const inpR2 = document.getElementById('inpR2');
      if (!inpR2.dataset.touched) inpR2.value = d.names?.rear || '';
      inp2f.className = d.floor2.connected ? 'ok' : '';
      inp1f.className = d.floor1.connected ? 'ok' : '';
      inpR2.className = d.rear.connected ? 'ok' : '';

      // 연결 중 → 버튼 비활성
      const btn = document.getElementById('btnConnect');
      if (!d.connecting) btn.disabled = false;

      // log
      const ul = document.getElementById('logList');
      ul.innerHTML = '';
      (d.log || []).forEach(e => {
        const li = document.createElement('li');
        li.innerHTML = `<span class="log-ts">${e.ts}</span><span>${e.msg}</span>`;
        ul.appendChild(li);
      });
    })
    .catch(console.error);
}

['inp2f','inp1f','inpR2'].forEach(id => {
  document.getElementById(id).addEventListener('input', e => {
    e.target.dataset.touched = '1';
  });
});

const DIR_LABEL = { forward: '↑  멀어짐', back: '↓  다가옴', left: '←  왼쪽', right: '→  오른쪽', stop: '■  정지' };
let followActive = false;
function renderFollow(active) {
  followActive = active;
  const btn = document.getElementById('btnFollowToggle');
  btn.textContent = active ? '⏹ 감지 OFF' : '▶ 감지 ON';
  btn.className   = 'btn ' + (active ? 'btn-follow-off' : 'btn-follow-on');
}
function toggleFollow() {
  fetch('/api/follow/toggle', { method: 'POST' })
    .then(r => r.json())
    .then(d => renderFollow(d.active));
}
setInterval(() => {
  fetch('/api/follow/status').then(r => r.json()).then(f => {
    renderFollow(f.active);
    renderFall(f);
    const dir  = document.getElementById('followDir');
    const info = document.getElementById('followInfo');
    if (!f.active) { dir.textContent = '■'; info.textContent = '감지 꺼짐'; dir.style.color = ''; return; }
    if (f.fallen) {
      dir.textContent = '⚠ 넘어짐'; dir.style.color = 'var(--red)';
      info.textContent = '대피자 넘어짐 감지 (3초 이상)';
    } else {
      dir.textContent = DIR_LABEL[f.direction] || f.direction; dir.style.color = '';
      info.textContent = '대피자 움직임 감지 중';
    }
  }).catch(() => {});
}, 500);

// 프리뷰 snapshot polling (150ms) — 단순 src 교체
setInterval(() => {
  const img = document.getElementById('followPreview');
  img.src = '/api/follow/snapshot?t=' + Date.now();
  img.style.display = 'block';
  document.getElementById('followPlaceholder').style.display = 'none';
}, 150);

update();
setInterval(update, 800);
</script>
</body>
</html>"""

# ── 진입점 ────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Sphero BOLT PoC 브로커")
    parser.add_argument("--sim", action="store_true",
                        help="시뮬레이션 모드 (Sphero 없이 대시보드 테스트)")
    parser.add_argument("--port", type=int, default=FLASK_PORT)
    args = parser.parse_args()

    with _state_lock:
        state["sim_mode"] = args.sim

    log_event(f"🚀 Sphero PoC 브로커 시작 — 대시보드: http://localhost:{args.port}")
    if args.sim:
        log_event("💡 --sim 모드: 대시보드 버튼으로 신호 테스트 가능")

    broker_thread = threading.Thread(
        target=_broker_main, args=(args.sim,), daemon=True)
    broker_thread.start()

    app.run(host="0.0.0.0", port=args.port, debug=False, use_reloader=False, threaded=True)
