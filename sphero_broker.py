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
import threading
import time
import json
import logging
import base64
import queue as _queue
from datetime import datetime
from flask import Flask, jsonify, render_template_string, request as flask_request
from flask_sock import Sock

# 동네 프로젝트 등에서 사용하는 TypeSafe AI API 키
JEV_KEY       = os.environ.get("JEV_API_KEY", "").strip()
ANTHROPIC_KEY = os.environ.get("ANTHROPIC_API_KEY", "").strip()
_vision_busy  = threading.Lock()   # Vision 호출 중복 방지

logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')

# ── 설정 ───────────────────────────────────────────────────────────────────────
SPHERO_2F_NAME  = "SB-25D4"   # 2층 Sphero BLE 이름 (R1 선두)
SPHERO_1F_NAME  = "SB-D721"   # 1층 Sphero BLE 이름 (R3 대기)
SPHERO_R2_NAME  = ""          # 2층 후방 Sphero BLE 이름 (R2, 스마트폰 트롤리)
# R2는 R1 바로 뒤 같은 선상에 두고 출발 → 이 시간만큼 직진해 R1 출발점에 온 뒤 R1 경로를 그대로 재생
# (간격 ≈ R1 첫 구간 속도 × 이 시간 — 현장에서 실측 후 조정)
R2_GAP_SEC      = 2.0
FLASK_PORT      = 5001
FLOORPLAN_PDF   = os.path.join(os.path.dirname(__file__), "팔달관_도면.pdf")
SHAKE_THRESHOLD = 2.5          # G  — 이 이상이면 흔들기 신호로 판정
SIGNAL_COOLDOWN = 10           # 초  — 한 번 신호 후 재차단 시간

# 2F 경로: (속도, 헤딩, 지속 초, 매트릭스 표시 문자)
# ※ 헤딩은 시연 장소 calibrate 후 수정 (0=북, 90=동, 180=남, 270=서)
EVAC_PATH_2F = [
    (60,   0,  2.0, "^"),   # 직진 2초 → 매트릭스 ↑
    (60,  90,  1.5, ">"),   # 우회전 1.5초 → 매트릭스 →
    (0,    0,  0.0, "v"),   # 정지 (계단 앞 핸드오프) → 매트릭스 ↓
]

# 1F 경로: (속도, 헤딩, 지속 초, 매트릭스 표시 문자)
EVAC_PATH_1F = [
    (60,   0,  2.0, "^"),   # 직진 2초
    (60,  90,  1.5, ">"),   # 우회전 1.5초
    (50, 180,  1.0, "v"),   # 비상구 방향 1초
    (0,    0,  0.0, "!"),   # 정지 (비상구 도달)
]

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
               "total_steps": len(EVAC_PATH_2F) + 1, "battery": 0},
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
_run_gen         = 0    # 트리거·리셋마다 증가 — 이전 세대 주행 스레드는 스스로 멈춘다

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

def _r2_path(path):
    """R1 출발점까지 간격 구간을 먼저 달린 뒤 R1 경로를 그대로 따라간다."""
    first_speed, first_heading = path[0][0], path[0][1]
    return [(first_speed, first_heading, R2_GAP_SEC, "^")] + list(path)

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
    if not r1_on and r2_on:
        raise_exception("E5", "R1 미연결 → R2가 R1 경로를 대신 선도")
    if not _active("floor1"):
        raise_exception("E5", "R3 미연결 → 1층 인수인계 불가")
    _set_phase("guiding")
    if r2_on:
        threading.Thread(
            target=_run_path,
            args=(_bolt_r2, _r2_path(EVAC_PATH_2F), "rear", "R2"),
            kwargs={"move_color": (255, 180, 0), "done_color": (255, 0, 0), "gen": gen},
            daemon=True,
        ).start()
    if r1_on or r2_on:
        if not r1_on:
            log_event("ℹ️  R1 미연결 — R2만 R1 경로를 따라 주행")
        if not _run_path(_bolt_2f, EVAC_PATH_2F, "floor2", "2F",
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

def _run_path(bolt, path, state_key, label, move_color, done_color, gen=None):
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
        log_event(f"⏹ {label} 주행 중단 (상황 해제)")
        return False

    with _state_lock:
        state[state_key]["moving"] = True
        state[state_key]["done"]   = False
        state[state_key]["step"]   = 0

    _set_led(bolt, *move_color)
    factor = TARGET_MODES[state["target"]]["mps"]

    for i, (speed, heading, duration, char) in enumerate(path):
        if cancelled():
            return abort()
        with _state_lock:
            state[state_key]["step"] = i + 1

        _set_matrix(bolt, *move_color, char)

        if duration > 0:
            speed, duration = int(speed * factor), round(duration / factor, 2)
            log_event(f"  → {label} step {i+1}/{len(path)}: "
                      f"속도={speed}  방향={heading}°  {duration}s  [{char}]")
            if bolt is not None:
                try:
                    bolt.drive_control.roll_start(heading, speed)
                except Exception as e:
                    log_event(f"  [이동 오류] {e}")
                    raise_exception("E4", f"{label} 명령 실패 — 남은 경로 계속 진행")
            if i == 0 and state_key in ("floor2", "rear") and state["latency"] is None:
                lat = round(time.time() - _trigger_ts, 2)
                with _state_lock:
                    state["latency"] = lat
                log_event(f"⏱ 트리거→선두 출발 {lat:.1f}s (목표 ≤ 2.0s)")
            end = time.time() + duration
            while time.time() < end and not cancelled():
                time.sleep(0.05)
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

        # 2F 가속도 센서 스트리밍 (sensor_control 고수준 API)
        try:
            b2.sensor_control.set_interval(200)
            b2.sensor_control.enable('accelerometer')
            b2.sensor_control.add_sensor_data_listener(_on_2f_sensor)
            log_event("📡 2F 가속도 센서 스트리밍 시작 (흔들면 신호)")
        except Exception as e:
            log_event(f"⚠ 센서 스트리밍 불가: {e} — 대시보드 버튼으로 트리거 가능")

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

@app.route("/api/disaster", methods=["POST"])
def api_disaster():
    kind = (flask_request.get_json(silent=True) or {}).get("type")
    if kind == "clear":
        api_reset()
        log_event("🟢 상황 해제 — 전 로봇 평시 복귀")
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
        _bolt_r2.reset_aim()
        log_event("🧭 R2 heading 0° 리셋 (R1과 방향 맞춤)")
        return jsonify({"ok": True})
    except Exception as e:
        log_event(f"🧭 캘리브레이션 실패: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500

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

    with _follow_lock:
        prev_dir, prev_fallen = _follow["direction"], _follow["fallen"]
        _follow["direction"] = direction
        _follow["fallen"] = fallen
        _follow["last_ts"] = time.time()

    if fallen and not prev_fallen:
        log_event("⚠ R2 카메라: 넘어짐 감지 (3초 이상)")
    elif prev_fallen and not fallen:
        log_event("✅ R2 카메라: 넘어짐 해제")
    if direction != prev_dir:
        log_event(f"👁 R2 카메라 → {direction} (x={ratio:.2f})")
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
    with _follow_lock:
        d = dict(_follow)
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
<title>Sphero PoC 브로커</title>
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

footer { padding: 0 20px 24px; color: var(--ink3); font-size: 11px; line-height: 1.75; }
footer b { color: var(--ink2); }
kbd { background: var(--panel2); border: 1px solid var(--bdr); border-radius: 4px; padding: 1px 5px; font-size: 10.5px; }
</style>
</head>
<body>

<header>
  <span style="font-size:22px;">🤖</span>
  <h1>Sphero PoC 브로커 &nbsp;—&nbsp; 재난 대피 안내 로봇</h1>
  <span class="sim-badge" id="simBadge">SIM</span>
  <span class="state-pill" id="statePill" title="시스템 상태">IDLE</span>
  <button class="hdr-btn" onclick="restartBroker()">↺ 브로커 재시작</button>
</header>

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

  <!-- Follow Mode 카드 -->
  <div class="follow-log-row">
  <div class="card follow-wrap">
    <h2>R2 후방 (R1 경로 추종 · 카메라 감지)</h2>
    <div class="conn-row" style="display:flex; align-items:center; gap:8px; margin-bottom:6px;">
      <div class="dot" id="dotR2"></div>
      <span id="connR2" style="font-weight:600;">연결 안됨</span>
      <span class="sub-label" id="batR2Txt">배터리 —</span>
    </div>
    <div class="path-steps" id="pathStepsR2"></div>
    <div class="sub-label" id="pathLabelR2" style="margin:6px 0 14px;">대기 중</div>
    <div class="follow-inner">
      <div class="follow-preview-wrap">
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
const PATH_2F = """ + json.dumps([
    {"step": i+1, "char": char, "speed": spd, "heading": hdg, "duration": dur}
    for i, (spd, hdg, dur, char) in enumerate(EVAC_PATH_2F)
]) + r""";
const PATH_1F = """ + json.dumps([
    {"step": i+1, "char": char, "speed": spd, "heading": hdg, "duration": dur}
    for i, (spd, hdg, dur, char) in enumerate(EVAC_PATH_1F)
]) + r""";
const PATH_R2 = """ + json.dumps([
    {"step": i+1, "char": char, "speed": spd, "heading": hdg, "duration": dur}
    for i, (spd, hdg, dur, char) in enumerate(_r2_path(EVAC_PATH_2F))
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
    const st = robotPhase(r, d);
    const online = r.key && (d[r.key].connected || d.sim_mode);
    const conn = !r.key ? '하드웨어 없음' : d[r.key].connected ? '연결됨' : d.sim_mode ? 'SIM' : '미연결';
    const lit = online && st !== 'IDLE';
    const ballStyle = lit ? `color:${r.color};box-shadow:inset 0 0 0 1px ${r.color}` : online ? 'color:#6b7785' : '';
    return `<div class="bot ${st.toLowerCase()}">
      <div class="rball" style="${ballStyle}">${r.id}</div>
      <div style="min-width:0"><div class="n">${r.name} <span>· ${conn}</span></div><div class="s">${r.role}</div></div>
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
const FP_W = 1150, FP_H = 740;
const ROUTE_2F = [[66.1,44.7],[64.1,45.3],[64.1,60.8],[54.6,60.8],[54.6,89.6]];
const ROUTE_1F = [[67.5,45.0],[56.1,45.0],[56.1,18.5]];
function pointOnRoute(route, t) {
  t = Math.max(0, Math.min(1, t));
  const seg = [];
  let total = 0;
  for (let i = 1; i < route.length; i++) {
    const dx = (route[i][0] - route[i-1][0]) * FP_W, dy = (route[i][1] - route[i-1][1]) * FP_H;
    const len = Math.hypot(dx, dy);
    seg.push(len); total += len;
  }
  let d = t * total;
  for (let i = 0; i < seg.length; i++) {
    if (d <= seg[i] || i === seg.length - 1) {
      const r = seg[i] ? Math.min(1, d / seg[i]) : 0;
      return [route[i][0] + (route[i+1][0] - route[i][0]) * r,
              route[i][1] + (route[i+1][1] - route[i][1]) * r];
    }
    d -= seg[i];
  }
}
// leadSteps: R2처럼 R1 출발점까지 먼저 달리는 간격 구간 수
function placeRobot(id, s, route, leadSteps) {
  const el = document.getElementById(id);
  let t = 0, cls = 'idle', word = '대기';
  if (!s.connected && !simMode && !s.moving) { cls = ''; word = '미연결'; }
  if (s.moving) {
    const n = Math.max(1, s.total_steps - leadSteps);
    t = Math.max(0, (s.step || 0) - leadSteps) / n;
    cls = 'moving'; word = '주행';
  } else if (s.done) {
    t = 1; cls = 'done'; word = '도착';
  }
  const [x, y] = pointOnRoute(route, t);
  el.style.left = x + '%';
  el.style.top  = y + '%';
  el.className  = 'robot ' + cls + (id === 'robotR2' ? ' rb-rear' : '');
  el.querySelector('.rb-tag').textContent = el.querySelector('.rb-dot').textContent + ' ' + word;
}
let simMode = false;
function updateFloorplan(d) {
  simMode = !!d.sim_mode;
  placeRobot('robotR1', d.floor2, ROUTE_2F, 0);
  placeRobot('robotR2', d.rear,   ROUTE_2F, 1);
  placeRobot('robotR3', d.floor1, ROUTE_1F, 0);
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
      function renderSteps(containerId, path, curStep) {
        const el = document.getElementById(containerId);
        el.innerHTML = '';
        path.forEach(p => {
          const sp = document.createElement('div');
          sp.className = 'step-box' + (curStep === p.step ? ' active' : curStep > p.step ? ' done' : '');
          sp.textContent = `${p.char} ${p.duration > 0 ? p.duration + 's' : '정지'}`;
          sp.title = `step${p.step}: 속도${p.speed} 방향${p.heading}°`;
          el.appendChild(sp);
        });
      }
      renderSteps('pathSteps2f', PATH_2F, d.floor2.step || 0);
      const pct2f = d.floor2.total_steps > 0
        ? Math.round((d.floor2.step || 0) / d.floor2.total_steps * 100) : 0;
      document.getElementById('pathProgress2f').style.width = pct2f + '%';
      document.getElementById('pathLabel2f').textContent = d.floor2.moving
        ? `경로 실행 중: ${d.floor2.step} / ${d.floor2.total_steps}`
        : ((d.floor2.step || 0) === 0 ? '대기 중' : '완료');

      // 1F path steps
      renderSteps('pathSteps', PATH_1F, d.floor1.step);
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
      renderSteps('pathStepsR2', PATH_R2, r2.step || 0);
      document.getElementById('pathLabelR2').textContent = r2.moving
        ? `R1 경로 추종 중: ${r2.step} / ${r2.total_steps}`
        : ((r2.step || 0) === 0 ? '대기 중' : '완료');

      updateFloorplan(d);
      renderFlow(d);

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
