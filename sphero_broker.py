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

def trigger_signal(source: str = "unknown"):
    """2F Sphero 신호 수신 → 1F Sphero 대피 경로 시작"""
    global _triggered
    with _trigger_lock:
        if _triggered:
            log_event(f"⚠ 중복 신호 무시 ({source})")
            return
        _triggered = True

    with _state_lock:
        state["floor2"]["signal"] = True
        state["trigger_count"] += 1

    log_event(f"🚨 신호 수신 [{source}] → 1F Sphero 대피 경로 시작")

    # 2F LED: 빨강 (위험 알림) → 2F 경로 실행 → 완료 후 1F 자동 출발
    _set_led(_bolt_2f, 255, 0, 0)
    _set_led(_bolt_r2, 255, 0, 0)
    t = threading.Thread(target=_run_2f_then_1f, daemon=True)
    t.start()

    # cooldown 후 신호 리셋
    threading.Thread(target=_reset_signal_after, args=(SIGNAL_COOLDOWN,), daemon=True).start()

def _reset_signal_after(seconds: float):
    global _triggered
    time.sleep(seconds)
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

def _run_2f_then_1f():
    """2F 경로(R1 선두 + R2 후방 추종) 완료 → 1F 자동 출발 (연결된 쪽만 실행)"""
    r1_on = state["floor2"]["connected"]
    r2_on = state["rear"]["connected"]
    if r2_on:
        threading.Thread(
            target=_run_path,
            args=(_bolt_r2, _r2_path(EVAC_PATH_2F), "rear", "R2"),
            kwargs={"move_color": (255, 180, 0), "done_color": (255, 0, 0)},
            daemon=True,
        ).start()
    if r1_on or r2_on:
        if not r1_on:
            log_event("ℹ️  R1 미연결 — R2만 R1 경로를 따라 주행")
        _run_path(_bolt_2f, EVAC_PATH_2F, "floor2", "2F",
                  move_color=(255, 80, 0), done_color=(255, 0, 0))
        log_event("🏁 2F 핸드오프 완료 → 1F Sphero 출발")
    else:
        log_event("ℹ️  2F 미연결 — 1F 직접 출발")
    if state["floor1"]["connected"]:
        _run_path(_bolt_1f, EVAC_PATH_1F, "floor1", "1F",
                  move_color=(0, 100, 255), done_color=(0, 255, 0))

def _run_path(bolt, path, state_key, label, move_color, done_color):
    """범용 경로 실행 — 2F/1F 공통"""
    with _state_lock:
        state[state_key]["moving"] = True
        state[state_key]["step"]   = 0

    _set_led(bolt, *move_color)

    for i, (speed, heading, duration, char) in enumerate(path):
        with _state_lock:
            state[state_key]["step"] = i + 1

        _set_matrix(bolt, *move_color, char)

        if duration > 0:
            log_event(f"  → {label} step {i+1}/{len(path)}: "
                      f"속도={speed}  방향={heading}°  {duration}s  [{char}]")
            if bolt is not None:
                try:
                    bolt.drive_control.roll_start(heading, speed)
                except Exception as e:
                    log_event(f"  [이동 오류] {e}")
            time.sleep(duration)
        else:
            log_event(f"  → {label} 정지 (핸드오프 포인트)  [{char}]")
            if bolt is not None:
                try:
                    bolt.drive_control.roll_stop(heading)
                except Exception:
                    pass

    _set_led(bolt, *done_color)
    floor_num = "1" if state_key == "floor1" else "2"
    threading.Thread(target=_scroll_connected, args=(bolt, floor_num, *done_color), daemon=True).start()

    with _state_lock:
        state[state_key]["moving"] = False
        state[state_key]["step"]   = 0

    log_event(f"✅ {label} 경로 완료")

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
            trigger_signal(f"2F-shake({magnitude:.1f}G)")
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

@app.route("/api/status")
def api_status():
    with _state_lock:
        return jsonify({
            "floor2":        state["floor2"],
            "floor1":        state["floor1"],
            "rear":          state["rear"],
            "trigger_count": state["trigger_count"],
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

@app.route("/api/reset", methods=["POST"])
def api_reset():
    global _triggered
    with _state_lock:
        state["floor2"]["signal"] = False
        state["floor1"]["moving"] = False
        state["floor1"]["step"]   = 0
        state["rear"]["moving"]   = False
        state["rear"]["step"]     = 0
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

header { padding: 20px 24px 12px; border-bottom: 1px solid var(--bdr); display: flex; align-items: center; gap: 12px; }
header h1 { font-size: 18px; font-weight: 700; }
.sim-badge { background: #78350f; color: #fde68a; font-size: 11px; padding: 2px 8px; border-radius: 4px; display: none; }

.grid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; padding: 20px 24px; }
@media (max-width: 700px) { .grid { grid-template-columns: 1fr; } }

.card { background: var(--card); border: 1px solid var(--bdr); border-radius: 12px; padding: 20px; }
.card h2 { font-size: 13px; color: var(--dim); text-transform: uppercase; letter-spacing: .06em; margin-bottom: 14px; }

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
.btn-row { display: flex; gap: 10px; margin-top: 16px; flex-wrap: wrap; }
.btn { padding: 10px 20px; border-radius: 8px; border: none; cursor: pointer;
  font-size: 14px; font-weight: 600; transition: opacity .15s; }
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
</style>
</head>
<body>

<header>
  <span style="font-size:22px;">🤖</span>
  <h1>Sphero PoC 브로커 &nbsp;—&nbsp; 재난 대피 안내 로봇</h1>
  <span class="sim-badge" id="simBadge">SIM</span>
</header>

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
      <button class="btn btn-restart" onclick="restartBroker()">↺ 브로커 재시작</button>
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
