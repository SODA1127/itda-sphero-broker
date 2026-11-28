"""BLE 연결 — Sphero 연결·센서 스트리밍·하트비트, R1 흔들기(지진) 감지"""

import threading
import time

from .config import SHAKE_QUIET_SEC, SHAKE_THRESHOLD, _AIM_LABEL
from . import matrix as _mx
from .hw import LOC_INTERVAL_MS, _is_real, _make_loc_listener, _scroll_connected, _set_led
from .nav import _clear_trail, _reset_origin
from .scenario import trigger_signal
from .state import _broker_stop_evt, _robots_in_motion, _state_lock, log_event, rt, state


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
            if _robots_in_motion() or time.time() - rt.last_motion_end < SHAKE_QUIET_SEC:
                return   # 로봇 자신의 주행·회전·충돌 — 지진 아님
            rt.last_shake_ts = time.time()
            trigger_signal(f"2F-shake({magnitude:.1f}G)", "quake")
    except Exception:
        pass

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

        rt.bolt_2f = _Dummy()
        rt.bolt_1f = _Dummy()
        rt.bolt_r2 = _Dummy()
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
        rt.bolt_2f, rt.bolt_1f, rt.bolt_r2 = b2, b1, br

        # wake → 매트릭스 그림 저장 → 초기 LED: 녹색 + 매트릭스 "Floor N" 스크롤 → 잠시 뒤 끔(⓪ 평소 대기)
        for bolt, floor, num, key in [(b2, "2F", "2", "floor2"), (b1, "1F", "1", "floor1"), (br, "R2", "2", "rear")]:
            try:
                bolt.wake()
            except Exception:
                pass
            if _is_real(bolt):
                _mx.forget(bolt)
                _mx.prepare(bolt)
            _set_led(bolt, 0, 255, 0)
            threading.Thread(
                target=_scroll_connected, args=(bolt, num, 0, 255, 0, key), daemon=True
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
    rt.bolt_2f = None
    rt.bolt_1f = None
    rt.bolt_r2 = None
    with _state_lock:
        state["floor2"]["connected"] = False
        state["floor1"]["connected"] = False
        state["rear"]["connected"]   = False
    log_event("🔌 Sphero 연결 해제 완료")
