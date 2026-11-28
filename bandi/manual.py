"""R2 수동 조작(원격) — 진입·해제, WASD 감시, 폰 음성 안내(TTS) 큐"""

import threading
import time

from .config import TARGET_MODES
from .hw import _halt, _pos, _raw, _set_led, _set_matrix
from .state import _state_lock, log_event, raise_exception, rt, state



MANUAL_TURN_DEG = 15           # A/D 한 번에 도는 각도

MANUAL_HOLD_SEC = 0.5          # W/S 신호가 이만큼 끊기면 정지 (버튼 뗌·통신 끊김 대비)

TTS_MAX_AGE    = 15.0          # 폰이 이보다 오래된 TTS는 무시

_manual_lock = threading.Lock()

_manual_last = {"ts": 0.0, "turn_ts": 0.0, "turn_from": None}
TURN_LOG_GAP_SEC = 0.8     # A/D를 떼고 이만큼 지나면 연속 회전을 한 줄로 기록


def manual_speed():
    """수동 조작 속도 = 주행 설정의 R2 속도 × 안내 대상 모드 배율 (자동 따라가기와 같은 계산)"""
    return min(255, int(state["speeds"]["rear"] * TARGET_MODES[state["target"]]["mps"]))


def _log_manual_drive(cmd, heading, reason=""):
    """수동 조작 주행 상태가 바뀔 때만 기록 (W/S는 누르는 동안 0.15초마다 오므로).
    정지할 때는 R2 위치도 남겨, 시나리오대로 조작했을 때 어디서 틀어지는지 비교할 수 있게"""
    word = {"w": "전진", "s": "후진", None: "정지"}[cmd]
    detail = f"방향 {heading}°"
    if cmd is None:
        # 좌표축 보정 전(자동 주행 없이 바로 수동 조작)이면 위치 센서 원래 값 — 보정값은 보통 0°라 거의 같다
        p = _pos("rear", rt.bolt_r2) or _raw("rear")
        if p is not None:
            detail += f", 위치 ({p[0]:.0f}, {p[1]:.0f})cm"
    else:
        detail += f", 속도 {manual_speed()}"
    log_event(f"  🎮 R2 {word} ({detail})" + (f" — {reason}" if reason else ""))


def _note_manual_turn(before):
    """A/D 회전 시작 방향을 기억 — 연속으로 누른 회전은 손을 뗀 뒤 한 줄로 기록"""
    if _manual_last["turn_from"] is None:
        _manual_last["turn_from"] = before
    _manual_last["turn_ts"] = time.time()


def _flush_manual_turn(force=False):
    start = _manual_last["turn_from"]
    if start is None or (not force and time.time() - _manual_last["turn_ts"] < TURN_LOG_GAP_SEC):
        return
    _manual_last["turn_from"] = None
    end = state["manual"]["heading"]
    delta = (end - start + 540) % 360 - 180
    if delta:
        log_event(f"  🎮 R2 {'우' if delta > 0 else '좌'}회전 {abs(delta)}° (방향 {start}° → {end}°)")

_tts = {"id": 0, "text": "", "ts": 0.0}

def _queue_tts(text, log=True):
    with _state_lock:
        # 앱은 마지막으로 읽은 id보다 큰 것만 읽음 → 브로커를 재시작해도 줄지 않게 시각(초) 기준으로 증가
        _tts["id"] = max(_tts["id"] + 1, int(time.time()))
        _tts["text"] = text
        _tts["ts"] = time.time()
    if log:
        log_event(f"🔊 R2 폰 음성 안내: {text}")

def _enter_manual(reason, auto=False):
    """R2 자동 주행을 멈추고 수동 조작으로 — 따라가기 루프는 manual.on을 보고 스스로 빠진다"""
    with _state_lock:
        if state["manual"]["on"]:
            return False
        state["manual"].update(on=True, reason=reason, cmd=None, since=time.time())
    _halt("rear", rt.bolt_r2, state["manual"]["heading"])
    _set_led(rt.bolt_r2, 255, 0, 255)
    _set_matrix(rt.bolt_r2, 255, 0, 255, "?")
    if auto:
        raise_exception("E2", reason)
        _queue_tts("안내 로봇과 연결이 끊겼습니다. 잠시 멈춰 주세요. 원격으로 안내합니다.")
    log_event(f"🎮 R2 수동 조작 전환 — {reason}")
    return True

def _exit_manual():
    with _state_lock:
        if not state["manual"]["on"]:
            return False
        state["manual"].update(on=False, cmd=None, reason="")
    _halt("rear", rt.bolt_r2, state["manual"]["heading"])
    _set_led(rt.bolt_r2, 255, 180, 0)
    _set_matrix(rt.bolt_r2, 255, 180, 0, "^")
    log_event("🎮 R2 수동 조작 해제 — 다음 주행부터 자동")
    return True

def _manual_watchdog():
    """W/S를 누르고 있는 동안만 달림 — 신호가 끊기면 세움"""
    while True:
        time.sleep(0.1)
        m = state["manual"]
        with _manual_lock:
            _flush_manual_turn()
            if m["on"] and m["cmd"] in ("w", "s") and time.time() - _manual_last["ts"] > MANUAL_HOLD_SEC:
                m["cmd"] = None
                _halt("rear", rt.bolt_r2, m["heading"])
                _log_manual_drive(None, m["heading"], "조작 신호 끊김")

MANUAL_MATRIX = {
    "up":    ("^", (0, 255, 0)),
    "left":  ("<", (0, 255, 0)),
    "right": (">", (0, 255, 0)),
    "down":  ("v", (0, 255, 0)),
    "stop":  ("X", (255, 0, 0)),
    "warn":  ("!", (255, 180, 0)),
    "help":  ("?", (255, 0, 255)),
    "off":   ("",  (0, 0, 0)),
}
