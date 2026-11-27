"""API — R2 수동 조작(주행·매트릭스·음성 안내)"""

import time

from flask import Blueprint, jsonify, request as flask_request

from ..aim import PHONE_HEADING_MAX_AGE
from ..follow import _set_ir
from ..hw import _drive, _drive_reverse, _halt, _set_matrix
from ..manual import (
    MANUAL_MATRIX,
    MANUAL_SPEED,
    MANUAL_TURN_DEG,
    _enter_manual,
    _exit_manual,
    _manual_last,
    _manual_lock,
    _queue_tts,
)
from ..state import _active, log_event, rt, state

bp = Blueprint("api_manual", __name__)


@bp.route("/api/manual", methods=["POST"])
def api_manual():
    """수동 조작 켜기/끄기 — {on: bool, reason?: str, sim?: bool(IR 끊김 흉내)}"""
    data = flask_request.get_json(silent=True) or {}
    if data.get("on"):
        if not _active("rear"):
            return jsonify({"ok": False, "error": "R2 연결 안 됨"}), 400
        if data.get("sim"):
            _set_ir("lost")
            _enter_manual("IR 끊김 (테스트)", auto=True)
        else:
            _enter_manual(str(data.get("reason") or "대시보드에서 전환"))
    else:
        _exit_manual()
        if state.get("ir") == "lost":
            _set_ir("off")
    return jsonify({"ok": True, "manual": state["manual"]["on"]})

@bp.route("/api/manual/drive", methods=["POST"])
def api_manual_drive():
    """WASD — w 전진 · s 후진 (누르는 동안 반복 전송) · a/d 15° 회전 · stop"""
    cmd = ((flask_request.get_json(silent=True) or {}).get("cmd") or "").lower()
    m = state["manual"]
    if not m["on"]:
        return jsonify({"ok": False, "error": "수동 조작 모드 아님"}), 409
    if cmd not in ("w", "a", "s", "d", "stop"):
        return jsonify({"ok": False, "error": "cmd는 w/a/s/d/stop"}), 400
    with _manual_lock:
        _manual_last["ts"] = time.time()
        if cmd in ("a", "d"):
            m["heading"] = int(m["heading"] + (MANUAL_TURN_DEG if cmd == "d" else -MANUAL_TURN_DEG)) % 360
            if m["cmd"] == "s":
                _drive_reverse("rear", rt.bolt_r2, m["heading"], MANUAL_SPEED)   # 후진 중이면 앞 방향만 바꿈
            else:                                              # 전진 중이면 방향만, 서 있으면 제자리 회전
                _drive("rear", rt.bolt_r2, m["heading"], MANUAL_SPEED if m["cmd"] == "w" else 0)
        elif cmd == "w":
            m["cmd"] = "w"
            _drive("rear", rt.bolt_r2, m["heading"], MANUAL_SPEED)
        elif cmd == "s":
            m["cmd"] = "s"
            _drive_reverse("rear", rt.bolt_r2, m["heading"], MANUAL_SPEED)   # 방향 그대로 뒤로
        else:
            m["cmd"] = None
            _halt("rear", rt.bolt_r2, m["heading"])
    return jsonify({"ok": True, "heading": m["heading"]})

@bp.route("/api/manual/matrix", methods=["POST"])
def api_manual_matrix():
    """R2 매트릭스 표시 — up/left/right/down/stop/warn/help/off"""
    name = (flask_request.get_json(silent=True) or {}).get("pattern")
    if name not in MANUAL_MATRIX:
        return jsonify({"ok": False, "error": "알 수 없는 표시"}), 400
    if not _active("rear"):
        return jsonify({"ok": False, "error": "R2 연결 안 됨"}), 400
    ch, (r, g, b) = MANUAL_MATRIX[name]
    _set_matrix(rt.bolt_r2, r, g, b, ch)
    log_event(f"  🎮 R2 매트릭스 '{ch or '끄기'}'")
    return jsonify({"ok": True})

@bp.route("/api/manual/tts", methods=["POST"])
def api_manual_tts():
    """R2 폰 음성 안내 — 폰이 0.5초마다 방위 보낼 때 응답으로 받아 읽음"""
    text = str((flask_request.get_json(silent=True) or {}).get("text") or "").strip()[:200]
    if not text:
        return jsonify({"ok": False, "error": "문구 없음"}), 400
    _queue_tts(text)
    phone = state.get("phone") or {}
    online = bool(phone) and time.time() - phone.get("ts", 0) < PHONE_HEADING_MAX_AGE
    return jsonify({"ok": True, "phone_online": online})
