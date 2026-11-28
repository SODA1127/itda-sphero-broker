"""API — R2 수동 조작(주행·매트릭스·음성 안내)"""

import time

from flask import Blueprint, jsonify, request as flask_request

from .. import matrix as _mx
from ..aim import PHONE_HEADING_MAX_AGE
from ..follow import _set_ir
from ..hw import _drive, _drive_reverse, _halt, _set_matrix
from ..manual import (
    MANUAL_MATRIX,
    MANUAL_TURN_DEG,
    _enter_manual,
    _exit_manual,
    _manual_last,
    _manual_lock,
    _manual_speed,
    _queue_tts,
)
from ..state import _active, _robots_in_motion, log_event, rt, state

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
    speed = _manual_speed()
    msg = None
    with _manual_lock:
        _manual_last["ts"] = time.time()
        prev = m["cmd"]
        if cmd in ("a", "d"):
            m["heading"] = int(m["heading"] + (MANUAL_TURN_DEG if cmd == "d" else -MANUAL_TURN_DEG)) % 360
            if m["cmd"] == "s":
                _drive_reverse("rear", rt.bolt_r2, m["heading"], speed)   # 후진 중이면 앞 방향만 바꿈
            else:                                              # 전진 중이면 방향만, 서 있으면 제자리 회전
                _drive("rear", rt.bolt_r2, m["heading"], speed if m["cmd"] == "w" else 0)
            msg = f"{'오른쪽' if cmd == 'd' else '왼쪽'} {MANUAL_TURN_DEG}° → 방향 {m['heading']}°"
        elif cmd == "w":
            m["cmd"] = "w"
            _drive("rear", rt.bolt_r2, m["heading"], speed)
            if prev != "w":                                     # 누르는 동안 반복 전송 — 시작할 때만 기록
                msg = f"전진 (방향 {m['heading']}° · 속도 {speed})"
        elif cmd == "s":
            m["cmd"] = "s"
            _drive_reverse("rear", rt.bolt_r2, m["heading"], speed)   # 방향 그대로 뒤로
            if prev != "s":
                msg = f"후진 (방향 {m['heading']}° · 속도 {speed})"
        else:
            m["cmd"] = None
            _halt("rear", rt.bolt_r2, m["heading"])
            if prev is not None:
                msg = f"정지 (방향 {m['heading']}°)"
    if msg:
        log_event(f"🎮 R2 {msg}")
    return jsonify({"ok": True, "heading": m["heading"], "speed": speed})

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

_ROBOT_LABEL = {"floor2": "R1", "rear": "R2", "floor1": "R3"}

@bp.route("/api/matrix/preview", methods=["POST"])
def api_matrix_preview():
    """상황별 매트릭스 미리보기 — {robots: ["floor2", ...], pattern: 이름 | null(끄기)}.
    주행·복귀·비상 정지 중에는 화면이 섞이므로 막는다. 다음 재난·주행이 시작되면 그 화면으로 바뀜"""
    data = flask_request.get_json(silent=True) or {}
    name = data.get("pattern")
    robots = [k for k in (data.get("robots") or []) if k in _ROBOT_LABEL]
    if name is not None and name not in _mx.PATTERNS:
        return jsonify({"ok": False, "error": "알 수 없는 화면"}), 400
    if not robots:
        return jsonify({"ok": False, "error": "로봇을 하나 이상 고르세요"}), 400
    if state.get("estop"):
        return jsonify({"ok": False, "error": "비상 정지 중 — 먼저 '리셋'"}), 400
    if _robots_in_motion():
        return jsonify({"ok": False, "error": "주행·복귀 중에는 미리보기 불가"}), 400
    on = [k for k in robots if _active(k)]
    if not on:
        return jsonify({"ok": False, "error": "고른 로봇이 연결 안 됨"}), 400
    for k in on:
        if name is None:
            _mx.off(k)
        else:
            _mx.show(k, name)
    label = _mx.LABELS[name][0] if name else "끄기"
    log_event(f"  🖼 매트릭스 미리보기 {label} → {', '.join(_ROBOT_LABEL[k] for k in on)}")
    return jsonify({"ok": True, "robots": on})
