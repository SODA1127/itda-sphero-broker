"""API — R2 카메라 앱 연동(감지·방향·프리뷰·스트림), 폰 방위"""

import queue as _queue
import time

from flask import Blueprint, jsonify, request as flask_request

from ..camera import (
    BLIND_BACK_TTS,
    BLIND_LOST_SEC,
    BLIND_LOST_TTS,
    BLIND_REPEAT_SEC,
    BLIND_REPEAT_TTS,
    _FOLLOW_INTERVAL,
    _follow,
    _follow_lock,
    _frame_lock,
    _jev_direction,
    _update_still,
    _mjpeg_clients,
    _mjpeg_clients_lock,
)
from .. import matrix as _mx
from ..manual import TTS_MAX_AGE, _queue_tts, _tts
from ..state import _state_lock, log_event, rt, state
from .server import sock

bp = Blueprint("api_camera", __name__)


@bp.route("/api/follow/toggle", methods=["POST"])
def api_follow_toggle():
    with _follow_lock:
        _follow["active"] = not _follow["active"]
        active = _follow["active"]
        if not active:
            _follow.update(fallen=False, fallen_since=None, direction="stop")
        _follow.update(person=False, seen=False, lost_since=None, lost_alert_ts=None,
                       still_since=None, still_bad_since=None)
    log_event(f"{'▶' if active else '⏹'} R2 카메라 감지 {'ON' if active else 'OFF'}")
    return jsonify({"ok": True, "active": active})

@bp.route("/api/follow/frame", methods=["POST"])
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

@bp.route("/api/phone/heading", methods=["POST"])
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
        tts = dict(_tts)
    resp = {"ok": True}
    if tts["id"] and time.time() - tts["ts"] < TTS_MAX_AGE:
        resp["tts"] = {"id": tts["id"], "text": tts["text"]}     # 폰이 새 id면 읽어 줌
    return jsonify(resp)

@bp.route("/api/follow/direction", methods=["POST"])
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
    person = bool(data.get("person", True))      # 옛 앱은 안 보냄 → 보이는 것으로

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
        # 시야 이탈 판단 — ⑩ 모든 안내 대상 (로봇은 계속 감)
        speak, back = None, False
        _follow["person"] = person
        _update_still(person, fallen, direction, now)
        if person:
            back = _follow["lost_alert_ts"] is not None
            _follow.update(seen=True, lost_since=None, lost_alert_ts=None)
        elif _follow["seen"]:
            _follow["lost_since"] = _follow["lost_since"] or now
            last = _follow["lost_alert_ts"]
            if now - _follow["lost_since"] >= BLIND_LOST_SEC:
                if last is None:
                    speak = BLIND_LOST_TTS
                elif now - last >= BLIND_REPEAT_SEC:
                    speak = BLIND_REPEAT_TTS
                if speak:
                    _follow["lost_alert_ts"] = now
        lost_for = now - _follow["lost_since"] if _follow["lost_since"] else 0

    if speak == BLIND_LOST_TTS:
        log_event(f"👀 R2 카메라: 대피자 시야 이탈 {lost_for:.0f}초 → 폰 음성 안내 + R1 ↓ 노랑")
        _queue_tts(speak)
    elif speak:
        _queue_tts(speak, log=False)
    elif back:
        log_event("👀 R2 카메라: 대피자 다시 보임 → 경로 복귀 안내")
        _queue_tts(BLIND_BACK_TTS)
    # R1 매트릭스: 이탈 안내 중이고 R1이 유도 중이면 ↓ 사람 쪽 화살표(노랑 깜빡), 아니면 걷어 냄
    lost_now = _follow["lost_alert_ts"] is not None
    _mx.overlay("floor2", "person" if lost_now and state["floor2"]["moving"] else None)

    if fallen and not prev_fallen:
        metrics = f"w/h={aspect}" + (f", 기울기 {tilt:.0f}°" if tilt is not None else "")
        log_event(f"🚨 R2 카메라: 대피자 넘어짐 감지 (3초 이상 · {metrics})")
    elif prev_fallen and not fallen:
        log_event("✅ R2 카메라: 넘어짐 해제")
    # 방향 변화는 초당 여러 번이라 로그에 남기지 않음 (주행 로그가 밀려남) — 대시보드 카메라 카드에 실시간 표시
    return jsonify({"direction": direction, "active": True})

@bp.route("/api/follow/preview", methods=["POST"])
def api_follow_preview():
    """Android 앱에서 JPEG 수신 → 대시보드 저장 + (JEV_KEY 있으면) 방향 판단"""
    jpeg = flask_request.data
    if not jpeg:
        return "", 204

    with _frame_lock:
        rt.latest_frame = jpeg
        rt.frame_ts = time.time()

    # MJPEG 클라이언트들에게 프레임 push
    with _mjpeg_clients_lock:
        for q in _mjpeg_clients:
            if q.full():
                try: q.get_nowait()
                except: pass
            try: q.put_nowait(jpeg)
            except: pass

    return "", 204

@bp.route("/api/follow/snapshot")
def api_follow_snapshot():
    """최신 카메라 프레임을 JPEG로 반환 — 대시보드 폴링용"""
    with _frame_lock:
        frame = rt.latest_frame
        ts    = rt.frame_ts
    if not frame:
        return "", 204
    from flask import Response
    resp = Response(frame, mimetype="image/jpeg")
    resp.headers["Cache-Control"] = "no-store"
    resp.headers["X-Frame-Ts"] = str(ts)
    return resp

@bp.route("/api/follow/stream")
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
                first = rt.latest_frame
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

@sock.route("/api/follow/ws", bp=bp)
def follow_ws(ws):
    """WebSocket 프리뷰 스트리밍 — 브라우저와 1:1 binary 연결"""
    q = _queue.Queue(maxsize=2)
    with _mjpeg_clients_lock:
        _mjpeg_clients.append(q)
    try:
        # 연결 직후 최신 프레임 즉시 전송
        with _frame_lock:
            first = rt.latest_frame
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

@bp.route("/api/follow/status")
def api_follow_status():
    now = time.time()
    with _follow_lock:
        d = dict(_follow)
    d["fallen_secs"] = round(now - d["fallen_since"], 1) if d["fallen_since"] else 0
    d["lost_secs"]   = round(now - d["lost_since"], 1) if d["lost_since"] else 0
    d["blind"]       = state["target"] == "blind"
    d["stale_secs"]  = round(now - d["last_ts"], 1) if d["last_ts"] else None
    with _frame_lock:
        d["preview_ts"] = rt.frame_ts
        d["has_preview"] = rt.latest_frame is not None
    return jsonify(d)
