"""API — 상태·연결·재난 트리거·안내 대상·예외·리셋·주행 설정·재시작"""

import threading
import time

from flask import Blueprint, jsonify, request as flask_request

from ..ble import _broker_main
from ..config import (
    DEFAULT_SPEEDS,
    DISASTERS,
    EXCEPTIONS,
    GAP_MAX_M,
    GAP_MIN_M,
    TARGET_MODES,
    _AIM_LABEL,
    _ROBOT_KEYS,
    _build_2f_paths,
    _path_json,
)
from ..hw import _bolt_of, _pos, _set_led
from ..manual import _tts
from ..nav import _return_home
from ..scenario import trigger_signal
from ..state import (
    _active,
    _broker_stop_evt,
    _lan_ip,
    _save_settings,
    _state_lock,
    _test_running,
    _trigger_lock,
    log_event,
    raise_exception,
    rt,
    state,
)

bp = Blueprint("api_system", __name__)


@bp.route("/api/status")
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
            "manual":        {k: state["manual"][k] for k in ("on", "reason", "heading", "cmd")},
            "ir":            state.get("ir", "off"),
            "tts":           {"id": _tts["id"], "text": _tts["text"]},
            "positions":     {k: (lambda p: {"x": round(p[0]), "y": round(p[1])} if p else None)(_pos(k, _bolt_of(k)))
                              for k in _ROBOT_KEYS},
            "paths":         dict(zip(("floor2", "rear"),
                                      map(_path_json, _build_2f_paths(state["follow_gap"])))),
        })

@bp.route("/api/connect", methods=["POST"])
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

    rt.bolt_2f = None
    rt.bolt_1f = None
    rt.bolt_r2 = None

    log_event(f"🔌 재연결 요청 — 2F: {name_2f}  1F: {name_1f}  R2: {name_r2}")
    t = threading.Thread(target=_broker_main, args=(False,), daemon=True)
    t.start()
    return jsonify({"ok": True})

@bp.route("/api/trigger", methods=["POST"])
def api_trigger():
    trigger_signal("대시보드-수동")
    return jsonify({"ok": True})

@bp.route("/api/disaster", methods=["POST"])
def api_disaster():
    kind = (flask_request.get_json(silent=True) or {}).get("type")
    if kind == "clear":
        api_reset()
        log_event("🟢 상황 해제 — 전 로봇 평시 복귀")
        gen = rt.run_gen
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

@bp.route("/api/target", methods=["POST"])
def api_target():
    mode = (flask_request.get_json(silent=True) or {}).get("mode")
    if mode not in TARGET_MODES:
        return jsonify({"ok": False, "error": "unknown mode"}), 400
    with _state_lock:
        state["target"] = mode
    m = TARGET_MODES[mode]
    log_event(f"🎯 안내 대상 → {m['icon']} {m['label']} ({m['mps']} m/s · {m['led']}) — 다음 주행부터 적용")
    return jsonify({"ok": True})

@bp.route("/api/exception", methods=["POST"])
def api_exception():
    eid = (flask_request.get_json(silent=True) or {}).get("id")
    if eid not in EXCEPTIONS:
        return jsonify({"ok": False, "error": "unknown id"}), 400
    raise_exception(eid, "대시보드 수동 발생")
    return jsonify({"ok": True})

@bp.route("/api/reset", methods=["POST"])
def api_reset():
    with _trigger_lock:
        rt.run_gen += 1
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
        rt.triggered = False
    _set_led(rt.bolt_2f, 0, 255, 0)
    _set_led(rt.bolt_1f, 0, 255, 0)
    _set_led(rt.bolt_r2, 0, 255, 0)
    log_event("🔄 수동 리셋")
    return jsonify({"ok": True})

@bp.route("/api/speeds", methods=["POST"])
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

@bp.route("/api/disconnect", methods=["POST"])
def api_disconnect():
    _broker_stop_evt.set()   # 하트비트 루프 종료 → with 블록 탈출 → BLE 해제
    log_event("🔌 연결 해제 요청")
    return jsonify({"ok": True})

@bp.route("/api/restart", methods=["POST"])
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
