"""API — 테스트 경로, 방향 보정(수동·자동)"""

import threading
import time

from flask import Blueprint, jsonify, request as flask_request

from ..aim import (
    PHONE_HEADING_MAX_AGE,
    PHONE_MIN_QUALITY,
    _aim_heading,
    _auto_aim,
    _set_aim_led,
    _set_aim_status,
)
from ..config import DEPLOY_STILL_SEC, DEPLOY_THEN_TEST, TEST_ROUTE_M, _AIM_LABEL, _path_meters
from ..deploy import run_deploy_test
from ..hw import _idle_later, _set_led
from ..nav import _reset_origin
from ..scenario import _run_test_path, _test_paths
from ..state import (
    _active,
    _robots_in_motion,
    _save_settings,
    _state_lock,
    _test_running,
    _trigger_lock,
    log_event,
    rt,
    state,
)

bp = Blueprint("api_robot", __name__)


@bp.route("/api/test_route", methods=["POST"])
def api_test_route():
    """테스트 경로 — 연결된 로봇마다 제자리에서 직진 2m → 좌회전 1.5m. 상황 해제로 출발점 복귀"""
    keys = [k for k in ("floor2", "rear", "floor1") if _active(k)]
    if not keys:
        return jsonify({"ok": False, "error": "연결된 로봇 없음"}), 400
    if _robots_in_motion():
        return jsonify({"ok": False, "error": "주행·복귀 중 — 끝난 뒤 실행"}), 409
    with _trigger_lock:
        rt.run_gen += 1
        gen = rt.run_gen
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

@bp.route("/api/deploy_test", methods=["POST"])
def api_deploy_test():
    """수납함 출동 — 수납함 칸에서 ㄱ자로 나와 R1 앞·R2 뒤 배치. DEPLOY_THEN_TEST면 이어서 대피자 확인 → ㄱ자 테스트 경로"""
    keys = [k for k in ("floor2", "rear") if _active(k)]
    if not keys:
        return jsonify({"ok": False, "error": "R1·R2 미연결"}), 400
    if _robots_in_motion() or state["deploy"]["stage"]:
        return jsonify({"ok": False, "error": "주행·복귀·출동 중 — 끝난 뒤 실행"}), 409
    with _trigger_lock:
        rt.run_gen += 1
        gen = rt.run_gen
    with _state_lock:
        _test_running["on"] = True
        state["deploy"] = {"stage": "deploying", "still": 0.0}
    names = ", ".join(_AIM_LABEL[k] for k in keys)
    threading.Thread(target=run_deploy_test, args=(gen,), daemon=True).start()
    return jsonify({"ok": True, "robots": names, "still_sec": DEPLOY_STILL_SEC, "then_test": DEPLOY_THEN_TEST})

@bp.route("/api/follow/calibrate", methods=["POST"])
def api_follow_calibrate():
    """R2 방향 맞추기: R2가 지금 보는 방향을 heading 0°로 (R1과 같은 방향을 보게 둔 뒤 누른다)"""
    if not state["rear"]["connected"]:
        return jsonify({"ok": False, "error": "R2 미연결"}), 400
    try:
        rt.bolt_r2.drive_control.reset_heading()
        log_event("🧭 R2 heading 0° 리셋 (지금 정면 = 셀피 정면 기준)")
        return jsonify({"ok": True})
    except Exception as e:
        log_event(f"🧭 캘리브레이션 실패: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500

@bp.route("/api/aim", methods=["POST"])
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
    bolt = {"floor2": rt.bolt_2f, "rear": rt.bolt_r2, "floor1": rt.bolt_1f}[key]
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
            _idle_later(key)                 # ⓪ 완료 표시 잠깐 → 평소 대기(꺼짐)
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

@bp.route("/api/aim/auto", methods=["POST"])
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
