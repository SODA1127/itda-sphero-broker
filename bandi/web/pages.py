"""페이지 — 대시보드(템플릿), 도면·로고 이미지"""

import os

from flask import Blueprint, current_app, render_template

from .. import matrix as _mx
from .auth import is_external
from ..config import (
    DEFAULT_R2_DELAY_SEC,
    DEFAULT_R2_DIST_PCT,
    DEFAULT_SPEEDS,
    DEFAULT_TURN_PAUSE_SEC,
    DISASTERS,
    DRIVE_MPS,
    EVAC_PATH_1F,
    EVAC_PATH_2F,
    EVAC_PATH_R2,
    EXCEPTIONS,
    FLOORPLAN_PDF,
    FOLLOW_GAP_M,
    ROOT,
    ROUTE_1F_M,
    ROUTE_2F_M,
    ROUTES_2F,
    SHAKE_THRESHOLD,
    SPEED_REF,
    TARGET_MODES,
)

bp = Blueprint("pages", __name__)


AIM_ROWS = [
    ("rear",   "R2", "경로 출발점 · 셀피 카메라가 진행 방향(첫 구간)을 보게"),
    ("floor2", "R1", "R2에서 경로 따라 간격(기본 400cm)만큼 앞 · 셀피 정면과 같은 방향"),
    ("floor1", "R3", "1층 출발점 · 1층 첫 구간(로비 쪽) 방향"),
]

def _frontend_config():
    """대시보드 JS가 window.BANDI로 받는 서버 설정값"""
    def steps(path):
        return [{"step": i + 1, "char": char, "speed": spd, "heading": hdg, "duration": dur,
                 "meters": round(dur * DRIVE_MPS, 2) if spd > 0 else 0}
                for i, (spd, hdg, dur, char) in enumerate(path)]
    return {
        "PATH_2F": steps(EVAC_PATH_2F), "PATH_1F": steps(EVAC_PATH_1F), "PATH_R2": steps(EVAC_PATH_R2),
        "TARGET_MODES": TARGET_MODES, "DISASTERS": DISASTERS, "EXCEPTIONS": EXCEPTIONS,
        "SHAKE_THRESHOLD": SHAKE_THRESHOLD, "SPEED_REF": SPEED_REF, "DEFAULT_SPEEDS": DEFAULT_SPEEDS,
        "DEFAULT_R2_DELAY_SEC": DEFAULT_R2_DELAY_SEC, "DEFAULT_TURN_PAUSE_SEC": DEFAULT_TURN_PAUSE_SEC,
        "DEFAULT_R2_DIST_PCT": DEFAULT_R2_DIST_PCT, "FOLLOW_GAP_M": FOLLOW_GAP_M,
        "ROUTE_2F_LEN_M": [m for _, m in ROUTE_2F_M], "ROUTE_1F_LEN_M": [m for _, m in ROUTE_1F_M],
        "MATRIX_PATTERNS": _mx.catalog(),
        "ROUTES_2F": {k: {"label": v["label"], "end": v["end"],
                          "legs": [{"heading": h, "meters": m} for h, m in v["route"]]}
                      for k, v in ROUTES_2F.items()},
    }

def _static_version():
    """정적 파일이 바뀌면 브라우저가 새로 받도록 ?v= 값"""
    newest = 0
    for base, _, files in os.walk(current_app.static_folder):
        for f in files:
            newest = max(newest, os.path.getmtime(os.path.join(base, f)))
    return int(newest)

@bp.route("/")
def dashboard():
    cfg = _frontend_config()
    # 외부(Worker) 접속은 요청 수를 줄이려 갱신을 느리게 — Workers 무료 한도 하루 10만 건
    cfg["POLL"] = ({"status": 2000, "follow": 1500, "snapshot": 1000} if is_external()
                   else {"status": 800, "follow": 500, "snapshot": 150})
    return render_template("index.html", cfg=cfg, aim_rows=AIM_ROWS, version=_static_version())

@bp.route("/floorplan")
def floorplan():
    from flask import send_file, Response
    try:
        return send_file(FLOORPLAN_PDF, mimetype="application/pdf")
    except FileNotFoundError:
        return Response("팔달관_도면.pdf 파일을 찾을 수 없습니다.", status=404)

BRAND_FILES = {"logo": "bandi_logo.png", "symbol": "bandi_symbol.png", "favicon": "favicon.png"}

@bp.route("/brand/<name>")
def brand_img(name):
    from flask import send_file, Response
    if name not in BRAND_FILES:
        return Response("not found", status=404)
    try:
        return send_file(os.path.join(ROOT, "brand", BRAND_FILES[name]),
                         mimetype="image/png", max_age=86400)
    except FileNotFoundError:
        return Response(f"brand/{BRAND_FILES[name]} 파일을 찾을 수 없습니다.", status=404)

@bp.route("/floorplan-img/<floor>")
def floorplan_img(floor):
    from flask import send_file, Response
    if floor not in ("1f", "2f"):
        return Response("not found", status=404)
    try:
        return send_file(os.path.join(ROOT, f"floorplan_{floor}.png"),
                         mimetype="image/png")
    except FileNotFoundError:
        return Response(f"floorplan_{floor}.png 파일을 찾을 수 없습니다.", status=404)
