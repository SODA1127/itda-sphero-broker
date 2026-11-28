"""외부 접속 보호 — Cloudflare 터널로 들어온 요청은 보기만, 비밀번호로 로그인하면 조작도

같은 와이파이(폰 앱·노트북)에서 직접 들어온 요청은 지금처럼 모두 허용한다.
외부 요청은 bandi Worker(cloudflare/)가 X-Bandi-Remote 헤더를 붙여 터널로 넘기므로 이것으로 구분한다
(터널에 공개 호스트 이름을 붙인 경우를 대비해 CF-Connecting-IP도 외부로 본다).
비밀번호는 환경 변수 BANDI_REMOTE_PASSWORD 또는 저장소 루트의 remote_password.txt (git 제외).
둘 다 없으면 외부는 보기 전용.
"""

import hmac
import os
import secrets
import threading
import time

from flask import Blueprint, jsonify, request, session

from ..config import ROOT

bp = Blueprint("auth", __name__)

PASSWORD_FILE = os.path.join(ROOT, "remote_password.txt")
SECRET_FILE = os.path.join(ROOT, ".flask_secret")
LOGIN_LIMIT = 5            # IP별 1분에 로그인 시도 허용 횟수
SAFE_METHODS = ("GET", "HEAD", "OPTIONS")
WS_PATHS = ("/api/follow/ws",)   # 웹소켓은 GET이지만 프레임을 써 넣음 → 외부는 로그인 필요
PRIVATE_GET = ("/api/logs/",)    # 내부 IP 등이 담길 수 있는 로그 — 외부는 로그인 필요

_attempts = {}
_attempts_lock = threading.Lock()


def secret_key():
    """세션 서명 키 — 재시작해도 로그인이 유지되게 파일에 보관"""
    try:
        with open(SECRET_FILE, encoding="utf-8") as f:
            key = f.read().strip()
        if key:
            return key
    except OSError:
        pass
    key = secrets.token_hex(32)
    try:
        with open(SECRET_FILE, "w", encoding="utf-8") as f:
            f.write(key)
        os.chmod(SECRET_FILE, 0o600)
    except OSError:
        pass
    return key


def remote_password():
    pw = os.environ.get("BANDI_REMOTE_PASSWORD", "").strip()
    if pw:
        return pw
    try:
        with open(PASSWORD_FILE, encoding="utf-8") as f:
            return f.read().strip()
    except OSError:
        return ""


def is_external():
    return request.headers.get("X-Bandi-Remote") == "1" or bool(request.headers.get("CF-Connecting-IP"))


def client_ip():
    return (request.headers.get("X-Bandi-Client-IP") or request.headers.get("CF-Connecting-IP")
            or request.remote_addr or "?")


def is_authed():
    return not is_external() or session.get("remote_ok") is True


def guard():
    """before_request — 외부에서 로그인 없이 바꾸는 요청은 막음"""
    if not is_external() or session.get("remote_ok") is True:
        return None
    if request.path.startswith("/api/auth"):
        return None
    if (request.method in SAFE_METHODS and request.path not in WS_PATHS
            and not request.path.startswith(PRIVATE_GET)):
        return None
    return jsonify({"ok": False, "error": "외부 접속은 보기 전용입니다. 조작하려면 비밀번호로 로그인하세요.",
                    "auth_required": True}), 401


@bp.route("/api/auth")
def api_auth():
    return jsonify({"external": is_external(), "authed": is_authed(),
                    "login_enabled": bool(remote_password())})


@bp.route("/api/auth/login", methods=["POST"])
def api_auth_login():
    pw = remote_password()
    if not pw:
        return jsonify({"ok": False, "error": "원격 조작 비밀번호가 설정되지 않았습니다 (보기 전용)."}), 403
    ip = client_ip()
    now = time.time()
    with _attempts_lock:
        recent = [t for t in _attempts.get(ip, []) if now - t < 60]
        if len(recent) >= LOGIN_LIMIT:
            return jsonify({"ok": False, "error": "시도가 너무 많습니다. 1분 뒤 다시 하세요."}), 429
        recent.append(now)
        _attempts[ip] = recent
    given = str((request.get_json(silent=True) or {}).get("password") or "")
    if not hmac.compare_digest(given.encode(), pw.encode()):
        return jsonify({"ok": False, "error": "비밀번호가 틀렸습니다."}), 403
    session.permanent = True
    session["remote_ok"] = True
    with _attempts_lock:
        _attempts.pop(ip, None)
    return jsonify({"ok": True})


@bp.route("/api/auth/logout", methods=["POST"])
def api_auth_logout():
    session.pop("remote_ok", None)
    return jsonify({"ok": True})
