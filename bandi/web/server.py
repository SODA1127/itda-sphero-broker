"""Flask 앱 조립 — 프론트엔드(frontend/) 경로 지정, Blueprint·웹소켓 등록"""

from flask import Flask
from flask_sock import Sock

from ..config import FRONTEND_DIR

sock = Sock()


def create_app():
    app = Flask(__name__, template_folder=f"{FRONTEND_DIR}/templates", static_folder=f"{FRONTEND_DIR}/static")
    from . import api_camera, api_manual, api_robot, api_system, pages
    for mod in (pages, api_system, api_robot, api_manual, api_camera):
        app.register_blueprint(mod.bp)
    sock.init_app(app)
    return app
