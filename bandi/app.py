"""실행 진입점 — 로그 설정, BLE 스레드·수동 조작 감시 시작, 웹 서버 실행"""

import argparse
import logging
import threading

from .ble import _broker_main
from .config import FLASK_PORT, LOG_FILE
from .manual import _manual_watchdog
from .state import _state_lock, log_event, rt, state
from .web.server import create_app


def _setup_logging():
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
    try:    # 재시작해도 남는 로그 — 문제 진단용
        fh = logging.FileHandler(LOG_FILE, encoding="utf-8")
        fh.setFormatter(logging.Formatter('%(asctime)s %(message)s'))
        fh.addFilter(lambda r: r.name != "werkzeug")     # HTTP 요청 줄은 빼고
        logging.getLogger().addHandler(fh)
    except OSError:
        pass


def main(argv=None):
    parser = argparse.ArgumentParser(description="반디(BANDI) Sphero BOLT 브로커")
    parser.add_argument("--sim", action="store_true",
                        help="시뮬레이션 모드 (Sphero 없이 대시보드 테스트)")
    parser.add_argument("--port", type=int, default=FLASK_PORT)
    args = parser.parse_args(argv)

    _setup_logging()
    app = create_app()

    rt.port = args.port
    with _state_lock:
        state["sim_mode"] = args.sim

    log_event(f"🚀 Sphero PoC 브로커 시작 — 대시보드: http://localhost:{args.port}")
    if args.sim:
        log_event("💡 --sim 모드: 대시보드 버튼으로 신호 테스트 가능")

    threading.Thread(target=_manual_watchdog, daemon=True).start()
    threading.Thread(target=_broker_main, args=(args.sim,), daemon=True).start()

    app.run(host="0.0.0.0", port=args.port, debug=False, use_reloader=False, threaded=True)
