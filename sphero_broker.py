"""
반디(BANDI) — Sphero BOLT 재난 대피 안내 브로커 실행 파일
==========================================================
코드는 bandi/ 패키지, 대시보드 화면은 frontend/ 폴더에 있다.

실행:
    pip install spherov2 flask flask-sock   # 최초 1회
    python3.9 sphero_broker.py              # 실제 BLE 연결  (macOS: python3.9 권장)
    python3.9 sphero_broker.py --sim        # 시뮬레이션(Sphero 없이 대시보드 테스트)
    python3.9 sphero_broker.py --port 5002  # 포트 변경

대시보드: http://localhost:5001
트리거:   2F Sphero 세게 흔들기 (가속도 2.5G↑) 또는 대시보드 버튼
"""

from bandi.app import main

if __name__ == "__main__":
    main()
