# 잇다 (ITDA) — Sphero 재난 대피 안내 로봇 브로커

ISE103 융합시스템창의설계 팀 프로젝트 (4조 잇다) — Sphero 로봇 2대(R1·R2)와 Android 앱을 연결하는 Flask 브로커 서버.

## 구성

| 파일 | 설명 |
|------|------|
| `sphero_broker.py` | Flask 브로커 서버 (BLE 연결, 경로 주행, 카메라 감지, 대시보드) |
| `재난대피로봇_시연/index.html` | 웹 대시보드 (내장) |
| `동작 알고리즘 개선안 v2.md` | 알고리즘 설계 문서 |
| `Sphero 제약 시트.md` | Sphero BOLT 제약사항 정리 |

## 실행 방법

```bash
# 의존성 설치
pip3.9 install flask spherov2

# 브로커 실행 (포트 5001)
python3.9 sphero_broker.py

# 백그라운드 실행
nohup python3.9 sphero_broker.py > /tmp/broker.log 2>&1 &
```

## 주요 기능

- **R1 (2층 Sphero)**: 2층 대피 경로(EVAC_PATH_2F) 자동 주행
- **R2 (후방 Sphero)**: R1과 동시 출발, R2_GAP_SEC(기본 2초) 뒤 R1 경로 그대로 추종
- **R3 (1층 Sphero)**: 1층 대피 경로(EVAC_PATH_1F) 주행
- **카메라 감지**: R2 트롤리 Android 앱에서 대피자 움직임·넘어짐 감지 → 대시보드 표시
- **SIM 모드**: BLE 없이 `SIM=1 python3.9 sphero_broker.py` 로 테스트

## API 엔드포인트

| 메서드 | 경로 | 설명 |
|--------|------|------|
| `GET` | `/` | 웹 대시보드 |
| `GET` | `/api/status` | Sphero 연결·배터리·경로 상태 |
| `POST` | `/api/connect` | BLE 연결 시작 |
| `POST` | `/api/trigger` | 대피 경로 주행 시작 |
| `POST` | `/api/reset` | 상태 초기화 |
| `POST` | `/api/follow/toggle` | R2 카메라 감지 ON/OFF |
| `POST` | `/api/follow/direction` | 카메라 방향·넘어짐 수신 |
| `POST` | `/api/follow/preview` | JPEG 프리뷰 수신 |
| `POST` | `/api/follow/calibrate` | R2 heading 0° 리셋 |
