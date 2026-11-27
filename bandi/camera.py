"""R2 카메라 — 앱 감지 상태(넘어짐·시야 이탈), 프리뷰 프레임, (구) 비전 판단"""

import base64
import os
import threading

from .state import log_event


# 동네 프로젝트 등에서 사용하는 TypeSafe AI API 키
JEV_KEY       = os.environ.get("JEV_API_KEY", "").strip()

ANTHROPIC_KEY = os.environ.get("ANTHROPIC_API_KEY", "").strip()

_vision_busy  = threading.Lock()   # Vision 호출 중복 방지

_follow = {
    "active": False,
    "direction": "stop",
    "fallen": False,
    "fallen_since": None,   # 넘어짐 확정 시각 (epoch 초)
    "aspect": None,         # 앱이 보낸 사람 박스 가로/세로 (화면 픽셀 기준)
    "tilt": None,           # 앱이 보낸 상체 기울기 (도, 0 = 서 있음)
    "last_ts": 0.0,
    "person": False,        # 지금 카메라에 대피자가 보이는지
    "seen": False,          # 감지 켠 뒤 한 번이라도 보였는지 (처음부터 없으면 이탈로 안 봄)
    "lost_since": None,     # 안 보이기 시작한 시각
    "lost_alert_ts": None,  # 마지막 이탈 음성 안내 시각 (None = 안내 안 함)
}

# 시각장애 모드: R2 폰 카메라에서 앞사람이 사라지면 폰이 음성으로 경로 이탈을 알림
BLIND_LOST_SEC    = 2.0    # 이만큼 안 보이면 이탈

BLIND_REPEAT_SEC  = 8.0    # 돌아올 때까지 이 간격으로 다시 알림

BLIND_LOST_TTS    = "경로를 벗어났습니다. 그 자리에 멈춰 주세요. 안내 소리 쪽으로 천천히 돌아오세요."

BLIND_REPEAT_TTS  = "경로를 벗어났습니다. 멈춰 주세요."

BLIND_BACK_TTS    = "다시 경로에 들어왔습니다. 앞으로 계속 가세요."

_follow_lock = threading.Lock()

_FOLLOW_INTERVAL = 0.12  # 초 — 앱 100ms 주기에 맞춰 단축

_frame_lock = threading.Lock()

# MJPEG 스트리밍 클라이언트 큐 목록
_mjpeg_clients: list = []

_mjpeg_clients_lock = threading.Lock()

def _jev_direction(jpeg_bytes: bytes) -> str:
    """api.typesafe.ai (인증 필요) 로 사람 위치 → 방향 판단"""
    if not JEV_KEY:
        return "stop"
    import requests as _req
    img_b64 = base64.b64encode(jpeg_bytes).decode()
    payload = {
        "model": "imajev-4b",
        "images": [f"data:image/jpeg;base64,{img_b64}"],
        "state": "로봇의 전면 카메라 프레임(세로 방향 사진). 사람 추종 로봇이 다음에 어느 방향으로 움직여야 하는지 판단해주세요.",
        "questions": {
            "direction": {
                "type": "choice",
                "instructions": "사진 속 사람을 따라가려는 로봇이 어느 방향으로 움직여야 하나요?",
                "criteria": {
                    "forward": "사람이 화면 중앙(가로 1/3~2/3)에 있음",
                    "left":    "사람이 화면 왼쪽 1/3에 있음",
                    "right":   "사람이 화면 오른쪽 1/3에 있음",
                    "stop":    "사람이 없거나 너무 가까워 화면 절반 이상 차지",
                },
            }
        },
    }
    try:
        resp = _req.post(
            "https://api.typesafe.ai/v1/systemone",
            headers={"Authorization": f"Bearer {JEV_KEY}"},
            json=payload,
            timeout=5,
        )
        if resp.status_code != 200:
            log_event(f"⚠ Jev Vision HTTP {resp.status_code}: {resp.text[:80]}")
            return "stop"
        result    = resp.json()
        ans       = result.get("answers", {}).get("direction", {})
        direction = ans.get("choice", "stop")
        if ans.get("abstained", False):
            direction = "stop"
        return direction if direction in ("forward", "left", "right", "stop") else "stop"
    except Exception as e:
        log_event(f"⚠ Jev Vision 오류: {e}")
        return "stop"

def _claude_direction(jpeg_bytes: bytes) -> str:
    """Claude Haiku Vision으로 사람 위치 → 방향 판단"""
    if not ANTHROPIC_KEY:
        return "stop"
    import anthropic, base64 as _b64
    img_b64 = _b64.b64encode(jpeg_bytes).decode()
    try:
        # OAuth 토큰(sk-ant-oat01-)은 auth_token, API 키(sk-ant-api03-)는 api_key
        if ANTHROPIC_KEY.startswith("sk-ant-oat"):
            client = anthropic.Anthropic(auth_token=ANTHROPIC_KEY)
        else:
            client = anthropic.Anthropic(api_key=ANTHROPIC_KEY)
        resp = client.messages.create(
            model="claude-haiku-5-5",
            max_tokens=10,
            messages=[{
                "role": "user",
                "content": [
                    {
                        "type": "image",
                        "source": {"type": "base64", "media_type": "image/jpeg", "data": img_b64},
                    },
                    {
                        "type": "text",
                        "text": (
                            "로봇이 사람을 따라갑니다. 사진 속 사람 위치를 보고 "
                            "다음 중 하나만 답하세요(다른 말 금지):\n"
                            "forward - 사람이 화면 중앙(좌우 1/3~2/3)\n"
                            "left    - 사람이 화면 왼쪽 1/3\n"
                            "right   - 사람이 화면 오른쪽 1/3\n"
                            "stop    - 사람 없음 또는 너무 가까움\n"
                            "답:"
                        ),
                    },
                ],
            }],
        )
        text = next((b.text for b in resp.content if hasattr(b, "text")), "stop")
        direction = text.strip().lower().split()[0]
        return direction if direction in ("forward", "left", "right", "stop") else "stop"
    except Exception as e:
        log_event(f"⚠ Claude Vision 오류: {e}")
        return "stop"

def _judge_and_move(jpeg: bytes):
    """Jev Vision 방향 판단 → Sphero 이동 (백그라운드 스레드 전용)"""
    if not _vision_busy.acquire(blocking=False):
        return   # 이전 판단이 아직 진행 중 → 스킵
    try:
        direction = _jev_direction(jpeg)
        with _follow_lock:
            _follow["direction"] = direction
        log_event(f"👁 R2 카메라(Jev) → {direction}")
    finally:
        _vision_busy.release()
