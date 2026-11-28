"""설정 — 로봇 이름·포트·대피 경로·주행 기본값·안내 대상 모드·재난·예외 정의, 설정 파일 읽기"""

import json
import os
import tempfile


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # 저장소 루트

SPHERO_2F_NAME  = "SB-25D4"   # 2층 Sphero BLE 이름 (R1 선두)

SPHERO_1F_NAME  = "SB-D721"   # 1층 Sphero BLE 이름 (R3 대기)

SPHERO_R2_NAME  = "SB-3601"   # 2층 후방 Sphero BLE 이름 (R2, 스마트폰 트롤리)

FLASK_PORT      = 5001

FLOORPLAN_PDF   = os.path.join(ROOT, "팔달관_도면.pdf")

SHAKE_THRESHOLD = 2.5          # G  — 이 이상이면 흔들기 신호로 판정

SIGNAL_COOLDOWN = 10           # 초  — 한 번 신호 후 재차단 시간

# 구간 = (헤딩, 거리 m). 헤딩 0° = R2 스마트폰 셀피 카메라 정면 = 경로 첫 구간 방향.
# 90 = 우회전, 270 = 좌회전. 출발 전 대시보드 "방향 보정"으로 각 로봇 정면을 셀피 정면에 맞춰 0°로 저장.
# 속도 기본값 (0~255) — 대시보드 "주행 속도"에서 바꾸면 speed_settings.json에 저장돼 재시작 후에도 유지
DEFAULT_SPEEDS = {
    "floor2": 100,     # R1 (2층 선두)
    "rear":   150,     # R2 (2층 후방) — 스마트폰을 실어 무거우므로 더 세게. 실제 속도가 R1과 같아야 간격 유지
    "floor1": 100,     # R3 (1층)
}

DEFAULT_TURN_PAUSE_SEC = 1.0 # 꺾기 전 정지 시간 — 멈춘 동안 매트릭스에 꺾는 방향 화살표 (0~10초)

DEFAULT_R2_DIST_PCT = 100     # R2 거리 보정(%) — 폰 무게로 덜 가면 올림. 구간 시간에 곱해 같은 비율로 더 감 (50~200)

DEFAULT_R2_DELAY_SEC = 0.0   # R2(후방) 출발 지연 — 0이면 R1과 동시 출발해 R1이 지나간 경로를 4m 뒤에서 따라감 (0~30초)

SPEED_REF      = 100   # DRIVE_MPS를 잰 기준 속도 값

DRIVE_MPS      = 0.8   # R1이 SPEED_REF로 1초에 가는 거리(m) — 현장 바닥에서 실측 후 수정

FOLLOW_GAP_M   = 4.0   # R1(전방)–R2(후방) 간격 기본값 — 대시보드에서 바꿈. R2는 경로 출발점, R1은 이만큼 앞에서 출발

FOLLOW_END_GAP_M = 1.0 # 도착 때 R2가 R1 뒤에 남기는 간격 — R2는 출발 4m 차이를 따라잡아 R1 바로 뒤까지 간다

# 2층 대피 경로 3개 — 대시보드에서 고름. 셋 다 R1 출발점(오른쪽 계단 위 FH 앞, 정면 = 0°)에서 시작하는 R1 경로.
# R2는 R1 뒤 간격(4m)만큼 첫 구간을 거꾸로 늘인 곳에서 출발해 R1이 간 길을 따라감 (_full_route).
# 현장에서 R1을 속도 100으로 몰아 잰 이동 시간(초)을 m로 바꾼 값: 빨강 첫 구간 29초 = 47칸(14.1m) 실측 → 1초 ≈ 0.486m.
# end: "exit" = 2층 출구에서 끝(R3 안 넘김), "stairs" = 계단 앞에서 R3에 1층 경로를 넘김
_SEC_TO_M = 14.1 / 29
def _sec_route(*legs):
    return [(h, round(sec * _SEC_TO_M, 2)) for h, sec in legs]

ROUTES_2F = {
    "short":  {"label": "① 최단 — 2층 출구", "end": "exit",            # 보라
               "route": _sec_route((0, 6.5), (270, 15), (0, 10), (270, 15))},
    "void":   {"label": "② 보이드 돌아 — 2층 출구", "end": "exit",      # 빨강
               "route": _sec_route((0, 29), (270, 15), (180, 10.5), (270, 15))},
    "stairs": {"label": "③ 계단 — 1층(R3) 인수인계", "end": "stairs",   # 초록
               "route": _sec_route((0, 7), (270, 5))},
}

DEFAULT_ROUTE = "short"    # ① 최단 — 2층 출구

ROUTE_2F_M = ROUTES_2F[DEFAULT_ROUTE]["route"]     # 기본 경로 (대시보드 도면 표시용)

ROUTE_1F_M = [         # 1층: 137호 옆 계단(출발) → 로비 → 주출입구(EXIT)
    (0,   7.5),        # ① 25칸
    (90,  7.5),        # ② 25칸 — 우회전
]

_ARROW = {0: "^", 90: ">", 180: "v", 270: "<"}

def _route_len(key):
    return sum(m for _, m in ROUTES_2F[key]["route"])

def _full_route(key, gap):
    """R2 출발점 기준 전체 경로 — R1 경로 앞에 R2→R1 간격(첫 구간 방향 일직선)을 붙임"""
    legs = ROUTES_2F[key]["route"]
    h0, m0 = legs[0]
    if h0 == 0:
        return [(0, round(gap + m0, 2))] + legs[1:]
    return [(0, gap)] + legs


def _slice_route(route, start_m, end_m):
    """경로 중 start_m ~ end_m 구간만 (헤딩, m) 목록으로 잘라 낸다 (꺾이는 지점은 그대로)"""
    out, pos = [], 0.0
    for h, m in route:
        a, b = max(pos, start_m), min(pos + m, end_m)
        if b - a > 1e-6:
            out.append((h, round(b - a, 2)))
        pos += m
    return out

def _route_to_path(route, end_char, speed):
    """(헤딩, m) 구간 → (속도, 헤딩, 지속 초, 매트릭스 문자) 주행 단계. 시간은 R1 기준 속도로 환산"""
    path = [(speed, h, round(m / DRIVE_MPS, 2), _ARROW.get(h % 360, "^")) for h, m in route]
    return path + [(0, route[-1][0], 0.0, end_char)]

def _path_meters(path):
    return [round(dur * DRIVE_MPS, 2) if spd > 0 else 0.0 for spd, _, dur, _ in path]

ROUTE_2F_LEN = sum(m for _, m in ROUTE_2F_M)

# 주행 단계: (속도, 헤딩, 지속 초, 매트릭스 표시 문자)
# R1·R2는 같은 경로 위를 달려 같은 지점에서 꺾는다. R1은 간격만큼 앞에서, R2는 출발점에서 출발해 R1 정지점 1m 뒤까지 간다
# 지속 시간은 SPEED_REF 기준값 — 실제 주행 때 설정 속도에 맞춰 _run_path가 늘리거나 줄인다
def _build_follow_paths(route, gap):
    """R2 출발점 기준 경로(route)에서 R1은 gap만큼 앞에서, R2는 출발점에서 같은 경로를 따라간다.
    R2는 R1 정지점 min(1m, gap) 뒤까지 — R1이 꺾은 지점에서 똑같이 꺾는다"""
    length = sum(m for _, m in route)
    end_gap = min(FOLLOW_END_GAP_M, gap)
    r1 = _route_to_path(_slice_route(route, gap, length), "v", DEFAULT_SPEEDS["floor2"])
    r2 = _route_to_path(_slice_route(route, 0, length - end_gap), "v", DEFAULT_SPEEDS["rear"])
    return r1, r2

def _build_2f_paths(gap, key=DEFAULT_ROUTE):
    """간격(m)에 맞춘 2층 R1·R2 주행 단계 — R1은 FH 앞에서 고른 경로 전체, R2는 간격만큼 뒤에서 같은 길"""
    return _build_follow_paths(_full_route(key, gap), gap)

def _path_json(path):
    return [{"step": i + 1, "char": ch, "speed": spd, "heading": hdg, "duration": dur,
             "meters": round(dur * DRIVE_MPS, 2) if spd > 0 else 0}
            for i, (spd, hdg, dur, ch) in enumerate(path)]

GAP_MIN_M = 0.1                            # 10cm — 짧은 테스트용

GAP_MAX_M = 14.0                           # R2는 R1 뒤 위쪽 복도 일직선 위 — 이보다 멀리 둘 일 없음

EVAC_PATH_2F, EVAC_PATH_R2 = _build_2f_paths(FOLLOW_GAP_M)

EVAC_PATH_1F = _route_to_path(ROUTE_1F_M, "!", DEFAULT_SPEEDS["floor1"])    # 마지막: 정지 (비상구 도달)

SPEED_FILE = os.path.join(ROOT, "speed_settings.json")

def _load_speeds():
    speeds = dict(DEFAULT_SPEEDS)
    try:
        with open(SPEED_FILE, encoding="utf-8") as f:
            saved = json.load(f)
        for k in speeds:
            if isinstance(saved.get(k), int) and 1 <= saved[k] <= 255:
                speeds[k] = saved[k]
    except (OSError, ValueError):
        pass
    return speeds

def _load_aim_trim():
    """로봇별 자동 에임 보정값(도) — 자동 에임 뒤 수동 미세 조정한 만큼 누적"""
    trim = {"floor2": 0, "rear": 0, "floor1": 0}
    try:
        with open(SPEED_FILE, encoding="utf-8") as f:
            saved = json.load(f).get("aim_trim") or {}
        for k in trim:
            if isinstance(saved.get(k), (int, float)):
                trim[k] = int(saved[k])
    except (OSError, ValueError, AttributeError):
        pass
    return trim

def _load_route():
    """마지막으로 고른 2층 경로 — 재시작해도 유지"""
    try:
        with open(SPEED_FILE, encoding="utf-8") as f:
            v = json.load(f).get("route")
        if v in ROUTES_2F:
            return v
    except (OSError, ValueError, AttributeError):
        pass
    return DEFAULT_ROUTE

def _load_setting(key, default, lo, hi):
    try:
        with open(SPEED_FILE, encoding="utf-8") as f:
            v = json.load(f).get(key)
        if isinstance(v, (int, float)) and lo <= v <= hi:
            return float(v)
    except (OSError, ValueError, AttributeError):
        pass
    return default

# 안내 대상 모드 — mps 비율로 Sphero 속도를 낮추고 지속 시간을 늘려 같은 거리를 간다
TARGET_MODES = {
    "normal": {"label": "일반",     "icon": "🚶", "mps": 1.0, "led": "표준 화살표",     "voice": "기본 안내 1회"},
    "blind":  {"label": "시각장애", "icon": "🦯", "mps": 0.6, "led": "고휘도 단색 점멸", "voice": "상황·방향 반복 안내"},
    "elder":  {"label": "노인",     "icon": "🧓", "mps": 0.6, "led": "대형 화살표",     "voice": "주기적으로 또렷하게"},
    "child":  {"label": "어린이",   "icon": "🧒", "mps": 0.8, "led": "친근한 캐릭터",   "voice": "쉽고 짧은 문구"},
}

DISASTERS = {
    "fire":  {"label": "화재", "icon": "🔥", "src": "서버 신호",
              "note": "발화실 번호를 받는 즉시 시작 (시연: 대시보드 입력으로 수신반 신호 대체)"},
    "quake": {"label": "지진", "icon": "🌍", "src": "가속도계 감지",
              "note": "흔들리는 동안 음성 안내만 — 진동이 멈추고 2초 뒤 유도 시작"},
    "black": {"label": "정전", "icon": "💡", "src": "조도 센서 감지",
              "note": "조도 10 lux 미만 3초 지속 시 시작 — 네트워크 없이 저장 경로로 자립 유도"},
}

EXCEPTIONS = {
    "E1": "센서 오인식 — 3초 미만 조도 하강 무시",
    "E2": "경로 이탈 — IR 끊김·경로 벗어남·R1과 1m 넘게 뒤처짐·정체면 R2 수동 조작(원격) 전환",
    "E3": "장애물 충돌 — 0.3m 후진 후 회피 재시도",
    "E4": "통신 단절 — 저장 경로로 계속 유도",
    "E5": "로봇 고장 — 예비 로봇이 역할 넘겨받아 투입",
}

QUAKE_SETTLE_SEC = 2.0

# ── LED 매트릭스 수정안 (알고리즘/Sphero_LED 매트릭스_수정안.xlsx) ─────────────
MATRIX_ANIMATIONS = True   # False면 그림 대신 글자 한 개·단색으로만 표시 (현장에서 그림이 깨질 때)

MATRIX_ROTATE_DEG = 270    # 매트릭스 그림 회전 (시계 방향 0·90·180·270) — 실기에서 위쪽이 진행 방향이 되게 맞춤.
                           # 그림만 돌리고 글자 표시(R2 ^ 등)는 그대로. 바꾸면 로봇을 다시 연결해야 반영

ALARM_SEC        = 1.5     # ② 화재·정전 경보 화면 시간 → 끝나면 바로 출발 (트리거→첫 이동 ≤ 2.0초)

IDLE_AFTER_SEC   = 3.0     # ⓪ 연결 직후 'Floor N' 표시 뒤 이만큼 지나면 LED·매트릭스 끔

# ③ 첫 안내 음성 — 재난 × 안내 대상 (대상별 동작 시트). 지진은 (흔들릴 때, 멈춘 뒤)
VOICE_FIRST = {
    "fire": {
        "normal": "화재가 발생했습니다. 로봇을 따라 대피하세요.",
        "blind":  "화재가 발생했습니다. 로봇이 안내합니다. 앞으로 직진합니다.",
        "elder":  "불이 났습니다. 천천히 로봇을 따라오세요.",
        "child":  "불이 났어요! 로봇 친구를 따라가요!",
    },
    "quake": {
        "normal": ("지진입니다. 머리를 보호하세요.", "이제 로봇을 따라 이동하세요."),
        "blind":  ("지진입니다. 그 자리에서 머리를 감싸고 앉으세요.", "흔들림이 멈췄습니다. 앞으로 이동합니다."),
        "elder":  ("지진입니다. 머리를 보호하고 앉아 계세요.", "천천히 일어나서 따라오세요."),
        "child":  ("땅이 흔들려요! 머리를 감싸요!", "이제 로봇 친구 따라가요!"),
    },
    "black": {
        "normal": "정전입니다. 로봇의 화살표를 따라 대피하세요.",
        "blind":  "정전입니다. 안내를 따라 이동합니다. 앞으로 직진합니다.",
        "elder":  "불이 꺼졌습니다. 천천히 로봇 불빛을 따라오세요.",
        "child":  "깜깜해졌어요. 반짝이는 로봇을 따라가요!",
    },
}

VOICE_TURN   = {90: "오른쪽으로 돕니다.", 270: "왼쪽으로 돕니다."}   # ⑥ 화재 때만 (재난별 동작 시트)

VOICE_STAIRS = "계단 주의. 계단으로 내려가세요."      # ⑫

VOICE_EXIT   = "비상구입니다. 밖으로 나가세요."       # ⑭

EXCEPTION_SHOW_SEC = 4.0

SHAKE_QUIET_SEC  = 3.0  # 주행·복귀가 끝나고 이 시간 동안은 흔들기 무시

_ROBOT_KEYS = ("floor2", "rear", "floor1")

FRONTEND_DIR = os.path.join(ROOT, "frontend")
LOG_FILE = os.path.join(tempfile.gettempdir(), "bandi_broker.log")   # 재시작해도 남는 브로커 로그

TEST_ROUTE_M = [(0, 2.0), (270, 1.5)]   # 테스트: R1 기준 직진 2m → 좌회전 1.5m (ㄱ자). R2는 간격만큼 뒤에서 같은 경로

_AIM_LABEL = {"floor2": "R1", "rear": "R2", "floor1": "R3"}

# 수납함 출동 — 수납함 칸에서 바깥을 보고 선 R1·R2가 ㄱ자로 나와 R1 앞·R2 뒤로 한 줄 배치
DEPLOY_OUT_M     = 0.8    # 수납함에서 곧게 나오는 거리 (R1·R2 같음)
DEPLOY_R1_SIDE_M = 1.4    # R1이 (로봇 기준) 왼쪽으로 꺾은 뒤 가는 거리 — R2가 꺾어 뒤에 설 자리
DEPLOY_EXTRA_GAP_M = 0.3  # 대기 때 R2는 R1 뒤 (전방–후방 간격 + 이만큼) — 대피자가 설 자리
DEFAULT_DEPLOY_SLOT_CM = 35   # 수납함 R1 칸–R2 칸 중심 간격 (R2는 R1의 로봇 기준 오른쪽 칸)
DEPLOY_STILL_SEC = 3.0    # 대피자가 R1·R2 사이에서 이만큼 머물면(카메라 다가옴·정지·멀어짐) 경로 안내 시작
DEPLOY_START_TTS = "경로 안내를 시작합니다"
