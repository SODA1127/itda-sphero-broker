"""8×8 매트릭스 화면 — 상황별 그림(LED 매트릭스 수정안 '화면 모양' 시트)과 깜빡임

그림은 로봇마다 연결 직후 애니메이션으로 한 번 올려 두고, 바꿀 때는 '재생' 명령 하나만 보낸다.
깜빡임·번갈아 보이기는 로봇이 스스로 돌리므로 주행 중 BLE 명령이 늘지 않는다.
애니메이션 명령이 안 되는 로봇(구형 펌웨어 등)은 글자 한 개 / 단색으로 대신 표시한다.

화면은 로봇마다 base(지금 단계의 기본 화면)와 overlay(잠깐 덮는 경고) 두 겹이다.
overlay가 있으면 overlay를, 없으면 base를 보여 준다. 비상 정지 중에는 비상 정지 화면만 보인다.
"""

import threading

from .config import MATRIX_ANIMATIONS, MATRIX_ROTATE_DEG
from .state import _state_lock, log_event, rt, state

# ── 그림 (위쪽 = 로봇이 가는 방향, '#' = 켜짐) ─────────────────────────────
SHAPES = {
    "flame": ["...#....", "...##...", "..###...", "..####..",
              ".######.", ".######.", ".######.", "..####.."],
    "wave":  ["........", "#...#...", ".#.#.#..", "..#...#.",
              "........", "#...#...", ".#.#.#..", "..#...#."],
    "bulb":  ["..####..", ".######.", ".######.", ".######.",
              "..####..", "...##...", "..####..", "...##..."],
    "arrow": ["...##...", "..####..", ".######.", "########",
              "..####..", "..####..", "..####..", "..####.."],
    "arrow_big": ["...##...", "..####..", ".######.", "########",
                  "########", ".######.", ".######.", ".######."],
    "face":  ["..####..", ".#....#.", "#.#..#.#", "#......#",
              "#.#..#.#", "#..##..#", ".#....#.", "..####.."],
    "right": ["....#...", "....##..", "#######.", "########",
              "########", "#######.", "....##..", "....#..."],
    "left":  ["...#....", "..##....", ".#######", "########",
              "########", ".#######", "..##....", "...#...."],
    "person": ["..####..", "..####..", "..####..", "..####..",
               "########", ".######.", "..####..", "...##..."],
    "stairs": ["...##...", "...##...", "...##...", "...##...",
               "##.##.##", ".######.", "..####..", "...##..."],
    "full":  ["########"] * 8,
}

GREEN, GREEN_MAX = (0, 170, 0), (0, 255, 0)
YELLOW, RED, ORANGE = (255, 170, 0), (255, 0, 0), (255, 70, 0)
WHITE, WHITE_DIM = (255, 255, 255), (90, 90, 90)

# 이름: (프레임 그림 목록 — None은 꺼진 프레임, 색, fps, 대신 표시할 글자 — None이면 단색)
PATTERNS = {
    "alarm_fire":  (["flame", None], RED, 2, None),          # ② 화재 경보 — 빨강 깜빡
    "alarm_quake": (["wave", None], ORANGE, 2, None),        # ② 지진 경보 — 주황 깜빡
    "alarm_black": (["bulb", None], WHITE, 2, None),         # ② 정전 경보 — 흰색 깜빡
    "guide":       (["arrow"], GREEN, 1, "^"),               # ⑤ 길 안내 (일반) — 켜둠
    "guide_blind": (["arrow", None], GREEN_MAX, 2, "^"),     # ⑤ 시각장애 — 최대 밝기, 1초 1번 깜빡
    "guide_big":   (["arrow_big"], GREEN, 1, "^"),           # ⑤ 노인 — 큰 화살표
    "guide_child": (["arrow", "arrow", "face"], GREEN, 1, "^"),  # ⑤ 어린이 — 화살표 2초 · 얼굴 1초
    "turn_right":  (["right"], GREEN, 1, ">"),               # ⑥ 돌기 전 미리
    "turn_left":   (["left"], GREEN, 1, "<"),
    "person":      (["person", None], YELLOW, 2, "v"),       # ⑩ 사람이 안 따라옴 — 노랑 깜빡
    "stairs":      (["stairs"], GREEN, 1, "v"),              # ⑫ 계단 앞 도착
    "exit":        (["full"], GREEN, 1, None),               # ⑭ 1층 비상구 도착 — 초록 전체
    "estop":       (["full"], RED, 1, None),                 # ⑬ 비상 정지 — 빨강 전체
    "return":      (["arrow"], WHITE_DIM, 1, "^"),           # ⑰ 평시 복귀
}
_ANIM_ID = {name: i for i, name in enumerate(PATTERNS)}

GUIDE_BY_TARGET = {"normal": "guide", "blind": "guide_blind", "elder": "guide_big", "child": "guide_child"}

# 대시보드 미리보기 버튼 — 수정안 '재난별 동작' 번호 순서
LABELS = {
    "alarm_fire":  ("② 화재 경보", "빨강 깜빡 1.5초"),
    "alarm_quake": ("② 지진 경보", "주황 깜빡 · 흔들리는 동안"),
    "alarm_black": ("② 정전 경보", "흰색 깜빡 1.5초"),
    "guide":       ("⑤ 길 안내 · 일반", "초록 켜둠"),
    "guide_blind": ("⑤ 길 안내 · 시각장애", "초록 최대 밝기 깜빡"),
    "guide_big":   ("⑤ 길 안내 · 노인", "큰 화살표"),
    "guide_child": ("⑤ 길 안내 · 어린이", "화살표 2초 · 얼굴 1초"),
    "turn_right":  ("⑥ 오른쪽으로 돌 때", "돌기 전 미리"),
    "turn_left":   ("⑥ 왼쪽으로 돌 때", "돌기 전 미리"),
    "person":      ("⑩ 사람이 안 따라옴", "노랑 깜빡 (R1)"),
    "stairs":      ("⑫ 계단 앞 도착", "초록 켜둠"),
    "estop":       ("⑬ 비상 정지", "빨강 전체"),
    "exit":        ("⑭ 1층 비상구 도착", "초록 전체 (R3)"),
    "return":      ("⑰ 평시 복귀", "흐린 흰색"),
}


def catalog():
    """대시보드가 그림을 그릴 수 있게 — 이름·설명·프레임(8줄 문자열, 꺼짐은 None)·색·fps"""
    return [{"name": n, "label": LABELS[n][0], "desc": LABELS[n][1],
             "frames": [SHAPES.get(s) for s in PATTERNS[n][0]],
             "color": "#%02x%02x%02x" % PATTERNS[n][1], "fps": PATTERNS[n][2]}
            for n in LABELS]

_lock = threading.Lock()
_layers = {}      # key → {"base": 이름|None, "overlay": 이름|None}
_shown = {}       # key → 마지막으로 보낸 화면 (같은 걸 다시 보내지 않음)
_uploaded = {}    # id(bolt) → True(애니메이션 사용) / False(글자 대체)


def _bolt(key):
    return {"floor2": rt.bolt_2f, "rear": rt.bolt_r2, "floor1": rt.bolt_1f}[key]


def _compress(shape):
    """8×8 → BOLT 64비트 프레임 (색 번호 4비트 평면 × 8줄) — spherov2 sphero_edu와 같은 방식"""
    grid = [[1 if ch == "#" else 0 for ch in row] for row in (shape or ["........"] * 8)]
    for _ in range((MATRIX_ROTATE_DEG // 90) % 4):          # 시계 방향 90°씩
        grid = [[grid[7 - c][r] for c in range(8)] for r in range(8)]
    out = []
    for bit in range(4):
        for r in range(7, -1, -1):
            v = 0
            for c in range(8):
                v |= ((grid[r][c] >> bit) & 1) << (7 - c)
            out.append(v)
    return out


def prepare(bolt):
    """연결 직후 한 번 — 모든 그림을 로봇에 애니메이션으로 저장. 실패하면 글자 대체로 표시"""
    if bolt is None or id(bolt) in _uploaded:
        return
    ok = False
    if MATRIX_ANIMATIONS and hasattr(bolt, "save_compressed_frame_player_animation"):
        try:
            bolt.delete_all_compressed_frame_player_animations_and_frames()
            frame_idx = 0
            for name, (frames, color, fps, _) in PATTERNS.items():
                idxs = []
                for shape_name in frames:
                    bolt.save_compressed_frame_player64_bit_frame(frame_idx, _compress(SHAPES.get(shape_name)))
                    idxs.append(frame_idx)
                    frame_idx += 1
                palette = [0, 0, 0, *color]           # 0 = 꺼짐, 1 = 그림 색
                bolt.save_compressed_frame_player_animation(_ANIM_ID[name], fps, False, palette, idxs)
            ok = True
        except Exception as e:
            log_event(f"  [Matrix] 그림 저장 실패 — 글자로 대신 표시: {e}")
    _uploaded[id(bolt)] = ok


def _send(bolt, name):
    """화면 하나를 실제로 보냄 (None = 끔)"""
    if bolt is None:
        return
    try:
        if name is None:
            if _uploaded.get(id(bolt)):
                bolt.reset_compressed_frame_player_animation()
            bolt.set_compressed_frame_player_one_color(0, 0, 0)
            return
        frames, color, _, char = PATTERNS[name]
        if id(bolt) not in _uploaded:
            prepare(bolt)
        if _uploaded.get(id(bolt)):
            bolt.play_compressed_frame_player_animation_with_loop_option(_ANIM_ID[name], True)
        elif char:
            bolt.set_compressed_frame_player_single_character(*color, char)
        else:
            bolt.set_compressed_frame_player_one_color(*color)
    except Exception as e:
        log_event(f"  [Matrix 오류] {e}")


def _refresh(key):
    lay = _layers.setdefault(key, {"base": None, "overlay": None})
    name = "estop" if state.get("estop") else (lay["overlay"] or lay["base"])
    if _shown.get(key, "?") == name:
        return
    _shown[key] = name
    with _state_lock:
        state.setdefault("matrix", {})[key] = name
    _send(_bolt(key), name)


def show(key, name):
    """기본 화면 바꾸기 (None = 끔)"""
    with _lock:
        _layers.setdefault(key, {"base": None, "overlay": None})["base"] = name
        _refresh(key)


def overlay(key, name):
    """잠깐 덮는 화면 (None = 걷어 내고 기본 화면으로)"""
    with _lock:
        _layers.setdefault(key, {"base": None, "overlay": None})["overlay"] = name
        _refresh(key)


def off(key):
    """⓪ 평소 대기 — 매트릭스 끔 (다른 코드가 직접 그렸을 수 있으므로 항상 다시 보냄)"""
    with _lock:
        _layers[key] = {"base": None, "overlay": None}
        _shown.pop(key, None)
        _refresh(key)


def redraw(key):
    """비상 정지 걸기·풀기처럼 화면 우선순위가 바뀌었을 때"""
    with _lock:
        _shown.pop(key, None)
        _refresh(key)


def invalidate(bolt):
    """다른 코드가 이 로봇 매트릭스에 직접 글자를 그림 — 다음 show는 같은 화면이라도 다시 보냄"""
    for key in ("floor2", "rear", "floor1"):
        if bolt is not None and _bolt(key) is bolt:
            _shown.pop(key, None)
            state.setdefault("matrix", {})[key] = "text"     # 글자 직접 표시 (R2 주행·수동 조작 등)


def forget(bolt):
    """연결 해제·재연결 — 새 로봇 객체에 그림을 다시 올리게"""
    _uploaded.pop(id(bolt), None)
    _shown.clear()


def guide_pattern():
    return GUIDE_BY_TARGET.get(state["target"], "guide")
