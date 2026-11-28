// BANDI 대시보드 — 도면 위 로봇 위치 표시
function toggleFloorplan() {
  const wrap = document.getElementById('floorplanWrap');
  const btn  = document.getElementById('btnFloorplanToggle');
  const open = wrap.style.display === 'none';
  wrap.style.display = open ? 'block' : 'none';
  btn.textContent = open ? '▲ 접기' : '▼ 펼치기';
}
let fpManual = false, fpFloor = '2f';
function floorplanTab(floor, manual) {
  if (manual) fpManual = true;
  fpFloor = floor;
  ['2f','1f'].forEach(f => {
    document.getElementById('fpStage' + f).style.display = f === floor ? 'block' : 'none';
    const t = document.getElementById('fpTab' + f);
    t.style.background = f === floor ? 'var(--blu)' : 'var(--bdr)';
    t.style.color      = f === floor ? '#fff' : 'var(--txt)';
  });
  const wrap = document.getElementById('floorplanWrap');
  if (wrap.style.display === 'none') toggleFloorplan();
}

// 2층 도면(floorplan_2f.png, 1370×910 — 팔달관_도면.html 2층 SVG에서 예전 경로만 지워 렌더링) 위 경로 꺾임점.
// SVG 좌표 그대로 — 도면은 축척 없는 개략도라 m가 아니라 복도 모양을 따라 그림. 구간 수는 config.ROUTES_2F와 1:1
const FP_W = 1370, FP_H = 910;
// 셋 다 R1 출발점 = 오른쪽 계단 위 FH 앞(948, 315). R2는 그 뒤(오른쪽) 위쪽 복도에서 간격만큼 떨어져 대기
const FP_R1_START = [948, 315];
const FP_PX_PER_M = 21.5;                 // R2 대기 자리 그리기용 (① 첫 구간 68px ≈ 3.16m)
const FP_ROUTES_2F = {
  short:  [FP_R1_START, [880, 315], [880, 530], [750, 530], [750, 790]],   // ① 보이드·239 사이로 내려가 2층 출구
  void:   [FP_R1_START, [560, 315], [560, 530], [750, 530], [750, 790]],   // ② 보이드 왼쪽(214 옆)으로 돌아 2층 출구
  stairs: [FP_R1_START, [880, 315], [880, 385]],                           // ③ 오른쪽(남자화장실 쪽) 계단 앞
};
const toPct = pts => pts.map(([x, y]) => [x / FP_W * 100, y / FP_H * 100]);
const fpR2Start = gap => [Math.min(1300, FP_R1_START[0] + gap * FP_PX_PER_M), FP_R1_START[1]];
let ROUTE_KEY = 'short';
let ROUTE_2F = [], ROUTE_2F_LEN_M = [];
// 로봇 표시용 전체 경로 = R2 대기 자리 → R1 출발점(FH) → 고른 경로 (서버 config._full_route와 같은 모양)
function buildRoute2F(key, gap) {
  ROUTE_2F = toPct([fpR2Start(gap), ...FP_ROUTES_2F[key]]);
  ROUTE_2F_LEN_M = [gap, ...BANDI.ROUTES_2F[key].legs.map(l => l.meters)];
}
const ROUTE_1F = [[67.5,45.0],[56.1,45.0],[56.1,18.5]];
const ROUTE_1F_LEN_M = BANDI.ROUTE_1F_LEN_M;
let FOLLOW_GAP_M = BANDI.FOLLOW_GAP_M;

// 고른 경로를 도면 위에 그림 — 고른 것은 파랑 실선·화살표·구간 거리, 나머지는 회색 점선
function drawRoutes2F(key) {
  const svg = document.getElementById('fpRoutes2f');
  if (!svg || svg.dataset.key === key) return;
  svg.dataset.key = key;
  const R = BANDI.ROUTES_2F, pts = p => p.map(q => q.join(',')).join(' ');
  let html = '<defs><marker id="fpArr" viewBox="0 0 10 10" refX="6" refY="5" markerWidth="4" markerHeight="4" orient="auto">' +
             '<path d="M0,0 L10,5 L0,10 z" fill="#2563eb"/></marker></defs>';
  Object.entries(FP_ROUTES_2F).filter(([k]) => k !== key).forEach(([k, p]) =>
    html += `<polyline points="${pts(p)}" fill="none" stroke="#94a3b8" stroke-width="3" stroke-dasharray="8 7" opacity="0.7"/>`);
  const p = FP_ROUTES_2F[key], r = R[key];
  html += `<polyline points="${pts(p)}" fill="none" stroke="#2563eb" stroke-width="6" stroke-linejoin="round" ` +
          `stroke-linecap="round" opacity="0.92" marker-end="url(#fpArr)"/>`;
  r.legs.forEach((l, i) => {                       // 구간 거리 라벨
    const [x1, y1] = p[i], [x2, y2] = p[i + 1];
    const t = `${i + 1}구간 · ${l.meters.toFixed(1)}m`, w = t.length * 7.2 + 14;
    const short = Math.hypot(x2 - x1, y2 - y1) < w + 30;   // 짧은 구간은 R1 출발 표시와 안 겹치게 더 비켜서
    const mx = (x1 + x2) / 2 + (x1 === x2 ? 52 : 0) - (short && y1 === y2 ? w / 2 + 20 : 0);
    const my = (y1 + y2) / 2 + (y1 === y2 ? -16 : 0) + (short && x1 === x2 ? 0 : 0);
    html += `<rect x="${mx - w / 2}" y="${my - 11}" width="${w}" height="21" rx="4" fill="#fff" stroke="#2563eb" stroke-width="1.2"/>` +
            `<text x="${mx}" y="${my + 4}" font-size="12" font-weight="700" text-anchor="middle" fill="#2563eb">${t}</text>`;
  });
  const [sx, sy] = p[0], [ex, ey] = p[p.length - 1], [rx, ry] = fpR2Start(FOLLOW_GAP_M);
  html += `<line x1="${rx}" y1="${ry}" x2="${sx}" y2="${sy}" stroke="#f59e0b" stroke-width="3" stroke-dasharray="6 6"/>` +
          `<circle cx="${sx}" cy="${sy}" r="9" fill="#fff" stroke="#2563eb" stroke-width="3"/>` +   // R1 출발(FH 앞) — 이름은 로봇 표시가 보여 줌
          `<text x="${ex - 14}" y="${ey + (r.end === 'stairs' ? 26 : -6)}" font-size="12" font-weight="700" ` +
          `text-anchor="end" fill="#2563eb">${r.end === 'stairs' ? '계단 → R3(1층)' : '2층 출구'}</text>`;
  const total = r.legs.reduce((a, l) => a + l.meters, 0);
  html += `<line x1="745" y1="28" x2="775" y2="28" stroke="#2563eb" stroke-width="5" stroke-linecap="round"/>` +
          `<text x="783" y="33" font-size="13" fill="#444">대피 경로 <tspan font-weight="700" fill="#2563eb">${r.label} · ${total.toFixed(1)}m</tspan></text>`;
  svg.innerHTML = html;
}
function setRoute2F(key) {
  if (!FP_ROUTES_2F[key] || !BANDI.ROUTES_2F[key]) return;
  ROUTE_KEY = key;
  drawRoutes2F(key);
}
// 경로 출발점에서 m미터 지점의 도면 좌표 — 구간(꺾임점) 단위로 도면 실측 거리를 따라감
function pointAtMeters(route, lens, m) {
  m = Math.max(0, m);
  for (let i = 0; i < lens.length; i++) {
    if (m <= lens[i] || i === lens.length - 1) {
      const r = lens[i] ? Math.min(1, m / lens[i]) : 0;
      return [route[i][0] + (route[i+1][0] - route[i][0]) * r,
              route[i][1] + (route[i+1][1] - route[i][1]) * r];
    }
    m -= lens[i];
  }
}
let modeFactor = 1;
// offsetM: 이 로봇이 경로 출발점에서 몇 m 앞에서 출발하는지 (R1 = 4m, R2·R3 = 0)
let testRunning = false;
function placeRobot(id, s, route, lens, path, offsetM, timeScale) {
  const el = document.getElementById(id);
  let m = offsetM, glide = 1.4, cls = 'idle', word = '대기';
  if (!s.connected && !simMode && !s.moving) { cls = ''; word = '미연결'; }
  if (s.moving && s.progress_m != null) {
    m = s.progress_m; glide = 0.3; cls = 'moving'; word = '따라가기';
    if (id === 'robotR1') word = '주행';
  } else if (testRunning && s.moving) {
    cls = 'moving'; word = '테스트';
  } else if (s.returning) {
    glide = s.return_secs || 1.4; cls = 'moving'; word = '복귀';
  } else if (s.moving) {
    const k = s.step || 0;
    m = offsetM + path.slice(0, k).reduce((a, p) => a + (p.meters || 0), 0);   // 현재 구간 끝 지점
    const cur = path[k - 1];
    if (cur && cur.duration > 0) glide = cur.duration * timeScale / modeFactor;   // 실제 주행 시간만큼 이동
    cls = 'moving'; word = '주행';
  } else if (s.done) {
    m = offsetM + path.reduce((a, p) => a + (p.meters || 0), 0);
    cls = 'done'; word = '도착';
  }
  const [x, y] = pointAtMeters(route, lens, m);
  if (el.dataset.m !== String(m)) {
    el.style.transition = `left ${glide}s linear, top ${glide}s linear`;
    el.dataset.m = String(m);
  }
  el.style.left = x + '%';
  el.style.top  = y + '%';
  el.className  = 'robot ' + cls + (id === 'robotR2' ? ' rb-rear' : '');
  el.querySelector('.rb-tag').textContent = el.querySelector('.rb-dot').textContent + ' ' + word;
}
let simMode = false;
let followFallen = false;
function updateFloorplan(d) {
  simMode = !!d.sim_mode;
  if (d.route && d.route !== ROUTE_KEY) setRoute2F(d.route);
  const gapKey = ROUTE_KEY + '@' + FOLLOW_GAP_M;            // 간격이 바뀌면 R2 대기 자리도 다시 그림
  if (document.getElementById('fpRoutes2f').dataset.key !== gapKey) {
    document.getElementById('fpRoutes2f').dataset.key = '';
    drawRoutes2F(ROUTE_KEY);
    document.getElementById('fpRoutes2f').dataset.key = gapKey;
  }
  buildRoute2F(ROUTE_KEY, FOLLOW_GAP_M);
  modeFactor = (TARGET_MODES[d.target] || {}).mps || 1;
  const sp = d.speeds || DEFAULT_SPEEDS;
  // R1은 R2 대기 자리에서 간격만큼 앞(FH 앞)에서 출발
  placeRobot('robotR1', d.floor2, ROUTE_2F, ROUTE_2F_LEN_M, PATH_2F, FOLLOW_GAP_M, SPEED_REF / sp.floor2);
  // 따라가기 중 R1 진행 거리는 R2 출발점 기준 — offset 없이 그대로
  placeRobot('robotR2', d.rear,   ROUTE_2F, ROUTE_2F_LEN_M, PATH_R2, 0, SPEED_REF / sp.rear * (d.r2_dist_pct || 100) / 100);
  placeRobot('robotR3', d.floor1, ROUTE_1F, ROUTE_1F_LEN_M, PATH_1F, 0, SPEED_REF / sp.floor1);
  if (followFallen) {
    const r2 = document.getElementById('robotR2');
    r2.classList.add('fallen');
    r2.querySelector('.rb-tag').textContent = 'R2 ⚠ 넘어짐 감지';
  }
  // 주행 중인 층을 자동으로 보여줌 (사용자가 탭을 누르면 자동 전환 중단)
  if (!fpManual) {
    if (d.floor1.moving && fpFloor !== '1f') floorplanTab('1f', false);
    else if ((d.floor2.moving || d.rear.moving) && fpFloor !== '2f') floorplanTab('2f', false);
  }
}
