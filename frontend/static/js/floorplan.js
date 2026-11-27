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

// 도면 이미지 기준 % 좌표 (도면 PNG의 파란 스피로 경로 꺾임점)
const ROUTE_2F = [[64.1,60.8],[54.6,60.8],[54.6,89.6]];   // ROUTE_2F_M 구간과 1:1 (전체: [66.1,44.7],[64.1,45.3] 앞에 추가)
const ROUTE_1F = [[67.5,45.0],[56.1,45.0],[56.1,18.5]];
const ROUTE_2F_LEN_M = BANDI.ROUTE_2F_LEN_M;
const ROUTE_1F_LEN_M = BANDI.ROUTE_1F_LEN_M;
let FOLLOW_GAP_M = BANDI.FOLLOW_GAP_M;
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
  modeFactor = (TARGET_MODES[d.target] || {}).mps || 1;
  const sp = d.speeds || DEFAULT_SPEEDS;
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
