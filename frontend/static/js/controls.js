// BANDI 대시보드 — 재난 트리거·테스트 경로·안내 대상·예외·대피 흐름 표시
function testRoute() {
  fetch('/api/test_route', { method: 'POST' })
    .then(r => r.json())
    .then(j => {
      const note = document.getElementById('triggerNote');
      note.innerHTML = j.ok
        ? `<b>🧪 테스트 경로</b> — ${j.robots} R1 직진 1m → 좌회전 1.5m, R2는 간격만큼 뒤에서 R1이 꺾은 곳에서 꺾음 (이벤트 로그에 구간 표시). 끝나면 '상황 해제'로 출발점 복귀`
        : '⚠ ' + j.error;
      update();
    })
    .catch(console.error);
}
function setTarget(mode)    { postJSON('/api/target', { mode }); }

// 2층 대피 경로 — ① 최단 · ② 보이드 돌아 · ③ 계단(R3)
const ROUTES_2F = BANDI.ROUTES_2F;
const ROUTE_DIR = { 0: '직진', 90: '우회전', 180: '뒤로', 270: '좌회전' };
let routeMsg = '';
document.getElementById('routeSeg').innerHTML = Object.entries(ROUTES_2F)
  .sort((a, b) => a[1].label.localeCompare(b[1].label))   // ①②③ 순서 (서버가 키를 알파벳순으로 보냄)
  .map(([k, r]) => {
  const total = r.legs.reduce((a, l) => a + l.meters, 0);
  const legs = r.legs.map(l => `${ROUTE_DIR[l.heading] ?? l.heading + '°'} ${l.meters.toFixed(1)}m`).join(' → ');
  return `<button id="route_${k}" onclick="setRoute('${k}')"><b>${r.label}</b>` +
         `<span class="spd">${legs} · 총 ${total.toFixed(1)}m</span></button>`;
}).join('');
function setRoute(route) {
  fetch('/api/route', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ route }) })
    .then(r => r.json())
    .then(j => { routeMsg = j.ok ? '' : '⚠ ' + j.error; update(); })
    .catch(console.error);
}
function renderRoute(d) {
  Object.keys(ROUTES_2F).forEach(k =>
    document.getElementById('route_' + k).classList.toggle('on', d.route === k));
  const r = ROUTES_2F[d.route];
  document.getElementById('routeNote').innerHTML = routeMsg || (r ? (r.end === 'stairs'
    ? `<b>${r.label}</b> — 계단 앞에서 멈추고 R3가 1층 경로로 이어서 안내`
    : `<b>${r.label}</b> — 2층 출구에서 끝 (R3는 출발 안 함)`) : '');
}
function fireException(id)  { postJSON('/api/exception', { id }); }

document.getElementById('seg').innerHTML = Object.entries(TARGET_MODES).map(([k, m]) =>
  `<button id="seg_${k}" onclick="setTarget('${k}')"><span class="ico">${m.icon}</span>${m.label}` +
  `<span class="spd">${m.mps} m/s · ${m.led}</span></button>`).join('');
document.getElementById('excepts').innerHTML = Object.entries(EXCEPTIONS).map(([k, t]) =>
  `<button class="ex" id="ex_${k}" onclick="fireException('${k}')"><span class="eid">${k}</span>` +
  `<span class="etxt">${t}</span><span class="est"></span></button>`).join('');

document.addEventListener('keydown', e => {
  if (['INPUT', 'TEXTAREA'].includes(e.target.tagName) || e.metaKey || e.ctrlKey || e.altKey || e.repeat) return;
  const m = { '1': 'fire', '2': 'quake', '3': 'black', '0': 'clear' }[e.key];
  if (m) sendDisaster(m);
});

function robotPhase(r, d) {
  const ph = d.phase;
  if (!r.key) return d.exceptions.E5 === 'triggered' ? '투입 요청' : '예비 대기';
  const s = d[r.key];
  if (s.returning) return '복귀 중';
  if (r.key === 'floor1') {
    if (ph === 'idle') return 'IDLE';
    if (ph === 'armed' || ph === 'guiding') return 'ARMED';
    if (ph === 'handoff') return s.moving ? 'GUIDING' : 'HANDOFF';
    return 'COMPLETE';
  }
  if (ph === 'idle' || ph === 'armed') return ph.toUpperCase();
  if (ph === 'guiding') return s.moving || !s.done ? 'GUIDING' : 'HANDOFF';
  return ph === 'handoff' ? 'HANDOFF' : 'COMPLETE';
}

function renderFlow(d) {
  const ph = d.phase || 'idle', idx = PHASES.indexOf(ph);
  const pill = document.getElementById('statePill');
  pill.className = 'state-pill ' + ph;
  pill.textContent = ph.toUpperCase();
  PHASES.forEach((_, i) => {
    document.getElementById('tn' + i).className = 'tnode' + (i === idx ? ' active' : i < idx ? ' done' : '');
    if (i < 4) document.getElementById('ta' + i).className = 'tarrow' + (i < idx ? ' on' : '');
  });

  const dz = d.disaster ? DISASTERS[d.disaster] : null;
  const tm = TARGET_MODES[d.target];
  document.getElementById('metaDisaster').textContent = dz ? `${dz.icon} ${dz.label}` : '없음';
  document.getElementById('metaTarget').textContent   = `${tm.icon} ${tm.label} · ${tm.mps} m/s`;
  document.getElementById('metaLatency').textContent  = d.latency == null ? '—'
    : d.latency.toFixed(1) + 's' + (d.disaster === 'quake' ? ' (진동 종료 대기 포함)' : '');

  ['fire', 'quake', 'black'].forEach(k =>
    document.getElementById('dz' + k).classList.toggle('on', d.disaster === k));
  document.getElementById('dzestop').classList.toggle('on', !!d.estop);
  document.getElementById('triggerNote').innerHTML = dz
    ? `<b>${dz.label}</b> — ${dz.note}`
    : '평시 — 로봇 연결 상태(하트비트) 확인 중. 버튼이나 키보드 1·2·3으로 재난 상황을 발생시키세요.';

  Object.keys(TARGET_MODES).forEach(k =>
    document.getElementById('seg_' + k).classList.toggle('on', d.target === k));
  renderRoute(d);
  document.getElementById('targetNote').innerHTML =
    `<b>${tm.label}</b> — ${tm.led}, ${tm.mps} m/s. 음성: ${tm.voice}. ` +
    `Sphero 속도 ×${tm.mps}, 구간 시간 ÷${tm.mps}로 같은 거리 유지 (다음 주행부터 적용).`;

  document.getElementById('bots').innerHTML = ROBOTS.map(r => {
    const fall = r.id === 'R2' && followFallen;
    const st = fall ? '대피자 넘어짐' : robotPhase(r, d);
    const online = r.key && (d[r.key].connected || d.sim_mode);
    const conn = !r.key ? '하드웨어 없음' : d[r.key].connected ? '연결됨' : d.sim_mode ? 'SIM' : '미연결';
    const lit = online && st !== 'IDLE';
    const ballStyle = lit ? `color:${r.color};box-shadow:inset 0 0 0 1px ${r.color}` : online ? 'color:#6b7785' : '';
    return `<div class="bot ${fall ? 'fall' : st.toLowerCase()}">
      <div class="rball" style="${ballStyle}">${r.id}</div>
      <div style="min-width:0"><div class="n">${r.name} <span>· ${conn}${(r.key && d.positions && d.positions[r.key]) ? ` · 📍 ${d.positions[r.key].x}, ${d.positions[r.key].y}cm` : ''}</span></div><div class="s">${r.role}</div></div>
      <div class="st">${st}</div></div>`;
  }).join('');

  const anyConn = d.sim_mode || d.floor2.connected || d.floor1.connected || d.rear.connected;
  const running = ['guiding', 'handoff', 'complete'].includes(ph);
  let cs = ['', '', '', ''];
  if (d.disaster === 'black') cs = ['cut', 'cut', 'cut', running ? 'on' : ''];
  else if (d.disaster) cs = ['on', 'on', anyConn ? 'on' : 'cut', running || d.disaster === 'quake' ? 'on' : ''];
  cs.forEach((c, i) => {
    document.getElementById('cn' + i).className = 'cnode' + (c === 'on' ? ' active' : c === 'cut' ? ' cut' : '');
    if (i < 3) document.getElementById('cl' + i).className = 'clink' +
      (cs[i] === 'cut' || cs[i + 1] === 'cut' ? ' cut' : cs[i] === 'on' && cs[i + 1] === 'on' ? ' on' : '');
  });
  document.getElementById('pathNote').innerHTML = d.disaster
    ? PATH_NOTES[d.disaster] : '평시 — 신호 없음. 연결 상태로 로봇 고장 감지.';

  Object.keys(EXCEPTIONS).forEach(k => {
    const v = d.exceptions[k] || '';
    const el = document.getElementById('ex_' + k);
    el.className = 'ex ' + v;
    el.querySelector('.est').textContent = v === 'triggered' ? '발생' : v === 'resolved' ? '처리됨' : '';
  });
}
