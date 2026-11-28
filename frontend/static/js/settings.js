// BANDI 대시보드 — 방향 보정(자동 에임)·주행 속도 설정
const ACC_LABEL = ['불안정 — 폰으로 8자를 그려 보정', '정확도 낮음', '정확도 보통', '정확도 높음'];
function renderAim(d) {
  const ph = d.phone || {};
  const fresh = ph.azimuth != null && ph.age != null && ph.age < 5;
  document.getElementById('phoneAz').textContent = fresh ? Math.round(ph.azimuth) + '°' : '—';
  document.getElementById('phoneAzNote').textContent =
    !fresh ? '앱에서 방위 수신 없음' :
    ph.quality < 0.5 ? '폰을 세워 주세요 (셀피가 진행 방향을 보게)' :
    (ACC_LABEL[ph.accuracy] || '');
  Object.entries(d.aim_trim || {}).forEach(([k, t]) => {
    const el = document.getElementById('aimTrim_' + k);
    if (el) el.textContent = '보정 ' + (t > 0 ? '+' : '') + t + '°';
  });
  Object.entries(d.aim || {}).forEach(([k, a]) => {
    const el = document.getElementById('aimSt_' + k);
    if (!el) return;
    el.textContent = a.msg || '';
    el.className = 'aim-st ' + (a.status || '');
    if (a.status === 'done') document.getElementById('aimDeg_' + k).textContent = '0° ✔';
  });
}
function autoAim() {
  const hint = document.getElementById('aimHint');
  fetch('/api/aim/auto', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' })
    .then(r => r.json())
    .then(j => {
      hint.style.color = j.ok ? '' : 'var(--red)';
      hint.textContent = j.ok
        ? `🧭 자동 에임 시작: ${j.started.join(', ')} — 로봇이 제자리에서 돌며 북쪽을 찾습니다 (셀피 방위 ${Math.round(j.azimuth)}°)`
          + (j.skipped.length ? ` · 건너뜀: ${j.skipped.join(', ')}` : '')
        : '⚠ ' + j.error;
    })
    .catch(e => { hint.textContent = '⚠ 요청 실패: ' + e; hint.style.color = 'var(--red)'; });
}

const SPEED_REF = BANDI.SPEED_REF;
const DEFAULT_SPEEDS = BANDI.DEFAULT_SPEEDS;
let speeds = Object.assign({}, DEFAULT_SPEEDS);
const SPEED_KEYS = ['floor2', 'rear', 'floor1'];
const DEFAULT_R2_DELAY = BANDI.DEFAULT_R2_DELAY_SEC;
const DEFAULT_TURN_PAUSE = BANDI.DEFAULT_TURN_PAUSE_SEC;
const DEFAULT_R2_DIST = BANDI.DEFAULT_R2_DIST_PCT;
const DEFAULT_GAP = BANDI.FOLLOW_GAP_M;
const DEFAULT_SLOT = BANDI.DEFAULT_DEPLOY_SLOT_CM;
function renderSpeeds(d) {
  if (!d.speeds) return;
  speeds = d.speeds;
  SPEED_KEYS.forEach(k => {
    const el = document.getElementById('spd_' + k);
    if (!el.dataset.touched) el.value = d.speeds[k];
  });
  const dl = document.getElementById('spd_r2delay');
  if (!dl.dataset.touched && d.r2_delay != null) dl.value = d.r2_delay;
  const tp = document.getElementById('spd_turnpause');
  if (!tp.dataset.touched && d.turn_pause != null) tp.value = d.turn_pause;
  const rd = document.getElementById('spd_r2dist');
  if (!rd.dataset.touched && d.r2_dist_pct != null) rd.value = d.r2_dist_pct;
  const gp = document.getElementById('spd_gap');
  if (!gp.dataset.touched && d.follow_gap != null) gp.value = Math.round(d.follow_gap * 100);
  const sl = document.getElementById('spd_slot');
  if (!sl.dataset.touched && d.deploy_slot != null) sl.value = Math.round(d.deploy_slot);
}
function postSpeeds(body) {
  const hint = document.getElementById('speedHint');
  fetch('/api/speeds', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) })
    .then(r => r.json())
    .then(j => {
      if (!j.ok) { hint.textContent = '⚠ ' + j.error; hint.style.color = 'var(--red)'; return; }
      SPEED_KEYS.forEach(k => { const el = document.getElementById('spd_' + k); delete el.dataset.touched; el.value = j.speeds[k]; });
      const dl = document.getElementById('spd_r2delay'); delete dl.dataset.touched; dl.value = j.r2_delay;
      const tp = document.getElementById('spd_turnpause'); delete tp.dataset.touched; tp.value = j.turn_pause;
      const rd = document.getElementById('spd_r2dist'); delete rd.dataset.touched; rd.value = j.r2_dist_pct;
      const gp = document.getElementById('spd_gap'); delete gp.dataset.touched; gp.value = Math.round(j.follow_gap * 100);
      const sl = document.getElementById('spd_slot'); delete sl.dataset.touched; sl.value = Math.round(j.deploy_slot);
      const t2 = (PATH_2F.reduce((a, p) => a + p.duration, 0) * SPEED_REF / j.speeds.floor2).toFixed(1);
      const t1 = (PATH_1F.reduce((a, p) => a + p.duration, 0) * SPEED_REF / j.speeds.floor1).toFixed(1);
      hint.style.color = '';
      hint.textContent = `✔ 저장됨 — R1 ${j.speeds.floor2} · R2 ${j.speeds.rear} (${j.r2_delay}초 뒤 출발, 거리 ${j.r2_dist_pct}%) · R3 ${j.speeds.floor1} · 꺾기 전 ${j.turn_pause}초 정지 · 간격 ${Math.round(j.follow_gap * 100)}cm · 칸 간격 ${Math.round(j.deploy_slot)}cm · 예상 2층 ${t2}초 / 1층 ${t1}초 (일반 모드)`;
      update();
    })
    .catch(e => { hint.textContent = '⚠ 요청 실패: ' + e; hint.style.color = 'var(--red)'; });
}
function saveSpeeds() {
  const body = {};
  SPEED_KEYS.forEach(k => { body[k] = parseInt(document.getElementById('spd_' + k).value, 10); });
  body.r2_delay = parseFloat(document.getElementById('spd_r2delay').value);
  body.turn_pause = parseFloat(document.getElementById('spd_turnpause').value);
  body.r2_dist_pct = parseFloat(document.getElementById('spd_r2dist').value);
  body.follow_gap = parseFloat(document.getElementById('spd_gap').value) / 100;   // cm → m
  body.deploy_slot = parseFloat(document.getElementById('spd_slot').value);
  postSpeeds(body);
}
function resetSpeeds() { postSpeeds(Object.assign({}, DEFAULT_SPEEDS, { r2_delay: DEFAULT_R2_DELAY, turn_pause: DEFAULT_TURN_PAUSE, r2_dist_pct: DEFAULT_R2_DIST, follow_gap: DEFAULT_GAP, deploy_slot: DEFAULT_SLOT })); }
SPEED_KEYS.concat(['r2delay', 'turnpause', 'r2dist', 'gap', 'slot']).forEach(k => document.getElementById('spd_' + k)
  .addEventListener('input', e => { e.target.dataset.touched = '1'; }));

function aim(robot, action, delta) {
  const hint = document.getElementById('aimHint');
  fetch('/api/aim', { method: 'POST', headers: { 'Content-Type': 'application/json' },
                      body: JSON.stringify({ robot, action, delta: delta || 0 }) })
    .then(r => r.json())
    .then(j => {
      if (!j.ok) { hint.textContent = '⚠ ' + j.error; hint.style.color = 'var(--red)'; return; }
      hint.style.color = '';
      document.getElementById('aimDeg_' + robot).textContent = action === 'set' ? '0° ✔' : j.heading + '°';
    })
    .catch(e => { hint.textContent = '⚠ 요청 실패: ' + e; hint.style.color = 'var(--red)'; });
}
