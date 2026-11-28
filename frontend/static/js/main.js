// BANDI 대시보드 — 상태 폴링 — /api/status 받아 화면 전체 갱신
function update() {
  fetch('/api/status')
    .then(r => r.json())
    .then(d => {
      if (d.paths) { PATH_2F = d.paths.floor2; PATH_R2 = d.paths.rear; }
      if (d.follow_gap != null) FOLLOW_GAP_M = d.follow_gap;
      testRunning = !!d.test_running;
      renderManual(d);
      renderMatrixState(d);
      // sim badge
      if (d.sim_mode) document.getElementById('simBadge').style.display = 'inline-block';

      // 2F
      const c2 = d.floor2.connected;
      document.getElementById('dot2f').className   = 'dot' + (c2 ? ' on' : '');
      document.getElementById('conn2f').textContent = c2 ? '연결됨' : '연결 안됨';
      document.getElementById('bat2f').style.width  = d.floor2.battery + '%';
      document.getElementById('bat2fTxt').textContent = `배터리 ${d.floor2.battery}%`;
      document.getElementById('ax').textContent = (d.floor2.accel?.x ?? 0).toFixed(2);
      document.getElementById('ay').textContent = (d.floor2.accel?.y ?? 0).toFixed(2);
      document.getElementById('az').textContent = (d.floor2.accel?.z ?? 0).toFixed(2);

      const pill = document.getElementById('signalPill');
      if (d.floor2.signal) {
        pill.textContent = '신호!'; pill.className = 'signal-pill active';
      } else {
        pill.textContent = '대기';  pill.className = 'signal-pill';
      }

      // 1F
      const c1 = d.floor1.connected;
      document.getElementById('dot1f').className   = 'dot' + (c1 ? ' on' : '');
      document.getElementById('conn1f').textContent = c1 ? '연결됨' : '연결 안됨';
      document.getElementById('bat1f').style.width  = d.floor1.battery + '%';
      document.getElementById('bat1fTxt').textContent = `배터리 ${d.floor1.battery}%`;

      const mov = d.floor1.moving;
      document.getElementById('movingBadge').textContent = mov ? '🏃 이동 중' : '';

      // 2F path steps
      function renderSteps(containerId, path, curStep, timeScale) {
        const el = document.getElementById(containerId);
        el.innerHTML = '';
        path.forEach(p => {
          const sp = document.createElement('div');
          sp.className = 'step-box' + (curStep === p.step ? ' active' : curStep > p.step ? ' done' : '');
          sp.textContent = `${p.char} ${p.duration > 0 ? (p.duration * timeScale).toFixed(1) + 's' : '정지'}`;
          sp.title = `step${p.step}: ${p.meters}m 방향${p.heading}°`;
          el.appendChild(sp);
        });
      }
      const ts2 = SPEED_REF / (d.speeds ? d.speeds.floor2 : SPEED_REF);
      const ts1 = SPEED_REF / (d.speeds ? d.speeds.floor1 : SPEED_REF);
      renderSteps('pathSteps2f', PATH_2F, d.floor2.step || 0, ts2);
      const pct2f = d.floor2.total_steps > 0
        ? Math.round((d.floor2.step || 0) / d.floor2.total_steps * 100) : 0;
      document.getElementById('pathProgress2f').style.width = pct2f + '%';
      document.getElementById('pathLabel2f').textContent = d.floor2.moving
        ? `경로 실행 중: ${d.floor2.step} / ${d.floor2.total_steps}`
        : ((d.floor2.step || 0) === 0 ? '대기 중' : '완료');

      // 1F path steps
      renderSteps('pathSteps', PATH_1F, d.floor1.step, ts1);
      const pct = d.floor1.total_steps > 0
        ? Math.round(d.floor1.step / d.floor1.total_steps * 100) : 0;
      document.getElementById('pathProgress').style.width = pct + '%';
      document.getElementById('pathLabel').textContent = mov
        ? `경로 실행 중: ${d.floor1.step} / ${d.floor1.total_steps}`
        : (d.floor1.step === 0 ? '대기 중' : '완료');

      // R2 후방
      const r2 = d.rear;
      document.getElementById('dotR2').className   = 'dot' + (r2.connected ? ' on' : '');
      document.getElementById('connR2').textContent = r2.connected ? '연결됨' : '연결 안됨';
      document.getElementById('batR2Txt').textContent = `배터리 ${r2.battery}%`;
      const tsR2 = SPEED_REF / (d.speeds ? d.speeds.rear : SPEED_REF) * (d.r2_dist_pct || 100) / 100;
      renderSteps('pathStepsR2', PATH_R2, r2.step || 0, tsR2);
      document.getElementById('pathLabelR2').textContent = r2.moving
        ? `R1 경로 추종 중: ${r2.step} / ${r2.total_steps}`
        : ((r2.step || 0) === 0 ? '대기 중' : '완료');

      updateFloorplan(d);
      renderFlow(d);
      renderAim(d);
      renderSpeeds(d);
      document.getElementById('appUrl').textContent =
        `http://${d.server_ip}:${location.port || (location.protocol === 'https:' ? 443 : 80)}`;

      // counter
      document.getElementById('triggerCount').textContent = d.trigger_count;

      // connect form: 현재 이름 채우기 (한 번만)
      const inp2f = document.getElementById('inp2f');
      const inp1f = document.getElementById('inp1f');
      if (!inp2f.dataset.touched) inp2f.value = d.names?.floor2 || '';
      if (!inp1f.dataset.touched) inp1f.value = d.names?.floor1 || '';
      const inpR2 = document.getElementById('inpR2');
      if (!inpR2.dataset.touched) inpR2.value = d.names?.rear || '';
      inp2f.className = d.floor2.connected ? 'ok' : '';
      inp1f.className = d.floor1.connected ? 'ok' : '';
      inpR2.className = d.rear.connected ? 'ok' : '';

      // 연결 중 → 버튼 비활성
      const btn = document.getElementById('btnConnect');
      if (!d.connecting) btn.disabled = false;

      // log
      const ul = document.getElementById('logList');
      ul.innerHTML = '';
      (d.log || []).forEach(e => {
        const li = document.createElement('li');
        li.innerHTML = `<span class="log-ts">${e.ts}</span><span>${e.msg}</span>`;
        ul.appendChild(li);
      });
    })
    .catch(console.error);
}

['inp2f','inp1f','inpR2'].forEach(id => {
  document.getElementById(id).addEventListener('input', e => {
    e.target.dataset.touched = '1';
  });
});
