// BANDI 대시보드 — R2 수동 조작 (경로 이탈 폴백) — WASD·매트릭스·음성 안내
// ── R2 수동 조작 (경로 이탈 폴백) ──
let manualOn = false;
const IR_TEXT = { off: 'IR —', wait: 'IR 확인 중', ok: 'IR R1 인식', lost: 'IR 끊김' };
function renderManual(d) {
  const m = d.manual || {};
  manualOn = !!m.on;
  const btn = document.getElementById('btnManual');
  btn.classList.toggle('on', manualOn);
  btn.textContent = manualOn ? '🎮 수동 조작 끝내기' : '🎮 수동 조작';
  document.getElementById('manualPanel').hidden = !manualOn;
  document.getElementById('manualReason').textContent = '🎮 수동 조작 중 — ' + (m.reason || '대시보드에서 전환');
  document.getElementById('manualHeading').textContent = `방향 ${m.heading ?? 0}°` + (m.cmd === 'w' ? ' · 전진 중' : m.cmd === 's' ? ' · 후진 중' : '');
  const chip = document.getElementById('irChip');
  chip.className = 'ir-chip ' + (d.ir || 'off');
  chip.textContent = IR_TEXT[d.ir] || 'IR —';
  if (!manualOn) stopAllKeys();
}
function manualPost(url, body) {
  return fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) })
    .then(r => r.json()).then(j => { if (!j.ok) alert(j.error); return j; }).catch(console.error);
}
function toggleManual() { manualPost('/api/manual', { on: !manualOn }).then(update); }
function simIrLost()   { manualPost('/api/manual', { on: true, sim: true }).then(update); }
function manualMatrix(pattern) { manualPost('/api/manual/matrix', { pattern }); }
function sendTts(e) {
  e.preventDefault();
  const inp = document.getElementById('ttsInput');
  const text = inp.value.trim();
  if (!text) return;
  manualPost('/api/manual/tts', { text }).then(j => {
    if (!j || !j.ok) return;
    document.getElementById('ttsNote').textContent = j.phone_online
      ? `🔊 "${text}" — 폰으로 보냄` : `⚠ 폰 연결 안 됨 — "${text}"는 폰이 붙으면 15초 안에 읽음`;
    inp.value = '';
  });
}
// 누르는 동안 150ms마다 반복 — 서버는 0.5초 신호 없으면 세움
const keyTimers = {};
function keyDown(cmd) {
  if (!manualOn || keyTimers[cmd]) return;
  const send = () => fetch('/api/manual/drive', { method: 'POST', headers: { 'Content-Type': 'application/json' },
                                                  body: JSON.stringify({ cmd }) }).catch(() => {});
  send();
  keyTimers[cmd] = setInterval(send, cmd === 'a' || cmd === 'd' ? 250 : 150);
  document.querySelector(`.key[data-cmd="${cmd}"]`)?.classList.add('down');
}
function keyUp(cmd) {
  if (!keyTimers[cmd]) return;
  clearInterval(keyTimers[cmd]);
  delete keyTimers[cmd];
  document.querySelector(`.key[data-cmd="${cmd}"]`)?.classList.remove('down');
  if (cmd === 'w' || cmd === 's') fetch('/api/manual/drive', { method: 'POST', headers: { 'Content-Type': 'application/json' },
                                                             body: JSON.stringify({ cmd: 'stop' }) }).catch(() => {});
}
function stopAllKeys() { Object.keys(keyTimers).forEach(keyUp); }
document.querySelectorAll('.key[data-cmd]').forEach(b => {
  const cmd = b.dataset.cmd;
  b.addEventListener('pointerdown', e => { e.preventDefault(); b.setPointerCapture(e.pointerId); keyDown(cmd); });
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(t => b.addEventListener(t, () => keyUp(cmd)));
});
// 물리 키 위치(e.code)로 판단 — 한글 입력 상태(ㅈㅁㄴㅇ)에서도 동작. 방향키도 같은 동작
const KEY_CMD = { KeyW: 'w', ArrowUp: 'w', KeyA: 'a', ArrowLeft: 'a',
                  KeyS: 's', ArrowDown: 's', KeyD: 'd', ArrowRight: 'd' };
document.addEventListener('keydown', e => {
  const cmd = KEY_CMD[e.code];
  if (!cmd || !manualOn || ['INPUT', 'TEXTAREA'].includes(e.target.tagName) || e.metaKey || e.ctrlKey || e.altKey) return;
  e.preventDefault();                 // 방향키 페이지 스크롤 막음
  if (!e.repeat) keyDown(cmd);
});
document.addEventListener('keyup', e => { const cmd = KEY_CMD[e.code]; if (cmd) keyUp(cmd); });
window.addEventListener('blur', stopAllKeys);
