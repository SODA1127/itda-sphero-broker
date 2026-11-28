// BANDI 대시보드 — LED 매트릭스 미리보기 (상황별 화면을 골라 로봇에 띄우기)
const MATRIX_PATTERNS = BANDI.MATRIX_PATTERNS;
const MX_LABEL = Object.fromEntries(MATRIX_PATTERNS.map(p => [p.name, p.label]));
const MX_ROBOTS = [['floor2', 'R1'], ['rear', 'R2'], ['floor1', 'R3']];

function mxCells(frame, color) {
  let html = '';
  for (let r = 0; r < 8; r++)
    for (let c = 0; c < 8; c++)
      html += (frame && frame[r][c] === '#') ? `<i style="background:${color}"></i>` : '<i></i>';
  return html;
}

document.getElementById('mxpGrid').innerHTML = MATRIX_PATTERNS.map(p =>
  `<button class="mxp" id="mxp_${p.name}" onclick="matrixPreview('${p.name}')" title="${p.label} — ${p.desc}">` +
  `<div class="mxp-px" data-frame="-1">${mxCells(p.frames[0], p.color)}</div>` +
  `<b>${p.label}</b><small>${p.desc}</small></button>`).join('');

// 깜빡임·번갈아 보이기도 로봇과 같은 fps로 그려 보여 줌
setInterval(() => {
  const t = Date.now() / 1000;
  MATRIX_PATTERNS.forEach(p => {
    if (p.frames.length < 2) return;
    const idx = Math.floor(t * p.fps) % p.frames.length;
    const px = document.querySelector(`#mxp_${p.name} .mxp-px`);
    if (px && px.dataset.frame !== String(idx)) {
      px.dataset.frame = idx;
      px.innerHTML = mxCells(p.frames[idx], p.color);
    }
  });
}, 100);

document.querySelectorAll('.mxp-rb').forEach(b => b.addEventListener('click', () => b.classList.toggle('on')));

function matrixPreview(pattern) {
  const robots = [...document.querySelectorAll('.mxp-rb.on')].map(b => b.dataset.k);
  const msg = document.getElementById('mxpMsg');
  fetch('/api/matrix/preview', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ robots, pattern }),
  })
    .then(r => r.json())
    .then(j => {
      msg.textContent = j.ok ? '' : '⚠ ' + j.error;
      update();
    })
    .catch(console.error);
}

function renderMatrixState(d) {
  const m = d.matrix || {};
  const name = v => v == null ? '꺼짐' : v === 'text' ? '글자 표시' : (MX_LABEL[v] || v);
  document.getElementById('mxpNow').textContent =
    '지금 — ' + MX_ROBOTS.map(([k, l]) => `${l}: ${name(m[k])}`).join(' · ');
  const cur = new Set(Object.values(m));
  MATRIX_PATTERNS.forEach(p =>
    document.getElementById('mxp_' + p.name).classList.toggle('cur', cur.has(p.name)));
}
