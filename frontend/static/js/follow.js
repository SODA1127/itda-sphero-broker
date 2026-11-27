// BANDI 대시보드 — R2 카메라 감지 ON/OFF·프리뷰
const DIR_LABEL = { forward: '↑  멀어짐', back: '↓  다가옴', left: '←  왼쪽', right: '→  오른쪽', stop: '■  정지' };
let followActive = false;
function renderFollow(active) {
  followActive = active;
  const btn = document.getElementById('btnFollowToggle');
  btn.textContent = active ? '⏹ 감지 OFF' : '▶ 감지 ON';
  btn.className   = 'btn ' + (active ? 'btn-follow-off' : 'btn-follow-on');
}
function toggleFollow() {
  fetch('/api/follow/toggle', { method: 'POST' })
    .then(r => r.json())
    .then(d => renderFollow(d.active));
}
setInterval(() => {
  fetch('/api/follow/status').then(r => r.json()).then(f => {
    renderFollow(f.active);
    renderFall(f);
    const dir  = document.getElementById('followDir');
    const info = document.getElementById('followInfo');
    if (!f.active) { dir.textContent = '■'; info.textContent = '감지 꺼짐'; dir.style.color = ''; return; }
    if (f.fallen) {
      dir.textContent = '⚠ 넘어짐'; dir.style.color = 'var(--red)';
      info.textContent = '대피자 넘어짐 감지 (3초 이상)';
    } else if (f.seen && !f.person && f.lost_secs >= 2) {
      dir.textContent = '🦯 시야 이탈'; dir.style.color = 'var(--yel)';
      info.textContent = `대피자 안 보임 ${f.lost_secs.toFixed(0)}초` + (f.blind ? ' — 폰 음성으로 이탈 안내 중' : ' (시각장애 모드에서만 음성 안내)');
    } else {
      dir.textContent = DIR_LABEL[f.direction] || f.direction; dir.style.color = '';
      info.textContent = '대피자 움직임 감지 중';
    }
  }).catch(() => {});
}, 500);

// 프리뷰 snapshot polling (150ms) — 단순 src 교체
setInterval(() => {
  const img = document.getElementById('followPreview');
  img.src = '/api/follow/snapshot?t=' + Date.now();
  img.style.display = 'block';
  document.getElementById('followPlaceholder').style.display = 'none';
}, 150);
