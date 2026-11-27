// BANDI 대시보드 — 앱 연결 주소·넘어짐 표시
function copyAppUrl() {
  const url = document.getElementById('appUrl').textContent;
  const done = () => {
    const el = document.getElementById('appUrlCopied');
    el.style.display = 'inline';
    setTimeout(() => { el.style.display = 'none'; }, 1500);
  };
  if (navigator.clipboard && window.isSecureContext) {
    navigator.clipboard.writeText(url).then(done).catch(() => {});
  } else {
    const ta = document.createElement('textarea');
    ta.value = url; document.body.appendChild(ta); ta.select();
    try { document.execCommand('copy'); done(); } catch (e) {}
    ta.remove();
  }
}

function fallMetricText(f) {
  return `w/h ${f.aspect == null ? '—' : f.aspect.toFixed(2)} · 기울기 ${f.tilt == null ? '—' : Math.round(f.tilt) + '°'}`;
}
function renderFall(f) {
  const was = followFallen;
  followFallen = !!(f.active && f.fallen);
  document.getElementById('fallAlert').classList.toggle('on', followFallen);
  document.getElementById('followPreviewWrap').classList.toggle('fallen', followFallen);
  document.getElementById('followMetrics').textContent = f.active ? fallMetricText(f) : '';
  if (followFallen) {
    document.getElementById('fallSecs').textContent = Math.floor(f.fallen_secs || 0);
    const stale = f.stale_secs != null && f.stale_secs > 3;
    document.getElementById('fallMeta').textContent =
      stale ? `카메라 신호 ${Math.floor(f.stale_secs)}초 끊김` : fallMetricText(f);
  }
  if (was !== followFallen) update();
}
