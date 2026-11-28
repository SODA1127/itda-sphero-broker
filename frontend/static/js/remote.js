// BANDI 대시보드 — 외부(Cloudflare 터널) 접속: 보기 전용 안내·비밀번호 로그인
let remote = { external: false, authed: true, login_enabled: false };

function renderRemote(flash) {
  const bar = document.getElementById('remoteBar');
  bar.hidden = !remote.external;
  if (!remote.external) return;
  document.getElementById('remoteText').textContent = remote.authed
    ? '🌐 외부 접속 — 조작 가능'
    : remote.login_enabled ? '🌐 외부 접속 — 보기 전용 (조작하려면 로그인)' : '🌐 외부 접속 — 보기 전용';
  document.getElementById('remoteForm').hidden = remote.authed || !remote.login_enabled;
  document.getElementById('remoteLogout').hidden = !remote.authed;
  if (flash) { bar.classList.remove('flash'); void bar.offsetWidth; bar.classList.add('flash'); }
}
function loadRemote() {
  return fetch('/api/auth').then(r => r.json()).then(d => { remote = d; renderRemote(); }).catch(() => {});
}
function remoteLogin(e) {
  e.preventDefault();
  const pw = document.getElementById('remotePw');
  fetch('/api/auth/login', { method: 'POST', headers: { 'Content-Type': 'application/json' },
                             body: JSON.stringify({ password: pw.value }) })
    .then(r => r.json())
    .then(j => {
      document.getElementById('remoteErr').textContent = j.ok ? '' : '⚠ ' + j.error;
      if (j.ok) { pw.value = ''; loadRemote(); }
    });
}
function remoteLogout() { fetch('/api/auth/logout', { method: 'POST' }).then(loadRemote); }

// 조작 요청이 401(보기 전용)로 막히면 로그인 바를 깜빡여 알림
const _fetch = window.fetch.bind(window);
window.fetch = (...args) => _fetch(...args).then(r => {
  if (r.status === 401) { remote.authed = false; renderRemote(true); }
  return r;
});
loadRemote();
