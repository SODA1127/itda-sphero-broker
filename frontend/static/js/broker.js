// BANDI 대시보드 — BLE 연결·리셋·재시작
function triggerSignal() {
  fetch('/api/trigger', { method: 'POST' }).catch(console.error);
}
function resetBroker() {
  fetch('/api/reset', { method: 'POST' }).catch(console.error);
}

function restartBroker() {
  if (!confirm('브로커를 재시작합니다. 잠시 후 페이지를 새로고침하세요.')) return;
  fetch('/api/restart', { method: 'POST' })
    .finally(() => setTimeout(() => location.reload(), 1500));
}

function disconnectSphero() {
  const hint = document.getElementById('connectHint');
  hint.textContent = '⏏ 연결 해제 중…';
  hint.style.color = 'var(--yel)';
  fetch('/api/disconnect', { method: 'POST' })
    .then(r => r.json())
    .then(() => {
      hint.textContent = '✅ 연결 해제됨 — 다시 연결하려면 연결 시작을 누르세요';
      hint.style.color = 'var(--dim)';
      document.getElementById('btnConnect').disabled = false;
    })
    .catch(e => {
      hint.textContent = '❌ 해제 실패: ' + e;
      hint.style.color = 'var(--red)';
    });
}

function connectSphero() {
  const n2f = document.getElementById('inp2f').value.trim();
  const n1f = document.getElementById('inp1f').value.trim();
  const nR2 = document.getElementById('inpR2').value.trim();
  const hint = document.getElementById('connectHint');
  const btn  = document.getElementById('btnConnect');

  if (!n2f && !n1f && !nR2) {
    hint.textContent = '⚠ 이름을 최소 하나 입력하세요.';
    hint.style.color = 'var(--red)';
    return;
  }

  btn.disabled = true;
  hint.textContent = '🔍 BLE 스캔 중… (10~30초 소요)';
  hint.style.color = 'var(--yel)';

  fetch('/api/connect', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name_2f: n2f, name_1f: n1f, name_r2: nR2 })
  })
  .then(r => r.json())
  .then(d => {
    if (d.ok) {
      hint.textContent = '✅ 연결 시도 중 — 로그 확인';
      hint.style.color = 'var(--grn)';
    } else {
      hint.textContent = '❌ ' + (d.error || '오류');
      hint.style.color = 'var(--red)';
      btn.disabled = false;
    }
  })
  .catch(e => {
    hint.textContent = '❌ 요청 실패: ' + e;
    hint.style.color = 'var(--red)';
    btn.disabled = false;
  });
}
