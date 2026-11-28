// BANDI 대시보드 — 서버 설정값(BANDI)·공용 상수·POST 도우미
let PATH_2F = BANDI.PATH_2F;
const PATH_1F = BANDI.PATH_1F;
let PATH_R2 = BANDI.PATH_R2;
const TARGET_MODES = BANDI.TARGET_MODES;
const DISASTERS    = BANDI.DISASTERS;
const EXCEPTIONS   = BANDI.EXCEPTIONS;
const PHASES = ['idle', 'armed', 'guiding', 'handoff', 'complete'];
const PATH_NOTES = {
  fire:  '<b>네트워크 경유</b> — 화재는 로봇이 직접 감지할 수 없음. 외부 신호(수신반 대역)를 중계기가 받아 전달.',
  quake: '<b>다중 교차검증</b> — 1대만 흔들리면 외부 접촉, 여러 대가 동시에 흔들리면 지진 확정. (PoC: R1 가속도 ' + BANDI.SHAKE_THRESHOLD + 'G↑)',
  black: '<b>자립 감지</b> — 정전 시 공유기도 꺼짐. 로봇이 광센서로 스스로 판정하고 저장 경로로 유도. (PoC: 브로커가 저장 경로를 그대로 실행)'
};
const ROBOTS = [
  { id: 'R1', key: 'floor2', name: 'R1 선두',    role: 'LED 화살표 방향 표시',      color: '#ff6b35' },
  { id: 'R2', key: 'rear',   name: 'R2 후방',    role: '이탈 감지 · 스마트폰 탑재', color: '#fbbf24' },
  { id: 'R3', key: 'floor1', name: 'R3 1층 대기', role: '서버 신호로 층간 인수인계', color: '#34d399' },
  { id: 'R4', key: null,     name: 'R4 예비',    role: '고장 시 R1·R2 역할 대체',   color: '#9aa7b4' }
];

function postJSON(url, body) {
  return fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) })
    .then(() => update()).catch(console.error);
}
function sendDisaster(type) { postJSON('/api/disaster', { type }); }
function sendEstop()         { postJSON('/api/estop', {}); }
