// 반디 브로커 외부 접속 프록시 — 모든 요청을 VPC 서비스(터널 너머 맥의 브로커)로 넘긴다.
// 브로커는 X-Bandi-Remote 헤더로 외부 접속을 알아보고 보기 전용(비밀번호 로그인 시 조작)으로 처리한다.
const ORIGIN = "http://127.0.0.1:5002";

export default {
  async fetch(request, env) {
    const url = new URL(request.url);
    const headers = new Headers(request.headers);
    headers.set("X-Bandi-Remote", "1");                       // 클라이언트가 보낸 값은 덮어씀
    headers.set("X-Bandi-Client-IP", request.headers.get("CF-Connecting-IP") || "");
    const init = {
      method: request.method,
      headers,
      body: ["GET", "HEAD"].includes(request.method) ? undefined : request.body,
      redirect: "manual",
    };
    try {
      return await env.BROKER.fetch(ORIGIN + url.pathname + url.search, init);
    } catch (e) {
      return new Response(OFFLINE_HTML, {
        status: 502,
        headers: { "content-type": "text/html; charset=utf-8", "cache-control": "no-store" },
      });
    }
  },
};

const OFFLINE_HTML = `<!doctype html><html lang="ko"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>BANDI — 브로커 꺼짐</title>
<body style="margin:0;min-height:100vh;display:grid;place-items:center;background:#0f1117;color:#e2e8f0;font-family:system-ui,sans-serif">
<div style="text-align:center;padding:24px"><h1 style="font-size:20px">반디 브로커에 연결할 수 없습니다</h1>
<p style="color:#94a3b8">현장 맥의 브로커 또는 터널이 꺼져 있습니다. 잠시 뒤 새로고침하세요.</p></div></body></html>`;
