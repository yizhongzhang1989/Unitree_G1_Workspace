'use strict';

// 两个面板共用的部分。各自的页面脚本在这之后加载。

const $ = (id) => document.getElementById(id);

// 拆成两个函数而不是靠 body 有没有来选方法：漏传 body 会静默变成 GET，
// 而 POST 路由表匹配不上就是 404 —— 「结束 session」按钮曾经这样哑了很久。
async function get(path) {
  const res = await fetch(path);
  const data = await res.json().catch(() => ({ error: '响应不是 JSON' }));
  if (!res.ok || data.ok === false) throw new Error(data.error || res.statusText);
  return data;
}

async function post(path, body = {}) {
  const res = await fetch(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  const data = await res.json().catch(() => ({ error: '响应不是 JSON' }));
  if (!res.ok || data.ok === false) throw new Error(data.error || res.statusText);
  return data;
}

let HOLD = 0;   // 要紧提示的保留截止时刻

// fromPoll 的提示抢不过正在展示的错误 —— 否则报错不到一秒就被 1 Hz 轮询冲掉
function banner(text, cls, fromPoll = false) {
  if (fromPoll && Date.now() < HOLD) return;
  if (!fromPoll) HOLD = cls === 'bad' ? Date.now() + 15000 : 0;
  const el = $('banner');
  el.textContent = text;
  el.className = 'banner' + (cls ? ' ' + cls : '');
  el.title = HOLD ? '点一下收起' : '';
}

// 对方面板没起时后端会把 peer_port 报成 0，别放一个死链接
function peerLink(port) {
  const el = $('peer');
  el.hidden = !port;
  if (port) el.href = `${location.protocol}//${location.hostname}:${port}/`;
}

const mb = (n) => (n >= 1e9 ? (n / 1e9).toFixed(2) + ' GB' : (n / 1e6).toFixed(0) + ' MB');

$('banner').onclick = () => { HOLD = 0; };
