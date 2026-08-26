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

// 对方面板没起时不藏链接，只置灰 —— 藏起来会让人以为压根没这个页
function peerLink(port, alive) {
  const el = $('peer');
  el.href = `${location.protocol}//${location.hostname}:${port}/`;
  el.classList.toggle('down', !alive);
  el.title = alive ? '在新标签页打开' : `那边的节点没在跑，链接指向 :${port}`;
}

const mb = (n) => (n >= 1e9 ? (n / 1e9).toFixed(2) + ' GB' : (n / 1e6).toFixed(0) + ' MB');

// 单个文件的体量跨度太大（manifest 几百字节、head.mkv 八十兆），
// 全按 mb() 抹成“0 MB”就看不出哪个大哪个小了。
const size = (n) => {
  for (const [lim, unit] of [[1e9, 'GB'], [1e6, 'MB'], [1e3, 'KB']]) {
    if (n >= lim) return (n / lim).toFixed(n >= lim * 10 ? 0 : 1) + ' ' + unit;
  }
  return n + ' B';
};

$('banner').onclick = () => { HOLD = 0; };
