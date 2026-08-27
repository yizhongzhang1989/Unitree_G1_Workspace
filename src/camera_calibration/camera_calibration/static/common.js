'use strict';

// 页面骨架共用的部分。app.js 在这之后加载。

const $ = (id) => document.getElementById(id);

// 拆成两个函数而不是靠 body 有没有来选方法：漏传 body 会静默变成 GET，
// 而 POST 路由表匹配不上就是 404，按钮会哑掉且没有任何提示。
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
  return data.result === undefined ? data : data.result;
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

// 按钮点下去到响应回来这段时间要禁用，否则连点会重复采集/重复求解。
// fn 返回值里带 _banner 就用它报，给“部分成功”留个口子
async function guard(button, fn) {
  const label = button.textContent;
  button.disabled = true;
  button.textContent = '…';
  try {
    const out = await fn();
    const custom = out && out._banner;
    banner(custom ? custom[0] : (button.dataset.done || '完成'),
           custom ? custom[1] : 'good');
    return out;
  } catch (err) {
    banner(err.message, 'bad');
    return null;
  } finally {
    button.disabled = false;
    button.textContent = label;
    refresh();
  }
}

const num = (v, digits = 3) => (v === null || v === undefined ? '—' : Number(v).toFixed(digits));
const profileKey = (w, h) => `${w}x${h}`;
const vec = (values, digits = 5) => values.map((v) => num(v, digits)).join(', ');
const degs = (radians, digits = 3) =>
  radians.map((v) => (v * 180 / Math.PI).toFixed(digits)).join(', ');

// 内容指纹没变就别重建 DOM：轮询每秒重建会打断滚动和悬停，
// 自动化点击也会因为元素失效而超时
function stale(host, sig) {
  if (host.dataset.sig === sig) return false;
  host.dataset.sig = sig;
  return true;
}

// 上一张还在路上就改 src，浏览器会把它 abort 掉：白发一次请求，而且帧大一点
// 每次都来不及加完，画面就一直空着
function refreshPreview(img, url) {
  if (img.dataset.busy === '1') return;
  img.dataset.busy = '1';
  const done = () => { img.dataset.busy = '0'; };
  img.onload = done;
  img.onerror = done;
  img.src = url;
}

$('banner').onclick = () => { HOLD = 0; };
