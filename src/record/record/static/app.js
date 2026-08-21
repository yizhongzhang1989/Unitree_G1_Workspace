'use strict';

const $ = (id) => document.getElementById(id);
let STATE = null;          // 最近一次 /api/state
let PICKED = null;         // 开录前的勾选；开录后由后端的 recording 字段接管
let DONE = new Set();      // 本轮已标注的 episode

async function api(path, body) {
  const res = await fetch(path, body === undefined ? {} : {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  const data = await res.json().catch(() => ({ error: '响应不是 JSON' }));
  if (!res.ok || data.ok === false) throw new Error(data.error || res.statusText);
  return data;
}

function banner(text, cls) {
  const el = $('banner');
  el.textContent = text;
  el.className = 'banner' + (cls ? ' ' + cls : '');
}

// 开录前默认全勾，深度图除外（16UC1 压不了，21 GB/h 比三路彩色加起来还大）
function defaultPicks(streams) {
  const picked = {};
  for (const s of streams) picked[s.key] = s.default_on;
  return picked;
}

function renderStreams(streams, locked) {
  if (PICKED === null) PICKED = defaultPicks(streams);
  const body = $('streams').querySelector('tbody');
  body.innerHTML = '';
  let on = 0;
  for (const s of streams) {
    const checked = locked ? s.recording : !!PICKED[s.key];
    if (checked) on += 1;
    const tr = document.createElement('tr');
    if (!checked) tr.className = 'off';
    const dot = s.online ? 'on' : (s.age > 900 ? 'idle' : 'off');
    const state = s.online ? `${s.age.toFixed(1)}s 前` : (s.age > 900 ? '无数据' : '断流');
    tr.innerHTML = `
      <td><input type="checkbox" data-key="${s.key}" ${checked ? 'checked' : ''}
                 ${locked ? 'disabled' : ''}></td>
      <td>${s.key}</td>
      <td class="hint">${s.topic}</td>
      <td class="hint">${s.type}</td>
      <td class="num">${s.columns || ''}</td>
      <td class="num">${s.received}</td>
      <td class="num">${s.written}</td>
      <td><span class="dot ${dot}"></span>${state}</td>
      <td class="hint">${s.note || ''}</td>`;
    body.appendChild(tr);
  }
  body.querySelectorAll('input[type=checkbox]').forEach((box) => {
    box.onchange = () => { PICKED[box.dataset.key] = box.checked; };
  });
  $('stream-summary').textContent =
    locked ? `已冻结 ${on} 路（开录后不可更改）` : `已勾选 ${on} / ${streams.length} 路`;
}

function renderEpisodes(status) {
  const box = $('episodes');
  const detail = status.round_detail;
  if (!detail) {
    box.innerHTML = '<p class="hint">还没有摆放样例。点「生成新一轮摆放」。</p>';
    return;
  }
  const inEpisode = status.state === 'episode';
  box.innerHTML = '';
  detail.episodes.forEach((ep, i) => {
    const div = document.createElement('div');
    const active = inEpisode && status.episode === i;
    div.className = 'ep' + (active ? ' active' : '') + (DONE.has(i) ? ' done' : '');
    const pass = ep.verb === 'Pass' ? '<span class="tag pass">交接</span>' : '';
    const warn = (ep.lint_warnings || []).length
      ? `<span class="tag">lint ${ep.lint_warnings.length}</span>` : '';
    div.innerHTML = `
      <div><span class="tag">${i + 1}/${detail.episodes.length}</span>
           <span class="tag">${ep.verb}</span>
           <span class="tag">${ep.arm}</span>${pass}${warn}</div>
      <div class="en">${ep.instruction_en}</div>
      <div class="zh">${ep.instruction_zh}</div>
      <div class="acts"></div>`;
    const acts = div.querySelector('.acts');
    if (active) {
      for (const [outcome, label, cls] of [
        ['success', '成功', 'ok'], ['fail', '失败', 'danger'], ['discard', '丢弃', '']]) {
        const b = document.createElement('button');
        b.textContent = label;
        if (cls) b.className = cls;
        b.onclick = async () => {
          await api('/api/episode/end', { outcome });
          DONE.add(i);
          refresh();
        };
        acts.appendChild(b);
      }
    } else if (!inEpisode && status.state === 'round') {
      const b = document.createElement('button');
      b.textContent = DONE.has(i) ? '重录' : '开始';
      b.onclick = async () => {
        await api('/api/episode/start', { index: i });
        DONE.delete(i);
        refresh();
      };
      acts.appendChild(b);
    }
    box.appendChild(div);
  });
}

async function loadScene() {
  const svg = await fetch('/api/round/svg').then((r) => r.text());
  $('scene').innerHTML = svg || '<p class="hint">还没有摆放样例</p>';
}

function renderControls(status) {
  const live = status.state !== 'idle';
  $('btn-start').disabled = live;
  $('btn-stop').disabled = !live;
  $('btn-round').disabled = status.state !== 'session';
  $('btn-round-end').disabled = status.state !== 'round';
  $('note').disabled = live;
  const c = status.counts || {};
  if (!live) {
    banner(status.library_error ? '物品库异常：' + status.library_error : '待命',
           status.library_error ? 'bad' : '');
  } else {
    banner(`录制中 · ${status.session} · round ${status.round} · `
      + `episode ${c.episodes || 0}（成 ${c.success || 0} / 败 ${c.fail || 0}）· `
      + `${(status.bytes / 1e6).toFixed(1)} MB · 盘余 ${status.disk_free_gb} GB`, 'live');
  }
}

async function refresh() {
  let data;
  try {
    data = await api('/api/state');
  } catch (err) {
    banner('后端连不上：' + err.message, 'bad');
    return;
  }
  const prevRound = STATE && STATE.status ? STATE.status.round : -1;
  STATE = data;
  const locked = data.status.state !== 'idle';
  renderStreams(data.streams, locked);
  renderControls(data.status);
  renderEpisodes(data.status);
  if (data.status.round !== prevRound) {
    DONE = new Set();
    if (data.status.round >= 0) loadScene();
  }
}

$('btn-start').onclick = async () => {
  try {
    await api('/api/session/start', { streams: PICKED, note: $('note').value });
    refresh();
  } catch (err) { banner('开录失败：' + err.message, 'bad'); }
};
$('btn-stop').onclick = async () => {
  if (!confirm('结束 session？会收尾所有文件并封口。')) return;
  try {
    const r = await api('/api/session/stop');
    banner(`已封口 ${r.result.session}：${r.result.files} 个文件 `
      + `${(r.result.bytes / 1e9).toFixed(2)} GB`);
    PICKED = null;
    refresh();
  } catch (err) { banner('收尾失败：' + err.message, 'bad'); }
};
$('btn-round').onclick = async () => {
  try { await api('/api/round/start', {}); await refresh(); await loadScene(); }
  catch (err) { banner('生成失败：' + err.message, 'bad'); }
};
$('btn-round-end').onclick = async () => {
  try { await api('/api/round/end', {}); refresh(); }
  catch (err) { banner(err.message, 'bad'); }
};

refresh();
setInterval(refresh, 1000);
