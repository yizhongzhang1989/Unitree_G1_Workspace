'use strict';

let STATE = null;          // 最近一次 /api/state
let PICKED = null;         // 开录前的勾选；开录后由后端的 recording 字段接管
let DONE = new Set();      // 本轮已标注的 episode
let SCENE_KEY = '';        // 当前画的是哪一轮，变了才重拉 SVG
let FOLDED = false;
let WAS_LOCKED = null;

function setFold(v) {
  FOLDED = v;
  $('streams-card').classList.toggle('folded', v);
  $('btn-fold').textContent = v ? '▸' : '▾';
  $('btn-fold').title = v ? '展开' : '折叠';
}
$('btn-fold').onclick = () => setFold(!FOLDED);

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
  // 折叠起来后这行是唯一还看得见的健康指示，所以录制时要带上断流数
  const dead = streams.filter((s) => (locked ? s.recording : !!PICKED[s.key]) && !s.online).length;
  const sum = $('stream-summary');
  sum.textContent = locked
    ? `已冻结 ${on} 路` + (dead ? ` · ⚠ ${dead} 路没在收` : ' · 全部在收')
    : `已勾选 ${on} / ${streams.length} 路`;
  sum.className = 'hint' + (locked && dead ? ' warn' : '');
}

function renderEpisodes(status) {
  const box = $('episodes');
  const committed = status.round_detail;
  const detail = committed || status.pending_round;
  if (!detail) {
    box.innerHTML = '<p class="hint">还没有摆放样例。点「生成任务 / 重 roll」。</p>';
    return;
  }
  const preview = !committed;
  const inEpisode = status.state === 'episode';
  box.innerHTML = '';
  detail.episodes.forEach((ep, i) => {
    const div = document.createElement('div');
    const active = inEpisode && status.episode === i;
    div.className = 'ep' + (active ? ' active' : '') + (DONE.has(i) ? ' done' : '')
      + (preview ? ' preview' : '');
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
          await post('/api/episode/end', { outcome });
          DONE.add(i);
          refresh();
        };
        acts.appendChild(b);
      }
    } else if (!inEpisode && status.state === 'round') {
      const b = document.createElement('button');
      b.textContent = DONE.has(i) ? '重录' : '开始';
      b.onclick = async () => {
        await post('/api/episode/start', { index: i });
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
  const inRound = status.state === 'round' || status.state === 'episode';
  const pending = !!status.pending_round;
  $('btn-start').disabled = live;
  $('btn-stop').disabled = !live;
  $('btn-preview').disabled = !!status.library_error;
  $('btn-shot').disabled = live;
  $('shot-hint').textContent = live
    ? '录制中不抓快照 —— 解码会跟录制抢 CPU。'
    : '帧计数只能发现流哑了，发现不了相机对着墙。';
  $('btn-round').disabled = status.state !== 'session' || !pending;
  $('btn-round-end').disabled = status.state !== 'round';
  $('note').disabled = live;

  const hint = $('round-hint');
  if (inRound) {
    hint.textContent = '本轮已固化，逐条执行。';
  } else if (pending && !live) {
    hint.textContent = '预览中，未开录。照样例摆好桌子后再点「开始 session」，摆桌过程不进视频。';
  } else if (pending) {
    hint.textContent = '预览中。摆好桌子后点「开始本轮」固化。';
  } else {
    hint.textContent = '先生成任务，照着样例摆好真实桌面，再开录。';
  }

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

// 预览和已固化用同一个 seed，加前缀区分；固化那一下也算变化，DONE 跟着清
function sceneKey(st) {
  if (st.round_detail) return 'c' + st.round_detail.seed;
  if (st.pending_round) return 'p' + st.pending_round.seed;
  return '';
}

async function refresh() {
  let data;
  try {
    data = await get('/api/state');
  } catch (err) {
    banner('后端连不上：' + err.message, 'bad', true);
    return;
  }
  STATE = data;
  const locked = data.status.state !== 'idle';
  // 开录后勾选已冻结，这张表就不再需要占位置；收尾后展回来好检查下一轮
  if (WAS_LOCKED !== null && WAS_LOCKED !== locked) setFold(locked);
  WAS_LOCKED = locked;
  renderStreams(data.streams, locked);
  renderControls(data.status);
  renderEpisodes(data.status);
  peerLink(data.status.peer_port);
  const key = sceneKey(data.status);
  if (key !== SCENE_KEY) {
    SCENE_KEY = key;
    DONE = new Set();
    if (key) loadScene();
    else $('scene').innerHTML = '<p class="hint">还没有摆放样例</p>';
  }
}

$('btn-preview').onclick = async () => {
  const st = (STATE && STATE.status) || {};
  const inRound = st.state === 'round' || st.state === 'episode';
  if (inRound && !confirm('本轮还没结束。生成新任务会先结束本轮，已录的部分照常保留。')) return;
  try {
    if (inRound) await post('/api/round/end');
    await post('/api/round/preview');
    await refresh();
  } catch (err) { banner('生成失败：' + err.message, 'bad'); }
};
$('btn-start').onclick = async () => {
  try {
    const had = STATE && STATE.status && STATE.status.pending_round;
    await post('/api/session/start', { streams: PICKED, note: $('note').value });
    if (had) await post('/api/round/start', {});   // 开录即固化，不再单独点一次
    refresh();
  } catch (err) { banner('开录失败：' + err.message, 'bad'); }
};
$('btn-stop').onclick = async () => {
  if (!confirm('结束 session？进行中的 episode 会记为丢弃，本轮自动收掉，然后封口。')) return;
  banner('收尾中…');
  try {
    const r = await post('/api/session/stop');
    banner(`已封口 ${r.result.session}：${r.result.files} 个文件 `
      + `${(r.result.bytes / 1e9).toFixed(2)} GB`);
    PICKED = null;
    refresh();
  } catch (err) { banner('收尾失败：' + err.message, 'bad'); }
};
$('btn-round').onclick = async () => {
  try { await post('/api/round/start', {}); await refresh(); }
  catch (err) { banner('固化失败：' + err.message, 'bad'); }
};
$('btn-round-end').onclick = async () => {
  try { await post('/api/round/end', {}); refresh(); }
  catch (err) { banner(err.message, 'bad'); }
};

$('btn-shot').onclick = async () => {
  const box = $('shots');
  const keys = (STATE ? STATE.streams : [])
    .filter((s) => s.kind === 'video' && s.key !== 'head_depth')
    .map((s) => s.key);
  if (!keys.length) { banner('没有可抓的视频流', 'bad'); return; }
  box.innerHTML = '<span class="hint">抓取中… 腕部要等 RTSP 握手，约 1~2 秒</span>';
  const imgs = keys.map((k) => {
    const img = document.createElement('img');
    img.className = 'shot';
    img.alt = k;
    img.title = k;
    img.src = `/api/snapshot?key=${k}&_=${Date.now()}`;   // 时间戳绕开缓存
    img.onerror = () => { img.replaceWith(Object.assign(
      document.createElement('span'), { className: 'hint', textContent: `${k} 抓不到` })); };
    return img;
  });
  box.innerHTML = '';
  imgs.forEach((i) => box.appendChild(i));
};

refresh();
setInterval(refresh, 1000);
