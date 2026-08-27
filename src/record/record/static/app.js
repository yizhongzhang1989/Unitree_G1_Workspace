'use strict';

let STATE = null;          // 最近一次 /api/state
let PICKED = null;         // 开录前的勾选；开录后由后端的 recording 字段接管
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

const OUTCOME_ZH = { success: '成', fail: '败', discard: '弃' };

// 已录次数由后端给（刷页不丢）。每录一遍就是独立的一条 episode，旧的不会被覆盖
function takesTag(takes) {
  const n = {};
  for (const o of takes) n[o] = (n[o] || 0) + 1;
  const parts = Object.keys(OUTCOME_ZH).filter((k) => n[k]).map((k) => OUTCOME_ZH[k] + n[k]);
  return `<span class="tag take" title="每次重录都另存一条，不覆盖旧的；不想要的标「丢弃」">`
    + `已录 ${takes.length} · ${parts.join(' ')}</span>`;
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
  const slotTakes = status.slot_takes || {};
  box.innerHTML = '';
  detail.episodes.forEach((ep, i) => {
    const div = document.createElement('div');
    const active = inEpisode && status.episode_slot === i;
    const takes = (committed && slotTakes[i]) || [];
    const ok = takes.filter((o) => o === 'success').length;
    // 正在录的这条不淡出：重录旧的成功记录还在，但眼下要读的就是它
    div.className = 'ep' + (preview ? ' ep-preview' : '')
      + (active ? ' active' : (ok ? ' done' : (takes.length ? ' tried' : '')));
    const pass = ep.verb === 'Pass' ? '<span class="tag pass">交接</span>' : '';
    const warn = (ep.lint_warnings || []).length
      ? `<span class="tag">lint ${ep.lint_warnings.length}</span>` : '';
    div.innerHTML = `
      <div><span class="tag">${i + 1}/${detail.episodes.length}</span>
           <span class="tag">${ep.verb}</span>
           <span class="tag">${ep.arm}</span>${pass}${warn}`
      + `${takes.length ? takesTag(takes) : ''}</div>
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
        // 不接住的话请求失败就是「点了没反应」，连错误条都不亮
        b.onclick = async () => {
          try {
            await post('/api/episode/end', { outcome });
          } catch (err) { banner('标注失败：' + err.message, 'bad'); }
          refresh();
        };
        acts.appendChild(b);
      }
    } else if (!inEpisode && status.state === 'round') {
      const b = document.createElement('button');
      b.textContent = takes.length ? '重录' : '开始';
      b.onclick = async () => {
        try {
          await post('/api/episode/start', { index: i });
        } catch (err) { banner('开始 episode 失败：' + err.message, 'bad'); }
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
  $('btn-start').disabled = live || !pending;
  $('btn-start').title = pending || live ? ''
    : '先生成一轮任务并摆好桌子 —— 开录后再摆，那几十秒会全录进视频';
  $('btn-stop').disabled = !live;
  // 本轮固化后不能重 roll：桌上已经照它摆好了，换任务得先「结束本轮」
  $('btn-preview').disabled = !!status.library_error || inRound;
  $('btn-reroll').disabled = !!status.library_error || inRound || !pending;
  $('btn-shot').disabled = live;
  $('shot-hint').textContent = live
    ? '录制中不抓快照 —— 解码会跟录制抢 CPU。'
    : '帧计数只能发现流哑了，发现不了相机对着墙。';
  $('btn-round').disabled = status.state !== 'session' || !pending;
  $('btn-round-end').disabled = status.state !== 'round';
  $('note').disabled = live;

  const hint = $('round-hint');
  if (inRound) {
    hint.textContent = '本轮已固化，逐条执行。同一条可以重录多遍，旧的不会被覆盖；要换任务先「结束本轮」。';
  } else if (pending && !live) {
    hint.textContent = '预览中，未开录。照样例摆好桌子后再点「开始 session」，摆桌过程不进视频。';
  } else if (pending) {
    hint.textContent = '预览中。摆好桌子后点「开始本轮」固化；「结束本轮」不丢这套配置，可以直接再开一轮。';
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
  renderCams(data.streams
    .filter((s) => s.kind === 'video' && s.key !== 'head_depth')
    .map((s) => s.key)
    .sort((a, b) => camRank(a) - camRank(b)));
  peerLink(data.status.peer_port, data.status.peer_alive);
  const key = sceneKey(data.status);
  if (key !== SCENE_KEY) {
    SCENE_KEY = key;
    if (key) loadScene();
    else $('scene').innerHTML = '<p class="hint">还没有摆放样例</p>';
  }
}

// keepItems: 桌上那几件东西不动，只重摆位置和换指令 —— 换物品得起身去找，贵得多
async function rollRound(keepItems) {
  banner('生成中…');            // 摆不开就重摆，实测能到 2.3 s，不吭声会以为点漏了
  try {
    await post('/api/round/preview', keepItems ? { keep_items: true } : {});
    await refresh();
  } catch (err) { banner('生成失败：' + err.message, 'bad'); }
}

$('btn-preview').onclick = () => rollRound(false);
$('btn-reroll').onclick = () => rollRound(true);
$('btn-start').onclick = async () => {
  try {
    await post('/api/session/start', { streams: PICKED, note: $('note').value });
    await post('/api/round/start', {});   // 开录即固化，不再单独点一次
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

// ---- 三路相机的低帧率预览 ----------------------------------------------
// 摆放样例说的是「该摆成什么样」，预览说的是「现在什么样」，挨着放才好对照。
// 后端按需起解码、没人取帧就收；这边只要「不看时别取」就行。
const CAMS = new Map();      // key -> {img, note, busy}

// 左腕 / 头 / 右腕，摆成真实的空间关系 —— 图的位置本身就是「哪路是哪路」的线索。
// 认不出来的排到最后，而不是静悄悄插到最前面
const CAM_ORDER = ['wrist_left', 'head', 'wrist_right'];
const camRank = (k) => (CAM_ORDER.indexOf(k) + 1 || 99);

function renderCams(keys) {
  // 每秒无条件重建的话图会闪，而且刚拿到的帧会被扔掉
  if (keys.join() === [...CAMS.keys()].join()) return;
  const box = $('cams');
  box.innerHTML = '';
  CAMS.clear();
  for (const k of keys) {
    const cell = document.createElement('div');
    cell.className = 'cam';
    cell.innerHTML = `<img alt="${k}"><span class="tag">${k}</span>`
      + '<span class="note">连接中…</span>';
    box.appendChild(cell);
    CAMS.set(k, { img: cell.querySelector('img'),
                  note: cell.querySelector('.note'), busy: false });
  }
}

async function pullCam(key) {
  const cam = CAMS.get(key);
  if (!cam || cam.busy) return;           // 慢的那一路别堆请求
  cam.busy = true;
  try {
    const res = await fetch(`/api/preview?key=${key}`);
    if (!res.ok) {
      const err = await res.json().catch(() => ({}));
      throw new Error(err.error || res.statusText);
    }
    const url = URL.createObjectURL(await res.blob());
    const old = cam.img.src;
    cam.img.src = url;
    if (old.startsWith('blob:')) URL.revokeObjectURL(old);   // 不撤会一直吃内存
    cam.note.hidden = true;
  } catch (err) {
    cam.note.textContent = err.message;
    cam.note.hidden = false;              // 压在画面上，不清掉已有的帧
  } finally {
    cam.busy = false;
  }
}

function camLoop() {
  // 切到别的标签页就停：后台标签页的画面没人看，白烧 CPU
  const on = $('cam-on').checked && document.visibilityState === 'visible';
  $('cams').hidden = !on;
  $('cam-hint').textContent = on
    ? '腕部走 640x360 子码流 2 fps，不碰录制那条 -c copy 的主码流，录制中也能开。'
    : '已关。页面切到后台也会自动停，节点那边跟着把解码进程收掉。';
  if (!on) return;
  for (const k of CAMS.keys()) pullCam(k);
}
setInterval(camLoop, 500);

refresh();
setInterval(refresh, 1000);
