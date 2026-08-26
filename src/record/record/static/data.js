'use strict';

let PICKED = null;      // 当前选中的那次录制，右边详情跟着它
let DETAIL = null;      // 它的详情，避免每次轮询都重拉
let PLAYING = false;
let NOW = { session: '', label: '' };   // 正在放哪一段，用于列表高亮
let SIG = '';           // 上一次渲染的列表指纹，没变就不重建 DOM
let CSIG = '';          // 同上，转换进度的指纹
let CONVERT = { running: false, session: '', format: '', token: '', log: [],
                error: '', done: false, bytes: 0, progress: 0 };
let FORMATS = [];       // 转换格式，来自 tools/converters.py，A/B 同一张表
const RAW = {};         // session id -> 文件清单，展开过的才拉
const OPEN = new Set(); // 展开着的树节点，key 是 `${id}/${目录相对路径}`
let CHOICE = '';        // 详情里选中的转换格式
let FILE = null;        // 正在预览的文件 {id, path}；不为空时右边是预览而不是详情

//: 能直接看的。没后缀的也试一下（`DONE` 就是），真不是文本由后端说
const TEXTY = /(^[^.]+$)|\.(json|jsonl|txt|ya?ml|md|csv|svg)$/i;

const clock = (s) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, '0')}`;

const rawURL = (id, file) =>
  `/raw?id=${encodeURIComponent(id)}&file=${encodeURIComponent(file)}`;
const zipURL = (id, dir) =>
  `/raw.zip?id=${encodeURIComponent(id)}&dir=${encodeURIComponent(dir)}`;

/** 把扁平的相对路径清单折成目录树。后端只给路径和大小，层级在这边拼。 */
function buildTree(files) {
  const root = { dirs: new Map(), files: [], bytes: 0 };
  for (const f of files) {
    const parts = f.path.split('/');
    let node = root;
    node.bytes += f.bytes;
    for (const part of parts.slice(0, -1)) {
      if (!node.dirs.has(part)) node.dirs.set(part, { dirs: new Map(), files: [], bytes: 0 });
      node = node.dirs.get(part);
      node.bytes += f.bytes;
    }
    node.files.push({ name: parts[parts.length - 1], path: f.path, bytes: f.bytes });
  }
  return root;
}

function treeRow(cls, indent, name, meta, href, title) {
  const row = document.createElement('div');
  row.className = 'node' + (cls ? ' ' + cls : '');
  row.style.paddingLeft = `${.3 + indent * .8}rem`;

  const tw = document.createElement('span');
  tw.className = 'twist';
  const nm = document.createElement('span');
  nm.className = 'name';
  nm.textContent = name;
  const mt = document.createElement('span');
  mt.className = 'meta';
  mt.textContent = meta;
  row.append(tw, nm, mt);

  if (href) {
    const dl = document.createElement('a');
    dl.className = 'dl';
    dl.href = href;
    dl.textContent = '⤓';
    dl.title = title;
    dl.onclick = (e) => e.stopPropagation();   // 别顺手把这一行也选中/折叠了
    row.appendChild(dl);
  }
  return row;
}

/** 一次录制底下的目录树。目录整个打包下，文件单独下。 */
function subtree(id, node, prefix, indent) {
  const box = document.createElement('div');
  for (const [name, child] of [...node.dirs].sort((a, b) => a[0].localeCompare(b[0]))) {
    const path = prefix ? `${prefix}/${name}` : name;
    const key = `${id}/${path}`;
    const open = OPEN.has(key);
    const row = treeRow('dir' + (open ? ' open' : ''), indent, name, size(child.bytes),
                        zipURL(id, path), `打包下载 ${name}/（${size(child.bytes)}）`);
    const kids = subtree(id, child, path, indent + 1);
    kids.className = 'kids' + (open ? ' open' : '');
    row.onclick = () => {
      OPEN.has(key) ? OPEN.delete(key) : OPEN.add(key);
      row.classList.toggle('open');
      kids.classList.toggle('open');
    };
    box.append(row, kids);
  }
  for (const f of node.files) {
    const picked = FILE && FILE.id === id && FILE.path === f.path;
    const row = treeRow('file' + (picked ? ' active' : ''), indent, f.name,
                        size(f.bytes), rawURL(id, f.path),
                        `下载 ${f.name}（${size(f.bytes)}）`);
    // 能看的就在右边开；mkv/bin 点了也没用，只能走 ⤓
    if (TEXTY.test(f.name)) {
      row.onclick = () => openFile(id, f.path);
    } else {
      row.classList.add('mute');
      row.title = '二进制文件，只能下载';
    }
    box.appendChild(row);
  }
  return box;
}

function renderSessions(list) {
  const box = $('sessions');
  const total = list.reduce((a, s) => a + s.bytes, 0);
  $('count').textContent = `${list.length} 次 · 共 ${mb(total)}`;
  // 每秒重建会把滚动位置、悬停和展开状态都打断，内容没变就别动
  const sig = JSON.stringify(list) + '|' + PICKED + '|' + [...OPEN].sort().join()
            + '|' + Object.keys(RAW).sort().join() + '|' + NOW.session + PLAYING;
  if (sig === SIG) return;
  SIG = sig;
  if (!list.length) { box.innerHTML = '<p class="hint">还没有采集数据。</p>'; return; }

  box.innerHTML = '';
  for (const s of list) {
    const key = `${s.id}/`;
    const open = OPEN.has(key);
    let cls = 'root' + (s.id === PICKED ? ' active' : '') + (open ? ' open' : '');
    if (PLAYING && NOW.session === s.id) cls += ' playing';
    const meta = `${s.episodes} 条 · ${mb(s.bytes)}` + (s.sealed ? '' : ' · 未封口');
    const row = treeRow(cls, 0, s.id, meta,
                        zipURL(s.id, ''), `打包下载整次采集（${mb(s.bytes)}）`);
    const kids = document.createElement('div');
    kids.className = 'kids' + (open ? ' open' : '');
    if (RAW[s.id]) kids.appendChild(subtree(s.id, buildTree(RAW[s.id].files), '', 1));
    else kids.innerHTML = '<p class="hint" style="padding:.2rem 1rem">正在列文件…</p>';

    // 顶层这一行同时干两件事：选中它（右边出详情）+ 展开文件树
    row.onclick = () => {
      const wasFile = FILE !== null;
      FILE = null;
      if (PICKED !== s.id) { PICKED = s.id; DETAIL = null; CHOICE = ''; loadDetail(); }
      else if (wasFile) loadDetail();   // 从文件预览切回来才重拉，否则每次折叠都重取 6 张预览帧
      OPEN.has(key) ? OPEN.delete(key) : OPEN.add(key);
      if (!RAW[s.id]) loadRaw(s.id);
      SIG = '';
      renderSessions(list);
    };
    box.append(row, kids);
  }
}


function frameImg(sid, stream, t) {
  const img = document.createElement('img');
  img.className = 'shot';
  img.loading = 'lazy';
  img.alt = stream;
  img.src = `/api/frame?id=${encodeURIComponent(sid)}&stream=${stream}&t=${t.toFixed(2)}`;
  img.title = `${stream} @ +${t.toFixed(1)}s`;
  return img;
}

function shots(sid, streams, at) {
  const row = document.createElement('div');
  row.className = 'shots';
  if (!streams.length) {
    row.innerHTML = '<span class="hint">这次没录视频</span>';
    return row;
  }
  for (const s of streams) row.appendChild(frameImg(sid, s, at));
  return row;
}

function playBtn(label, sid, t0, t1) {
  const b = document.createElement('button');
  b.textContent = '回放';
  b.disabled = PLAYING;
  b.onclick = async () => {
    try {
      const r = await post('/api/replay/start', {
        session: sid, t0, t1, label, speed: parseFloat($('speed').value),
      });
      banner(`回放 ${label} · 约 ${r.result.duration}s`, 'live');
      refresh();
    } catch (err) { banner('起不来：' + err.message, 'bad'); }
  };
  return b;
}

/** 只改按钮可用性与高亮。不能重跑 renderDetail —— 那会重拉 6 张预览帧，每张一个 ffmpeg */
function syncDetail() {
  for (const el of document.querySelectorAll('#detail .card')) {
    el.classList.toggle('playing',
      PLAYING && NOW.session === PICKED && NOW.label === el.dataset.label);
    const b = el.querySelector('.top button');
    if (b) b.disabled = PLAYING;
  }
}

/** 转换区。原始文件在左边的树里下，这里只管「转成别的格式再下」。 */
function convertBox(d) {
  const box = document.createElement('div');
  box.className = 'export';
  const usable = FORMATS.filter((f) => !f.missing.length);
  if (!CHOICE) CHOICE = (usable[0] || FORMATS[0] || {}).id || '';
  if (!FORMATS.length) return box;

  const label = document.createElement('span');
  label.className = 'hint';
  label.textContent = '转换成';
  const pick = document.createElement('select');
  for (const f of FORMATS) {
    const o = document.createElement('option');
    o.value = f.id;
    o.textContent = f.label + (f.missing.length ? `（缺 ${f.missing.join('、')}）` : '');
    o.disabled = f.missing.length > 0;
    o.title = f.note;
    pick.appendChild(o);
  }
  pick.value = CHOICE;
  pick.onchange = () => { CHOICE = pick.value; renderDetail(d); };
  box.append(label, pick);

  const busy = CONVERT.running;
  const mine = CONVERT.session === d.id;
  const b = document.createElement('button');
  b.className = 'primary';
  // 一次只跑一个（转换要抢 3.8 个核），所以别的采集在转时这里也得锁上，
  // 但文案要说清是「别人占着」而不是「你点的这个在跑」
  b.textContent = !busy ? '开始转换' : (mine ? '转换中…' : '有别的在转');
  b.disabled = !d.sealed || busy || !CHOICE;
  b.title = !d.sealed ? '没封口的不给转'
    : (busy && !mine ? `${CONVERT.session} 正在转，一次只能跑一个`
                     : '在服务器上转好，然后下载');
  b.onclick = async () => {
    try {
      await post('/api/convert/start', { session: d.id, format: CHOICE });
      banner(`${d.id} 开始转换`);
      refresh();
    } catch (err) {
      banner('转换起不来：' + err.message, 'bad');
    }
  };
  box.appendChild(b);

  if (busy && mine) {
    const pct = document.createElement('span');
    pct.className = 'pct';
    pct.textContent = `${Math.round((CONVERT.progress || 0) * 100)}%`;
    const tip = document.createElement('span');
    tip.className = 'hint';
    tip.textContent = '在服务器上跑，关掉这页也不会停';
    const bar = document.createElement('div');
    bar.className = 'bar';
    bar.innerHTML = `<div style="width:${((CONVERT.progress || 0) * 100).toFixed(1)}%"></div>`;
    const pre = document.createElement('pre');
    pre.className = 'log';
    pre.textContent = CONVERT.log.slice(-6).join('\n') || '启动中…';
    box.append(pct, tip, bar, pre);
  }
  if (mine && CONVERT.error) {
    const p = document.createElement('p');
    p.className = 'hint warn';
    p.textContent = CONVERT.error;
    box.appendChild(p);
  }
  if (mine && CONVERT.done && CONVERT.token) {
    const a = document.createElement('a');
    a.className = 'btn';
    a.href = `/bundle.zip?token=${encodeURIComponent(CONVERT.token)}`;
    a.textContent = `下载 ${size(CONVERT.bytes || 0)}`;
    const tip = document.createElement('span');
    tip.className = 'hint';
    tip.textContent = '转换结果不留在服务器上，这个链接下完即失效，要再拿就再转一次';
    box.append(a, tip);
  }
  return box;
}

function deleteBtn(d) {
  const b = document.createElement('button');
  b.className = 'danger';
  b.textContent = '删除';
  b.disabled = !d.sealed;
  b.title = d.sealed ? '' : '没封口的不给删';
  b.onclick = async () => {
    if (!confirm(`删除 ${d.id}？\n\n${mb(d.bytes || 0)}，删了不可恢复。`)) return;
    try {
      const r = await post('/api/session/delete', { session: d.id, confirm: d.id });
      banner(`已删除 ${r.result.id}（${mb(r.result.bytes)}）`);
      PICKED = null; DETAIL = null;
      $('detail').innerHTML = '<p class="hint">左边选一次采集</p>';
      refresh();
    } catch (err) {
      banner('删除失败：' + err.message, 'bad');
    }
  };
  return b;
}

/** 点了树里的文本文件：右边整块换成预览，不再显示回放/删除/转换。 */
async function openFile(id, path) {
  FILE = { id, path };
  SIG = '';
  const box = $('detail');
  box.innerHTML = '<p class="hint">读取中…</p>';
  let data;
  try {
    data = await get(`/api/preview?id=${encodeURIComponent(id)}&file=${encodeURIComponent(path)}`);
  } catch (err) {
    box.innerHTML = `<p class="hint warn">读不出：${err.message}</p>`;
    return;
  }
  if (!FILE || FILE.path !== path) return;      // 期间又点了别处

  box.innerHTML = '';
  const wrap = document.createElement('div');
  wrap.className = 'preview';

  const head = document.createElement('div');
  head.className = 'dhead';
  const title = document.createElement('span');
  title.className = 'title';
  title.textContent = path;
  const meta = document.createElement('span');
  meta.className = 'hint';
  meta.textContent = size(data.bytes) + (data.truncated ? ' · 只显示开头一部分' : '');
  const dl = document.createElement('a');
  dl.className = 'btn spacer';
  dl.href = rawURL(id, path);
  dl.textContent = '下载';
  head.append(title, meta, dl);

  const pre = document.createElement('pre');
  pre.textContent = data.binary ? '二进制文件，只能下载。' : data.text;
  wrap.append(head, pre);
  box.appendChild(wrap);
}

function renderDetail(d) {
  const box = $('detail');
  box.innerHTML = '';

  const head = document.createElement('div');
  head.className = 'dhead';
  const title = document.createElement('span');
  title.className = 'title';
  title.textContent = d.id;
  const note = document.createElement('span');
  note.className = 'hint';
  note.textContent = d.note || '';        // 备注是操作者手打的，不能当 HTML 插
  const kill = deleteBtn(d);
  kill.classList.add('spacer');           // 破坏性操作推到最右，手滑点不到
  head.append(title, note, kill);
  box.appendChild(head);
  box.appendChild(convertBox(d));

  // 读不出的那一次恰恰最该能删掉，所以错误提示放在删除按钮之后而不是取而代之
  if (d.error) {
    const p = document.createElement('p');
    p.className = 'hint warn';
    p.textContent = d.error;
    box.appendChild(p);
    return;
  }

  const list = document.createElement('div');
  list.className = 'eplist';
  box.appendChild(list);
  list.appendChild(epCard(d, {
    label: '整段', kind: '', duration: d.whole.duration,
    extra: `${d.whole.commands} 指令`,
    instruction: '从头到尾，包含没标注成 episode 的那些部分',
    t0: d.whole.t0, t1: d.whole.t1, at: 1.0,
  }));

  for (const e of d.episodes) {
    list.appendChild(epCard(d, {
      label: e.label, kind: e.outcome, duration: e.duration, extra: '',
      instruction: e.instruction, t0: e.t0, t1: e.t1,
      // 取 episode 中点那一帧：开头往往手还没进画面
      at: (e.t0 + e.t1) / 2 - d.whole.t0,
    }));
  }
  if (!d.episodes.length) {
    const p = document.createElement('p');
    p.className = 'hint';
    p.textContent = '这次采集没有标注 episode，只能整段回放。';
    box.appendChild(p);
  }
}

const OUTCOME = {
  success: { cls: 'ok', mark: '✓ 成功' },
  fail: { cls: 'bad', mark: '✗ 失败' },
  discard: { cls: 'off', mark: '− 已弃' },
};

function epCard(d, e) {
  const state = OUTCOME[e.kind] || {};
  const card = document.createElement('div');
  card.className = 'card' + (state.cls ? ' ' + state.cls : '');
  card.dataset.label = e.label;

  const top = document.createElement('div');
  top.className = 'top';
  const who = document.createElement('span');
  who.className = 'who';
  who.textContent = e.label;
  const dur = document.createElement('span');
  dur.className = 'dur';
  dur.textContent = [`${e.duration}s`, state.mark, e.extra].filter(Boolean).join(' · ');
  top.append(who, dur, playBtn(e.label, d.id, e.t0, e.t1));

  const say = document.createElement('div');
  say.className = 'say';
  say.textContent = e.instruction || '';
  card.append(top, say, shots(d.id, d.streams, e.at));
  return card;
}

async function loadDetail() {
  if (!PICKED) { $('detail').innerHTML = '<p class="hint">左边选一次采集</p>'; return; }
  $('detail').innerHTML = '<p class="hint">读取中…</p>';
  try {
    DETAIL = await get('/api/session?id=' + encodeURIComponent(PICKED));
    if (FILE) return;                    // 期间又点了文件，别把预览盖回去
    renderDetail(DETAIL);
    syncDetail();
  } catch (err) {
    $('detail').innerHTML = `<p class="hint">读不出：${err.message}</p>`;
  }
}

let LIVE = false;       // 上肢是否已接管 —— 决定底部那个按钮是 engage 还是急停
let LAST_PROG = 0;

function transport(st) {
  const ramping = st.phase === 'ramp';
  $('seek').className = 'bar' + (ramping ? ' ramp' : '');
  // 1 Hz 轮询，靠 CSS 过渡补成连续推进；进度回退时（缓入结束）不要倒着滑
  $('prog').style.transition = st.progress < LAST_PROG ? 'none' : '';
  $('prog').style.width = `${(st.progress * 100).toFixed(1)}%`;
  LAST_PROG = st.progress;

  $('now-label').textContent = st.playing
    ? `${ramping ? '缓入中' : '回放中'} · ${st.label || '整段'}`
    : (st.session ? `已停 · ${st.label || '整段'}` : '未在回放');
  $('clock').textContent = `${clock(st.elapsed)} / ${clock(st.duration)}`;
  $('ready').textContent = st.ready ? '就绪，选一段回放' : st.blocked;
  $('ready').className = 'hint' + (st.ready ? '' : ' warn');
  $('btn-stop').disabled = !st.playing;

  LIVE = st.arms_live;
  const fresh = st.status_age >= 0 && st.status_age <= 1;
  const b = $('btn-arm');
  b.textContent = LIVE ? '急停卸力' : '接管上肢';
  b.className = LIVE ? 'danger' : 'ok';
  // 已接管时永远能按急停 —— 哪怕状态话题哑了，那正是最需要它的时候
  b.disabled = !LIVE && !fresh;
  b.title = LIVE ? '立刻卸力，手臂会掉下来'
                 : (fresh ? '激活 FPC 并插值到站立位姿' : '控制层没有数据，控制栈起了吗');
}

async function refresh() {
  let data;
  try { data = await get('/api/state'); }
  catch (err) { banner('后端连不上：' + err.message, 'bad', true); return; }

  const st = data.status;
  const wasPlaying = PLAYING;
  PLAYING = st.playing;
  NOW = { session: st.session, label: st.label };
  CONVERT = data.convert || CONVERT;
  FORMATS = data.formats || FORMATS;
  renderSessions(data.sessions);
  transport(st);
  syncDetail();
  peerLink(st.peer_port, st.peer_alive);
  // 转换那一格得跟着进度动（进度条、日志滚动、完成后冒出下载链接），
  // 但别每秒无脑重建整个详情 —— 那会把缩略图闪掉、把展开的段落收回去
  const sig = [CONVERT.running, CONVERT.done, CONVERT.token, CONVERT.error,
               CONVERT.log.length, (CONVERT.progress || 0).toFixed(3)].join('|');
  if (sig !== CSIG) {
    CSIG = sig;
    if (DETAIL && !FILE) renderDetail(DETAIL);
  }

  if (st.error) banner('回放出错：' + st.error, 'bad', true);
  else if (st.playing) banner('回放中 —— 手放在急停上', 'live', true);
  else if (CONVERT.running) banner(`转换中：${CONVERT.session} → ${CONVERT.format}`, 'live', true);
  else if (wasPlaying) banner('回放结束，手臂停在原地', '', true);
  else banner(`盘余 ${st.disk_free_gb} GB`, '', true);
}

async function loadRaw(id) {
  try { RAW[id] = await get('/api/raw?id=' + encodeURIComponent(id)); }
  catch (err) { RAW[id] = { id, files: [], bytes: 0 }; }
  SIG = '';
  refresh();
}

$('btn-stop').onclick = async () => {
  try { await post('/api/replay/stop'); banner('已停止，手臂保持在原地'); refresh(); }
  catch (err) { banner(err.message, 'bad'); }
};

$('btn-arm').onclick = async () => {
  if (LIVE) {
    // 急停不加确认：要按它的时候没工夫再点一个弹窗
    try {
      const r = await post('/api/control/estop');
      banner('已急停：' + (r.result.message || 'ok'), 'bad');
    } catch (err) { banner('急停失败！' + err.message, 'bad'); }
  } else {
    if (!confirm('接管上肢会让机器人真的动：激活 FPC 并在 2.5 s 内插值到默认站立位姿。\n\n'
                + '只调手臂时机器人必须被吊起或另行支撑 —— 下肢只是被位置环钉住，没人在平衡。\n\n'
                + '确认周围无人、手臂没卡在腿上？')) return;
    banner('接管中，站立插值约 2.5 s…');
    try {
      const r = await post('/api/control/engage');
      banner('已接管：' + (r.result.message || 'ok'), 'live');
    } catch (err) { banner('接管失败：' + err.message, 'bad'); }
  }
  refresh();
};

refresh();
setInterval(refresh, 1000);
