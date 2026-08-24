'use strict';

let PICKED = null;      // 当前展开的 session id
let DETAIL = null;      // 它的详情，避免每次轮询都重拉
let PLAYING = false;
let NOW = { session: '', label: '' };   // 正在放哪一段，用于列表高亮
let SIG = '';           // 上一次渲染的列表指纹，没变就不重建 DOM

const clock = (s) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, '0')}`;

function renderSessions(list) {
  const box = $('sessions');
  const total = list.reduce((a, s) => a + s.bytes, 0);
  $('count').textContent = `${list.length} 次 · 共 ${mb(total)}`;
  // 每秒重建会把滚动位置和悬停都打断，内容没变就别动
  const sig = JSON.stringify(list) + '|' + PICKED;
  if (sig === SIG) return;
  SIG = sig;
  if (!list.length) { box.innerHTML = '<p class="hint">还没有采集数据。</p>'; return; }

  box.innerHTML = '';
  for (const s of list) {
    const div = document.createElement('div');
    div.className = 'ep' + (s.id === PICKED ? ' active' : '') + (s.sealed ? '' : ' done');
    const warn = s.warnings ? `<span class="tag">告警 ${s.warnings}</span>` : '';
    const seal = s.sealed ? '' : '<span class="tag pass">未封口</span>';
    div.innerHTML = `<div class="en">${s.id} ${seal}${warn}</div>
                     <div class="zh">${mb(s.bytes)} · ${s.episodes} 条
                       （成 ${s.success}） · ${s.commands} 指令</div>`;
    div.onclick = () => { PICKED = s.id; DETAIL = null; loadDetail(); };
    box.appendChild(div);
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
  for (const el of document.querySelectorAll('#detail .ep')) {
    el.classList.toggle('playing',
      PLAYING && NOW.session === PICKED && NOW.label === el.dataset.label);
    const b = el.querySelector('.acts button');
    if (b) b.disabled = PLAYING;
  }
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

function renderDetail(d) {
  const box = $('detail');
  box.innerHTML = '';

  const head = document.createElement('div');
  head.className = 'row';
  const title = document.createElement('b');
  title.textContent = d.id;
  const note = document.createElement('span');
  note.className = 'hint';
  note.textContent = d.note || '';        // 备注是操作者手打的，不能当 HTML 插
  head.append(title, note, deleteBtn(d));
  box.appendChild(head);

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

  const whole = document.createElement('div');
  whole.className = 'ep';
  whole.dataset.label = '整段';
  whole.innerHTML = `<div><span class="tag">整段</span>
                       <span class="tag">${d.whole.duration}s</span>
                       <span class="tag">${d.whole.commands} 指令</span></div>
                     <div class="acts"></div>`;
  whole.querySelector('.acts').appendChild(playBtn('整段', d.id, d.whole.t0, d.whole.t1));
  whole.appendChild(shots(d.id, d.streams, 1.0));
  list.appendChild(whole);

  if (!d.episodes.length) {
    const p = document.createElement('p');
    p.className = 'hint';
    p.textContent = '这次采集没有标注 episode，只能整段回放。';
    box.appendChild(p);
    return;
  }
  for (const e of d.episodes) {
    const div = document.createElement('div');
    const flag = { success: '✓', fail: '✗', discard: '−' }[e.outcome] || '';
    div.className = 'ep' + (e.outcome === 'discard' ? ' done' : '');
    div.dataset.label = e.label;
    div.innerHTML = `<div><span class="tag">${flag} ${e.label}</span>
                          <span class="tag">${e.duration}s</span></div>
                     <div class="en"></div>
                     <div class="acts"></div>`;
    div.querySelector('.en').textContent = e.instruction;
    div.querySelector('.acts').appendChild(playBtn(e.label, d.id, e.t0, e.t1));
    // 取 episode 中点那一帧：开头往往手还没进画面
    div.appendChild(shots(d.id, d.streams, (e.t0 + e.t1) / 2 - d.whole.t0));
    list.appendChild(div);
  }
}

async function loadDetail() {
  if (!PICKED) { $('detail').innerHTML = '<p class="hint">左边选一次采集</p>'; return; }
  $('detail').innerHTML = '<p class="hint">读取中…</p>';
  try {
    DETAIL = await get('/api/session?id=' + encodeURIComponent(PICKED));
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
  renderSessions(data.sessions);
  transport(st);
  syncDetail();
  peerLink(st.peer_port);

  if (st.error) banner('回放出错：' + st.error, 'bad', true);
  else if (st.playing) banner('回放中 —— 手放在急停上', 'live', true);
  else if (wasPlaying) banner('回放结束，手臂停在原地', '', true);
  else banner(`盘余 ${st.disk_free_gb} GB`, '', true);
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
