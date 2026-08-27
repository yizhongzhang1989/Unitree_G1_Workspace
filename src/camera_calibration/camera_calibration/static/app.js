'use strict';

let STATE = null;
let SIG = '';               // 相机/档位的结构指纹，没变就不重建 DOM
const PICKED = new Set();   // 勾选要拍摄的相机
const PROFILE = {};         // 每个相机当前选中的档位，轮询不能把它冲掉
let TAB = 'intrinsic';

// ---------- tab ----------

document.querySelectorAll('.tab').forEach((btn) => {
  btn.onclick = () => {
    TAB = btn.dataset.tab;
    document.querySelectorAll('.tab').forEach((b) => b.classList.toggle('active', b === btn));
    $('tab-intrinsic').hidden = TAB !== 'intrinsic';
    $('tab-extrinsic').hidden = TAB !== 'extrinsic';
    render();
  };
});

// ---------- 轮询 ----------

async function refresh() {
  let data;
  try {
    data = await get('/api/state');
  } catch (err) {
    banner('拿不到状态：' + err.message, 'bad', true);
    return;
  }
  STATE = data;
  // 渲染的错要和取数的错分开报，不然前端的 bug 会伪装成后端连不上
  try {
    render();
  } catch (err) {
    banner('渲染出错：' + err.message + ' @ '
      + (err.stack || '').split('\n')[1], 'bad', true);
  }
}

function render() {
  if (!STATE) return;
  const b = STATE.board;
  $('board-info').textContent =
    `${b.squares_x}×${b.squares_y} 格 · ${(b.square_size * 1000).toFixed(1)}mm/`
    + `${(b.marker_size * 1000).toFixed(1)}mm · ${b.dictionary} · ${b.corner_count} 角点`;
  $('calib-path').textContent = STATE.calib_file;
  if (TAB === 'intrinsic') renderCameras(); else renderExtrinsic();
}

// ---------- 内参 tab ----------

function renderCameras() {
  const sig = STATE.cameras.map((c) =>
    c.name + ':' + c.profiles.map((p) => profileKey(p.width, p.height)).join(',')).join('|');
  if (sig !== SIG) { SIG = sig; buildCameras(); }
  STATE.cameras.forEach(updateCamera);
}

// PROFILE 里只存 "WxH"，option 的 value 也必须是纯的 "WxH"（显示文字里的 ← 不能进去）
const wh = (name) => {
  const [width, height] = PROFILE[name].split('x').map(Number);
  return { width, height };
};

function buildCameras() {
  const host = $('cameras');
  host.innerHTML = '';
  STATE.cameras.forEach((cam) => {
    if (!PROFILE[cam.name]) {
      // 在推的分辨率不一定是声明过的档位（camera_node 可能被 launch 缩放过），
      // 不在 select 的选项里就会出现「显示的和选中的不是一个」
      const live = profileKey(cam.width, cam.height);
      const known = cam.profiles.some((p) => profileKey(p.width, p.height) === live);
      PROFILE[cam.name] = known ? live : profileKey(cam.profiles[0].width,
                                                    cam.profiles[0].height);
    }
    PICKED.add(cam.name);

    const card = document.createElement('div');
    card.className = 'card';
    card.dataset.cam = cam.name;
    card.innerHTML = `
      <div class="cam-head">
        <span class="dot"></span>
        <label><input type="checkbox" class="pick" checked></label>
        <span class="title">${cam.label}</span>
        <span class="grow"></span>
        <select class="profile">${cam.profiles.map((p) => {
          const k = profileKey(p.width, p.height);
          return `<option value="${k}">${k}</option>`;
        }).join('')}</select>
        <button class="apply small" ${cam.switchable ? '' : 'disabled'}>切换</button>
      </div>
      <img class="preview" alt="${cam.label}">
      <div class="stats"></div>
      <div class="toolbar">
        <button class="solve small" data-done="求解完成">求解</button>
        <button class="save small" data-done="已保存">保存</button>
        <button class="drop small" data-done="已删除离群张">删离群</button>
        <button class="clear small danger" data-done="已清空">清空本档位</button>
      </div>
      <div class="shots thumbs"></div>
      <div class="result"></div>`;
    host.appendChild(card);

    const sel = card.querySelector('.profile');
    sel.value = PROFILE[cam.name];
    sel.onchange = () => { PROFILE[cam.name] = sel.value; render(); };
    card.querySelector('.pick').onchange = (e) => {
      e.target.checked ? PICKED.add(cam.name) : PICKED.delete(cam.name);
    };
    Object.entries({
      apply: '/api/profile',
      solve: '/api/solve_intrinsic',
      save: '/api/save_intrinsic',
      drop: '/api/drop_outliers',
      clear: '/api/clear_shots',
    }).forEach(([cls, api]) => {
      card.querySelector('.' + cls).onclick = (e) =>
        guard(e.target, () => post(api, { camera: cam.name, ...wh(cam.name) }));
    });
  });
}

function updateCamera(cam) {
  const card = document.querySelector(`[data-cam="${cam.name}"]`);
  if (!card) return;
  card.querySelector('.dot').classList.toggle('on', cam.online);

  const img = card.querySelector('.preview');
  const overlay = $('chk-overlay').checked ? 1 : 0;
  refreshPreview(img, `/api/preview?camera=${cam.name}&overlay=${overlay}&t=${Date.now()}`);

  const picked = wh(cam.name);
  const shot = (cam.shots || []).find(
    (s) => s.width === picked.width && s.height === picked.height) || {};
  const live = cam.online ? profileKey(cam.width, cam.height) : '离线';
  const mismatch = cam.online && live !== PROFILE[cam.name];
  card.querySelector('.stats').innerHTML = `
    <span>在推 <b class="${mismatch ? 'warnish' : ''}">${live}</b>${
      mismatch ? '（和选中档位不符，先点切换）' : ''}</span>
    <span>角点 <b class="${cam.corners >= cam.max_corners * 0.5 ? 'ok' : 'no'}">${
      cam.corners}</b>/${cam.max_corners}</span>
    <span>marker <b>${cam.markers}</b></span>
    <span>单张覆盖 <b>${(cam.coverage * 100).toFixed(0)}%</b></span>
    <span>已拍 <b>${shot.shots || 0}</b> 张</span>
    <span>已存 RMS <b>${shot.rms === null || shot.rms === undefined ? '—' : shot.rms}</b></span>`;

  renderShots(card.querySelector('.shots'), cam.name, picked, shot.names || []);
  renderResult(card.querySelector('.result'), cam.name, picked);
}

// 缩略图只在名单变了的时候重建，否则每秒重下一遍图、鼠标都悬不住
function renderShots(host, camera, picked, names) {
  if (!stale(host, `${picked.width}x${picked.height}:${names.join(',')}`)) return;
  host.innerHTML = '';
  if (!names.length) { host.innerHTML = '<span class="muted">还没拍</span>'; return; }
  names.forEach((name) => {
    const box = document.createElement('div');
    box.className = 'thumb';
    box.innerHTML = `
      <img loading="lazy" src="/api/shot?camera=${camera}&width=${picked.width}`
      + `&height=${picked.height}&name=${name}">
      <div class="cap">${name}</div>
      <button class="x" title="删掉这张">×</button>`;
    box.querySelector('.x').onclick = (e) =>
      guard(e.target, () => post('/api/delete_shot', { camera, ...picked, name }));
    host.appendChild(box);
  });
}

function renderResult(host, camera, picked) {
  const result = STATE.results[`${camera}:${picked.width}x${picked.height}`];
  if (!stale(host, JSON.stringify(result || null))) return;
  if (!result) { host.innerHTML = ''; return; }
  if (!result.ok) {
    host.innerHTML = `<div class="no">求解失败：${result.reason}</div>`;
    return;
  }
  const k = result.camera_matrix;
  const outliers = new Set(result.outliers || []);
  host.innerHTML = `
    <table>
      <tr><th>RMS</th><td>${result.rms} px</td>
          <th>用了</th><td>${result.images} 张</td></tr>
      <tr><th>fx / fy</th><td>${num(k[0], 2)} / ${num(k[4], 2)}</td>
          <th>cx / cy</th><td>${num(k[2], 2)} / ${num(k[5], 2)}</td></tr>
      <tr><th>畸变</th><td colspan="3">${vec(result.distortion_coefficients, 4)}</td></tr>
      <tr><th>FOV</th><td>${result.fov_deg[0]}° × ${result.fov_deg[1]}°</td>
          <th>整体覆盖</th><td class="${result.coverage > 0.6 ? 'ok' : 'warnish'}">${
            (result.coverage * 100).toFixed(0)}%</td></tr>
      ${result.vs_factory ? `<tr><th>vs 出厂</th><td colspan="3" class="${
        Math.abs(result.vs_factory.fx_pct) < 1 ? 'ok' : 'warnish'}">
        fx ${result.vs_factory.fx_pct}% · fy ${result.vs_factory.fy_pct}% ·
        cx ${result.vs_factory.cx_px}px · cy ${result.vs_factory.cy_px}px</td></tr>` : ''}
    </table>
    <table>
      <tr><th>图</th><th>角点</th><th>重投影 RMS</th><th>最大</th><th>覆盖</th></tr>
      ${result.per_image.map((r) => `<tr class="${outliers.has(r.name) ? 'bad' : ''}">
        <td>${r.name}</td><td>${r.corners}</td><td>${r.error_px}</td>
        <td>${r.max_px}</td><td>${(r.coverage * 100).toFixed(0)}%</td></tr>`).join('')}
    </table>`;
}

$('btn-capture').onclick = (e) => guard(e.target, async () => {
  const out = await post('/api/capture', { cameras: [...PICKED] });
  const good = out.saved.filter((s) => s.ok);
  const bad = out.saved.filter((s) => !s.ok);
  const why = bad.map((s) => `${s.camera}(${s.reason})`).join('、');
  if (!good.length) throw new Error('一张都没存下：' + why);
  // 只有部分相机看得见板是常态，别让成的那台看起来也失败了
  out._banner = bad.length
    ? [`已拍 ${good.map((s) => s.camera).join('、')}；跳过 ${why}`, 'warn']
    : [`已拍 ${good.map((s) => `${s.camera} ${s.corners} 角点`).join('、')}`, 'good'];
  return out;
});

$('btn-probe').onclick = (e) => guard(e.target, async () => {
  const camera = STATE.cameras.find((c) => PICKED.has(c.name) && c.online)
    || STATE.cameras.find((c) => c.online);
  if (!camera) throw new Error('没有在线的相机');
  const out = await post('/api/probe_board', { camera: camera.name });
  const panel = $('probe-panel');
  panel.hidden = false;
  $('probe-table').innerHTML = `
    <p class="muted">用 ${camera.label} 当前这一帧探测</p>
    <table>
      <tr><th>字典</th><th>格数</th><th>marker</th><th>角点</th><th>共面残差</th></tr>
      ${out.candidates.map((c, i) => `<tr class="${i === 0 && c.corners ? 'best' : ''}">
        <td>${c.dictionary}</td><td>${c.squares_x}×${c.squares_y}</td>
        <td>${c.markers ?? '—'}</td><td>${c.corners ?? 0}/${c.max_corners ?? '—'}</td>
        <td>${c.residual_px === null || c.residual_px === undefined
          ? (c.reason || '—') : c.residual_px + ' px'}</td></tr>`).join('')}
    </table>`;
  return out;
});

$('chk-overlay').onchange = render;

// ---------- 外参 tab ----------

function renderExtrinsic() {
  renderExtrinsicPreviews();
  renderPreflight();
  renderPoses();
  renderExtrinsicResult();
}

const INTRINSIC_LABEL = {
  calibrated: ['标定值', 'ok'],
  factory: ['出厂值', 'warnish'],
  none: ['无', 'no'],
};

// 外参阶段一定要看得见画面：板子摆歪了、被遮了、离得太远，光看角点数判断不了
function renderExtrinsicPreviews() {
  const host = $('ext-cameras');
  if (stale(host, STATE.cameras.map((c) => c.name).join('|'))) {
    host.innerHTML = STATE.cameras.map((cam) => `
      <div class="card" data-ext="${cam.name}">
        <div class="cam-head">
          <span class="dot"></span>
          <span class="title">${cam.label}</span>
          <span class="grow"></span>
          <span class="muted">${cam.role === 'reference' ? '参考（定板）' : cam.parent_frame}</span>
        </div>
        <img class="preview" alt="${cam.label}">
        <div class="stats"></div>
      </div>`).join('');
  }
  STATE.cameras.forEach((cam) => {
    const card = host.querySelector(`[data-ext="${cam.name}"]`);
    if (!card) return;
    card.querySelector('.dot').classList.toggle('on', cam.online);
    refreshPreview(card.querySelector('.preview'),
                   `/api/preview?camera=${cam.name}&overlay=1&t=${Date.now()}`);
    const [label, cls] = INTRINSIC_LABEL[cam.intrinsic_source] || INTRINSIC_LABEL.none;
    const enough = cam.corners >= 6;
    card.querySelector('.stats').innerHTML = `
      <span>${cam.online ? profileKey(cam.width, cam.height) : '离线'}</span>
      <span>角点 <b class="${enough ? 'ok' : 'no'}">${cam.corners}</b>/${cam.max_corners}</span>
      <span>内参 <b class="${cls}">${label}</b></span>`;
  });
}

function renderPreflight() {
  const rows = [];
  const motion = STATE.motion;
  rows.push([motion.ok, '手臂静止',
    motion.ok ? `已静止 ${motion.still_s}s（${motion.speed} rad/s）` : motion.reason]);

  const tf = STATE.tf;
  Object.entries(tf.links).forEach(([name, link]) => {
    rows.push([link.ok, `TF ${tf.base_frame} → ${link.frame}`,
      link.ok ? '可用' : '查不到，whole body 起了吗']);
  });

  STATE.cameras.forEach((cam) => {
    rows.push([cam.online, `${cam.label} 在线`,
      cam.online ? `${cam.width}×${cam.height}` : '收不到图']);
    rows.push([cam.intrinsic_source !== 'none', `${cam.label} 内参`,
      { calibrated: '用已标定的值',
        factory: '当前档位没标过，用相机自报的出厂值',
        none: '当前档位既没标过也没出厂值，先去内参 tab',
      }[cam.intrinsic_source]]);
    rows.push([cam.corners >= 6, `${cam.label} 看得见板`,
      `${cam.corners} 个角点`]);
  });

  const sig = JSON.stringify(rows);
  const host = $('preflight');
  if (!stale(host, sig)) return;
  host.innerHTML = '<h3>采集前检查</h3><div class="checks">'
    + rows.map(([ok, what, detail]) =>
      `<div><span class="${ok ? 'ok' : 'no'}">${ok ? '✓' : '✗'}</span>
       <b>${what}</b><span class="muted">${detail}</span></div>`).join('')
    + '</div>';
}

function renderPoses() {
  const host = $('pose-list');
  if (!stale(host, JSON.stringify(STATE.poses))) return;
  if (!STATE.poses.length) {
    host.innerHTML = '<h3>已采姿态</h3><p class="muted">还没采</p>';
    return;
  }
  host.innerHTML = `<h3>已采姿态（${STATE.poses.length}）</h3>
    <table>
      <tr><th>名字</th><th>时间</th><th>头部角点</th><th>头部内参</th>
          <th>腕相机角点</th><th></th></tr>
      ${STATE.poses.map((p) => `<tr>
        <td>${p.name}</td><td class="muted">${p.stamp.replace('T', ' ')}</td>
        <td>${p.reference_corners}</td>
        <td class="${p.reference_intrinsic === 'factory' ? 'warnish' : ''}">${
          (INTRINSIC_LABEL[p.reference_intrinsic] || ['—'])[0]}</td>
        <td>${Object.entries(p.targets).map(([k, v]) => `${k} ${v}`).join(' · ') || '—'}</td>
        <td><button class="small danger del" data-name="${p.name}">删</button></td>
      </tr>`).join('')}
    </table>`;
  host.querySelectorAll('.del').forEach((btn) => {
    btn.onclick = (e) => guard(e.target,
      () => post('/api/delete_pose', { name: btn.dataset.name }));
  });
}

function renderExtrinsicResult() {
  const results = STATE.results.extrinsic;
  const host = $('extrinsic-result');
  if (!stale(host, JSON.stringify(results || null))) return;
  if (!results) { host.innerHTML = ''; return; }

  host.innerHTML = combinedBlock(results.combined)
    + Object.values(results.cameras || {}).map(cameraBlock).join('');
  host.querySelectorAll('.save-ext').forEach((btn) => {
    btn.onclick = (e) => guard(e.target, () => post('/api/save_extrinsic',
      { camera: btn.dataset.camera, method: btn.dataset.method }));
  });
  host.querySelectorAll('.save-ref').forEach((btn) => {
    btn.onclick = (e) => guard(e.target, () => post('/api/save_reference', {}));
  });
}

// ΔH 是三个相机共享的未知量，一起解才只有一个答案；分开解会得到两个不一样的值
function combinedBlock(all) {
  if (!all) return '';
  if (!all.ok) {
    return `<div class="card"><h3>三相机联合解</h3>
      <span class="no">${all.reason}</span></div>`;
  }
  const fix = all.reference_correction;
  const abs = all.reference_absolute;
  return `<div class="card">
    <h3>三相机联合解（推荐）</h3>
    <p class="hint">头部外参偏差 ΔH 当共享未知量，两只手臂的运动一起约束它 ——
      所以这里的腕相机外参不受头部准不准影响，头部外参本身也一并解了出来。</p>
    <table>
      <tr><th>最小二乘残差</th>
        <td class="${all.residual_mm < 10 ? 'ok' : 'warnish'}">
          ${all.residual_mm} mm rms（优化前 ${all.residual_mm_before}）</td></tr>
      <tr><th>头部外参偏差</th>
        <td class="${!all.well_posed ? 'no' : (fix.angle_deg < 2 ? 'ok' : 'warnish')}">
          ${fix.angle_deg}° / ${fix.trans_mm} mm${all.well_posed ? ''
            : `（条件数 ${all.condition}，姿态激励不够，别当真）`}</td></tr>
      <tr><th>头部外参（修正后 xyz / rpy°）</th>
        <td>${vec(abs.translation)}　|　${degs(abs.rpy)}</td></tr>
      <tr><th class="muted">URDF 里的名义值</th>
        <td class="muted">${vec(all.reference_nominal.translation)}　|　${
          degs(all.reference_nominal.rpy)}</td></tr>
    </table>
    ${urdfBlock(all.urdf)}
    ${Object.entries(all.cameras).map(([name, cam]) => {
      const t = cam.transform;
      return `<p><b>${cam.label} → ${cam.parent}</b>（${cam.samples} 组）
        <button class="small save-ext" data-camera="${name}" data-method="all"
          data-done="已保存">保存</button><br>
        <span class="muted">xyz ${vec(t.translation)}　|　
        rpy ${degs(t.rpy)}°　|　
        板位姿残差 ${num(cam.consistency.trans_rms_mm, 2)} mm rms</span>
        ${targetUrdfLine(cam.urdf)}</p>`;
    }).join('')}
  </div>`;
}

// 保存时会顺手往 urdf_overrides 里插一条边，控制栈启动时就跟头部一样
// 由 robot_state_publisher 发 TF。
function targetUrdfLine(urdf) {
  if (!urdf) return '';
  if (urdf.error) return `<br><span class="no">插不进 URDF：${urdf.error}</span>`;
  return `<br><span class="muted">存成 URDF 边 <code>${urdf.joint}</code>：
    ${urdf.parent} → ${urdf.child}</span>`;
}

// 头部不能像腕相机那样发 static TF：camera_color_optical_frame 已经有 URDF +
// realsense-ros 在发了，再发一份就是两个 publisher 抢同一个 child。
function urdfBlock(urdf) {
  if (!urdf) return '<p class="muted">没配 mount_joint，算不出要改哪个关节</p>';
  if (urdf.error) return `<p class="no">算不出关节 origin：${urdf.error}</p>`;
  return `<p><b>落地方式：覆盖 URDF 关节 <code>${urdf.joint}</code></b>
      （${urdf.parent} → ${urdf.child}）
      <button class="small save-ref" data-done="已保存">保存头部外参</button></p>
    <p class="hint">存进 <code>urdf_overrides</code>，控制栈 launch 展开 URDF 时叠加，
      <b>不动 submodule 里的 final.urdf</b>，也不发 static TF（那个 frame 已经有人在发了）。
      不想用就 <code>use_camera_calibration:=false</code>。</p>
    <pre class="mat">rpy°  ${degs(urdf.previous_rpy, 4)}   →   ${degs(urdf.rpy, 4)}
xyz   ${vec(urdf.previous_xyz, 6)}   →   ${vec(urdf.xyz, 6)}</pre>`;
}

function cameraBlock(entry) {
  const cross = entry.cross_check;
  const bias = entry.reference_bias;
  const agree = cross && cross.angle_deg < 2 && cross.trans_mm < 10;
  const board = entry.board_stability || {};
  return `<div class="card">
    <h3>${entry.label} → ${entry.parent}（${entry.samples} 组）</h3>
    <p class="${board.fixed ? 'muted' : 'warnish'}">
      板在各组之间最大差 ${num(board.angle_max_deg, 2)}° / ${num(board.trans_max_mm, 1)} mm ——
      ${board.fixed ? '板没挪过，三条路都可用'
        : '板被挪过。联合最小二乘和参考相机法照常（头部每组重测一次板），'
          + 'AX=XB 用不了 —— 它不看头部，板一动每多一组就多 6 个未知量、也只多 6 个方程'}
    </p>
    ${solutionTable('联合最小二乘（板可动，且不依赖头部外参准不准）', entry.joint, entry.camera, 'joint')}
    ${solutionTable('参考相机法（逐组解再取平均，头部偏多少它就偏多少）', entry.reference, entry.camera, 'reference')}
    ${solutionTable('AX=XB 手眼（不用头部外参，但要求板全程不动）', entry.handeye, entry.camera, 'handeye')}
    ${cross ? `<p class="${agree ? 'ok' : 'no'}">
      参考相机法 vs AX=XB：${cross.angle_deg}° / ${cross.trans_mm} mm
      ${agree ? '—— 一致，头部外参可信'
        : `—— 超出 2°/10mm。说明 URDF 里的头部外参不准；
           按 AX=XB 反推，头部该往回修 ${bias ? bias.delta.angle_deg + '° / '
           + bias.delta.trans_mm + ' mm' : '—'}`}</p>` : ''}
  </div>`;
}

function solutionTable(title, solution, camera, method) {
  if (!solution || !solution.ok) {
    return `<p><b>${title}</b> <span class="no">${solution ? solution.reason : '无解'}</span></p>`;
  }
  const t = solution.transform;
  const c = solution.consistency;
  const spread = solution.spread;
  const fix = solution.reference_correction;
  return `<p><b>${title}</b>${solution.best ? `（取 ${solution.best}）` : ''}
      <button class="small save-ext" data-camera="${camera}" data-method="${method}"
        data-done="已保存">保存这一组</button></p>
    <table>
      <tr><th>xyz (m)</th><td>${vec(t.translation)}</td></tr>
      <tr><th>rpy (°)</th><td>${degs(t.rpy)}</td></tr>
      <tr><th>quat xyzw</th><td>${vec(t.rotation)}</td></tr>
      ${solution.residual_mm !== undefined ? `<tr><th>最小二乘残差</th>
        <td class="${solution.residual_mm < 10 ? 'ok' : 'warnish'}">
          ${solution.residual_mm} mm rms（优化前 ${solution.residual_mm_before}）</td></tr>` : ''}
      ${fix ? `<tr><th title="把头部外参的修正量当自由参数一起解，于是上面的 X 不再受头部误差影响">
        头部外参偏差（同时解出）</th>
        <td class="${!solution.well_posed ? 'no' : (fix.angle_deg < 2 ? 'ok' : 'warnish')}">
          ${fix.angle_deg}° / ${fix.trans_mm} mm${solution.well_posed ? ''
            : `（条件数 ${solution.condition}，这批姿态撑不起 12 个未知量，别当真）`}
        </td></tr>` : ''}
      <tr><th title="腕相机反推的板位姿，逐组和头部实测的比。板动没动都成立">
        板位姿残差（vs 头部）</th>
        <td class="${c.trans_rms_mm < 5 ? 'ok' : 'warnish'}">
          ${num(c.angle_rms_deg, 3)}° rms / ${num(c.trans_rms_mm, 2)} mm rms
          （最大 ${num(c.angle_max_deg, 3)}° / ${num(c.trans_max_mm, 2)} mm）</td></tr>
      ${spread ? `<tr><th>各姿态单独解的离散</th>
        <td class="${spread.angle_rms_deg < 1 ? 'ok' : 'warnish'}">
          ${num(spread.angle_rms_deg, 3)}° rms / ${num(spread.trans_rms_mm, 2)} mm rms</td></tr>`
        : ''}
    </table>`;
}

$('btn-capture-pose').onclick = (e) =>
  guard(e.target, () => post('/api/capture_pose', {}));
$('btn-solve-extrinsic').onclick = (e) =>
  guard(e.target, () => post('/api/solve_extrinsic', {}));
$('btn-clear-poses').onclick = (e) =>
  guard(e.target, () => post('/api/clear_poses', {}));

refresh();
setInterval(refresh, 1000);
