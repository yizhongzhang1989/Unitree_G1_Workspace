const ui = {};
let state = null;
let toastTimer = null;

const phaseNames = {
  idle: "等待状态",
  passive_capture: "被动采点",
  ready: "准备标定",
  preflight: "接管检查",
  complete: "标定完成",
  error: "标定错误"
};
const stageNames = {
  idle: "等待开始",
  preflight: "接管检查",
  imu_average: "IMU 稳定平均",
  release_motion: "释放运动模式",
  torque_ramp: "重力扭矩缓升",
  move: "纯扭矩移动",
  settle: "等待实测姿态稳定",
  static_average: "同步平均姿态与 IMU",
  fit: "全部姿态统一辨识"
};

function cacheUi() {
  document.querySelectorAll("[id]").forEach(el => { ui[el.id] = el; });
}

function toast(message, error = false) {
  ui.toast.textContent = message;
  ui.toast.classList.toggle("error", error);
  ui.toast.classList.add("show");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => ui.toast.classList.remove("show"), 3200);
}

async function api(path, body = {}) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body)
  });
  const payload = await response.json();
  if (!response.ok || payload.ok === false) throw new Error(payload.message || "请求失败");
  return payload;
}

function shortJoint(name) {
  return name.replace("_joint", "").replace("left_", "L ").replace("right_", "R ");
}

function selectedJoints() {
  return [...document.querySelectorAll(".joint-check:checked")].map(input => input.value);
}

function updateArmWarning() {
  const joints = selectedJoints();
  const sides = [
    ...(joints.some(name => name.startsWith("left_")) ? ["左臂"] : []),
    ...(joints.some(name => name.startsWith("right_")) ? ["右臂"] : []),
  ];
  ui.armWarning.textContent = sides.length
    ? `启动后将运动：${sides.join(" + ")}（共 ${joints.length} 个关节）`
    : "未选择任何关节";
}

function buildJointSelection(snapshot) {
  const selected = new Set(snapshot.selected_joints);
  ["left", "right"].forEach(side => {
    const root = side === "left" ? ui.leftJoints : ui.rightJoints;
    const groups = snapshot.parameter_groups[side];
    root.innerHTML = groups.map(group => {
      const links = group.links.map(item => item.name).join(" · ");
      return `<label class="joint-row">
        <input class="joint-check" type="checkbox" value="${group.joint}" ${selected.has(group.joint) ? "checked" : ""}>
        <strong>${group.joint}</strong>
        <small>${links}</small>
      </label>`;
    }).join("");
  });
  [ui.xAxis, ui.yAxis].forEach((select, index) => {
    const previous = select.value;
    select.innerHTML = snapshot.joint_names.map((name, jointIndex) =>
      `<option value="${name}" ${(!previous && jointIndex === (index ? 3 : 0)) || previous === name ? "selected" : ""}>${shortJoint(name)}</option>`
    ).join("");
  });
}

function updateFiles(snapshot) {
  ui.parameterPath.textContent = snapshot.files.parameter;
  ui.sourcePath.textContent = snapshot.files.source_urdf;
  ui.outputPath.textContent = snapshot.files.calibrated_urdf;
  ui.tablePath.textContent = snapshot.files.gravity_table;
  ui.frictionPath.textContent = snapshot.files.friction_table;
  ui.ftPath.textContent = snapshot.files.ft_calibration;
  ui.sourceHash.textContent = snapshot.files.source_sha256;
  ui.schemaValue.textContent = `schema v${snapshot.files.schema_version}`;
  ui.parameterState.textContent = "READY";
}

function updateRuntime(snapshot) {
  const runtime = snapshot.runtime;
  const age = runtime.lowstate_age;
  const online = age !== null && age < 0.5;
  ui.connectionDot.classList.toggle("online", online && runtime.phase !== "error");
  ui.connectionDot.classList.toggle("error", runtime.phase === "error");
  ui.phaseLabel.textContent = phaseNames[runtime.phase] || runtime.phase;
  ui.messageLabel.textContent = runtime.message;
  ui.stateAge.textContent = age === null ? "NO DATA" : `${age.toFixed(3)} s`;
  ui.modePr.textContent = String(runtime.mode_pr);
  ui.accelValue.textContent = runtime.accelerometer.map(value => value.toFixed(3)).join(" / ");
  ui.targetCount.textContent = String(snapshot.targets.length);
  ui.outputLock.textContent = runtime.lowcmd_active ? "LOWCMD TAU ACTIVE" : "LOWCMD OFF";
  ui.outputLock.classList.toggle("active", runtime.lowcmd_active);
  ui.lowcmdBadge.textContent = runtime.lowcmd_active ? "LOWCMD TAU ACTIVE" : "LOWCMD INACTIVE";
  ui.lowcmdBadge.classList.toggle("off", !runtime.lowcmd_active);

  const gravity = runtime.gravity;
  ui.gravityX.textContent = gravity[0].toFixed(3);
  ui.gravityY.textContent = gravity[1].toFixed(3);
  ui.gravityZ.textContent = gravity[2].toFixed(3);
  const norm = Math.hypot(...gravity);
  ui.gravityNorm.textContent = `|g| = ${norm.toFixed(3)} m/s²`;
  ui.gravityBar.style.width = `${Math.min(100, norm / 10.5 * 100)}%`;

  const progress = runtime.progress;
  const ratio = progress.total ? Math.min(1, progress.target / progress.total) : 0;
  ui.progressRing.style.setProperty("--progress", `${ratio * 360}deg`);
  ui.progressNumber.textContent = `${Math.round(ratio * 100)}%`;
  ui.progressStage.textContent = stageNames[progress.stage] || progress.stage;
  ui.progressTarget.textContent = progress.total ? `姿态 ${progress.target} / ${progress.total}` : "尚未运行";
  ui.progressSide.textContent = progress.side ? `${progress.side.toUpperCase()} ARM · SAMPLE ${progress.iteration}` : "-";

  const captureActive = runtime.phase === "passive_capture";
  const calibrationActive = ["preflight"].includes(runtime.phase) || runtime.lowcmd_active || ["imu_average", "move", "settle", "static_average", "fit", "release_motion", "torque_ramp"].includes(progress.stage);
  ui.startCapture.disabled = captureActive || calibrationActive;
  ui.capturePoint.disabled = !captureActive;
  ui.stopCapture.disabled = !captureActive;
  ui.startCalibration.disabled = calibrationActive || !online || !runtime.torque_output_allowed || snapshot.targets.length === 0 || selectedJoints().length === 0;
  updateArmWarning();  ui.stopCalibration.disabled = !calibrationActive;
}

function updateTargets(snapshot) {
  ui.targetRows.innerHTML = snapshot.targets.length ? snapshot.targets.map(target => {
    const values = snapshot.joint_names.map(name => target.positions[name]);
    const left = values.slice(0, 7);
    const right = values.slice(7);
    const range = list => `${Math.min(...list).toFixed(2)} … ${Math.max(...list).toFixed(2)}`;
    return `<tr>
      <td><code>#${target.id}</code></td>
      <td>${{ left: "左臂", right: "右臂" }[target.side] ?? "双臂"}</td>
      <td>${target.source}</td>
      <td>${target.captured_at.replace("T", " ").slice(0, 19)}</td>
      <td><code>${range(left)}</code></td><td><code>${range(right)}</code></td>
      <td><button class="delete-button" data-remove="${target.id}" title="删除姿态">×</button></td>
    </tr>`;
  }).join("") : `<tr><td class="empty-row" colspan="7">尚未记录标定姿态</td></tr>`;
  drawPlot(snapshot);
}

function drawPlot(snapshot) {
  const canvas = ui.posePlot;
  const context = canvas.getContext("2d");
  const scale = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(320, Math.round(rect.width * scale));
  const height = Math.max(260, Math.round(rect.height * scale));
  if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
  context.clearRect(0, 0, width, height);
  context.fillStyle = "#161c1a";
  context.fillRect(0, 0, width, height);
  const xName = ui.xAxis.value || snapshot.joint_names[0];
  const yName = ui.yAxis.value || snapshot.joint_names[3];
  const points = snapshot.targets.map(target => ({ id: target.id, x: target.positions[xName], y: target.positions[yName] }));
  const margin = 45 * scale;
  const xs = points.map(point => point.x);
  const ys = points.map(point => point.y);
  const xMin = points.length ? Math.min(...xs) : -1;
  const xMax = points.length ? Math.max(...xs) : 1;
  const yMin = points.length ? Math.min(...ys) : -1;
  const yMax = points.length ? Math.max(...ys) : 1;
  const xPad = Math.max(.15, (xMax - xMin) * .16);
  const yPad = Math.max(.15, (yMax - yMin) * .16);
  const mapX = value => margin + (value - xMin + xPad) / (xMax - xMin + 2 * xPad) * (width - 2 * margin);
  const mapY = value => height - margin - (value - yMin + yPad) / (yMax - yMin + 2 * yPad) * (height - 2 * margin);
  context.strokeStyle = "#34403c";
  context.lineWidth = scale;
  for (let index = 0; index <= 5; index += 1) {
    const x = margin + index / 5 * (width - 2 * margin);
    const y = margin + index / 5 * (height - 2 * margin);
    context.beginPath(); context.moveTo(x, margin); context.lineTo(x, height - margin); context.stroke();
    context.beginPath(); context.moveTo(margin, y); context.lineTo(width - margin, y); context.stroke();
  }
  context.strokeStyle = "#76d6ad";
  context.lineWidth = 2 * scale;
  if (points.length > 1) {
    context.beginPath();
    points.forEach((point, index) => index ? context.lineTo(mapX(point.x), mapY(point.y)) : context.moveTo(mapX(point.x), mapY(point.y)));
    context.stroke();
  }
  context.font = `${11 * scale}px IBM Plex Mono, monospace`;
  points.forEach(point => {
    const x = mapX(point.x); const y = mapY(point.y);
    context.fillStyle = "#55c394";
    context.beginPath(); context.arc(x, y, 5 * scale, 0, Math.PI * 2); context.fill();
    context.fillStyle = "#d8e3de"; context.fillText(`#${point.id}`, x + 8 * scale, y - 7 * scale);
  });
  context.fillStyle = "#9fa9a4";
  context.fillText(shortJoint(xName), margin, height - 14 * scale);
  context.save(); context.translate(14 * scale, height - margin); context.rotate(-Math.PI / 2); context.fillText(shortJoint(yName), 0, 0); context.restore();
  if (!points.length) {
    context.fillStyle = "#78847f"; context.textAlign = "center";
    context.fillText("等待采点", width / 2, height / 2); context.textAlign = "left";
  }
}

function updateParameters(snapshot) {
  const rows = [];
  ["left", "right"].forEach(side => snapshot.parameter_groups[side].forEach(group => group.links.forEach(link => {
    const observation = Number(link.identification.observability || 0);
    rows.push(`<tr>
      <td>${side.toUpperCase()}</td><td><code>${group.joint}</code></td><td><code>${link.name}</code></td>
      <td>${Number(link.scale).toFixed(6)}</td><td>${Number(link.mass).toFixed(6)}</td>
      <td><div class="observability"><i style="--value:${observation * 100}%"></i><span>${(observation * 100).toFixed(1)}%</span></div></td>
      <td><span class="source-label ${link.identification.source}">${link.identification.source}</span></td>
    </tr>`);
  })));
  ui.parameterRows.innerHTML = rows.join("");

  const iterations = [...snapshot.iterations].reverse();
  ui.iterationCount.textContent = `${iterations.length} rounds`;
  ui.iterationRows.innerHTML = iterations.length ? iterations.map(item => {
    const targets = Array.isArray(item.target_ids)
      ? item.target_ids.map(id => `#${id}`).join(" / ")
      : `#${item.target_id ?? "-"}`;
    return `<tr>
    <td>${String(item.timestamp || "-").replace("T", " ").slice(0, 19)}</td>
    <td>${String(item.side || "-").toUpperCase()}</td><td>${targets}</td>
    <td>${Number(item.rmse_before ?? 0).toFixed(6)}</td><td>${Number(item.rmse_after ?? 0).toFixed(6)}</td>
    <td>${item.rank ?? "-"}</td><td>${item.nullity ?? "-"}</td>
    <td>${Number.isFinite(item.condition_number) ? item.condition_number.toFixed(1) : "-"}</td>
    <td>${item.inlier_fraction === undefined ? "-" : `${(item.inlier_fraction * 100).toFixed(1)}%`}</td>
  </tr>`;
  }).join("") : `<tr><td class="empty-row" colspan="9">尚无标定迭代</td></tr>`;
}

async function refresh() {
  try {
    const response = await fetch("/api/state", { cache: "no-store" });
    if (!response.ok) throw new Error("状态接口不可用");
    const snapshot = await response.json();
    const firstLoad = state === null;
    state = snapshot;
    if (firstLoad) buildJointSelection(snapshot);
    updateFiles(snapshot); updateRuntime(snapshot); updateTargets(snapshot); updateParameters(snapshot);
    updateForceSensor(snapshot);
  } catch (error) {
    ui.connectionDot.classList.remove("online"); ui.connectionDot.classList.add("error");
    ui.phaseLabel.textContent = "连接断开"; ui.messageLabel.textContent = error.message;
  }
}

function updateForceSensor(snapshot) {
  const sensors = snapshot.ft_sensor || {};
  ui.ftPanels.innerHTML = ["left", "right"].map(side => {
    const sensor = sensors[side];
    if (!sensor) return "";
    const online = sensor.age !== null && sensor.age < 0.5;
    const result = sensor.result;
    const solved = result ? result.calibration : null;
    const diagnostics = result ? result.diagnostics : null;
    const coverage = sensor.coverage || { count: 0, spread: 0 };
    const rows = solved ? `
      <div><dt>工具质量</dt><dd>${solved.tool_mass.toFixed(4)} kg（模型 ${Number(diagnostics.modelled_tool_mass ?? 0).toFixed(4)}）</dd></div>
      <div><dt>质心 (m)</dt><dd>${solved.tool_com.map(v => v.toFixed(4)).join(" / ")}</dd></div>
      <div><dt>模型质心 (m)</dt><dd>${(diagnostics.modelled_tool_com || []).map(v => v.toFixed(4)).join(" / ") || "-"}</dd></div>
      <div><dt>取矩点 (m)</dt><dd>${(solved.measurement_origin || [0, 0, 0]).map(v => v.toFixed(4)).join(" / ")}</dd></div>
      <div><dt>建议取矩点<span class="hint" tabindex="0" role="button" aria-label="说明" data-tip="传感器对自己的力矩参考点取矩，厂家把它放在工具侧法兰面上，而不是 URDF link 原点。纯重力标定解不出它（只有“质心减取矩点”可辨识），所以这里用 CAD 质心反推：量级应该正好等于一个传感器高度（KWR57 为 53 mm），对得上才说明这个假设成立。"></span></dt><dd>${(diagnostics.suggested_origin || []).map(v => v.toFixed(4)).join(" / ") || "-"}</dd></div>
      <div><dt>力零偏 (N)</dt><dd>${solved.force_bias.map(v => v.toFixed(2)).join(" / ")}</dd></div>
      <div><dt>力矩零偏</dt><dd>${solved.torque_bias.map(v => v.toFixed(3)).join(" / ")}</dd></div>
      <div><dt>残差 RMS</dt><dd>${diagnostics.force_residual_rms.toFixed(3)} N · ${diagnostics.torque_residual_rms.toFixed(4)} N·m</dd></div>
      <div><dt>安装姿态</dt><dd>${diagnostics.orientation_estimated
        ? `已采纳 ${Number(diagnostics.misalignment_deg ?? 0).toFixed(2)}°`
        : `名义值 (p=${Number(diagnostics.orientation_probability ?? 1).toFixed(3)})`}</dd></div>
      <div><dt>轴增益偏差</dt><dd>${diagnostics.shape_error === undefined ? "样本不足" : `${(diagnostics.shape_error * 100).toFixed(1)}%`}</dd></div>
      <div><dt>内点</dt><dd>${(diagnostics.inlier_fraction * 100).toFixed(1)}%</dd></div>` : "";
    return `<div class="ft-card">
      <div class="ft-card-head">
        <h3>${side === "left" ? "左" : "右"} · ${sensor.frame}</h3>
        <span class="danger-badge ${online ? "off" : ""}">${online ? "ONLINE" : "NO DATA"}</span>
      </div>
      <code>${sensor.topic}</code>
      <dl class="metric-grid">
        <div><dt>原始读数</dt><dd>${sensor.wrench ? sensor.wrench.map(v => v.toFixed(2)).join(" ") : "-"}</dd></div>
        <div><dt>朝向覆盖</dt><dd>${coverage.count} 个 · spread ${Number(coverage.spread || 0).toFixed(3)}</dd></div>
        ${rows}
      </dl>
      ${result ? `<p class="ft-stamp">解算于 ${result.calibrated_at.replace("T", " ").slice(0, 19)}</p>` : "<p class=\"ft-stamp\">尚未求解</p>"}
    </div>`;
  }).join("");

  const samples = ["left", "right"].flatMap(side =>
    (sensors[side] ? sensors[side].samples : []).map(item => ({ ...item, side })));
  samples.sort((first, second) => first.id - second.id);
  ui.ftRows.innerHTML = samples.length ? samples.map(item => `<tr>
    <td>#${item.id}</td><td>${item.side === "left" ? "左" : "右"}</td>
    <td>${item.source === "manual" ? "手动" : "自动"}</td>
    <td>${item.captured_at.replace("T", " ").slice(0, 19)}</td>
    <td>${item.wrench.slice(0, 3).map(v => v.toFixed(2)).join(" / ")}</td>
    <td>${item.wrench.slice(3).map(v => v.toFixed(3)).join(" / ")}</td>
    <td>${Math.max(...item.wrench_std.slice(0, 3)).toFixed(2)} / ${Math.max(...item.wrench_std.slice(3)).toFixed(3)}</td>
    <td><button class="text-button" data-ft-remove="${item.id}">删除</button></td>
  </tr>`).join("") : `<tr><td class="empty-row" colspan="8">尚无力传感器样本</td></tr>`;
}

function bind() {
  document.querySelectorAll(".step").forEach(button => button.addEventListener("click", () => {
    document.getElementById(button.dataset.scroll).scrollIntoView();
  }));
  const chooseSide = side => document.querySelectorAll(`.joint-check[value^="${side}_"]`).forEach(input => { input.checked = true; });
  ui.selectLeft.addEventListener("click", () => chooseSide("left"));
  ui.selectRight.addEventListener("click", () => chooseSide("right"));
  ui.clearSelection.addEventListener("click", () => document.querySelectorAll(".joint-check").forEach(input => { input.checked = false; }));
  ui.startCapture.addEventListener("click", async () => {
    try { const result = await api("/api/capture/start", { selected_joints: selectedJoints(), automatic: ui.autoCapture.checked }); toast(result.message); await refresh(); }
    catch (error) { toast(error.message, true); }
  });
  ui.capturePoint.addEventListener("click", async () => {
    try { await api("/api/capture/point"); toast("已记录当前姿态"); await refresh(); }
    catch (error) { toast(error.message, true); }
  });
  ui.stopCapture.addEventListener("click", async () => {
    try { const result = await api("/api/capture/stop"); toast(result.message); await refresh(); }
    catch (error) { toast(error.message, true); }
  });
  ui.clearTargets.addEventListener("click", async () => {
    if (!confirm("删除全部已记录姿态？")) return;
    try { const result = await api("/api/targets/clear"); toast(result.message); await refresh(); }
    catch (error) { toast(error.message, true); }
  });
  ui.targetRows.addEventListener("click", async event => {
    const button = event.target.closest("[data-remove]"); if (!button) return;
    try { await api("/api/targets/remove", { id: Number(button.dataset.remove) }); await refresh(); }
    catch (error) { toast(error.message, true); }
  });
  ui.startCalibration.addEventListener("click", async () => {
    try { const result = await api("/api/calibration/start", { confirmation: ui.confirmationInput.value, selected_joints: selectedJoints() }); toast(result.message); await refresh(); }
    catch (error) { toast(error.message, true); }
  });
  ui.stopCalibration.addEventListener("click", async () => {
    try { const result = await api("/api/calibration/stop"); toast(result.message); }
    catch (error) { toast(error.message, true); }
  });
  ui.exportButton.addEventListener("click", async () => {
    try {
      const result = await api("/api/export");
      const written = ["calibrated.urdf", "gravity_table.yaml"];
      // 只有跑过双向采样才有摩擦数据，没有就不写这个文件。
      if (result.friction_table) written.push("friction_table.yaml");
      if (result.ft_calibration) written.push("ft_calibration.yaml");
      toast(`已写入 ${written.join(" · ")}`);
      await refresh();
    }
    catch (error) { toast(error.message, true); }
  });
  ui.ftCapture.addEventListener("click", async () => {
    try { const result = await api("/api/ft/capture"); toast(`已记录：${result.sides.join(" / ")}`); await refresh(); }
    catch (error) { toast(error.message, true); }
  });
  ui.ftSolve.addEventListener("click", async () => {
    const sides = ["left", "right"].filter(side => state && state.ft_sensor[side].samples.length);
    if (!sides.length) { toast("还没有力传感器样本", true); return; }
    try { await api("/api/ft/solve", { sides }); toast(`已求解：${sides.join(" / ")}`); await refresh(); }
    catch (error) { toast(error.message, true); }
  });
  ui.ftClear.addEventListener("click", async () => {
    if (!confirm("删除全部力传感器样本？")) return;
    try { const result = await api("/api/ft/clear"); toast(result.message); await refresh(); }
    catch (error) { toast(error.message, true); }
  });
  ui.ftAdoptOrigin.addEventListener("click", async () => {
    const origins = {};
    ["left", "right"].forEach(side => {
      const result = state && state.ft_sensor[side].result;
      const suggested = result && result.diagnostics.suggested_origin;
      if (suggested) origins[side] = suggested;
    });
    const sides = Object.keys(origins);
    if (!sides.length) { toast("先求解一次才有建议值", true); return; }
    try { await api("/api/ft/solve", { sides, origins }); toast(`已采用建议取矩点：${sides.join(" / ")}`); await refresh(); }
    catch (error) { toast(error.message, true); }
  });
  ui.ftRows.addEventListener("click", async event => {
    const button = event.target.closest("[data-ft-remove]"); if (!button) return;
    try { await api("/api/ft/remove", { id: Number(button.dataset.ftRemove) }); await refresh(); }
    catch (error) { toast(error.message, true); }
  });
  [ui.leftJoints, ui.rightJoints].forEach(root =>
    root.addEventListener("change", updateArmWarning));
  [ui.xAxis, ui.yAxis].forEach(select => select.addEventListener("change", () => state && drawPlot(state)));
  window.addEventListener("resize", () => state && drawPlot(state));

  document.addEventListener("pointerover", event => showHint(event.target.closest(".hint")));
  document.addEventListener("pointerout", event => event.target.closest(".hint") && hideHint());
  document.addEventListener("focusin", event => showHint(event.target.closest(".hint")));
  document.addEventListener("focusout", hideHint);
  document.addEventListener("scroll", hideHint, true);
  window.addEventListener("resize", hideHint);
}

function showHint(anchor) {
  const text = anchor && anchor.dataset.tip;
  if (!text) return;
  ui.tooltip.textContent = text;
  ui.tooltip.classList.add("show");
  const margin = 12;
  const target = anchor.getBoundingClientRect();
  const bubble = ui.tooltip.getBoundingClientRect();
  const left = target.left + target.width / 2 - bubble.width / 2;
  let top = target.bottom + 8;
  if (top + bubble.height > window.innerHeight - margin) {
    top = target.top - bubble.height - 8;
  }
  ui.tooltip.style.left = `${Math.min(Math.max(margin, left), window.innerWidth - bubble.width - margin)}px`;
  ui.tooltip.style.top = `${Math.max(margin, top)}px`;
}

function hideHint() {
  ui.tooltip.classList.remove("show");
}

document.addEventListener("DOMContentLoaded", async () => {
  cacheUi(); await refresh(); bind(); setInterval(refresh, 500);
});