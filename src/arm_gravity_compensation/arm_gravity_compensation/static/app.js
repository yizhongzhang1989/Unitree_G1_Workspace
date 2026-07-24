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
  ui.stopCalibration.disabled = !calibrationActive;
}

function updateTargets(snapshot) {
  ui.targetRows.innerHTML = snapshot.targets.length ? snapshot.targets.map(target => {
    const values = snapshot.joint_names.map(name => target.positions[name]);
    const left = values.slice(0, 7);
    const right = values.slice(7);
    const range = list => `${Math.min(...list).toFixed(2)} … ${Math.max(...list).toFixed(2)}`;
    return `<tr>
      <td><code>#${target.id}</code></td><td>${target.source}</td>
      <td>${target.captured_at.replace("T", " ").slice(0, 19)}</td>
      <td><code>${range(left)}</code></td><td><code>${range(right)}</code></td>
      <td><button class="delete-button" data-remove="${target.id}" title="删除姿态">×</button></td>
    </tr>`;
  }).join("") : `<tr><td class="empty-row" colspan="6">尚未记录标定姿态</td></tr>`;
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
    <td>${item.inlier_fraction === undefined ? "-" : `${(item.inlier_fraction * 100).toFixed(1)}%`}</td>
  </tr>`;
  }).join("") : `<tr><td class="empty-row" colspan="8">尚无标定迭代</td></tr>`;
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
  } catch (error) {
    ui.connectionDot.classList.remove("online"); ui.connectionDot.classList.add("error");
    ui.phaseLabel.textContent = "连接断开"; ui.messageLabel.textContent = error.message;
  }
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
    try { const result = await api("/api/calibration/start", { confirmation: ui.confirmationInput.value }); toast(result.message); await refresh(); }
    catch (error) { toast(error.message, true); }
  });
  ui.stopCalibration.addEventListener("click", async () => {
    try { const result = await api("/api/calibration/stop"); toast(result.message); }
    catch (error) { toast(error.message, true); }
  });
  ui.exportButton.addEventListener("click", async () => {
    try { const result = await api("/api/export"); toast(`已写入 ${result.path}`); }
    catch (error) { toast(error.message, true); }
  });
  [ui.xAxis, ui.yAxis].forEach(select => select.addEventListener("change", () => state && drawPlot(state)));
  window.addEventListener("resize", () => state && drawPlot(state));
}

document.addEventListener("DOMContentLoaded", async () => {
  cacheUi(); await refresh(); bind(); setInterval(refresh, 500);
});