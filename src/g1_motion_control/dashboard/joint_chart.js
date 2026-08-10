"use strict";

const WINDOW_SECONDS = 10;
const STALE_SECONDS = 0.6;
const STORAGE_KEY = "g1-dashboard-selected-joints";
const DEFAULT_SELECTED = [
  "waist_yaw_joint",
  "waist_roll_joint",
  "waist_pitch_joint",
];
const GROUPS = new Map([
  ["left_hip_pitch_joint", "左腿"],
  ["right_hip_pitch_joint", "右腿"],
  ["waist_yaw_joint", "腰部"],
  ["left_shoulder_pitch_joint", "左臂"],
  ["right_shoulder_pitch_joint", "右臂"],
]);
const HUES = [
  192, 35, 133, 280, 8, 214, 76, 329, 157, 48,
  244, 104, 309, 22, 177, 61, 263, 122, 345, 200,
  91, 293, 14, 167, 54, 229, 115, 322, 185,
];

const list = document.getElementById("joint-list");
const canvas = document.getElementById("joint-chart");
const empty = document.getElementById("chart-empty");
const lowstate = document.getElementById("lowstate");
const context = canvas.getContext("2d");

const rows = new Map();
const history = new Map();
const colors = new Map();
let selected = null;
let lastSequence = null;
let lastSampleAt = 0;
let previousRateAt = 0;
let previousRateSequence = 0;
let sampleRate = null;

function color(index) {
  return `hsl(${HUES[index % HUES.length]} 72% 64%)`;
}

function loadSelection(names) {
  try {
    const stored = JSON.parse(localStorage.getItem(STORAGE_KEY));
    if (Array.isArray(stored)) {
      return new Set(stored.filter((name) => names.includes(name)));
    }
  } catch {
    // 损坏或旧版本的浏览器状态直接回到默认选择。
  }
  return new Set(DEFAULT_SELECTED.filter((name) => names.includes(name)));
}

function saveSelection() {
  localStorage.setItem(STORAGE_KEY, JSON.stringify([...selected]));
}

function setSelection(names) {
  selected = new Set(names);
  for (const [name, row] of rows) {
    const checked = selected.has(name);
    row.input.checked = checked;
    row.element.classList.toggle("selected", checked);
  }
  saveSelection();
  draw(performance.now() / 1000);
}

function buildList(names) {
  selected = loadSelection(names);
  names.forEach((name, index) => {
    const group = GROUPS.get(name);
    if (group) {
      const heading = document.createElement("div");
      heading.className = "joint-group";
      heading.textContent = group;
      list.appendChild(heading);
    }

    const element = document.createElement("label");
    element.className = "joint-option";
    const nameWrap = document.createElement("span");
    nameWrap.className = "joint-name-wrap";
    const input = document.createElement("input");
    input.type = "checkbox";
    input.checked = selected.has(name);
    input.setAttribute("aria-label", `显示 ${name}`);
    const swatch = document.createElement("i");
    swatch.className = "joint-swatch";
    swatch.style.background = color(index);
    const label = document.createElement("span");
    label.className = "joint-name";
    label.textContent = name.replace(/_joint$/, "");
    label.title = name;
    const angle = document.createElement("span");
    angle.className = "joint-angle";
    angle.textContent = "—";
    const temperature = document.createElement("span");
    temperature.className = "joint-temperature";
    temperature.textContent = "—";
    const code = document.createElement("code");
    code.className = "joint-code";
    code.textContent = "—";

    nameWrap.append(input, swatch, label);
    element.append(nameWrap, angle, temperature, code);
    element.classList.toggle("selected", input.checked);
    input.addEventListener("change", () => {
      if (input.checked) selected.add(name);
      else selected.delete(name);
      element.classList.toggle("selected", input.checked);
      saveSelection();
      draw(performance.now() / 1000);
    });
    list.appendChild(element);
    rows.set(name, { element, input, angle, temperature, code });
    history.set(name, []);
    colors.set(name, color(index));
  });
}

function formatCode(value) {
  return "0x" + (Number(value) >>> 0).toString(16).toUpperCase().padStart(8, "0");
}

function updateRate(sequence, now) {
  if (!previousRateAt) {
    previousRateAt = now;
    previousRateSequence = sequence;
    return;
  }
  const elapsed = now - previousRateAt;
  if (elapsed >= 1) {
    sampleRate = (sequence - previousRateSequence) / elapsed;
    previousRateAt = now;
    previousRateSequence = sequence;
  }
}

function setLowstatePill(text, kind = "") {
  lowstate.textContent = text;
  lowstate.className = "pill" + (kind ? " " + kind : "");
}

export function setConnected(connected) {
  if (!connected) setLowstatePill("HTTP 断开", "bad");
}

export function update(snapshot) {
  const motors = snapshot.motors || {};
  const names = snapshot.motor_names || Object.keys(motors);
  const hasMotors = Object.keys(motors).length > 0;
  const now = performance.now() / 1000;
  if (!rows.size && names.length) buildList(names);

  const sequence = Number(snapshot.motor_seq || 0);
  const fresh = hasMotors && sequence !== lastSequence;
  if (fresh) {
    lastSequence = sequence;
    lastSampleAt = now;
    updateRate(sequence, now);
    for (const [name, values] of Object.entries(motors)) {
      const row = rows.get(name);
      if (!row) continue;
      const degrees = Number(values[0]) * 180 / Math.PI;
      const code = Number(values[1]);
      const shellTemperature = Number(values[2]);
      const windingTemperature = Number(values[3]);
      row.angle.textContent = `${degrees.toFixed(1)}°`;
      row.temperature.textContent = `${shellTemperature}/${windingTemperature} °C`;
      row.temperature.title = `外壳 ${shellTemperature} °C · 绕组 ${windingTemperature} °C`;
      row.code.textContent = formatCode(code);
      row.code.classList.toggle("fault", code !== 0);
      const points = history.get(name);
      points.push([now, degrees]);
      while (points.length && points[0][0] < now - WINDOW_SECONDS - 1) points.shift();
    }
  }

  if (!lastSampleAt) {
    setLowstatePill("等待 /lowstate");
    empty.textContent = "等待 /lowstate";
  } else if (now - lastSampleAt > STALE_SECONDS) {
    setLowstatePill("/lowstate 停更", "bad");
  } else {
    const rate = sampleRate == null ? "采样中" : `${sampleRate.toFixed(1)} Hz`;
    setLowstatePill(`/lowstate · ${rate}`, "ok");
  }
  draw(now);
}

function chartBounds(now) {
  const values = [];
  for (const name of selected || []) {
    for (const point of history.get(name) || []) {
      if (point[0] >= now - WINDOW_SECONDS) values.push(point[1]);
    }
  }
  if (!values.length) return null;
  let low = Math.min(...values);
  let high = Math.max(...values);
  const span = Math.max(high - low, 20);
  const middle = (low + high) / 2;
  low = middle - span * 0.6;
  high = middle + span * 0.6;
  return [low, high];
}

function sizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.max(1, Math.round(rect.width * ratio));
  const height = Math.max(1, Math.round(rect.height * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  return [rect.width, rect.height];
}

function draw(now) {
  const [width, height] = sizeCanvas();
  context.clearRect(0, 0, width, height);
  const bounds = chartBounds(now);
  const hasSelection = selected && selected.size;
  empty.hidden = Boolean(bounds);
  if (!bounds) {
    if (lastSampleAt && !hasSelection) empty.textContent = "勾选左侧关节以显示曲线";
    else if (lastSampleAt) empty.textContent = "等待采样";
    return;
  }

  const margin = { left: 48, right: 12, top: 14, bottom: 26 };
  const plotWidth = Math.max(1, width - margin.left - margin.right);
  const plotHeight = Math.max(1, height - margin.top - margin.bottom);
  const [low, high] = bounds;
  const x = (time) => margin.left + (time - (now - WINDOW_SECONDS)) / WINDOW_SECONDS * plotWidth;
  const y = (value) => margin.top + (high - value) / (high - low) * plotHeight;

  context.lineWidth = 1;
  context.strokeStyle = "#29313a";
  context.fillStyle = "#77818f";
  context.font = '10px "DejaVu Sans Mono", monospace';
  context.textBaseline = "middle";
  for (let tick = 0; tick <= 4; tick += 1) {
    const value = high - (high - low) * tick / 4;
    const py = margin.top + plotHeight * tick / 4;
    context.beginPath();
    context.moveTo(margin.left, py);
    context.lineTo(width - margin.right, py);
    context.stroke();
    context.textAlign = "right";
    context.fillText(`${value.toFixed(0)}°`, margin.left - 7, py);
  }
  context.textBaseline = "top";
  for (let tick = 0; tick <= 5; tick += 1) {
    const seconds = -WINDOW_SECONDS + WINDOW_SECONDS * tick / 5;
    const px = margin.left + plotWidth * tick / 5;
    context.beginPath();
    context.moveTo(px, margin.top);
    context.lineTo(px, margin.top + plotHeight);
    context.stroke();
    context.textAlign = tick === 0 ? "left" : tick === 5 ? "right" : "center";
    context.fillText(tick === 5 ? "现在" : `${seconds.toFixed(0)}s`, px, height - margin.bottom + 7);
  }

  context.save();
  context.beginPath();
  context.rect(margin.left, margin.top, plotWidth, plotHeight);
  context.clip();
  for (const name of selected) {
    const points = history.get(name) || [];
    context.beginPath();
    context.strokeStyle = colors.get(name);
    context.lineWidth = 1.8;
    let previous = null;
    for (const point of points) {
      if (point[0] < now - WINDOW_SECONDS) continue;
      const px = x(point[0]);
      const py = y(point[1]);
      if (previous == null || point[0] - previous > 0.25) context.moveTo(px, py);
      else context.lineTo(px, py);
      previous = point[0];
    }
    context.stroke();
  }
  context.restore();
}

document.getElementById("joint-all").addEventListener("click", () => setSelection(rows.keys()));
document.getElementById("joint-clear").addEventListener("click", () => setSelection([]));
new ResizeObserver(() => draw(performance.now() / 1000)).observe(canvas.parentElement);