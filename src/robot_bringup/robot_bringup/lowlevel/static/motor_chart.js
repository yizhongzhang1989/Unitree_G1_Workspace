"use strict";

const WINDOW_SECONDS = 10;
// 它决定一行能摆几张图；CHART_GAP 要和 CSS 里 #chart-grid 的 gap 对上。
const MIN_CHART_WIDTH = 600;
const CHART_GAP = 8;
const MOTOR_KEY = "g1-lowlevel-selected-motors";
const SIGNAL_KEY = "g1-lowlevel-signals";
const DEFAULT_MOTORS = [
  "left_elbow_joint",
  "right_elbow_joint",
  "waist_pitch_joint",
];
const DEFAULT_SIGNALS = ["q", "tau_est"];
const RAD_TO_DEG = 180 / Math.PI;
// scale 把 SI 原值换成显示单位；span 是 Y 轴最小跨度，避免噪声被放大成满屏。
const SIGNALS = new Map([
  ["q", { label: "角度", unit: "°", scale: RAD_TO_DEG, span: 20, decimals: 0 }],
  ["dq", { label: "角速度", unit: "°/s", scale: RAD_TO_DEG, span: 20, decimals: 0 }],
  ["ddq", { label: "角加速度", unit: "°/s²", scale: RAD_TO_DEG, span: 20, decimals: 0 }],
  ["tau_est", { label: "力矩", unit: "N·m", scale: 1, span: 2, decimals: 1 }],
  ["temp_shell", { label: "外壳温度", unit: "°C", scale: 1, span: 5, decimals: 0 }],
  ["temp_winding", { label: "绕组温度", unit: "°C", scale: 1, span: 5, decimals: 0 }],
  ["vol", { label: "电压", unit: "V", scale: 1, span: 2, decimals: 1 }],
]);
const SIGNAL_ORDER = [...SIGNALS.keys()];
const HUES = [
  192, 35, 133, 280, 8, 214, 76, 329, 157, 48,
  244, 104, 309, 22, 177, 61, 263, 122, 345, 200,
  91, 293, 14, 167, 54, 229, 115, 322, 185, 40,
  270, 100, 320, 150, 210,
];

const grid = document.getElementById("chart-grid");
const empty = document.getElementById("chart-empty");
const hint = document.getElementById("chart-hint");

const listeners = [];
const columns = new Map();      // field -> {element, canvas, context, history}
const motors = new Set();
const signals = new Set();
const slot = new Map();         // 电机名 -> 物理槽位，绘制里用得很密
let index = {};
let sampled = false;
let frame = 0;

export function color(position) {
  return `hsl(${HUES[position % HUES.length]} 72% 64%)`;
}

function label(field) {
  return `${SIGNALS.get(field).label} (${SIGNALS.get(field).unit})`;
}

function restore(key, fallback, valid) {
  try {
    const stored = JSON.parse(localStorage.getItem(key));
    if (Array.isArray(stored)) return stored.filter(valid);
  } catch {
    // 损坏或旧版本的浏览器状态直接回到默认选择。
  }
  return fallback.filter(valid);
}

function announce() {
  localStorage.setItem(MOTOR_KEY, JSON.stringify([...motors]));
  localStorage.setItem(SIGNAL_KEY, JSON.stringify([...signals]));
  hint.textContent = motors.size && signals.size
    ? `${motors.size} 电机 × ${signals.size} 信号`
    : "勾选左侧电机";
  // 取消勾选后就不再采样了，旧点再留着只是占内存。
  for (const { history } of columns.values()) {
    for (const name of history.keys()) if (!motors.has(name)) history.delete(name);
  }
  for (const listener of listeners) listener(motors, signals);
  layout();
  draw();
}

function ordered() {
  return SIGNAL_ORDER.filter((field) => columns.has(field))
    .map((field) => columns.get(field));
}

function gcd(a, b) {
  return b ? gcd(b, a % b) : a;
}

/** 一行摆不下就折行，并把图均分到各行，不让最后一行只剩一张。 */
function layout() {
  const items = ordered();
  if (!items.length) return;
  const width = grid.clientWidth;
  const perRow = Math.max(1, Math.floor(
    (width + CHART_GAP) / (MIN_CHART_WIDTH + CHART_GAP)));
  const rows = Math.ceil(items.length / perRow);
  const base = Math.floor(items.length / rows);
  const extra = items.length % rows;
  const counts = Array.from(
    { length: rows }, (_, row) => base + (row < extra ? 1 : 0));
  // 各行张数不同时，用它们的最小公倍数当轨道数，每张图再跨多个轨道。
  const tracks = counts.reduce((all, one) => all * one / gcd(all, one), 1);
  grid.style.gridTemplateColumns = `repeat(${tracks}, minmax(0, 1fr))`;
  grid.style.gridTemplateRows = `repeat(${rows}, minmax(0, 1fr))`;
  let at = 0;
  for (const count of counts) {
    for (let seat = 0; seat < count; seat += 1) {
      items[at].element.style.gridColumn = `span ${tracks / count}`;
      at += 1;
    }
  }
}

function addColumn(field) {
  const element = document.createElement("div");
  element.className = "chart-col";
  const title = document.createElement("div");
  title.className = "chart-title";
  title.textContent = label(field);
  const wrap = document.createElement("div");
  wrap.className = "chart-wrap";
  const canvas = document.createElement("canvas");
  canvas.setAttribute("aria-label", `${label(field)} 时间曲线`);
  wrap.appendChild(canvas);
  element.append(title, wrap);
  columns.set(field, {
    element, canvas, context: canvas.getContext("2d"), history: new Map(),
  });
  // DOM 顺序跟着表头列顺序走，而不是勾选的先后。
  const next = SIGNAL_ORDER.slice(SIGNAL_ORDER.indexOf(field) + 1)
    .find((other) => columns.has(other));
  grid.insertBefore(element, next ? columns.get(next).element : null);
}

/** 首帧调用：``motorNames`` 定序，``motorFields`` 决定各信号在行数组里的下标。 */
export function init(motorNames, motorFields) {
  motorNames.forEach((name, at) => slot.set(name, at));
  index = Object.fromEntries(motorFields.map((field, at) => [field, at]));
  for (const name of restore(
    MOTOR_KEY, DEFAULT_MOTORS, (name) => slot.has(name))) {
    motors.add(name);
  }
  for (const field of restore(
    SIGNAL_KEY, DEFAULT_SIGNALS, (field) => SIGNALS.has(field) && field in index)) {
    signals.add(field);
    addColumn(field);
  }
  announce();
}

export function isSignalOn(field) {
  return signals.has(field);
}

export function setSignal(field, on) {
  if (!SIGNALS.has(field) || on === signals.has(field)) return;
  if (on) {
    signals.add(field);
    addColumn(field);
  } else {
    signals.delete(field);
    columns.get(field).element.remove();
    columns.delete(field);
  }
  announce();
}

export function isMotorOn(name) {
  return motors.has(name);
}

export function setMotor(name, on) {
  if (on) motors.add(name);
  else motors.delete(name);
  announce();
}

export function setMotors(wanted) {
  motors.clear();
  for (const name of wanted) motors.add(name);
  announce();
}

export function allNames() {
  return [...slot.keys()];
}

export function onChange(listener) {
  listeners.push(listener);
}

/** ``rows`` 按物理槽位排，每行按后端的 ``motor_fields`` 排。只采勾选的那几路。 */
export function sample(now, rows) {
  sampled = true;
  const cutoff = now - WINDOW_SECONDS - 1;
  for (const [field, column] of columns) {
    const at = index[field];
    const scale = SIGNALS.get(field).scale;
    for (const name of motors) {
      const raw = rows[slot.get(name)]?.[at];
      if (raw == null) continue;
      let points = column.history.get(name);
      if (!points) column.history.set(name, points = []);
      points.push([now, raw * scale]);
      while (points.length && points[0][0] < cutoff) points.shift();
    }
  }
}

function bounds(column, field, now) {
  // 一遍扫完：十秒窗口在 100 Hz 下是千级点数，Math.min(...points) 会爆参数栈。
  let low = Infinity;
  let high = -Infinity;
  for (const name of motors) {
    for (const [at, value] of column.history.get(name) || []) {
      if (at < now - WINDOW_SECONDS) continue;
      if (value < low) low = value;
      if (value > high) high = value;
    }
  }
  if (low > high) return null;
  const span = Math.max(high - low, SIGNALS.get(field).span);
  const middle = (low + high) / 2;
  return [middle - span * 0.6, middle + span * 0.6];
}

function sizeCanvas(canvas, context) {
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

function drawColumn(field, column, now) {
  const { canvas, context } = column;
  const [width, height] = sizeCanvas(canvas, context);
  context.clearRect(0, 0, width, height);
  const range = bounds(column, field, now);
  if (!range) return;

  const decimals = SIGNALS.get(field).decimals;
  const margin = { left: 42, right: 8, top: 12, bottom: 22 };
  const plotWidth = Math.max(1, width - margin.left - margin.right);
  const plotHeight = Math.max(1, height - margin.top - margin.bottom);
  const [low, high] = range;
  const x = (time) =>
    margin.left + (time - (now - WINDOW_SECONDS)) / WINDOW_SECONDS * plotWidth;
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
    context.fillText(value.toFixed(decimals), margin.left - 5, py);
  }
  // 列被压窄时只留两端时间标签，中间那些会糊成一团。
  context.textBaseline = "top";
  for (let tick = 0; tick <= 2; tick += 1) {
    const seconds = -WINDOW_SECONDS + WINDOW_SECONDS * tick / 2;
    const px = margin.left + plotWidth * tick / 2;
    context.beginPath();
    context.moveTo(px, margin.top);
    context.lineTo(px, margin.top + plotHeight);
    context.stroke();
    context.textAlign = tick === 0 ? "left" : tick === 2 ? "right" : "center";
    context.fillText(tick === 2 ? "现在" : `${seconds.toFixed(0)}s`,
      px, height - margin.bottom + 5);
  }

  context.save();
  context.beginPath();
  context.rect(margin.left, margin.top, plotWidth, plotHeight);
  context.clip();
  for (const name of motors) {
    const points = column.history.get(name) || [];
    context.beginPath();
    context.strokeStyle = color(slot.get(name));
    context.lineWidth = 1.6;
    let previous = null;
    for (const point of points) {
      if (point[0] < now - WINDOW_SECONDS) continue;
      const px = x(point[0]);
      const py = y(point[1]);
      // 采样断档时不要拿一条直线把两段接起来，那是假数据。
      if (previous == null || point[0] - previous > 0.25) context.moveTo(px, py);
      else context.lineTo(px, py);
      previous = point[0];
    }
    context.stroke();
  }
  context.restore();
}

function render(now) {
  for (const [field, column] of columns) drawColumn(field, column, now);
  if (!sampled) empty.textContent = "等待 /lowstate";
  else if (!signals.size) empty.textContent = "勾选表头的列以显示曲线";
  else if (!motors.size) empty.textContent = "勾选左侧电机以显示曲线";
  empty.hidden = Boolean(sampled && signals.size && motors.size);
}

/** 交给 rAF：轮询拉到 100 Hz 时没必要每帧都重画整片画布。 */
export function draw() {
  if (frame) return;
  frame = requestAnimationFrame(() => {
    frame = 0;
    render(performance.now() / 1000);
  });
}

new ResizeObserver(() => {
  layout();
  draw();
}).observe(grid);
