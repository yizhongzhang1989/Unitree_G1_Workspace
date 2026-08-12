"use strict";

import * as chart from "/motor_chart.js";

// 固件的 /lf/* 就是 20 Hz，再快只会拿到重复帧。
const POLL_MS = 50;
const STALE_SECONDS = 0.6;
const RAD_TO_DEG = 180 / Math.PI;
const WARM_C = 60;
const HOT_C = 80;

const conn = document.getElementById("conn");
const rate = document.getElementById("rate");
const machine = document.getElementById("machine");
const modePr = document.getElementById("mode-pr");
const tick = document.getElementById("tick");
const count = document.getElementById("motor-count");
const table = document.getElementById("motors");
const body = table.querySelector("tbody");
const remoteHex = document.getElementById("remote-hex");
const version = document.getElementById("version");
const crc = document.getElementById("crc");

const rows = new Map();
const imuCells = new Map();
const signalInputs = new Map();
let column = {};
let names = [];
let named = 0;
let showReserved = false;
let lastSeq = null;
let lastSampleAt = 0;
let rateAt = 0;
let rateSeq = 0;
let sampleRate = null;

for (const row of document.querySelectorAll("#imu tbody tr")) {
  imuCells.set(row.dataset.field, row.querySelectorAll("td"));
}

function pill(element, text, kind = "") {
  element.textContent = text;
  element.className = "pill" + (kind ? " " + kind : "");
}

function fixed(value, digits) {
  return value == null ? "—" : Number(value).toFixed(digits);
}

function deg(value, digits) {
  return value == null ? "—" : (value * RAD_TO_DEG).toFixed(digits);
}

function hex32(value) {
  return "0x" + (Number(value) >>> 0).toString(16).toUpperCase().padStart(8, "0");
}

function buildRows(meta) {
  names = meta.motor_names;
  named = meta.named_motors;
  column = Object.fromEntries(meta.motor_fields.map((field, index) => [field, index]));
  chart.init(names, meta.motor_fields);
  buildSignalToggles();
  names.forEach((name, index) => {
    const row = document.createElement("tr");
    if (index >= named) row.className = "reserved";

    const pick = document.createElement("td");
    pick.className = "col-pick";
    const input = document.createElement("input");
    input.type = "checkbox";
    input.checked = chart.isMotorOn(name);
    input.setAttribute("aria-label", `曲线显示 ${name}`);
    input.addEventListener("change", () => chart.setMotor(name, input.checked));
    pick.appendChild(input);

    const label = document.createElement("td");
    label.className = "col-name";
    label.title = `槽位 ${index} · ${name}`;
    const swatch = document.createElement("i");
    swatch.className = "swatch";
    swatch.style.background = chart.color(index);
    label.append(swatch, document.createTextNode(name.replace(/_joint$/, "")));

    const cells = Array.from({ length: 10 }, () => document.createElement("td"));
    row.append(pick, label, ...cells);
    body.appendChild(row);
    rows.set(name, { row, input, cells });
  });
  count.textContent = `${named} 轴本体 + ${names.length - named} 预留槽`;
}

/** 表头那一行就是曲线列的开关：勾一个，右边就多一列。 */
function buildSignalToggles() {
  for (const head of table.querySelectorAll("thead th[data-signal]")) {
    const field = head.dataset.signal;
    const toggle = document.createElement("label");
    toggle.className = "col-toggle";
    const input = document.createElement("input");
    input.type = "checkbox";
    input.checked = chart.isSignalOn(field);
    input.addEventListener("change", () => chart.setSignal(field, input.checked));
    toggle.append(input, document.createTextNode(head.textContent));
    head.textContent = "";
    head.appendChild(toggle);
    signalInputs.set(field, input);
  }
}

function temperatureClass(value) {
  if (value >= HOT_C) return "hot";
  if (value >= WARM_C) return "warm";
  return "";
}

function updateRows(motors) {
  // 折起来的预留槽没人看，别白写 60 个单元格。
  const limit = showReserved ? motors.length : named;
  for (let at = 0; at < limit; at += 1) {
    const values = motors[at];
    const cells = rows.get(names[at]).cells;
    const mode = values[column.mode];
    const shell = values[column.temp_shell];
    const winding = values[column.temp_winding];
    const state = values[column.motorstate];
    cells[0].textContent = mode;
    cells[0].className = mode ? "" : "off";
    cells[1].textContent = deg(values[column.q], 2);
    cells[2].textContent = deg(values[column.dq], 1);
    cells[3].textContent = deg(values[column.ddq], 0);
    cells[4].textContent = fixed(values[column.tau_est], 2);
    cells[5].textContent = shell;
    cells[5].className = temperatureClass(shell);
    cells[6].textContent = winding;
    cells[6].className = temperatureClass(winding);
    cells[7].textContent = fixed(values[column.vol], 1);
    cells[8].textContent = hex32(state);
    cells[8].className = state ? "fault" : "";
    cells[9].textContent = `${values[column.sensor0]}/${values[column.sensor1]}`;
  }
}

function updateHeader(header) {
  pill(machine, `mode_machine ${header.mode_machine}`);
  pill(modePr, `mode_pr ${header.mode_pr} · ${header.mode_pr ? "AB 并联" : "PR 串联"}`);
  pill(tick, `tick ${header.tick}`);
  version.textContent = `version ${header.version.join(".")}`;
  crc.textContent = `crc ${hex32(header.crc)}`;
  remoteHex.textContent = header.wireless_remote.match(/../g).join(" ");
}

const IMU_TEXT = {
  quaternion: (imu) => imu.quaternion.map((value) => fixed(value, 3)).join("  "),
  rpy: (imu) => imu.rpy.map((value) => deg(value, 1)).join("  "),
  gyroscope: (imu) => imu.gyroscope.map((value) => fixed(value, 3)).join("  "),
  accelerometer: (imu) => imu.accelerometer.map((value) => fixed(value, 3)).join("  "),
  temperature: (imu) => imu.temperature,
};

function updateImu(imu) {
  for (const [field, cells] of imuCells) {
    [imu.pelvis, imu.torso].forEach((source, at) => {
      cells[at].textContent = source ? IMU_TEXT[field](source) : "—";
      cells[at].classList.toggle("stale", !source);
    });
  }
}

function updateRate(seq, now) {
  if (!rateAt) {
    rateAt = now;
    rateSeq = seq;
    return;
  }
  const elapsed = now - rateAt;
  if (elapsed >= 1) {
    sampleRate = (seq - rateSeq) / elapsed;
    rateAt = now;
    rateSeq = seq;
  }
}

function apply(snapshot) {
  const now = performance.now() / 1000;
  const seq = Number(snapshot.seq || 0);
  if (snapshot.motors.length && seq !== lastSeq) {
    lastSeq = seq;
    lastSampleAt = now;
    updateRate(seq, now);
    updateRows(snapshot.motors);
    updateHeader(snapshot.header);
    updateImu(snapshot.imu);
    chart.sample(now, snapshot.motors);
  }

  if (!lastSampleAt) pill(rate, "等待 /lowstate");
  else if (now - lastSampleAt > STALE_SECONDS) pill(rate, "/lowstate 停更", "bad");
  else {
    const hz = sampleRate == null ? "采样中" : `${sampleRate.toFixed(0)} Hz`;
    pill(rate, `/lowstate · ${hz}`, "ok");
  }
  chart.draw();
}

let inFlight = false;

async function poll() {
  // 频率拉高后一次请求可能跨过一个周期，不拦住就会叠出一堆并发。
  if (inFlight) return;
  inFlight = true;
  try {
    const response = await fetch("/api/state");
    if (!response.ok) throw new Error(String(response.status));
    apply(await response.json());
    pill(conn, "已连接", "ok");
  } catch {
    pill(conn, "HTTP 断开", "bad");
    pill(rate, "HTTP 断开", "bad");
  } finally {
    inFlight = false;
  }
}

chart.onChange((selectedMotors, selectedSignals) => {
  for (const [name, entry] of rows) {
    const on = selectedMotors.has(name);
    entry.input.checked = on;
    entry.row.classList.toggle("selected", on);
  }
  for (const [field, input] of signalInputs) {
    input.checked = selectedSignals.has(field);
  }
});

document.getElementById("opt-reserved").addEventListener("change", (event) => {
  showReserved = event.target.checked;
  table.classList.toggle("show-reserved", showReserved);
});
document.getElementById("pick-all").addEventListener(
  "click", () => chart.setMotors(chart.allNames()));
document.getElementById("pick-none").addEventListener(
  "click", () => chart.setMotors([]));

// 电机名单与字段顺序一整场不变，只取一次。
buildRows(await fetch("/api/motors").then((response) => response.json()));
poll();
setInterval(poll, POLL_MS);
