"use strict";
// 取数 + 数字面板。3D 全在 viewer.js 里
// 用**短轮询**而不是 WebSocket：状态与低层数据约 1 KiB/次，轮询的代码量和故障模式都比长连接少一大截
import * as viewer from "/viewer.js";

const POLL_MS = 50;             // 20 Hz，够跟上 10 Hz 的 status，也不浪费
const $ = (id) => document.getElementById(id);
const SIDES = ["left", "right"];

// 表格是静态的，开局把单元格查一次存下来：轮询里就不用每帧再走一遍
// querySelectorAll（20 Hz × 4 次属性选择器）。
const CELLS = Object.fromEntries(SIDES.map((side) => {
  const rows = document.querySelectorAll(`tr[data-side="${side}"]`);
  const pick = (row, prefix) =>
    ["x", "y", "z"].map((axis) => row.querySelector("." + prefix + axis));
  return [side, {
    t: pick(rows[0], "t"), a: pick(rows[1], "a"), m: pick(rows[2], "m"),
    cmdGap: rows[1].querySelector(".gap"), trackGap: rows[2].querySelector(".gap"),
  }];
}));

function pill(el, text, kind) {
  el.textContent = text;
  el.className = "pill" + (kind ? " " + kind : "");
}

function cells(side, prefix, pose) {
  CELLS[side][prefix].forEach((cell, i) => {
    cell.textContent = pose ? (pose[i] * 1000).toFixed(0) : "—";
  });
}

let modelLoaded = false;
let missed = 0;

async function poll() {
  let snapshot;
  try {
    snapshot = await (await fetch("/api/state")).json();
    missed = 0;
  } catch {
    if (++missed === 3) {
      pill($("conn"), "断开", "bad");
    }
    return;
  }
  if (!modelLoaded) {
    // 模型只取一次。控制栈还没起来时 /api/model 会 503，下一拍再试
    const response = await fetch("/api/model");
    if (!response.ok) { pill($("conn"), "等 /robot_description", "bad"); return; }
    const info = viewer.setModel(await response.json());
    modelLoaded = true;
    pill($("conn"), `${info.links} link · ${info.joints} 关节`, "ok");
  }

  pill($("state"), snapshot.state || "策略层离线",
    snapshot.state === "running" ? "ok" : snapshot.state ? "" : "bad");
  const warn = $("warn");
  warn.hidden = !snapshot.stale;
  if (snapshot.stale) pill(warn, snapshot.stale, "bad");

  const measured = viewer.setJoints(snapshot.q || {});
  const command = snapshot.command_pose || {};
  const limited = snapshot.limited_pose || {};
  viewer.setMarkers("command", command);
  viewer.setMarkers("limited", limited);
  for (const side of SIDES) {
    cells(side, "t", command[side]);
    cells(side, "a", limited[side]);
    cells(side, "m", measured[side]);
    setGap(CELLS[side].cmdGap, distance(command[side], limited[side]));
    setGap(CELLS[side].trackGap, distance(limited[side], measured[side]));
  }
  $("ikms").textContent =
    snapshot.ik_ms == null ? "IK —" :
      `IK ${snapshot.ik_ms.toFixed(2)} ms · 残差 ${
        snapshot.ik_pos_err == null ? "—" : (snapshot.ik_pos_err * 1000).toFixed(1) + " mm"}`;
  $("fps").textContent = `${viewer.drawRate()} fps`;
  syncWeight(snapshot.ik_rotation_weight);
}

// 滑条与轮询会互相打架：拖动期间必须让页面说了算，否则每 50 ms 一次的回写
// 会把滑块弹回旧值。pending 不为空 = 本地有一次还没被 status 确认的改动。
let pendingWeight = null;
let pendingSince = 0;
const PENDING_TIMEOUT_MS = 1500;

// 滑条走 log10：有用区间几乎全在 0.1 以下（实测 1.0->0.1 位置只改善 10%，
// 0.1->0.01 才是另一个数量级），线性刻度的上半段基本是空行程。
const W_LOG_MIN = -3;
const toSlider = (w) =>
  Math.max(W_LOG_MIN, Math.min(0, Math.log10(Math.max(w, 1e-9))));
// 两位有效数字，正好和 status 回显的 4 位小数对得上，滑块不会被舍入弹回。
const fromSlider = (t) => Number(Math.pow(10, t).toPrecision(2));
const fmtWeight = (w) => (w >= 0.1 ? w.toFixed(2) : w.toFixed(3));

function syncWeight(value) {
  const slider = $("rotw");
  if (value == null) {
    slider.disabled = true;
    $("rotw-val").textContent = "—";
    return;
  }
  slider.disabled = false;
  if (pendingWeight != null) {
    if (Math.abs(value - pendingWeight) < 1e-9) {
      pendingWeight = null;                              // 已确认
    } else if (performance.now() - pendingSince < PENDING_TIMEOUT_MS) {
      return;                                            // 还在等确认，别动滑块
    } else {
      pendingWeight = null;   // 没写进去（参数服务不可用）——如实弹回实际生效值
    }
  }
  slider.value = toSlider(value);
  $("rotw-val").textContent = fmtWeight(value);
}

async function sendWeight(value) {
  try {
    await fetch("/api/ik_weight", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ value }),
    });
  } catch {
    /* 下一次拖动会重发；确认值以 status 回读为准 */
  }
}

function distance(a, b) {
  if (!a || !b) return null;
  return 1000 * Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

function setGap(cell, value) {
  cell.textContent = value == null ? "—" : value.toFixed(1);
  cell.classList.toggle("far", value > 10);
}

for (const what of ["mesh", "command", "limited", "measured"]) {
  $("opt-" + what).addEventListener(
    "change", (event) => viewer.setVisible(what, event.target.checked));
}
$("fit").addEventListener("click", () => viewer.fit());

// 拖动过程中就发：后端只保留最后一次，节流交给它，这里不必再攒。
$("rotw").addEventListener("input", (event) => {
  const value = fromSlider(Number(event.target.value));
  pendingWeight = value;
  pendingSince = performance.now();
  $("rotw-val").textContent = fmtWeight(value);
  sendWeight(value);
});

// 必须挡住重入：首拍要多拉一趟 /api/model，比 50 ms 长得多，期间后面的定时器
// 会看到 modelLoaded 还是 false，于是把整棵关节树和 mesh 又建一遍（场景里出现两副手臂）。
let inflight = false;

function tick() {
  if (inflight) return;
  inflight = true;
  poll().finally(() => { inflight = false; });
}

setInterval(tick, POLL_MS);
tick();
