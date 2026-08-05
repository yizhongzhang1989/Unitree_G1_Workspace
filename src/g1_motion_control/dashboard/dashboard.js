"use strict";
// 取数 + 数字面板。3D 全在 viewer.js 里
// 用**短轮询**而不是 WebSocket：数据源 `~/status` 本来就只有 10 Hz，每次几百字节，轮询的代码量和故障模式都比长连接少一大截
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
    if (++missed === 3) pill($("conn"), "断开", "bad");
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

setInterval(poll, POLL_MS);
poll();
