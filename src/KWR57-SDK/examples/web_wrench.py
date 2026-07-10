"""SSH-friendly web viewer for KWR57 wrench data.

Run:
    python examples/web_wrench.py --demo
    python examples/web_wrench.py --interface canalystii --channel 0

Open in browser:
    http://127.0.0.1:8765

For SSH tunnel from local machine:
    ssh -L 8765:127.0.0.1:8765 user@server
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from kwr57_sensor import KWR57Sensor  # noqa: E402
from kwr57_sensor.protocol import Wrench  # noqa: E402


@dataclass
class SharedState:
    lock: threading.Lock
    latest: dict[str, Any]


def now_ms() -> int:
    return int(time.time() * 1000)


def initial_payload(force_scale: float, torque_scale: float) -> dict[str, Any]:
    return {
        "timestamp_ms": now_ms(),
        "fx": 0.0,
        "fy": 0.0,
        "fz": 0.0,
        "mx": 0.0,
        "my": 0.0,
        "mz": 0.0,
        "hz": 0.0,
        "status": "starting",
        "force_scale": force_scale,
        "torque_scale": torque_scale,
    }


def update_payload(state: SharedState, wrench: Wrench, hz: float, status: str) -> None:
    with state.lock:
        state.latest = {
            **state.latest,
            "timestamp_ms": now_ms(),
            "fx": float(wrench.fx),
            "fy": float(wrench.fy),
            "fz": float(wrench.fz),
            "mx": float(wrench.mx),
            "my": float(wrench.my),
            "mz": float(wrench.mz),
            "hz": float(hz),
            "status": status,
        }


def set_status(state: SharedState, status: str) -> None:
    with state.lock:
        state.latest = {**state.latest, "timestamp_ms": now_ms(), "status": status}


def sensor_worker(args: argparse.Namespace, state: SharedState, stop_event: threading.Event) -> None:
    count = 0
    start = time.monotonic()
    try:
        with KWR57Sensor.open(interface=args.interface, channel=args.channel) as sensor:
            sensor.start_stream(period_ms=args.period_ms, rate_hz=args.rate_hz)
            set_status(state, f"{args.interface}:{args.channel} connected")
            while not stop_event.is_set():
                wrench = sensor.read_wrench(timeout=0.2)
                if wrench is None:
                    continue
                count += 1
                elapsed = time.monotonic() - start
                hz = count / elapsed if elapsed > 0 else 0.0
                update_payload(state, wrench, hz, f"{args.interface}:{args.channel} {hz:5.1f} Hz")
    except Exception as exc:  # noqa: BLE001
        set_status(state, f"sensor error: {exc}")


def demo_worker(state: SharedState, stop_event: threading.Event) -> None:
    start = time.monotonic()
    count = 0
    while not stop_event.is_set():
        t = time.monotonic() - start
        wrench = Wrench(
            fx=7.0 * math.sin(t * 1.2),
            fy=5.5 * math.cos(t * 0.9),
            fz=3.0 * math.sin(t * 0.6),
            mx=0.18 * math.cos(t * 1.5),
            my=0.14 * math.sin(t * 1.1),
            mz=0.10 * math.cos(t * 0.7),
        )
        count += 1
        hz = count / t if t > 0 else 0.0
        update_payload(state, wrench, hz, f"demo {hz:5.1f} Hz")
        time.sleep(1 / 60)


def make_handler(state: SharedState) -> type[BaseHTTPRequestHandler]:
    html = """<!doctype html>
<html lang=\"en\">
<head>
  <meta charset=\"utf-8\" />
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />
  <title>KWR57 Web Visualizer</title>
  <style>
    :root {
      --bg: #f6f3ea;
      --panel: #fffaf0;
      --text: #231f1a;
      --muted: #6f6659;
      --line: #d9cfbf;
      --fx: #d94f45;
      --fy: #2f8f6b;
      --fz: #2f6fb4;
      --mx: #d98d2b;
      --my: #7c5ab8;
      --mz: #4e8f9f;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: \"Segoe UI\", \"Noto Sans\", sans-serif;
      color: var(--text);
      background: radial-gradient(circle at 0% 0%, #fefcf6, var(--bg));
    }
    .wrap { max-width: 1100px; margin: 0 auto; padding: 18px; }
    .head {
      display: flex; justify-content: space-between; align-items: baseline; gap: 12px;
      margin-bottom: 14px;
    }
    .title { font-size: 24px; font-weight: 700; letter-spacing: 0.2px; }
    .status { color: var(--muted); font-size: 13px; }
    .grid { display: grid; grid-template-columns: 1.4fr 1fr; gap: 14px; }
    .card {
      border: 1px solid var(--line);
      border-radius: 12px;
      background: var(--panel);
      padding: 12px;
    }
    .card h3 { margin: 0 0 8px; font-size: 14px; color: #5e5548; }
    .bars { display: grid; gap: 8px; }
    .row {
      display: grid;
      grid-template-columns: 30px 1fr 100px;
      gap: 8px;
      align-items: center;
      font-size: 13px;
    }
    .track {
      position: relative;
      height: 16px;
      background: #f1eadd;
      border: 1px solid #ddd4c6;
      border-radius: 8px;
      overflow: hidden;
    }
    .track::before {
      content: \"\";
      position: absolute;
      left: 50%;
      top: 0;
      bottom: 0;
      width: 1px;
      background: #b9af9f;
    }
    .fill { position: absolute; top: 0; bottom: 0; }
    .val { text-align: right; font-family: Consolas, monospace; font-weight: 700; }
    .side { display: grid; gap: 14px; }
    canvas {
      width: 100%;
      height: 240px;
      border: 1px solid var(--line);
      border-radius: 10px;
      background: #fbf8f0;
    }
    .legend { margin-top: 6px; color: var(--muted); font-size: 12px; }
    @media (max-width: 960px) { .grid { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <div class=\"wrap\">
    <div class=\"head\">
      <div class=\"title\">KWR57 Web Visualizer</div>
      <div class=\"status\" id=\"status\">connecting...</div>
    </div>
    <div class=\"grid\">
      <section class=\"card\">
        <h3>Live Axes</h3>
        <div class=\"bars\" id=\"bars\"></div>
      </section>
      <section class=\"side\">
        <div class=\"card\">
          <h3>Force XY Projection</h3>
          <canvas id=\"forceCanvas\"></canvas>
          <div class=\"legend\" id=\"forceNorm\">|F| = 0.0000</div>
        </div>
        <div class=\"card\">
          <h3>Torque XY Projection</h3>
          <canvas id=\"torqueCanvas\"></canvas>
          <div class=\"legend\" id=\"torqueNorm\">|M| = 0.0000</div>
        </div>
      </section>
    </div>
  </div>

  <script>
    const axes = [
      [\"Fx\", \"fx\", \"--fx\", \"force\"],
      [\"Fy\", \"fy\", \"--fy\", \"force\"],
      [\"Fz\", \"fz\", \"--fz\", \"force\"],
      [\"Mx\", \"mx\", \"--mx\", \"torque\"],
      [\"My\", \"my\", \"--my\", \"torque\"],
      [\"Mz\", \"mz\", \"--mz\", \"torque\"],
    ];

    const barsRoot = document.getElementById(\"bars\");
    const statusEl = document.getElementById(\"status\");
    const forceNormEl = document.getElementById(\"forceNorm\");
    const torqueNormEl = document.getElementById(\"torqueNorm\");
    const forceCanvas = document.getElementById(\"forceCanvas\");
    const torqueCanvas = document.getElementById(\"torqueCanvas\");

    const refs = {};
    for (const [name, key, cssVar] of axes) {
      const row = document.createElement(\"div\");
      row.className = \"row\";
      row.innerHTML = `<div>${name}</div><div class=\"track\"><div class=\"fill\"></div></div><div class=\"val\">+0.0000</div>`;
      barsRoot.appendChild(row);
      const fill = row.querySelector(\".fill\");
      fill.style.background = `var(${cssVar})`;
      refs[key] = {
        fill,
        val: row.querySelector(\".val\"),
        mode: name.startsWith(\"F\") ? \"force\" : \"torque\",
      };
    }

    function drawVector(canvas, xVal, yVal, zVal, scale, color) {
      const dpr = window.devicePixelRatio || 1;
      const w = canvas.clientWidth;
      const h = canvas.clientHeight;
      const cw = Math.max(1, Math.floor(w * dpr));
      const ch = Math.max(1, Math.floor(h * dpr));
      if (canvas.width !== cw || canvas.height !== ch) {
        canvas.width = cw;
        canvas.height = ch;
      }
      const ctx = canvas.getContext(\"2d\");
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, w, h);

      const cx = w / 2;
      const cy = h / 2;
      const radius = Math.max(30, Math.min(w, h) * 0.34);

      ctx.strokeStyle = \"#c9bfae\";
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.arc(cx, cy, radius, 0, Math.PI * 2);
      ctx.stroke();
      ctx.setLineDash([]);

      ctx.beginPath();
      ctx.moveTo(cx - radius - 8, cy);
      ctx.lineTo(cx + radius + 8, cy);
      ctx.moveTo(cx, cy - radius - 8);
      ctx.lineTo(cx, cy + radius + 8);
      ctx.stroke();

      const rx = scale > 0 ? Math.max(-1, Math.min(1, xVal / scale)) : 0;
      const ry = scale > 0 ? Math.max(-1, Math.min(1, yVal / scale)) : 0;
      const ex = cx + rx * radius;
      const ey = cy - ry * radius;

      ctx.strokeStyle = color;
      ctx.fillStyle = color;
      ctx.lineWidth = 4;
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(ex, ey);
      ctx.stroke();

      const angle = Math.atan2(ey - cy, ex - cx);
      const ah = 12;
      ctx.beginPath();
      ctx.moveTo(ex, ey);
      ctx.lineTo(ex - ah * Math.cos(angle - Math.PI / 7), ey - ah * Math.sin(angle - Math.PI / 7));
      ctx.lineTo(ex - ah * Math.cos(angle + Math.PI / 7), ey - ah * Math.sin(angle + Math.PI / 7));
      ctx.closePath();
      ctx.fill();

      const zRatio = scale > 0 ? Math.max(0, Math.min(1, Math.abs(zVal) / scale)) : 0;
      const zr = 6 + zRatio * 16;
      ctx.lineWidth = 2;
      ctx.strokeStyle = zVal >= 0 ? \"#1e1b16\" : color;
      ctx.fillStyle = zVal >= 0 ? color : \"transparent\";
      ctx.beginPath();
      ctx.arc(26 + zr, 24 + zr, zr, 0, Math.PI * 2);
      ctx.stroke();
      if (zVal >= 0) ctx.fill();
    }

    function updateUI(data) {
      statusEl.textContent = `${data.status} | ${data.hz.toFixed(1)} Hz`;
      for (const [key, item] of Object.entries(refs)) {
        const v = Number(data[key] || 0);
        const scale = item.mode === \"force\" ? Number(data.force_scale || 1) : Number(data.torque_scale || 1);
        const ratio = scale > 0 ? Math.max(-1, Math.min(1, v / scale)) : 0;
        const left = ratio >= 0 ? 50 : 50 + ratio * 50;
        const width = Math.abs(ratio) * 50;
        item.fill.style.left = `${left}%`;
        item.fill.style.width = `${width}%`;
        item.val.textContent = v >= 0 ? `+${v.toFixed(4)}` : v.toFixed(4);
      }

      const fn = Math.sqrt(data.fx * data.fx + data.fy * data.fy + data.fz * data.fz);
      const mn = Math.sqrt(data.mx * data.mx + data.my * data.my + data.mz * data.mz);
      forceNormEl.textContent = `|F| = ${fn.toFixed(4)} (XY projection)`;
      torqueNormEl.textContent = `|M| = ${mn.toFixed(4)} (XY projection)`;

      drawVector(forceCanvas, data.fx, data.fy, data.fz, Number(data.force_scale || 1), \"#2f8f6b\");
      drawVector(torqueCanvas, data.mx, data.my, data.mz, Number(data.torque_scale || 1), \"#7c5ab8\");
    }

    async function poll() {
      try {
        const resp = await fetch(\"/api/latest\", { cache: \"no-store\" });
        if (!resp.ok) {
          throw new Error(`HTTP ${resp.status}`);
        }
        const data = await resp.json();
        updateUI(data);
      } catch (_err) {
        statusEl.textContent = \"connection lost, retrying...\";
      }
    }

    setInterval(poll, 100);
    poll();
  </script>
</body>
</html>
"""

    class Handler(BaseHTTPRequestHandler):
        server_version = "KWR57Web/0.1"

        def log_message(self, format: str, *args: Any) -> None:
            return

        def do_GET(self) -> None:  # noqa: N802
            if self.path in ("/", "/index.html"):
                body = html.encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path == "/api/latest":
                with state.lock:
                    payload = dict(state.latest)
                body = json.dumps(payload, ensure_ascii=True).encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            self.send_error(HTTPStatus.NOT_FOUND, "Not Found")

    return Handler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="KWR57 web visualizer")
    parser.add_argument("--host", default="127.0.0.1", help="Bind host, default 127.0.0.1")
    parser.add_argument("--port", type=int, default=8765, help="Bind port, default 8765")
    parser.add_argument("--interface", default="canalystii", help="python-can interface")
    parser.add_argument("--channel", default="0", help="bus channel")
    parser.add_argument("--period-ms", type=int, default=1, help="stream period in ms, default 1 (~1000Hz, max)")
    parser.add_argument("--rate-hz", type=int, default=1000, help="internal sample rate (100/200/400/500/600/1000), default 1000 (max)")
    parser.add_argument("--force-scale", type=float, default=10.0, help="force full scale")
    parser.add_argument("--torque-scale", type=float, default=0.25, help="torque full scale")
    parser.add_argument("--demo", action="store_true", help="run with simulated data")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    stop_event = threading.Event()
    state = SharedState(
        lock=threading.Lock(),
        latest=initial_payload(
            force_scale=max(args.force_scale, 1e-9),
            torque_scale=max(args.torque_scale, 1e-9),
        ),
    )

    worker_target = demo_worker if args.demo else sensor_worker
    worker_args: tuple[Any, ...]
    if args.demo:
        worker_args = (state, stop_event)
    else:
        worker_args = (args, state, stop_event)
    worker = threading.Thread(target=worker_target, args=worker_args, daemon=True)
    worker.start()

    handler = make_handler(state)
    httpd = ThreadingHTTPServer((args.host, args.port), handler)
    print(f"Serving at http://{args.host}:{args.port} (demo={args.demo})")
    print("Press Ctrl+C to stop")

    try:
        httpd.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        httpd.shutdown()
        httpd.server_close()
        worker.join(timeout=1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
