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
    html = Path(__file__).with_name("web_wrench.html").read_bytes()

    class Handler(BaseHTTPRequestHandler):
        server_version = "KWR57Web/0.1"

        def log_message(self, format: str, *args: Any) -> None:
            return

        def do_GET(self) -> None:  # noqa: N802
            if self.path in ("/", "/index.html"):
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(html)))
                self.end_headers()
                self.wfile.write(html)
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
    parser.add_argument("--host", default="0.0.0.0", help="Bind host, default 0.0.0.0")
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
