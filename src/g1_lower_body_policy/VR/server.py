"""VR 头显 -> 机器人 的 WebXR 桥接服务（无需 Unity / 无需装 APK）。

已验证设备：Meta Quest 2/3、PICO 4 系列（任何支持 WebXR 的头显浏览器均可）。

数据链路：
    头显浏览器打开本服务的网页 -> WebXR 读取手柄位姿 -> WebSocket 推给本服务
    -> 本服务对外提供三种消费方式：
        1. GET  /state              轮询拉取最新一帧（HTTP 接口）
        2. WS   /ws/subscribe       实时订阅每一帧（主动推送）
        3. --forward-url            服务端定频 POST 到你指定的地址（主动往某处发数据）
    反向通道：POST /haptic 让手柄震动。

启动前提（WebXR 要求安全上下文，二选一）：
    A. adb reverse（推荐，无需证书）：
         adb reverse tcp:8000 tcp:8000
       然后在头显浏览器里访问 http://localhost:8000
       （也可用 adb shell am start -a android.intent.action.VIEW -d http://localhost:8000 直接拉起）
    B. 自签名 HTTPS：
         mkcert -install && mkcert -cert-file cert.pem -key-file key.pem <本机IP>
         python server.py --cert cert.pem --key key.pem
       头显浏览器访问 https://<本机IP>:8000
"""

from __future__ import annotations

import argparse
import asyncio
import hmac
import json
import ssl
import time
from pathlib import Path

import aiohttp
from aiohttp import web

STATIC_DIR = Path(__file__).parent / "static"

EMPTY_STATE = {
    "seq": 0,
    "t": 0.0,
    "recv_time": 0.0,
    "session_active": False,
    "head": None,
    "left": None,
    "right": None,
}


class Hub:
    def __init__(self) -> None:
        self.state: dict = dict(EMPTY_STATE)
        self.subscribers: set[web.WebSocketResponse] = set()
        self.devices: set[web.WebSocketResponse] = set()
        self.updated = asyncio.Event()

    def publish(self, frame: dict) -> None:
        frame["recv_time"] = time.time()
        self.state = frame
        self.updated.set()
        self.updated.clear()
        if not self.subscribers:
            return
        text = json.dumps(frame, separators=(",", ":"))
        for ws in list(self.subscribers):
            # 背压保护：消费端跟不上就丢帧，而不是把内存撑爆
            if ws.closed:
                self.subscribers.discard(ws)
            else:
                asyncio.create_task(_safe_send(ws, text))

    async def to_devices(self, msg: dict) -> int:
        text = json.dumps(msg, separators=(",", ":"))
        sent = 0
        for ws in list(self.devices):
            if not ws.closed:
                await _safe_send(ws, text)
                sent += 1
        return sent


async def _safe_send(ws: web.WebSocketResponse, text: str) -> None:
    try:
        await ws.send_str(text)
    except (ConnectionResetError, RuntimeError, aiohttp.ClientError):
        pass


def _check_token(request: web.Request) -> None:
    expected = request.app["token"]
    if not expected:
        return
    got = request.query.get("token") or request.headers.get("X-Auth-Token", "")
    if not hmac.compare_digest(got, expected):
        raise web.HTTPUnauthorized(text="invalid token")


async def index(request: web.Request) -> web.StreamResponse:
    return web.FileResponse(STATIC_DIR / "index.html")


async def monitor(request: web.Request) -> web.StreamResponse:
    return web.FileResponse(STATIC_DIR / "monitor.html")


async def get_state(request: web.Request) -> web.StreamResponse:
    _check_token(request)
    return web.json_response(request.app["hub"].state)


async def post_haptic(request: web.Request) -> web.StreamResponse:
    """POST {"hand": "right", "intensity": 0.6, "duration": 80}"""
    _check_token(request)
    body = await request.json()
    hand = body.get("hand", "right")
    if hand not in ("left", "right"):
        raise web.HTTPBadRequest(text="hand must be 'left' or 'right'")
    sent = await request.app["hub"].to_devices({
        "type": "haptic",
        "hand": hand,
        "intensity": float(body.get("intensity", 0.6)),
        "duration": float(body.get("duration", 80)),
    })
    return web.json_response({"delivered_to": sent})


async def ws_device(request: web.Request) -> web.StreamResponse:
    """头显上行：接收 WebXR 位姿帧。"""
    _check_token(request)
    hub: Hub = request.app["hub"]
    ws = web.WebSocketResponse(max_msg_size=1 << 20, heartbeat=20)
    await ws.prepare(request)
    hub.devices.add(ws)
    print("[device] connected")
    try:
        async for msg in ws:
            if msg.type is aiohttp.WSMsgType.TEXT:
                try:
                    hub.publish(json.loads(msg.data))
                except (json.JSONDecodeError, TypeError):
                    continue
            elif msg.type is aiohttp.WSMsgType.ERROR:
                break
    finally:
        hub.devices.discard(ws)
        hub.state = dict(EMPTY_STATE)
        print("[device] disconnected")
    return ws


async def ws_subscribe(request: web.Request) -> web.StreamResponse:
    """机器人端下行：实时订阅每一帧。"""
    _check_token(request)
    hub: Hub = request.app["hub"]
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)
    hub.subscribers.add(ws)
    print("[subscriber] connected")
    try:
        async for msg in ws:
            if msg.type is aiohttp.WSMsgType.TEXT:
                # 订阅方也可以经此通道请求震动
                try:
                    payload = json.loads(msg.data)
                except json.JSONDecodeError:
                    continue
                if payload.get("type") == "haptic":
                    await hub.to_devices(payload)
    finally:
        hub.subscribers.discard(ws)
        print("[subscriber] disconnected")
    return ws


async def forwarder(app: web.Application) -> None:
    """定频把最新状态 POST 到下游地址。"""
    url: str = app["forward_url"]
    period = 1.0 / max(app["forward_hz"], 1e-3)
    hub: Hub = app["hub"]
    timeout = aiohttp.ClientTimeout(total=period * 2)
    async with aiohttp.ClientSession(timeout=timeout) as sess:
        while True:
            await asyncio.sleep(period)
            if not hub.state.get("session_active"):
                continue
            try:
                await sess.post(url, json=hub.state)
            except (aiohttp.ClientError, asyncio.TimeoutError) as exc:
                print(f"[forward] {type(exc).__name__}: {exc}")


async def on_startup(app: web.Application) -> None:
    if app["forward_url"]:
        app["forward_task"] = asyncio.create_task(forwarder(app))


async def on_cleanup(app: web.Application) -> None:
    task = app.get("forward_task")
    if task:
        task.cancel()


def build_app(args: argparse.Namespace) -> web.Application:
    app = web.Application()
    app["hub"] = Hub()
    app["token"] = args.token
    app["forward_url"] = args.forward_url
    app["forward_hz"] = args.forward_hz
    app.add_routes([
        web.get("/", index),
        web.get("/monitor", monitor),
        web.get("/state", get_state),
        web.post("/haptic", post_haptic),
        web.get("/ws/device", ws_device),
        web.get("/ws/subscribe", ws_subscribe),
    ])
    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
    return app


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--cert", help="TLS 证书 (PEM)")
    p.add_argument("--key", help="TLS 私钥 (PEM)")
    p.add_argument("--token", default="", help="可选共享密钥，所有接口需带 ?token=")
    p.add_argument("--forward-url", default="", help="把每帧 POST 到该地址")
    p.add_argument("--forward-hz", type=float, default=30.0)
    args = p.parse_args()

    ssl_ctx = None
    if args.cert and args.key:
        ssl_ctx = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ssl_ctx.load_cert_chain(args.cert, args.key)

    scheme = "https" if ssl_ctx else "http"
    print(f"Serving on {scheme}://{args.host}:{args.port}")
    print("  GET  /              open this in the headset browser")
    print("  GET  /monitor       live data dashboard (open on the PC)")
    print("  GET  /state         latest frame (polling)")
    print("  WS   /ws/subscribe  live stream")
    print("  POST /haptic        trigger controller vibration")
    web.run_app(build_app(args), host=args.host, port=args.port, ssl_context=ssl_ctx, print=None)


if __name__ == "__main__":
    main()
