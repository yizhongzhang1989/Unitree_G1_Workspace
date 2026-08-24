"""两个面板共用的 HTTP 骨架。标准库 ``ThreadingHTTPServer``，不引 aiohttp。

**面板只是观察者 + 命令入口。** 逻辑都在各自的节点里，HTTP 线程崩了、没人开页面，
节点照常跑。页面靠 1 Hz 轮询，不用 WebSocket —— 刷新率要求低，多一个协议不值。

每个面板只需要给出：POST 动作表，和一个负责 GET 分发的 ``route(handler, url)``。
"""

from __future__ import annotations

import json
import threading
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

_STATIC = Path(__file__).with_name('static')
_MAX_BODY = 1 << 20
_CTYPE = {'.html': 'text/html; charset=utf-8',
          '.js': 'application/javascript; charset=utf-8',
          '.css': 'text/css; charset=utf-8'}


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    # 默认 accept 队列只有 5，而一个页面就开 6 条 keep-alive 连接：
    # 多开几个页面 SYN 就被丢，表现是请求挂住或偶发连接被拒
    request_queue_size = 128


def make_handler(node, actions: dict, route):
    log = node.get_logger()

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *args) -> None:      # 别把 ROS 日志刷爆
            pass

        def send_bytes(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def send_json(self, payload, code: int = 200) -> None:
            self.send_bytes(code, json.dumps(payload, ensure_ascii=False).encode(),
                            'application/json; charset=utf-8')

        def send_static(self, name: str) -> None:
            path = (_STATIC / name).resolve()
            if _STATIC.resolve() not in path.parents or not path.is_file():
                return self.send_json({'error': 'not found'}, 404)
            self.send_bytes(200, path.read_bytes(),
                            _CTYPE.get(path.suffix, 'application/octet-stream'))

        def body(self) -> dict:
            n = int(self.headers.get('Content-Length') or 0)
            if n <= 0:
                return {}
            if n > _MAX_BODY:
                raise ValueError('请求体过大')
            return json.loads(self.rfile.read(n) or b'{}')

        def _fail(self, exc: Exception) -> None:
            if isinstance(exc, (RuntimeError, ValueError, KeyError)):
                return self.send_json({'ok': False, 'error': str(exc)}, 400)
            log.error(f'{self.path} 失败: {traceback.format_exc()}')
            return self.send_json(
                {'ok': False, 'error': f'{type(exc).__name__}: {exc}'}, 500)

        def do_GET(self) -> None:                  # noqa: N802
            # 包一层：GET 里冒出未预料的异常会打死连接，浏览器只看到空响应
            try:
                return route(self, urlparse(self.path))
            except Exception as exc:               # noqa: BLE001
                return self._fail(exc)

        def do_POST(self) -> None:                 # noqa: N802
            fn = actions.get(urlparse(self.path).path)
            if fn is None:
                return self.send_json({'ok': False, 'error': 'not found'}, 404)
            try:
                return self.send_json({'ok': True, 'result': fn(self.body())})
            except Exception as exc:               # noqa: BLE001
                return self._fail(exc)

    return Handler


class Panel:
    """一个面板 = 后台线程里的一个 HTTP 服务。"""

    def __init__(self, node, handler, port: int, name: str,
                 host: str = '0.0.0.0') -> None:
        self.node, self.handler = node, handler
        self.port, self.host, self.name = port, host, name
        self._srv: _Server | None = None

    def start(self) -> None:
        self._srv = _Server((self.host, self.port), self.handler)
        threading.Thread(target=self._srv.serve_forever, daemon=True).start()
        self.node.get_logger().info(f'{self.name} http://{self.host}:{self.port}')

    def stop(self) -> None:
        if self._srv is not None:
            self._srv.shutdown()
            self._srv.server_close()
