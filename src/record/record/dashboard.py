"""采集面板。标准库 ``ThreadingHTTPServer``，不引 aiohttp。

**它只是观察者 + 命令入口。** 所有录制逻辑在 ``recorder_node.Recorder`` 里，这里不持有
任何采集状态；HTTP 线程崩了、没人开页面，录制照常。

页面靠 1 Hz 轮询 ``/api/state``，不用 WebSocket —— 刷新率要求低，多一个协议不值。
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


class Dashboard:
    def __init__(self, recorder, port: int = 8220, host: str = '0.0.0.0') -> None:
        self.recorder = recorder
        self.port, self.host = port, host
        self._srv: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        handler = _make_handler(self.recorder)
        self._srv = ThreadingHTTPServer((self.host, self.port), handler)
        self._srv.daemon_threads = True
        self._thread = threading.Thread(target=self._srv.serve_forever, daemon=True)
        self._thread.start()
        self.recorder.get_logger().info(f'采集面板 http://{self.host}:{self.port}')

    def stop(self) -> None:
        if self._srv is not None:
            self._srv.shutdown()
            self._srv.server_close()


def _make_handler(rec):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *args) -> None:      # 别把 ROS 日志刷爆
            pass

        # ------------------------------------------------------------ 基础设施

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _json(self, payload, code: int = 200) -> None:
            self._send(code, json.dumps(payload, ensure_ascii=False).encode(),
                       'application/json; charset=utf-8')

        def _body(self) -> dict:
            n = int(self.headers.get('Content-Length') or 0)
            if n <= 0:
                return {}
            if n > _MAX_BODY:
                raise ValueError('请求体过大')
            return json.loads(self.rfile.read(n) or b'{}')

        def _static(self, name: str) -> None:
            path = (_STATIC / name).resolve()
            if _STATIC.resolve() not in path.parents or not path.is_file():
                return self._json({'error': 'not found'}, 404)
            ctype = {'.html': 'text/html; charset=utf-8',
                     '.js': 'application/javascript; charset=utf-8',
                     '.css': 'text/css; charset=utf-8'}.get(path.suffix,
                                                            'application/octet-stream')
            self._send(200, path.read_bytes(), ctype)

        # ------------------------------------------------------------------ GET

        def do_GET(self) -> None:                       # noqa: N802
            path = urlparse(self.path).path
            if path in ('/', '/index.html'):
                return self._static('index.html')
            if path in ('/app.js', '/app.css'):
                return self._static(path.lstrip('/'))
            if path == '/api/state':
                return self._json({'status': rec.status(),
                                   'streams': rec.stream_overview()})
            if path == '/api/round/svg':
                svg = ''
                if rec.session is not None and rec.session.round_index >= 0:
                    f = (rec.session.paths.rounds
                         / f'round_{rec.session.round_index:03d}.svg')
                    svg = f.read_text(encoding='utf-8') if f.is_file() else ''
                return self._send(200, svg.encode(), 'image/svg+xml; charset=utf-8')
            return self._json({'error': 'not found'}, 404)

        # ----------------------------------------------------------------- POST

        def do_POST(self) -> None:                      # noqa: N802
            path = urlparse(self.path).path
            actions = {
                '/api/session/start': lambda b: rec.start_session(
                    b.get('streams') or {}, b.get('note', '')),
                '/api/session/stop': lambda b: rec.stop_session(),
                '/api/round/start': lambda b: rec.start_round(b.get('seed')),
                '/api/round/end': lambda b: rec.end_round(),
                '/api/episode/start': lambda b: rec.start_episode(int(b['index'])),
                '/api/episode/end': lambda b: rec.end_episode(
                    b.get('outcome', 'success'), b.get('note', '')),
            }
            fn = actions.get(path)
            if fn is None:
                return self._json({'error': 'not found'}, 404)
            try:
                return self._json({'ok': True, 'result': fn(self._body())})
            except (RuntimeError, ValueError, KeyError) as exc:
                return self._json({'ok': False, 'error': str(exc)}, 400)
            except Exception as exc:                    # noqa: BLE001
                rec.get_logger().error(f'{path} 失败: {traceback.format_exc()}')
                return self._json({'ok': False, 'error': f'{type(exc).__name__}: {exc}'},
                                  500)

    return Handler
