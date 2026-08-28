"""两个面板共用的 HTTP 骨架。标准库 ``ThreadingHTTPServer``，不引 aiohttp。

**面板只是观察者 + 命令入口。** 逻辑都在各自的节点里，HTTP 线程崩了、没人开页面，
节点照常跑。页面靠 1 Hz 轮询，不用 WebSocket —— 刷新率要求低，多一个协议不值。

每个面板只需要给出两张路由表：POST 动作表和 GET 表，以及首页用哪个 html。
静态资源、404/405、query 解析都在这里做，两边不各写一份。
"""

from __future__ import annotations

import json
import threading
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

_STATIC = Path(__file__).with_name('static')
_MAX_BODY = 1 << 20
_CHUNK = 1 << 18
_CTYPE = {'.html': 'text/html; charset=utf-8',
          '.js': 'application/javascript; charset=utf-8',
          '.css': 'text/css; charset=utf-8'}


class _Chunked:
    """把写操作包成 HTTP chunked。zipfile 只要求 write/flush，不 seek。"""

    def __init__(self, raw) -> None:
        self._raw = raw

    def write(self, data) -> int:
        if data:
            self._raw.write(f'{len(data):X}\r\n'.encode())
            self._raw.write(data)
            self._raw.write(b'\r\n')
        return len(data)

    def flush(self) -> None:
        self._raw.flush()

    def close(self) -> None:
        self._raw.write(b'0\r\n\r\n')


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    # 默认 accept 队列只有 5，而一个页面就开 6 条 keep-alive 连接：
    # 多开几个页面 SYN 就被丢，表现是请求挂住或偶发连接被拒
    request_queue_size = 128


def tree_entries(root: Path, prefix: str = '') -> list:
    """把一个目录摊成 ``send_zip`` 要的 ``(zip 内路径, 磁盘路径)`` 清单。

    跳过 ``__pycache__``：它不是内容，而且一份版本对不上的 .pyc 落到 B 上会盖掉源码。
    """
    return [(prefix + p.relative_to(root).as_posix(), p)
            for p in sorted(root.rglob('*'))
            if p.is_file() and '__pycache__' not in p.parts]


def make_handler(node, actions: dict, gets: dict, index: str = 'index.html'):
    """``actions``: POST 路径 -> ``fn(body)``；``gets``: GET 路径 -> ``fn(handler, arg)``，
    ``arg(key)`` 取 query 里的第一个值。
    """
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

        def send_file(self, path: Path, name: str = '',
                      ctype: str = 'application/octet-stream') -> None:
            """分块下发一个文件，支持 Range 断点续传。

            不能走 ``send_bytes``：那要先把整份读进内存，导出的视频动辄几百 MB，
            Orin 上几个并发就把内存吃光了。Range 是为了 WiFi 断了能接着下，
            也是 `<video>` 能拖进度条的前提。不给 ``name`` 就是内联打开而不是另存。
            """
            size = path.stat().st_size
            start, end = 0, size - 1
            code = 200
            unit = self.headers.get('Range', '')
            if unit.startswith('bytes=') and size:
                first, _, last = unit[6:].partition('-')
                try:
                    start = int(first) if first else max(size - int(last), 0)
                    end = int(last) if first and last else size - 1
                except ValueError:
                    start, end = 0, size - 1
                if start >= size or start > end:
                    self.send_response(416)
                    self.send_header('Content-Range', f'bytes */{size}')
                    self.send_header('Content-Length', '0')
                    self.end_headers()
                    return
                end = min(end, size - 1)
                code = 206
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(end - start + 1))
            self.send_header('Accept-Ranges', 'bytes')
            if code == 206:
                self.send_header('Content-Range', f'bytes {start}-{end}/{size}')
            if name:
                self.send_header('Content-Disposition',
                                 f'attachment; filename="{name}"')
            self.end_headers()
            remaining = end - start + 1
            try:
                with open(path, 'rb') as fh:
                    fh.seek(start)
                    while remaining > 0:
                        chunk = fh.read(min(_CHUNK, remaining))
                        if not chunk:
                            break
                        self.wfile.write(chunk)
                        remaining -= len(chunk)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def send_zip(self, name: str, entries) -> None:
            """边打包边发。``entries`` 是 ``(zip 内路径, 磁盘路径)`` 的序列。

            **不压缩**（ZIP_STORED）。之前否掉过打包，那是冲着「压缩省带宽」去的 ——
            mkv 压缩率 100%，白花 CPU。这里的目的只是「一次点击拿走一整套」，
            STORED 几乎零开销，是另一回事。
            代价是长度未知（chunked）、断了要重来；单个文件仍可分别下载并续传。
            """
            import zipfile
            self.send_response(200)
            self.send_header('Content-Type', 'application/zip')
            self.send_header('Transfer-Encoding', 'chunked')
            self.send_header('Content-Disposition', f'attachment; filename="{name}"')
            self.end_headers()
            stream = _Chunked(self.wfile)
            try:
                with zipfile.ZipFile(stream, 'w', zipfile.ZIP_STORED) as zf:
                    for arcname, path in entries:
                        zf.write(path, arcname)
                stream.close()
            except (BrokenPipeError, ConnectionResetError):
                pass

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
                u = urlparse(self.path)
                if u.path in ('/', '/index.html'):
                    return self.send_static(index)
                fn = gets.get(u.path)
                if fn is not None:
                    q = parse_qs(u.query)
                    return fn(self, lambda k: (q.get(k) or [''])[0])
                if u.path in actions:
                    # 前端漏传 body 就会变成 GET，一律 404 的话看不出来是方法错了
                    return self.send_json({'error': f'{u.path} 只接受 POST'}, 405)
                # 剩下的当静态资源：``send_static`` 自己挡路径穿越，不存在就 404
                return self.send_static(u.path.lstrip('/'))
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
