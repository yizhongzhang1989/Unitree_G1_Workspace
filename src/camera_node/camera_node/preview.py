"""MJPEG 预览页。只有 server_port > 0 时才被 import，顺带省掉 cv2 的加载开销。"""

import json
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

_PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>{name}</title></head>
<body style="margin:0;background:#111;color:#ccc;font-family:sans-serif">
<img src="/video_feed" style="display:block;margin:0 auto;max-width:100%">
<p id="s" style="text-align:center"></p>
<script>
setInterval(() => fetch('/status').then(r => r.json())
  .then(d => document.getElementById('s').textContent = d.message), 3000);
</script>
</body></html>
"""


def connected(sock):
    """对端还在不在。没帧可写的时候全靠它，不然断开要等到下一次 write 才发现"""
    try:
        return sock.recv(1, socket.MSG_DONTWAIT | socket.MSG_PEEK) != b''
    except BlockingIOError:
        return True
    except OSError:
        return False


class _Handler(BaseHTTPRequestHandler):
    """`/` 预览页、`/video_feed` MJPEG、`/status` 状态"""

    protocol_version = 'HTTP/1.0'

    def log_message(self, *_args):
        pass  # 别把每个请求都刷进 ROS 日志

    def do_GET(self):
        node = self.server.node  # type: ignore[attr-defined]
        path = self.path.split('?')[0]
        if path == '/video_feed':
            self._stream(node)
        elif path == '/status':
            self._send('application/json', json.dumps(node.status()).encode())
        elif path in ('/', '/index.html'):
            self._send('text/html; charset=utf-8',
                       _PAGE.format(name=node.get_name()).encode())
        else:
            self.send_error(404)

    def _send(self, content_type, body):
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _stream(self, node):
        """打开这条流本身就算一个观众，节点据此决定要不要拉 RTSP"""
        self.send_response(200)
        self.send_header('Cache-Control', 'no-store')
        self.send_header(
            'Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        node.add_viewer()
        seq = 0
        try:
            while not node.stopping:
                frame = node.wait_frame(seq, 1.0)
                if frame is None:
                    if not connected(self.connection):
                        break
                    continue
                seq, raw, width, height = frame
                image = np.frombuffer(raw, np.uint8).reshape(height, width, 3)
                ok, jpeg = cv2.imencode(
                    '.jpg', image,
                    [cv2.IMWRITE_JPEG_QUALITY, node.jpeg_quality])
                if ok:
                    self.wfile.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                        + jpeg.tobytes() + b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            node.remove_viewer()


class PreviewServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, node, port):
        self.node = node  # _Handler 通过 self.server.node 取
        super().__init__(('0.0.0.0', port), _Handler)
        threading.Thread(target=self.serve_forever, daemon=True,
                         name='camera-preview').start()

    def stop(self):
        self.shutdown()
        self.server_close()
