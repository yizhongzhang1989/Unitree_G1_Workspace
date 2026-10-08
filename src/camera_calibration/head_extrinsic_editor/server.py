#!/usr/bin/env python3
"""实时拍照并手调 head_mount_link -> d435_link 的 xyz。"""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
import yaml

HERE = Path(__file__).resolve().parent
WORKSPACE_SRC = HERE.parents[1]
HEAD_SENSORS = WORKSPACE_SRC / 'head_sensors'
sys.path.insert(0, str(HEAD_SENSORS))

from head_sensors.urdf_view import PinholeCamera, UrdfSceneRenderer  # noqa: E402
from head_sensors.verify_head_view import Capture, _package_dirs  # noqa: E402


class EditorHTTPServer(ThreadingHTTPServer):
    request_queue_size = 128


def message_to_bgr(message) -> np.ndarray:
    channels = max(message.step // message.width, 1)
    image = np.frombuffer(message.data, np.uint8).reshape(
        message.height, message.width, channels)
    code = {'rgb8': cv2.COLOR_RGB2BGR,
            'yuv422_yuy2': cv2.COLOR_YUV2BGR_YUY2}.get(message.encoding)
    return cv2.cvtColor(image, code) if code is not None else image.copy()


class Editor:
    def __init__(self, calibration: Path, window: float) -> None:
        self.calibration_path = calibration
        self.window = window
        self.capture_node = Capture()
        self.lock = threading.Lock()
        self.current = None

    def mount(self) -> dict:
        data = yaml.safe_load(self.calibration_path.read_text(encoding='utf-8')) or {}
        mount = (data.get('urdf_overrides') or {}).get('d435_joint') or {}
        if mount.get('parent') != 'head_mount_link' or mount.get('child') != 'd435_link':
            raise ValueError('calibration.yaml 没有 head_mount_link -> d435_link')
        if len(mount.get('xyz') or []) != 3 or len(mount.get('rpy') or []) != 3:
            raise ValueError('calibration.yaml 的 d435_joint xyz/rpy 不完整')
        return mount

    def state(self) -> dict:
        mount = self.mount()
        return {'xyz': [repr(float(value)) for value in mount['xyz']],
                'rpy': [repr(float(value)) for value in mount['rpy']]}

    def take_photo(self) -> dict:
        with self.lock:
            node = self.capture_node
            if not node.wait_until(lambda: node.urdf is not None, 10.0):
                raise RuntimeError('收不到 /robot_description')
            node.color = None
            node.info = None
            node.samples.clear()
            if not node.wait_until(
                    lambda: node.color is not None and node.info is not None and node.samples,
                    10.0):
                raise RuntimeError('收不到头相机、内参或 /joint_states')
            color = node.color
            info = node.info
            node.spin_for(self.window)
            samples = node.samples.copy()
            names = sorted(set.intersection(*(set(sample) for sample in samples)))
            table = np.array([[sample[name] for name in names] for sample in samples])
            joints = dict(zip(names, table.mean(axis=0)))
            movement = float(table.ptp(axis=0).max())
            optical = node.extrinsic('d435_link', 'camera_color_optical_frame',
                                     stamp=color.header.stamp)
            if optical is None:
                raise RuntimeError('查不到 d435_link -> camera_color_optical_frame')
            neck = node.extrinsic('torso_link', 'head_mount_link',
                                  stamp=color.header.stamp)
            if neck is None:
                raise RuntimeError('查不到照片时刻 torso_link -> head_mount_link')

            raw = message_to_bgr(color)
            camera = PinholeCamera(int(info.width), int(info.height),
                                   float(info.k[0]), float(info.k[4]),
                                   float(info.k[2]), float(info.k[5]))
            handle, urdf_path = tempfile.mkstemp(prefix='head_xyz_', suffix='.urdf')
            try:
                with os.fdopen(handle, 'w') as file:
                    file.write(node.urdf)
                renderer = UrdfSceneRenderer(urdf_path, _package_dirs())
            finally:
                os.unlink(urdf_path)
            self.current = (renderer, renderer.q_from_joint_map(joints),
                            camera, optical, neck, raw)
            return {'movement_rad': movement, 'samples': len(samples)}

    def preview(self, xyz_text: list[str], rpy_text: list[str]) -> bytes:
        xyz = [float(value) for value in xyz_text]
        rpy = [float(value) for value in rpy_text]
        if len(xyz) != 3 or len(rpy) != 3 or not np.isfinite(xyz + rpy).all():
            raise ValueError('xyz/rpy 必须是六个有限数字')
        with self.lock:
            if self.current is None:
                raise ValueError('请先拍照')
            renderer, configuration, camera, optical, neck, image = self.current
            mount = origin_matrix(xyz, rpy)
            extrinsic = neck @ mount @ optical
            rotation, position = renderer.camera_pose(
                configuration, 'torso_link', extrinsic=extrinsic)
            masks = arm_masks(renderer, configuration, camera, rotation, position)
            output = _overlay_bgr(image, masks)
            ok, blob = cv2.imencode('.jpg', output, [cv2.IMWRITE_JPEG_QUALITY, 92])
            if not ok:
                raise RuntimeError('预览编码失败')
            return blob.tobytes()

    def close(self) -> None:
        self.capture_node.destroy_node()


def arm_masks(renderer, configuration, camera, rotation, position) -> dict[str, np.ndarray]:
    masks = {side: np.zeros((camera.height, camera.width), dtype=np.uint8)
             for side in ('left', 'right')}
    for geometry, _, polygons in renderer._project(
            configuration, camera, rotation, position, 0.05, 20.0):
        name = renderer.names[geometry]
        side = next((value for value in masks if name.startswith(value + '_')), None)
        if side is not None:
            cv2.fillPoly(masks[side], np.round(polygons * 16.0).astype(np.int32),
                         255, lineType=cv2.LINE_8, shift=4)
    return masks


def _overlay_bgr(image: np.ndarray, masks: dict[str, np.ndarray]) -> np.ndarray:
    output = image.copy()
    for side, bgr in (('left', (0, 255, 0)), ('right', (0, 0, 255))):
        contours, _ = cv2.findContours(masks[side],
                                       cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(output, contours, -1, bgr, 2)
    return output


def origin_matrix(xyz: list[float], rpy: list[float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    output = np.eye(4)
    output[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    output[:3, 3] = xyz
    return output


def handler(editor: Editor):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def send(self, code: int, body: bytes, content_type: str) -> None:
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def send_json(self, value, code=200):
            self.send(code, json.dumps(value, ensure_ascii=False).encode(),
                      'application/json; charset=utf-8')

        def do_GET(self):  # noqa: N802
            try:
                url = urlparse(self.path)
                query = parse_qs(url.query)
                if url.path in ('/', '/index.html'):
                    return self.send(200, (HERE / 'index.html').read_bytes(),
                                     'text/html; charset=utf-8')
                if url.path == '/style.css':
                    return self.send(200, (HERE / 'style.css').read_bytes(), 'text/css')
                if url.path == '/app.js':
                    return self.send(200, (HERE / 'app.js').read_bytes(), 'text/javascript')
                if url.path == '/api/state':
                    return self.send_json(editor.state())
                if url.path == '/api/preview':
                    def arg(name):
                        return (query.get(name) or [''])[0]
                    blob = editor.preview([arg('x'), arg('y'), arg('z')],
                                          [arg('roll'), arg('pitch'), arg('yaw')])
                    return self.send(200, blob, 'image/jpeg')
                return self.send_json({'error': 'not found'}, 404)
            except Exception as error:  # noqa: BLE001
                return self.send_json({'error': str(error)}, 400)

        def do_POST(self):  # noqa: N802
            try:
                if urlparse(self.path).path == '/api/capture':
                    return self.send_json(editor.take_photo())
                return self.send_json({'error': 'not found'}, 404)
            except Exception as error:  # noqa: BLE001
                return self.send_json({'error': str(error)}, 400)

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--calibration', default=str(
        HERE.parent / 'config/calibration.yaml'))
    parser.add_argument('--window', type=float, default=0.5)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8231)
    args = parser.parse_args()
    rclpy.init()
    editor = Editor(Path(args.calibration), args.window)
    server = EditorHTTPServer((args.host, args.port), handler(editor))
    print(f'http://127.0.0.1:{args.port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        editor.close()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
