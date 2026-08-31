"""动捕重定向的可视化面板。**不控制任何硬件**，只连头显、算 FK、渲染姿态。

上真机之前用它看：人摆一个姿势，G1 会变成什么样。左边是人的 24 关节骨架，
右边是重定向 + G1 正运动学之后的位形，两边同一个视角、同一个尺度。

    ros2 launch g1_mocap dashboard.launch.py
    # 浏览器打开 http://<机器人IP>:8080

头显在 PicoBridge 面板里填 `<机器人IP>:8000`（和 mocap_node 一样的上行端口）。

> 这个节点、``mocap_node``、``g1_rgmt_tracking_global`` 的跟踪层**三选一**：
> 头显同一时刻只连一个上行地址。

面板上要盯的两处：

``关节角条``
    条超出灰色的训练分布区间就是危险信号——策略在分布外，输出什么都不奇怪。
    人站直时所有条都应该落在各自的 default 刻度上（校准把站立位形映射到了那里）。
``骨架叠加``
    人和 G1 的差异一眼可见。腿明显外撇、大腿后摆，说明校准没做或者做的时候人没站直。
"""

from __future__ import annotations

import json
import mimetypes
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from .kinematics import G1Kinematics
from .mocap_node import DEFAULT_URDF, resolve_package_path
from .retarget import Retargeter
from .skeleton import SMPL_JOINTS, STATUS_MESSAGES
from .stream import MocapStream
from .urdf import parse as parse_urdf
from .urdf import under

# SMPL 的运动学树，索引即 XrBodyJointBD 的枚举值。父节点 -1 表示根。
SMPL_PARENTS = (-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17,
                18, 19, 20, 21)


class _Handler(BaseHTTPRequestHandler):

    def log_message(self, *_args) -> None:
        """默认实现往 stderr 刷每一条请求，30 Hz 轮询会把节点日志淹掉。"""

    def do_GET(self) -> None:
        # 未捕获的异常会打死连接，浏览器只看到 ERR_EMPTY_RESPONSE，页面上一点线索都没有。
        try:
            self._route()
        except Exception as exc:  # noqa: BLE001
            self._send(500, 'application/json',
                       json.dumps({'error': f'{type(exc).__name__}: {exc}'}).encode())

    def do_POST(self) -> None:
        try:
            if self.path.rstrip('/') == '/calibrate':
                self._send(200, 'application/json',
                           json.dumps(self.server.dashboard.calibrate()).encode())
            else:
                self._send(404, 'text/plain', b'not found')
        except Exception as exc:  # noqa: BLE001
            self._send(500, 'application/json',
                       json.dumps({'error': f'{type(exc).__name__}: {exc}'}).encode())

    def _route(self) -> None:
        from urllib.parse import parse_qs, unquote, urlparse
        url = urlparse(self.path)
        path = url.path.rstrip('/') or '/'
        board = self.server.dashboard
        if path == '/layout':
            self._send(200, 'application/json', json.dumps(board.layout()).encode())
        elif path == '/model':
            self._send(200, 'application/json', json.dumps(board.model).encode())
        elif path == '/state':
            self._send(200, 'application/json', json.dumps(board.state()).encode())
        elif path == '/mesh':
            data = board.read_mesh(unquote((parse_qs(url.query).get('path') or [''])[0]))
            if data is None:
                self._send(404, 'text/plain', b'no such mesh')
            else:
                # mesh 一整场都不会变，让浏览器缓存住，别每次刷新都重下十几兆。
                self._send(200, 'model/stl', data, cache='public, max-age=86400')
        else:
            # 其余一律当 static/ 下的文件。read_static 里的 under() 挡住路径穿越。
            found = board.read_static('dashboard.html' if path == '/' else path.lstrip('/'))
            if found is None:
                self._send(404, 'text/plain', b'not found')
            else:
                cache = 'no-store' if path == '/' else 'public, max-age=86400'
                self._send(200, found[1], found[0], cache=cache)

    def _send(self, code: int, content_type: str, body: bytes,
              cache: str = 'no-store') -> None:
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', cache)
        self.end_headers()
        self.wfile.write(body)


class _Server(ThreadingHTTPServer):
    # 默认只有 5：一个页面就开好几条 keep-alive 连接，多开几个页面 SYN 就被丢。
    request_queue_size = 128
    daemon_threads = True
    allow_reuse_address = True


class DashboardNode(Node):

    def __init__(self) -> None:
        super().__init__('mocap_dashboard')
        p = self.declare_parameter

        joints = list(p('joints', Parameter.Type.STRING_ARRAY)
                      .get_parameter_value().string_array_value)
        if not joints:
            raise RuntimeError('参数 joints 为空：应由 launch 注入 29 轴动作关节顺序')
        key_bodies = list(p('key_bodies', Parameter.Type.STRING_ARRAY)
                          .get_parameter_value().string_array_value)
        default_pos = np.asarray(
            p('default_joint_pos', Parameter.Type.DOUBLE_ARRAY)
            .get_parameter_value().double_array_value, dtype=np.float64)

        urdf = resolve_package_path(
            p('urdf_path', DEFAULT_URDF).get_parameter_value().string_value)
        # 渲染用的 FK 单独一份：pinocchio 的 data 不可重入，收帧线程那份不能借。
        self._kin = G1Kinematics(urdf, joints)
        self._retarget = Retargeter(
            G1Kinematics(urdf, joints), key_bodies=key_bodies,
            anchor_body=p('anchor_body', 'torso_link').get_parameter_value().string_value,
            default_joint_pos=default_pos,
            foot_ground_clearance_m=float(
                p('foot_ground_clearance_m', 0.03).get_parameter_value().double_value))
        uplink_port = int(p('port', 8000).get_parameter_value().integer_value)
        self._stream = MocapStream(
            self._retarget,
            host=p('host', '0.0.0.0').get_parameter_value().string_value,
            port=uplink_port,
            token=p('token', '').get_parameter_value().string_value,
            buffer_s=float(p('buffer_s', 2.0).get_parameter_value().double_value),
            log=self.get_logger().info)

        self._joints = joints
        self._default = default_pos
        lower, upper = self._kin.limits()
        self._limits = (lower, upper)
        self._lock = threading.Lock()
        self._share = Path(get_package_share_directory('g1_mocap')) / 'static'
        # mesh 全部从描述包的 model/ 下取，路径参数只允许落在这棵子树里。
        self._mesh_root = Path(get_package_share_directory(
            p('mesh_package', 'unitree_g1_description').get_parameter_value().string_value
        )) / p('mesh_root', 'model').get_parameter_value().string_value
        self.model = parse_urdf(Path(urdf).read_text(encoding='utf-8'), 'pelvis')

        http_port = int(p('dashboard_port', 18080).get_parameter_value().integer_value)
        self._http = _Server((p('dashboard_host', '0.0.0.0')
                              .get_parameter_value().string_value, http_port), _Handler)
        self._http.dashboard = self
        threading.Thread(target=self._http.serve_forever, daemon=True).start()

        self.create_service(Trigger, '~/calibrate', self._on_calibrate)
        self._stream.start()
        self.get_logger().info(
            f'面板已就绪: http://<本机IP>:{http_port}   动捕上行端口 {uplink_port}')

    def destroy_node(self) -> bool:
        self._http.shutdown()
        self._stream.stop()
        return super().destroy_node()

    ##
    # 给 HTTP 用（跑在 HTTP 线程里，FK 必须加锁）
    ##

    def layout(self) -> dict:
        """静态部分，前端只取一次。"""
        lower, upper = self._limits
        return {
            'joints': [{'name': n, 'lower': float(lower[i]), 'upper': float(upper[i]),
                        'default': float(self._default[i])}
                       for i, n in enumerate(self._joints)],
            'human_joints': list(SMPL_JOINTS),
            'human_parents': list(SMPL_PARENTS),
        }

    def read_static(self, relative: str) -> tuple[bytes, str] | None:
        path = under(self._share, relative)
        if path is None or not path.is_file():
            return None
        kind = 'text/javascript' if path.suffix == '.js' else \
            mimetypes.guess_type(path.name)[0] or 'application/octet-stream'
        return path.read_bytes(), kind

    def read_mesh(self, relative: str) -> bytes | None:
        path = under(self._mesh_root, relative)
        if path is None:
            return None
        try:
            return path.read_bytes()
        except OSError:
            return None

    def calibrate(self) -> dict:
        try:
            calibration = self._stream.calibrate()
        except (RuntimeError, ValueError) as exc:
            return {'ok': False, 'message': str(exc)}
        return {'ok': True,
                'message': f'缩放 {calibration.scale:.3f}，'
                           f'站立高度 {calibration.stand_height:.3f} m'}

    def state(self) -> dict:
        stats = self._stream.stats()
        base = {
            'connected': stats.connected,
            'calibrated': self._stream.calibrated,
            'frames': stats.frames,
            'dropped': stats.dropped,
            'body_status': stats.status,
            'body_message': STATUS_MESSAGES.get(stats.message, str(stats.message)),
            'error': stats.last_error,
        }
        raw = self._stream.recent_frames()
        if raw:
            base['human'] = np.round(raw[-1].positions, 4).tolist()
        if not self._stream.calibrated:
            return base

        span = self._stream.span()
        if span is None:
            return base
        batch = self._stream.sample(np.array([span[1]]))
        if batch is None:
            return base
        base['angles'] = np.round(batch.joint_pos[0], 5).tolist()
        # 根位置不发：机器人固定画在原点，只有姿态才看得出人的前倾/侧倾。
        base['root_quat'] = np.round(batch.root_quat[0], 5).tolist()
        return base

    def _on_calibrate(self, _request, response):
        result = self.calibrate()
        response.success, response.message = result['ok'], result['message']
        return response


def main() -> None:
    rclpy.init()
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
