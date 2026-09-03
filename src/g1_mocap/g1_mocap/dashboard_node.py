"""动捕重定向的可视化面板。**不控制硬件，也不自己连头显。**

纯消费者：订 ``mocap_node`` 发的 ``/mocap/frame``，在浏览器里用 three.js 搭 URDF
关节树、贴 STL，把重定向结果画出来。上真机之前用它看：人摆一个姿势，G1 会变成什么样。

    ros2 launch g1_mocap mocap.launch.py        # 先起数据源
    ros2 launch g1_mocap dashboard.launch.py    # 再起面板
    # 浏览器打开 http://<机器人IP>:18080

因为不碰头显，它和 ``mocap_node`` **可以同时跑**——头显始终只连 ``mocap_node`` 一个地址。

面板上要盯的两处：

``关节角条``
    条超出灰色区间就是危险信号——参考落在策略训练分布之外，输出什么都不奇怪。
    人站直时所有条都应该落在各自的 default 刻度上（校准把站立位形映射到了那里）。
``骨架叠加``
    人和 G1 的差异一眼可见。腿明显外撇、大腿后摆，说明校准没做或者做的时候人没站直。

正运动学**在浏览器里算**：three.js 的 ``Object3D`` 嵌套每帧本来就要合成矩阵，
后端再算一遍纯属白花钱。所以本节点只解析一次 URDF，运行时转发关节角。
"""

from __future__ import annotations

import json
import mimetypes
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from g1_mocap_msgs.msg import MocapFrame, MocapStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from .skeleton import SMPL_JOINTS
from .urdf import DEFAULT_URDF, resolve_package_path
from .urdf import parse as parse_urdf
from .urdf import under

# SMPL 的运动学树，索引即 XrBodyJointBD 的枚举值。父节点 -1 表示根。
SMPL_PARENTS = (-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17,
                18, 19, 20, 21)


class _Handler(BaseHTTPRequestHandler):

    # 基类把它标成 BaseServer，窄化一下才能拿到 dashboard。
    server: _Server

    # 默认是 HTTP/1.0，每个 mesh 都要新开一条连接：一个模型四十多个零件，握手开销
    # 比传输还大，而且并发一多就有请求被截在半路（浏览器报 CONTENT_LENGTH_MISMATCH，
    # STLLoader 那边只是静默少一个零件）。keep-alive 要求每个响应都带准确的
    # Content-Length，_send 里是强制加的。
    protocol_version = 'HTTP/1.1'

    def log_message(self, *_args) -> None:
        """默认实现往 stderr 刷每一条请求，30 Hz 轮询会把节点日志淹掉。"""

    def do_GET(self) -> None:
        # 未捕获的异常会打死连接，浏览器只看到 ERR_EMPTY_RESPONSE，页面上一点线索都没有。
        try:
            self._route()
        except (BrokenPipeError, ConnectionResetError):
            # 浏览器刷新/关页时会把正在下的 mesh 取消。这时再发 500 只会再炸一次，
            # 并且把一堆无意义的 traceback 刷进节点日志。
            self.close_connection = True
        except Exception as exc:  # noqa: BLE001
            # body 可能已经写出去一半，这条连接的分帧就废了，不能再拿去接下一个请求。
            self.close_connection = True
            self._send(500, 'application/json',
                       json.dumps({'error': f'{type(exc).__name__}: {exc}'}).encode())

    def do_POST(self) -> None:
        try:
            if self.path.rstrip('/') == '/calibrate':
                self._send(200, 'application/json',
                           json.dumps(self.server.dashboard.calibrate()).encode())
            else:
                self._send(404, 'text/plain', b'not found')
        except (BrokenPipeError, ConnectionResetError):
            self.close_connection = True
        except Exception as exc:  # noqa: BLE001
            self.close_connection = True
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
                # 只有 vendor/ 下的第三方库是定版的。自己的 js/css 一缓存，改了代码
                # 刷新也不生效，还查不出来——ES module 连强刷都未必重取。
                cache = ('public, max-age=86400'
                         if path.startswith('/vendor/') else 'no-store')
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
    # 起完立刻赋上，处理线程靠它回到节点。
    dashboard: DashboardNode


class DashboardNode(Node):

    def __init__(self) -> None:
        super().__init__('mocap_dashboard')
        p = self.declare_parameter

        joints = list(p('joints', Parameter.Type.STRING_ARRAY)
                      .get_parameter_value().string_array_value)
        default_pos = list(p('default_joint_pos', Parameter.Type.DOUBLE_ARRAY)
                           .get_parameter_value().double_array_value)
        # 只用来标关节角条上的「策略见过的范围」，按名字对齐，顺序无所谓。
        self._defaults = dict(zip(joints, default_pos))
        self._default_names = joints
        self._default_pos = default_pos
        self._action_joints = set(joints)

        urdf = resolve_package_path(
            p('dashboard_urdf_path', '').get_parameter_value().string_value
            or p('urdf_path', DEFAULT_URDF).get_parameter_value().string_value)
        self.model = parse_urdf(Path(urdf).read_text(encoding='utf-8'), 'pelvis')
        self._share = Path(get_package_share_directory('g1_mocap')) / 'static'
        # mesh 全部从描述包的 model/ 下取，路径参数只允许落在这棵子树里。
        self._mesh_root = Path(get_package_share_directory(
            p('mesh_package', 'unitree_g1_description').get_parameter_value().string_value
        )) / p('mesh_root', 'model').get_parameter_value().string_value

        self._lock = threading.Lock()
        self._frame: MocapFrame | None = None
        self._status: MocapStatus | None = None

        # 必须和发布端一致：RELIABLE 订阅收不到 BEST_EFFORT 发布者，而且只在发现的
        # 那一刻打一次 incompatible QoS 警告，表现就是「话题在、自己收不到」。
        stream_qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                                reliability=ReliabilityPolicy.BEST_EFFORT)
        frame_topic = p('frame_topic', '/mocap/frame').get_parameter_value().string_value
        status_topic = p('status_topic', '/mocap/status').get_parameter_value().string_value
        self.create_subscription(MocapFrame, frame_topic, self._on_frame, stream_qos)
        self.create_subscription(MocapStatus, status_topic, self._on_status, 10)
        self._calibrate = self.create_client(
            Trigger, p('calibrate_service', '/mocap/calibrate')
            .get_parameter_value().string_value)

        http_port = int(p('dashboard_port', 18080).get_parameter_value().integer_value)
        self._http = _Server((p('dashboard_host', '0.0.0.0')
                              .get_parameter_value().string_value, http_port), _Handler)
        self._http.dashboard = self
        threading.Thread(target=self._http.serve_forever, daemon=True).start()
        self.get_logger().info(
            f'面板已就绪: http://<本机IP>:{http_port}   数据源 {frame_topic}')

    def destroy_node(self) -> None:
        self._http.shutdown()
        super().destroy_node()

    def _on_frame(self, message: MocapFrame) -> None:
        with self._lock:
            self._frame = message

    def _on_status(self, message: MocapStatus) -> None:
        with self._lock:
            self._status = message

    ##
    # 给 HTTP 线程用
    ##

    def layout(self) -> dict:
        """静态部分，前端只取一次。限位直接从 URDF 来，不需要运动学库。

        只列动作关节：面板的 URDF 是实机构型，夹爪那 80 个内部连杆也是可动关节，
        全列出来会把真正要看的 29 条淹掉，而且它们压根没有数据。
        """
        joints = [{'name': joint['name'],
                   'lower': joint['limit'][0], 'upper': joint['limit'][1],
                   'default': self._defaults.get(joint['name'], 0.0)}
                  for joint in self.model['joints']
                  if joint['type'] != 'fixed' and 'limit' in joint
                  and joint['name'] in self._action_joints]
        return {'joints': joints,
                'human_joints': list(SMPL_JOINTS),
                'human_parents': list(SMPL_PARENTS)}

    def calibrate(self) -> dict:
        """转发到数据源节点的校准服务。面板自己不持有标定。"""
        if not self._calibrate.wait_for_service(timeout_sec=2.0):
            return {'ok': False, 'message': f'{self._calibrate.srv_name} 不可用，'
                                            f'mocap_node 起来了吗？'}
        future = self._calibrate.call_async(Trigger.Request())
        # 跑在 HTTP 线程里，只能等，不能 spin：spin 会把节点从执行器里摘走。
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return {'ok': False, 'message': '校准服务超时'}
        result = future.result()
        if result is None:
            # future 也可能是因为异常而 done。
            return {'ok': False, 'message': '校准服务没有返回结果'}
        return {'ok': bool(result.success), 'message': result.message}

    def state(self) -> dict:
        with self._lock:
            frame, status = self._frame, self._status
        base = {
            'connected': bool(status.connected) if status else False,
            'calibrated': bool(status.calibrated) if status else False,
            'frames': int(status.frames) if status else 0,
            'dropped': int(status.dropped) if status else 0,
            'body_status': int(status.body_status) if status else 0,
            'body_message': status.body_message_text if status else '',
            'error': status.last_error if status else ('' if frame else '等 /mocap/frame'),
        }
        if frame is None:
            base['joint_names'] = self._default_names
            base['angles'] = self._default_pos
            return base
        base['joint_names'] = list(frame.joint_names)
        base['angles'] = list(frame.joint_positions)
        quat = frame.root.orientation
        base['root_quat'] = [quat.w, quat.x, quat.y, quat.z]
        # 只给高度：x/y 是动捕坐标系里会漂的绝对位置，画面里钉在原地才看得清姿态。
        base['root_z'] = float(frame.root.position.z)
        base['human'] = [[p.x, p.y, p.z] for p in frame.human_joints]
        return base

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


def main() -> None:
    rclpy.init()
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
