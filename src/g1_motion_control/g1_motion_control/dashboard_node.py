#!/usr/bin/env python3
"""双臂只读监控：上层命令、限速指令与实测关节模型三层对照。

    /motion_control/command ─┐
    /motion_control/status ──┼─► [ 本节点：stdlib HTTP + 静态页 ] ◄─轮询─ 浏览器
    /joint_states ───────────┤                                      (three.js FK)
    /robot_description ──────┘

**独立进程，不在控制链路上**。不开就是零开销；开着也只在浏览器轮询到来时才
组一次几百字节的 JSON。

刻意省掉的三件事（和 ``ikt_pose_commander`` 的 dashboard 比）：

* **后端不算正运动学、不依赖 pinocchio**。URDF 解析一次后把关节树发给前端，
  three.js 的 ``Object3D`` 嵌套本来就在算矩阵，再算一遍是白花钱。
    于是每次轮询只发手臂关节角 + 上层/限速位姿，不发整棵 link 变换树。
* **不订阅、不发布任何控制量，也不调用任何服务**。纯只读，所以不需要
  ``ReentrantCallbackGroup`` + ``MultiThreadedExecutor`` 那一套防死锁配置。
* **只保留手臂**：``base_frame`` 之下**含可动关节**的分支才留（头、雷达、相机
  那些 fixed 分支自动被剪掉）。不必配置关节名单，换 URDF 也不用改。

``/joint_states`` 是 100 Hz，回调里只存一个引用；取值推迟到浏览器真的来问的
那一刻（≤轮询频率）。而且**没人看页面就直接退订**：实测这一路的反序列化在
 Jetson 上占 12% 单核（空转 5% -> 订阅后 17%），而监控页大部分时间是关着的。
"""

from __future__ import annotations

import json
import math
import mimetypes
import threading
import time
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, quote, unquote, urlparse

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from g1_motion_control.command_protocol import split_command

_MOVABLE = ('revolute', 'continuous', 'prismatic')
_IDLE_RELEASE_S = 3.0


def _floats(text: str | None, fallback: tuple) -> list:
    if not text:
        return list(fallback)
    values = [float(v) for v in text.split()]
    return values if len(values) == 3 else list(fallback)


def rpy_to_quat(text: str | None) -> list:
    """URDF 的 ``rpy`` -> 四元数 ``[x, y, z, w]``。

    URDF 的 rpy 是**固定轴（外旋）X→Y→Z**，即 ``R = Rz(y)·Ry(p)·Rx(r)``。
    转换放在这里而不是前端，是因为 three.js 的 ``Euler`` 默认是**内旋** ``"XYZ"``
    （``Rx·Ry·Rz``），正好反过来：单轴的关节看不出差别，而夹爪那几个
    ``rpy="1.57 0 1.57"`` 会直接把滑块和连杆甩出手掌（已踩）。放在 Python 里
    还能被单测拿 pinocchio 对着量。
    """
    roll, pitch, yaw = (v / 2.0 for v in _floats(text, (0.0, 0.0, 0.0)))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy]


def parse_urdf(urdf: str, base: str) -> dict:
    """URDF -> ``{'base', 'joints', 'links'}``，只保留 ``base`` 之下的手臂分支。

    留下哪些分支的判据是**分支里有没有可动关节**，不是关节名单：头、相机、
    雷达挂在 ``torso_link`` 上但全是 fixed，于是自动被剪掉；换 URDF 也不用改配置。
    """
    root = ET.fromstring(urdf)
    children = {}
    for element in root.findall('joint'):
        parent = element.find('parent')
        child = element.find('child')
        if parent is None or child is None:
            continue
        origin = element.find('origin')
        axis = element.find('axis')
        mimic = element.find('mimic')
        limit = element.find('limit')
        item = {
            'name': element.get('name'),
            'type': element.get('type', 'fixed'),
            'parent': parent.get('link'),
            'child': child.get('link'),
            'xyz': _floats(origin.get('xyz') if origin is not None else None, (0, 0, 0)),
            'quat': rpy_to_quat(origin.get('rpy') if origin is not None else None),
            'axis': _floats(axis.get('xyz') if axis is not None else None, (1, 0, 0)),
        }
        if limit is not None and limit.get('lower') and limit.get('upper'):
            # 夹爪那两条 spline mimic 链是**按各自限位裁剪**的分段线性拟合：
            # 不裁的话中段的值会跑到区间外，滑块和连杆直接飞出手掌。
            item['limit'] = [float(limit.get('lower')), float(limit.get('upper'))]  # type: ignore
        if mimic is not None:
            item['mimic'] = {'joint': mimic.get('joint'),
                             'multiplier': float(mimic.get('multiplier', 1.0)),
                             'offset': float(mimic.get('offset', 0.0))}
        children.setdefault(item['parent'], []).append(item)

    if base not in children:
        raise ValueError(f'URDF 里 {base} 没有子关节，base_frame 写错了？')

    def branch(link: str) -> list:
        """``link`` 之下的全部关节，深度优先。"""
        out = []
        for joint in children.get(link, ()):
            out.append(joint)
            out.extend(branch(joint['child']))
        return out

    kept = []
    for joint in children[base]:
        limb = [joint] + branch(joint['child'])
        if any(item['type'] in _MOVABLE for item in limb):
            kept.extend(limb)
    if not kept:
        raise ValueError(f'{base} 之下没有可动分支，画不出手臂')

    wanted = {base} | {joint['child'] for joint in kept}
    links = []
    for element in root.findall('link'):
        name = element.get('name')
        if name not in wanted:
            continue
        visuals = []
        for visual in element.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                # 只画 mesh。全 URDF 的非 mesh 几何体就两个：手腕到夹爪中间
                # `*_kwr57b_link` 的圆柱转接件。所以那里会看到 9.5 cm 的空档，
                # **不是夹爪位置算错了**（已拿 pinocchio 的 visual 几何逐个对过）。
                continue
            url = mesh_url(mesh.get('filename') or '')
            if not url:
                continue
            origin = visual.find('origin')
            visuals.append({
                'url': url,
                'xyz': _floats(origin.get('xyz') if origin is not None else None, (0, 0, 0)),
                'quat': rpy_to_quat(origin.get('rpy') if origin is not None else None),
                'scale': _floats(mesh.get('scale'), (1, 1, 1)),
            })
        if visuals:
            links.append({'name': name, 'visuals': visuals})
    if not links:
        # 最常见的原因：URDF 里的 mesh 是裸相对路径。真机上 `control.launch.py`
        # 会先改写成 `package://`，直接发 `final.urdf` 原文则会全部掉。
        raise ValueError(f'{base} 之下一个 mesh 都解析不了（不是 package:// 写法？）')
    return {'base': base, 'joints': kept, 'links': links}


def mesh_url(filename: str) -> str:
    """``package://pkg/rel`` -> 本节点的 ``/mesh`` 代理 URL；其余写法返回空串。

    **只认 package://** 是故意的：``/mesh`` 的参数来自网络，而本节点默认听
    ``0.0.0.0``，放开任意路径就是一个任意文件读。``control.launch.py`` 在发
    ``/robot_description`` 前已经把裸相对路径统一改写成 ``package://``，所以
    真机上不会漏掉任何 mesh。
    """
    if not filename.startswith('package://'):
        return ''
    pkg, _, rel = filename[len('package://'):].partition('/')
    return f'/mesh?pkg={quote(pkg)}&path={quote(rel)}' if pkg and rel else ''


def under(base: Path, relative: str) -> Path | None:
    """把 ``relative`` 接到 ``base`` 下，越界就返回 None。

    校验的是**相对路径本身**，不是拼出来再 ``resolve()`` 比前缀——
    ``--symlink-install`` 把 share/ 里的每个文件都做成了指回 src/ 的符号链接，
    ``resolve()`` 一路跟过去就跑出了 base，所有页面和 mesh 都会 404。
    """
    path = Path(relative)
    if not relative or path.is_absolute() or '..' in path.parts:
        return None
    return base / path


class DashboardNode(Node):

    def __init__(self) -> None:
        super().__init__('motion_control_dashboard')
        p = self.declare_parameter
        self._host = p('bind_host', '0.0.0.0').get_parameter_value().string_value
        self._port = int(p('bind_port', 8181).get_parameter_value().integer_value)
        # 上层命令、限速指令与实测模型都用这个参考系，必须和 motion_control 一致。
        self._base = p('base_frame', 'torso_link').get_parameter_value().string_value

        self._lock = threading.Lock()
        self._model: dict | None = None
        self._status: dict = {}
        self._command_pose: dict[str, list] = {}
        self._joints: JointState | None = None
        self._index: dict[str, int] = {}
        self._names: list = []
        # 页面走 data_files 装到 share/，运行时只认这一条路径。用
        # Path(__file__).parent 在 --symlink-install 下能跑通、干净安装下会 404。
        self._share = Path(get_package_share_directory('g1_motion_control')) / 'dashboard'

        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._stream = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                  reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            String, p('robot_description_topic', '/robot_description')
            .get_parameter_value().string_value, self._on_description, latched)
        self.create_subscription(
            String, p('status_topic', '/motion_control/status')
            .get_parameter_value().string_value, self._on_status, 10)
        self.create_subscription(
            Float64MultiArray,
            p('command_topic', '/motion_control/command').get_parameter_value().string_value,
            self._on_command,
            QoSProfile(depth=4, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.BEST_EFFORT))
        # /joint_states 按需订阅：没人看页面就退订，省掉 12% 单核的反序列化。
        self._joint_topic = p('joint_states_topic', '/joint_states') \
            .get_parameter_value().string_value
        self._joint_sub = None
        self._last_poll = 0.0
        self.create_timer(0.2, self._gate)

        self._server = ThreadingHTTPServer((self._host, self._port), _handler(self))
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        self.get_logger().info(f'监控页已开：http://{self._host}:{self._port}/')

    # -- ROS 输入 --------------------------------------------------------------

    def _on_description(self, message: String) -> None:
        try:
            model = parse_urdf(message.data, self._base)
        except Exception as error:
            self.get_logger().error(f'URDF 解析失败，页面会一直空着: {error}')
            return
        with self._lock:
            self._model = model
        self.get_logger().info(
            f'模型就绪：{len(model["links"])} 个 link、{len(model["joints"])} 个关节')

    def _on_joints(self, message: JointState) -> None:
        # 100 Hz。这里只存引用，取值推迟到浏览器来问的那一刻。
        with self._lock:
            self._joints = message

    def _gate(self) -> None:
        """有人看才订 ``/joint_states``，没人看就退订。

        这一路是 100 Hz，rclpy 反序列化就要 12% 单核（实测），而监控页大部分
        时间是关着的。订/退都在定时器里做，不在 HTTP 线程里碰 ROS 实体。
        """
        with self._lock:
            watching = time.monotonic() - self._last_poll < _IDLE_RELEASE_S
        if watching and self._joint_sub is None:
            self._joint_sub = self.create_subscription(
                JointState, self._joint_topic, self._on_joints, self._stream)
        elif not watching and self._joint_sub is not None:
            self.destroy_subscription(self._joint_sub)
            self._joint_sub = None
            with self._lock:
                self._joints = None

    def _on_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except ValueError:
            return
        with self._lock:
            self._status = status

    def _on_command(self, message: Float64MultiArray) -> None:
        """只观察统一命令总线里的臂块，解析复用 motion_control 的唯一协议。"""
        try:
            chunks = split_command(message.data)
        except ValueError:
            return
        with self._lock:
            for side in ('left', 'right'):
                if side in chunks:
                    self._command_pose[side] = chunks[side].tolist()

    # -- HTTP 取数 --------------------------------------------------------------

    def snapshot(self) -> dict:
        with self._lock:
            self._last_poll = time.monotonic()   # 有人在看，_gate 据此保持订阅
            status, message = self._status, self._joints
            command_pose = dict(self._command_pose)
            if message is not None and message.name != self._names:
                # 名字表变了才重建索引；正常一整场只建一次。
                self._names = list(message.name)
                self._index = {name: i for i, name in enumerate(self._names)}
            index = self._index
            position = list(message.position) if message is not None else []
        q = {name: position[i] for name, i in index.items() if i < len(position)}
        return {
            'q': q,
            'state': status.get('state', ''),
            'stale': status.get('stale', ''),
            # 上层笛卡尔命令与 IK + 关节限速后的指令；实测末端由前端模型直接给出。
            'command_pose': command_pose,
            'limited_pose': status.get('limited_pose') or {},
            'ik_pos_err': status.get('ik_pos_err'),
            'ik_ms': status.get('ik_ms'),
        }

    def model(self) -> dict | None:
        with self._lock:
            return self._model

    def read_static(self, relative: str) -> tuple[bytes, str] | None:
        path = under(self._share, relative)
        if path is None or not path.is_file():
            return None
        kind = 'text/javascript' if path.suffix == '.js' else \
            mimetypes.guess_type(path.name)[0] or 'application/octet-stream'
        return path.read_bytes(), kind

    def read_mesh(self, pkg: str, relative: str) -> bytes | None:
        try:
            base = Path(get_package_share_directory(pkg))
        except (PackageNotFoundError, ValueError):
            return None
        path = under(base, relative)
        if path is None:
            return None
        try:
            return path.read_bytes()
        except OSError:
            return None

    def shutdown(self) -> None:
        self._server.shutdown()


def _handler(node: DashboardNode):

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *args) -> None:
            pass                            # 每次轮询打一行会把终端刷爆

        def _send(self, code: int, body: bytes, kind: str, cache: str = 'no-store') -> None:
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', cache)
            self.end_headers()
            self.wfile.write(body)

        def _json(self, payload) -> None:
            self._send(200, json.dumps(payload, separators=(',', ':')).encode(),
                       'application/json')

        def do_GET(self) -> None:            # noqa: N802 (BaseHTTPRequestHandler 的约定)
            url = urlparse(self.path)
            if url.path in ('/', '/index.html'):
                return self._static('index.html')
            if url.path == '/api/model':
                model = node.model()
                return self._json(model) if model else self._send(
                    503, b'{"error":"waiting for /robot_description"}', 'application/json')
            if url.path == '/api/state':
                return self._json(node.snapshot())
            if url.path == '/mesh':
                query = parse_qs(url.query)
                data = node.read_mesh(unquote((query.get('pkg') or [''])[0]),
                                      unquote((query.get('path') or [''])[0]))
                if data is None:
                    return self._send(404, b'not found', 'text/plain')
                # mesh 一整场都不会变，让浏览器缓存住，别每次刷新都重下 8 MB。
                return self._send(200, data, 'model/stl', 'public, max-age=86400')
            return self._static(url.path.lstrip('/'))

        def _static(self, relative: str) -> None:
            found = node.read_static(relative)
            if found is None:
                return self._send(404, b'not found', 'text/plain')
            body, kind = found
            return self._send(200, body, kind, 'no-cache')

    return Handler


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
