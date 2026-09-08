#!/usr/bin/env python3
"""VLA 推理服务与 ``g1_motion_control`` 之间的桥。

    ros2 launch g1_vla_bridge vla_bridge.launch.py
    ros2 service call /vla_bridge/start std_srvs/srv/Trigger
    ros2 topic pub --once /vla_bridge/task std_msgs/msg/String \\
        "{data: 'Pick up the bottled grape juice using the right arm.'}"
    ros2 service call /vla_bridge/stop std_srvs/srv/Trigger

**本文件里没有任何一家 VLA 的协议细节。** 流程是固定的：

    采观测 -> backend.infer() -> 重锚 -> 可选限幅 -> /motion_control/command

接口定义见 ``vla_backend.py``。换一家 VLA = 在 ``backends/`` 下加一个模块 + 改 ``vla_backend`` 参数。

两条线程各干各的：

* **推理线程**收到请求才采观测并调用 backend，播放期间不请求下一段。
* **下发定时器**按 ``action_rate_hz`` 从缓冲里逐个取 waypoint。缓冲走完就停在最后一个
    waypoint 上；manual 等待 ~/next，continuous 请求下一段。

本节点**不做使能**。启动前 ``motion_control`` 必须已经 ``~/engage`` 且
``arms_live=true``，否则 ``~/start`` 直接拒绝；运行中掉了会自动停。
"""

from __future__ import annotations

import json
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from g1_motion_control.command_protocol import join_command
from g1_vla_bridge.transforms import pose_matrix, quat_angle, quat_slerp, reanchor
from g1_vla_bridge.vla_backend import (
    SIDES,
    ActionChunk,
    CameraCalibration,
    Observation,
    backend_parameters,
    load_backend,
)

# 图像槽位 -> ROS 参数名 -> 默认话题。槽位名是规范名，见 vla_backend.IMAGE_SLOTS。
IMAGE_TOPICS = (('head', 'head_image_topic', '/head/camera/color/image_raw'),
                ('left_wrist', 'left_image_topic', '/camera_left/image_raw'),
                ('right_wrist', 'right_image_topic', '/camera_right/image_raw'))

CAMERA_INFO_TOPICS = (
    ('head', 'head_camera_info_topic', '/head/camera/color/camera_info'),
    ('left_wrist', 'left_camera_info_topic', '/camera_left/camera_info'),
    ('right_wrist', 'right_camera_info_topic', '/camera_right/camera_info'),
)

CAMERA_FRAMES = (
    ('head', 'head_camera_frame', 'camera_color_optical_frame'),
    ('left_wrist', 'left_camera_frame', 'camera_left'),
    ('right_wrist', 'right_camera_frame', 'camera_right'),
)


# 编码 -> (每像素字节数, 转 BGR 的 cv2 code)。bgr8 已经是目标格式，不用转。
_BGR_FROM = {'bgr8': (3, None),
             'rgb8': (3, cv2.COLOR_RGB2BGR),
             'yuv422_yuy2': (2, cv2.COLOR_YUV2BGR_YUY2)}


def playback_step(cursor: int, horizon: int) -> tuple[int, int, bool]:
    """返回本拍索引、下一 cursor，以及本拍是否播完整个 chunk。"""
    if horizon <= 0:
        raise ValueError('horizon 必须大于 0')
    index = min(max(0, cursor), horizon - 1)
    finished = index == horizon - 1
    return index, index if finished else index + 1, finished


def inference_request_error(running: bool, active: bool, has_chunk: bool) -> str:
    """返回当前状态拒绝新推理请求的原因；空串表示可以请求。"""
    if not running:
        return '尚未 start'
    if active:
        return '正在推理'
    if has_chunk:
        return '当前 chunk 尚未执行完'
    return ''


def image_to_bgr(msg: Image) -> np.ndarray:
    """ROS ``Image`` -> BGR numpy。头部相机发 rgb8 或 yuv422_yuy2，腕相机发 bgr8。"""
    spec = _BGR_FROM.get(msg.encoding.lower())
    if spec is None:
        raise ValueError(f'不支持的编码 {msg.encoding}，只认 {sorted(_BGR_FROM)}')
    depth, code = spec
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    expected = msg.height * msg.step
    if buf.size < expected:
        raise ValueError(f'图像数据长度 {buf.size} < {expected}')
    # 按 step 拆行再切，否则行尾有填充时 reshape 会直接抛。
    frame = buf[:expected].reshape(msg.height, msg.step)[:, :msg.width * depth]
    frame = frame.reshape(msg.height, msg.width, depth)
    if code is None:
        return frame
    # 必须走 cv2：numpy 的通道反序是负步长拷贝、没有 SIMD，424x240 实测 773 vs 69 us。
    return cv2.cvtColor(frame, code)


def camera_calibration(msg: CameraInfo) -> CameraCalibration:
    """ROS ``CameraInfo`` -> backend 认的内参结构。"""
    return CameraCalibration(intrinsics=(msg.k[0], msg.k[4], msg.k[2], msg.k[5]),
                             size=(int(msg.width), int(msg.height)),
                             distortion=tuple(msg.d) or (0.0,) * 5)


class VlaBridgeNode(Node):

    def __init__(self) -> None:
        super().__init__('vla_bridge')
        p = self.declare_parameter

        # -- backend：协议、坐标系、夹爪换算全在它那边 -------------------------
        name = p('vla_backend', 'cogact_unitree').get_parameter_value().string_value
        params = {key: p(key, default).value
                  for key, default in backend_parameters(name).items()}
        self._backend = load_backend(name, params)
        self._spec = self._backend.spec
        # 把实际发出去的图落盘，用来人工核对"模型到底看到了什么"。置空关掉。
        self._backend.debug_dir = p('debug_image_dir', '/tmp/vla_bridge') \
            .get_parameter_value().string_value

        self._task = p('task_description', '').get_parameter_value().string_value
        self._enabled = {
            'left': p('has_left', True).get_parameter_value().bool_value,
            'right': p('has_right', True).get_parameter_value().bool_value,
        }
        if not any(self._enabled.values()):
            raise ValueError('has_left 和 has_right 不能同时为假')
        # CogACT 的双臂开关由 server 启动参数决定；这里的开关只控制执行。
        self._hold = {s for s in SIDES
                      if p(f'hold_{s}', False).get_parameter_value().bool_value}
        self._active = {s: self._enabled[s] and s not in self._hold for s in SIDES}
        if not any(self._active.values()):
            raise ValueError('hold_left/hold_right 把所有启用的手臂都冻住了')

        self._image_timeout = float(
            p('image_timeout_s', 3.0).get_parameter_value().double_value)

        self._base_frame = p('base_frame', 'torso_link').get_parameter_value().string_value
        self._tip_frames = {
            'left': p('left_tip_frame', 'left_gripper_base').get_parameter_value().string_value,
            'right': p('right_tip_frame', 'right_gripper_base').get_parameter_value().string_value,
        }
        self._camera_frames = {
            slot: p(param, default).get_parameter_value().string_value
            for slot, param, default in CAMERA_FRAMES
        }

        rate = float(p('action_rate_hz', 30.0).get_parameter_value().double_value)
        self._execution_mode = p('execution_mode', 'manual') \
            .get_parameter_value().string_value
        if self._execution_mode not in ('continuous', 'manual'):
            raise ValueError("execution_mode 只能是 'continuous' 或 'manual'")
        # 位置和姿态分开选：位置的标定（frame.origin_in_base）不确定，姿态的
        # （tool_rotation_rpy）是确定的。
        self._delta_pos = p('delta_position', False).get_parameter_value().bool_value
        self._delta_rot = p('delta_rotation', False).get_parameter_value().bool_value
        self._delta = self._delta_pos or self._delta_rot
        if self._delta and self._spec.action_semantics != 'absolute':
            raise ValueError(f'{name} 输出的是 {self._spec.action_semantics} 动作，'
                             '不能再开 delta_position / delta_rotation')
        self._horizon = p('action_horizon', 0).get_parameter_value().integer_value
        self._cartesian_limit_enabled = p('cartesian_limit_enabled', False) \
            .get_parameter_value().bool_value
        self._max_step_pos = float(
            p('max_step_pos', 0.02).get_parameter_value().double_value)
        self._max_step_ori = float(
            p('max_step_ori', 0.10).get_parameter_value().double_value)
        self._retry_delay = float(p('retry_delay_s', 1.0).get_parameter_value().double_value)

        self._lock = threading.Lock()
        self._images: dict[str, tuple[float, Image]] = {}
        self._camera_info: dict[str, CameraInfo] = {}
        self._status: dict = {}
        self._chunk: ActionChunk | None = None
        self._cursor = 0
        self._command: dict[str, np.ndarray] = {}
        self._grip_command = {s: 0.0 for s in SIDES}
        self._infer_ms = 0.0
        self._lead = 0.0
        self._jump = 0.0
        self._error = ''
        self._running = threading.Event()
        self._infer_requested = threading.Event()
        self._inference_active = False
        self._generation = 0

        small = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                           reliability=ReliabilityPolicy.BEST_EFFORT)
        # 图像必须 RELIABLE：一帧拆成成百上千个 UDP 分片，BEST_EFFORT 不重传，丢一个分片
        # 整帧就废。实测 1080p 下 BEST_EFFORT 20 s 收到 0 帧，RELIABLE 3.5 Hz。
        image_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                               reliability=ReliabilityPolicy.RELIABLE)
        command_qos = QoSProfile(depth=4, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
        sensors = ReentrantCallbackGroup()
        for slot, param, default in IMAGE_TOPICS:
            topic = p(param, default).get_parameter_value().string_value
            if slot in self._spec.images.slots:
                self.create_subscription(
                    Image, topic, self._make_image_callback(slot), image_qos,
                    callback_group=sensors)

        self.create_subscription(
            String, p('status_topic', '/motion_control/status')
            .get_parameter_value().string_value,
            self._on_status, 10, callback_group=sensors)
        self.create_subscription(String, '~/task', self._on_task, 10, callback_group=sensors)
        # camera_info 只有几十字节，用 BEST_EFFORT 能同时匹配两种发布端。
        for slot, param, default in CAMERA_INFO_TOPICS:
            topic = p(param, default).get_parameter_value().string_value
            if slot in self._spec.images.slots:
                self.create_subscription(
                    CameraInfo, topic, self._make_camera_info_callback(slot), small,
                    callback_group=sensors)

        self._publisher = self.create_publisher(
            Float64MultiArray, p('command_topic', '/motion_control/command')
            .get_parameter_value().string_value, command_qos)
        self._status_publisher = self.create_publisher(String, '~/status', 10)

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

        control = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / rate, self._on_tick, callback_group=control)
        self.create_timer(0.2, self._publish_status, callback_group=control)
        self.create_service(Trigger, '~/start', self._on_start, callback_group=control)
        self.create_service(Trigger, '~/next', self._on_next, callback_group=control)
        self.create_service(Trigger, '~/stop', self._on_stop, callback_group=control)

        self._alive = True
        self._worker = threading.Thread(target=self._infer_loop, daemon=True)
        self._worker.start()
        held = '，冻结 ' + '/'.join(sorted(self._hold)) if self._hold else ''
        self.get_logger().info(
            'VLA 桥就绪%s，规格 %s，等待 ~/start'
            % (held, json.dumps(self._spec.summary(), ensure_ascii=False)))

    # -- 输入 ---------------------------------------------------------------

    def _make_image_callback(self, slot: str):
        def callback(msg: Image) -> None:
            # 只存消息不解码：解码放到推理线程里按需做，别占回调线程。
            with self._lock:
                self._images[slot] = (time.monotonic(), msg)
        return callback

    def _on_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self._status = status

    def _on_task(self, msg: String) -> None:
        with self._lock:
            self._task = msg.data
        self.get_logger().info(f'任务指令更新为: {msg.data!r}')

    def _make_camera_info_callback(self, slot: str):
        def callback(msg: CameraInfo) -> None:
            with self._lock:
                self._camera_info[slot] = msg
        return callback

    def _arms_ready(self) -> str:
        status = self._status
        if not status:
            return '收不到 /motion_control/status'
        # 只看 arms_live 不够：它在 _estop 里没被清掉，急停后仍是 True（2026-08-17 实测），
        # 那时 FPC 已经反激活，指令发出去没人收。
        state = status.get('state')
        if state not in ('stand', 'running'):
            return f"motion_control 在 {state} 态（{status.get('reason') or '—'}），先调 ~/engage"
        if not status.get('arms_live'):
            return f'motion_control 手臂未接管（state={state}），等站立插值走完'
        return ''

    def _lookup(self, child: str) -> np.ndarray:
        """``base_frame`` 下的 ``[x,y,z,qx,qy,qz,qw]``。"""
        tf = self._tf.lookup_transform(self._base_frame, child, Time()).transform
        return np.array([tf.translation.x, tf.translation.y, tf.translation.z,
                         tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])

    def _measured_pose(self, side: str) -> np.ndarray:
        return self._lookup(self._tip_frames[side])

    def _decode_images(self) -> tuple[dict[str, np.ndarray], str]:
        """解出 spec 要的那几路 BGR。第二个返回值非空 = 这组不能用。不能在持锁时调。"""
        now = time.monotonic()
        with self._lock:
            frames = dict(self._images)
        slots = self._spec.images.slots
        missing = [k for k in slots if k not in frames]
        if missing:
            return {}, f'图像未到达 {missing}'
        stale = [k for k in slots if now - frames[k][0] > self._image_timeout]
        if stale:
            return {}, f'图像过期 {stale}'
        return {k: image_to_bgr(frames[k][1]) for k in slots}, ''

    def _camera_calibrations(self) -> dict[str, CameraCalibration]:
        with self._lock:
            info = dict(self._camera_info)
        return {slot: camera_calibration(msg) for slot, msg in info.items()}

    def _observe(self) -> Observation:
        """采一组观测，全部表示在 ``base_frame`` 里。"""
        frames, reason = self._decode_images()
        if reason:
            raise RuntimeError(reason)
        with self._lock:
            task = self._task
            grippers = dict(self._grip_command)
        camera_poses = {
            slot: pose_matrix(pose[3:], pose[:3])
            for slot, pose in (
                (slot, self._lookup(self._camera_frames[slot]))
                for slot in self._spec.images.slots
            )
        }
        calibrations = self._camera_calibrations()
        return Observation(
            task=task,
            images=frames,
            poses={side: self._measured_pose(side) for side in SIDES},
            grippers=grippers,
            enabled=dict(self._enabled),
            calibrations=calibrations,
            camera_poses=camera_poses)

    # -- 推理线程 -----------------------------------------------------------

    def _infer_loop(self) -> None:
        while self._alive:
            if not self._infer_requested.wait(timeout=0.1):
                continue
            with self._lock:
                if not self._infer_requested.is_set():
                    continue
                self._infer_requested.clear()
                if not self._running.is_set():
                    continue
                generation = self._generation
            try:
                observation = self._observe()
                clock = time.monotonic()
                chunk = self._backend.infer(observation)
                elapsed = (time.monotonic() - clock) * 1e3
                self._accept(chunk, elapsed, generation)
            except Exception as error:  # 网络/服务/数据任何异常都只是这一轮作废。
                self._fail(f'{type(error).__name__}: {error}', generation)

    def _request_inference(self, generation: int | None = None) -> str:
        """空闲时排入一次推理；返回非空表示当前不能接新请求。"""
        with self._lock:
            if generation is not None and generation != self._generation:
                return '请求已取消'
            reason = inference_request_error(
                self._running.is_set(), self._inference_active,
                self._chunk is not None)
            if reason:
                return reason
            self._inference_active = True
            self._infer_requested.set()
        return ''

    def _fail(self, reason: str, generation: int) -> None:
        with self._lock:
            if generation != self._generation or not self._running.is_set():
                return
            self._inference_active = False
            self._error = reason
        self.get_logger().warning(f'推理失败，保持当前目标: {reason}',
                                  throttle_duration_sec=2.0)
        # 网络/服务出错时别原地空转把日志和服务端一起打爆。
        time.sleep(self._retry_delay)
        if self._execution_mode == 'continuous':
            self._request_inference(generation)

    def _accept(self, chunk: ActionChunk, elapsed_ms: float, generation: int) -> None:
        """按需重锚，并记录准入指标。收到的 chunk 已经在 ``base_frame`` 里。"""
        with self._lock:
            if generation != self._generation or not self._running.is_set():
                return
        measured = {side: self._measured_pose(side) for side in SIDES}
        with self._lock:
            anchor = {side: self._command.get(side, measured[side]) for side in SIDES}
        poses, jump = {}, 0.0
        for side in SIDES:
            poses[side] = (reanchor(chunk.poses[side], anchor[side],
                                    self._delta_pos, self._delta_rot)
                           if self._delta else chunk.poses[side])
            if self._active[side]:
                jump = max(jump,
                           float(np.linalg.norm(poses[side][0, :3] - measured[side][:3])))
        lead = max(float(np.linalg.norm(anchor[s][:3] - measured[s][:3])) for s in SIDES)
        with self._lock:
            if generation != self._generation or not self._running.is_set():
                return
            self._inference_active = False
            self._chunk = ActionChunk(poses=poses, grippers=chunk.grippers)
            self._cursor = 0
            self._infer_ms = elapsed_ms
            self._lead = lead
            self._jump = jump
            self._error = ''
        if lead > 10.0 * self._max_step_pos:
            # 限幅是从指令值出发的，指令跑飞了从轨迹上看不出来，只能靠这个报。
            self.get_logger().warning(f'指令领先实测 {lead:.3f} m，手臂没跟上',
                                      throttle_duration_sec=2.0)

    # -- 下发 ---------------------------------------------------------------

    def _limit(self, current: np.ndarray, target: np.ndarray) -> np.ndarray:
        """可选的 VLA 笛卡尔限速；关闭时原样返回 waypoint。"""
        if not self._cartesian_limit_enabled:
            return target.copy()
        out = current.copy()
        delta = target[:3] - current[:3]
        distance = float(np.linalg.norm(delta))
        out[:3] = target[:3] if distance <= self._max_step_pos else \
            current[:3] + delta * (self._max_step_pos / distance)
        angle = quat_angle(current[3:], target[3:])
        out[3:] = target[3:] if angle <= self._max_step_ori else \
            quat_slerp(current[3:], target[3:], self._max_step_ori / angle)
        return out

    def _on_tick(self) -> None:
        if not self._running.is_set():
            return
        with self._lock:
            reason = self._arms_ready()
            chunk, cursor = self._chunk, self._cursor
            command, grip = dict(self._command), dict(self._grip_command)
            if chunk is not None:
                limit = chunk.horizon if self._horizon <= 0 \
                    else min(self._horizon, chunk.horizon)
                index, self._cursor, finished = playback_step(cursor, limit)
            else:
                index, finished = 0, False
        if reason:
            self._stop(f'手臂不可用: {reason}')
            return
        if chunk is None:
            return

        for side in SIDES:
            if not self._active[side]:
                continue                     # 冻结：位姿和夹爪都停在 ~/start 那一刻。
            command[side] = self._limit(command[side], chunk.poses[side][index])
            grip[side] = float(chunk.grippers[side][index])

        if finished:
            finished = all(
                np.linalg.norm(command[side][:3] - chunk.poses[side][index, :3]) < 1e-9
                and quat_angle(command[side][3:], chunk.poses[side][index, 3:]) < 1e-6
                for side in SIDES if self._active[side])

        # 协议只认 14（双臂位姿）和 2（夹爪）这两种长度，拼不到一帧里，发两条。
        self._publisher.publish(Float64MultiArray(
            data=join_command(left=command['left'], right=command['right'])))
        self._publisher.publish(Float64MultiArray(
            data=join_command(grip=[grip[s] for s in SIDES])))
        with self._lock:
            self._command, self._grip_command = command, grip
            if finished and self._chunk is chunk:
                self._chunk = None
                self._cursor = 0
        if finished and self._execution_mode == 'continuous':
            self._request_inference()

    def _publish_status(self) -> None:
        with self._lock:
            chunk, cursor = self._chunk, self._cursor
            payload = {
                'backend': self._spec.name,
                'execution_mode': self._execution_mode,
                'cartesian_limit_enabled': self._cartesian_limit_enabled,
                'running': self._running.is_set(),
                'inference_active': self._inference_active,
                'task': self._task,
                'infer_ms': round(self._infer_ms, 1),
                'lead': round(self._lead, 4),
                'jump': round(self._jump, 4),
                'hold': sorted(self._hold),
                'error': self._error,
                'horizon': 0 if chunk is None else chunk.horizon,
                'cursor': int(cursor),
                'waiting_for_next': all((
                    self._execution_mode == 'manual', self._running.is_set(),
                    chunk is None, not self._inference_active)),
                'grip': {s: round(self._grip_command[s], 3) for s in SIDES},
                'images': sorted(self._images),
                'image_dir': self._backend.debug_dir,
            }
        payload.update(self._backend.stats())
        self._status_publisher.publish(String(data=json.dumps(payload)))

    # -- 服务 ---------------------------------------------------------------

    def _on_start(self, request, response):
        with self._lock:
            reason = self._arms_ready()
            task = self._task
        # 图像必须在放行前就真的可用，否则 ~/start 成功了推理线程才一轮轮撞灰帧。
        reason = reason or self._decode_images()[1]
        if reason:
            response.success, response.message = False, reason
            return response
        if not task:
            response.success = False
            response.message = '任务指令为空，先设 task_description 参数或发 ~/task'
            return response
        try:
            command = {side: self._measured_pose(side) for side in SIDES}
        except Exception as error:
            response.success, response.message = False, f'读不到实测末端位姿: {error}'
            return response
        with self._lock:
            if self._running.is_set():
                response.success, response.message = False, '已经在跑'
                return response
            # 从实测位姿起步，第一帧的限幅才是相对"手臂现在在哪"算的。
            self._command = command
            # 抓取任务从空手开始，先张开。
            opened = float(self._spec.gripper.to_robot(self._spec.gripper.model_open))
            self._grip_command = {s: opened for s in SIDES}
            self._chunk, self._cursor, self._error = None, 0, ''
            self._generation += 1
            self._running.set()
        if self._execution_mode == 'continuous':
            self._request_inference()
        self.get_logger().info(f'开始执行: {task!r}')
        state = '自动请求首段' if self._execution_mode == 'continuous' else '等待 ~/next'
        response.success, response.message = True, f'running: {task}，{state}'
        return response

    def _on_next(self, request, response):
        if self._execution_mode != 'manual':
            response.success, response.message = False, '仅 execution_mode=manual 可用'
            return response
        with self._lock:
            reason = self._arms_ready()
        if reason:
            response.success, response.message = False, reason
            return response
        reason = self._request_inference()
        response.success = not reason
        response.message = reason or '已请求下一段'
        return response

    def _on_stop(self, request, response):
        response.success = self._running.is_set()
        response.message = '已停止' if response.success else '本来就没在跑'
        self._stop('收到 ~/stop')
        return response

    def _stop(self, reason: str) -> None:
        with self._lock:
            if not self._running.is_set():
                return
            self._generation += 1
            self._running.clear()
            self._infer_requested.clear()
            self._chunk, self._cursor, self._inference_active = None, 0, False
        # 停止只是不再发新目标，手臂保持在最后一帧；卸力要走 motion_control 的 ~/estop。
        self.get_logger().warning(f'停止下发: {reason}')

    def shutdown(self) -> None:
        self._alive = False
        self._stop('节点退出')
        self._infer_requested.set()
        self._worker.join(timeout=2.0)
        self._backend.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VlaBridgeNode()
    # 多线程：图像回调很重，单线程执行器会把下发定时器一起拖慢。
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
