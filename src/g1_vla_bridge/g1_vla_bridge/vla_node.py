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

* **推理线程**收到请求才采观测并调用 backend；async 返回后立即请求下一段。
* **下发定时器**按 ``execution_rate_hz`` 从缓冲里逐个取 waypoint。缓冲走完就停在最后一个
    waypoint 上；manual 等待 ~/next，continuous 请求下一段，async 按时间取融合预测。

本节点**不做使能**。启动前 ``motion_control`` 必须已经 ``~/engage`` 且
``arms_live=true``，否则 ``~/start`` 直接拒绝；运行中掉了会自动停。
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener

from g1_motion_control.command_protocol import join_command
from g1_vla_bridge.record_observation import (
    ObservationBuffer, ObservationTiming, WristReader, measured_state, model_from_description,
)
from g1_vla_bridge.timed_actions import TimedActions
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


def playback_step(cursor: int, horizon: int,
                  skip_intermediate: bool = False) -> tuple[int, int, bool]:
    """返回本拍索引、下一 cursor，以及本拍是否播完整个 chunk。"""
    if horizon <= 0:
        raise ValueError('horizon 必须大于 0')
    if skip_intermediate:
        return horizon - 1, horizon - 1, True
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
        self._observation_max_age = float(p('observation_max_age_s', 0.5).value)
        if not math.isfinite(self._observation_max_age) or self._observation_max_age <= 0:
            raise ValueError('observation_max_age_s 必须是有限正数')

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
        self._action_rate = rate
        self._execution_rate = float(p('execution_rate_hz', rate).value)
        if not math.isfinite(self._execution_rate) or self._execution_rate <= 0:
            raise ValueError('execution_rate_hz 必须是有限正数')
        self._async_alpha = float(p('async_ema_alpha', 0.5).value)
        self._async_timeout = float(p('async_hold_timeout_s', 1.0).value)
        TimedActions(rate, self._async_alpha, 0.0)
        if not math.isfinite(self._async_timeout) or self._async_timeout <= 0:
            raise ValueError('async_hold_timeout_s 必须是有限正数')
        self._execution_mode = p('execution_mode', 'manual') \
            .get_parameter_value().string_value
        if self._execution_mode not in ('continuous', 'manual', 'async'):
            raise ValueError("execution_mode 只能是 'continuous'、'manual' 或 'async'")
        # 位置和姿态分开选：位置的标定（frame.origin_in_base）不确定，姿态的
        # （tool_rotation_rpy）是确定的。
        self._delta_pos = p('delta_position', False).get_parameter_value().bool_value
        self._delta_rot = p('delta_rotation', False).get_parameter_value().bool_value
        self._delta = self._delta_pos or self._delta_rot
        if self._delta and self._spec.action_semantics != 'absolute':
            raise ValueError(f'{name} 输出的是 {self._spec.action_semantics} 动作，'
                             '不能再开 delta_position / delta_rotation')
        self._horizon = p('action_horizon', 0).get_parameter_value().integer_value
        self._skip_intermediate = p('skip_intermediate_waypoints', False) \
            .get_parameter_value().bool_value
        reason = self._mode_error(self._execution_mode)
        if reason:
            raise ValueError(reason)
        self._cartesian_limit_enabled = p('cartesian_limit_enabled', False) \
            .get_parameter_value().bool_value
        self._max_step_pos = float(
            p('max_step_pos', 0.02).get_parameter_value().double_value)
        self._max_step_ori = float(
            p('max_step_ori', 0.10).get_parameter_value().double_value)
        self._retry_delay = float(p('retry_delay_s', 1.0).get_parameter_value().double_value)

        self._lock = threading.Lock()
        self._observations = ObservationBuffer(
            self._spec.images.slots, float(p('observation_rate_hz', 30.0).value),
            float(p('observation_sample_max_age_s', 0.1).value))
        self._model = None
        self._readers = {}
        calibration_path = p('observation_calibration_file', str(
            Path(get_package_share_directory('camera_calibration')) / 'config/calibration.yaml')).value
        self._calibration = yaml.safe_load(Path(calibration_path).read_text()) or {}
        self._camera_info: dict[str, CameraInfo] = {}
        self._status: dict = {}
        self._chunk: ActionChunk | None = None
        self._timed: TimedActions | None = None
        self._async_step = {}
        self._async_merge = {}
        self._async_last_publish = None
        self._cursor = 0
        self._command: dict[str, np.ndarray] = {}
        self._grip_command = {s: 0.0 for s in SIDES}
        self._infer_ms = 0.0
        self._observation_timing = None
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
            if slot in self._spec.images.slots and slot == 'head':
                self.create_subscription(
                    Image, topic, self._make_image_callback(slot), image_qos,
                    callback_group=sensors)

        self.create_subscription(
            JointState, p('joint_states_topic', '/joint_states').value,
            self._on_joints, small, callback_group=MutuallyExclusiveCallbackGroup())
        description_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, p('robot_description_topic', '/robot_description').value,
            self._on_description, description_qos, callback_group=sensors)

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
        self.create_timer(1.0 / self._execution_rate, self._on_tick, callback_group=control)
        self.create_timer(0.2, self._publish_status, callback_group=control)
        self.create_service(Trigger, '~/start', self._on_start, callback_group=control)
        self.create_service(Trigger, '~/next', self._on_next, callback_group=control)
        self.create_service(Trigger, '~/stop', self._on_stop, callback_group=control)
        self.create_service(SetBool, '~/set_auto', self._on_set_auto, callback_group=control)
        self.create_service(SetBool, '~/set_async', self._on_set_async, callback_group=control)
        self.create_service(SetBool, '~/set_skip_intermediate',
                            self._on_set_skip_intermediate, callback_group=control)

        self._alive = True
        for slot, address in (('left_wrist', '97'), ('right_wrist', '98')):
            url = p(f'{slot}_rtsp_url',
                    f'rtsp://admin:123456@192.168.123.{address}/stream0').value
            if slot in self._spec.images.slots:
                self._readers[slot] = WristReader(slot, url, self._observations)
        self._worker = threading.Thread(target=self._infer_loop, daemon=True)
        self._worker.start()
        held = '，冻结 ' + '/'.join(sorted(self._hold)) if self._hold else ''
        self.get_logger().info(
            'VLA 桥就绪%s，规格 %s，等待 ~/start'
            % (held, json.dumps(self._spec.summary(), ensure_ascii=False)))

    # -- 输入 ---------------------------------------------------------------

    def _on_joints(self, msg):
        if len(msg.name) != len(msg.position):
            return
        self._observations.add('joints', Time.from_msg(msg.header.stamp).nanoseconds / 1e9,
                               dict(zip(msg.name, msg.position)), time.monotonic())

    def _on_description(self, msg):
        try:
            model = model_from_description(msg.data)
        except Exception as error:
            self.get_logger().error(f'Invalid robot_description: {error}')
            return
        with self._lock:
            self._model = model

    def _make_image_callback(self, slot: str):
        def callback(msg: Image) -> None:
            self._observations.add(slot, Time.from_msg(msg.header.stamp).nanoseconds / 1e9,
                                   msg, time.monotonic())
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

    def _lookup(self, child: str, stamp: Time | None = None) -> np.ndarray:
        """``base_frame`` 下的 ``[x,y,z,qx,qy,qz,qw]``。"""
        tf = self._tf.lookup_transform(
            self._base_frame, child, Time() if stamp is None else stamp).transform
        return np.array([tf.translation.x, tf.translation.y, tf.translation.z,
                         tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])

    def _measured_pose(self, side: str) -> np.ndarray:
        return self._lookup(self._tip_frames[side])

    def _camera_calibrations(self) -> dict[str, CameraCalibration]:
        with self._lock:
            info = dict(self._camera_info)
        return {slot: camera_calibration(msg) for slot, msg in info.items()}

    def _observe(self) -> Observation:
        now = self.get_clock().now().nanoseconds / 1e9
        monotonic_now = time.monotonic()
        if abs(now - time.time()) > 0.1:
            raise RuntimeError('RTSP wallclock requires a synchronized ROS system clock')
        available_until = None
        if 'head' in self._spec.images.slots:
            latest_tf = self._tf.lookup_transform(
                self._base_frame, self._camera_frames['head'], Time())
            tf_stamp = Time.from_msg(latest_tf.header.stamp).nanoseconds / 1e9
            if tf_stamp > 0:
                available_until = tf_stamp
        selected = self._observations.select(now, self._observation_max_age, available_until)
        with self._lock:
            model, task = self._model, self._task
        if model is None:
            raise RuntimeError('robot_description not received')
        poses, camera_poses, grippers = measured_state(
            model, self._base_frame, self._tip_frames,
            {slot: self._camera_frames[slot] for slot in self._spec.images.slots if slot != 'head'},
            selected.joints.value)
        if 'head' in self._spec.images.slots:
            pose = self._lookup(self._camera_frames['head'],
                                Time(nanoseconds=round(selected.reference * 1e9)))
            camera_poses['head'] = pose_matrix(pose[3:], pose[:3])
        frames, calibrations = {}, self._camera_calibrations()
        stamps = {'joints': selected.joints.stamp}
        for slot, sample in selected.images.items():
            if monotonic_now - sample.received > self._image_timeout:
                raise RuntimeError(f'image arrival stale: {slot}')
            stamps[slot] = sample.stamp
            if slot == 'head':
                frames[slot] = image_to_bgr(sample.value)
            else:
                frames[slot] = sample.value.to_ndarray(format='bgr24')
                height, width = frames[slot].shape[:2]
                entries = self._calibration.get('intrinsics', {}).get(self._camera_frames[slot], [])
                entry = next((item for item in entries
                              if (item['width'], item['height']) == (width, height)), None)
                if entry is None:
                    raise RuntimeError(f'missing exact wrist calibration: {slot} {width}x{height}')
                matrix = entry['camera_matrix']
                calibrations[slot] = CameraCalibration(
                    (matrix[0], matrix[4], matrix[2], matrix[5]), (width, height),
                    tuple(entry['distortion_coefficients']))
        acquired = monotonic_now - (now - selected.reference)
        timing = ObservationTiming(round(selected.reference * 1e9), acquired,
                                   {slot: now - stamp for slot, stamp in stamps.items()},
                                   max(stamps.values()) - min(stamps.values()))
        if time.monotonic() - acquired > self._observation_max_age:
            raise RuntimeError('observation processing exceeded maximum age')
        with self._lock:
            self._observation_timing = timing
        return Observation(task, frames, poses, grippers, dict(self._enabled),
                           calibrations, camera_poses, acquired)

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
                clock = time.monotonic()
                observation = self._observe()
                origin = clock
                if self._execution_mode == 'async':
                    origin = observation.acquired_monotonic
                    if origin is None or not math.isfinite(origin) or origin > time.monotonic():
                        raise RuntimeError('async 需要有效的观测获取时间')
                chunk = self._backend.infer(observation)
                elapsed = (time.monotonic() - clock) * 1e3
                self._accept(chunk, elapsed, generation, requested_at=origin)
            except Exception as error:  # 网络/服务/数据任何异常都只是这一轮作废。
                self._fail(f'{type(error).__name__}: {error}', generation)

    def _request_inference(self, generation: int | None = None) -> str:
        """空闲时排入一次推理；返回非空表示当前不能接新请求。"""
        with self._lock:
            if generation is not None and generation != self._generation:
                return '请求已取消'
            reason = inference_request_error(
                self._running.is_set(), self._inference_active,
                self._chunk is not None and self._execution_mode != 'async')
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
        if self._execution_mode in ('continuous', 'async'):
            self._request_inference(generation)

    def _accept(self, chunk: ActionChunk, elapsed_ms: float, generation: int,
                requested_at: float | None = None) -> None:
        """按需重锚，并记录准入指标。收到的 chunk 已经在 ``base_frame`` 里。"""
        with self._lock:
            if generation != self._generation or not self._running.is_set():
                return
            asynchronous = self._execution_mode == 'async'
            if asynchronous:
                if requested_at is None or self._timed is None:
                    raise ValueError('async 缺少请求时间或时间队列')
                now = time.monotonic()
                if now >= self._timed.end + self._async_timeout:
                    raise RuntimeError('async 动作队列断供超时')
                limit = chunk.horizon if self._horizon <= 0 else min(self._horizon, chunk.horizon)
                prediction = ActionChunk(
                    poses={side: chunk.poses[side][:limit] for side in SIDES},
                    grippers={side: chunk.grippers[side][:limit] for side in SIDES})
                accepted = self._timed.merge(prediction, requested_at, now)
                self._async_merge = dict(self._timed.last_merge)
                self._inference_active = False
                self._infer_ms = elapsed_ms
                self._error = '' if accepted else '推理结果已全部过期，保持当前目标'
        if asynchronous:
            self._request_inference(generation)
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
        if self._execution_mode == 'async':
            self._on_async_tick()
            return
        with self._lock:
            reason = self._arms_ready()
            chunk, cursor = self._chunk, self._cursor
            command, grip = dict(self._command), dict(self._grip_command)
            if chunk is not None:
                limit = chunk.horizon if self._horizon <= 0 \
                    else min(self._horizon, chunk.horizon)
                index, self._cursor, finished = playback_step(
                    cursor, limit, self._skip_intermediate)
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

    def _on_async_tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            reason = self._arms_ready()
            queue = self._timed
            if queue is None:
                return
            if not reason and now >= queue.end + self._async_timeout:
                reason = 'async 动作队列断供超时'
            target = None if reason else queue.take(now)
            command, grip = dict(self._command), dict(self._grip_command)
        if reason:
            self._stop(reason)
            return
        if target is None:
            return
        step = {
            side: {
                'position_m': float(np.linalg.norm(target[0][side][:3] - command[side][:3])),
                'rotation_rad': quat_angle(command[side][3:], target[0][side][3:]),
                'gripper_rad': abs(float(target[1][side]) - grip[side]),
            }
            for side in SIDES if self._active[side]
        }
        for side in SIDES:
            if self._active[side]:
                command[side] = self._limit(command[side], target[0][side])
                grip[side] = target[1][side]
        self._publisher.publish(Float64MultiArray(
            data=join_command(left=command['left'], right=command['right'])))
        self._publisher.publish(Float64MultiArray(
            data=join_command(grip=[grip[side] for side in SIDES])))
        with self._lock:
            self._command, self._grip_command = command, grip

            self._async_step = step
            self._async_last_publish = now

    def _publish_status(self) -> None:
        with self._lock:
            chunk, cursor = self._chunk, self._cursor
            payload = {
                'backend': self._spec.name,
                'execution_mode': self._execution_mode,
                'action_rate_hz': self._action_rate,
                'execution_rate_hz': self._execution_rate,
                'skip_intermediate_waypoints': self._skip_intermediate,
                'cartesian_limit_enabled': self._cartesian_limit_enabled,
                'running': self._running.is_set(),
                'inference_active': self._inference_active,
                'async_ema_alpha': self._async_alpha,
                'async_hold_timeout_s': self._async_timeout,
                'async_pending': len(self._timed.samples) if self._timed else 0,
                'async_merge': self._async_merge,
                'async_target_step': self._async_step,
                'async_since_publish_s': (
                    time.monotonic() - self._async_last_publish
                    if self._async_last_publish is not None else None),
                'async_buffer_s': round(max(0., self._timed.end - time.monotonic()), 3)
                if self._timed else 0.,
                'task': self._task,
                'infer_ms': round(self._infer_ms, 1),
                'observation_sample_age_s': (
                    {slot: round(age, 4) for slot, age in self._observation_timing.ages_s.items()}
                    if self._observation_timing else {}),
                'observation_skew_s': (round(self._observation_timing.skew_s, 4)
                                       if self._observation_timing else None),
                'observation_age_s': (round(
                    time.monotonic() - self._observation_timing.acquired_monotonic, 4)
                                      if self._observation_timing else None),
                'lead': round(self._lead, 4) if self._execution_mode != 'async' else None,
                'jump': round(self._jump, 4) if self._execution_mode != 'async' else None,
                'hold': sorted(self._hold),
                'error': self._error,
                'horizon': 0 if chunk is None else chunk.horizon,
                'cursor': int(cursor),
                'waiting_for_next': all((
                    self._execution_mode == 'manual', self._running.is_set(),
                    chunk is None, not self._inference_active)),
                'grip': {s: round(self._grip_command[s], 3) for s in SIDES},
                'images': sorted(self._spec.images.slots),
                'wrist_stream_errors': {slot: reader.error for slot, reader in self._readers.items()},
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
        if not reason:
            try:
                self._observe()
            except Exception as error:
                reason = str(error)
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
            self._observation_timing = None
            self._async_step = {}
            self._async_merge = {}
            self._async_last_publish = None
            self._timed = (TimedActions(self._action_rate, self._async_alpha, time.monotonic())
                           if self._execution_mode == 'async' else None)
            self._generation += 1
            self._running.set()
        if self._execution_mode in ('continuous', 'async'):
            self._request_inference()
        self.get_logger().info(f'开始执行: {task!r}')
        state = '等待 ~/next' if self._execution_mode == 'manual' else '自动请求首段'
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

    def _on_set_auto(self, request, response):
        reason = self._set_execution_mode('continuous' if request.data else 'manual')
        response.success = not reason
        response.message = reason or f'执行模式: {self._execution_mode}'
        return response

    def _on_set_async(self, request, response):
        reason = self._set_execution_mode('async' if request.data else 'manual')
        response.success = not reason
        response.message = reason or f'执行模式: {self._execution_mode}'
        return response

    def _mode_error(self, mode: str) -> str:
        if mode == 'async' and (self._delta or self._spec.action_semantics != 'absolute'):
            return 'async 需要绝对动作，且 delta_position/delta_rotation 必须关闭'
        if mode == 'async' and self._skip_intermediate:
            return 'async 不支持末点直达，请先关闭 skip_intermediate_waypoints'
        return ''

    def _set_execution_mode(self, mode: str) -> str:
        with self._lock:
            if mode == self._execution_mode:
                return ''
            if self._running.is_set():
                return '切换执行模式前请先 /stop'
            reason = self._mode_error(mode)
            if reason:
                return reason
            self._generation += 1
            self._infer_requested.clear()
            self._chunk, self._cursor, self._inference_active = None, 0, False
            self._timed = None
            self._execution_mode = mode
        return ''

    def _on_set_skip_intermediate(self, request, response):
        with self._lock:
            if self._execution_mode == 'async' and request.data:
                response.success, response.message = False, 'async 不支持末点直达'
                return response
            self._skip_intermediate = bool(request.data)
        response.success = True
        response.message = ('已跳过中间 waypoint' if self._skip_intermediate
                            else '已恢复逐 waypoint 执行')
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
            self._timed = None
            self._error = reason
        # 停止只是不再发新目标，手臂保持在最后一帧；卸力要走 motion_control 的 ~/estop。
        self.get_logger().warning(f'停止下发: {reason}')

    def shutdown(self) -> None:
        self._alive = False
        self._stop('节点退出')
        for reader in self._readers.values():
            reader.close()
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
