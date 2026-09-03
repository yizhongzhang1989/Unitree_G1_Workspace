"""G1 全身动作跟踪节点（RGMT + 全局位置）：读一段参考动作，输出 31 轴关节位置目标。

和同工作区的 ``g1_motion_control`` 是**并列**的一层，不要同时启动——两者都往
``/forward_position_controller/commands`` 写，同时跑就是两个策略抢同一组电机。

分工：
* 29 轴（12 腿 + 3 腰 + 14 臂）由 RGMT 策略驱动；
* 2 个夹爪偏心轴由 VR trigger 控制；策略训练时没有它们，观测恒填 0。

状态机与 ``g1_motion_control`` 一致，便于同一套操作流程::

    IDLE --engage--> (激活控制器) --start--> STAND --(插值到位)--> RUNNING
                                                 任何时候 estop --> ESTOP

**与旧 g1_gmt_tracking 的关键差异**：本策略的参考窗口里有 15 维是"参考 key body 相对
机器人躯干"的位移，其中头 3 维就是漂移量本身。所以必须接里程计；录制动作在
``~/start`` 对齐，实时动捕在每次 squeeze 接合时对齐。旧包只锁偏航。
**两种参考源**，``reference_source`` 二选一：

``motion``（默认）
    从 ``motion_dir`` 读一段录好的 NPZ，放完回 IDLE。
``mocap``
    订 ``g1_mocap`` 的 ``/mocap/frame``，机器人实时跟着人走。参考没有终点，
    只能靠 estop 停。代价是 **0.34 s 的端到端延迟**：策略要 0.3 s 的前瞻，
    而实时流没有未来，只能把播放头往后挪。

    本节点**不碰头显**——收头显、跑重定向、做校准全在 ``mocap_node`` 那边，
    所以要先起它。这样三个节点（跟踪层 / mocap_node / dashboard）能同时跑。
"""

from __future__ import annotations

import math
import threading
import time
from enum import Enum
from functools import partial
from typing import TYPE_CHECKING

import numpy as np
import rclpy
from controller_manager_msgs.srv import SwitchController
from g1_mocap_msgs.msg import MocapControllers, MocapFrame, MocapStatus
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from .motion_library import MotionLibrary
from .mocap_gate import MocapFrameGate, ZeroReferenceFactory
from .odometry import OdometryFuser

if TYPE_CHECKING:
    # 只为类型标注。运行时这两个是延迟导入的——motion 那条路不该依赖 g1_mocap。
    from g1_mocap.consumer import FrameBuffer

    from .mocap_clip import MocapClip
from .rgmt_runtime import RgmtPolicy, resolve_policy_path, spec_matches
from .rotations import (
    quat_apply,
    quat_from_xyzw,
    rotate_inverse,
    torso_pos_from_pelvis,
    torso_quat_from_pelvis,
)

WAIST_JOINTS = ('waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint')
GRAVITY_W = np.array([0.0, 0.0, -1.0])


def _tracking_from_squeezes(current: bool, left: float, right: float,
                            press: float, release: float) -> bool:
    """双手 squeeze 迟滞门：两侧都越过同一条边界才切换。"""
    threshold = release if current else press
    return float(left) >= threshold and float(right) >= threshold


def _policy_joint_state(measured: np.ndarray, velocity: np.ndarray,
                        obs_slots: np.ndarray, uncontrolled_slots: np.ndarray,
                        default_joint_pos: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """组装策略本体观测；策略不控制的轴保持训练时默认值。"""
    position = measured[obs_slots].copy()
    speed = velocity[obs_slots].copy()
    position[uncontrolled_slots] = default_joint_pos[uncontrolled_slots]
    speed[uncontrolled_slots] = 0.0
    return position, speed


def _gripper_from_trigger(trigger: float, opened: float, closed: float) -> float:
    value = min(max(float(trigger), 0.0), 1.0)
    return opened + (closed - opened) * value


def _origin_stamp(message: str) -> float:
    for field in message.split():
        if field.startswith('origin_stamp='):
            return float(field.partition('=')[2])
    raise ValueError('响应缺少 origin_stamp')


class State(Enum):
    IDLE = 'idle'
    STAND = 'stand'
    RUNNING = 'running'
    ESTOP = 'estop'


ACTIVE_STATES = (State.STAND, State.RUNNING)


class RgmtTrackingNode(Node):

    def __init__(self) -> None:
        super().__init__('rgmt_tracking')
        p = self.declare_parameter

        # 名单类参数一律显式声明类型：默认值给 [] 会被 rclpy 推断成 BYTE_ARRAY，
        # 配置里注入字符串数组时直接抛 InvalidParameterTypeException。
        joints = list(p('joints', Parameter.Type.STRING_ARRAY)
                      .get_parameter_value().string_array_value)
        if not joints:
            raise RuntimeError('参数 joints 为空：应由 launch 从 FPC 的配置注入 31 轴顺序')
        self._joints = joints
        obs_joints = list(p('obs_joints', Parameter.Type.STRING_ARRAY)
                          .get_parameter_value().string_array_value)
        action_joints = list(p('action_joints', Parameter.Type.STRING_ARRAY)
                             .get_parameter_value().string_array_value)
        gripper_joints = list(p('gripper_joints', Parameter.Type.STRING_ARRAY)
                              .get_parameter_value().string_array_value)
        key_bodies = list(p('reference_key_bodies', Parameter.Type.STRING_ARRAY)
                          .get_parameter_value().string_array_value)

        lower = np.asarray(p('target_lower_limits', Parameter.Type.DOUBLE_ARRAY)
                           .get_parameter_value().double_array_value, dtype=np.float64)
        upper = np.asarray(p('target_upper_limits', Parameter.Type.DOUBLE_ARRAY)
                           .get_parameter_value().double_array_value, dtype=np.float64)
        self._max_offset = float(p('max_anchor_offset_m', 0.3)
                                 .get_parameter_value().double_value)
        self._policy = RgmtPolicy(
            p('policy_path', '').get_parameter_value().string_value,
            target_lower=lower, target_upper=upper,
            max_anchor_offset_m=self._max_offset)
        spec = self._policy.spec
        spec_matches(spec, obs_joints, action_joints, key_bodies)

        names = list(spec.all_body_names)
        # 训练侧前瞻特征取 body_names[0] 作根位姿，这里保持同一约定。
        self._lock = threading.Lock()
        self._state = State.IDLE
        self._reason = ''
        gripper_limits = np.asarray(
            p('gripper_limits', [0.0, 2.76377472169236])
            .get_parameter_value().double_array_value, dtype=np.float64)
        if gripper_limits.shape != (2,) or gripper_limits[0] > gripper_limits[1]:
            raise RuntimeError('gripper_limits 必须是 [闭合角, 打开角]')
        self._gripper_closed, self._gripper_open = gripper_limits
        self._gripper_targets = np.full(2, self._gripper_open)
        self._tracking = False
        self._mocap_had_tracking = False
        self._mocap_align_pending = None
        self._stand_tracks_mocap = False
        self._controllers_stamp = 0.0
        self._controllers_timeout = float(p('mocap_controllers_timeout_s', 0.2)
                                          .get_parameter_value().double_value)
        self._squeeze_press = float(p('squeeze_press_threshold', 0.7)
                                    .get_parameter_value().double_value)
        self._squeeze_release = float(p('squeeze_release_threshold', 0.5)
                                      .get_parameter_value().double_value)
        if not 0.0 <= self._squeeze_release < self._squeeze_press <= 1.0:
            raise RuntimeError('squeeze 门限必须满足 0 <= release < press <= 1')

        self._source = p('reference_source', 'motion').get_parameter_value().string_value
        if self._source not in ('motion', 'mocap'):
            raise RuntimeError(f"reference_source 只能是 motion 或 mocap，收到 {self._source!r}")
        self._mocap: FrameBuffer | None = None
        # 和 _clip 指向同一个对象，只是把「这条路才有 stale/describe」写进类型里。
        self._mocap_clip: MocapClip | None = None
        self._motions: MotionLibrary | None = None
        if self._source == 'mocap':
            self._clip = self._mocap_clip = self._build_mocap_clip(spec, action_joints)
        else:
            self._motions = MotionLibrary(
                resolve_policy_path(p('motion_dir', '').get_parameter_value().string_value),
                anchor_index=names.index(spec.anchor_body_name),
                root_index=0,
                key_indexes=[names.index(n) for n in spec.reference_key_bodies],
                policy_joint_ids=self._policy.action_slots(),
            )
            default_motion = p('motion', '').get_parameter_value().string_value
            self._clip = self._motions.get(default_motion or self._motions.names[0])
        self._loop = bool(p('loop', False).get_parameter_value().bool_value)

        self._rate = float(p('control_rate_hz', 50.0).get_parameter_value().double_value)
        if abs(1.0 / self._rate - spec.control_dt) > 1e-6:
            raise RuntimeError(
                f'控制周期与训练不一致: 本节点 {1.0 / self._rate:.4f}s, '
                f'ONNX 契约 {spec.control_dt:.4f}s')
        self._stand_s = float(p('stand_s', 3.0).get_parameter_value().double_value)
        self._timeout = float(p('state_timeout_s', 0.2).get_parameter_value().double_value)
        self._tilt_limit = float(p('tilt_limit_rad', 0.8).get_parameter_value().double_value)
        self._odom = OdometryFuser(
            mode=p('odometry_mode', 'fused').get_parameter_value().string_value,
            odom_timeout_s=float(p('odom_timeout_s', 0.2).get_parameter_value().double_value),
            lidar_timeout_s=float(p('lidar_timeout_s', 1.0).get_parameter_value().double_value),
            correction_tau_s=float(p('lidar_correction_tau_s', 2.0)
                                   .get_parameter_value().double_value),
        )

        self._action_slots = self._slots(action_joints)
        self._gripper_slots = self._slots(gripper_joints)
        self._obs_slots = self._slots(obs_joints)
        policy_action_slots = self._policy.action_slots()
        self._policy_uncontrolled_slots = np.setdiff1d(
            np.arange(len(obs_joints)), policy_action_slots)
        self._waist_slots = self._slots(list(WAIST_JOINTS))
        if len(action_joints) + len(gripper_joints) != len(joints):
            raise RuntimeError(
                f'动作 {len(action_joints)} + 夹爪 {len(gripper_joints)} '
                f'必须正好盖满 {len(joints)} 轴')

        self._measured: np.ndarray | None = None
        self._measured_vel: np.ndarray | None = None
        self._joint_stamp = 0.0
        self._imu_quat: np.ndarray | None = None
        self._imu_omega = np.zeros(3)
        self._imu_stamp = 0.0
        self._stand_from: np.ndarray | None = None
        self._stand_start = 0.0
        self._name_to_index: dict[str, int] = {}

        # 必须 BEST_EFFORT：RELIABLE 订阅收不到 BEST_EFFORT 发布者（反过来可以），
        # 而 g1_localization 的 ~/torso_pose 正是 BEST_EFFORT，写成 RELIABLE 时
        # 一条都收不到，只在发现的那一刻打一次 incompatible QoS 警告。
        # FPC 那侧的 ~/commands 也是 BEST_EFFORT depth=1。
        stream = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.BEST_EFFORT)
        self._message = Float64MultiArray()
        self._publisher = self.create_publisher(
            Float64MultiArray,
            p('target_topic', '/forward_position_controller/commands')
            .get_parameter_value().string_value, stream)
        self._status_publisher = self.create_publisher(String, '~/status', 10)

        control = MutuallyExclusiveCallbackGroup()
        # 状态订阅单独一组：和 50 Hz 控制环挤在一个互斥组里，控制环一慢就会把状态
        # 回调饿死，看门狗读到的是「我上次跑的时间」而不是数据到达时间，于是误急停。
        state = MutuallyExclusiveCallbackGroup()
        services = ReentrantCallbackGroup()
        self.create_subscription(JointState, '/joint_states', self._on_joint_states,
                                 stream, callback_group=state)
        self.create_subscription(
            Imu, p('imu_topic', '/pelvis_imu_broadcaster/imu').get_parameter_value().string_value,
            self._on_imu, stream, callback_group=state)
        # depth 必须是 1：实测 depth=50 时接收时刻恒定滞后约 48 ms，静默且不报错。
        self.create_subscription(
            Odometry, p('odom_topic', '/dog_odom').get_parameter_value().string_value,
            self._on_odom, stream, callback_group=state)
        self.create_subscription(
            Odometry,
            p('lidar_pose_topic', '/g1_localization/torso_pose')
            .get_parameter_value().string_value,
            self._on_lidar, stream, callback_group=state)
        self.create_subscription(String, '~/select_motion', self._on_select,
                                 10, callback_group=services)
        self.create_timer(1.0 / self._rate, self._control, callback_group=control)
        self.create_timer(0.1, self._publish_status, callback_group=control)

        manager = p('controller_manager', '/controller_manager').get_parameter_value().string_value
        self._controller = p('controller_name', 'forward_position_controller') \
            .get_parameter_value().string_value
        self._switch = self.create_client(
            SwitchController, f'{manager}/switch_controller', callback_group=services)
        self._set_origin = self.create_client(
            Trigger,
            p('set_origin_service', '/g1_localization/set_origin')
            .get_parameter_value().string_value,
            callback_group=services)
        self.create_service(Trigger, '~/engage', self._on_engage, callback_group=services)
        self.create_service(Trigger, '~/start', self._on_start, callback_group=services)
        self.create_service(Trigger, '~/estop', self._on_estop, callback_group=services)

        library = self._motions
        source = (f'实时动捕 {self._mocap_topic}' if library is None
                  else f'动作库 {len(library.names)} 段, 当前 {self._clip.name}'
                       f' ({self._clip.duration_s:.1f}s)')
        self.get_logger().info(
            f'RGMT 跟踪就绪: 历史 H={spec.history_len}, 参考窗口 '
            f'{len(spec.window_offsets)}x{spec.token_dim}; 里程计 {self._odom.mode}; '
            f'参考源 {source}')

    def _build_mocap_clip(self, spec, action_joints: list[str]):
        """接实时动捕。数据来自 ``g1_mocap`` 的 ``/mocap/frame``，本节点不碰头显。

        动捕那边的 29 轴顺序未必和策略一致，所以每帧都拿 ``joint_names`` 校一遍：
        对不上就丢帧，而不是静默地把左右腿接反。
        """
        from g1_mocap.consumer import FrameBuffer
        from g1_mocap.kinematics import G1Kinematics
        from g1_mocap.urdf import DEFAULT_URDF, resolve_package_path

        from .mocap_clip import MocapClip, lead_frames_for

        p = self.declare_parameter
        self._mocap_joints = list(action_joints)
        self._mocap_key_bodies = list(spec.reference_key_bodies)
        buffer = FrameBuffer(
            n_joints=len(action_joints), n_keys=len(spec.reference_key_bodies),
            buffer_s=float(p('mocap_buffer_s', 2.0).get_parameter_value().double_value))
        self._mocap = buffer
        kin = G1Kinematics(resolve_package_path(DEFAULT_URDF), action_joints)
        self._mocap_kin = kin
        stand = spec.default_joint_pos[self._policy.action_slots()]
        key_local = kin.key_body_pos(stand, spec.reference_key_bodies)
        anchor_local = kin.frame_pos(spec.anchor_body_name)
        anchor_rot = kin.frame_rot(spec.anchor_body_name)
        gate = MocapFrameGate(
            buffer,
            ZeroReferenceFactory(stand, anchor_local, anchor_rot, key_local),
        )

        self._mocap_topic = p('mocap_frame_topic', '/mocap/frame') \
            .get_parameter_value().string_value
        status_topic = p('mocap_status_topic', '/mocap/status') \
            .get_parameter_value().string_value
        # 单独一个互斥组：动捕是 90 Hz，和 500 Hz 的 odom 挤同一组会互相饿。
        mocap_group = MutuallyExclusiveCallbackGroup()
        # QoS 必须和发布端一致：RELIABLE 订阅收不到 BEST_EFFORT 发布者，而且只在发现
        # 那一刻打一次警告，表现就是「话题在、自己收不到」。
        stream = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(MocapFrame, self._mocap_topic,
                                 partial(self._on_mocap_frame, gate),
                                 stream, callback_group=mocap_group)
        self.create_subscription(MocapStatus, status_topic,
                                 buffer.push_status, 10,
                                 callback_group=mocap_group)
        controllers_topic = p('mocap_controllers_topic', '/mocap/controllers') \
            .get_parameter_value().string_value
        self.create_subscription(MocapControllers, controllers_topic,
                                 self._on_mocap_controllers, 10,
                                 callback_group=mocap_group)

        margin = int(p('mocap_lead_margin_frames', 2)
                     .get_parameter_value().integer_value)
        lead = lead_frames_for(spec.window_offsets, margin=margin)
        self.get_logger().warn(
            f'实时动捕参考：端到端延迟 {lead * spec.control_dt:.2f}s（{lead} 拍）。'
            f'这是策略要未来 {int(max(spec.window_offsets))} 帧带来的下界，砍不掉。')
        return MocapClip(
            buffer, control_dt=spec.control_dt, lead_frames=lead,
            stand_joint_pos=stand,
            stale_timeout_s=float(p('mocap_stale_timeout_s', 0.3)
                                  .get_parameter_value().double_value))

    def _on_mocap_frame(self, gate: MocapFrameGate, message: MocapFrame) -> None:
        """每帧核对两份名单。两者都是**顺序敏感**且错了不报错的。

        关节名单错位 = 左右腿指令互换；key body 名单错位 = 参考窗口后 30 维整段
        对不上号，策略照跑但完全无意义。宁可丢帧。
        """
        if list(message.joint_names) != self._mocap_joints:
            self.get_logger().error(
                f'{self._mocap_topic} 的关节名单和策略对不上，丢帧。'
                f'动捕端的 joints 参数得和策略的 action_joints 同一份。',
                throttle_duration_sec=5.0)
            return
        if list(message.key_body_names) != self._mocap_key_bodies:
            self.get_logger().error(
                f'{self._mocap_topic} 的 key body 名单和策略契约对不上，丢帧。'
                f'期望 {self._mocap_key_bodies}，收到 {list(message.key_body_names)}。',
                throttle_duration_sec=5.0)
            return
        with self._lock:
            if self._tracking:
                mode = 'live'
            elif self._mocap_had_tracking:
                mode = 'hold'
            else:
                mode = 'default'
            pending = self._mocap_align_pending if mode == 'live' else None
        accepted = gate.push(message, mode=mode)
        if accepted and pending is not None:
            payload = gate.last_payload
            if payload is None:
                return
            position = payload.anchor.position
            orientation = payload.anchor.orientation
            reference_ground_z = min(
                point.z for point in payload.key_body_positions[-2:])
            self._mocap_clip.align_from_reference(
                pending[0], pending[1],
                pending[2],
                float(message.header.stamp.sec) + message.header.stamp.nanosec * 1e-9,
                np.array([position.x, position.y, position.z]),
                quat_from_xyzw((orientation.x, orientation.y,
                                orientation.z, orientation.w)),
                reference_ground_z,
            )
            with self._lock:
                if self._mocap_align_pending is pending:
                    self._mocap_align_pending = None

    def _on_mocap_controllers(self, message: MocapControllers) -> None:
        left, right = message.left, message.right
        connected = left.connected and right.connected
        with self._lock:
            self._controllers_stamp = self._now()
            tracking = connected and _tracking_from_squeezes(
                self._tracking, left.squeeze, right.squeeze,
                self._squeeze_press, self._squeeze_release)
            changed = tracking != self._tracking
            self._tracking = tracking
            self._mocap_had_tracking = self._mocap_had_tracking or tracking
            if changed and tracking and self._measured is not None and self._imu_quat is not None:
                robot_pos = self._odom.torso_position()
                if robot_pos is not None:
                    robot_quat = self._odom.orientation_in_world(
                        self._torso_quat(self._measured, self._imu_quat))
                    robot_ground_z = self._robot_ground_z(
                        self._measured, self._imu_quat, robot_pos)
                    self._mocap_align_pending = (
                        robot_pos.copy(), robot_quat.copy(), robot_ground_z)
            if left.connected:
                self._gripper_targets[0] = _gripper_from_trigger(
                    left.trigger, self._gripper_open, self._gripper_closed)
            if right.connected:
                self._gripper_targets[1] = _gripper_from_trigger(
                    right.trigger, self._gripper_open, self._gripper_closed)
        if changed:
            self.get_logger().info(
                '双手 squeeze 接合，开始跟踪' if tracking else '双手 squeeze 断开，姿势冻结')

    def _slots(self, names: list[str]) -> np.ndarray:
        try:
            return np.array([self._joints.index(n) for n in names], dtype=np.intp)
        except ValueError as exc:
            raise RuntimeError(f'关节不在 31 轴名单里: {exc}') from exc

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    ##
    # 订阅
    ##

    def _on_joint_states(self, message: JointState) -> None:
        if not self._name_to_index:
            self._name_to_index = {n: i for i, n in enumerate(message.name)}
            missing = [n for n in self._joints if n not in self._name_to_index]
            if missing:
                self._estop(f'/joint_states 缺少关节: {missing[:4]}')
                return
        order = [self._name_to_index[n] for n in self._joints]
        pos = np.asarray(message.position, dtype=np.float64)
        vel = (np.asarray(message.velocity, dtype=np.float64)
               if len(message.velocity) == len(message.position) else np.zeros_like(pos))
        with self._lock:
            self._measured = pos[order]
            self._measured_vel = vel[order]
            self._joint_stamp = self._now()

    def _on_imu(self, message: Imu) -> None:
        q = message.orientation
        try:
            quat = quat_from_xyzw((q.x, q.y, q.z, q.w))
        except ValueError:
            return
        with self._lock:
            self._imu_quat = quat
            self._imu_omega = np.array([message.angular_velocity.x,
                                        message.angular_velocity.y,
                                        message.angular_velocity.z])
            self._imu_stamp = self._now()

    def _on_odom(self, message: Odometry) -> None:
        """``/dog_odom`` 给的是盆骨（robot_center 姿态逐位等于 pelvis），先 FK 到躯干。

        ``twist`` 一律不用：实测静止时 ``twist.linear`` 精确为 0，不是位置的微分。
        """
        with self._lock:
            if self._measured is None:
                return
            waist = self._measured[self._waist_slots].copy()
        pos = message.pose.pose.position
        q = message.pose.pose.orientation
        try:
            quat = quat_from_xyzw((q.x, q.y, q.z, q.w))
        except ValueError:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        torso = torso_pos_from_pelvis(np.array([pos.x, pos.y, pos.z]), quat, waist[0])
        torso_quat = torso_quat_from_pelvis(quat, waist[0], waist[1], waist[2])
        with self._lock:
            self._odom.push_odom(stamp, torso, torso_quat)

    def _on_lidar(self, message: Odometry) -> None:
        """雷达定位直接就是 ``world -> torso_link``，不需要 FK。

        未设原点时 ``covariance[0] = -1``（REP-145 惯例），**必须按帧判**——调用过
        set_origin 之后队列里的残留帧仍然带 -1。
        """
        if message.pose.covariance[0] < 0.0:
            return
        pos = message.pose.pose.position
        q = message.pose.pose.orientation
        try:
            quat = quat_from_xyzw((q.x, q.y, q.z, q.w))
        except ValueError:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        with self._lock:
            self._odom.push_lidar(stamp, np.array([pos.x, pos.y, pos.z]), quat)

    def _on_select(self, message: String) -> None:
        name = message.data.strip()
        if self._motions is None:
            self.get_logger().warn('参考源是实时动捕，没有动作可选')
            return
        with self._lock:
            if self._state is State.RUNNING:
                self.get_logger().warn('RUNNING 中不允许换动作，先 estop')
                return
            try:
                self._clip = self._motions.get(name)
            except KeyError as exc:
                self.get_logger().warn(str(exc))
                return
        self.get_logger().info(f'参考动作切换为 {name} ({self._clip.duration_s:.1f}s)')

    ##
    # 服务
    ##

    def _on_engage(self, _request, response):
        with self._lock:
            if self._state not in (State.IDLE, State.ESTOP):
                response.success, response.message = False, f'当前状态 {self._state.value}'
                return response
        if self._odom.mode != 'odom_only':
            ok, detail = self._set_world_origin()
            if not ok:
                response.success, response.message = False, detail
                return response
        ok, detail = self._switch_controller(activate=True)
        if ok:
            with self._lock:
                self._state = State.IDLE
                self._reason = ''
        response.success, response.message = ok, detail
        return response

    def _set_world_origin(self) -> tuple[bool, str]:
        if not self._set_origin.wait_for_service(timeout_sec=2.0):
            return False, f'定位定原点服务 {self._set_origin.srv_name} 不可用'
        future = self._set_origin.call_async(Trigger.Request())
        if not self._wait(future, 3.0):
            return False, '定位定原点超时'
        result = future.result()
        if result is None or not result.success:
            detail = '' if result is None else result.message
            return False, f'定位定原点失败: {detail or "无响应"}'
        try:
            origin_stamp = _origin_stamp(result.message)
        except ValueError as exc:
            return False, f'定位定原点响应无效: {exc}'
        with self._lock:
            self._odom.reset_lidar_origin(origin_stamp)
        return True, result.message

    def _on_start(self, _request, response):
        with self._lock:
            if self._state is not State.IDLE:
                response.success, response.message = False, f'当前状态 {self._state.value}'
                return response
            if self._measured is None or self._imu_quat is None:
                response.success, response.message = False, '还没收到关节或 IMU 数据'
                return response
            pose = self._odom.torso_position()
            if pose is None:
                response.success, response.message = False, '里程计尚未就绪，无法对齐参考'
                return response
            if self._mocap is not None and not self._mocap.calibrated:
                response.success, response.message = False, \
                    '动捕还没校准：让人站直，按双摇杆或调 /mocap/calibrate'
                return response
            stale = self._stale()
            if stale:
                response.success, response.message = False, stale
                return response
            snapshot = self._snapshot()
            if snapshot is None:
                response.success, response.message = False, '/joint_states 或 IMU 首帧未到'
                return response
            measured, _, pelvis_quat = snapshot
            self._stand_from = measured
            self._stand_start = self._now()
            self._mocap_had_tracking = self._tracking
            self._stand_tracks_mocap = self._mocap_clip is not None and self._tracking
            self._policy.reset()
            # 同时锁偏航与平移。中途重算等于把已产生的跟踪误差抹掉，那 15 维就永远读作零。
            torso_quat = self._torso_quat(measured, pelvis_quat)
            if self._mocap_clip is None:
                self._clip.align(pose, torso_quat)
            else:
                self._clip.align(
                    pose, torso_quat,
                    self._robot_ground_z(measured, pelvis_quat, pose))
            self._state = State.STAND
        response.success = True
        response.message = f'STAND 中，{self._stand_s:.1f}s 后开始放 {self._clip.name}'
        return response

    def _on_estop(self, _request, response):
        response.success = self._estop('人工急停')
        response.message = self._reason
        return response

    def _estop(self, reason: str) -> bool:
        with self._lock:
            if self._state is State.ESTOP:
                return True
            self._state = State.ESTOP
            self._reason = reason
        self.get_logger().error(f'急停: {reason}')
        self._switch_controller(activate=False)
        return True

    def _switch_controller(self, *, activate: bool) -> tuple[bool, str]:
        if not self._switch.wait_for_service(timeout_sec=2.0):
            return False, 'controller_manager/switch_controller 不可用'
        request = SwitchController.Request()
        request.activate_controllers = [self._controller] if activate else []
        request.deactivate_controllers = [] if activate else [self._controller]
        request.strictness = SwitchController.Request.BEST_EFFORT
        future = self._switch.call_async(request)
        # 反激活会在 controller_manager 线程里跑完 2 s 卸力斜坡再加几次电机模式 RPC，
        # 超时给窄了会在切换其实成功的情况下报「超时」。
        if not self._wait(future, 15.0):
            return False, 'switch_controller 超时'
        result = future.result()
        ok = bool(result and result.ok)
        return ok, ('已激活' if activate else '已反激活') if ok else '切换失败'

    def _wait(self, future, timeout: float) -> bool:
        # 只能等，不能在这里 spin：本节点由 MultiThreadedExecutor 托管，
        # rclpy.spin_once(self) 会把节点从那个执行器里摘走，之后定时器、订阅、服务
        # 全部停摆——表现就是 engage 之后 start 再也没有响应。
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        return future.done()

    ##
    # 控制环
    ##

    def _torso_quat(self, measured: np.ndarray, pelvis_quat: np.ndarray) -> np.ndarray:
        waist = measured[self._waist_slots]
        return torso_quat_from_pelvis(pelvis_quat, waist[0], waist[1], waist[2])

    def _robot_ground_z(self, measured: np.ndarray, pelvis_quat: np.ndarray,
                        torso_pos: np.ndarray) -> float:
        """当前姿态 FK 的双踝最低点世界 z；准备阶段该点定义地面。"""
        local = self._mocap_kin.key_body_pos(
            measured[self._action_slots],
            ('torso_link', 'left_ankle_roll_link', 'right_ankle_roll_link'))
        world_pelvis_quat = self._odom.orientation_in_world(pelvis_quat)
        pelvis_pos = np.asarray(torso_pos, dtype=np.float64) \
            - quat_apply(world_pelvis_quat, local[0])
        ankles = pelvis_pos + quat_apply(world_pelvis_quat, local[1:])
        return float(np.min(ankles[:, 2]))

    def _snapshot(self) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        """锁内取关节与 IMU。三者都是首帧到达才赋上，在那之前一律返回 None。"""
        measured, vel, quat = self._measured, self._measured_vel, self._imu_quat
        if measured is None or vel is None or quat is None:
            return None
        return measured.copy(), vel.copy(), quat.copy()

    def _control(self) -> None:
        with self._lock:
            if self._state not in ACTIVE_STATES:
                return
            controllers_timed_out = self._mocap_clip is not None and self._tracking \
                and self._now() - self._controllers_stamp > self._controllers_timeout
            if controllers_timed_out:
                self._tracking = False
            stale = self._stale()
            snapshot = self._snapshot()
            if snapshot is None:
                # _stale() 那边也会报超时，这里只是把「首帧还没到」写进类型。
                stale = stale or '/joint_states 或 IMU 首帧未到'
            else:
                measured, measured_vel, base_quat = snapshot
                omega = self._imu_omega.copy()
                torso_quat = self._odom.orientation_in_world(
                    self._torso_quat(measured, base_quat))
                torso_pos = self._odom.torso_position()
                state = self._state
                stand_from = None if self._stand_from is None else self._stand_from.copy()
                stand_elapsed = self._now() - self._stand_start
                clip = self._clip
                stand_tracks_mocap = self._stand_tracks_mocap
                tracking = self._tracking
                gripper_targets = self._gripper_targets.copy()
        if controllers_timed_out:
            self.get_logger().warning('VR 手柄状态超时，姿势冻结')
        if stale:
            self._estop(stale)
            return
        if torso_pos is None:
            self._estop('里程计无输出')
            return

        tilt = math.acos(min(max(-float(rotate_inverse(torso_quat, GRAVITY_W)[2]), -1.0), 1.0))
        if tilt > self._tilt_limit:
            self._estop(f'躯干倾角 {tilt:.2f} rad 超过 {self._tilt_limit:.2f}')
            return

        targets = np.empty(len(self._joints))
        targets[self._gripper_slots] = gripper_targets

        if state is State.STAND:
            if stand_from is None:
                # 和 State.STAND 是同时赋上的，走到这里就是内部不变式坏了。
                # 不接的话是 TypeError，控制环线程直接死掉——那比急停危险得多。
                self._estop('STAND 缺少插值起点')
                return
            goal = np.empty(len(self._joints))
            goal[self._gripper_slots] = gripper_targets
            goal[self._action_slots] = clip.live_joint_pos() \
                if stand_tracks_mocap else clip.stand_joint_pos()
            alpha = min(stand_elapsed / max(self._stand_s, 1e-3), 1.0)
            targets[:] = stand_from + alpha * (goal - stand_from)
            if alpha >= 1.0:
                # squeeze 在 start 前或 STAND 中已经接合时，把 RUNNING 边界视为
                # 有效接合点。start 的对齐只供固定站姿使用，不能覆盖实时参考的锚。
                if self._mocap_clip is not None and tracking:
                    clip.align(
                        torso_pos, torso_quat,
                        self._robot_ground_z(measured, base_quat, torso_pos))
                with self._lock:
                    if self._mocap_clip is not None and tracking:
                        self._mocap_align_pending = None
                    self._state = State.RUNNING
                self.get_logger().info(f'开始放 {clip.name}')
        else:
            try:
                policy_position, policy_velocity = _policy_joint_state(
                    measured, measured_vel, self._obs_slots,
                    self._policy_uncontrolled_slots,
                    self._policy.spec.default_joint_pos)
                action_targets = self._policy.step(
                    joint_pos=policy_position,
                    joint_vel=policy_velocity,
                    ang_vel=omega,
                    # 两个刚体不能混：投影重力与角速度挂 pelvis（IMU 直给），
                    # key body 局部化挂 anchor=torso（需 FK 和定位世界系 yaw 修正）。
                    base_quat=base_quat,
                    clip=clip,
                    robot_anchor_pos=torso_pos,
                    robot_anchor_quat=torso_quat,
                )
            except (ValueError, RuntimeError) as exc:
                self._estop(f'策略推理失败: {exc}')
                return
            targets[self._action_slots] = action_targets

            # 实时动捕没有终点，只能靠 estop 停。
            if not getattr(clip, 'streaming', False) \
                    and self._policy.frame >= clip.num_frames:
                if self._loop:
                    self._policy.frame = 0
                else:
                    self.get_logger().info(f'{clip.name} 放完，回到 IDLE')
                    with self._lock:
                        self._state = State.IDLE
                        self._stand_from = None

        self._message.data = targets.tolist()
        self._publisher.publish(self._message)

    def _stale(self) -> str:
        now = self._now()
        if self._measured is None or now - self._joint_stamp > self._timeout:
            return '/joint_states 超时'
        if self._imu_quat is None or now - self._imu_stamp > self._timeout:
            return 'IMU 超时'
        if self._mocap_clip is not None:
            # 断流后参考会被钳在最后一帧，机器人保持最后姿势继续站着，看起来毫无异样，
            # 实际上已经完全失去操作。必须当成急停条件。
            # 缓冲里的时间轴是消息的 header.stamp，所以比较基准用 ROS 时钟。
            mocap = self._mocap_clip.stale(now)
            if mocap:
                return mocap
        return self._odom.stale(now) or ''

    def _publish_status(self) -> None:
        with self._lock:
            state, reason, clip = self._state, self._reason, self._clip
            frame = self._policy.frame
            clamped = self._policy.anchor_clamped
            corr_pos, _ = self._odom.correction
            torso_pos = self._odom.torso_position()
            offset = ''
            if torso_pos is not None and clip.aligned:
                ref_pos, _ = clip.anchor_pose_world(frame)
                offset = f' offset={float(np.linalg.norm(ref_pos - torso_pos)):.3f}'
        message = String()
        total = 'live' if getattr(clip, 'streaming', False) else str(clip.num_frames)
        # offset 是参考锚点与机器人躯干的距离，也就是策略读到的漂移量；
        # drift 是里程计自身被雷达修正的累积量。两者一起看才能分清是谁在漂。
        message.data = (
            f'state={state.value} motion={clip.name} frame={frame}/{total}'
            f'{offset} drift={float(np.linalg.norm(corr_pos)):.3f}'
            + (' CLAMPED' if clamped else '')
            + (f' mocap[{self._mocap_clip.describe()}]'
               if self._mocap_clip is not None else '')
            + (f' reason={reason}' if reason else ''))
        self._status_publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = RgmtTrackingNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
