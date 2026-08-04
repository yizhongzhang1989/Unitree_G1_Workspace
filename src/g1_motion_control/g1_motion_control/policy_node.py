#!/usr/bin/env python3
"""整机运动控制：下肢 ONNX 策略 + 上肢 IK，forward_position_controller 之上唯一的一层。

    键盘/VLA --指令--> [ 本节点 50 Hz：ONNX 推理 + 双臂 IK ] --31 轴位置--> FPC(500 Hz,
                                                                     含手臂重力补偿)
                                                                          |
                                                              G1TopicSystem --> /lowcmd

一个定时器算齐 31 轴：下肢 15 轴（12 腿 + 3 腰）由 ONNX 策略给出，上肢 14 轴由双臂
IK 从两个末端位姿解出，2 个夹爪偏心轴直接透传。手臂始终开启，没有开关。

``~/command`` 一个话题按长度分块，各发布者只更新自己那部分，互不干扰：

    长度  2   [左夹爪, 右夹爪]
    长度  4   [vx, vy, wz, h]                      —— teleop_keyboard.py 发的就是它
    长度  7   右臂位姿 [x, y, z, qx, qy, qz, qw]
    长度 14   双臂位姿（先左后右）
    长度 20   全量，布局是 下肢 4 + 左臂 7 + 右臂 7 + 夹爪 2

末端位姿指 ``*_gripper_base`` 相对 ``torso_link``。参考系选 torso_link 而不是 pelvis，
是为了让手臂目标和策略摆腰彻底解耦——腰怎么动手臂都跟着躯干走，IK 链里根本没有腰。

状态机照搬 ``deploy/`` 里官方的 Passive -> FixStand -> RLBase 三段式，这是实机上
唯一被验证过的接管顺序：

    IDLE --engage--> STAND --start--> RUNNING
      ^                 |               |
      +------- estop ---+---------------+

* ``STAND``：激活 FPC，分两段把 31 轴从"当前实测位姿"插值到策略的默认位姿
  （对应官方 ``State_FixStand`` 的 ``ts: [0, 2]`` + ``qs``），插完就停在那儿等人确认。
  策略绝不能从任意位姿冷启动——训练里它只见过默认位姿附近的开局。
  第一段（``stand_clear_s``）**只把 shoulder_roll 往外张**：关节空间直线不避障，
  手臂贴着身体从背后摆回来时夹爪会扫进大腿（实测 ``shoulder_pitch`` 在 +60° 附近
  ``gripper_base`` 与 ``hip_pitch_link`` 间距 0.0 mm）。这一段手臂仍走
  ``passive_targets``，IK 还没接管。
* ``RUNNING``：策略与手臂 IK 同时接管。进入时清零 ``last_action`` 和步态相位，
  等价于官方 ``env->reset()``；手臂目标位姿用当前实测位形正解播种，所以没人发上肢
  指令时手臂就停在接管那一刻的姿态。
* ``ESTOP``：停止发目标并反激活 FPC。反激活会触发 G1TopicSystem 的卸力斜坡——
  kp 在 ``release_ramp_s`` 内降到 0（只剩 kd，阻尼模式），最后一帧 kd 也归零
  （零力矩模式）。

看门狗：状态超时、姿态倾覆（对应官方 ``mdp::bad_orientation``，阈值同为 1.0 rad）、
推理异常、输出非有限值，任一触发即急停。指令超时只把速度归零、保持高度，不急停
——遥控手松手不该让机器人卸力。上肢够不着或求解异常同样不急停，保持上一帧手臂目标
即可：正在平衡的下肢不该被上肢的问题拖下水。
"""

from __future__ import annotations

import json
import math
import threading
import time
from enum import Enum
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import SwitchController
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from g1_motion_control.arm_ik import ArmIK
from g1_motion_control.policy_runtime import (
    LocomotionPolicy,
    load_policy,
    projected_gravity,
    spec_matches,
)

# ``~/command`` 按长度分块。全量 20 值的布局是 下肢 4 + 左臂 7 + 右臂 7 + 夹爪 2，
# 其余长度都是它的连续子块——发多长就只覆写对应那几个字段，别的原样保留。
# _LAYOUT 的键由块大小相加得出（依次为 2 / 4 / 7 / 14 / 20），而不是另拄一份，
# 这样改了块大小也不会出现两张表对不上的情况。
_BLOCK = {'base': 4, 'left': 7, 'right': 7, 'grip': 2}
_LAYOUT = {sum(_BLOCK[name] for name in fields): fields for fields in (
    ('grip',), ('base',), ('right',), ('left', 'right'),
    ('base', 'left', 'right', 'grip'))}


class State(Enum):
    IDLE = 'idle'
    STAND = 'stand'
    RUNNING = 'running'
    ESTOP = 'estop'


ACTIVE_STATES = (State.STAND, State.RUNNING)


def _resolve(reference: str) -> Path:
    """支持 FPC 用的 ``package://`` 写法。"""
    prefix = 'package://'
    if not reference.startswith(prefix):
        return Path(reference).expanduser()
    package, _, relative = reference[len(prefix):].partition('/')
    return Path(get_package_share_directory(package)) / relative


def _tilt(quat) -> float:
    """躯干轴与竖直方向的夹角，四元数非法时按最坏情况返回 pi。"""
    try:
        return math.acos(min(max(-projected_gravity(quat)[2], -1.0), 1.0))
    except ValueError:
        return math.pi


class MotionControlNode(Node):

    def __init__(self) -> None:
        super().__init__('motion_control')
        p = self.declare_parameter

        # -- 关节 --------------------------------------------------------------
        self._joints = list(p('joints', ['']).get_parameter_value().string_array_value)
        if len(self._joints) < 16 or any(not name for name in self._joints):
            raise ValueError('joints 必须按 FPC 的顺序列出全部关节')
        # 下面这几个只在构造期用到（校验 + 算槽位），不是运行期状态，所以不挂到 self 上。
        policy_joints = list(
            p('policy_joints', ['']).get_parameter_value().string_array_value)
        unknown = [name for name in policy_joints if name not in self._joints]
        if not policy_joints or unknown:
            raise ValueError(f'policy_joints 必须是 joints 的子集: {unknown}')
        self._policy_slots = [self._joints.index(name) for name in policy_joints]
        owned = set(self._policy_slots)
        passive_slots = [i for i in range(len(self._joints)) if i not in owned]
        passive_values = np.asarray(
            p('passive_targets', [0.0] * len(passive_slots))
            .get_parameter_value().double_array_value)
        if passive_values.shape != (len(passive_slots),):
            raise ValueError(f'passive_targets 长度必须是 {len(passive_slots)}')

        # -- 上肢 --------------------------------------------------------------
        # 手臂 14 轴走 IK，夹爪偏心轴不进 IK 模型、直接透传。手臂的槽位要等
        # /robot_description 到了、缩减模型建好才能定（顺序以模型为准）。
        self._arm_names = list(
            p('arm_joints', ['']).get_parameter_value().string_array_value)
        grippers = list(
            p('gripper_joints', ['']).get_parameter_value().string_array_value)
        if sorted(self._arm_names + grippers) != sorted(
                self._joints[i] for i in passive_slots):
            raise ValueError('arm_joints + gripper_joints 必须正好是策略之外的全部关节')
        self._gripper_slots = [self._joints.index(name) for name in grippers]
        grip_limits = p('gripper_limits', [0.0, 2.7638]) \
            .get_parameter_value().double_array_value
        if len(grip_limits) != 2:
            raise ValueError('gripper_limits 必须是 2 个数：下界、上界')
        self._grip_lo, self._grip_hi = float(grip_limits[0]), float(grip_limits[1])
        # 限位收紧（长度同 arm_joints，与 URDF 取交集，只收不放）。肘必须挡在伸直
        # 奇异点之前，否则热启动会把解推过去再也回不来——理由见 arm_ik.py 文件头。
        limit_hi = list(p('ik_limit_upper', Parameter.Type.DOUBLE_ARRAY).get_parameter_value().double_array_value)
        if limit_hi and len(limit_hi) != len(self._arm_names):
            raise ValueError(f'ik_limit_upper 长度必须等于 arm_joints ({len(self._arm_names)})')
        joint_limits = {name: (-math.inf, high)
                        for name, high in zip(self._arm_names, limit_hi)} if limit_hi else {}
        self._ik_kwargs = dict(
            tip_frames={
                'left': p('left_tip_frame', 'left_gripper_base')
                .get_parameter_value().string_value,
                'right': p('right_tip_frame', 'right_gripper_base')
                .get_parameter_value().string_value,
            },
            base_frame=p('base_frame', 'torso_link')
            .get_parameter_value().string_value,
            max_iters=p('ik_max_iters', 10).get_parameter_value().integer_value,
            damping=p('ik_damping', 0.05).get_parameter_value().double_value,
            tol_pos=p('ik_tol_pos', 0.001).get_parameter_value().double_value,
            tol_ori=p('ik_tol_ori', 0.0035).get_parameter_value().double_value,
            max_step_pos=p('ik_max_step_pos', 0.1).get_parameter_value().double_value,
            max_step_ori=p('ik_max_step_ori', 0.5).get_parameter_value().double_value,
            joint_limits=joint_limits,
        )
        # 热启动陷进坏解支时的逃生阈值（m）。理由见 config 里那段注释。
        self._rescue_err = float(
            p('ik_rescue_err', 0.01).get_parameter_value().double_value)

        n = len(policy_joints)
        lower = np.asarray(p('target_lower_limits', [0.0] * n)
                           .get_parameter_value().double_array_value)
        upper = np.asarray(p('target_upper_limits', [0.0] * n)
                           .get_parameter_value().double_array_value)
        if lower.shape != (n,) or upper.shape != (n,):
            raise ValueError('target_{lower,upper}_limits 长度必须等于 policy_joints')

        # -- 时序与阈值 --------------------------------------------------------
        rate = float(p('control_rate_hz', 50.0).get_parameter_value().double_value)
        if rate <= 0.0:
            raise ValueError('control_rate_hz 必须为正')
        dt = 1.0 / rate
        # 手臂关节目标的变化率上限（rad/s）。**这是"手腕突然翻 180 度"唯一的根治手段**：
        # 多解支是这台机器人的固有属性（同一末端位姿、不同种子解出的位形最大差 2.67 rad），
        # 求解器跨支时一帧就能跳好几弧度，而上肢没有别的限速。见 config 里那段注释。
        arm_rate = float(p('arm_rate_limit', 10.0).get_parameter_value().double_value)
        if arm_rate <= 0.0:
            raise ValueError('arm_rate_limit 必须为正（rad/s）')
        self._arm_rate = arm_rate * dt
        self._stand_s = float(p('stand_s', 3.0).get_parameter_value().double_value)

        # 先让手张开再到前面
        self._clear_roll = float(p('stand_clear_roll', 0.7).get_parameter_value().double_value)
        self._clear_s = float(p('stand_clear_s', 0.4).get_parameter_value().double_value)
        if self._clear_s > 0.0 and self._clear_s >= self._stand_s:
            raise ValueError('stand_clear_s 必须小于 stand_s（它是总时长里的第一段）')
        # 向外张开的符号：左臂 +、右臂 −，URDF 里 shoulder_roll 的行程就是这么定的
        # （左 [-1.588, 2.252]、右 [-2.252, 1.588]）。没有这两个关节就自然退化成单段。
        self._clear_slots = [
            (index, 1.0 if name.startswith('left') else -1.0)
            for index, name in enumerate(self._joints) if 'shoulder_roll' in name]

        self._state_timeout = float(
            p('state_timeout_s', 0.1).get_parameter_value().double_value)
        self._command_timeout = float(
            p('command_timeout_s', 0.5).get_parameter_value().double_value)
        # 与官方 mdp::bad_orientation 的默认 limit_angle 一致。
        self._tilt_limit = float(
            p('tilt_limit_rad', 1.0).get_parameter_value().double_value)
        self._switch_timeout = float(
            p('switch_timeout_s', 15.0).get_parameter_value().double_value)
        self._controller = p('controller_name', 'forward_position_controller') \
            .get_parameter_value().string_value
        manager = p('controller_manager', '/controller_manager') \
            .get_parameter_value().string_value.rstrip('/')

        # -- 指令限幅与限速 ----------------------------------------------------
        limits = p('command_limits', [-0.3, 0.5, -0.3, 0.3, -0.5, 0.5, 0.62, 0.76]) \
            .get_parameter_value().double_array_value
        if len(limits) != 8:
            raise ValueError('command_limits 必须是 8 个数: vx/vy/wz/h 的上下界')
        self._cmd_lo = np.asarray(limits[0::2], dtype=np.float64)
        self._cmd_hi = np.asarray(limits[1::2], dtype=np.float64)
        accel = float(p('linear_accel_limit', 1.5).get_parameter_value().double_value)
        # 训练时高度指令按 BaseHeightCommandCfg.max_rate 缓变，策略没见过高度阶跃；
        # 实机上必须复现这个限速，否则遥控每按一下都是分布外输入。
        self._cmd_rate = dt * np.array([
            accel, accel,
            float(p('angular_accel_limit', 3.0).get_parameter_value().double_value),
            float(p('height_rate_limit', 0.15).get_parameter_value().double_value),
        ])
        self._initial_height = float(
            p('initial_height', 0.74).get_parameter_value().double_value)

        # -- 策略 --------------------------------------------------------------
        policy_path = _resolve(p('policy_path', '').get_parameter_value().string_value)
        if not policy_path.is_file():
            raise ValueError(f'找不到策略文件: {policy_path}')
        session, spec = load_policy(str(policy_path))
        spec_matches(spec, policy_joints)
        self._policy = LocomotionPolicy(session, spec, control_dt=dt,
                                        target_lower=lower, target_upper=upper)
        # 站立位姿 = 策略的默认位姿 + 被动关节目标，直接取自 ONNX metadata：这和
        # 训练里 reset 后的开局位姿是同一份数，不另抄一遍。
        self._stand_pose = np.empty(len(self._joints))
        self._stand_pose[self._policy_slots] = spec.default_pos
        self._stand_pose[passive_slots] = passive_values
        self.get_logger().info(
            f'策略已加载: {policy_path.name} obs={spec.obs_dim} act={spec.action_dim} '
            f'@ {rate:.0f} Hz')

        # -- 运行期状态（全部在 _lock 下访问）----------------------------------
        self._lock = threading.Lock()
        self._state = State.IDLE
        self._q = np.zeros(len(self._joints))
        self._dq = np.zeros(len(self._joints))
        self._joint_stamp = 0.0
        self._quat = (0.0, 0.0, 0.0, 1.0)
        self._ang_vel = (0.0, 0.0, 0.0)
        self._imu_stamp = 0.0
        self._js_names: list[str] = []
        self._js_index: list[int] = []
        self._request = np.array([0.0, 0.0, 0.0, self._initial_height])
        self._command = self._request.copy()
        self._command_stamp = 0.0
        self._stand_from = np.zeros(len(self._joints))
        self._stand_via = np.zeros(len(self._joints))
        self._stand_start = 0.0
        self._reason = ''
        self._spinning = True
        # 上肢：_pose 是末端位姿指令的缓存（唯一真值），_arm_target 是 IK 解出、
        # 实际发给 FPC 的关节目标，同时充当下一帧的热启动种子。
        self._ik: ArmIK | None = None
        self._arm_slots: list[int] = []
        self._arm_target = np.zeros(len(self._arm_names))
        self._pose: dict[str, np.ndarray] = {}
        self._arm_seed = False
        self._grip = np.zeros(len(self._gripper_slots))
        self._ik_stat = (0.0, 0.0, 0, 0.0)

        # -- ROS 接口 ----------------------------------------------------------
        stream = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.BEST_EFFORT)
        # 上下肢共用 ~/command，两个发布者并存时 depth=1 会让后到的挤掉先到的，
        # 于是下肢和上肢的指令互相吞。这里单独给 4。
        command_qos = QoSProfile(depth=4, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._message = Float64MultiArray()
        self._publisher = self.create_publisher(
            Float64MultiArray,
            p('target_topic', '/forward_position_controller/commands')
            .get_parameter_value().string_value, stream)
        self._status_publisher = self.create_publisher(String, '~/status', 10)

        control = MutuallyExclusiveCallbackGroup()
        # 状态订阅必须自己一组，不能和 50 Hz 的 _control 挤在一个互斥组里。
        # 看门狗量的是「我的回调上次跑在什么时候」，共用一组的话 _control 一旦变慢
        # （比如 IK 不收敛跑满迭代）就会把状态回调饿死，数据明明在到却报超时——
        # 实测过：IK 卡 95 ms，/joint_states 发布侧完好无损（100.0 Hz），照样急停。
        # 分组之后两者在 MultiThreadedExecutor 上并行，时间戳才真的反映数据到达。
        state = MutuallyExclusiveCallbackGroup()
        services = ReentrantCallbackGroup()
        self.create_subscription(JointState, '/joint_states', self._on_joint_states,
                                 stream, callback_group=state)
        self.create_subscription(
            Imu, p('imu_topic', '/pelvis_imu_broadcaster/imu')
            .get_parameter_value().string_value,
            self._on_imu, stream, callback_group=state)
        self.create_subscription(Float64MultiArray, '~/command', self._on_command,
                                 command_qos, callback_group=control)
        # 建 IK 模型要几百毫秒，必须避开控制环所在的互斥组。
        self.create_subscription(
            String, p('robot_description_topic', '/robot_description')
            .get_parameter_value().string_value,
            self._on_description, latched, callback_group=services)
        self.create_timer(dt, self._control, callback_group=control)
        self.create_timer(0.1, self._publish_status, callback_group=control)

        self._switch = self.create_client(
            SwitchController, f'{manager}/switch_controller', callback_group=services)
        self.create_service(Trigger, '~/engage', self._on_engage, callback_group=services)
        self.create_service(Trigger, '~/start', self._on_start, callback_group=services)
        self.create_service(Trigger, '~/estop', self._on_estop, callback_group=services)
        self.get_logger().info('待命。~/engage 站立，~/start 启动策略，~/estop 急停。')

    # -- 状态回调 --------------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_joint_states(self, message: JointState) -> None:
        names = list(message.name)
        if names != self._js_names:
            if any(name not in names for name in self._joints):
                return  # 广播还没收齐，等下一帧。
            self._js_names = names
            self._js_index = [names.index(name) for name in self._joints]
        if len(message.position) != len(names):
            return
        position = np.asarray(message.position)
        velocity = (np.asarray(message.velocity)
                    if len(message.velocity) == len(names) else None)
        with self._lock:
            # 一律存成 FPC 顺序的 31 轴，策略的 15 轴用 _policy_slots 现取（花式索引
            # 本就会拷贝）。另存一份 15 轴等于多一张会和它跑掉的表。
            self._q = position[self._js_index]
            if velocity is not None:
                self._dq = velocity[self._js_index]
            self._joint_stamp = self._now()

    def _on_imu(self, message: Imu) -> None:
        q, w = message.orientation, message.angular_velocity
        with self._lock:
            # G1TopicSystem 已经把 Unitree 的 (w,x,y,z) 转成 ROS 的 (x,y,z,w)。
            self._quat = (q.x, q.y, q.z, q.w)
            self._ang_vel = (w.x, w.y, w.z)
            self._imu_stamp = self._now()

    def _on_command(self, message: Float64MultiArray) -> None:
        """按长度分块，只覆写本次带来的字段，其余保持——上下肢发布者因此互不干扰。"""
        data = np.array(message.data, dtype=np.float64)
        fields = _LAYOUT.get(data.size)
        if fields is None or not np.all(np.isfinite(data)):
            self.get_logger().warning(
                f'丢弃非法指令：长度 {data.size} 不在 {sorted(_LAYOUT)} 里，或含非有限值')
            return
        chunk, offset = {}, 0
        for name in fields:
            chunk[name] = data[offset:offset + _BLOCK[name]]
            offset += _BLOCK[name]
        for name in ('left', 'right'):
            pose = chunk.get(name)
            if pose is None:
                continue
            norm = float(np.linalg.norm(pose[3:]))
            if not 0.5 < norm < 2.0:
                self.get_logger().warning(f'丢弃指令：{name} 四元数模长 {norm:.3f} 异常')
                return
            pose[3:] /= norm
        with self._lock:
            if 'base' in chunk:
                self._request = np.clip(chunk['base'], self._cmd_lo, self._cmd_hi)
                self._command_stamp = self._now()
            for name in ('left', 'right'):
                if name in chunk:
                    self._pose[name] = chunk[name]
            if 'grip' in chunk:
                self._grip = np.clip(chunk['grip'], self._grip_lo, self._grip_hi)

    def _on_description(self, message: String) -> None:
        """/robot_description 是 latched 的，正常只会进来一次。"""
        if self._ik is not None:
            return
        try:
            ik = ArmIK(message.data, self._arm_names, **self._ik_kwargs)  # type: ignore[arg-type]
            slots = [self._joints.index(name) for name in ik.joint_names]
        except Exception as error:  # 建模失败不能拖垮节点，但 ~/start 会拒绝。
            self.get_logger().error(f'手臂 IK 建模失败，上肢不可用: {error}')
            return
        with self._lock:
            self._ik, self._arm_slots = ik, slots
        tightened = [f'{name}≤{ik.upper[i]:.3f}' for i, name in enumerate(ik.joint_names)
                     if ik.upper[i] < ik.model.upperPositionLimit[i] - 1e-9]
        self.get_logger().info(
            f'手臂 IK 就绪（{ik.model.nq} 轴，限位取自 URDF'
            + (f'，已收紧：{", ".join(tightened)}）' if tightened else '）')
            + f'，关节限速 {self._arm_rate / 0.02:.1f} rad/s')

    # -- 服务 ------------------------------------------------------------------

    def _on_engage(self, _request, response):
        """IDLE -> STAND：激活 FPC，插值到默认位姿。"""
        with self._lock:
            if self._state in ACTIVE_STATES:
                response.success, response.message = False, f'当前是 {self._state.value}'
                return response
            stale = self._stale()
            start, quat = self._q.copy(), self._quat
        if stale:
            response.success, response.message = False, stale
            return response
        tilt = _tilt(quat)
        if tilt > self._tilt_limit:
            response.success = False
            response.message = f'姿态倾斜 {math.degrees(tilt):.0f}° 超限，拒绝使能'
            return response

        ok, detail = self._switch_controller(activate=True)
        if not ok:
            response.success, response.message = False, detail
            return response
        with self._lock:
            self._stand_from = start
            self._stand_via = self._clearance_pose(start)
            self._stand_start = self._now()
            self._reason = ''
            self._state = State.STAND
        self.get_logger().info(f'站立中，{self._stand_s:.1f} s 后可以 ~/start')
        response.success = True
        response.message = f'standing ({self._stand_s:.1f}s)'
        return response

    def _clearance_pose(self, start: np.ndarray) -> np.ndarray:
        """中转位姿：只把 shoulder_roll 往外张，其余关节原地不动

        只张不收：手臂本来就比 ``stand_clear_roll`` 更开时不能把它合回去，否则反而把它送进碰撞带。
        """
        via = start.copy()
        if self._clear_s <= 0.0:
            return via              # 关掉侧开：退化成原来的单段直插。
        for index, sign in self._clear_slots:
            via[index] = sign * max(sign * start[index], self._clear_roll)
        return via

    def _on_start(self, _request, response):
        """STAND -> RUNNING：策略接管，等价于官方的 env->reset() + 启动策略线程。"""
        with self._lock:
            if self._state is State.RUNNING:
                response.success, response.message = True, '已经在跑'
                return response
            if self._state is not State.STAND:
                response.success, response.message = False, '要先 ~/engage 站立'
                return response
            if self._now() - self._stand_start < self._stand_s:
                response.success, response.message = False, '站立插值还没走完'
                return response
            if self._ik is None:
                response.success = False
                response.message = '手臂 IK 未就绪，在等 /robot_description'
                return response
            self._policy.reset()
            self._request = np.array([0.0, 0.0, 0.0, self._initial_height])
            self._command = self._request.copy()
            self._command_stamp = self._now()
            # 手臂从当前实测位形接管；目标位姿的正解播种放到控制线程里做，
            # 这样 ArmIK 内部的 pinocchio data 缓存只被一个线程碰，不用额外加锁。
            self._arm_target = self._q[self._arm_slots].copy()
            self._grip = self._q[self._gripper_slots].copy()
            self._pose, self._arm_seed = {}, True
            self._state = State.RUNNING
        self.get_logger().info('策略接管')
        response.success, response.message = True, 'running'
        return response

    def _on_estop(self, _request, response):
        response.success = True
        response.message = ('已卸力（阻尼 -> 零力矩）' if self._estop('操作员急停')
                            else '本来就没在运行')
        return response

    def _estop(self, reason: str) -> bool:
        """置急停并卸力。并发触发时只有第一个真正执行切换。"""
        with self._lock:
            if self._state is State.ESTOP:
                return False
            was_active = self._state in ACTIVE_STATES
            self._state = State.ESTOP
            self._reason = reason
        self.get_logger().error(f'急停: {reason}')
        if was_active:
            ok, detail = self._switch_controller(activate=False)
            if not ok:
                self.get_logger().error(f'卸力失败，立刻用手柄断电: {detail}')
        return was_active

    def _switch_controller(self, *, activate: bool) -> tuple[bool, str]:
        if not self._switch.wait_for_service(timeout_sec=3.0):
            return False, 'controller_manager/switch_controller 不可用'
        request = SwitchController.Request()
        request.activate_controllers = [self._controller] if activate else []
        request.deactivate_controllers = [] if activate else [self._controller]
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        # 反激活会在 controller_manager 线程里跑完整个卸力斜坡，超时给得很宽。
        future = self._switch.call_async(request)
        if not self._wait(future, self._switch_timeout):
            return False, 'switch_controller 超时'
        result = future.result()
        if result is None or not result.ok:
            return False, f'switch_controller 拒绝{"激活" if activate else "反激活"}'
        return True, 'ok'

    def _wait(self, future, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            if self._spinning:
                time.sleep(0.005)
            elif self.executor is not None:
                # 退出路径上执行器已经不转了，自己转一下才能收到响应。
                self.executor.spin_once(timeout_sec=0.02)
            else:
                break
        return future.done()

    # -- 控制环 ----------------------------------------------------------------

    def _control(self) -> None:
        with self._lock:
            state = self._state
            if state not in ACTIVE_STATES:
                return
            stale = self._stale()
            quat = self._quat
            if state is State.STAND:
                # 两段：先把手臂侧开（腿不动），再全身走向站立位姿。理由见
                # stand_clear_roll 旁边那段注释。
                elapsed = self._now() - self._stand_start
                if elapsed < self._clear_s:
                    alpha = elapsed / self._clear_s
                    origin, goal = self._stand_from, self._stand_via
                else:
                    span = self._stand_s - self._clear_s
                    alpha = (1.0 if span <= 0.0 else
                             min((elapsed - self._clear_s) / span, 1.0))
                    origin, goal = self._stand_via, self._stand_pose
                target = origin + alpha * (goal - origin)
            else:
                joint_pos = self._q[self._policy_slots]
                joint_vel = self._dq[self._policy_slots]
                ang_vel = self._ang_vel
                request = self._request.copy()
                if self._now() - self._command_stamp > self._command_timeout:
                    request[:3] = 0.0  # 松手只停车，不卸力。
                self._command = np.clip(
                    self._command + np.clip(request - self._command, -self._cmd_rate, self._cmd_rate),
                    self._cmd_lo, self._cmd_hi)
                command = self._command.copy()
                if self._arm_seed:
                    # 没人发上肢指令时就停在接管那一刻的姿态；setdefault 让
                    # ~/start 与首帧之间已经收到的指令优先。
                    for side, pose in self._ik.fk(self._arm_target).items():
                        self._pose.setdefault(side, pose)
                    self._arm_seed = False
                poses, grip = dict(self._pose), self._grip.copy()

        if stale:
            self._estop(stale)
            return
        tilt = _tilt(quat)
        if tilt > self._tilt_limit:
            self._estop(f'姿态倾斜 {math.degrees(tilt):.0f}° 超限')
            return

        if state is State.RUNNING:
            try:
                target = np.empty(len(self._joints))
                target[self._policy_slots] = self._policy.step(
                    joint_pos=joint_pos, joint_vel=joint_vel, ang_vel=ang_vel,
                    quat_xyzw=quat, command=command)
            except Exception as error:  # 推理链路任何异常都当失效处理。
                self._estop(f'推理失败: {error}')
                return
            # IK 本身不抛异常（够不着就返回尽力而为的解），这个 try 是最后一道保险：
            # 上肢出什么事都只保持上一帧手臂目标，绝不能把正在平衡的下肢一起急停。
            try:
                clock = time.monotonic()
                # 热启动：种子就是上一帧已发布的目标，解天然连续。跨解支时仍可能跳，
                # 所以出口按 arm_rate_limit 限速——这才是防"手腕突然翻 180 度"的那一道。
                solved, pos_err, ori_err, iters = self._ik.solve(self._arm_target, poses)
                # 逃生：热启动会把解一路推进回不来的解支（实测手臂翻到肩后、
                # shoulder_roll 顶死限位，之后连原位都够不着）。残差大到不像"只是够不着"
                # 时，拿站立位形当种子再解一次——那个种子从不落进陷阱。只有明显更好才采纳，
                # 所以正常跟随根本不会触发（实测 ±3cm 轨迹 0.00%），代价也只有那一次求解。
                if pos_err > self._rescue_err:
                    alt, alt_pos, alt_ori, alt_iters = self._ik.solve(
                        self._stand_pose[self._arm_slots], poses)
                    if alt_pos < pos_err - self._rescue_err:
                        solved, pos_err, ori_err = alt, alt_pos, alt_ori
                        iters += alt_iters
                self._arm_target = self._arm_target + np.clip(
                    solved - self._arm_target, -self._arm_rate, self._arm_rate)
                self._ik_stat = (pos_err, ori_err, iters, (time.monotonic() - clock) * 1e3)
            except Exception as error:
                self.get_logger().warning(f'手臂 IK 异常，保持上一帧: {error}',
                                          throttle_duration_sec=1.0)
            target[self._arm_slots] = self._arm_target
            target[self._gripper_slots] = grip
        if not np.all(np.isfinite(target)):
            self._estop('输出出现非有限值')
            return

        self._message.data = target.tolist()
        self._publisher.publish(self._message)

    def _stale(self) -> str:
        now = self._now()
        if now - self._joint_stamp > self._state_timeout:
            return f'/joint_states 超时 {now - self._joint_stamp:.2f} s'
        if now - self._imu_stamp > self._state_timeout:
            return f'IMU 超时 {now - self._imu_stamp:.2f} s'
        return ''

    def _publish_status(self) -> None:
        with self._lock:
            ready = (self._state is State.STAND
                     and self._now() - self._stand_start >= self._stand_s)
            pos_err, ori_err, iters, solve_ms = self._ik_stat
            payload = {
                'state': self._state.value,
                'ready_to_start': ready,
                'command': [round(float(v), 3) for v in self._command],
                'request': [round(float(v), 3) for v in self._request],
                'reason': self._reason,
                'stale': self._stale(),
                'ik_ready': self._ik is not None,
                'ik_pos_err': round(pos_err, 4),
                'ik_ori_err': round(ori_err, 4),
                'ik_iters': iters,
                'ik_ms': round(solve_ms, 2),
                'grip': [round(float(v), 3) for v in self._grip],
                # 当前生效的末端位姿目标。遥操侧接管时直接拿它当原点，就不必自己
                # 再建一份 IK 模型算正解——两边的起点由构造保证完全一致。
                'pose': {side: [round(float(v), 5) for v in pose]
                         for side, pose in self._pose.items()},
            }
        self._status_publisher.publish(String(data=json.dumps(payload)))

    def shutdown(self) -> None:
        """退出时不能把机器人晾在最后一帧目标上，必须走一遍卸力。"""
        self._spinning = False
        with self._lock:
            active = self._state in ACTIVE_STATES
        if active:
            self.get_logger().warning('节点退出，正在卸力')
            self._estop('节点退出')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionControlNode()
    # 多线程：使能/急停里的 switch_controller 会阻塞数秒（硬件卸力斜坡），单线程
    # 执行器会连带把控制环和状态订阅一起冻住。
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
