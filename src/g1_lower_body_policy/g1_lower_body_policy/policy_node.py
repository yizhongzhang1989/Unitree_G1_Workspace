#!/usr/bin/env python3
"""下肢 ONNX 策略层：forward_position_controller 之上的一层。

    键盘/VLA --(vx, vy, w, h)--> [ 本节点 50 Hz ONNX ] --31 轴位置--> FPC(500 Hz,
                                                                    含手臂重力补偿)
                                                                        |
                                                            G1TopicSystem --> /lowcmd

本节点只负责下肢 15 轴（12 腿 + 3 腰）；上肢 14 轴和 2 个夹爪偏心轴写固定目标
（默认 0），由 FPC 的重力补偿托住。策略的职责是在上肢被别人（VLA）随意摆布时
保持平衡并跟随速度/高度指令。

状态机照搬 ``deploy/`` 里官方的 Passive -> FixStand -> RLBase 三段式，这是实机上
唯一被验证过的接管顺序：

    IDLE --engage--> STAND --start--> RUNNING
      ^                 |               |
      +------- estop ---+---------------+

* ``STAND``：激活 FPC，用 ``stand_s`` 秒把 31 轴从"当前实测位姿"线性插值到策略的
  默认位姿（对应官方 ``State_FixStand`` 的 ``ts: [0, 2]`` + ``qs``），插完就停在
  那儿等人确认。策略绝不能从任意位姿冷启动——训练里它只见过默认位姿附近的开局。
* ``RUNNING``：策略接管。进入时清零 ``last_action`` 和步态相位，等价于官方
  ``env->reset()``。
* ``ESTOP``：停止发目标并反激活 FPC。反激活会触发 G1TopicSystem 的卸力斜坡——
  kp 在 ``release_ramp_s`` 内降到 0（只剩 kd，阻尼模式），最后一帧 kd 也归零
  （零力矩模式）。

看门狗：状态超时、姿态倾覆（对应官方 ``mdp::bad_orientation``，阈值同为 1.0 rad）、
推理异常、输出非有限值，任一触发即急停。指令超时只把速度归零、保持高度，不急停
——遥控手松手不该让机器人卸力。
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
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from g1_lower_body_policy.policy_runtime import (
    LowerBodyPolicy,
    load_policy,
    projected_gravity,
    spec_matches,
)


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


class LowerBodyPolicyNode(Node):

    def __init__(self) -> None:
        super().__init__('lower_body_policy')
        p = self.declare_parameter

        # -- 关节 --------------------------------------------------------------
        self._joints = list(p('joints', ['']).get_parameter_value().string_array_value)
        if len(self._joints) < 16 or any(not name for name in self._joints):
            raise ValueError('joints 必须按 FPC 的顺序列出全部关节')
        self._policy_joints = list(
            p('policy_joints', ['']).get_parameter_value().string_array_value)
        unknown = [name for name in self._policy_joints if name not in self._joints]
        if not self._policy_joints or unknown:
            raise ValueError(f'policy_joints 必须是 joints 的子集: {unknown}')
        self._policy_slots = [self._joints.index(name) for name in self._policy_joints]
        owned = set(self._policy_slots)
        self._passive_slots = [i for i in range(len(self._joints)) if i not in owned]
        self._passive_values = np.asarray(
            p('passive_targets', [0.0] * len(self._passive_slots))
            .get_parameter_value().double_array_value)
        if self._passive_values.shape != (len(self._passive_slots),):
            raise ValueError(f'passive_targets 长度必须是 {len(self._passive_slots)}')

        n = len(self._policy_joints)
        lower = np.asarray(p('target_lower_limits', [0.0] * n)
                           .get_parameter_value().double_array_value)
        upper = np.asarray(p('target_upper_limits', [0.0] * n)
                           .get_parameter_value().double_array_value)
        if lower.shape != (n,) or upper.shape != (n,):
            raise ValueError('target_{lower,upper}_limits 长度必须等于 policy_joints')

        # -- 时序与阈值 --------------------------------------------------------
        self._rate = float(p('control_rate_hz', 50.0).get_parameter_value().double_value)
        if self._rate <= 0.0:
            raise ValueError('control_rate_hz 必须为正')
        self._dt = 1.0 / self._rate
        self._stand_s = float(p('stand_s', 3.0).get_parameter_value().double_value)
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
        self._cmd_rate = self._dt * np.array([
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
        spec_matches(spec, self._policy_joints)
        self._policy = LowerBodyPolicy(session, spec, control_dt=self._dt,
                                       target_lower=lower, target_upper=upper)
        # 站立位姿 = 策略的默认位姿 + 被动关节目标，直接取自 ONNX metadata：这和
        # 训练里 reset 后的开局位姿是同一份数，不另抄一遍。
        self._stand_pose = np.empty(len(self._joints))
        self._stand_pose[self._policy_slots] = spec.default_pos
        self._stand_pose[self._passive_slots] = self._passive_values
        self.get_logger().info(
            f'策略已加载: {policy_path.name} obs={spec.obs_dim} act={spec.action_dim} '
            f'@ {self._rate:.0f} Hz')

        # -- 运行期状态（全部在 _lock 下访问）----------------------------------
        self._lock = threading.Lock()
        self._state = State.IDLE
        self._q = np.zeros(len(self._joints))
        self._q_policy = np.zeros(n)
        self._dq_policy = np.zeros(n)
        self._joint_stamp = 0.0
        self._quat = (0.0, 0.0, 0.0, 1.0)
        self._ang_vel = (0.0, 0.0, 0.0)
        self._imu_stamp = 0.0
        self._js_names: list[str] = []
        self._js_index: list[int] = []
        self._js_policy_index: list[int] = []
        self._request = np.array([0.0, 0.0, 0.0, self._initial_height])
        self._command = self._request.copy()
        self._command_stamp = 0.0
        self._stand_from = np.zeros(len(self._joints))
        self._stand_start = 0.0
        self._reason = ''
        self._spinning = True

        # -- ROS 接口 ----------------------------------------------------------
        stream = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.BEST_EFFORT)
        self._message = Float64MultiArray()
        self._publisher = self.create_publisher(
            Float64MultiArray,
            p('target_topic', '/forward_position_controller/commands')
            .get_parameter_value().string_value, stream)
        self._status_publisher = self.create_publisher(String, '~/status', 10)

        control = MutuallyExclusiveCallbackGroup()
        services = ReentrantCallbackGroup()
        self.create_subscription(JointState, '/joint_states', self._on_joint_states,
                                 stream, callback_group=control)
        self.create_subscription(
            Imu, p('imu_topic', '/pelvis_imu_broadcaster/imu')
            .get_parameter_value().string_value,
            self._on_imu, stream, callback_group=control)
        self.create_subscription(Float64MultiArray, '~/command', self._on_command,
                                 stream, callback_group=control)
        self.create_timer(self._dt, self._control, callback_group=control)
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
            self._js_policy_index = [names.index(name) for name in self._policy_joints]
        if len(message.position) != len(names):
            return
        position = np.asarray(message.position)
        velocity = (np.asarray(message.velocity)
                    if len(message.velocity) == len(names) else None)
        with self._lock:
            self._q = position[self._js_index]
            self._q_policy = position[self._js_policy_index]
            if velocity is not None:
                self._dq_policy = velocity[self._js_policy_index]
            self._joint_stamp = self._now()

    def _on_imu(self, message: Imu) -> None:
        q, w = message.orientation, message.angular_velocity
        with self._lock:
            # G1TopicSystem 已经把 Unitree 的 (w,x,y,z) 转成 ROS 的 (x,y,z,w)。
            self._quat = (q.x, q.y, q.z, q.w)
            self._ang_vel = (w.x, w.y, w.z)
            self._imu_stamp = self._now()

    def _on_command(self, message: Float64MultiArray) -> None:
        data = np.asarray(message.data, dtype=np.float64)
        if data.shape != (4,) or not np.all(np.isfinite(data)):
            self.get_logger().warning('丢弃非法指令，需要 4 个有限值 [vx, vy, wz, h]')
            return
        with self._lock:
            self._request = np.clip(data, self._cmd_lo, self._cmd_hi)
            self._command_stamp = self._now()

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
            self._stand_start = self._now()
            self._reason = ''
            self._state = State.STAND
        self.get_logger().info(f'站立中，{self._stand_s:.1f} s 后可以 ~/start')
        response.success = True
        response.message = f'standing ({self._stand_s:.1f}s)'
        return response

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
            self._policy.reset()
            self._request = np.array([0.0, 0.0, 0.0, self._initial_height])
            self._command = self._request.copy()
            self._command_stamp = self._now()
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
                alpha = (1.0 if self._stand_s <= 0.0 else
                         min((self._now() - self._stand_start) / self._stand_s, 1.0))
                target = self._stand_from + alpha * (self._stand_pose - self._stand_from)
            else:
                joint_pos, joint_vel = self._q_policy.copy(), self._dq_policy.copy()
                ang_vel = self._ang_vel
                request = self._request.copy()
                if self._now() - self._command_stamp > self._command_timeout:
                    request[:3] = 0.0  # 松手只停车，不卸力。
                self._command = np.clip(
                    self._command + np.clip(request - self._command,
                                            -self._cmd_rate, self._cmd_rate),
                    self._cmd_lo, self._cmd_hi)
                command = self._command.copy()

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
                target[self._passive_slots] = self._passive_values
            except Exception as error:  # 推理链路任何异常都当失效处理。
                self._estop(f'推理失败: {error}')
                return
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
            payload = {
                'state': self._state.value,
                'ready_to_start': ready,
                'command': [round(float(v), 3) for v in self._command],
                'request': [round(float(v), 3) for v in self._request],
                'reason': self._reason,
                'stale': self._stale(),
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
    node = LowerBodyPolicyNode()
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
