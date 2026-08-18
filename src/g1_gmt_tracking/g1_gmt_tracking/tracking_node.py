"""G1 全身动作跟踪节点：读一段参考动作，输出 31 轴关节位置目标。

和同工作区的 ``g1_motion_control`` 是**并列**的一层，不要同时启动——两者都往
``/forward_position_controller/commands`` 写，同时跑就是两个策略抢同一组电机。

分工：
* 29 轴（12 腿 + 3 腰 + 14 臂）由 GMT 策略驱动；
* 2 个夹爪偏心轴透传 ``gripper_targets``，策略不管它们，但它们的**实测角要进观测**。

状态机与 ``g1_motion_control`` 一致，便于同一套操作流程：

    IDLE --engage--> (激活控制器) --start--> STAND --(插值到位)--> RUNNING
                                                 任何时候 estop --> ESTOP

STAND 阶段把 31 轴从实测位形插值到参考动作第 0 帧的位形；插值完才开始放动作，
所以不会一上来就跳变。
"""

from __future__ import annotations

import math
import threading
import time
from enum import Enum
from pathlib import Path

import numpy as np
import rclpy
from controller_manager_msgs.srv import SwitchController
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from g1_gmt_tracking.gmt_runtime import (
    GmtPolicy,
    load_policy,
    resolve_policy_path,
    spec_matches,
)
from g1_gmt_tracking.motion_library import MotionClip, resolve_indices
from g1_gmt_tracking.rotations import quat_from_xyzw, quat_to_mat, torso_quat_from_pelvis

WAIST_JOINTS = ('waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint')


class State(Enum):
    IDLE = 'idle'
    STAND = 'stand'
    RUNNING = 'running'
    ESTOP = 'estop'


ACTIVE_STATES = (State.STAND, State.RUNNING)


class GmtTrackingNode(Node):

    def __init__(self) -> None:
        super().__init__('gmt_tracking')
        p = self.declare_parameter

        joints = self._names('joints')
        if not joints:
            raise RuntimeError('参数 joints 为空：应由 launch 从 FPC 的配置注入 31 轴顺序')
        self._joints = joints

        obs_joints = self._names('obs_joints')
        action_joints = self._names('action_joints')
        gripper_joints = self._names('gripper_joints')

        session, spec = load_policy(resolve_policy_path(self._text('policy_path')))
        spec_matches(spec, obs_joints, action_joints)

        lower = np.asarray(p('target_lower_limits', Parameter.Type.DOUBLE_ARRAY)
                           .get_parameter_value().double_array_value, dtype=np.float64)
        upper = np.asarray(p('target_upper_limits', Parameter.Type.DOUBLE_ARRAY)
                           .get_parameter_value().double_array_value, dtype=np.float64)
        self._policy = GmtPolicy(session, spec, target_lower=lower, target_upper=upper)

        self._rate = self._number('control_rate_hz', 50.0)
        if abs(1.0 / self._rate - spec.control_dt) > 1e-6:
            raise RuntimeError(
                f'控制周期与训练不一致: 本节点 {1.0 / self._rate:.4f}s, '
                f'ONNX 契约 {spec.control_dt:.4f}s')

        # 参考动作：目录下所有 NPZ 一起加载，靠 ~/select_motion 切换。
        motion_dir = resolve_policy_path(self._text('motion_dir'))
        idx = resolve_indices(spec.all_body_names, spec.anchor_body_name,
                              spec.root_body_name, spec.obs_joint_names,
                              spec.action_joint_names, spec.control_dt)
        files = sorted(Path(motion_dir).glob('*.npz'))
        if not files:
            raise RuntimeError(f'参考动作目录里没有 NPZ: {motion_dir}')
        self._clips = {f.stem: MotionClip(f, **idx) for f in files}
        default_motion = self._text('motion', files[0].stem)
        if default_motion not in self._clips:
            raise RuntimeError(f'找不到参考动作 {default_motion}，可选: {sorted(self._clips)}')
        self._clip = self._clips[default_motion]
        self._loop = bool(p('loop', False).get_parameter_value().bool_value)

        self._stand_s = self._number('stand_s', 2.5)
        self._timeout = self._number('state_timeout_s', 0.2)
        self._tilt_limit = self._number('tilt_limit_rad', 0.8)
        self._gripper_targets = np.asarray(
            p('gripper_targets', [0.0, 0.0]).get_parameter_value().double_array_value,
            dtype=np.float64)

        # 31 轴里各名单的槽位。名字对不上直接抛，不静默跳过。
        self._action_slots = self._slots(action_joints)
        self._gripper_slots = self._slots(gripper_joints)
        self._obs_slots = self._slots(obs_joints)
        self._waist_slots = self._slots(list(WAIST_JOINTS))
        if len(action_joints) + len(gripper_joints) != len(joints):
            raise RuntimeError(
                f'动作 {len(action_joints)} + 夹爪 {len(gripper_joints)} '
                f'必须正好盖满 {len(joints)} 轴')

        self._lock = threading.Lock()
        self._state = State.IDLE
        self._reason = ''
        self._measured: np.ndarray | None = None
        self._measured_vel: np.ndarray | None = None
        self._joint_stamp = 0.0
        self._imu_quat: np.ndarray | None = None
        self._imu_omega = np.zeros(3)
        self._imu_stamp = 0.0
        self._stand_from: np.ndarray | None = None
        self._stand_start = 0.0
        self._name_to_index: dict[str, int] = {}

        stream = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.RELIABLE)
        self._message = Float64MultiArray()
        self._publisher = self.create_publisher(
            Float64MultiArray,
            self._text('target_topic', '/forward_position_controller/commands'), stream)
        self._status_publisher = self.create_publisher(String, '~/status', 10)

        control = MutuallyExclusiveCallbackGroup()
        # 状态订阅单独一组：和 50 Hz 控制环挤在一个互斥组里，控制环一慢就会把状态
        # 回调饿死，看门狗读到的是「我上次跑的时间」而不是数据到达时间，于是误急停。
        state = MutuallyExclusiveCallbackGroup()
        services = ReentrantCallbackGroup()
        self.create_subscription(JointState, '/joint_states', self._on_joint_states,
                                 stream, callback_group=state)
        self.create_subscription(
            Imu, self._text('imu_topic', '/pelvis_imu_broadcaster/imu'),
            self._on_imu, stream, callback_group=state)
        self.create_subscription(String, '~/select_motion', self._on_select,
                                 10, callback_group=services)
        self.create_timer(1.0 / self._rate, self._control, callback_group=control)
        self.create_timer(0.1, self._publish_status, callback_group=control)

        manager = self._text('controller_manager', '/controller_manager')
        self._controller = self._text('controller_name', 'forward_position_controller')
        self._switch = self.create_client(
            SwitchController, f'{manager}/switch_controller', callback_group=services)
        self.create_service(Trigger, '~/engage', self._on_engage, callback_group=services)
        self.create_service(Trigger, '~/start', self._on_start, callback_group=services)
        self.create_service(Trigger, '~/estop', self._on_estop, callback_group=services)

        self.get_logger().info(
            f'GMT 跟踪就绪: 观测 {spec.obs_dim} -> 动作 {spec.action_dim}; '
            f'参考动作 {len(self._clips)} 段, 当前 {self._clip.name} '
            f'({self._clip.duration_s:.1f}s)')

    def _names(self, key: str) -> list[str]:
        """名单类参数一律显式声明类型：默认值给 ``[]`` 会被 rclpy 推断成 BYTE_ARRAY，
        配置里注入字符串数组时直接抛 InvalidParameterTypeException。
        """
        return list(self.declare_parameter(key, Parameter.Type.STRING_ARRAY)
                    .get_parameter_value().string_array_value)

    def _text(self, key: str, default: str = '') -> str:
        return self.declare_parameter(key, default).get_parameter_value().string_value

    def _number(self, key: str, default: float) -> float:
        return float(self.declare_parameter(key, default)
                     .get_parameter_value().double_value)

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
               if len(message.velocity) == len(message.position)
               else np.zeros_like(pos))
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

    def _on_select(self, message: String) -> None:
        name = message.data.strip()
        if name not in self._clips:
            self.get_logger().warn(f'未知参考动作 {name}，可选: {sorted(self._clips)}')
            return
        with self._lock:
            if self._state is State.RUNNING:
                self.get_logger().warn('RUNNING 中不允许换动作，先 estop')
                return
            self._clip = self._clips[name]
        self.get_logger().info(f'参考动作切换为 {name} ({self._clip.duration_s:.1f}s)')

    ##
    # 服务
    ##

    def _on_engage(self, _request, response):
        with self._lock:
            if self._state not in (State.IDLE, State.ESTOP):
                response.success, response.message = False, f'当前状态 {self._state.value}'
                return response
        ok, detail = self._switch_controller(activate=True)
        if ok:
            with self._lock:
                self._state = State.IDLE
                self._reason = ''
        response.success, response.message = ok, detail
        return response

    def _on_start(self, _request, response):
        with self._lock:
            if self._state is not State.IDLE:
                response.success, response.message = False, f'当前状态 {self._state.value}'
                return response
            if self._measured is None or self._imu_quat is None:
                response.success, response.message = False, '还没收到关节或 IMU 数据'
                return response
            self._stand_from = self._measured.copy()
            self._stand_start = self._now()
            self._policy.reset()
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
        # 超时给窄了会在切换其实成功的情况下报「超时」。与 g1_motion_control 同值。
        if not self._wait(future, 15.0):
            return False, 'switch_controller 超时'
        ok = bool(future.result() and future.result().ok)
        return ok, ('已激活' if activate else '已反激活') if ok else '切换失败'

    def _wait(self, future, timeout: float) -> bool:
        # 只能等，不能在这里 spin：本节点由 MultiThreadedExecutor 托管，
        # rclpy.spin_once(self) 会把节点从那个执行器里摘走，之后定时器、订阅、
        # 服务全部停摆——表现就是 engage 之后 start 再也没有响应。
        # 响应由 services 那个 Reentrant 组的另一个线程收下。
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        return future.done()

    ##
    # 控制环
    ##

    def _torso_quat_locked(self) -> np.ndarray:
        """由盆骨 IMU 与腰三轴推出躯干姿态。调用方需持锁。"""
        waist = self._measured[self._waist_slots]
        return torso_quat_from_pelvis(self._imu_quat, waist[0], waist[1], waist[2])

    def _control(self) -> None:
        with self._lock:
            if self._state not in ACTIVE_STATES:
                return
            stale = self._stale()
            if not stale:
                measured = self._measured.copy()
                measured_vel = self._measured_vel.copy()
                omega = self._imu_omega.copy()
                torso = self._torso_quat_locked()
                state = self._state
                stand_from = None if self._stand_from is None else self._stand_from.copy()
                stand_elapsed = self._now() - self._stand_start
                clip = self._clip
        if stale:
            self._estop(stale)
            return

        # 躯干 z 轴与世界 z 的夹角，旋转矩阵的 R[2][2] 就是它的余弦。
        tilt = math.acos(min(max(float(quat_to_mat(torso)[2, 2]), -1.0), 1.0))
        if tilt > self._tilt_limit:
            self._estop(f'躯干倾角 {tilt:.2f} rad 超过 {self._tilt_limit:.2f}')
            return

        targets = np.empty(len(self._joints))
        targets[self._gripper_slots] = self._gripper_targets

        if state is State.STAND:
            # 从实测位形插值到参考动作第 0 帧的位形，插完再切 RUNNING。
            goal = np.empty(len(self._joints))
            goal[self._gripper_slots] = self._gripper_targets
            goal[self._action_slots] = clip.joint_pos[0][self._policy.action_slots]
            alpha = min(stand_elapsed / max(self._stand_s, 1e-3), 1.0)
            targets[:] = stand_from + alpha * (goal - stand_from)
            if alpha >= 1.0:
                with self._lock:
                    self._state = State.RUNNING
                self.get_logger().info(f'开始放 {clip.name}')
        else:
            # 偏航在放第 0 帧那一拍才锁：STAND 会把腰三轴插到参考位形，躯干朝向随之改变，
            # 在 STAND 起点锁会把 waist_yaw 的变化量当成偏航差带进观测。loop 回卷同理重锁。
            if self._policy.frame == 0:
                clip.align_yaw(torso)
            try:
                action_targets = self._policy.step(
                    clip=clip,
                    joint_pos=measured[self._obs_slots],
                    joint_vel=measured_vel[self._obs_slots],
                    ang_vel=omega,
                    anchor_quat=torso,
                )
            except ValueError as exc:
                self._estop(f'策略推理失败: {exc}')
                return
            targets[self._action_slots] = action_targets

            if self._policy.frame >= clip.num_frames:
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
        return ''

    def _publish_status(self) -> None:
        with self._lock:
            state, reason, clip = self._state, self._reason, self._clip
            frame = self._policy.frame
        message = String()
        message.data = (f'state={state.value} motion={clip.name} '
                        f'frame={frame}/{clip.num_frames}'
                        + (f' reason={reason}' if reason else ''))
        self._status_publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = GmtTrackingNode()
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
