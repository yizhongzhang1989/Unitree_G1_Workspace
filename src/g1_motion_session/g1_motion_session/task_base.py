"""任务在本地处理生命周期与控制输出，session 只协调事件"""

import json
import threading
import time
from enum import Enum
from pathlib import Path
from typing import overload

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String

from g1_motion_session.msg import MotionEvent
from g1_motion_session.srv import RegisterTask
from .joint_output import JointOutput, two_stage
from .robot_state import RobotState


class State(str, Enum):
    IDLE = 'IDLE'
    STAND = 'STAND'
    RUNNING = 'RUNNING'
    STOPPED = 'STOPPED'
    ERROR = 'ERROR'


ACTIVE_STATES = (State.STAND, State.RUNNING)


class TaskBase(Node):
    def __init__(self, node_name, *, slots, imu_required=True):
        super().__init__(node_name)
        hardware = Path(get_package_share_directory('unitree_g1_ros2_control'))
        defaults = yaml.safe_load((hardware / 'config/default_31dof_param.yaml').read_text())
        p = self.declare_parameter
        self.joints31 = list(p('joints', defaults['/**']['ros__parameters']['joints'])
                             .get_parameter_value().string_array_value)
        self.joints = self.joints31
        self.slots = list(slots)
        self.dt = 1.0 / p('control_rate_hz', 50.0).get_parameter_value().double_value
        self.stand_s = p('stand_s', 2.5).get_parameter_value().double_value
        self.clear_s = p('stand_clear_s', 0.4).get_parameter_value().double_value
        self.clear_roll = p('stand_clear_roll', 0.7).get_parameter_value().double_value
        self.target_topic = p('target_topic', '/forward_position_controller/joint_targets')\
            .get_parameter_value().string_value
        p('robot_description_topic', '/robot_description')
        passive = p('passive_targets', [
            0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, -0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).get_parameter_value().double_array_value
        self._stand_goal = np.zeros(31)
        self._stand_goal[15:] = passive
        self._state_timeout = p('state_timeout_s', 0.2).get_parameter_value().double_value
        self._imu_required = imu_required
        self.lock = threading.RLock()
        # 停止与发布共用执行锁，阻止停止回调之后提交迟到目标
        self.execution_lock = threading.RLock()
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.state = State.IDLE
        self.generation = 0
        self.stand_start = 0.0
        self._epoch = 0
        self._preparing = False
        self._registered = False
        self._register_future = None
        self._error = ''
        self._events = {}
        self._q, self._dq = np.zeros(31), np.zeros(31)
        self._quat, self._omega = (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0)
        self._joint_stamp = self._imu_stamp = 0.0
        self._joint_names, self._joint_indices = [], []
        self._prepare_start = self._q.copy()
        self._prepare_via = self._q.copy()
        self.output = JointOutput(self, self.target_topic, self.slots)
        self._mask = self.output.message.mask
        events = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE)
        stream = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        sensors = MutuallyExclusiveCallbackGroup()
        self._task_status_publisher = self.create_publisher(String, '/motion_control/tasks', events)
        self._register = self.create_client(RegisterTask, '/motion_control/register')
        self.create_subscription(MotionEvent, '/motion_control/events', self._on_event,
                                 events, callback_group=self.control_group)
        self._subscribe_feedback(stream, sensors)
        if imu_required:
            imu_topic = p('imu_topic', '/pelvis_imu_broadcaster/imu').get_parameter_value().string_value
            self.create_subscription(Imu, imu_topic,
                                     self._on_imu, stream, callback_group=sensors)
        self.create_timer(self.dt, self._tick_lifecycle, callback_group=self.control_group)
        self.create_timer(0.1, self._publish_task_status, callback_group=self.control_group)
        self._registration_timer = self.create_timer(0.5, self._register_task)

    def _subscribe_feedback(self, qos, callback_group):
        self.create_subscription(JointState, '/joint_states', self._on_joint_states,
                                 qos, callback_group=callback_group)

    @overload
    def declare(self, name: str, default: bool) -> bool: ...

    @overload
    def declare(self, name: str, default: int) -> int: ...

    @overload
    def declare(self, name: str, default: float) -> float: ...

    @overload
    def declare(self, name: str, default: str) -> str: ...

    @overload
    def declare(self, name: str, default: list[str]) -> list[str]: ...

    @overload
    def declare(self, name: str, default: list[float]) -> list[float]: ...

    def declare(self, name: str, default: bool | int | float | str | list[str] | list[float]):
        parameter = (self.get_parameter(name) if self.has_parameter(name)
                     else self.declare_parameter(name, default))
        value = parameter.get_parameter_value()
        if isinstance(default, bool) and parameter.type_ is parameter.Type.BOOL:
            return value.bool_value
        if isinstance(default, int) and parameter.type_ is parameter.Type.INTEGER:
            return value.integer_value
        if isinstance(default, float) and parameter.type_ is parameter.Type.DOUBLE:
            return value.double_value
        if isinstance(default, str) and parameter.type_ is parameter.Type.STRING:
            return value.string_value
        if isinstance(default, list):
            if parameter.type_ is parameter.Type.STRING_ARRAY:
                return list(value.string_array_value)
            if parameter.type_ is parameter.Type.DOUBLE_ARRAY:
                return list(value.double_array_value)
        raise TypeError(f'{name} has incompatible parameter type {parameter.type_}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def snapshot(self):
        with self.lock:
            return RobotState(self._q.copy(), self._dq.copy(), self._quat, self._omega)

    @property
    def ready(self):
        return True

    def register_event(self, command, callback):
        self._events[command] = callback

    def _register_task(self):
        if self._registered or self._register_future is not None or not self._register.service_is_ready():
            return
        self._register_future = self._register.call_async(RegisterTask.Request(
            task_id=self.get_fully_qualified_name(), mask=self._mask, release=False))
        self._register_future.add_done_callback(self._registered_task)

    def _registered_task(self, future):
        self._register_future = None
        try:
            response = future.result()
            self._registered = response.accepted
            self._error = '' if response.accepted else response.reason
            if not response.accepted:
                self.get_logger().error(f'Task registration rejected: {response.reason}')
            self._registration_timer.cancel()
        except Exception as error:
            self._error = str(error)

    def _on_joint_states(self, message):
        if list(message.name) != self._joint_names:
            if any(name not in message.name for name in self.joints31):
                return
            self._joint_names = list(message.name)
            self._joint_indices = [self._joint_names.index(name) for name in self.joints31]
        if len(message.position) != len(message.name):
            return
        with self.lock:
            self._q = np.asarray(message.position)[self._joint_indices]
            if len(message.velocity) == len(message.name):
                self._dq = np.asarray(message.velocity)[self._joint_indices]
            self._joint_stamp = time.monotonic()

    def _on_imu(self, message):
        quat, omega = message.orientation, message.angular_velocity
        with self.lock:
            self._quat = (quat.x, quat.y, quat.z, quat.w)
            self._omega = (omega.x, omega.y, omega.z)
            self._imu_stamp = time.monotonic()

    def stale_reason(self):
        now = time.monotonic()
        if now - self._joint_stamp > self._state_timeout:
            return '/joint_states timeout'
        if self._imu_required and now - self._imu_stamp > self._state_timeout:
            return 'IMU timeout'
        return ''

    def _on_event(self, event):
        if not self._registered or event.epoch <= self._epoch:
            return
        with self.execution_lock:
            with self.lock:
                self._epoch = event.epoch
                command = event.command
                if command == 'engage' and self.state is State.RUNNING:
                    return
                if command == 'start' and (self.state is not State.STAND or self._preparing or not self.ready):
                    return
                if command in ('engage', 'stop'):
                    self.generation += 1
                try:
                    if command == 'engage':
                        reason = self.stale_reason()
                        if reason:
                            raise RuntimeError(reason)
                        self._error = ''
                        self.output.message.mask = self._mask
                        self.stand_start = event.start_time
                        self.state, self._preparing = State.STAND, True
                        self.engage(self._q.copy())
                        callback = self._events.get('engage')
                        if callback is not None:
                            callback(self._q.copy())
                    elif command == 'start':
                        self.start(self.snapshot())
                        self.state = State.RUNNING
                        callback = self._events.get('start')
                        if callback is not None:
                            callback(self.snapshot())
                    elif command == 'stop':
                        self.state, self._preparing = State.STOPPED, False
                        self.stop()
                        callback = self._events.get('stop')
                        if callback is not None:
                            callback()
                except Exception as error:
                    self.state, self._preparing, self._error = State.ERROR, False, str(error)
        self._publish_task_status()

    def engage(self, positions31):
        self._prepare_start = positions31.copy()
        self._prepare_via = positions31.copy()
        for side, sign in (('left', 1.0), ('right', -1.0)):
            index = self.joints31.index(f'{side}_shoulder_roll_joint')
            self._prepare_via[index] = sign * max(sign * positions31[index], self.clear_roll)
        self.output.positions[:] = positions31

    def start(self, snapshot):
        pass

    def prepared(self, snapshot):
        """预备终点已发布，通知任务完成播种；不替任务调用业务 start"""
        pass

    def stop(self):
        pass

    def status(self):
        return {}

    def _tick_lifecycle(self):
        with self.execution_lock:
            try:
                if self._preparing:
                    elapsed = self.now() - self.stand_start
                    if elapsed < 0.0:
                        return
                    target = two_stage(elapsed, self.clear_s, self.stand_s,
                                       self._prepare_start, self._prepare_via, self._stand_goal)
                    if not self.publish_target(target):
                        return
                    if elapsed >= self.stand_s and self.ready:
                        with self.lock:
                            self._preparing = False
                            snapshot = RobotState(self.output.positions.copy(), self._dq.copy(),
                                                  self._quat, self._omega)
                            self.prepared(snapshot)
                        self._publish_task_status()
            except Exception as error:
                self.estop(str(error))

    def publish_target(self, positions=None):
        with self.execution_lock, self.lock:
            if not self._registered or self.state not in ACTIVE_STATES:
                return False
            if positions is not None:
                self.output.positions[:] = positions
            self.output.publish()
            return True

    def estop(self, reason):
        with self.execution_lock:
            with self.lock:
                self.generation += 1
                self.state, self._preparing, self._error = State.ERROR, False, reason
            try:
                self.stop()
                callback = self._events.get('stop')
                if callback is not None:
                    callback()
            except Exception as error:
                self._error = f'{reason}; stop failed: {error}'
            try:
                self._publish_task_status()
            except Exception as error:
                self._error = f'{self._error}; status failed: {error}'

    def _publish_task_status(self):
        if not self._registered:
            return
        with self.lock:
            detail = self.status()
            callback = self._events.get('status')
            if callback is not None:
                detail.update(callback())
            if any(index >= 15 for index in self.slots):
                detail.update(upper_node=self.get_fully_qualified_name(),
                              upper_state='active' if self.state is State.RUNNING else 'idle',
                              arms_live=self.state is State.RUNNING,
                              arm_joints=self.joints31[15:29])
            state = 'preparing' if self._preparing else (
                'ready' if self.state is State.STAND else self.state.value.lower())
            payload = dict(detail, task_id=self.get_fully_qualified_name(), mask=self._mask,
                           state=state, epoch=self._epoch, ready=self.ready and not self._preparing,
                           error=self._error or detail.get('error', ''))
        self._task_status_publisher.publish(String(data=json.dumps(payload)))

    def destroy_node(self):
        with self.execution_lock, self.lock:
            self.generation += 1
            self.state = State.STOPPED
        if self._registered and self.context.ok() and self._register.service_is_ready():
            self._register.call_async(RegisterTask.Request(
                task_id=self.get_fully_qualified_name(), mask=self._mask, release=True))
        return super().destroy_node()
