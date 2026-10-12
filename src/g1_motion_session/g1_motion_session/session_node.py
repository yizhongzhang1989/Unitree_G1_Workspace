"""协调任务注册和生命周期，默认输出只覆盖尚未认领的关节"""

import json
import math
import threading
import time
from enum import Enum

import numpy as np
import rclpy
from controller_manager_msgs.srv import SwitchController
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from g1_motion_session.msg import MotionEvent
from g1_motion_session.srv import RegisterTask


FULL_MASK = (1 << 31) - 1


class State(str, Enum):
    IDLE = 'idle'
    ENGAGING = 'engaging'
    STAND = 'stand'
    RUNNING = 'running'
    ESTOP = 'estop'


ACTIVE_STATES = (State.ENGAGING, State.STAND, State.RUNNING)


def tilt(quat):
    length = sum(value * value for value in quat)
    if not math.isfinite(length) or length <= 0.0:
        return math.pi
    cosine = 1.0 - 2.0 * (quat[0] ** 2 + quat[1] ** 2) / length
    return math.acos(min(max(cosine, -1.0), 1.0))


class DefaultJointOutput:
    def __init__(self, node, topic):
        from g1_motion_session.joint_output import JointOutput

        self._output = JointOutput(node, topic, list(range(31)))
        self.message = self._output.message
        self.positions = self._output.positions
        self.positions31 = self.positions

    def publish(self):
        self._output.publish()


class MotionSession(Node):
    def __init__(self):
        super().__init__('motion_control')
        parameter = self.declare_parameter
        self.joints = list(parameter('joints', ['']).get_parameter_value().string_array_value)
        if len(self.joints) != 31 or len(set(self.joints)) != 31:
            raise ValueError('joints must contain 31 distinct hardware joints')
        self.dt = 1.0 / parameter('control_rate_hz', 50.0).get_parameter_value().double_value
        self.stand_s = parameter('stand_s', 2.5).get_parameter_value().double_value
        self.clear_s = parameter('stand_clear_s', 0.4).get_parameter_value().double_value
        self.clear_roll = parameter('stand_clear_roll', 0.7).get_parameter_value().double_value
        if not 0.0 <= self.clear_s < self.stand_s:
            raise ValueError('clearance time must be shorter than stand')
        self._safety_checks_enabled = parameter(
            'safety_checks_enabled', True,
            descriptor=ParameterDescriptor(read_only=True)).get_parameter_value().bool_value
        self._state_timeout = parameter('state_timeout_s', 0.2).get_parameter_value().double_value
        self._tilt_limit = parameter('tilt_limit_rad', 1.0).get_parameter_value().double_value
        self._switch_timeout = parameter('switch_timeout_s', 15.0).get_parameter_value().double_value
        self._controller = parameter('controller_name', 'forward_position_controller').get_parameter_value().string_value
        manager = parameter('controller_manager', '/controller_manager').get_parameter_value().string_value.rstrip('/')
        self.target_topic = parameter('target_topic', '/forward_position_controller/joint_targets').get_parameter_value().string_value
        lower = parameter('default_lower_targets', [
            -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
            -0.1, 0.0, 0.0, 0.3, -0.2, 0.0, 0.0, 0.0, 0.0]).get_parameter_value().double_array_value
        upper = parameter('passive_targets', [
            0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, -0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).get_parameter_value().double_array_value
        if len(lower) != 15 or len(upper) != 16:
            raise ValueError('default targets must contain 15 lower and 16 upper joints')
        self._default_goal = np.asarray([*lower, *upper], dtype=float)
        self.lock = threading.RLock()
        self._switch_lock = threading.Lock()
        self.state = State.IDLE
        self.generation = 0
        self._event_epoch = 0
        self._claimed_epoch = 0
        self._controller_active = False
        self._deactivating = False
        self._reason = ''
        self._tasks = {}
        self._q = np.zeros(31)
        self._quat = (0.0, 0.0, 0.0, 1.0)
        self._joint_stamp = self._imu_stamp = 0.0
        self._js_names, self._js_index = [], []
        self._default_preparing = False
        self._default_start_time = 0.0
        self._default_start = self._q.copy()
        self._default_via = self._q.copy()
        self.default_output = DefaultJointOutput(self, self.target_topic)
        services = ReentrantCallbackGroup()
        state_group = MutuallyExclusiveCallbackGroup()
        # 注册与默认发布共用锁，避免掩码交接期间迟到输出
        self.control_group = MutuallyExclusiveCallbackGroup()
        stream = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        events = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE)
        self._events_publisher = self.create_publisher(MotionEvent, '/motion_control/events', events)
        self._status_publisher = self.create_publisher(String, '~/status', 10)
        self.create_subscription(String, '/motion_control/tasks', self._on_task_status,
                                 events, callback_group=state_group)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states,
                                 stream, callback_group=state_group)
        if self._safety_checks_enabled:
            self.create_subscription(Imu, parameter('imu_topic', '/pelvis_imu_broadcaster/imu').get_parameter_value().string_value,
                                     self._on_imu, stream, callback_group=state_group)
        self._switch = self.create_client(SwitchController, f'{manager}/switch_controller',
                                          callback_group=services)
        self.create_service(RegisterTask, '/motion_control/register', self._on_register,
                            callback_group=services)
        self.create_service(Trigger, '~/engage', self._on_engage, callback_group=services)
        self.create_service(Trigger, '~/start', self._on_start, callback_group=services)
        self.create_service(Trigger, '~/stop', self._on_stop, callback_group=services)
        self.create_service(Trigger, '~/estop', self._on_estop, callback_group=services)
        self.create_timer(self.dt, self._tick_defaults, callback_group=self.control_group)
        self.create_timer(self.dt, self._publish_status, callback_group=MutuallyExclusiveCallbackGroup())

    def now(self):
        return time.monotonic()

    def ros_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _unowned_mask(self):
        owned = 0
        for task in self._tasks.values():
            owned |= task['mask']
        return FULL_MASK & ~owned

    def _on_register(self, request, response):
        with self.lock:
            if request.release:
                self._tasks.pop(request.task_id, None)
                self.default_output.message.mask = self._unowned_mask()
                self.default_output.positions[:] = self._q
                response.accepted, response.reason = True, 'released'
                return response
            mask = request.mask & FULL_MASK
            if not request.task_id or not mask:
                response.accepted, response.reason = False, 'task_id and a nonempty joint mask required'
                return response
            for task_id, task in self._tasks.items():
                if task_id != request.task_id and task['mask'] & mask:
                    response.accepted, response.reason = False, f'mask overlaps {task_id}'
                    return response
            existing = self._tasks.get(request.task_id)
            if existing is None or existing['mask'] != mask:
                self._tasks[request.task_id] = dict(
                    task_id=request.task_id, mask=mask, state='idle', epoch=0,
                    ready=False, error='', status={})
            self.default_output.message.mask = self._unowned_mask()
            response.accepted, response.reason = True, 'registered'
        return response

    def _on_task_status(self, message):
        try:
            payload = json.loads(message.data)
            task_id = payload['task_id']
            epoch = int(payload['epoch'])
            mask = int(payload['mask']) & FULL_MASK
        except (ValueError, TypeError, KeyError):
            return
        with self.lock:
            task = self._tasks.get(task_id)
            if task is None or mask != task['mask'] or epoch < task['epoch']:
                return
            task['epoch'] = epoch
            task['state'] = str(payload.get('state', 'idle')).lower()
            task['ready'] = bool(payload.get('ready', False))
            task['error'] = payload.get('error', '')
            task['status'] = {key: value for key, value in payload.items()
                              if key not in ('task_id', 'mask', 'state', 'epoch', 'ready', 'error')}

    def _broadcast(self, command):
        self._event_epoch += 1
        event = MotionEvent(command=command, epoch=self._event_epoch, start_time=self.ros_now() + 0.05)
        self._events_publisher.publish(event)
        return event

    def _prepare_defaults(self, start_time):
        self._default_start = self._q.copy()
        self._default_via = self._q.copy()
        left = self.joints.index('left_shoulder_roll_joint')
        right = self.joints.index('right_shoulder_roll_joint')
        self._default_via[left] = max(self._q[left], self.clear_roll)
        self._default_via[right] = min(self._q[right], -self.clear_roll)
        self._default_start_time = start_time
        self._default_preparing = bool(self.default_output.message.mask)
        self.default_output.positions[:] = self._q

    def _on_engage(self, request, response):
        with self.lock:
            if self._deactivating:
                response.success, response.message = False, 'controller deactivation pending'
                return response
            reason = self.safety_reason()
            if reason:
                response.success, response.message = False, reason
                return response
            if self.state is State.ENGAGING and not self._controller_active:
                response.success, response.message = True, 'activation pending'
                return response
            self.generation += 1
            generation = self.generation
            if self._controller_active:
                self._begin_engage()
            else:
                self.state = State.ENGAGING
                self._reason = ''
                threading.Thread(target=self._activate, args=(generation,), daemon=True).start()
        response.success, response.message = True, 'engage accepted'
        return response

    def _activate(self, generation):
        with self._switch_lock:
            with self.lock:
                if generation != self.generation or self.state is State.ESTOP:
                    return
            ok, detail = self._switch_controller(activate=True)
            with self.lock:
                cancelled = generation != self.generation or self.state is State.ESTOP
                emergency = self.state is State.ESTOP
                if ok and not emergency:
                    self._controller_active = True
                    self._claimed_epoch = time.monotonic_ns()
                if not cancelled and ok:
                    self._begin_engage()
                elif not cancelled:
                    self.state, self._reason = State.ESTOP, detail
                    self._broadcast('stop')
            if emergency or not ok:
                self._switch_controller(activate=False)

    def _begin_engage(self):
        event = self._broadcast('engage')
        for task in self._tasks.values():
            if task['state'] != 'running':
                task.update(state='preparing', ready=False, error='', epoch=event.epoch)
        self._prepare_defaults(event.start_time)
        if self.state is not State.RUNNING:
            self.state = State.STAND
        self._reason = ''

    def _ready_to_start(self):
        return (self._controller_active and self.state in (State.STAND, State.RUNNING)
                and not self._default_preparing
                and all(task['state'] == 'running' or
                        (task['ready'] and task['state'] == 'ready' and not task['error'])
                        for task in self._tasks.values()))

    def _on_start(self, request, response):
        with self.lock:
            reason = self.safety_reason()
            if reason:
                response.success, response.message = False, reason
                return response
            if not self._ready_to_start():
                response.success, response.message = False, 'tasks or default joints not ready'
                return response
            event = self._broadcast('start')
            for task in self._tasks.values():
                if task['state'] != 'running':
                    task.update(state='running', ready=True, epoch=event.epoch)
            self.state = State.RUNNING
        response.success, response.message = True, 'running'
        return response

    def _stop_locked(self, reason):
        self.generation += 1
        self._broadcast('stop')
        self._default_preparing = False
        for task in self._tasks.values():
            task.update(state='stopped', ready=False, epoch=self._event_epoch)
        self._reason = reason

    def _on_stop(self, request, response):
        with self.lock:
            self._stop_locked('stopped')
            if self.state is not State.ESTOP:
                self.state = State.IDLE
        response.success, response.message = True, 'stopped; controller holding'
        return response

    def _on_estop(self, request, response):
        self.estop('operator estop')
        response.success, response.message = True, 'controller deactivation requested'
        return response

    def estop(self, reason):
        with self.lock:
            if self.state is State.ESTOP:
                return False
            active = self._controller_active or self.state is State.ENGAGING
            self._stop_locked(reason)
            self.state = State.ESTOP
            self._controller_active = False
            self._deactivating = active
        self.get_logger().error(f'estop: {reason}')
        if active:
            try:
                with self._switch_lock:
                    ok, detail = self._switch_controller(activate=False)
            finally:
                with self.lock:
                    self._deactivating = False
            if not ok:
                self.get_logger().error(f'controller deactivation failed: {detail}')
        return active

    def _tick_defaults(self):
        from g1_motion_session.joint_output import two_stage

        with self.lock:
            if not self._controller_active or self.state not in (State.STAND, State.RUNNING):
                return
            if not self.default_output.message.mask:
                self._default_preparing = False
                return
            if self._default_preparing:
                elapsed = self.ros_now() - self._default_start_time
                if elapsed < 0.0:
                    return
                self.default_output.positions[:] = two_stage(
                    elapsed, self.clear_s, self.stand_s,
                    self._default_start, self._default_via, self._default_goal)
                if elapsed >= self.stand_s:
                    self._default_preparing = False
            self.default_output.publish()

    def _on_joint_states(self, message):
        if list(message.name) != self._js_names:
            self._js_names = list(message.name)
            self._js_index = [(self.joints.index(name), index)
                              for index, name in enumerate(message.name) if name in self.joints]
        if len(message.position) != len(message.name):
            return
        with self.lock:
            for slot, index in self._js_index:
                self._q[slot] = message.position[index]
            if self._js_index:
                self._joint_stamp = self.now()

    def _on_imu(self, message):
        quat = message.orientation
        with self.lock:
            self._quat = (quat.x, quat.y, quat.z, quat.w)
            self._imu_stamp = self.now()

    def stale_reason(self):
        if not self._safety_checks_enabled:
            return ''
        now = self.now()
        if now - self._joint_stamp > self._state_timeout:
            return '/joint_states timeout'
        if now - self._imu_stamp > self._state_timeout:
            return 'IMU timeout'
        return ''

    def safety_reason(self):
        if not self._safety_checks_enabled:
            return ''
        return self.stale_reason() or (
            'tilt limit exceeded' if tilt(self._quat) > self._tilt_limit else '')

    def _publish_status(self):
        with self.lock:
            reason = self.safety_reason()
            active = self.state in ACTIVE_STATES
        if active and reason:
            self.estop(reason)
        with self.lock:
            payload = {}
            for task in self._tasks.values():
                payload.update(task['status'])
            payload.update(state=self.state.value, ready_to_start=self._ready_to_start(),
                           execution_epoch=self._claimed_epoch, reason=self._reason,
                           stale=self.stale_reason(), safety_checks_enabled=self._safety_checks_enabled,
                           joints=self.joints)
            payload['tasks'] = [{key: value for key, value in task.items() if key != 'status'}
                                for task in self._tasks.values()]
        self._status_publisher.publish(String(data=json.dumps(payload)))

    def _switch_controller(self, *, activate):
        if not self._switch.wait_for_service(timeout_sec=3.0):
            return False, 'controller_manager unavailable'
        request = SwitchController.Request()
        request.activate_controllers = [self._controller] if activate else []
        request.deactivate_controllers = [] if activate else [self._controller]
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        future = self._switch.call_async(request)
        deadline = self.now() + self._switch_timeout
        while not future.done() and self.now() < deadline:
            threading.Event().wait(0.005)
        if not future.done():
            return False, 'switch_controller timeout'
        try:
            result = future.result()
        except Exception as error:
            return False, str(error)
        return (True, 'ok') if result is not None and result.ok else (False, 'switch_controller rejected')

    def shutdown(self):
        if self.state in ACTIVE_STATES or self._controller_active:
            self.estop('node shutdown')


def main(args=None):
    rclpy.init(args=args)
    node = MotionSession()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
