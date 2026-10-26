"""Real ROS lifecycle/data-flow checks using only domain 216 and fake hardware."""

import json
import os
from pathlib import Path
import threading
import time
import uuid

import numpy as np
import pytest
import rclpy
from controller_manager_msgs.srv import SwitchController
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from unitree_g1_ros2_control.msg import JointTargets
import yaml

from g1_motion_session.msg import MotionEvent
from g1_motion_session.srv import RegisterTask
from g1_motion_session.session_node import MotionSession, State as SessionState
from g1_motion_session.task_base import TaskBase, State as TaskState


PACKAGE = Path(__file__).resolve().parents[1]
HARDWARE_CONFIG = PACKAGE.parent / 'unitree_g1_ros2_control/config/default_31dof_param.yaml'
FULL_MASK = (1 << 31) - 1


class LifecycleExecutor(MultiThreadedExecutor):
    def shutdown(self, timeout_sec=None):
        stopped = super().shutdown(timeout_sec=timeout_sec)
        if stopped:
            self._executor.shutdown(wait=True)
            for future in self._futures:
                if future.done():
                    future.result()
            self._futures.clear()
        return stopped

    def _take_subscription(self, subscription):
        with subscription.handle:
            result = subscription.handle.take_message(subscription.msg_type, subscription.raw)
        if result is None:
            return None
        message, info = result
        if subscription.msg_type is JointTargets:
            harness = subscription.callback.__self__
            with harness.lock:
                harness.target_stamps.append(info['source_timestamp'])
        return message


class LifecycleTask(TaskBase):
    def __init__(self, name, slots, *, ready=True):
        self.ready_event = threading.Event()
        if ready:
            self.ready_event.set()
        self.engage_seeds = []
        self.start_seeds = []
        self.stop_calls = 0
        self.accepted = 0
        self.blocked = 0
        self.preparation = []
        self.run_goal = None
        super().__init__(name, slots=slots)
        self.create_timer(self.dt, self._step, callback_group=self.control_group)

    @property
    def ready(self):
        return self.ready_event.is_set()

    def engage(self, positions31):
        self.engage_seeds.append(positions31.copy())
        super().engage(positions31)

    def start(self, snapshot):
        self.start_seeds.append(snapshot.position.copy())
        self.run_goal = snapshot.position.copy()
        self.run_goal[self.slots] += 0.05

    def stop(self):
        self.stop_calls += 1

    def _step(self):
        with self.execution_lock:
            elapsed = self.now() - self.stand_start
            if self.state is TaskState.STAND and self.engage_seeds:
                if elapsed < 0.0:
                    return
                if elapsed < self.clear_s:
                    alpha = elapsed / self.clear_s
                    goal = self._prepare_start + alpha * (self._prepare_via - self._prepare_start)
                else:
                    alpha = min((elapsed - self.clear_s) / (self.stand_s - self.clear_s), 1.0)
                    goal = self._prepare_via + alpha * (self._stand_goal - self._prepare_via)
            else:
                goal = self.run_goal if self.state is TaskState.RUNNING else self.output.positions.copy()
            assert goal is not None
            if self.publish_target(goal):
                self.accepted += 1
                if self.state is TaskState.STAND and self._preparing:
                    self.preparation.append((elapsed, goal.copy()))
            else:
                self.blocked += 1


class RosHarness:
    def __init__(self):
        self.nodes = []
        self.tasks = []
        self.session = None
        self.executor = LifecycleExecutor(num_threads=6)
        self.finished = threading.Event()
        self.changed = threading.Event()
        self.lock = threading.RLock()
        self.spin_errors = []
        self.targets = []
        self.target_stamps = []
        self.events = []
        self.switches = []
        self.status = {}
        self.sensor_ticks = 0
        self.thread = None
        self.joints = yaml.safe_load(HARDWARE_CONFIG.read_text())['/**']['ros__parameters']['joints']
        self.sensor_positions = np.arange(31, dtype=float) / 1000.0 + 0.1
        self.sensor_positions[16], self.sensor_positions[23] = 0.1, -0.1
        self.sensor_positions[18], self.sensor_positions[25] = 0.2, -0.2

    def start(self):
        self.io = self.add(Node('lifecycle_fake_hardware'))
        stream = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.joint_publisher = self.io.create_publisher(JointState, '/joint_states', stream)
        self.imu_publisher = self.io.create_publisher(Imu, '/pelvis_imu_broadcaster/imu', stream)
        self.event_publisher = self.io.create_publisher(MotionEvent, '/motion_control/events', 10)
        self.io.create_subscription(JointTargets, '/forward_position_controller/joint_targets',
                                    self._target, stream)
        self.io.create_subscription(MotionEvent, '/motion_control/events', self._event, 10)
        self.io.create_subscription(String, '/motion_control/status', self._status, 10)
        self.io.create_service(SwitchController, '/controller_manager/switch_controller',
                               self._switch, callback_group=MutuallyExclusiveCallbackGroup())
        self.io.create_timer(0.005, self._sensors,
                             callback_group=MutuallyExclusiveCallbackGroup())
        self.clients = {
            command: self.io.create_client(Trigger, f'/motion_control/{command}')
            for command in ('engage', 'start', 'stop', 'estop')
        }
        self.registration = self.io.create_client(RegisterTask, '/motion_control/register')
        self.thread = threading.Thread(target=self._spin, name='lifecycle_executor')
        self.thread.start()

    def add(self, node):
        self.nodes.append(node)
        self.executor.add_node(node)
        return node

    def add_task(self, name, slots, *, ready=True):
        task = LifecycleTask(name, slots, ready=ready)
        self.tasks.append(task)
        return self.add(task)

    def add_session(self):
        session = MotionSession()
        self.session = self.add(session)
        assert self.session.joints == self.joints
        config = yaml.safe_load((PACKAGE / 'config/session.yaml').read_text())['/**']['ros__parameters']
        assert self.session.target_topic == config['target_topic']
        assert self.session._state_timeout == config['state_timeout_s']
        np.testing.assert_array_equal(self.session._default_goal,
                                      config['default_lower_targets'] + config['passive_targets'])
        assert self.session.stand_s == 0.12 and self.session.clear_s == 0.03
        self.wait(lambda: not session.stale_reason(), 'session receives both fake sensors')
        return self.session

    def _spin(self):
        try:
            while not self.finished.is_set():
                self.executor.spin_once(timeout_sec=0.05)
        except Exception as error:
            self.spin_errors.append(error)
            self.changed.set()

    def _sensors(self):
        with self.lock:
            positions = self.sensor_positions.tolist()
        stamp = self.io.get_clock().now().to_msg()
        joints = JointState(name=self.joints, position=positions, velocity=[0.0] * 31)
        joints.header.stamp = stamp
        imu = Imu()
        imu.header.stamp = stamp
        imu.orientation.w = 1.0
        self.joint_publisher.publish(joints)
        self.imu_publisher.publish(imu)
        with self.lock:
            self.sensor_ticks += 1
        self.changed.set()

    def _target(self, message):
        with self.lock:
            self.targets.append((message.mask, np.asarray(message.positions).copy()))
        self.changed.set()

    def _event(self, message):
        with self.lock:
            self.events.append((message.command, message.epoch, message.start_time))
        self.changed.set()

    def _status(self, message):
        with self.lock:
            self.status = json.loads(message.data)
        self.changed.set()

    def _switch(self, request, response):
        with self.lock:
            self.switches.append((list(request.activate_controllers),
                                  list(request.deactivate_controllers)))
        response.ok = True
        self.changed.set()
        return response

    def wait(self, predicate, description, timeout=5.0):
        deadline = time.monotonic() + timeout
        while True:
            self.changed.clear()
            assert not self.spin_errors, self.spin_errors
            if predicate():
                return
            remaining = deadline - time.monotonic()
            assert remaining > 0.0, f'{description}: status={self.status}, switches={self.switches}'
            self.changed.wait(min(remaining, 0.02))

    def observe_for(self, duration):
        deadline = time.monotonic() + duration
        self.wait(lambda: time.monotonic() >= deadline, 'bounded observation', duration + 1.0)

    def call(self, command):
        client = self.clients[command]
        self.wait(client.service_is_ready, f'{command} service discovery')
        future = client.call_async(Trigger.Request())
        self.wait(future.done, f'{command} service response')
        return future.result()

    def target_rows(self, mask):
        with self.lock:
            return [positions for current_mask, positions in self.targets if current_mask == mask]

    def task_status(self, task):
        with self.lock:
            return next((entry for entry in self.status.get('tasks', [])
                         if entry['task_id'] == task.get_fully_qualified_name()), {})

    def fresh(self, task):
        self.wait(lambda: not task.stale_reason(), f'{task.get_name()} receives sensors')
        assert task.joints31 == self.joints

    def destroy_task(self, task):
        task_id = task.get_fully_qualified_name()
        assert self.thread is not None
        assert self.session is not None
        self.finished.set()
        self.executor.wake()
        self.thread.join(timeout=5.0)
        assert not self.thread.is_alive(), 'executor thread did not pause'
        assert self.executor.shutdown(timeout_sec=5.0), 'task callbacks did not drain'
        task.destroy_node()
        self.executor.remove_node(task)
        self.tasks.remove(task)
        self.nodes.remove(task)
        self.executor = LifecycleExecutor(num_threads=6)
        for node in self.nodes:
            self.executor.add_node(node)
        self.finished.clear()
        self.thread = threading.Thread(target=self._spin, name='lifecycle_executor')
        self.thread.start()
        self.wait(lambda: all(entry['task_id'] != task_id for entry in self.status.get('tasks', [])),
                  'destroy releases registration through ROS')
        with self.session.lock:
            assert task_id not in self.session._tasks

    def close(self):
        try:
            if self.session is not None and self.thread is not None and self.thread.is_alive():
                self.session.shutdown()
        finally:
            self.finished.set()
            self.executor.wake()
            if self.thread is not None:
                self.thread.join(timeout=5.0)
                assert not self.thread.is_alive(), 'executor thread did not exit'
            assert self.executor.shutdown(timeout_sec=5.0), 'executor callbacks did not exit'
            for node in reversed(self.nodes):
                node.destroy_node()
            self.nodes.clear()
            self.tasks.clear()


@pytest.fixture
def ros():
    assert os.environ.get('ROS_DOMAIN_ID') == '216', 'run with explicit env ROS_DOMAIN_ID=216'
    assert not rclpy.ok(), 'test requires its own ROS context'
    args = [
        '--ros-args', '--params-file', str(HARDWARE_CONFIG),
        '--params-file', str(PACKAGE / 'config/session.yaml'),
        '-p', 'stand_s:=0.12', '-p', 'stand_clear_s:=0.03', '-p', 'control_rate_hz:=200.0',
    ]
    prefix = f'/lifecycle_{uuid.uuid4().hex}'
    for endpoint in (
        '/joint_states', '/pelvis_imu_broadcaster/imu',
        '/forward_position_controller/joint_targets', '/controller_manager/switch_controller',
        '/motion_control/events', '/motion_control/tasks', '/motion_control/register',
        '/motion_control/status', '/motion_control/engage', '/motion_control/start',
        '/motion_control/stop', '/motion_control/estop',
    ):
        args.extend(['-r', f'{endpoint}:={prefix}{endpoint}'])
    rclpy.init(args=args, domain_id=216, signal_handler_options=SignalHandlerOptions.NO)
    harness = None
    try:
        assert rclpy.get_default_context().get_domain_id() == 216
        harness = RosHarness()
        harness.start()
        yield harness
    finally:
        try:
            if harness is not None:
                harness.close()
        finally:
            rclpy.shutdown()


def test_registration_gates_real_targets_and_overlap_is_rejected(ros):
    task = ros.add_task('lifecycle_unregistered', [16, 18])
    ros.fresh(task)
    ros.wait(lambda: ros.event_publisher.get_subscription_count() >= 2, 'event discovery')
    ros.event_publisher.publish(MotionEvent(command='engage', epoch=1, start_time=task.now()))
    ros.observe_for(0.1)
    assert not task._registered and task.state is TaskState.IDLE
    assert not task.engage_seeds and not ros.target_rows(task._mask)
    with task.execution_lock:
        task.state = TaskState.STAND
        assert not task.publish_target(np.ones(31))
    ros.observe_for(0.1)
    assert task.blocked > 0 and not ros.target_rows(task._mask)
    with task.execution_lock:
        task.state = TaskState.IDLE
    ros.add_session()
    ros.wait(lambda: task._registered and bool(ros.task_status(task)), 'successful real registration')
    conflict = ros.add_task('lifecycle_overlap', [18, 23])
    ros.fresh(conflict)
    ros.wait(lambda: bool(conflict._error), 'overlapping registration rejected')
    assert not conflict._registered and 'mask overlaps' in conflict._error
    assert ros.task_status(conflict) == {}
    with conflict.execution_lock:
        conflict.state = TaskState.STAND
        assert not conflict.publish_target(np.ones(31))
    ros.observe_for(0.1)
    assert not ros.target_rows(conflict._mask)
    assert ros.session.default_output.message.mask == FULL_MASK & ~task._mask
    ros.destroy_task(task)
    assert ros.session.default_output.message.mask == FULL_MASK
    ros.wait(ros.registration.service_is_ready, 'release/reclaim service discovery')
    request = RegisterTask.Request(task_id=conflict.get_fully_qualified_name(),
                                   mask=conflict._mask, release=False)
    future = ros.registration.call_async(request)
    ros.wait(future.done, 'released slots can be claimed')
    assert future.result().accepted
    request = RegisterTask.Request(task_id='/lifecycle_reverse_overlap', mask=(1 << 16) | (1 << 18))
    future = ros.registration.call_async(request)
    ros.wait(future.done, 'reverse overlap rejected')
    assert not future.result().accepted and 'mask overlaps' in future.result().reason
    assert ros.switches == []


def test_engage_prepare_dynamic_registration_start_stop_estop_and_destroy(ros):
    session = ros.add_session()
    first = ros.add_task('lifecycle_first', [16, 18])
    ros.fresh(first)
    ros.wait(lambda: first._registered and bool(ros.task_status(first)), 'first task registration')
    assert not ros.call('start').success
    assert ros.call('engage').success
    ros.wait(lambda: ros.status.get('ready_to_start'), 'initial two-stage preparation complete')
    assert ros.switches == [(['forward_position_controller'], [])]
    assert first.state is TaskState.STAND and first.generation == 1
    assert len(first.engage_seeds) == 1 and not first.start_seeds
    ros.wait(lambda: any(np.array_equal(row[first.slots], [0.6, 0.0])
                         for row in ros.target_rows(first._mask)), 'task preparation endpoint received')
    first_rows = ros.target_rows(first._mask)
    for lower, upper in ((0.0, first.clear_s), (first.clear_s, first.stand_s)):
        phase = [goal for elapsed, goal in first.preparation if lower < elapsed < upper]
        assert phase, f'no actual preparation samples in {lower}..{upper}'
        assert any(any(np.array_equal(goal, row) for row in first_rows) for goal in phase)
    assert any(0.1 < row[16] < 0.7 and row[18] == 0.2 for row in first_rows)
    assert any(0.6 < row[16] < 0.7 and 0.0 < row[18] < 0.2 for row in first_rows)
    np.testing.assert_allclose(first_rows[-1][first.slots], [0.6, 0.0], atol=1e-12)
    defaults_mask = FULL_MASK & ~first._mask
    ros.wait(lambda: any(np.allclose(row, session._default_goal, rtol=0.0, atol=1e-12)
                         for row in ros.target_rows(defaults_mask)), 'default preparation endpoint received')
    default_rows = ros.target_rows(defaults_mask)
    assert any(-0.7 < row[23] < -0.1 and row[0] == first.engage_seeds[0][0]
               for row in default_rows)
    assert any(-0.7 < row[23] < -0.6 and -0.1 < row[0] < first.engage_seeds[0][0]
               for row in default_rows)
    np.testing.assert_allclose(default_rows[-1], session._default_goal, atol=1e-12)
    assert ros.call('start').success
    ros.wait(lambda: first.state is TaskState.RUNNING and bool(first.start_seeds), 'first starts')
    ros.wait(lambda: any(np.array_equal(row, first.run_goal) for row in ros.target_rows(first._mask)),
             'first running targets received')
    generation, execution_epoch = first.generation, ros.status['execution_epoch']
    prepare_seed, start_seed = first.engage_seeds[0].copy(), first.start_seeds[0].copy()
    with ros.lock:
        ros.sensor_positions += 0.03
        new_positions = ros.sensor_positions.copy()
    second = ros.add_task('lifecycle_second', [23, 25], ready=False)
    ros.fresh(second)
    ros.wait(lambda: second._registered and bool(ros.task_status(second)), 'late task registers')
    ros.wait(lambda: np.array_equal(second.snapshot().position, new_positions), 'fresh late-task seed')
    assert ros.call('engage').success
    ros.wait(lambda: bool(second.engage_seeds), 'late task prepares')
    ros.observe_for(0.25)
    assert session.state is SessionState.RUNNING and first.state is TaskState.RUNNING
    assert first.generation == generation and len(first.engage_seeds) == len(first.start_seeds) == 1
    np.testing.assert_array_equal(first.engage_seeds[0], prepare_seed)
    np.testing.assert_array_equal(first.start_seeds[0], start_seed)
    np.testing.assert_array_equal(second.engage_seeds[0], new_positions)
    assert second._preparing and not second.start_seeds
    assert ros.status['execution_epoch'] == execution_epoch
    assert ros.switches == [(['forward_position_controller'], [])]
    assert not ros.call('start').success
    assert first.state is TaskState.RUNNING and not second.start_seeds
    second.ready_event.set()
    ros.wait(lambda: ros.status.get('ready_to_start') and ros.task_status(second).get('state') == 'ready',
             'only ready late module may start')
    assert ros.call('start').success
    ros.wait(lambda: second.state is TaskState.RUNNING and bool(second.start_seeds), 'late module starts')
    ros.wait(lambda: any(np.array_equal(row, second.run_goal) for row in ros.target_rows(second._mask)),
             'late-module running targets received')
    assert first.generation == generation and len(first.start_seeds) == 1
    assert session.default_output.message.mask == FULL_MASK & ~(first._mask | second._mask)
    assert ros.call('stop').success
    ros.wait(lambda: first.state is second.state is TaskState.STOPPED and ros.status.get('state') == 'idle',
             'stop reaches both tasks')
    stop_stamp = time.time_ns()
    ros.observe_for(0.08)
    counts = len(ros.targets), first.accepted, second.accepted
    blocked, ticks = first.blocked + second.blocked, ros.sensor_ticks
    ros.observe_for(0.12)
    assert (first.accepted, second.accepted) == counts[1:]
    assert ros.target_stamps and all(0 < stamp <= stop_stamp for stamp in ros.target_stamps)
    assert first.blocked + second.blocked > blocked and ros.sensor_ticks > ticks
    assert first.stop_calls == second.stop_calls == 1
    assert session._controller_active and ros.switches == [(['forward_position_controller'], [])]
    assert ros.call('estop').success
    ros.wait(lambda: ros.status.get('state') == 'estop', 'estop status arrives')
    assert ros.switches == [(['forward_position_controller'], []), ([], ['forward_position_controller'])]
    assert not session._controller_active and not session._deactivating
    ros.observe_for(0.08)
    assert (first.accepted, second.accepted) == counts[1:]
    assert all(0 < stamp <= stop_stamp for stamp in ros.target_stamps)
    ros.destroy_task(first)
    assert session.default_output.message.mask == FULL_MASK & ~second._mask
    ros.destroy_task(second)
    assert session.default_output.message.mask == FULL_MASK
    assert not ros.spin_errors
