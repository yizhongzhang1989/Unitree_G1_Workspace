"""Real VLA ROS initialization and lifecycle, with isolated backend I/O."""

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from uuid import uuid4

import numpy as np
import pytest
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from g1_motion_session.arm_ik import ArmIK
from g1_motion_session.arm_ik_mixin import ArmIKMixin
from g1_motion_session.msg import MotionEvent
from g1_motion_session.srv import RegisterTask
from g1_motion_session.task_base import State, TaskBase
from upper_vla import vla_node
from upper_vla.record_observation import record_tool
from upper_vla.vla_backend import ImageSpec, VlaBackend, VlaSpec


class IsolatedBackend(VlaBackend):
    def __init__(self):
        super().__init__(VlaSpec(name='isolated_initialization', images=ImageSpec(slots=())))
        self.infer_calls = 0
        self.closed = False

    def infer(self, observation):
        self.infer_calls += 1
        raise AssertionError('Initialization must not request inference')

    def close(self):
        self.closed = True

    def stats(self):
        return {'initialization_infer_calls': self.infer_calls}


def isolated_arguments(namespace):
    return [
        '--ros-args', '-r', f'__ns:={namespace}', '-r', '__node:=bridge',
        '-r', f'/joint_states:={namespace}/joints',
        '-r', f'/robot_description:={namespace}/description',
        '-r', f'/motion_control/register:={namespace}/register',
        '-r', f'/motion_control/events:={namespace}/events',
        '-r', f'/motion_control/tasks:={namespace}/tasks',
        '-r', f'/tf:={namespace}/tf', '-r', f'/tf_static:={namespace}/tf_static',
        '-p', f'target_topic:={namespace}/targets',
        '-p', f'joint_states_topic:={namespace}/joints',
        '-p', f'robot_description_topic:={namespace}/description',
        '-p', 'stand_s:=0.08', '-p', 'stand_clear_s:=0.02',
        '-p', 'state_timeout_s:=2.0',
        '-p', 'left_wrist_rtsp_url:=disabled', '-p', 'right_wrist_rtsp_url:=disabled',
    ]


def deny_external_io(*args, **kwargs):
    raise AssertionError('External HTTP/camera I/O is forbidden in this test')


class RosHarness:
    def __init__(self, *, child=False):
        self.namespace = f'/vla_init_{uuid4().hex}'
        self.node = None
        self.driver = None
        self.executor = None
        self.registrations = []
        rclpy.init(args=isolated_arguments(self.namespace), domain_id=215,
                   signal_handler_options=SignalHandlerOptions.NO)
        try:
            self.driver = Node('initialization_driver', use_global_arguments=False)
            self.service = self.driver.create_service(
                RegisterTask, f'{self.namespace}/register', self.register)
            if not child:
                self.node = vla_node.VlaBridgeNode()
            hardware = Path(get_package_share_directory('unitree_g1_ros2_control'))
            defaults = yaml.safe_load((hardware / 'config/default_31dof_param.yaml').read_text())
            self.joint_names = defaults['/**']['ros__parameters']['joints']
            self.executor = MultiThreadedExecutor(num_threads=2)
            self.executor.add_node(self.driver)
            if self.node is not None:
                self.executor.add_node(self.node)
            self.description = self.driver.create_publisher(
                String, f'{self.namespace}/description',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.joints = self.driver.create_publisher(
                JointState, f'{self.namespace}/joints',
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.events = self.driver.create_publisher(MotionEvent, f'{self.namespace}/events', 10)
            self.feed_timer = self.driver.create_timer(0.02, self.feed_joints)
        except BaseException:
            self.close()
            raise

    def register(self, request, response):
        self.registrations.append(request)
        response.accepted = True
        return response

    def feed_joints(self):
        driver = self.driver
        assert driver is not None
        message = JointState(name=self.joint_names, position=[0.0] * 31, velocity=[0.0] * 31)
        message.header.stamp = driver.get_clock().now().to_msg()
        self.joints.publish(message)

    def until(self, predicate, timeout=10.0):
        executor = self.executor
        assert executor is not None
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        detail = f'state={self.node.state}, error={self.node._error}' if self.node is not None else self.namespace
        assert predicate(), f'ROS condition timed out; {detail}'

    def close(self):
        try:
            if self.node is not None:
                self.node.shutdown()
            if self.executor is not None:
                assert self.executor.shutdown(timeout_sec=5.0), 'Executor threads did not stop'
                if self.node is not None:
                    self.executor.remove_node(self.node)
                if self.driver is not None:
                    self.executor.remove_node(self.driver)
            if self.node is not None:
                self.node.destroy_node()
                assert not list(self.node.publishers) and not list(self.node.subscriptions)
                assert not self.node._worker.is_alive()
                assert isinstance(self.node._backend, IsolatedBackend)
                assert self.node._backend.closed
            if self.driver is not None:
                self.driver.destroy_node()
        finally:
            rclpy.try_shutdown()


@pytest.fixture
def ros_harness(monkeypatch):
    import requests

    backend = IsolatedBackend()
    monkeypatch.setattr(vla_node, 'load_backend', lambda name, params: backend)
    monkeypatch.setattr(vla_node, 'WristReader', deny_external_io)
    monkeypatch.setattr(requests.sessions.Session, 'request', deny_external_io)
    harness = RosHarness()
    try:
        yield harness
    finally:
        harness.close()


def test_real_node_registers_binds_installed_urdf_and_prepares_without_inference(ros_harness):
    harness = ros_harness
    node = harness.node
    assert isinstance(node, vla_node.VlaBridgeNode)
    assert isinstance(node, Node) and isinstance(node, TaskBase)
    assert isinstance(node, ArmIKMixin)
    assert node.state is State.IDLE and not node.ready
    assert node._readers == {} and node._worker.is_alive()
    harness.until(lambda: node._registered)
    assert len(harness.registrations) == 1
    request = harness.registrations[0]
    assert request.task_id == f'{harness.namespace}/bridge'
    assert request.mask == sum(1 << index for index in range(15, 31))
    assert not request.release
    harness.until(lambda: harness.description.get_subscription_count() == 2)
    description = Path(get_package_share_directory('unitree_g1_description')) / 'model/final.urdf'
    assert description.is_file()
    harness.description.publish(String(data=description.read_text(encoding='utf-8')))
    harness.until(lambda: node.ready and node._model is not None)
    assert isinstance(node._ik, ArmIK)
    assert isinstance(node._model, record_tool('urdf_fk').RobotModel)
    assert node._model is not None
    assert node._ik_seed is not None
    assert node._ik.model.nq == 14
    assert set(node._ik.joint_names) == set(node.joints31[15:29])
    np.testing.assert_array_equal(node.arm_slots, np.arange(15, 29))
    joints = dict(zip(node.joints31, node.output.positions31))
    poses = node._ik.fk(node._ik_seed, clamp=False)
    for side, tip in node.tip_frames.items():
        expected = record_tool('urdf_fk').matrix_to_pose(node._model.poses(node.base_frame, tip, joints)[0])
        np.testing.assert_allclose(poses[side][:3], expected[:3], atol=1e-9)
        assert abs(np.dot(poses[side][3:], expected[3:])) == pytest.approx(1.0, abs=1e-9)
    solved, position_error, orientation_error, _ = node.solve_ik(poses)
    assert solved.shape == (14,) and np.isfinite(solved).all()
    assert position_error < 1e-6 and orientation_error < 1e-6
    harness.until(lambda: node._joint_stamp > 0 and harness.events.get_subscription_count() == 1)
    harness.events.publish(MotionEvent(command='engage', epoch=1, start_time=node.now() + 0.1))
    harness.until(lambda: node.state is State.STAND and node._preparing)
    assert not node._running.is_set() and not node._infer_requested.is_set()
    harness.until(lambda: node.state is State.RUNNING and not node._preparing)
    np.testing.assert_allclose(node.output.positions31, node._stand_goal)
    assert node._ik_seed is not None
    np.testing.assert_allclose(node._ik_seed, node.output.positions31[node.arm_indices])
    assert not node._running.is_set() and not node._infer_requested.is_set()
    assert isinstance(node._backend, IsolatedBackend)
    assert node._backend.infer_calls == 0


def test_real_main_sigint_after_registration_and_preparation_exits_cleanly():
    harness = RosHarness(child=True)
    process = None
    tasks = []
    statuses = []
    try:
        driver = harness.driver
        assert driver is not None

        def on_task(message: String):
            tasks.append(json.loads(message.data))

        def on_status(message: String):
            statuses.append(json.loads(message.data))

        driver.create_subscription(
            String, f'{harness.namespace}/tasks', on_task, 10)
        driver.create_subscription(
            String, f'{harness.namespace}/bridge/status',
            on_status, 10)
        environment = dict(os.environ, ROS_DOMAIN_ID='215', PYTHONFAULTHANDLER='1')
        process = subprocess.Popen(
            [sys.executable, '-u', str(Path(__file__).resolve()), '--isolated-main', harness.namespace],
            env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        child = process
        harness.until(lambda: harness.description.get_subscription_count() == 2
                      or child.poll() is not None, timeout=15.0)
        if process.poll() is not None:
            stdout, stderr = process.communicate(timeout=5.0)
            pytest.fail(f'Child exited before binding: {process.returncode}\n{stdout}\n{stderr}')
        description = Path(get_package_share_directory('unitree_g1_description')) / 'model/final.urdf'
        harness.description.publish(String(data=description.read_text(encoding='utf-8')))
        harness.until(lambda: bool(tasks) and tasks[-1]['ik_ready'] and bool(statuses))
        assert len(harness.registrations) == 1
        request = harness.registrations[0]
        assert request.task_id == f'{harness.namespace}/bridge' and not request.release
        assert request.mask == sum(1 << index for index in range(15, 31))
        assert tasks[-1]['state'] == 'idle'
        harness.until(lambda: harness.events.get_subscription_count() == 1)
        harness.events.publish(MotionEvent(command='engage', epoch=1,
                                           start_time=driver.get_clock().now().nanoseconds * 1e-9 + 0.1))
        harness.until(lambda: tasks[-1]['state'] == 'running' or child.poll() is not None)
        assert process.poll() is None
        assert tasks[-1]['ik_ready'] and tasks[-1]['epoch'] == 1
        assert not statuses[-1]['running'] and not statuses[-1]['inference_active']
        assert statuses[-1]['initialization_infer_calls'] == 0
        process.send_signal(signal.SIGINT)
        stdout, stderr = process.communicate(timeout=10.0)
        assert process.returncode == 0, f'SIGINT exit={process.returncode}\n{stdout}\n{stderr}'
        assert 'ISOLATED_BACKEND_CLOSED infer_calls=0' in stdout
        assert 'Traceback' not in stderr and 'Fatal Python error' not in stderr, stderr
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.communicate(timeout=10.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate(timeout=5.0)
        harness.close()


def isolated_main(namespace):
    import requests

    backend = IsolatedBackend()

    def closed():
        IsolatedBackend.close(backend)
        print(f'ISOLATED_BACKEND_CLOSED infer_calls={backend.infer_calls}', flush=True)

    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(vla_node, 'load_backend', lambda name, params: backend)
        patch.setattr(vla_node, 'WristReader', deny_external_io)
        patch.setattr(requests.sessions.Session, 'request', deny_external_io)
        patch.setattr(backend, 'close', closed)
        vla_node.main(args=isolated_arguments(namespace))


if __name__ == '__main__':
    if len(sys.argv) != 3 or sys.argv[1] != '--isolated-main':
        raise SystemExit('This entry point is only for the isolated pytest child')
    isolated_main(sys.argv[2])
