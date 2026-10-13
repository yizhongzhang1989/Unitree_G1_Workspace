"""Isolated registration, measured 14-arm FPC output and gripper hold checks."""

import time

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from unitree_g1_ros2_control.msg import JointTargets

from g1_motion_session.msg import MotionEvent
from g1_motion_session.srv import RegisterTask
from g1_motion_session.task_base import State
from upper_float.float_node import FloatTask

UPPER_MASK = sum(1 << slot for slot in range(15, 31))


def wait(executor, predicate):
    deadline = time.monotonic() + 3.0
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    assert predicate()


@pytest.mark.parametrize(('ros_args', 'gripper_source', 'gripper_topic'), [
    ([], 'keyboard', '/motion_control/upper/input/keyboard'),
    (['--ros-args', '-p', 'gripper_source:=vr'], 'vr', '/motion_control/upper/input/vr'),
    (['--ros-args', '-p', 'gripper_source:=custom', '-p',
      'gripper_command_topic:=/upper_float/test/gripper'], 'custom', '/upper_float/test/gripper'),
])
def test_measured_arms_direct_fpc_with_gripper_hold_and_home(ros_args, gripper_source, gripper_topic):
    rclpy.init(args=ros_args, domain_id=217)
    task = observer = executor = None
    try:
        task = FloatTask()
        assert task.get_parameter('gripper_command_topic').value == gripper_topic
        assert task.status()['gripper_source'] == gripper_source
        for timer in task.timers:
            timer.cancel()
        observer = Node('float_task_check')
        executor = SingleThreadedExecutor()
        executor.add_node(task)
        executor.add_node(observer)
        targets, masks = [], []
        stream = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        observer.create_subscription(JointTargets, task.target_topic, targets.append, stream)
        freeze = observer.create_client(Trigger, '/upper_float/freeze')
        unfreeze = observer.create_client(Trigger, '/upper_float/unfreeze')

        def call(client):
            wait(executor, client.service_is_ready)
            future = client.call_async(Trigger.Request())
            wait(executor, future.done)
            response = future.result()
            assert isinstance(response, Trigger.Response)
            return response

        def register(request, response):
            masks.append(request.mask)
            response.accepted = request.mask == UPPER_MASK
            return response

        observer.create_service(RegisterTask, '/motion_control/register', register)
        wait(executor, task._register.service_is_ready)
        task._register_task()
        wait(executor, lambda: task._registered)
        assert masks == [UPPER_MASK]
        measured = np.linspace(-0.3, 0.3, 31)
        feedback = observer.create_publisher(JointState, '/joint_states', stream)
        gripper = observer.create_publisher(
            Float64MultiArray,
            task.get_parameter('gripper_command_topic').get_parameter_value().string_value,
            stream)
        wait(executor, lambda: feedback.get_subscription_count() > 0
             and gripper.get_subscription_count() > 0)
        feedback.publish(JointState(name=task.joints31, position=measured.tolist()))
        wait(executor, lambda: task._joint_stamp > 0.0)
        task._on_event(MotionEvent(command='engage', epoch=1, start_time=task.now()))
        task._control()
        assert task.state is State.STAND and task._preparing
        assert not call(freeze).success
        assert not call(unfreeze).success
        np.testing.assert_array_equal(task.output.positions, measured)
        task.stand_start = task.now() - task.stand_s
        task._tick_lifecycle()
        assert task.state is State.RUNNING
        gripper.publish(Float64MultiArray(data=[-1.0, 5.0]))
        wait(executor, lambda: task.output.positions[30] == pytest.approx(task._grip_hi))
        task._control()
        expected = task.output.positions.copy()
        wait(executor, lambda: any(np.array_equal(message.positions, expected) for message in targets))
        np.testing.assert_array_equal(expected[15:29], measured[15:29])
        assert all(message.mask == UPPER_MASK for message in targets)
        assert expected[29] == 0.0 and expected[30] == task._grip_hi
        assert task.status()['upper_source'] == 'none'
        for data in ([], [1.0], [1.0, 1.0, 1.0],
                     [float('nan'), 1.0], [1.0, float('inf')], [float('-inf'), 1.0]):
            task._on_gripper(Float64MultiArray(data=data))
            np.testing.assert_array_equal(task.output.positions, expected)

        posed = measured + 0.15
        feedback.publish(JointState(name=task.joints31, position=posed.tolist()))
        wait(executor, lambda: np.array_equal(task._q, posed))
        assert call(freeze).success
        assert task.status()['frozen'] is True
        np.testing.assert_array_equal(task.output.positions[15:29], posed[15:29])
        moved = posed + 0.2
        feedback.publish(JointState(name=task.joints31, position=moved.tolist()))
        wait(executor, lambda: np.array_equal(task._q, moved))
        assert call(freeze).success
        task._on_gripper(Float64MultiArray(data=[1.0, 1.0]))
        task._control()
        frozen_target = task.output.positions.copy()
        np.testing.assert_array_equal(frozen_target[15:29], posed[15:29])
        np.testing.assert_array_equal(frozen_target[29:], [1.0, 1.0])
        wait(executor, lambda: any(np.array_equal(message.positions, frozen_target) for message in targets))
        assert call(unfreeze).success
        assert call(unfreeze).success
        assert task.status()['frozen'] is False
        task._control()
        resumed_target = task.output.positions.copy()
        np.testing.assert_array_equal(resumed_target[15:29], moved[15:29])
        wait(executor, lambda: any(np.array_equal(message.positions, resumed_target) for message in targets))
        task._joint_stamp = time.monotonic() - task._state_timeout - 1.0
        assert not call(freeze).success
        assert task.status()['frozen'] is False
        task._joint_stamp = time.monotonic()
        task._q[15] = float('nan')
        assert not call(freeze).success
        task._q[:] = moved
        assert call(freeze).success
        task._on_event(MotionEvent(command='stop', epoch=2))
        assert not call(freeze).success
        assert not call(unfreeze).success
        held = task.output.positions.copy()
        task._q[:] += 0.1
        task._control()
        task._on_gripper(Float64MultiArray(data=[1.0, 1.0]))
        np.testing.assert_array_equal(task.output.positions, held)
        task._on_event(MotionEvent(command='engage', epoch=3, start_time=task.now()))
        task.stand_start = task.now() - task.stand_s
        task._tick_lifecycle()
        assert task.state is State.RUNNING and task.status()['frozen'] is False
    finally:
        if executor is not None:
            executor.shutdown()
        if task is not None:
            task.destroy_node()
        if observer is not None:
            observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
