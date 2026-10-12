"""Real TaskBase, IK and isolated ROS outputs; no HTTP or robot controllers."""

import json
import time
from pathlib import Path
from typing import TypedDict
from unittest.mock import patch

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from unitree_g1_ros2_control.msg import JointTargets

from g1_motion_session.msg import MotionEvent
from g1_motion_session.srv import RegisterTask
from g1_motion_session.task_base import State
from upper_vr.vr_teleop import VRTeleop

ROOT = Path(__file__).resolve().parents[2]
UPPER_MASK = sum(1 << slot for slot in range(15, 31))


class Grip(TypedDict):
    position: list[float]
    orientation: list[float]


class Hand(TypedDict):
    grip: Grip
    buttons: dict[str, float]
    thumbstick: list[float]


class Frame(TypedDict):
    session_active: bool
    left: Hand
    right: Hand


def wait(executor, predicate):
    deadline = time.monotonic() + 3.0
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    assert predicate()


@pytest.fixture
def graph():
    rclpy.init(args=[], domain_id=217)
    task = observer = executor = None
    try:
        with patch.object(VRTeleop, '_serve', return_value=None):
            task = VRTeleop()
        for timer in (task._registration_timer, *task.timers):
            timer.cancel()
        observer = Node('vr_task_check')
        executor = SingleThreadedExecutor()
        executor.add_node(task)
        executor.add_node(observer)
        targets, velocities, statuses, masks = [], [], [], []
        stream = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        observer.create_subscription(JointTargets, task.target_topic, targets.append, stream)
        observer.create_subscription(Float64MultiArray, '/motion_control/lower/input/vr',
                                     velocities.append, stream)
        observer.create_subscription(String, '/motion_control/tasks', statuses.append, 10)

        def register(request, response):
            masks.append(request.mask)
            response.accepted = request.mask == UPPER_MASK
            return response

        observer.create_service(RegisterTask, '/motion_control/register', register)
        wait(executor, task._register.service_is_ready)
        task._register_task()
        wait(executor, lambda: task._registered)
        assert masks == [UPPER_MASK]
        task._bind_arm_ik(String(data=(ROOT / 'unitree_g1_description/model/final.urdf').read_text()))
        assert task.ik_ready
        feedback = JointState(name=task.joints31, position=task._stand_goal.tolist())
        task._on_joint_states(feedback)
        yield task, executor, targets, velocities, statuses
    finally:
        if executor is not None:
            executor.shutdown()
        if task is not None:
            task.shutdown()
            task.destroy_node()
        if observer is not None:
            observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def engage(task):
    task._on_event(MotionEvent(command='engage', epoch=1, start_time=task.now()))
    task._control()
    assert task._preparing and task.state is State.STAND
    task.stand_start = task.now() - task.stand_s
    task._tick_lifecycle()
    assert task.state is State.RUNNING and not task._preparing


def frame() -> Frame:
    def hand() -> Hand:
        return Hand(grip=Grip(position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0]),
                    buttons=dict(squeeze=1.0, trigger=0.5), thumbstick=[0.0, -1.0])

    return Frame(session_active=True, left=hand(), right=hand())


def test_prepare_enables_vr_direct_fpc(graph):
    task, executor, targets, velocities, statuses = graph
    assert task.get_parameter('arm_rotation_frame').value == 'world'
    assert task.get_parameter('height').value == 0.80
    limits = task.get_parameter('ik_limit_upper').get_parameter_value().double_array_value
    assert limits[3] == 1.4
    assert limits[10] == 1.4
    assert list(task.get_parameter('ik_null_gain').get_parameter_value().double_array_value)[1:5] == [0.05, 0.20, 0.20, 0.20]
    engage(task)
    current = frame()
    task._frame, task._frame_stamp = current, task.now()
    task._control()
    before = task.output.positions.copy()
    current['left']['grip']['position'][2] = -0.02
    task._frame_stamp = task.now()
    task._control()
    expected = task.output.positions.copy()
    assert np.max(np.abs(expected[15:22] - before[15:22])) > 1e-5
    assert np.max(np.abs(expected[15:29] - before[15:29])) <= 10.0 * task.dt + 1e-8
    task._publish_task_status()
    wait(executor, lambda: any(np.allclose(message.positions, expected) for message in targets)
         and velocities and statuses)
    assert all(message.mask == UPPER_MASK for message in targets)
    assert velocities[-1].data[0] == pytest.approx(0.4)
    assert list(task.output.positions[29:]) == pytest.approx([task._grip_open / 2] * 2)
    assert np.array_equal(task._ik_seed, expected[task.arm_indices])
    detail = json.loads(statuses[-1].data)
    assert detail['arms_live'] and detail['state'] == 'running'
    count = len(targets)
    task._frame = None
    task._control()
    assert np.array_equal(task.output.positions, expected)
    task._on_event(MotionEvent(command='stop', epoch=2))
    task._frame, task._frame_stamp = frame(), task.now()
    task._control()
    assert np.array_equal(task.output.positions, expected)
    assert count > 0


@pytest.mark.parametrize('finish', [True, False], ids=['complete', 'cancel'])
@pytest.mark.parametrize('side', ['left', 'right'])
def test_home_side_never_overwritten_and_reanchors(graph, finish, side):
    task, _, _, _, _ = graph
    engage(task)
    current = frame()
    task._frame, task._frame_stamp = current, task.now()
    task._control()
    current[side]['buttons']['squeeze'] = 0.0
    task._start_home(side)
    slots, start, via, stamp = task._homes[side]
    task._homes[side] = slots, start, via, stamp - task.home_s / 2
    task._tick_homes()
    assert task.output.message.mask == UPPER_MASK
    np.testing.assert_array_equal(task._ik_seed, task.output.positions[task.arm_indices])
    home = task.output.positions[slots].copy()
    current[side]['grip']['position'][2] = -0.04
    task._pose[side][2] -= 0.04
    task._send(left=task._pose['left'], right=task._pose['right'], grip=task._grip)
    np.testing.assert_array_equal(task.output.positions[slots], home)
    assert side in task._homes
    np.testing.assert_array_equal(task._ik_seed, task.output.positions[task.arm_indices])
    if finish:
        task._homes[side] = slots, start, via, task.now() - task.home_s
        task._tick_homes()
    else:
        task._cancel_home('test')
    target = task.output.positions.copy()
    reached = task.status_ik()['limited_pose'][side]
    task._control()
    np.testing.assert_allclose(task._pose[side], reached)
    assert np.max(np.abs(task.output.positions[slots] - target[slots])) < 0.01
