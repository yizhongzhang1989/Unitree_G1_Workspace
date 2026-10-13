import math
from importlib import import_module
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

State = pytest.importorskip('g1_motion_session.task_base').State
FootstepTask = import_module('lower_footstep_tracking.footstep_task').FootstepTask


@pytest.fixture
def task():
    task = FootstepTask.__new__(FootstepTask)
    task.execution_lock = threading.RLock()
    task.lock = threading.RLock()
    task.runtime = Mock()
    task.runtime.ready = False
    task.policy_slots = list(range(15))
    task.dt = .02
    task.waist_output_cutoff_hz = 10.
    tangent = math.tan(math.pi * task.waist_output_cutoff_hz * task.dt)
    task._waist_b = tangent / (1. + tangent)
    task._waist_a = (tangent - 1.) / (1. + tangent)
    task._waist_slots = [12, 13, 14]
    task._waist_previous_input = np.zeros(3)
    task._waist_previous_output = np.zeros(3)
    task.joints31 = [f'joint_{index}' for index in range(31)]
    task.joints31[16] = 'left_shoulder_roll_joint'
    task.joints31[23] = 'right_shoulder_roll_joint'
    task._stand_goal = np.zeros(31)
    task._stand_goal[:15] = .2
    task._dq = np.zeros(31)
    task._quat, task._omega = (0., 0., 0., 1.), (0., 0., 0.)
    task._registered, task._preparing = True, True
    task._publish_task_status = Mock()
    task._stopped = True
    task.state = State.STAND
    task.generation = 1
    task.stand_start = 0.
    task.clear_s = .4
    task.clear_roll = .7
    task.stand_s = 2.5
    task.now = lambda: 3.
    task.snapshot = Mock(return_value=SimpleNamespace(position=np.zeros(31)))
    task.stale_reason = lambda: ''
    task.estop = Mock()
    task.output = SimpleNamespace(positions=np.zeros(31), publish=Mock())
    return task


def test_stand_holds_until_start_and_stop_ceases_output(task):
    task.runtime.ready = True
    task.engage(np.zeros(31))
    task._tick_lifecycle()
    task.control()
    np.testing.assert_array_equal(task.output.positions[:15], task._stand_goal[:15])
    task._tick_lifecycle()
    task.control()
    assert task.output.publish.call_count == 1
    task.runtime.step.assert_not_called()
    snapshot = task.snapshot()
    task.start(snapshot)
    task.runtime.reset.assert_called_once_with(snapshot, 3.)
    task.state = State.RUNNING
    task.runtime.step.return_value = np.full(15, .3)
    task.control()
    np.testing.assert_array_equal(task.output.positions[:12], np.full(12, .3))
    np.testing.assert_allclose(task.output.positions[12:15], task._waist_b * .3)
    np.testing.assert_array_equal(task.output.positions[15:], np.zeros(16))
    task.stop()
    task.control()
    assert task.output.publish.call_count == 2
    task.runtime.step.assert_called_once()


@pytest.mark.parametrize('enabled', [False, True])
def test_waist_output_lowpass_frequency_response_and_restart(task, enabled):
    task.waist_output_cutoff_hz = 10. if enabled else 0.
    state = SimpleNamespace(position=np.full(31, .2))
    task.snapshot.return_value = state
    task.output.positions[15:] = .7
    task.runtime.step.return_value = np.full(15, .2)
    task.start(state)
    task.state = State.RUNNING
    task.control()
    np.testing.assert_allclose(task.output.positions[:15], .2, atol=1e-15)
    for frequency in (1., 10., 20.):
        task.stop()
        task.start(state)
        time = np.arange(600) * task.dt
        signal = .05 * np.sin(math.tau * frequency * time[:, None] + np.array([0., .4, .8]))
        measured = []
        for value in signal:
            target = np.r_[np.full(12, .2 + value[0]), .2 + value]
            task.runtime.step.return_value = target
            task.control()
            np.testing.assert_array_equal(task.output.positions[:12], target[:12])
            np.testing.assert_array_equal(task.output.positions[15:], np.full(16, .7))
            if not enabled:
                np.testing.assert_array_equal(task.output.positions[:15], target)
            measured.append(task.output.positions[12:15].copy() - .2)
        gain = np.sqrt(np.mean(np.square(measured[100:]), axis=0)
                       / np.mean(np.square(signal[100:]), axis=0))
        ratio = math.tan(math.pi * frequency * task.dt) / math.tan(math.pi * 10. * task.dt)
        expected = 1. / math.sqrt(1. + ratio ** 2) if enabled else 1.
        np.testing.assert_allclose(gain, expected, atol=1e-12)
    task.stop()
    task.runtime.controller = object()
    task.runtime.reset.reset_mock()
    import_module('lower_footstep_tracking.footstep_web').FootstepWebTask.start(task, state)
    task.runtime.reset.assert_not_called()
    task.runtime.step.return_value = np.full(15, .2)
    task.control()
    np.testing.assert_allclose(task.output.positions[:15], .2, atol=1e-15)
    task.estop.assert_not_called()
