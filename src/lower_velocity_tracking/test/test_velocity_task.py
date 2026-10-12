from pathlib import Path
from importlib import import_module
import runpy
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

State = pytest.importorskip('g1_motion_session.task_base').State
VelocityTask = import_module('lower_velocity_tracking.velocity_task').VelocityTask


def task_fixture():
    task = VelocityTask.__new__(VelocityTask)
    task.execution_lock = threading.RLock()
    task.lock = threading.RLock()
    task.runtime = Mock()
    task.policy_slots = list(range(15))
    task.joints31 = [f'joint_{index}' for index in range(31)]
    task.joints31[16] = 'left_shoulder_roll_joint'
    task.joints31[23] = 'right_shoulder_roll_joint'
    task._stand_goal = np.zeros(31)
    task._stand_goal[:15] = .2
    task._dq = np.zeros(31)
    task._quat, task._omega = (0., 0., 0., 1.), (0., 0., 0.)
    task._registered, task._preparing = True, True
    task._homes = {}
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


def test_velocity_ros_adapter_and_local_input_use_same_runtime():
    task = task_fixture()
    task.on_command(SimpleNamespace(data=[.2, .1, .3, .74]))
    first = task.runtime.set_command.call_args.args
    task.set_command([.2, .1, .3, .74])
    second = task.runtime.set_command.call_args.args
    np.testing.assert_array_equal(first[0], second[0])
    assert first[1] == second[1] == 3.


def test_stand_holds_until_start_and_stop_ceases_output():
    task = task_fixture()
    positions = np.arange(31, dtype=float)
    task.engage(positions)
    task._tick_lifecycle()
    task.control()
    np.testing.assert_allclose(task.output.positions[:15], task._stand_goal[:15], atol=1e-14)
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
    np.testing.assert_array_equal(task.output.positions[:15], np.full(15, .3))
    np.testing.assert_array_equal(task.output.positions[15:], task._stand_goal[15:])
    task.stop()
    task.control()
    assert task.output.publish.call_count == 2
    task.runtime.step.assert_called_once()


def test_publish_discards_cancelled_generation():
    task = task_fixture()
    task.start(task.snapshot())
    task.state = State.RUNNING

    def cancel(*args):
        task.generation += 1
        return np.zeros(15)

    task.runtime.step.side_effect = cancel
    task.control()
    task.output.publish.assert_not_called()


def test_preparation_waits_for_shared_start_time():
    task = task_fixture()
    task.engage(np.ones(31))
    task.stand_start = 3.05
    task._tick_lifecycle()
    task.control()
    task.output.publish.assert_not_called()
    np.testing.assert_array_equal(task.output.positions[:15], np.ones(15))


def test_launch_uses_task_parameters_and_single_threaded_blas(monkeypatch):
    pytest.importorskip('g1_motion_session.launch_support')
    from launch import LaunchContext

    launch = runpy.run_path(str(Path(__file__).parents[1] / 'launch/velocity_tracking.launch.py'))
    factory, parameters = Mock(), Mock(return_value=['/tmp/lower.yaml'])
    monkeypatch.setitem(launch['_nodes'].__globals__, 'Node', factory)
    monkeypatch.setitem(launch['_nodes'].__globals__, 'task_parameters', parameters)
    context = LaunchContext()
    context.launch_configurations['config'] = '/tmp/lower.yaml'
    launch['_nodes'](context)
    parameters.assert_called_once_with('lower_velocity_tracking', '/tmp/lower.yaml')
    assert factory.call_args.kwargs['parameters'] == ['/tmp/lower.yaml']
    assert factory.call_args.kwargs['additional_env'] == {
        'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}
