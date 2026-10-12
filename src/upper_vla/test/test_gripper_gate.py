
from pathlib import Path
import runpy
from unittest.mock import Mock

import numpy as np
import pytest
from std_srvs.srv import SetBool

from upper_vla.backends.cogact_unitree import SPEC
from upper_vla.gripper_gate_node import GripperGateNode, retained_prefix
from upper_vla.vla_backend import ActionChunk, SIDES
from upper_vla.vla_node import VlaBridgeNode
import test_async_execution


class GateTestNode(GripperGateNode):
    publish_control_spy: Mock


def prediction(horizon=30):
    poses = np.tile([0., 0., 0., 0., 0., 0., 1.], (horizon, 1))
    poses[:, 0] = np.arange(horizon) * .001
    return ActionChunk(
        poses={side: poses.copy() for side in SIDES},
        grippers={side: SPEC.gripper.to_robot(np.zeros(horizon)) for side in SIDES})


@pytest.mark.parametrize('side', SIDES)
@pytest.mark.parametrize('index', [15, 20, 29])
@pytest.mark.parametrize('before,after', [
    (0.75, 0.74), (0.9, 0.6), (0.25, 0.26), (0.1, 0.4), (0., 1.), (1., 0.)])
def test_tail_transition_excludes_changed_action(side, index, before, after):
    chunk = prediction()
    chunk.grippers[side][:] = SPEC.gripper.to_robot(before)
    chunk.grippers[side][index:] = SPEC.gripper.to_robot(after)
    assert retained_prefix(chunk, SPEC.gripper, dict.fromkeys(SIDES, True),
                           dict.fromkeys(SIDES, float(SPEC.gripper.to_robot(before)))) == index


@pytest.mark.parametrize('side', SIDES)
@pytest.mark.parametrize('before,after', [
    (0.74, 0.75), (0.8, 0.75), (0.26, 0.25), (0.2, 0.25),
    (0.4, 0.6), (0.6, 0.4), (0.25, 0.25), (0.75, 0.75)])
def test_other_transitions_do_not_truncate(side, before, after):
    chunk = prediction()
    chunk.grippers[side][:] = SPEC.gripper.to_robot(before)
    chunk.grippers[side][20:] = SPEC.gripper.to_robot(after)
    assert retained_prefix(chunk, SPEC.gripper, dict.fromkeys(SIDES, True),
                           dict.fromkeys(SIDES, float(SPEC.gripper.to_robot(before)))) == 30


def test_early_transition_and_inactive_side_do_not_truncate():
    chunk = prediction()
    chunk.grippers['left'][14:] = SPEC.gripper.to_robot(1.)
    chunk.grippers['right'][20:] = SPEC.gripper.to_robot(1.)
    assert retained_prefix(chunk, SPEC.gripper, {'left': True, 'right': False},
                           dict.fromkeys(SIDES, float(SPEC.gripper.to_robot(0.)))) == 30


@pytest.fixture
def gate(monkeypatch):
    original, _ = test_async_execution.bridge.__wrapped__(monkeypatch)
    node = GateTestNode.__new__(GateTestNode)
    node.__dict__.update(original.__dict__)
    node._execution_mode = 'manual'
    node._action_rate = 10.
    node._execution_rate = 30.
    node._grip_command = dict.fromkeys(SIDES, float(SPEC.gripper.to_robot(0.)))
    node._timed = None
    monkeypatch.setattr(node, '_request_inference',
                        lambda generation=None: VlaBridgeNode._request_inference(node, generation))
    del node._publish_control
    node.publish_control_spy = Mock(wraps=node._publish_control)
    monkeypatch.setattr(node, '_publish_control', node.publish_control_spy)
    return node


def test_publication_never_interpolates_toward_discarded_action(gate):
    chunk = prediction()
    chunk.grippers['left'][20:] = SPEC.gripper.to_robot(1.)
    chunk.poses['left'][20:, 0] = 10.
    gate._inference_active = True
    GripperGateNode._accept(gate, chunk, 20., gate._generation)
    assert gate._chunk is not None
    assert gate._chunk.horizon == 20
    for _ in range(95):
        VlaBridgeNode._on_tick(gate)
    messages = [call.args[0] for call in gate.publish_control_spy.call_args_list]
    poses = [command['left'] for command in messages]
    grips = [call.args[1] for call in gate.publish_control_spy.call_args_list]
    assert len(poses) == 60
    assert max(pose[0] for pose in poses) == pytest.approx(.019)
    assert poses[-1][0] == pytest.approx(.019)
    assert all(grip['left'] == pytest.approx(SPEC.gripper.to_robot(0.)) for grip in grips)
    assert gate._chunk is None
    assert gate._running.is_set()
    assert not gate._infer_requested.is_set()
    assert gate._request_inference() == ''


def test_first_action_transition_waits_without_publication(gate):
    chunk = prediction(10)
    chunk.grippers['left'][:] = SPEC.gripper.to_robot(1.)
    gate._inference_active = True
    GripperGateNode._accept(gate, chunk, 20., gate._generation)
    VlaBridgeNode._on_tick(gate)
    gate.publish_control_spy.assert_not_called()
    assert not gate._inference_active
    assert gate._running.is_set()
    assert gate._request_inference() == ''


@pytest.mark.parametrize('mode', ['async', 'continuous'])
def test_automatic_modes_rejected(gate, mode):
    assert GripperGateNode._mode_error(gate, mode)


def test_skip_rejected(gate):
    response = GripperGateNode._on_set_skip_intermediate(
        gate, SetBool.Request(data=True), SetBool.Response())
    assert not response.success


def test_pause_keeps_history_and_pending_controls(gate):
    history = Mock()
    gate._history = history
    pending = (1., dict(gate._command), dict(gate._grip_command))
    gate._pending_control.append(pending)
    chunk = prediction(10)
    chunk.grippers['left'][:] = SPEC.gripper.to_robot(1.)
    GripperGateNode._accept(gate, chunk, 20., gate._generation)
    history.clear.assert_not_called()
    assert list(gate._pending_control) == [pending]


def test_stale_response_cannot_replace_current_chunk(gate):
    current = prediction()
    gate._chunk = current
    GripperGateNode._accept(gate, prediction(), 20., gate._generation - 1)
    assert gate._chunk is current


def test_no_transition_executes_full_chunk(gate):
    GripperGateNode._accept(gate, prediction(), 20., gate._generation)
    for _ in range(95):
        VlaBridgeNode._on_tick(gate)
    assert gate.publish_control_spy.call_count == 90
    assert gate._chunk is None


def test_experiment_launch_selects_node_and_preserves_proxy(monkeypatch):
    import ament_index_python.packages
    import g1_motion_session.launch_support
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration

    directory = Path(__file__).resolve().parents[1] / 'launch'

    def package_directory(name):
        return str(directory.parent.parent / name)

    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory', package_directory)
    monkeypatch.setattr(g1_motion_session.launch_support, 'get_package_share_directory', package_directory)
    experiment = runpy.run_path(str(directory / 'gripper_gate.launch.py'))
    common = runpy.run_path(str(directory / 'vla_bridge.launch.py'))
    context = LaunchContext()
    context.launch_configurations['proxy'] = 'socks5h://127.0.0.1:1080'
    include = experiment['generate_launch_description']().entities[0]
    for action in include.execute(context):
        if isinstance(action, SetLaunchConfiguration):
            action.execute(context)
    for action in common['generate_launch_description']().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    factory = Mock()
    monkeypatch.setitem(common['_node'].__globals__, 'Node', factory)
    common['_node'](context)
    arguments = factory.call_args.kwargs
    assert arguments['executable'].perform(context) == 'gripper_gate_node'
    assert arguments['name'] == 'vla_bridge'
    assert arguments['parameters'][-1]['execution_mode'] == 'manual'
    assert arguments['parameters'][-1]['skip_intermediate_waypoints'] is False
    assert arguments['parameters'][-1]['proxy'] == 'socks5h://127.0.0.1:1080'
