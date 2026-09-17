from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from g1_vla_bridge.vla_node import HOME_POSES, VlaBridgeNode
from test_execution_regression import executor_fixture


def home_node(mode='manual'):
    node = executor_fixture(mode)
    node._base_frame = 'torso_link'
    node._tip_frames = {'left': 'left_gripper_base', 'right': 'right_gripper_base'}
    node._infer_requested.set()
    node._inference_active = True
    node._timed = object()
    node.get_logger = Mock(return_value=Mock())
    node._stop = Mock(side_effect=lambda reason: VlaBridgeNode._stop(node, reason))
    return node


@pytest.mark.parametrize('mode', ['manual', 'continuous', 'async'])
@pytest.mark.parametrize('running', [False, True])
def test_home_immediately_replaces_both_targets_and_invalidates_playback(mode, running):
    node = home_node(mode)
    if not running:
        node._running.clear()
    node._active['right'] = False
    node._grip_command = {'left': .4, 'right': .7}
    generation = node._generation
    old_chunk = node._chunk
    response = VlaBridgeNode._on_home(node, None, SimpleNamespace())
    assert response.success
    node._publisher.publish.assert_called_once()
    sent = node._publisher.publish.call_args.args[0].data
    np.testing.assert_allclose(sent, [*HOME_POSES['left'], *HOME_POSES['right']])
    assert len(sent) == 14
    assert node._grip_command == {'left': .4, 'right': .7}
    assert not node._running.is_set()
    assert not node._infer_requested.is_set()
    assert not node._inference_active
    assert node._chunk is None and node._timed is None and node._cursor == 0
    assert node._generation > generation
    node._stop.assert_called_once_with('收到 ~/home')
    VlaBridgeNode._accept(node, old_chunk, 10., generation)
    VlaBridgeNode._on_tick(node)
    node._publisher.publish.assert_called_once()
    node._request_inference.assert_not_called()


@pytest.mark.parametrize('problem', ['arms', 'base', 'tip'])
def test_home_rejection_never_publishes(problem):
    node = home_node()
    if problem == 'arms':
        node._arms_ready = Mock(return_value='not engaged')
    elif problem == 'base':
        node._base_frame = 'pelvis'
    else:
        node._tip_frames['right'] = 'right_wrist_yaw_link'
    response = VlaBridgeNode._on_home(node, None, SimpleNamespace())
    assert not response.success
    node._publisher.publish.assert_not_called()
    assert node._running.is_set()


def test_home_publish_failure_keeps_vla_stopped():
    node = home_node()
    node._publisher.publish.side_effect = RuntimeError('publisher failed')
    response = VlaBridgeNode._on_home(node, None, SimpleNamespace())
    assert not response.success
    assert 'publisher failed' in response.message
    assert not node._running.is_set()
    assert node._chunk is None and node._timed is None
    node._stop.assert_called_once_with('收到 ~/home')


@pytest.mark.parametrize('running', [False, True])
def test_home_clears_control_history(running):
    node = home_node()
    node._history = Mock()
    if not running:
        node._running.clear()
    response = VlaBridgeNode._on_home(node, None, SimpleNamespace())
    assert response.success
    node._history.clear.assert_called_once_with()
