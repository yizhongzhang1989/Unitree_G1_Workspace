"""Service handlers tested with a disabled headset server in an isolated ROS domain."""

import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest
import rclpy
import yaml
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from g1_mocap.capture_stream import CaptureStream
from g1_mocap.motion_capture_node import MotionCaptureNode
from g1_mocap.skeleton import ControllerState
from test_stream import payload


ROOT = Path(__file__).resolve().parents[2]
CONFIG = yaml.safe_load((ROOT / 'g1_mocap/config/mocap.yaml').read_text())['/mocap']['ros__parameters']


@pytest.fixture
def node(tmp_path):
    parameters = dict(CONFIG)
    parameters.update(output_dir=str(tmp_path), model_confirmed=True,
                      urdf_path=str(ROOT / 'unitree_g1_description/model/g1_description/g1_29dof_mode_15.urdf'))
    rclpy.init()
    original = rclpy.node.Node.__init__

    def initialize(instance, name):
        overrides = [Parameter(key, value=value) for key, value in parameters.items()]
        original(instance, name, parameter_overrides=overrides)

    with patch.object(rclpy.node.Node, '__init__', initialize), \
            patch.object(CaptureStream, 'start'), patch.object(CaptureStream, 'stop'):
        instance = MotionCaptureNode()
        try:
            yield instance
        finally:
            instance.destroy_node()
            rclpy.shutdown()


def start_and_fill(node, *, root_offset=0.0):
    node.stream._device = SimpleNamespace(closed=False)
    node.stream._calibration = object()
    node.stream.last_valid_arrival = time.monotonic()
    response = node._start(None, Trigger.Response())
    assert response.success, response.message
    joints = np.asarray(CONFIG['default_joint_pos'])
    height = node.stream._retarget.stand_height + root_offset
    row = np.r_[[0, 0, height], [0, 0, 0, 1], joints]
    for stamp in np.arange(271) / 90:
        node.stream.clip.append(stamp, row, id(node.stream.calibration))
    node.stream.last_valid_arrival = time.monotonic()


def test_services_save_one_valid_take(node, tmp_path):
    start_and_fill(node)
    assert not node._start(None, Trigger.Response()).success
    response = node._stop(None, Trigger.Response())
    assert response.success, response.message
    files = list((tmp_path / 'motions').glob('*.csv'))
    assert len(files) == 1
    assert np.loadtxt(files[0], delimiter=',').shape == (151, 36)
    assert not node._stop(None, Trigger.Response()).success


def test_ground_penetration_is_saved_for_offline_refinement(node, tmp_path):
    start_and_fill(node, root_offset=-0.5)
    response = node._stop(None, Trigger.Response())
    assert response.success, response.message
    files = list((tmp_path / 'motions').glob('*.csv'))
    assert len(files) == 1
    assert np.loadtxt(files[0], delimiter=',').shape == (151, 36)


def test_airborne_take_is_allowed_without_profile(node, tmp_path):
    assert not node.has_parameter('ground_profile')
    assert node.max_duration == 60.0
    start_and_fill(node, root_offset=0.5)
    response = node._stop(None, Trigger.Response())
    assert response.success, response.message
    assert len(list((tmp_path / 'motions').glob('*.csv'))) == 1


def test_discard_and_timeout_never_save(node, tmp_path):
    start_and_fill(node)
    assert node._discard(None, Trigger.Response()).success
    start_and_fill(node)
    node.stream.last_valid_arrival = time.monotonic() - 1
    node._tick()
    assert node.stream.clip is None
    assert 'timeout' in node.last_outcome
    assert not list(tmp_path.iterdir())


def test_model_confirmation_required(node):
    node.model_confirmed = False
    response = node._start(None, Trigger.Response())
    assert not response.success and 'model' in response.message


def test_shutdown_does_not_save(node, tmp_path):
    start_and_fill(node)
    assert not list(tmp_path.iterdir())


def test_auto_stop_uses_completed_source(node, tmp_path):
    start_and_fill(node)
    node.stream.complete = True
    node.stream.last_valid_arrival = time.monotonic() - 1
    node._tick()
    assert node.stream.clip is None
    assert len(list((tmp_path / 'motions').glob('*.csv'))) == 1


@pytest.mark.parametrize('defect', ['invalid', 'orientation'])
def test_real_ingest_recovery_cannot_save_failed_take(node, tmp_path, defect):
    start_and_fill(node)
    row = node.stream.clip.rows[-1]
    solved = SimpleNamespace(root_pos=row[:3], root_quat=row[[6, 3, 4, 5]],
                             joint_pos=row[7:])
    node.stream.on_preview = None
    with patch.object(node.stream._retarget, 'solve', return_value=solved):
        for seq in (1, 2, 3):
            message = payload(np.zeros((24, 3)))
            message.update(seq=seq, t=3.0 + seq / 90)
            if seq == 2:
                if defect == 'invalid':
                    message['body']['status'] = 0
                else:
                    message['body']['joints']['LEFT_WRIST']['orientation_valid'] = False
            node.stream._ingest(message)
    response = node._stop(None, Trigger.Response())
    assert not response.success and 'Tracking' in response.message
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('button', ['a_x', 'b_y'])
def test_stale_controller_command_does_not_touch_next_take(node, button):
    start_and_fill(node)
    node._controllers((ControllerState(), ControllerState(connected=True, **{button: True})))
    node._discard(None, Trigger.Response())
    start_and_fill(node)
    current = node.stream.clip
    node._tick()
    assert node.stream.clip is current


def test_queued_stop_cannot_restart_after_discard(node):
    start_and_fill(node)
    node._controllers((ControllerState(), ControllerState(connected=True, a_x=True)))
    node._discard(None, Trigger.Response())
    node._tick()
    assert node.stream.clip is None


def test_stop_button_seals_take_before_reset_and_timer(node, tmp_path):
    start_and_fill(node)
    clip = node.stream.clip
    rows_before = np.asarray(clip.rows).copy()
    row = rows_before[-1]
    solved = SimpleNamespace(root_pos=row[:3] + [10.0, 0.0, 0.0],
                             root_quat=row[[6, 3, 4, 5]], joint_pos=row[7:])
    node.stream.on_preview = None
    controllers = (ControllerState(), ControllerState(connected=True, a_x=True))
    with patch('g1_mocap.stream.parse_controllers', return_value=controllers), \
            patch.object(node.stream._retarget, 'solve', return_value=solved):
        for seq in (1, 2):
            message = payload(np.zeros((24, 3)))
            message.update(seq=seq, t=3.0 + seq / 90)
            node.stream._ingest(message)
    assert not clip.reason
    np.testing.assert_array_equal(clip.rows, rows_before)
    node._tick()
    assert node.stream.clip is None
    assert node.last_outcome.startswith('Saved ')
    saved = list((tmp_path / 'motions').glob('*.csv'))
    assert len(saved) == 1
    np.testing.assert_allclose(np.loadtxt(saved[0], delimiter=',')[:, 0], 0.0)


def test_reset_while_idle_does_not_report_recording_error(node):
    start_and_fill(node)
    node._discard(None, Trigger.Response())
    outcome = node.last_outcome
    node.stream.on_preview = None
    solved = SimpleNamespace(root_pos=np.array([100.0, 0.0, 0.8]),
                             root_quat=np.array([1.0, 0.0, 0.0, 0.0]), joint_pos=np.zeros(29))
    with patch.object(node.stream._retarget, 'solve', return_value=solved):
        node.stream._ingest(payload(np.zeros((24, 3))))
    node._tick()
    assert node.stream.clip is None
    assert node.last_outcome == outcome


def test_stop_button_does_not_clear_jump_during_recording(node, tmp_path):
    start_and_fill(node)
    clip = node.stream.clip
    row = clip.rows[-1]
    solved = SimpleNamespace(root_pos=row[:3] + [10.0, 0.0, 0.0],
                             root_quat=row[[6, 3, 4, 5]], joint_pos=row[7:])
    node.stream.on_preview = None
    message = payload(np.zeros((24, 3)))
    message.update(seq=1, t=3.0 + 1 / 90)
    with patch.object(node.stream._retarget, 'solve', return_value=solved):
        node.stream._ingest(message)
    assert clip.reason == 'Root translation jump'
    node._controllers((ControllerState(), ControllerState(connected=True, a_x=True)))
    node._tick()
    assert node.stream.clip is None
    assert node.last_outcome == 'Not saved: Root translation jump'
    assert not list(tmp_path.iterdir())
