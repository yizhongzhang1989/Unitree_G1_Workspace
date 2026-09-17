"""Unitree G1 CogACT 请求协议与坐标系。"""

import json
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

from g1_vla_bridge.backends.cogact_unitree import (
    IMAGE_TYPES,
    SPEC,
    CogACTUnitreeBackend,
    build_payload,
    create,
    encode_jpeg,
)
from g1_vla_bridge.control_history import ControlHistory
from g1_vla_bridge.vla_backend import CameraCalibration, Observation


def _observation():
    images = {
        'head': np.zeros((240, 424, 3), np.uint8),
        'left_wrist': np.zeros((360, 640, 3), np.uint8),
        'right_wrist': np.zeros((360, 640, 3), np.uint8),
    }
    calibrations = {
        slot: CameraCalibration(
            intrinsics=(width / 2, height / 2, width / 2, height / 2),
            size=(width, height))
        for slot, image in images.items()
        for height, width in (image.shape[:2],)
    }
    camera_poses = {slot: np.eye(4) for slot in images}
    for matrix in camera_poses.values():
        matrix[2, 3] = .4
    camera_poses['left_wrist'][0, 3] = 0.2
    pose = np.array([0.3, 0.2, 0.1, 0.0, 0.0, 0.0, 1.0])
    return Observation(
        task='Pick Up The Cup', images=images,
        poses={'left': pose, 'right': pose},
        grippers={'left': 1.0, 'right': 1.0},
        enabled={'left': True, 'right': True},
        calibrations=calibrations, camera_poses=camera_poses)


@pytest.mark.parametrize('shape', [(240, 424), (360, 640)])
def test_encode_jpeg_keeps_original_size(shape):
    encoded = encode_jpeg(np.zeros((*shape, 3), np.uint8))
    decoded = cv2.imdecode(np.frombuffer(encoded, np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape[:2] == shape


def test_payload_matches_cogact_raype_contract():
    observation = _observation()
    payload = build_payload(observation, SPEC.frame.transform())
    assert payload['task_description'] == observation.task
    assert payload['return_dict'] is True
    assert payload['image_types'] == list(IMAGE_TYPES)
    assert 'has_left' not in payload and 'has_right' not in payload
    assert 'head_camera_in_world' not in payload
    assert set(payload['state']) == {
        'ROBOT_LEFT_TRANS', 'ROBOT_LEFT_ROT_MAT',
        'ROBOT_RIGHT_TRANS', 'ROBOT_RIGHT_ROT_MAT',
        'ROBOT_LEFT_GRIPPER', 'ROBOT_RIGHT_GRIPPER'}
    for side in ('LEFT', 'RIGHT'):
        for suffix, shape in [('TRANS', (3,)), ('ROT_MAT', (3, 3)), ('GRIPPER', (1,))]:
            assert np.shape(payload['state'][f'ROBOT_{side}_{suffix}']) == shape
    assert np.allclose(payload['intrinsics_per_view'][0],
                       [[0.5, 0, 0.5], [0, 0.5, 0.5], [0, 0, 1]])
    # 与 record/YB 训练数据一致：world_xyz = extrinsic @ camera_xyz。
    assert payload['extrinsics_per_view'][1][0][3] == pytest.approx(0.2)


def test_payload_uses_unified_gripper_orientation():
    payload = build_payload(_observation(), SPEC.frame.transform())
    rotation = np.asarray(payload['state']['ROBOT_LEFT_ROT_MAT'])
    assert np.allclose(rotation, [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-7)


def test_payload_requires_all_camera_calibrations():
    observation = _observation()
    del observation.calibrations['right_wrist']
    with pytest.raises(ValueError, match='三视角标定不完整'):
        build_payload(observation, SPEC.frame.transform())


def test_payload_rejects_camera_info_resolution_mismatch():
    observation = _observation()
    observation.calibrations['head'] = CameraCalibration(
        intrinsics=(1, 1, 1, 1), size=(640, 360))
    with pytest.raises(ValueError, match='分辨率'):
        build_payload(observation, SPEC.frame.transform())


def history_config(rotation='6d'):
    return {'history_action': {'enabled': True, 'num_tokens': 15, 'pose_only': False},
            'history_state': {'enabled': True, 'num_tokens': 15, 'pose_only': False},
            'rotation_type': rotation, 'action_chunk_size': 30, 'action_fps': 10.,
            'action_horizon_seconds': 3., 'history_fps': 10.,
            'history_sample_interval_seconds': .1, 'history_managed_by': 'client',
            'history_resampling': 'client', 'history_order': 'oldest_first',
            'state_include_gripper': True, 'state_space': 'robot', 'action_space': 'robot',
            'translation_action_from_state_type': 'abs',
            'rotation_action_from_state_type': 'abs', 'view_number': 3,
            'training_source_image_size': [640, 360], 'resize_short_edge': 256,
            'history_pairing': 'past_observation_and_corresponding_training_action'}


def configured_backend(monkeypatch, config, status=200):
    session = Mock()
    session.get.return_value.status_code = status
    session.get.return_value.json.side_effect = [
        {'model': 'CogACT', 'status': 'healthy'}, config]
    body = {}
    for side in ('LEFT', 'RIGHT'):
        body[f'ROBOT_{side}_TRANS'] = [[0., 0., 0.]] * 30
        body[f'ROBOT_{side}_ROT_MAT'] = [np.eye(3).tolist()] * 30
        body[f'ROBOT_{side}_GRIPPER'] = [[0.]] * 30
    session.post.return_value.json.return_value = {
        'action': body, 'action_type_info': {'translation': 'abs', 'rotation': 'abs'}}
    monkeypatch.setattr('g1_vla_bridge.backends.cogact_unitree.requests.Session', lambda: session)
    backend = create({'server_url': 'http://model/api/inference'})
    assert [call.args[0] for call in session.get.call_args_list] == [
        'http://model/api/health', 'http://model/api/config']
    return backend, session


@pytest.mark.parametrize('count', [0, 3, 15, 22])
def test_multipart_history_real_steps_only(monkeypatch, count):
    backend, session = configured_backend(monkeypatch, history_config())
    backend.dump = Mock(side_effect=AssertionError('disk writes forbidden'))
    observation = _observation()
    history = ControlHistory()
    for index in range(count):
        action = {side: pose.copy() for side, pose in observation.poses.items()}
        state = {side: pose.copy() for side, pose in observation.poses.items()}
        for side in action:
            action[side][0] = index + 100
            state[side][0] = index
        history.append(
            index / 10 + .01, action, state,
            observation.grippers, observation.grippers, index / 10)
    observation.history = history.snapshot()
    chunk = backend.infer(observation)
    backend.dump.assert_not_called()
    assert chunk.horizon == 30
    files = dict(session.post.call_args.kwargs['files'])
    assert set(files) == {'image_0', 'image_1', 'image_2', 'json'}
    assert files['json'][0] == 'query.json'
    assert files['json'][2] == 'application/json'
    for index in range(3):
        assert cv2.imdecode(np.frombuffer(files[f'image_{index}'][1], np.uint8), 1).shape == (360, 640, 3)
    payload = json.loads(files['json'][1])
    original = build_payload(observation, SPEC.frame.transform())
    assert {key: payload[key] for key in original} == original
    if count == 0:
        assert payload['history_action'] is payload['history_state'] is None
        return
    for field, offset in (('history_action', 100), ('history_state', 0)):
        assert set(payload[field]) == set(original['state'])
        for side in ('LEFT', 'RIGHT'):
            positions = np.array(payload[field][f'ROBOT_{side}_TRANS'])
            rotations = np.array(payload[field][f'ROBOT_{side}_ROT_MAT'])
            assert positions.shape == (min(count, 15), 3)
            assert rotations.shape == (min(count, 15), 3, 3)
            assert np.shape(payload[field][f'ROBOT_{side}_GRIPPER']) == (min(count, 15), 1)
            np.testing.assert_allclose(positions[:, 0], np.arange(max(0, count - 15), count) + offset)
            np.testing.assert_allclose(rotations[0], original['state'][f'ROBOT_{side}_ROT_MAT'])


@pytest.mark.parametrize('config,status', [({}, 200), ({}, 404), ({}, 405),
                         ({'history_action': {'enabled': False},
                           'history_state': {'enabled': False}}, 200)])
def test_old_server_is_rejected(monkeypatch, config, status):
    with pytest.raises(ValueError):
        configured_backend(monkeypatch, config, status)


@pytest.mark.parametrize('field,value', [('enabled', False), ('num_tokens', 30), ('pose_only', True)])
def test_rejects_mismatched_history_contract(monkeypatch, field, value):
    config = history_config()
    config['history_state'][field] = value
    with pytest.raises(ValueError, match='requires both'):
        configured_backend(monkeypatch, config)


def test_configuration_errors_are_not_silently_legacy(monkeypatch):
    session = Mock()
    session.get.side_effect = RuntimeError('unreachable')
    monkeypatch.setattr('g1_vla_bridge.backends.cogact_unitree.requests.Session', lambda: session)
    with pytest.raises(RuntimeError, match='unreachable'):
        create({'server_url': 'http://model/api/inference'})
    session.close.assert_called_once()


def test_reset_does_not_modify_server(monkeypatch):
    backend, session = configured_backend(monkeypatch, history_config())
    assert isinstance(backend, CogACTUnitreeBackend)
    backend.reset()
    session.post.assert_not_called()


@pytest.mark.parametrize('field,value', [('action_fps', 30.), ('history_fps', 30.),
                         ('action_chunk_size', 29), ('action_horizon_seconds', 1.),
                         ('history_order', 'newest_first'), ('state_include_gripper', False),
                         ('rotation_action_from_state_type', 'delta')])
def test_config_mismatch_stops(monkeypatch, field, value):
    config = history_config()
    config[field] = value
    with pytest.raises(ValueError, match='config mismatch'):
        configured_backend(monkeypatch, config)
