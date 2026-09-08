"""Unitree G1 CogACT 请求协议与坐标系。"""

import cv2
import numpy as np
import pytest

from g1_vla_bridge.backends.cogact_unitree import (
    IMAGE_TYPES,
    SPEC,
    build_payload,
    encode_jpeg,
)
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
    assert payload['task_description'] == 'pick up the cup'
    assert payload['image_types'] == list(IMAGE_TYPES)
    assert 'has_left' not in payload and 'has_right' not in payload
    assert 'head_camera_in_world' not in payload
    assert set(payload['state']) == {
        'ROBOT_LEFT_TRANS', 'ROBOT_LEFT_ROT_MAT',
        'ROBOT_RIGHT_TRANS', 'ROBOT_RIGHT_ROT_MAT'}
    assert np.allclose(payload['intrinsics_per_view'][0],
                       [[0.5, 0, 0.5], [0, 0.5, 0.5], [0, 0, 1]])
    assert payload['extrinsics_per_view'][1][0][3] == pytest.approx(-0.2)


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
