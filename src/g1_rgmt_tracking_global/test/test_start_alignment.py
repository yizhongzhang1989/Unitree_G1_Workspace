import threading
from types import SimpleNamespace

import numpy as np
import pytest

from g1_rgmt_tracking_global.rotations import quat_apply, quat_conj, quat_from_axis, quat_mul
from g1_rgmt_tracking_global.tracking_node import RgmtTrackingNode, State


@pytest.mark.parametrize('streaming', [False, True])
@pytest.mark.parametrize('correction', [0.0, -1.4, 0.9])
def test_start_aligns_position_and_orientation_in_same_world(streaming, correction):
    position_world = np.array([1.2, -0.7, 0.8])
    pelvis_quat = quat_from_axis('z', 0.3)
    torso_quat = quat_mul(pelvis_quat, quat_from_axis('z', 0.1))
    correction_quat = quat_from_axis('z', correction)
    calls = []
    clip = SimpleNamespace(name='reference', align=lambda *args: calls.append(args))
    measured = np.zeros(31)
    node = SimpleNamespace(
        _lock=threading.Lock(), _state=State.IDLE,
        _measured=measured, _imu_quat=pelvis_quat,
        _odom=SimpleNamespace(
            torso_position=lambda: position_world.copy(),
            orientation_in_world=lambda quat: quat_mul(correction_quat, quat)),
        _mocap=SimpleNamespace(calibrated=True) if streaming else None,
        _mocap_clip=clip if streaming else None,
        _clip=clip, _tracking=False, _stand_s=3.0,
        _stale=lambda: '',
        _snapshot=lambda: (measured.copy(), np.zeros(31), pelvis_quat.copy()),
        _now=lambda: 100.0,
        _policy=SimpleNamespace(reset=lambda: None),
        _torso_quat=lambda measured, pelvis: torso_quat.copy(),
        _robot_ground_z=lambda measured, pelvis, position: 0.035,
    )
    response = RgmtTrackingNode._on_start(node, None, SimpleNamespace())
    assert response.success
    assert node._state is State.STAND
    assert len(calls) == 1
    assert np.allclose(calls[0][0], position_world)
    assert np.allclose(calls[0][1], quat_mul(correction_quat, torso_quat))
    foot_separation_local = np.array([0.0, 0.237, 0.0])
    separation_world = quat_apply(calls[0][1], foot_separation_local)
    running_orientation = quat_mul(correction_quat, torso_quat)
    separation_in_running_frame = quat_apply(quat_conj(running_orientation), separation_world)
    assert np.allclose(separation_in_running_frame, foot_separation_local)
    if streaming:
        assert calls[0][2] == 0.035