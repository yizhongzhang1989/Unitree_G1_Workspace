"""Raw PICO sidecar replay tests without ROS or hardware."""

from pathlib import Path

import numpy as np

from g1_mocap.capture_stream import SourceClip
from g1_mocap.motion_capture import JOINT_NAMES
from g1_mocap.motion_model import MotionModel
from g1_mocap.retarget import Retargeter
from g1_mocap.skeleton import BodyFrame, SMPL_JOINTS
import g1_mocap.source_replay as source_replay_module
from g1_mocap.source_replay import retarget_source
from test_retarget import DEFAULT_Q, KEY_BODIES, skeleton_from_pose


ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / 'unitree_g1_description/model/g1_description/g1_29dof_mode_15.urdf'


def test_source_replay_uses_live_retargeter_and_resamples(tmp_path, monkeypatch):
    model = MotionModel(str(URDF))
    retargeter = Retargeter(
        model.kin, key_bodies=KEY_BODIES, anchor_body='torso_link',
        default_joint_pos=DEFAULT_Q, landmark_iterations=2)
    pelvis = np.array([0.0, 0.0, 0.78])
    standing = skeleton_from_pose(model.kin, DEFAULT_Q, pelvis_pos=pelvis,
                                  pelvis_rot=np.eye(3), frozen_centers=True)
    calibration = retargeter.calibrate([
        BodyFrame(0.0, index, standing, 1, 0) for index in range(30)])
    source = SourceClip(calibration, landmark_iterations=1)
    for index, timestamp in enumerate(np.arange(181) / 90):
        q = DEFAULT_Q.copy()
        q[JOINT_NAMES.index('left_shoulder_pitch_joint')] += 0.4 * timestamp / 2.0
        positions = skeleton_from_pose(model.kin, q, pelvis_pos=pelvis,
                                       pelvis_rot=np.eye(3), frozen_centers=True)
        source.append(BodyFrame(timestamp, index, positions, 1, 0,
                                np.tile(np.eye(3), (len(SMPL_JOINTS), 1, 1))))
    path = tmp_path / 'motion.source.npz'
    source.save(path)

    instances = []

    class RecordingRetargeter(Retargeter):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.previous = []
            self.outputs = []
            instances.append(self)

        def solve(self, frame, calibration, **kwargs):
            previous = kwargs.get('previous_joint_pos')
            self.previous.append(None if previous is None else previous.copy())
            result = super().solve(frame, calibration, **kwargs)
            self.outputs.append(result.joint_pos.copy())
            return result

    monkeypatch.setattr(source_replay_module, 'Retargeter', RecordingRetargeter)

    rows = retarget_source(model, path)
    explicit = retarget_source(model, path, landmark_iterations=1)
    two_iterations = retarget_source(model, path, landmark_iterations=2)
    assert rows.shape == (101, 36)
    assert np.isfinite(rows).all()
    np.testing.assert_array_equal(rows, explicit)
    assert np.max(np.abs(rows[:, 7:] - two_iterations[:, 7:])) > 1e-6
    np.testing.assert_allclose(np.linalg.norm(rows[:, 3:7], axis=1), 1.0)
    assert rows[-1, 7 + JOINT_NAMES.index('left_shoulder_pitch_joint')] > rows[
        0, 7 + JOINT_NAMES.index('left_shoulder_pitch_joint')] + 0.2
    assert instances[0].previous[0] is None
    for previous, expected in zip(instances[0].previous[1:], instances[0].outputs[:-1]):
        np.testing.assert_array_equal(previous, expected)
