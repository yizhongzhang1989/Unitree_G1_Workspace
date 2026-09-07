"""Floor checks use the actual G1 model rather than assumed ankle offsets."""

from pathlib import Path

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from g1_mocap.motion_capture import JOINT_NAMES, validate_ground
from g1_mocap.motion_model import MotionModel


ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / 'unitree_g1_description/model/g1_description/g1_29dof_mode_15.urdf'
CONFIG = yaml.safe_load((ROOT / 'g1_mocap/config/mocap.yaml').read_text())['/mocap']['ros__parameters']


def test_joint_order_and_standing_floor():
    assert tuple(CONFIG['joints']) == JOINT_NAMES
    model = MotionModel(str(URDF))
    joints = np.array(CONFIG['default_joint_pos'])
    height = model.kin.pelvis_height(joints, model.feet) + CONFIG['foot_ground_clearance_m']
    row = np.r_[[0, 0, height], [0, 0, 0, 1], joints]
    heights = model.foot_heights(row[None, :])
    assert np.max(np.abs(heights)) < 0.04
    validate_ground(heights)
    raised = row.copy()
    raised[2] += 0.25
    np.testing.assert_allclose(model.foot_heights(raised[None, :]), heights + 0.25, atol=1e-12)


def test_rotated_foot_includes_collision_origin():
    model = MotionModel(str(URDF))
    joints = np.array(CONFIG['default_joint_pos'])
    row = np.r_[[1, 2, 0.8], Rotation.from_euler('xyz', [0.3, 0.4, 0.2]).as_quat(), joints]
    actual = model.foot_heights(row[None, :])
    model.kin.key_body_pos(joints, model.feet)
    root_rotation = Rotation.from_quat(row[3:7]).as_matrix()
    for side, name in enumerate(model.feet):
        values = [row[2] + (root_rotation @ (
            model.kin.frame_pos(name) + model.kin.frame_rot(name) @ center))[2] - radius
            for center, radius in model.contacts[side]]
        assert actual[0, side] == pytest.approx(min(values))


def test_pelvis_root_required(tmp_path):
    path = tmp_path / 'bad.urdf'
    path.write_text(URDF.read_text().replace('name="pelvis"', 'name="wrong_root"', 1))
    with pytest.raises(ValueError, match='pelvis-rooted'):
        MotionModel(str(path))
