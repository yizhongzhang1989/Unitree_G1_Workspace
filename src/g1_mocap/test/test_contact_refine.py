"""Synthetic experiment checks; never modifies recordings or robot control."""

from pathlib import Path

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from g1_mocap.contact_refine import contact_targets, geometry, metrics, refine
from g1_mocap.motion_model import MotionModel


ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / 'unitree_g1_description/model/g1_description/g1_29dof_mode_15.urdf'


def test_contact_hysteresis_preserves_lift():
    points = np.zeros((100, 2, 4, 3))
    points[30:70, 0, :, 2] = 0.2
    contacts, _, _ = contact_targets(points)
    assert contacts[:25].all() and contacts[75:].all()
    assert not contacts[35:65, 0].any()
    assert contacts[:, 1].all()


def test_refinement_reduces_slip_and_preserves_upper_body():
    model = MotionModel(str(URDF))
    config = yaml.safe_load((ROOT / 'g1_mocap/config/mocap.yaml').read_text())['/mocap']['ros__parameters']
    joints = np.asarray(config['default_joint_pos'])
    height = model.kin.pelvis_height(joints, model.feet) + 0.03
    rows = np.tile(np.r_[[0, 0, height], [0, 0, 0, 1], joints], (21, 1))
    rows[:, 0] += np.linspace(0, 0.04, len(rows))
    original = rows.copy()
    output, contacts, weights, targets, solver = refine(model, rows)
    before = metrics(model, rows, contacts, weights, targets)
    after = metrics(model, output, contacts, weights, targets)
    assert after['support_segment_slip_max_m'] < 0.001
    assert after['penetration_max_m'] < 0.001
    assert after['support_xy_speed_m_s']['p95'] < before['support_xy_speed_m_s']['p95'] / 10
    np.testing.assert_array_equal(rows, original)
    np.testing.assert_array_equal(output[:, 3:7], rows[:, 3:7])
    np.testing.assert_array_equal(output[:, 19:], rows[:, 19:])
    assert solver['converged_frames'] == len(rows)
    assert geometry(model, output[0])[0].shape == (2, 4, 3)


def test_airborne_motion_is_not_glued_to_floor():
    model = MotionModel(str(URDF))
    config = yaml.safe_load((ROOT / 'g1_mocap/config/mocap.yaml').read_text())['/mocap']['ros__parameters']
    joints = np.asarray(config['default_joint_pos'])
    rows = np.tile(np.r_[[0, 0, 1.2], [0, 0, 0, 1], joints], (12, 1))
    rows[:, 0] = np.linspace(0, 0.1, len(rows))
    output, contacts, _, _, solver = refine(model, rows)
    assert not contacts.any()
    np.testing.assert_allclose(output, rows, atol=1e-9)
    assert solver['converged_frames'] == len(rows)


def test_dynamic_contact_preserves_short_flight_and_fast_departure():
    points = np.zeros((50, 2, 4, 3))
    points[20:26, ..., 2] = np.array([0.015, 0.04, 0.055, 0.055, 0.04, 0.015])[:, None, None]
    contacts, targets, weights = contact_targets(points)
    assert contacts[5:15].all()
    assert not contacts[20:26].any()
    np.testing.assert_array_equal(targets[20:26], points[20:26])
    assert not weights[20:26].any()


def test_dynamic_toe_contact_does_not_flatten_heel():
    points = np.zeros((30, 2, 4, 3))
    points[:, :, 2:, 2] = 0.08
    contacts, targets, weights = contact_targets(points)
    assert contacts[:, :, :2].all()
    assert not contacts[:, :, 2:].any()
    np.testing.assert_array_equal(targets[:, :, 2:], points[:, :, 2:])
    np.testing.assert_allclose(targets[:, :, :2, 2], 0.0001)
    assert not weights[:, :, 2:].any()


def test_dynamic_full_jump_keeps_flight_trajectory():
    model = MotionModel(str(URDF))
    config = yaml.safe_load((ROOT / 'g1_mocap/config/mocap.yaml').read_text())['/mocap']['ros__parameters']
    joints = np.asarray(config['default_joint_pos'])
    height = model.kin.pelvis_height(joints, model.feet) + 0.03
    rows = np.tile(np.r_[[0, 0, height], [0, 0, 0, 1], joints], (90, 1))
    lift = np.zeros(len(rows))
    lift[25:56] = 0.18 * np.sin(np.linspace(0, np.pi, 31))
    rows[:, 2] += lift
    output, contacts, weights, targets, solver = refine(model, rows)
    assert not contacts[30:51].any()
    np.testing.assert_array_equal(output[30:51], rows[30:51])
    assert solver['converged_frames'] == len(rows)
    report = metrics(model, output, contacts, weights, targets)
    assert report['penetration_max_m'] < 0.001
    assert report['stable_support_intervals'] > 0
    assert report['support_xy_speed_m_s']['p95'] < 0.001
    np.testing.assert_array_equal(output[:, 3:7], rows[:, 3:7])
    np.testing.assert_array_equal(output[:, 19:], rows[:, 19:])


def test_dynamic_release_returns_to_original_without_target_jump():
    points = np.zeros((100, 2, 4, 3))
    points[:, :, :, 0] = np.linspace(0, 0.08, len(points))[:, None, None]
    points[60:, :, :, 2] = 0.2
    contacts, targets, weights = contact_targets(points)
    for side in range(2):
        for corner in range(4):
            last_contact = np.flatnonzero(contacts[:, side, corner])[-1]
            assert weights[last_contact, side, corner] == 0
            np.testing.assert_array_equal(targets[last_contact:, side, corner],
                                          points[last_contact:, side, corner])


def test_dynamic_full_support_targets_form_rigid_foot():
    model = MotionModel(str(URDF))
    points = np.zeros((40, 2, 4, 3))
    for side in range(2):
        local = np.array([center for center, radius in model.contacts[side]])
        points[:, side] = local + [0, side * 0.2, 0.03]
    points[:, :, 0, 0] += np.linspace(0, 0.02, len(points))[:, None]
    rotations = np.tile(np.eye(3), (len(points), 2, 1, 1))
    contacts, targets, weights = contact_targets(points, model, rotations)
    assert contacts.all() and np.all(weights == 1)
    for side in range(2):
        local = np.array([center for center, radius in model.contacts[side]])
        expected = np.linalg.norm(local[:, None] - local[None, :], axis=2)
        actual = np.linalg.norm(targets[0, side, :, None] - targets[0, side, None, :], axis=2)
        np.testing.assert_allclose(actual, expected, atol=1e-12)


@pytest.mark.parametrize('pitch', [-0.3, 0.3])
def test_tilted_support_preserves_unloaded_edge(pitch):
    model = MotionModel(str(URDF))
    config = yaml.safe_load((ROOT / 'g1_mocap/config/mocap.yaml').read_text())['/mocap']['ros__parameters']
    row = np.r_[[0, 0, 0.8], Rotation.from_euler('y', pitch).as_quat(), config['default_joint_pos']]
    initial_points, _ = geometry(model, row)
    row[2] -= initial_points[..., 2].min()
    rows = np.tile(row, (21, 1))
    points, rotations = geometry(model, row)
    contacts, targets, weights = contact_targets(
        np.tile(points, (len(rows), 1, 1, 1)), model,
        np.tile(rotations, (len(rows), 1, 1, 1)))
    assert contacts.any() and not contacts.all()
    np.testing.assert_array_equal(targets[0][~contacts[0]], points[~contacts[0]])
    assert np.all(weights[0][~contacts[0]] == 0)
    output, _, _, _, solver = refine(model, rows)
    assert solver['converged_frames'] == len(rows)
    after, _ = geometry(model, output[-1])
    assert np.max(np.abs(after[~contacts[-1]] - points[~contacts[-1]])) < 0.001
    assert after[..., 2].min() > -0.001


def test_one_sequence_uses_same_solver_for_support_roll_flight_landing():
    model = MotionModel(str(URDF))
    config = yaml.safe_load((ROOT / 'g1_mocap/config/mocap.yaml').read_text())['/mocap']['ros__parameters']
    rows = np.tile(np.r_[[0, 0, 0.8], [0, 0, 0, 1], config['default_joint_pos']], (120, 1))
    for index, row in enumerate(rows):
        pitch = 0.3 * min(1, max(0, (index - 20) / 20)) if index < 60 else 0.0
        row[3:7] = Rotation.from_euler('y', pitch).as_quat()
        points, _ = geometry(model, row)
        row[2] -= points[..., 2].min()
    rows[60:90, 2] += 0.2 * np.sin(np.linspace(0, np.pi, 30))
    output, contacts, weights, targets, solver = refine(model, rows)
    assert contacts[5:15].all()
    assert contacts[45:55].any() and not contacts[45:55].all()
    assert not contacts[65:85].any()
    assert contacts[100:].all()
    np.testing.assert_array_equal(output[65:85], rows[65:85])
    np.testing.assert_array_equal(output[:, 3:7], rows[:, 3:7])
    np.testing.assert_array_equal(output[:, 19:], rows[:, 19:])
    assert solver['converged_frames'] == len(rows)
    report = metrics(model, output, contacts, weights, targets)
    assert report['penetration_max_m'] < 0.001


def test_alternating_support_has_independent_foot_anchors():
    points = np.zeros((120, 2, 4, 3))
    points[20:50, 0, :, 2] = 0.15
    points[65:95, 1, :, 2] = 0.15
    points[50:, 0, :, 0] = 0.25
    points[95:, 1, :, 0] = 0.3
    contacts, targets, _ = contact_targets(points)
    assert not contacts[25:45, 0].any() and contacts[25:45, 1].all()
    assert contacts[70:90, 0].all() and not contacts[70:90, 1].any()
    np.testing.assert_array_equal(targets[25:45, 0], points[25:45, 0])
    np.testing.assert_array_equal(targets[70:90, 1], points[70:90, 1])
    np.testing.assert_allclose(targets[-1, 0, :, 0], 0.25)
    np.testing.assert_allclose(targets[-1, 1, :, 0], 0.3)
