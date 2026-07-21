from pathlib import Path
from types import SimpleNamespace
from xml.etree import ElementTree

import pytest

from unitree_g1_description.joint_state_mapping import (
    G1_JOINT_NAMES,
    gripper_state_to_model_fields,
    load_gripper_model_spec,
    motor_states_to_joint_fields,
)


def _motor(index):
    return SimpleNamespace(q=index + 0.1, dq=index + 0.2, tau_est=index + 0.3)


def test_official_29_motor_order_matches_mode15_urdf():
    urdf_path = (
        Path(__file__).parents[1]
        / "model"
        / "g1_description"
        / "g1_29dof_mode_15.urdf"
    )
    root = ElementTree.parse(str(urdf_path)).getroot()
    body_joints = tuple(
        joint.get("name")
        for joint in root.findall("joint")
        if joint.get("type") == "revolute"
    )
    assert G1_JOINT_NAMES == body_joints


def test_maps_first_29_motor_states_to_joint_fields():
    positions, velocities, efforts = motor_states_to_joint_fields(
        [_motor(index) for index in range(35)])
    assert len(positions) == len(velocities) == len(efforts) == 29
    assert positions[0] == pytest.approx(0.1)
    assert positions[-1] == pytest.approx(28.1)
    assert velocities[-1] == pytest.approx(28.2)
    assert efforts[-1] == pytest.approx(28.3)


def test_rejects_incomplete_or_invalid_positions():
    with pytest.raises(ValueError, match="expected at least 29"):
        motor_states_to_joint_fields([_motor(index) for index in range(28)])

    motors = [_motor(index) for index in range(29)]
    motors[5].q = float("nan")
    with pytest.raises(ValueError, match="non-finite joint position"):
        motor_states_to_joint_fields(motors)


def test_omits_optional_vector_when_value_is_not_finite():
    motors = [_motor(index) for index in range(29)]
    motors[3].dq = float("inf")
    motors[7].tau_est = float("nan")
    positions, velocities, efforts = motor_states_to_joint_fields(motors)
    assert len(positions) == 29
    assert velocities == []
    assert efforts == []


def _left_gripper_model():
    urdf_path = Path(__file__).parents[1] / "model" / "final.urdf"
    return load_gripper_model_spec(urdf_path, "left_")


def test_maps_gripper_state_to_model_fields():
    model = _left_gripper_model()
    positions, velocities, efforts = gripper_state_to_model_fields(
        [0.0, 9.0], [0.5, 8.0], [-0.2, 7.0], model)
    assert model.source_name == "left_eccentric_joint"
    assert positions[0] == 0.0
    assert velocities[0] == 0.5
    assert efforts[0] == -0.2
    assert len(positions) == len(velocities) == len(efforts) == 33


def test_expands_open_gripper_with_each_mimic_stage_clamped_to_urdf_limits():
    model = _left_gripper_model()

    positions, _velocities, _efforts = gripper_state_to_model_fields(
        [2.77], [], [], model)
    by_name = dict(zip(model.joint_names, positions))

    assert len(model.mimic_joints) == 32
    assert by_name["left_eccentric_joint"] == pytest.approx(
        2.76377472169236)
    assert by_name[
        "left_internal_left_slider_spline_01_joint"] == pytest.approx(0.0)
    assert by_name[
        "left_internal_left_connecting_rod_spline_01_joint"] == pytest.approx(
            0.0)
    for joint in model.mimic_joints:
        assert joint.lower <= by_name[joint.name] <= joint.upper


def test_rejects_invalid_gripper_position_and_omits_invalid_optional_fields():
    model = _left_gripper_model()
    with pytest.raises(ValueError, match="no position"):
        gripper_state_to_model_fields([], [], [], model)
    with pytest.raises(ValueError, match="non-finite position"):
        gripper_state_to_model_fields([float("nan")], [], [], model)
    positions, velocities, efforts = gripper_state_to_model_fields(
        [1.0], [float("nan")], [float("inf")], model)
    assert positions[0] == 1.0
    assert velocities == []
    assert efforts == []