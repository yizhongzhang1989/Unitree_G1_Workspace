from pathlib import Path

import numpy as np

from arm_gravity_compensation.gravity_model import TorsoArmGravityModel


URDF = (Path(__file__).parents[2] / "unitree_g1_description" / "model" /
        "g1_description" / "g1_29dof_mode_15.urdf")
FINAL_URDF = (Path(__file__).parents[2] / "unitree_g1_description" / "model" /
          "final.urdf")


def test_real_urdf_is_reduced_to_two_torso_relative_arms():
    model = TorsoArmGravityModel.from_urdf_file(str(URDF))

    assert model.model.nq == 14
    assert model.joint_names[0] == "left_shoulder_pitch_joint"
    assert model.joint_names[-1] == "right_wrist_yaw_joint"
    np.testing.assert_allclose(model.imu_to_torso, np.eye(3), atol=1e-12)


def test_mass_scale_is_linear_and_arms_are_decoupled():
    model = TorsoArmGravityModel.from_urdf_file(str(URDF))
    q = model.configuration({
        "left": [0.4, 0.3, -0.2, 0.8, 0.1, -0.3, 0.2],
        "right": [-0.2, -0.4, 0.3, 0.6, -0.2, 0.2, -0.1],
    })
    gravity = [0.3, -0.4, -9.79]
    matrix = model.design_matrix("left", q, gravity)
    scales = np.array([0.8, 1.1, 1.2, 0.9, 1.3, 0.7, 1.05])
    bias = np.linspace(-0.1, 0.1, 7)

    model.set_arm_parameters("left", scales, bias)
    expected = matrix @ np.concatenate([scales, bias])
    np.testing.assert_allclose(
        model.compensation("left", q, gravity), expected, atol=1e-10)

    changed_right_q = q.copy()
    changed_right_q[model.q_indices("right")] += 0.7
    np.testing.assert_allclose(
        model.compensation("left", changed_right_q, gravity), expected,
        atol=1e-10)


def test_gravity_direction_reverses_compensation():
    model = TorsoArmGravityModel.from_urdf_file(str(URDF))
    q = model.configuration({"left": [0.5, 0.2, 0.1, 0.7, 0.2, 0.1, 0.0]})
    down = model.compensation("left", q, [0.0, 0.0, -9.81])
    up = model.compensation("left", q, [0.0, 0.0, 9.81])

    np.testing.assert_allclose(up, -down, atol=1e-10)


def test_final_urdf_keeps_each_payload_link_as_an_individual_parameter():
    model = TorsoArmGravityModel.from_urdf_file(str(FINAL_URDF))
    wrist_group = model.parameter_groups("left")["left_wrist_yaw_joint"]

    assert model.model.nq == 14
    assert len(model.parameter_links["left"]) == 14
    assert "left_wrist_yaw_link" in wrist_group
    assert "left_kwr57b_link" in wrist_group
    assert "left_gripper_base" in wrist_group
    assert "left_eccentric" in wrist_group
    assert len(wrist_group) == 8


def test_final_urdf_link_columns_reproduce_individual_link_scales():
    model = TorsoArmGravityModel.from_urdf_file(str(FINAL_URDF))
    q = model.configuration({
        "left": [0.4, 0.3, -0.2, 0.8, 0.1, -0.3, 0.2],
        "right": np.zeros(7),
    })
    gravity = [0.3, -0.4, -9.79]
    matrix = model.design_matrix("left", q, gravity)
    scales = np.linspace(0.75, 1.25, len(model.parameter_links["left"]))
    bias = np.linspace(-0.1, 0.1, 7)

    model.set_arm_parameters("left", scales, bias)

    np.testing.assert_allclose(
        model.compensation("left", q, gravity),
        matrix @ np.concatenate([scales, bias]),
        atol=1e-10,
    )