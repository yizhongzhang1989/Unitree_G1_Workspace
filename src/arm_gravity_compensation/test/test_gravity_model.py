from pathlib import Path

import numpy as np

from arm_gravity_compensation.constants import ARM_JOINTS
from arm_gravity_compensation.gravity_model import TorsoArmGravityModel


URDF = (Path(__file__).parents[2] / "unitree_g1_description" / "model" /
        "g1_description" / "g1_29dof_mode_15.urdf")
FINAL_URDF = (Path(__file__).parents[2] / "unitree_g1_description" / "model" /
          "final.urdf")


def test_gravity_table_reproduces_pinocchio_for_both_arms():
    model = TorsoArmGravityModel.from_urdf_file(str(FINAL_URDF))
    aggregation = model.group_aggregation("left")
    model.set_arm_parameters(
        "left", aggregation @ np.array([1.05, 0.9, 1.1, 0.95, 1.2, 1.0, 1.4]),
        np.array([0.3, -0.2, 0.15, 0.25, -0.05, 0.1, -0.08]))
    model.set_arm_parameters(
        "right", np.full(len(model.parameter_links["right"]), 1.12),
        np.linspace(-0.2, 0.2, 7))
    table = model.gravity_table()
    random = np.random.RandomState(5)

    for _ in range(50):
        q = random.uniform(-2.0, 2.0, size=14)
        gravity = random.normal(size=3)
        gravity *= 9.81 / np.linalg.norm(gravity)
        for offset, side in ((0, "left"), (7, "right")):
            # 导出表只带纯重力项，标定出的力矩偏置不外传，所以要减掉。
            _, biases = model.arm_parameters(side)
            np.testing.assert_allclose(
                model.gravity_from_table(
                    table, side, q[offset:offset + 7], gravity),
                model.compensation(side, q, gravity) - biases,
                atol=1e-12)

    assert "torque_bias" not in table["left"]

    # 归并后每侧只剩 7 个刚体，腕偏航体的质量等于它名下 8 个固连 link 的标定后质量之和。
    assert table["left"]["joints"] == list(ARM_JOINTS["left"])
    assert len(table["left"]["origin_rotation"]) == 63
    scales, _ = model.arm_parameters("left")
    welded = [index for index, name in enumerate(model.parameter_links["left"])
              if model.parameter_owner[name] == "left_wrist_yaw_joint"]
    assert len(welded) == 8
    np.testing.assert_allclose(
        table["left"]["mass"][6],
        float(np.sum(model.parameter_masses["left"][welded] * scales[welded])),
        rtol=1e-12)


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


def test_welded_links_collapse_into_one_parameter_group():
    model = TorsoArmGravityModel.from_urdf_file(str(FINAL_URDF))
    aggregation = model.group_aggregation("left")

    assert aggregation.shape == (14, 7)
    np.testing.assert_allclose(aggregation.sum(axis=1), np.ones(14))
    assert aggregation[:, 6].sum() == 8

    expected = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.4])
    model.set_arm_parameters("left", aggregation @ expected, np.zeros(7))
    np.testing.assert_allclose(
        model.group_scales("left"), expected, atol=1e-12)


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