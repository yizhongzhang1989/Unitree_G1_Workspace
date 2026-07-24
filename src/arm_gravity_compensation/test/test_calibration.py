from pathlib import Path

import numpy as np

from arm_gravity_compensation.calibration import StaticSample, fit_selected_joints
from arm_gravity_compensation.constants import ARM_JOINTS
from arm_gravity_compensation.gravity_model import TorsoArmGravityModel


URDF = (Path(__file__).parents[2] / "unitree_g1_description" / "model" /
        "g1_description" / "g1_29dof_mode_15.urdf")


def test_selected_joint_fit_recovers_scales_and_biases():
    random = np.random.RandomState(12)
    model = TorsoArmGravityModel.from_urdf_file(str(URDF))
    expected_scales = np.array([0.83, 1.14, 0.91, 1.22, 1.0, 1.0, 1.0])
    expected_biases = np.array([0.12, -0.08, 0.05, -0.03, 0.0, 0.0, 0.0])
    selected = ARM_JOINTS["left"][:4]
    samples = []
    for target_id in range(20):
        q = model.configuration({
            "left": random.uniform(-0.8, 0.8, size=7),
            "right": np.zeros(7),
        })
        gravity = np.array([0.2, -0.15, -9.806])
        design = model.design_matrix("left", q, gravity)
        torque = design @ np.concatenate([expected_scales, expected_biases])
        torque += random.normal(scale=0.002, size=7)
        samples.append(StaticSample(
            target_id=target_id,
            q=q,
            gravity=gravity,
            applied_torque=torque,
            estimated_torque=torque,
            position_error=np.zeros(7),
            velocity_std=np.zeros(7),
        ))

    fit = fit_selected_joints(model, "left", selected, samples)

    np.testing.assert_allclose(fit.mass_scales[:4], expected_scales[:4], atol=0.02)
    np.testing.assert_allclose(fit.torque_bias[:4], expected_biases[:4], atol=0.02)
    np.testing.assert_allclose(fit.mass_scales[4:], np.ones(3))
    assert fit.rmse_after < fit.rmse_before * 0.1


def test_rank_deficient_pose_set_keeps_urdf_nearest_solution():
    model = TorsoArmGravityModel.from_urdf_file(str(URDF))
    q = model.configuration({"right": np.zeros(7)})
    torque = model.design_matrix("right", q, [0, 0, -9.81]) @ np.r_[
        np.ones(7), np.zeros(7)]
    sample = StaticSample(1, q, np.array([0, 0, -9.81]), torque, torque,
                          np.zeros(7), np.zeros(7))

    fit = fit_selected_joints(
        model, "right", ARM_JOINTS["right"], [sample])

    assert fit.nullity > 0
    np.testing.assert_allclose(fit.mass_scales, np.ones(7), atol=1e-8)
    np.testing.assert_allclose(fit.torque_bias, np.zeros(7), atol=1e-8)


def test_final_urdf_reports_payload_null_space_and_link_observability():
    final_urdf = (Path(__file__).parents[2] / "unitree_g1_description" /
                  "model" / "final.urdf")
    random = np.random.RandomState(8)
    model = TorsoArmGravityModel.from_urdf_file(str(final_urdf))
    expected_scales = np.ones(len(model.parameter_links["left"]))
    expected_biases = np.zeros(7)
    samples = []
    for target_id in range(25):
        q = model.configuration({
            "left": random.uniform(-0.9, 0.9, size=7),
            "right": np.zeros(7),
        })
        gravity = np.array([0.15, -0.1, -9.808])
        design = model.design_matrix("left", q, gravity)
        torque = design @ np.concatenate([expected_scales, expected_biases])
        samples.append(StaticSample(
            target_id, q, gravity, torque, torque,
            np.zeros(7), np.zeros(7)))

    fit = fit_selected_joints(
        model, "left", ["left_wrist_yaw_joint"], samples)

    wrist_links = model.parameter_groups("left")["left_wrist_yaw_joint"]
    assert len(wrist_links) == 8
    assert fit.nullity >= 3
    assert fit.parameter_links == model.parameter_links["left"]
    np.testing.assert_allclose(fit.mass_scales, expected_scales, atol=1e-8)
    assert np.count_nonzero(fit.scale_observability < 1.0 - 1e-6) >= 3