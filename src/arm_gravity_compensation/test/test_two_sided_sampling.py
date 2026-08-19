"""Two-sided approach sampling, against a simulated joint with friction.

The workflow node needs a robot, but the thing worth testing does not: whether
averaging a pose reached from below with the same pose reached from above
removes the friction from the regression observable. That is arithmetic on
``StaticSample``, so it is exercised here directly.
"""

import numpy as np
import pytest

from arm_gravity_compensation.calibration import StaticSample


STIFFNESS = 14.3


def rest_torque(gravity_load, stiction, direction):
    """What ``TorquePoseController.step`` reports once the joint has stopped.

    静止时力矩平衡是 ``tau_applied + tau_g + tau_f = 0``；关节是朝哪个方向走过来
    停下的，摩擦就顶在哪一边，于是 ``tau_f = -stiction * direction``。
    """
    return -gravity_load + stiction * direction


def sample(gravity_load, stiction, direction, *, target_id=0):
    seven = np.arange(7, dtype=float)
    return StaticSample(
        target_id=target_id,
        q=np.zeros(14),
        gravity=np.array([0.0, 0.0, -9.81]),
        applied_torque=rest_torque(gravity_load, stiction, direction),
        estimated_torque=rest_torque(gravity_load, stiction, direction),
        position_error=np.full(7, 0.001 * direction),
        velocity_std=seven * 1.0e-4,
    )


def merge(below, above):
    """The averaging `_sample_from_both_sides` performs."""
    return StaticSample(
        target_id=below.target_id,
        q=0.5 * (below.q + above.q),
        gravity=0.5 * (below.gravity + above.gravity),
        applied_torque=0.5 * (below.applied_torque + above.applied_torque),
        estimated_torque=0.5 * (
            below.estimated_torque + above.estimated_torque),
        position_error=0.5 * (below.position_error + above.position_error),
        velocity_std=np.maximum(below.velocity_std, above.velocity_std),
        friction=0.5 * (below.applied_torque - above.applied_torque),
    )


def test_one_sided_sampling_carries_the_whole_stiction():
    load = np.array([-4.3, 4.5, 3.0, -3.9, 0.03, -1.5, 0.6])
    stiction = np.array([0.77, 0.38, 0.24, 0.55, 0.08, 0.22, 0.11])

    below = sample(load, stiction, +1.0)

    # 单次采样的观测量偏离真实重力恰好一个 tau_s，实测 0.08~0.77 N.m。
    assert np.allclose(below.applied_torque + load, stiction)
    assert np.max(np.abs(below.applied_torque + load)) == pytest.approx(0.77)


def test_averaging_the_two_sides_cancels_it_exactly():
    load = np.array([-4.3, 4.5, 3.0, -3.9, 0.03, -1.5, 0.6])
    stiction = np.array([0.77, 0.38, 0.24, 0.55, 0.08, 0.22, 0.11])

    merged = merge(sample(load, stiction, +1.0), sample(load, stiction, -1.0))

    assert merged.applied_torque == pytest.approx(-load)
    assert merged.estimated_torque == pytest.approx(-load)


def test_only_the_direction_asymmetry_survives():
    """A joint whose friction differs by direction keeps half of the gap."""
    load = np.full(7, -4.0)
    forward, backward = np.full(7, 0.60), np.full(7, 0.40)

    merged = merge(sample(load, forward, +1.0), sample(load, backward, -1.0))

    assert merged.applied_torque - (-load) == pytest.approx(
        0.5 * (forward - backward))


def test_the_difference_hands_back_the_friction_for_free():
    """和给重力，差给摩擦——同一对样本的两个无关分量。"""
    load = np.array([-4.3, 4.5, 3.0, -3.9, 0.03, -1.5, 0.6])
    stiction = np.array([0.77, 0.38, 0.24, 0.55, 0.08, 0.22, 0.11])

    merged = merge(sample(load, stiction, +1.0), sample(load, stiction, -1.0))

    assert merged.applied_torque == pytest.approx(-load)
    assert merged.friction == pytest.approx(stiction)


def test_an_asymmetric_joint_reports_the_mean_of_the_two_directions():
    load = np.full(7, -4.0)
    forward, backward = np.full(7, 0.60), np.full(7, 0.40)

    merged = merge(sample(load, forward, +1.0), sample(load, backward, -1.0))

    assert merged.friction == pytest.approx(0.5 * (forward + backward))


def test_a_one_sided_sample_has_no_friction_to_report():
    load, stiction = np.full(7, -4.0), np.full(7, 0.5)

    assert sample(load, stiction, +1.0).friction == pytest.approx(np.zeros(7))


def test_position_error_averages_out_but_jitter_takes_the_worse():
    load, stiction = np.full(7, -4.0), np.full(7, 0.5)
    below, above = sample(load, stiction, +1.0), sample(load, stiction, -1.0)

    merged = merge(below, above)

    assert merged.position_error == pytest.approx(np.zeros(7))
    # 速度抖动是质量指标，平均会把一次坏采样藏起来，所以取大的。
    assert merged.velocity_std == pytest.approx(
        np.maximum(below.velocity_std, above.velocity_std))


def test_the_retreat_must_clear_the_dead_band():
    """Why ``approach_offset_rad`` defaults above the largest measured band.

    If the retreat is smaller than ``2 * tau_s / kp`` the joint never leaves the
    stuck zone, both visits settle at the same point with the same friction
    sign, and the average changes nothing.
    """
    stiction = 0.77
    dead_band = 2.0 * stiction / STIFFNESS
    assert dead_band == pytest.approx(0.1077, abs=1.0e-3)

    load = np.full(7, -4.0)
    stuck = merge(sample(load, np.full(7, stiction), +1.0),
                  sample(load, np.full(7, stiction), +1.0))

    assert np.max(np.abs(stuck.applied_torque + load)) == pytest.approx(stiction)
