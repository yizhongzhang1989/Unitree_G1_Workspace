import numpy as np

from arm_gravity_compensation.torque_control import (
    PoseStabilityWindow,
    TorquePoseController,
)


def test_torque_controller_uses_gravity_pd_and_slew_rate():
    controller = TorquePoseController(
        stiffness=np.ones(7) * 10.0,
        damping=np.ones(7),
        torque_slew_rate=np.ones(7) * 5.0,
        maximum_speed=1.0,
        minimum_duration=1.0,
    )
    controller.start(0.0, np.zeros(7), np.ones(7))

    first = controller.step(0.1, np.zeros(7), np.zeros(7), np.ones(7))
    second = controller.step(0.2, np.zeros(7), np.zeros(7), np.ones(7))

    assert np.all(first.torque <= 0.5 + 1e-12)
    assert np.all(second.torque <= 1.0 + 1e-12)
    assert not second.trajectory_complete


def test_torque_controller_reports_settle_with_noisy_velocity():
    controller = TorquePoseController(minimum_duration=1.0)
    target = np.linspace(0.0, 0.3, 7)
    controller.start(0.0, np.zeros(7), target)

    settled = controller.step(1.1, target, np.zeros(7), np.zeros(7))
    assert settled.trajectory_complete
    assert settled.settled

    controller.start(2.0, target, target)
    moving = controller.step(
        2.1, target, np.ones(7) * 3.0, np.zeros(7))
    assert not moving.trajectory_complete


def test_pose_stability_uses_measured_stationarity_not_target_error():
    window = PoseStabilityWindow(
        duration=0.5,
        position_range_tolerance=0.02,
    )
    equilibrium = np.array([0.4, -0.2, 0.1, 0.8, 0.0, 0.2, -0.1])

    stable = False
    for index in range(7):
        stable = window.update(
            index * 0.1,
            equilibrium + index * 0.001,
            np.full(7, 0.01),
        )

    assert stable
    assert window.max_position_range <= 0.02


def test_pose_stability_rejects_drift_and_motion():
    window = PoseStabilityWindow(duration=0.4)
    for index in range(6):
        stable = window.update(
            index * 0.1,
            np.full(7, index * 0.02),
            np.full(7, 0.1),
        )

    assert not stable
    assert window.max_position_range > window.position_range_tolerance


def test_pose_stability_ignores_noisy_reported_velocity():
    window = PoseStabilityWindow(
        duration=0.5,
        position_range_tolerance=0.01,
    )

    stable = False
    for index in range(7):
        stable = window.update(
            index * 0.1,
            np.ones(7),
            np.full(7, 0.4 if index % 2 else -0.4),
        )

    assert stable
    assert window.max_velocity == 0.4


def test_pose_stability_handles_irregular_realtime_samples():
    window = PoseStabilityWindow(duration=0.6)
    timestamps = [0.0, 0.073, 0.149, 0.226, 0.304, 0.381,
                  0.459, 0.537, 0.614, 0.692]

    results = [
        window.update(timestamp, np.ones(7), np.full(7, 0.02))
        for timestamp in timestamps
    ]

    assert any(results)
    assert window.span >= 0.6