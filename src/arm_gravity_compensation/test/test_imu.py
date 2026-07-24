import numpy as np

from arm_gravity_compensation.imu import ImuSampleWindow


def test_stable_acceleration_is_averaged_and_converted_to_gravity():
    random = np.random.RandomState(4)
    window = ImuSampleWindow()
    for index in range(120):
        window.add(
            index / 100.0,
            [0.0, 0.0, 9.81] + random.normal(scale=0.02, size=3),
            random.normal(scale=0.005, size=3),
        )

    assert window.ready(1.0, 100)
    estimate = window.estimate(np.eye(3))
    np.testing.assert_allclose(estimate.gravity, [0.0, 0.0, -9.81], atol=0.01)
    assert estimate.sample_count == 120


def test_moving_window_is_rejected():
    window = ImuSampleWindow()
    for index in range(100):
        window.add(index / 100.0, [0.0, 0.0, 8.0 + index % 2 * 4.0], [0, 0, 0])

    try:
        window.estimate(np.eye(3))
    except ValueError as error:
        assert "not stable" in str(error)
    else:
        raise AssertionError("moving IMU window was accepted")