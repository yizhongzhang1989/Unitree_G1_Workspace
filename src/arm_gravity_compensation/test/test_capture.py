import numpy as np

from arm_gravity_compensation.capture import PassivePoseCapture


def test_auto_capture_requires_motion_then_settle():
    capture = PassivePoseCapture([0, 1], settle_duration=0.5)
    zero = np.zeros(14)

    assert capture.update(0.0, zero, zero) is None
    assert capture.update(1.0, zero, zero) is None
    moving = zero.copy()
    moving[0] = 0.3
    velocity = zero.copy()
    velocity[0] = 0.2
    assert capture.update(1.1, moving, velocity) is None
    assert capture.update(1.2, moving, zero) is None
    captured = capture.update(1.8, moving, zero)

    np.testing.assert_allclose(captured, moving)
    assert capture.update(2.8, moving, zero) is None


def test_near_duplicate_pose_is_not_emitted():
    capture = PassivePoseCapture(
        [0], movement_threshold=0.01, minimum_pose_distance=0.2,
        settle_duration=0.1)
    zero = np.zeros(14)
    first = zero.copy()
    first[0] = 0.3
    capture.update(0.0, zero, zero)
    capture.update(0.1, first, zero)
    assert capture.update(0.3, first, zero) is not None
    near = first.copy()
    near[0] = 0.35
    capture.update(0.4, near, zero)

    assert capture.update(0.6, near, zero) is None