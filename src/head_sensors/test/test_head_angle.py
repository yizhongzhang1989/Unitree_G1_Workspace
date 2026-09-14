import numpy as np
from scipy.spatial.transform import Rotation

from head_sensors.head_angle import joint_angle, zero_rotation


def test_zero_and_signed_angles():
    axis = np.array([-0.03266098964066959, 0.9993252197702058, -0.016803716461596174])
    head = np.array([-.005788, -.003218, -1.00001])
    torso = np.array([.399699, .072819, 9.870483])
    rotation = zero_rotation(head, torso)
    assert abs(joint_angle(head, torso, rotation, axis)) < 1e-12
    for angle in (-.5, -.2, 0, .3):
        gravity = Rotation.from_euler('xyz', [.02, -.03, .01]).apply([0, 0, 1])
        observed = rotation.T @ Rotation.from_rotvec(-axis * angle).apply(gravity)
        np.testing.assert_allclose(joint_angle(observed, gravity, rotation, axis), angle, atol=1e-12)