import numpy as np
from scipy.spatial.transform import Rotation


def unit(vector):
    vector = np.asarray(vector, dtype=float)
    length = np.linalg.norm(vector)
    if not np.isfinite(vector).all() or length < 1e-6:
        raise ValueError('Invalid gravity vector')
    return vector / length


def zero_rotation(head, torso):
    nominal = Rotation.from_euler('xyz', [np.pi, 0.05112069379091391, 0]).as_matrix()
    source, target = unit(nominal @ unit(head)), unit(torso)
    cross = np.cross(source, target)
    sine = np.linalg.norm(cross)
    if sine < 1e-8:
        if source @ target < 0:
            raise ValueError('Opposite gravity vectors')
        return nominal
    correction = Rotation.from_rotvec(cross / sine * np.arctan2(sine, source @ target)).as_matrix()
    return correction @ nominal


def joint_angle(head, torso, rotation_zero, axis):
    axis = unit(axis)
    source, target = unit(rotation_zero @ unit(head)), unit(torso)
    source = source - axis * (axis @ source)
    target = target - axis * (axis @ target)
    if min(np.linalg.norm(source), np.linalg.norm(target)) < 0.1:
        raise ValueError('Gravity nearly parallel to head axis')
    source, target = unit(source), unit(target)
    return float(np.arctan2(axis @ np.cross(source, target), source @ target))