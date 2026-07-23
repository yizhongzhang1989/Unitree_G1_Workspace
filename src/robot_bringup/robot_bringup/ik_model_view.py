"""Dynamic active-joint view over an immutable IKT robot model."""

from typing import Sequence

import numpy as np


def joints_between(model, base: str, target: str,
                   allowed: Sequence[str]):
    target_chain = model.supporting_joints(target)
    if not base or not model.has_frame(base):
        base_chain = []
    else:
        base_chain = model.supporting_joints(base)
        if target_chain[:len(base_chain)] != base_chain:
            raise ValueError(
                f"base frame {base!r} is not an ancestor of {target!r}")

    allowed_set = set(allowed)
    joints = [
        joint for joint in target_chain[len(base_chain):]
        if joint in allowed_set
    ]
    if not joints:
        raise ValueError(
            f"no movable joints between {base or '<root>'!r} and {target!r}")
    return joints


class ActiveJointModel:
    """Present only the selected DOFs while evaluating the full model."""

    def __init__(self, model, q_seed, joints: Sequence[str]) -> None:
        self._model = model
        self._seed = np.asarray(q_seed, dtype=float).copy()
        self.joint_names = list(joints)
        self._index = {
            name: position for position, name in enumerate(self.joint_names)}
        self._full_indices = np.asarray(
            [model.q_index(name) for name in self.joint_names], dtype=int)
        self.nq = len(self.joint_names)

    def reduce(self, q_full):
        return np.asarray(q_full, dtype=float)[self._full_indices].copy()

    def expand(self, q_active):
        q_full = self._seed.copy()
        q_full[self._full_indices] = np.asarray(q_active, dtype=float)
        return q_full

    def q_index(self, name: str) -> int:
        return self._index[name]

    def joint_limits(self):
        lower, upper = self._model.joint_limits()
        return lower[self._full_indices], upper[self._full_indices]

    def active_mask(self, joints):
        if not joints:
            return np.ones(self.nq, dtype=bool)
        selected = set(joints)
        return np.asarray(
            [name in selected for name in self.joint_names], dtype=bool)

    def pose_error(self, q, frame, xyz, quat):
        return self._model.pose_error(self.expand(q), frame, xyz, quat)

    def frame_jacobian(self, q, frame):
        jacobian = self._model.frame_jacobian(self.expand(q), frame)
        return jacobian[:, self._full_indices]

    def manipulability(self, q, frame, rows=None):
        jacobian = self.frame_jacobian(q, frame)
        if rows is not None:
            jacobian = jacobian[list(rows), :]
        values = np.linalg.svd(jacobian, compute_uv=False)
        sigma_min = float(values[-1]) if values.size else 0.0
        positive = values[values > 1e-12]
        measure = float(np.sqrt(max(0.0, np.prod(positive ** 2))))
        return measure, sigma_min
