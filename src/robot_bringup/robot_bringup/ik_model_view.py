"""Dynamic active-joint view over an immutable IKT robot model."""

from typing import Sequence

import numpy as np
import pinocchio as pin

from ikt_core.robot_model import R_from_quat_wxyz


def joints_between(model, base: str, target: str,
                   allowed: Sequence[str]):
    target_chain = model.supporting_joints(target)
    if not base or not model.has_frame(base):
        base_chain = []
    else:
        base_chain = model.supporting_joints(base)

    common = 0
    for base_joint, target_joint in zip(base_chain, target_chain):
        if base_joint != target_joint:
            break
        common += 1

    allowed_set = set(allowed)
    joints = [
        joint
        for joint in (
            list(reversed(base_chain[common:])) + target_chain[common:]
        )
        if joint in allowed_set
    ]
    if not joints:
        raise ValueError(
            f"no movable joints between {base or '<root>'!r} and {target!r}")
    return joints


class ActiveJointModel:
    """Present only the selected DOFs while evaluating the full model."""

    def __init__(self, model, q_seed, joints: Sequence[str],
                 base_frame: str = "") -> None:
        self._model = model
        self._seed = np.asarray(q_seed, dtype=float).copy()
        self._base_frame = (
            base_frame if base_frame and model.has_frame(base_frame) else "")
        self._base_seed = (
            model.fk_se3(self._seed, self._base_frame)
            if self._base_frame else None)
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
        q_full = self.expand(q)
        if not self._base_frame:
            return self._model.pose_error(q_full, frame, xyz, quat)

        base = self._model.fk_se3(q_full, self._base_frame)
        target = self._model.fk_se3(q_full, frame)
        current_position = base.rotation.T @ (
            target.translation - base.translation)
        desired_position = self._base_seed.rotation.T @ (
            np.asarray(xyz, dtype=float) - self._base_seed.translation)
        current_rotation = base.rotation.T @ target.rotation
        desired_rotation = (
            self._base_seed.rotation.T @ R_from_quat_wxyz(quat))
        orientation_error = pin.log3(
            desired_rotation @ current_rotation.T)
        return np.hstack([
            desired_position - current_position,
            orientation_error,
        ])

    def frame_jacobian(self, q, frame):
        q_full = self.expand(q)
        target_jacobian = self._model.frame_jacobian(q_full, frame)
        if not self._base_frame:
            return target_jacobian[:, self._full_indices]

        base = self._model.fk_se3(q_full, self._base_frame)
        target = self._model.fk_se3(q_full, frame)
        base_jacobian = self._model.frame_jacobian(
            q_full, self._base_frame)
        rotation = base.rotation.T
        relative_position = rotation @ (
            target.translation - base.translation)
        position_cross = np.array([
            [0.0, -relative_position[2], relative_position[1]],
            [relative_position[2], 0.0, -relative_position[0]],
            [-relative_position[1], relative_position[0], 0.0],
        ])
        relative_jacobian = np.vstack([
            rotation @ (target_jacobian[:3] - base_jacobian[:3]) +
            position_cross @ rotation @ base_jacobian[3:],
            rotation @ (target_jacobian[3:] - base_jacobian[3:]),
        ])
        return relative_jacobian[:, self._full_indices]

    def manipulability(self, q, frame, rows=None):
        jacobian = self.frame_jacobian(q, frame)
        if rows is not None:
            jacobian = jacobian[list(rows), :]
        values = np.linalg.svd(jacobian, compute_uv=False)
        sigma_min = float(values[-1]) if values.size else 0.0
        positive = values[values > 1e-12]
        measure = float(np.sqrt(max(0.0, np.prod(positive ** 2))))
        return measure, sigma_min
