#!/usr/bin/env python3
"""In-process arm IK in base_frame, using xyzw unit quaternions.

solve(seed, targets) retains the original DLS loop and final SVD update.
step(seed, targets, dt) adds reference-seed rescue and a rate * dt limit.
Both return (target, position_error_m, orientation_error_rad, iterations).
step errors describe the rate-limited target; iterations include rescue work.
The caller owns the last published seed. Only the rescue miss count is stateful.
"""

from __future__ import annotations

import numpy as np
import pinocchio as pin

_I6 = np.eye(6)


def _pose_errors(reached, targets):
    position = orientation = 0.0
    for side, pose in targets.items():
        if side not in reached:
            continue
        actual = reached[side]
        position = max(position, float(np.linalg.norm(np.asarray(pose[:3]) - actual[:3])))
        desired_rotation = pin.Quaternion(np.asarray(pose[3:7], dtype=np.float64)).matrix()
        actual_rotation = pin.Quaternion(actual[3:]).matrix()
        orientation = max(orientation, float(np.linalg.norm(pin.log3(desired_rotation @ actual_rotation.T))))
    return position, orientation


class ArmIK:
    """Reduced arm model; solve is mathematical, step is the control pipeline."""

    def __init__(self, urdf_xml: str, arm_joints, tip_frames: dict,
                 base_frame: str = 'torso_link', max_iters: int = 10,
                 damping: float = 0.05, tol_pos: float = 1e-3,
                 tol_ori: float = 3.5e-3, joint_limits: dict | None = None,
                 max_step_pos: float = 0.1, max_step_ori: float = 0.5,
                 rotation_weight: float = 1.0,
                 null_gain: dict | None = None,
                 null_target: dict | None = None,
                 null_gate: tuple = (0.6, 1.2), *,
                 reference=None, rate_limit: float = 10.0,
                 rescue_err: float = 0.01, rescue_giveup: int = 3) -> None:
        full = pin.buildModelFromXML(urdf_xml)
        missing = [name for name in arm_joints if not full.existJointName(name)]
        if missing:
            raise ValueError(f'URDF 缺少手臂关节: {missing}')
        keep = {full.getJointId(name) for name in arm_joints}
        self.model = pin.buildReducedModel(
            full, [j for j in range(1, full.njoints) if j not in keep],
            pin.neutral(full))
        if self.model.nq != len(arm_joints):
            raise ValueError(
                f'缩减模型应为 {len(arm_joints)} 轴，实际 {self.model.nq}')
        self.data = self.model.createData()
        self.joint_names = [self.model.names[j]
                            for j in range(1, self.model.njoints)]
        self.lower = np.array(self.model.lowerPositionLimit, dtype=np.float64)
        self.upper = np.array(self.model.upperPositionLimit, dtype=np.float64)
        for name, bounds in (joint_limits or {}).items():
            if name not in self.joint_names:
                raise ValueError(f'joint_limits 里的 {name} 不在手臂关节里')
            index = self.joint_names.index(name)
            low, high = (float(v) for v in bounds)
            self.lower[index] = max(self.lower[index], low)
            self.upper[index] = min(self.upper[index], high)
            if self.lower[index] >= self.upper[index]:
                raise ValueError(f'{name} 的限位收紧后上下界颠倒了')

        for name in (base_frame, *tip_frames.values()):
            if not self.model.existFrame(name):
                raise ValueError(f'URDF 缺少坐标帧 {name}')
        pin.forwardKinematics(self.model, self.data, pin.neutral(self.model))
        pin.updateFramePlacements(self.model, self.data)
        self._oMb = self.data.oMf[self.model.getFrameId(base_frame)].copy()

        self._tip = {}
        for side, frame in tip_frames.items():
            fid = self.model.getFrameId(frame)
            support = set(self.model.supports[self.model.frames[fid].parentJoint])
            cols = np.array([self.model.idx_qs[j]
                             for j in range(1, self.model.njoints)
                             if j in support], dtype=int)
            self._tip[side] = (fid, cols)

        self._iters = int(max_iters)
        self._lambda2 = float(damping) ** 2
        self._tol_pos = float(tol_pos)
        self._tol_ori = float(tol_ori)
        self._max_step_pos = float(max_step_pos)
        self._max_step_ori = float(max_step_ori)
        self.set_rotation_weight(rotation_weight)
        self._null_gain = None
        self._null_target = np.full(self.model.nq, np.nan)
        self._null_eye = {}
        gate_lo, gate_hi = (float(v) for v in null_gate)
        if not 0.0 <= gate_lo < gate_hi:
            raise ValueError('null_gate 必须满足 0 <= lo < hi')
        self._gate_lo, self._gate_span = gate_lo, gate_hi - gate_lo
        if null_gain:
            gains = np.zeros(self.model.nq)
            for name, gain in null_gain.items():
                if name not in self.joint_names:
                    raise ValueError(f'null_gain 里的 {name} 不在手臂关节里')
                value = float(gain)
                if not np.isfinite(value) or value < 0.0:
                    raise ValueError(f'{name} 的 null_gain 必须是非负有限值')
                gains[self.joint_names.index(name)] = value
            if np.any(gains > 0.0):
                self._null_gain = gains
                self._null_eye = {side: np.eye(cols.size)
                                  for side, (_, cols) in self._tip.items()}
        for name, target in (null_target or {}).items():
            if name not in self.joint_names:
                raise ValueError(f'null_target 里的 {name} 不在手臂关节里')
            value = float(target)
            if not np.isfinite(value):
                raise ValueError(f'{name} 的 null_target 必须是有限值')
            index = self.joint_names.index(name)
            if self._null_gain is None or self._null_gain[index] <= 0.0:
                raise ValueError(f'{name} 的 null_target 需要正的 null_gain')
            self._null_target[index] = value

        self._damping_matrix = self._lambda2 * _I6
        self._null_terms = {}
        for side, (_, cols) in self._tip.items():
            reference_angles = self._null_target[cols]
            ungated = np.isfinite(reference_angles)
            self._null_terms[side] = (
                ungated, bool(ungated.any()), np.where(ungated, reference_angles, 0.0),
                self._null_gain[cols] if self._null_gain is not None else None,
            )
        by_name = dict(zip(arm_joints, reference)) if reference is not None else {}
        self.reference = np.array([by_name.get(name, 0.0) for name in self.joint_names])
        self.rate_limit = float(rate_limit)
        self.rescue_err = float(rescue_err)
        self.rescue_giveup = int(rescue_giveup)
        if not np.isfinite(self.rate_limit) or self.rate_limit < 0.0:
            raise ValueError('rate_limit must be finite and nonnegative')
        if not np.isfinite(self.rescue_err) or self.rescue_err < 0.0 or self.rescue_giveup < 0:
            raise ValueError('rescue limits must be finite and nonnegative')
        if reference is not None and (len(reference) != len(arm_joints) or not np.isfinite(self.reference).all()):
            raise ValueError('reference must contain one finite value per arm joint')
        self._rescue_miss = 0

    @property
    def rotation_weight(self) -> float:
        return self._rotation_weight

    def set_rotation_weight(self, value: float) -> None:
        """设置旋转相对位置的任务权重；0 = 只跟位置，1 = 与位置同权。"""
        weight = float(value)
        if not np.isfinite(weight) or not 0.0 <= weight <= 1.0:
            raise ValueError('rotation_weight 必须是 [0, 1] 内的有限值')
        self._rotation_weight = weight

    def _place(self, q: np.ndarray) -> None:
        """computeJointJacobians 内部已经做过正运动学，不必再单独调一次。"""
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

    def fk(self, q, *, clamp=True) -> dict:
        """Base-frame poses; clamp=False evaluates actual published/measured joints."""
        if clamp:
            bounded = np.maximum(np.asarray(q, dtype=np.float64), self.lower)
            np.minimum(bounded, self.upper, out=bounded)
        else:
            bounded = np.asarray(q, dtype=np.float64)
        self._place(bounded)
        poses = {}
        for side, (fid, _) in self._tip.items():
            bMt = self._oMb.actInv(self.data.oMf[fid])
            poses[side] = np.concatenate(
                [bMt.translation, pin.Quaternion(bMt.rotation).coeffs()])
        return poses

    def _preference(self, angles, side):
        ungated, _, reference, gains = self._null_terms[side]
        if gains is None:
            return None
        offset = angles - reference
        gate = np.where(ungated, 1.0, np.minimum(np.maximum(
            (np.abs(offset) - self._gate_lo) / self._gate_span, 0.0), 1.0))
        preference = -gains * gate * offset
        return preference if preference.any() else None

    def solve(self, seed, targets: dict):
        """targets: ``{side: [x, y, z, qx, qy, qz, qw]}``，base_frame 系。

        没给到的一侧保持种子值不动。返回 ``(q, pos_err, ori_err, iters)``，
        q 始终落在 URDF 限位内。有效目标固定追加一次严格零空间精修，
        iters 包含该次更新；返回误差按最终输出重新 FK 计算。
        """
        q = np.maximum(np.asarray(seed, dtype=np.float64), self.lower)
        np.minimum(q, self.upper, out=q)
        goal = {side: self._oMb * pin.SE3(
            pin.Quaternion(np.asarray(pose[3:7], dtype=np.float64)).matrix(),
            np.asarray(pose[:3], dtype=np.float64))
            for side, pose in targets.items() if side in self._tip}
        rotation_weight = self._rotation_weight
        task_scale = np.array([1.0, 1.0, 1.0,
                               np.sqrt(rotation_weight),
                               np.sqrt(rotation_weight),
                               np.sqrt(rotation_weight)])
        has_ungated = any(self._null_terms[side][1] for side in goal)
        for step in range(self._iters):
            self._place(q)
            errors = {}
            for side, oMt in goal.items():
                cur = self.data.oMf[self._tip[side][0]]
                error = np.concatenate([oMt.translation - cur.translation,
                                        pin.log3(oMt.rotation @ cur.rotation.T)])
                side_pos = float(np.linalg.norm(error[:3]))
                side_ori = float(np.linalg.norm(error[3:]))
                chasing_ori = rotation_weight > 0.0 and side_ori > self._tol_ori
                if side_pos > self._tol_pos or chasing_ori:
                    capped = error.copy()
                    if side_pos > self._max_step_pos > 0.0:
                        capped[:3] *= self._max_step_pos / side_pos
                    if side_ori > self._max_step_ori > 0.0:
                        capped[3:] *= self._max_step_ori / side_ori
                    errors[side] = capped
            if not errors and not has_ungated:
                return self._final_update(q, goal, task_scale, step)
            for side in goal:
                fid, cols = self._tip[side]
                error = errors.get(side)
                if error is None and not self._null_terms[side][1]:
                    continue
                jac = pin.getFrameJacobian(
                    self.model, self.data, fid, pin.LOCAL_WORLD_ALIGNED)[:, cols]
                weighted_jac = task_scale[:, None] * jac
                jjt = weighted_jac @ weighted_jac.T
                preference = self._preference(q[cols], side)
                if preference is None:
                    if error is None:
                        continue
                    task = weighted_jac.T @ np.linalg.solve(
                        jjt + self._damping_matrix, task_scale * error)
                else:
                    sharp = weighted_jac.T @ np.linalg.inv(jjt + self._damping_matrix)
                    task = (self._null_eye[side] - sharp @ weighted_jac) @ preference
                    if error is not None:
                        task += sharp @ (task_scale * error)
                q[cols] = np.minimum(np.maximum(q[cols] + task, self.lower[cols]), self.upper[cols])
            if not errors:
                return self._final_update(q, goal, task_scale, step + 1)
        return self._final_update(q, goal, task_scale, self._iters)

    def _final_update(self, q, goals, task_scale, iterations):
        """原版求解之后固定追加一次 DLS + 严格零空间偏好，不做回溯。"""
        self._place(q)
        for side, target in goals.items():
            fid, cols = self._tip[side]
            actual = self.data.oMf[fid]
            error = np.concatenate((target.translation - actual.translation,
                                    pin.log3(target.rotation @ actual.rotation.T)))
            position = float(np.linalg.norm(error[:3]))
            orientation = float(np.linalg.norm(error[3:]))
            if position > self._max_step_pos > 0:
                error[:3] *= self._max_step_pos / position
            if orientation > self._max_step_ori > 0:
                error[3:] *= self._max_step_ori / orientation
            jacobian = pin.getFrameJacobian(
                self.model, self.data, fid, pin.LOCAL_WORLD_ALIGNED)[:, cols]
            weighted = task_scale[:, None] * jacobian
            step = weighted.T @ np.linalg.solve(
                weighted @ weighted.T + self._damping_matrix, task_scale * error)
            preference = self._preference(q[cols], side)
            if preference is not None:
                _, singular, vectors = np.linalg.svd(weighted, full_matrices=True)
                threshold = np.finfo(float).eps * max(weighted.shape) * singular[0]
                basis = vectors[int(np.count_nonzero(singular > threshold)):].T
                step += basis @ (basis.T @ preference)
            q[cols] = np.minimum(np.maximum(q[cols] + step, self.lower[cols]), self.upper[cols])
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        position = orientation = 0.0
        for side, target in goals.items():
            actual = self.data.oMf[self._tip[side][0]]
            position = max(position, float(np.linalg.norm(target.translation - actual.translation)))
            orientation = max(orientation, float(np.linalg.norm(pin.log3(target.rotation @ actual.rotation.T))))
        return q, position, orientation, iterations + bool(goals)

    def step(self, seed, targets: dict, dt: float):
        """Return (model-order target, post-limit pos/ori errors, total iterations).

        dt is seconds. Rate limiting happens once, after both IK solves. Rescue
        re-arms only when the unlimited solve succeeds, as in the old pipeline.
        """
        dt = float(dt)
        if not np.isfinite(dt) or dt < 0.0:
            raise ValueError('dt must be finite and nonnegative')
        seed = np.asarray(seed, dtype=np.float64)
        solved, position, orientation, iterations = self.solve(seed, targets)
        if position <= self.rescue_err:
            self._rescue_miss = 0
        elif self._rescue_miss < self.rescue_giveup:
            alternate, alt_position, alt_orientation, alt_iterations = self.solve(
                self.reference, targets)
            iterations += alt_iterations
            if alt_position < position - self.rescue_err:
                solved = alternate
                self._rescue_miss = 0
            else:
                self._rescue_miss += 1
        active = np.zeros(self.model.nq, dtype=bool)
        for side in targets:
            if side in self._tip:
                active[self._tip[side][1]] = True
        solved[~active] = seed[~active]
        limit = self.rate_limit * dt
        target = seed + np.clip(solved - seed, -limit, limit)
        reached = self.fk(target, clamp=False)
        position, orientation = _pose_errors(reached, targets)
        return target, position, orientation, iterations
