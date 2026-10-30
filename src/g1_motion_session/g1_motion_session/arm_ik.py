#!/usr/bin/env python3
"""In-process arm IK in base_frame, using xyzw unit quaternions.

solve(seed, targets) separates primary pose solving and strict-null preference.
step(seed, targets, dt) solves an executable pose step inside the per-tick joint
window with reference-seed rescue and a rate * dt limit.
Both return (target, position_error_m, orientation_error_rad, iterations).
step errors describe the rate-limited target; iterations include rescue work.
The caller owns the last published seed. Only the rescue state is stateful.
"""

from __future__ import annotations

from typing import TypedDict

import numpy as np
import pinocchio as pin


class NullStatus(TypedDict):
    state: str
    iterations: int
    initial_cost: float
    final_cost: float
    initial_elbow_cost: float
    final_elbow_cost: float
    projected_gradient: float
    elbow_priority: bool
    rate_limited: bool
    task_fraction: float


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
        gains = np.zeros(self.model.nq)
        null_angles = np.full(self.model.nq, np.nan)
        gate_lo, gate_hi = (float(v) for v in null_gate)
        if not 0.0 <= gate_lo < gate_hi:
            raise ValueError('null_gate 必须满足 0 <= lo < hi')
        self._gate_lo, self._gate_span = gate_lo, gate_hi - gate_lo
        if null_gain:
            for name, gain in null_gain.items():
                if name not in self.joint_names:
                    raise ValueError(f'null_gain 里的 {name} 不在手臂关节里')
                value = float(gain)
                if not np.isfinite(value) or value < 0.0:
                    raise ValueError(f'{name} 的 null_gain 必须是非负有限值')
                gains[self.joint_names.index(name)] = value
        for name, target in (null_target or {}).items():
            if name not in self.joint_names:
                raise ValueError(f'null_target 里的 {name} 不在手臂关节里')
            value = float(target)
            if not np.isfinite(value):
                raise ValueError(f'{name} 的 null_target 必须是有限值')
            index = self.joint_names.index(name)
            if gains[index] <= 0.0:
                raise ValueError(f'{name} 的 null_target 需要正的 null_gain')
            null_angles[index] = value

        self._null_terms = {}
        has_gains = np.any(gains > 0.0)
        for side, (_, cols) in self._tip.items():
            reference_angles = null_angles[cols]
            ungated = np.isfinite(reference_angles)
            self._null_terms[side] = (
                ungated, np.where(ungated, reference_angles, 0.0),
                gains[cols] if has_gains else None,
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
        self._rescue_target: np.ndarray | None = None
        self._null_status: NullStatus | None = None
        try:
            from . import _arm_ik_native
        except ImportError as error:
            raise RuntimeError(
                'Native IK extension unavailable; build g1_motion_session with colcon'
            ) from error
        self._native_module = _arm_ik_native
        self._native = _arm_ik_native.create(urdf_xml, tuple(arm_joints))

    @property
    def rotation_weight(self) -> float:
        return self._rotation_weight

    @property
    def null_status(self) -> NullStatus | None:
        """Last high-precision solve's local optimization status, not collision clearance."""
        return None if self._null_status is None else self._null_status.copy()

    def set_rotation_weight(self, value: float) -> None:
        """设置旋转相对位置的任务权重；0 = 只跟位置，1 = 与位置同权。"""
        weight = float(value)
        if not np.isfinite(weight) or not 0.0 <= weight <= 1.0:
            raise ValueError('rotation_weight 必须是 [0, 1] 内的有限值')
        self._rotation_weight = weight

    def fk(self, q, *, clamp=True) -> dict:
        """Base-frame poses; clamp=False evaluates actual published/measured joints."""
        if clamp:
            bounded = np.maximum(np.asarray(q, dtype=np.float64), self.lower)
            np.minimum(bounded, self.upper, out=bounded)
        else:
            bounded = np.asarray(q, dtype=np.float64)
        pin.forwardKinematics(self.model, self.data, bounded)
        pin.updateFramePlacements(self.model, self.data)
        poses = {}
        for side, (fid, _) in self._tip.items():
            bMt = self._oMb.actInv(self.data.oMf[fid])
            poses[side] = np.concatenate(
                [bMt.translation, pin.Quaternion(bMt.rotation).coeffs()])
        return poses

    def solve(self, seed, targets: dict, *, joint_window=None):
        """targets: ``{side: [x, y, z, qx, qy, qz, qw]}``，base_frame 系。

        先解主任务，再迭代求解单拍严格零空间偏好并检查非线性位姿误差。
        没给到的一侧保持种子值不动。返回 ``(q, pos_err, ori_err, iters)``，
        q 始终落在 URDF 限位内。iters 包括主任务、零空间迭代及候选回投影。
        null_status 区分局部驻点、预算耗尽和受阻，不保证偏好角度全部满足。
        joint_window 约束整个求解；完整任务不可执行时沿 SE(3) 路径缩步。
        """
        self._null_status = None
        q = np.maximum(np.asarray(seed, dtype=np.float64), self.lower)
        np.minimum(q, self.upper, out=q)
        goal = {side: self._oMb * pin.SE3(
            pin.Quaternion(np.asarray(pose[3:7], dtype=np.float64)).matrix(),
            np.asarray(pose[:3], dtype=np.float64))
            for side, pose in targets.items() if side in self._tip}
        bounds = () if joint_window is None else tuple(
            np.ascontiguousarray(bound, dtype=np.float64) for bound in joint_window)
        if len(bounds) not in (0, 2):
            raise ValueError('joint_window must contain lower and upper joint bounds')
        position, orientation, iterations = self._native_module.solve(
            self._native, self, q, goal, *bounds)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return q, position, orientation, iterations

    def step(self, seed, targets: dict, dt: float):
        """Return (model-order target, post-limit pos/ori errors, total iterations).

        Solve an executable pose step inside the per-tick joint window.
        """
        dt = float(dt)
        if not np.isfinite(dt) or dt < 0.0:
            raise ValueError('dt must be finite and nonnegative')
        seed = np.asarray(seed, dtype=np.float64)
        limit = self.rate_limit * dt
        window = (seed - limit, seed + limit)
        solved, position, orientation, iterations = self.solve(
            seed, targets, joint_window=window)
        selected_status = self._null_status
        executable_partial = selected_status is not None and 0. < selected_status['task_fraction'] < 1.
        if position <= self.rescue_err:
            self._rescue_miss = 0
            self._rescue_target = None
        elif (
            selected_status is not None
            and getattr(self, '_rescue_target', None) is not None
            and _pose_errors(self.fk(self._rescue_target, clamp=False), targets)[0]
            < position - self.rescue_err
        ):
            solved = self._rescue_target.copy()
            self._rescue_miss = 0
        elif executable_partial:
            self._rescue_miss = 0
        elif self._rescue_miss < self.rescue_giveup:
            alternate, alt_position, alt_orientation, alt_iterations = self.solve(
                self.reference, targets, joint_window=window)
            iterations += alt_iterations
            if alt_position < position - self.rescue_err:
                solved = alternate
                self._rescue_target = alternate.copy()
                self._rescue_miss = 0
            else:
                self._rescue_miss += 1
                self._null_status = selected_status
        active = np.zeros(self.model.nq, dtype=bool)
        for side in targets:
            if side in self._tip:
                active[self._tip[side][1]] = True
        solved[~active] = seed[~active]
        if self._null_status is not None:
            self._null_status['rate_limited'] = bool(np.any(np.abs(solved - seed) > limit))
        target = seed + np.clip(solved - seed, -limit, limit)
        reached = self.fk(target, clamp=False)
        position, orientation = _pose_errors(reached, targets)
        return target, position, orientation, iterations
