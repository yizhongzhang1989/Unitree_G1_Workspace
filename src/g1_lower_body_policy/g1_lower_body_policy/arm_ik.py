#!/usr/bin/env python3
"""双臂逆运动学：URDF -> 锁死非臂关节的 14 轴缩减模型 -> 阻尼最小二乘。

只依赖 pinocchio + numpy，不 import rclpy，可以脱离 ROS 直接单测。

末端位姿一律相对 ``base_frame``（torso_link）表达。缩减模型把腰和腿全锁死了，
torso_link 在模型世界系里就是个常量位姿，构造时算一次 ``_oMb`` 就够——求解时完全
不必关心策略把腰摆到了哪儿。这正是选 torso_link 而不是 pelvis 当参考系的理由。

求解是纯 DLS 牛顿迭代：没有线搜索、没有奇异值分解、迭代次数硬上限，成本可预测。
50 Hz 下用上一帧的解做种子，实测厘米级小步 3 次以内收敛到亚毫米。到了上限也不报错，
直接返回当前迭代值——上肢够不着不该把正在平衡的下肢一起拖下水。

手臂是 7 自由度、任务只有 6 维，多出来的那一维**必须管**。纯最小范数 DLS 对零空间
没有任何约束，实测画一个 6 cm 的圆就能让 `shoulder_yaw` 慢慢漂到 -1.23 rad，然后在某
一帧被迫重构、单帧跳 0.23 rad——上肢不限速，这一下就是电机全权限的弹动。所以把姿态
偏置投影到任务零空间里，把冗余自由度钉在 `rest_posture` 附近：同一条圆的单帧最大跳变
从 0.234 降到 0.048 rad，且不可达目标下的残留抖动从 1.4e-2 降到 1e-7 rad。
它在零空间里，不影响末端精度。

关节限位直接取 URDF，不额外收紧。
"""

from __future__ import annotations

import numpy as np
import pinocchio as pin

_I6 = np.eye(6)

# 零空间投影算子的阻尼，故意远小于任务步的 damping**2。两者用同一个值是个陷阱：
# 本机手臂雅可比的最小奇异值约 0.083（σ²≈0.0069），和默认阻尼 0.05²=0.0025 同量级，
# 投影算子会严重泄漏到任务空间里去，姿态偏置就变成和末端目标抢——实测残差从
# 0.45 mm 恶化到 0.91 mm、且 31% 的帧永远达不到收敛判据而白白跑满迭代。
# 也不能取得更小：1e-9 时投影几乎精确，近奇异方向病态，画圆的单帧跳变反而从
# 0.073 恶化到 0.295 rad。1e-4 是实测拐点。
_NULL_DAMPING = 1e-4


class ArmIK:
    """双臂 IK。``fk`` / ``solve`` 都不抛异常、不改外部状态，可以直接放进控制环。

    调用方必须传单位四元数（xyzw）；校验属于系统边界，留在节点那一侧做。
    """

    def __init__(self, urdf_xml: str, arm_joints, tip_frames: dict,
                 base_frame: str = 'torso_link', max_iters: int = 10,
                 damping: float = 0.05, tol_pos: float = 1e-3,
                 tol_ori: float = 3.5e-3, rest_posture: dict = None,
                 posture_weight: float = 0.05) -> None:
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
        # q 的顺序由缩减模型自己定，调用方按 joint_names 去映射 31 轴槽位，
        # 不要反过来假设它和 URDF 里的书写顺序一致。
        self.joint_names = [self.model.names[j]
                            for j in range(1, self.model.njoints)]
        self.lower = np.asarray(self.model.lowerPositionLimit, dtype=np.float64)
        self.upper = np.asarray(self.model.upperPositionLimit, dtype=np.float64)

        for name in (base_frame, *tip_frames.values()):
            if not self.model.existFrame(name):
                raise ValueError(f'URDF 缺少坐标帧 {name}')
        pin.forwardKinematics(self.model, self.data, pin.neutral(self.model))
        pin.updateFramePlacements(self.model, self.data)
        self._oMb = self.data.oMf[self.model.getFrameId(base_frame)].copy()

        self._tip = {}
        for side, frame in tip_frames.items():
            fid = self.model.getFrameId(frame)
            # 该末端的活动列 = 它支撑链上的关节。按名字前缀分左右在换 URDF 时会错。
            support = set(self.model.supports[self.model.frames[fid].parentJoint])
            cols = np.array([self.model.idx_qs[j]
                             for j in range(1, self.model.njoints)
                             if j in support], dtype=int)
            self._tip[side] = (fid, cols, np.eye(cols.size))

        # 姿态偏置的靶位形，按关节名给，内部自己排成模型的 q 顺序。
        rest = rest_posture or {}
        self.rest = np.clip([float(rest.get(name, 0.0)) for name in self.joint_names],
                            self.lower, self.upper)
        self._weight = float(posture_weight)
        self._iters = int(max_iters)
        self._lambda2 = float(damping) ** 2
        self._tol_pos = float(tol_pos)
        self._tol_ori = float(tol_ori)

    def _place(self, q: np.ndarray) -> None:
        """computeJointJacobians 内部已经做过正运动学，不必再单独调一次。"""
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

    def fk(self, q) -> dict:
        """各末端相对 base_frame 的 ``[x, y, z, qx, qy, qz, qw]``。"""
        self._place(np.clip(np.asarray(q, dtype=np.float64), self.lower, self.upper))
        poses = {}
        for side, (fid, _, _) in self._tip.items():
            bMt = self._oMb.actInv(self.data.oMf[fid])
            poses[side] = np.concatenate(
                [bMt.translation, pin.Quaternion(bMt.rotation).coeffs()])
        return poses

    def solve(self, seed, targets: dict):
        """targets: ``{side: [x, y, z, qx, qy, qz, qw]}``，base_frame 系。

        没给到的一侧保持种子值不动。返回 ``(q, pos_err, ori_err, iters)``，
        q 始终落在 URDF 限位内。
        """
        q = np.clip(np.asarray(seed, dtype=np.float64), self.lower, self.upper)
        goal = {side: self._oMb * pin.SE3(
            pin.Quaternion(np.asarray(pose[3:7], dtype=np.float64)).matrix(),
            np.asarray(pose[:3], dtype=np.float64))
            for side, pose in targets.items() if side in self._tip}
        pos_err = ori_err = 0.0
        for step in range(self._iters):
            self._place(q)
            errors, pos_err, ori_err = {}, 0.0, 0.0
            for side, oMt in goal.items():
                cur = self.data.oMf[self._tip[side][0]]
                error = np.concatenate([oMt.translation - cur.translation,
                                        pin.log3(oMt.rotation @ cur.rotation.T)])
                side_pos = float(np.linalg.norm(error[:3]))
                side_ori = float(np.linalg.norm(error[3:]))
                pos_err = max(pos_err, side_pos)
                ori_err = max(ori_err, side_ori)
                # 收敛判据按侧算：已经到位的一侧这一轮完全不碰。否则它的任务项虽然
                # 是 0，零空间姿态偏置却照样推着它的肘部走——只发右臂指令时左臂会
                # 悄悄改位形（末端不动，但关节在动），违反"只更新本次字段"的契约。
                if side_pos > self._tol_pos or side_ori > self._tol_ori:
                    errors[side] = error
            if not errors:
                return q, pos_err, ori_err, step
            # 两条手臂是彼此独立的分支，雅可比不含对方的列，同一轮里各更新各的 7 列。
            for side, error in errors.items():
                fid, cols, eye = self._tip[side]
                jac = pin.getFrameJacobian(
                    self.model, self.data, fid, pin.LOCAL_WORLD_ALIGNED)[:, cols]
                # 阻尼最小二乘：加了 lambda^2*I 之后矩阵恒正定，solve 不会奇异。
                jjt = jac @ jac.T
                task = jac.T @ np.linalg.solve(jjt + self._lambda2 * _I6, error)
                # 姿态偏置投影到任务零空间，不影响末端；投影阻尼单独取值，理由见文件头。
                null = eye - jac.T @ np.linalg.solve(jjt + _NULL_DAMPING * _I6, jac)
                step_q = task + null @ (self._weight * (self.rest[cols] - q[cols]))
                q[cols] = np.clip(q[cols] + step_q,
                                  self.lower[cols], self.upper[cols])
        return q, pos_err, ori_err, self._iters
