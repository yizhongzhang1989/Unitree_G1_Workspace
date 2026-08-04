#!/usr/bin/env python3
"""双臂逆运动学：URDF -> 锁死非臂关节的 14 轴缩减模型 -> 阻尼最小二乘。

只依赖 pinocchio + numpy，不 import rclpy，可以脱离 ROS 直接单测。
参数取值的实测依据写在 config/motion_control.yaml，这里只讲模块自身的契约。

末端位姿相对 ``base_frame``（torso_link）。缩减模型锁死了腰和腿，torso_link 是常量
位姿，构造时算一次 ``_oMb`` 就够——求解时不必关心策略把腰摆到了哪儿。

纯 DLS 牛顿迭代：无线搜索、无 SVD、迭代数硬上限，成本可预测。到上限也不报错，直接
返回尽力而为的解——上肢够不着不该把正在平衡的下肢一起拖下水。

阻尼项写在任务空间（6x6，比 7x7 快），恒等于对**步长**的 L2 正则
``min ||J dq - e||^2 + lambda^2 ||dq||^2``。惩罚 ``dq`` 而非 ``q - q_ref``，所以冗余
那一维没有回中力、完全从种子继承——这正是 ``null_bias`` 要补的那一块。

种子由**调用方**给，本模块不持有跨帧状态。节点用上一帧已发布的目标做种子（热启动），
解天然连续，代价是会一帧帧累积漂移，所以需要三道防线：

1. 本模块的 ``joint_limits`` 把肘上限收到 1.4。shoulder_yaw 与 wrist_roll 都是沿肢体
   长轴的自转轴，肘一伸直两者共线（实测 1.571 处夹角 0.0°，雅可比最差点在 1.44），而
   URDF 行程 ``[-1.047, 2.094]`` 把这一段夹在中间，热启动被推过去就再也回不来。
2. 本模块的 ``null_bias`` 在**零空间**里把这三根轴拉回 0。收限位只能拦住"足尖推过头"，
   拦不住"肘长期顶在 1.4"：目标够不着时肘就得伸直，而那里 3<->5 夹角只剩 9.8°。
   实测 VR 幅度（OU 15 cm）下肘顶限位的帧占 16.6%。
3. 肩上还有一条镜像解支陷阱，收限位治不好，那一道放在**节点**里（``ik_rescue_err``
   残差超限就换站立位形重解）。本模块对此无感——只是被多调了一次。

``null_bias`` 必须投影到零空间，不能直接加进步长：实测同一个偏置向量的末端污染
``||J b|| = 0.30``，而那个构型下要修的任务残差才 0.0725 m——直接加的话偏置项会把手
拽偏、任务项再拉回来，两边对抗，位形没改善而跟随变差。

``max_step_pos`` / ``max_step_ori`` 是**稳定性**参数不是精度参数：步长正比于误差，
目标够不着时误差不收敛，步长会大到把关节顶穿限位再弹回来。实测目标放到够不着的
前方 60 cm：不限幅稳态每帧抖 5.76 rad，限到 0.1 m 后 1e-14。正常跟随每帧才 ~2 mm，
对精度是空操作。
"""

from __future__ import annotations

import numpy as np
import pinocchio as pin

_I6 = np.eye(6)


class ArmIK:
    """双臂 IK。``fk`` / ``solve`` 都不抛异常、不改外部状态，可以直接放进控制环。

    调用方必须传单位四元数（xyzw）；校验属于系统边界，留在节点那一侧做。
    """

    def __init__(self, urdf_xml: str, arm_joints, tip_frames: dict,
                 base_frame: str = 'torso_link', max_iters: int = 10,
                 damping: float = 0.05, tol_pos: float = 1e-3,
                 tol_ori: float = 3.5e-3, joint_limits: dict | None = None,
                 max_step_pos: float = 0.1, max_step_ori: float = 0.5,
                 null_bias: dict | None = None,
                 null_bias_gate: tuple = (0.8, 1.2)) -> None:
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
        # 用 np.array 而不是 np.asarray：后者对 pinocchio 返回的数组是**视图**，
        # 谁不小心原地改一下就把模型自己的行程篡改了。
        self.lower = np.array(self.model.lowerPositionLimit, dtype=np.float64)
        self.upper = np.array(self.model.upperPositionLimit, dtype=np.float64)
        # 按关节收紧限位。**只收不放**（与 URDF 取交集），写错了最多让手臂可用范围变小，
        # 不会把关节送出真实行程。肘必须收，理由见文件头。
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
            # 该末端的活动列 = 它支撑链上的关节。按名字前缀分左右在换 URDF 时会错。
            support = set(self.model.supports[self.model.frames[fid].parentJoint])
            cols = np.array([self.model.idx_qs[j]
                             for j in range(1, self.model.njoints)
                             if j in support], dtype=int)
            self._tip[side] = (fid, cols)

        self._iters = int(max_iters)
        self._lambda2 = float(damping) ** 2
        self._tol_pos = float(tol_pos)
        self._tol_ori = float(tol_ori)
        # 稳定性参数，不是精度参数——理由见文件头。
        self._max_step_pos = float(max_step_pos)
        self._max_step_ori = float(max_step_ori)
        # 零空间偏置增益，按 joint_names 排。全零等于关闭，走原来那条不算投影的快路径。
        self._null_gain = None
        self._null_eye = {}
        gate_lo, gate_hi = (float(v) for v in null_bias_gate)
        if not 0.0 <= gate_lo < gate_hi:
            raise ValueError('null_bias_gate 必须满足 0 <= lo < hi')
        self._gate_lo, self._gate_span = gate_lo, gate_hi - gate_lo
        if null_bias:
            gains = np.zeros(self.model.nq)
            for name, gain in null_bias.items():
                if name not in self.joint_names:
                    raise ValueError(f'null_bias 里的 {name} 不在手臂关节里')
                value = float(gain)
                if not np.isfinite(value) or value < 0.0:
                    raise ValueError(f'{name} 的 null_bias 增益必须是非负有限值')
                gains[self.joint_names.index(name)] = value
            if np.any(gains > 0.0):
                self._null_gain = gains
                self._null_eye = {side: np.eye(cols.size)
                                  for side, (_, cols) in self._tip.items()}

    def _place(self, q: np.ndarray) -> None:
        """computeJointJacobians 内部已经做过正运动学，不必再单独调一次。"""
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

    def fk(self, q) -> dict:
        """各末端相对 base_frame 的 ``[x, y, z, qx, qy, qz, qw]``。"""
        self._place(np.clip(np.asarray(q, dtype=np.float64), self.lower, self.upper))
        poses = {}
        for side, (fid, _) in self._tip.items():
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
                # 收敛判据按侧算：已经到位的一侧这一轮完全不碰，省一次 DLS 求解，
                # 也保证“只发右臂指令时左臂位形一动不动”。
                if side_pos > self._tol_pos or side_ori > self._tol_ori:
                    # 报出去的 pos_err/ori_err 仍是**真实**误差，只有喂给 DLS 的这份限幅。
                    capped = error.copy()
                    if side_pos > self._max_step_pos > 0.0:
                        capped[:3] *= self._max_step_pos / side_pos
                    if side_ori > self._max_step_ori > 0.0:
                        capped[3:] *= self._max_step_ori / side_ori
                    errors[side] = capped
            if not errors:
                return q, pos_err, ori_err, step
            # 两条手臂是彼此独立的分支，雅可比不含对方的列，同一轮里各更新各的 7 列。
            for side, error in errors.items():
                fid, cols = self._tip[side]
                jac = pin.getFrameJacobian(
                    self.model, self.data, fid, pin.LOCAL_WORLD_ALIGNED)[:, cols]
                # 阻尼最小二乘：加了 lambda^2*I 之后矩阵恒正定，solve 不会奇异。
                jjt = jac @ jac.T
                gain = None
                if self._null_gain is not None:
                    # 逐轴门控：只有那根轴自己偏离 0 超过 gate_lo 才开始偏置。
                    gain = self._null_gain[cols] * np.clip(
                        (np.abs(q[cols]) - self._gate_lo) / self._gate_span, 0.0, 1.0)
                    if not gain.any():
                        gain = None
                if gain is None:
                    task = jac.T @ np.linalg.solve(jjt + self._lambda2 * _I6, error)
                else:
                    # 要显式取伪逆才能构造零空间投影 (I - J# J)，比 solve 略贵；
                    # 正常构型下门全关，走上面那条快路径，开销与不带偏置时一致。
                    sharp = jac.T @ np.linalg.inv(jjt + self._lambda2 * _I6)
                    task = sharp @ error + (
                        self._null_eye[side] - sharp @ jac) @ (-gain * q[cols])
                q[cols] = np.clip(q[cols] + task, self.lower[cols], self.upper[cols])
        return q, pos_err, ori_err, self._iters
