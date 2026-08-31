"""G1 的正运动学与零位几何，pinocchio 封装。

存在的理由有两条：

1. 重定向出 29 轴关节角之后，参考窗口还要 5 个 key body 的**笛卡尔位置**。直接拿人的
   关节点当 key body 会因为人机肢长不同而系统性偏掉，用 G1 自己的 FK 算出来的才和
   那 29 个角自洽——策略读到的参考才是它做得到的姿态。
2. 重定向公式里的零位常量（上臂零位指向、肘的零位弯角、腿长……）**全部从 URDF 现算**。
   G1 零位并不是人的"立正"：手臂是大臂垂下、小臂前伸，肘的零位弯角有 82°，
   照着"人伸直手臂 = 关节角 0"写死会整条手臂差 80 度。

.. warning::
   pinocchio 的 ``data`` 不可重入。本类的实例只允许在收帧线程里用，别跨线程共享。
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
import pinocchio as pin


@dataclass(frozen=True)
class HingeGeometry:
    """一个单铰链关节在零位附近的「弯角 <-> 关节角」标定。

    ``bend`` 是近端段与远端段的夹角，恒为正；``bend = bend0 + slope * q`` 在铰链的
    整个有效行程内都是精确的线性关系（单铰链的定义）。

    ``placement_offset`` 是关节 origin 自带的那一段旋转：G1 的膝 origin 绕 y 转了
    10 度，所以「远端段相对近端段转过多少」是 ``q + placement_offset`` 而不是 ``q``。
    """

    bend0: float
    slope: float
    placement_offset: float = 0.0

    def angle_for(self, bend: float) -> float:
        return (float(bend) - self.bend0) / self.slope

    @property
    def axis_sign(self) -> float:
        """``cross(近端段, 远端段)`` 相对铰链轴的符号，用来定球窝那三轴的自转。"""
        return 1.0 if self.slope > 0.0 else -1.0


@dataclass(frozen=True)
class BallGeometry:
    """球窝那三轴（pitch-roll-yaw）与标准 ``Ry Rx Rz`` 之间的差。

    G1 的髋和肩**都不是理想球窝**：``left_hip_roll_joint`` 的 origin 绕 y 转了 −10 度，
    ``left_shoulder_roll_joint`` 绕 x 转了 −16 度。这些旋转夹在三个轴中间，直接按
    ``Ry(p)Rx(r)Rz(y)`` 分解会**静默**给出错的角，而且误差随姿态变化。

    好在它们恰好都与相邻关节同轴，可以并进那一轴的零点::

        R = pre @ Ry(p + offset_p) @ Rx(r + offset_r) @ Rz(y + offset_y)
    """

    pre: np.ndarray
    offsets: np.ndarray


@dataclass(frozen=True)
class LimbGeometry:
    """一条「球窝三轴 + 单铰链」肢体链的零位几何。"""

    proximal_dir: np.ndarray
    distal_dir: np.ndarray
    proximal_len: float
    distal_len: float
    hinge: HingeGeometry
    ball: BallGeometry


class G1Kinematics:
    """按名字寻址的 FK。``joint_names`` 就是策略的 29 个动作关节，顺序照它给的来。"""

    def __init__(self, urdf_path: str, joint_names: Sequence[str]) -> None:
        self._model = pin.buildModelFromUrdf(str(urdf_path))
        self._data = self._model.createData()
        available = {self._model.names[i]: i for i in range(1, self._model.njoints)}
        missing = [n for n in joint_names if n not in available]
        if missing:
            raise ValueError(f'URDF {urdf_path} 里没有这些关节: {missing}')
        if self._model.nq != self._model.njoints - 1:
            raise ValueError(f'URDF {urdf_path} 含非单自由度关节，本模块只支持全 revolute 的 29 轴模型')
        # 按名字建映射而不是假定顺序一致：URDF 换一版顺序变了也不会静默错位。
        self._q_index = np.array([available[n] - 1 for n in joint_names], dtype=np.intp)
        self._names = tuple(joint_names)
        self._q = pin.neutral(self._model)
        self._fk(np.zeros(len(joint_names)))

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self._names

    def _fk(self, q29: np.ndarray) -> None:
        self._q[self._q_index] = np.asarray(q29, dtype=np.float64)
        pin.forwardKinematics(self._model, self._data, self._q)
        pin.updateFramePlacements(self._model, self._data)

    def frame_pos(self, name: str) -> np.ndarray:
        """link 原点在 pelvis 系下的位置（模型的根就是 pelvis）。"""
        return np.array(self._data.oMf[self._model.getFrameId(name)].translation)

    def frame_rot(self, name: str) -> np.ndarray:
        return np.array(self._data.oMf[self._model.getFrameId(name)].rotation)

    def key_body_pos(self, q29: np.ndarray, names: Sequence[str]) -> np.ndarray:
        """(K, 3) key body 位置，pelvis 系。"""
        self._fk(q29)
        return np.array([self.frame_pos(n) for n in names])

    def limits(self) -> tuple[np.ndarray, np.ndarray]:
        lower = np.array([self._model.lowerPositionLimit[i] for i in self._q_index])
        upper = np.array([self._model.upperPositionLimit[i] for i in self._q_index])
        return lower, upper

    ##
    # 零位标定：以下几个量全部现算，不写死
    ##

    def segment(self, proximal: str, distal: str, q29: np.ndarray | None = None) -> np.ndarray:
        self._fk(np.zeros(len(self._names)) if q29 is None else q29)
        return self.frame_pos(distal) - self.frame_pos(proximal)

    def _bend(self, a: str, b: str, c: str) -> float:
        u = self.frame_pos(b) - self.frame_pos(a)
        v = self.frame_pos(c) - self.frame_pos(b)
        cosine = float(u @ v) / (float(np.linalg.norm(u)) * float(np.linalg.norm(v)))
        return float(np.arccos(np.clip(cosine, -1.0, 1.0)))

    def calibrate_hinge(self, joint: str, a: str, b: str, c: str) -> HingeGeometry:
        """量出 ``bend = bend0 + slope * q``。

        ``bend`` 取的是三点夹角，恒为正，所以真实关系其实是 V 形：过了共线点就折返。
        **标定点必须挑在行程的远端**——``q=0`` 附近往往正好压在折返点上（G1 的膝零位
        弯角只有 0.4 度），在那儿测斜率会偏 4%，而且折返的那一支还会把符号带反。
        """
        slot = self._names.index(joint)
        lower = float(self._model.lowerPositionLimit[self._q_index[slot]])
        upper = float(self._model.upperPositionLimit[self._q_index[slot]])

        def bend_at(value: float) -> float:
            q = np.zeros(len(self._names))
            q[slot] = value
            self._fk(q)
            return self._bend(a, b, c)

        values = np.linspace(lower, upper, 41)
        bends = np.array([bend_at(v) for v in values])
        # V 形的最大值必在某个端点，从那儿往内两步取拟合点，中间那点留作校验。
        far = int(np.argmax(bends))
        step = 1 if far == 0 else -1
        near, middle = far + 2 * step, far + step
        slope = (bends[near] - bends[far]) / (values[near] - values[far])
        bend0 = float(bends[far] - slope * values[far])
        residual = abs(bends[middle] - (bend0 + slope * values[middle]))
        if residual > 1e-3:
            raise ValueError(
                f'{joint} 的弯角对关节角不是线性的（残差 {residual:.2e} rad），'
                f'{a}-{b}-{c} 之间大概不止一个自由度')
        return HingeGeometry(bend0=bend0, slope=float(slope))

    def calibrate_limb(self, *, root: str, proximal: str, mid: str, distal: str,
                       hinge_joint: str, ball_joints: Sequence[str],
                       ball_frame: str) -> LimbGeometry:
        """标定一条肢体链。

        Args:
            root: 球窝所挂的刚体（pelvis 或 torso_link）。
            ball_frame: 球窝末端的 link（``*_hip_yaw_link`` / ``*_shoulder_yaw_link``）。
                零位肢体方向必须表示在**它自己**的局部系里：髙的零位朝向是
                ``Ry(-10度)`` 而不是单位阵（那 10 度要到膝那里才抵消），拿 root 系
                当参考会让解出来的旋转不是任何一个真实 link 的朝向，就无法分解了。
        """
        self._fk(np.zeros(len(self._names)))
        upper = self.frame_pos(mid) - self.frame_pos(proximal)
        lower = self.frame_pos(distal) - self.frame_pos(mid)
        rest = self.frame_rot(ball_frame)
        _, hinge_offset = self._placement_axis_angle(hinge_joint)
        hinge = self.calibrate_hinge(hinge_joint, proximal, mid, distal)
        return LimbGeometry(
            proximal_dir=rest.T @ (upper / np.linalg.norm(upper)),
            distal_dir=rest.T @ (lower / np.linalg.norm(lower)),
            proximal_len=float(np.linalg.norm(upper)),
            distal_len=float(np.linalg.norm(lower)),
            hinge=HingeGeometry(bend0=hinge.bend0, slope=hinge.slope,
                                placement_offset=hinge_offset),
            ball=self.calibrate_ball(ball_joints),
        )

    def _placement_rotation(self, joint: str) -> np.ndarray:
        return np.array(self._model.jointPlacements[
            int(self._q_index[self._names.index(joint)]) + 1].rotation)

    def _placement_axis_angle(self, joint: str) -> tuple[str, float]:
        """关节 origin 自带的旋转，化成「绕哪根坐标轴、转多少」。

        返回的角带符号，可以直接并进同轴关节的零点。origin 不是绕单根坐标轴转的话
        就没法这么并——那种模型本模块处理不了，直接报错而不是给一个差几度的答案。
        """
        rot = self._placement_rotation(joint)
        angle = float(np.arccos(np.clip((np.trace(rot) - 1.0) / 2.0, -1.0, 1.0)))
        if angle < 1e-6:
            return 'x', 0.0
        vector = np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0],
                           rot[1, 0] - rot[0, 1]]) / (2.0 * np.sin(angle))
        which = int(np.argmax(np.abs(vector)))
        if abs(abs(vector[which]) - 1.0) > 1e-3:
            raise ValueError(
                f'{joint} 的 origin 旋转不是绕单根坐标轴的（轴 {np.round(vector, 4)}），'
                f'没法并进关节零点')
        return 'xyz'[which], angle * float(np.sign(vector[which]))

    def calibrate_ball(self, joints: Sequence[str]) -> BallGeometry:
        """标定球窝三轴。``joints`` 依次是绕 y / x / z 的那三个关节。"""
        axes = ('y', 'x', 'z')
        offsets = np.zeros(3)
        pre = self._placement_rotation(joints[0])
        for k in (1, 2):
            axis, angle = self._placement_axis_angle(joints[k])
            if angle == 0.0:
                continue
            # 只有和相邻两轴之一同轴才能并进零点，否则这条链就不是「三个正交轴」的形状。
            if axis == axes[k - 1]:
                offsets[k - 1] += angle
            elif axis == axes[k]:
                offsets[k] += angle
            else:
                raise ValueError(
                    f'{joints[k]} 的 origin 绕 {axis} 转了 {angle:.4f} rad，'
                    f'既不与 {joints[k - 1]} 同轴也不与自身同轴，球窝分解不成立')
        return BallGeometry(pre=pre, offsets=offsets)

    def pelvis_height(self, q29: np.ndarray, feet: Sequence[str]) -> float:
        """给定位形下 pelvis 原点比最低的踝原点高多少。人机身高对齐的基准。"""
        self._fk(q29)
        return -min(float(self.frame_pos(n)[2]) for n in feet)
