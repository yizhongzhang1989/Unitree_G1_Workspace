"""SMPL 24 关节位置 -> G1 29 轴关节角。纯几何 + numpy + pinocchio，可离线单测。

**只吃位置，不吃厂商给的关节朝向。** PICO 没公开 ``XrBodyJointBD`` 各关节的局部系
定义，拿它去对 G1 的关节轴等于赌一个没写进文档的约定；而"三点定一个刚体朝向"是
确定的几何。代价是**绕肢体自身轴的自转解不出来**：前臂 roll 与脚的内外翻按 0 处理，
这两轴在 G1 上行程本来就窄（腕 roll ±113 度、踝 roll ±15 度），影响有限。

每条肢体按「球窝三轴 + 单铰链」解：

1. 铰链角由近端段与远端段的**夹角**定，零位弯角和斜率从 URDF 现算
   （G1 零位不是人的立正——肘的零位弯角有 82 度）；
2. 球窝三轴由两个约束定死：近端段方向对上，铰链转轴方向对上。
   转轴取 ``cross(近端段, 远端段)``，肢体伸直时这个叉乘退化，改用躯干的侧向轴兜底。

人机差异全部收在 :class:`RetargetCalibration` 里，只标一次，看那里的注释。
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, replace

import numpy as np

from .kinematics import G1Kinematics, LimbGeometry
from .rotations import quat_from_mat
from .skeleton import JOINT_INDEX, BodyFrame

_EPS = 1e-9


@dataclass(frozen=True)
class LimbSpec:
    """一条肢体：SMPL 侧取哪四个点，G1 侧对哪五个关节、挂在哪个刚体上。

    ``tip_defines_hinge`` 说的是**末端朝向能不能反推铰链轴**。腿可以：踝只有
    pitch/roll，没有偏航，所以脚尖方向和大腿一起就把膝轴钉死了。臂不行：腕有三个
    自由度，手的指向和肘轴之间没有固定关系。

    ``kin_links`` 的第三项是远端段的末端，必须停在**铰链链的末尾**而不是整条肢体的
    末端 link：G1 的腕是三轴串联，取到 ``wrist_yaw_link`` 会让"前臂长度"随腕角变，
    肘角跟着错十几度。``tip_link`` 才是脚尖 / 手尖所依附的那个 link。
    """

    root_body: str
    smpl: tuple[str, str, str, str]
    ball_joints: tuple[str, str, str]
    ball_frame: str
    hinge_joint: str
    tip_joints: tuple[str, ...]
    kin_links: tuple[str, str, str]
    tip_link: str
    tip_defines_hinge: bool


LEGS = {
    'left': LimbSpec(
        root_body='pelvis',
        smpl=('LEFT_HIP', 'LEFT_KNEE', 'LEFT_ANKLE', 'LEFT_FOOT'),
        ball_joints=('left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint'),
        ball_frame='left_hip_yaw_link',
        hinge_joint='left_knee_joint',
        tip_joints=('left_ankle_pitch_joint', 'left_ankle_roll_joint'),
        kin_links=('left_hip_roll_link', 'left_knee_link', 'left_ankle_roll_link'),
        tip_link='left_ankle_roll_link',
        tip_defines_hinge=True,
    ),
    'right': LimbSpec(
        root_body='pelvis',
        smpl=('RIGHT_HIP', 'RIGHT_KNEE', 'RIGHT_ANKLE', 'RIGHT_FOOT'),
        ball_joints=('right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint'),
        ball_frame='right_hip_yaw_link',
        hinge_joint='right_knee_joint',
        tip_joints=('right_ankle_pitch_joint', 'right_ankle_roll_joint'),
        kin_links=('right_hip_roll_link', 'right_knee_link', 'right_ankle_roll_link'),
        tip_link='right_ankle_roll_link',
        tip_defines_hinge=True,
    ),
}

ARMS = {
    'left': LimbSpec(
        root_body='torso_link',
        smpl=('LEFT_SHOULDER', 'LEFT_ELBOW', 'LEFT_WRIST', 'LEFT_HAND'),
        ball_joints=('left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
                     'left_shoulder_yaw_joint'),
        ball_frame='left_shoulder_yaw_link',
        hinge_joint='left_elbow_joint',
        tip_joints=('left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint'),
        kin_links=('left_shoulder_roll_link', 'left_elbow_link', 'left_wrist_roll_link'),
        tip_link='left_wrist_yaw_link',
        tip_defines_hinge=False,
    ),
    'right': LimbSpec(
        root_body='torso_link',
        smpl=('RIGHT_SHOULDER', 'RIGHT_ELBOW', 'RIGHT_WRIST', 'RIGHT_HAND'),
        ball_joints=('right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
                     'right_shoulder_yaw_joint'),
        ball_frame='right_shoulder_yaw_link',
        hinge_joint='right_elbow_joint',
        tip_joints=('right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint'),
        kin_links=('right_shoulder_roll_link', 'right_elbow_link', 'right_wrist_roll_link'),
        tip_link='right_wrist_yaw_link',
        tip_defines_hinge=False,
    ),
}

WAIST_JOINTS = ('waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint')
ANKLE_LINKS = ('left_ankle_roll_link', 'right_ankle_roll_link')


# 下面这几个都只吃 3 维向量。numpy 的通用版本要走广播分派（np.cross 会 moveaxis、
# np.clip 标量要过 nan 检查），实测分别是 33 us / 13.6 us，而 solve() 每帧要调几十次。
def _norm(v: np.ndarray) -> float:
    return math.hypot(*v)


def _cross(u: np.ndarray, v: np.ndarray) -> np.ndarray:
    return np.array([u[1] * v[2] - u[2] * v[1],
                     u[2] * v[0] - u[0] * v[2],
                     u[0] * v[1] - u[1] * v[0]])


def _clamp(value: float, low: float, high: float) -> float:
    return low if value < low else (high if value > high else value)


def _unit(v: np.ndarray) -> np.ndarray:
    n = _norm(v)
    if n < _EPS:
        raise ValueError('零向量无法定方向')
    return v / n


def _columns(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
    return np.array((x, y, z)).T


def _frame_zy(z_hint: np.ndarray, y_hint: np.ndarray) -> np.ndarray:
    """由「侧向」和「向上」两个粗方向定一个右手系，列为 [x|y|z]。

    ``y_hint`` 精确保留（骨架里左右成对的连线最可靠，两个 tracker 都在同一段躯干上），
    ``z_hint`` 只用来定平面。
    """
    y = _unit(y_hint)
    z = _unit(z_hint - float(z_hint @ y) * y)
    return _columns(_cross(y, z), y, z)


def _frame_along(primary: np.ndarray, secondary: np.ndarray) -> np.ndarray:
    """列为 [l|m|n] 的右手系：``n`` 就是 ``primary``，``m`` 由 ``secondary`` 正交化而来。"""
    n = _unit(primary)
    m = _unit(secondary - float(secondary @ n) * n)
    return _columns(_cross(m, n), m, n)


def _rotation_between(from_primary: np.ndarray, from_secondary: np.ndarray,
                      to_primary: np.ndarray, to_secondary: np.ndarray) -> np.ndarray:
    """求 R，把 (primary, secondary) 这一对方向从 from 转到 to。primary 精确对上。"""
    return _frame_along(to_primary, to_secondary) @ _frame_along(from_primary,
                                                                 from_secondary).T


def _decompose_zxy(mat: np.ndarray) -> tuple[float, float, float]:
    """``R = Rz(yaw) Rx(roll) Ry(pitch)`` -> (yaw, roll, pitch)。腰三轴就是这个顺序。"""
    roll = math.asin(_clamp(float(mat[2, 1]), -1.0, 1.0))
    pitch = math.atan2(-mat[2, 0], mat[2, 2])
    yaw = math.atan2(-mat[0, 1], mat[1, 1])
    return yaw, roll, pitch


def _decompose_yxz(mat: np.ndarray) -> tuple[float, float, float]:
    """``R = Ry(pitch) Rx(roll) Rz(yaw)`` -> (pitch, roll, yaw)。髋和肩都是这个顺序。"""
    roll = math.asin(_clamp(float(-mat[1, 2]), -1.0, 1.0))
    yaw = math.atan2(mat[1, 0], mat[1, 1])
    pitch = math.atan2(mat[0, 2], mat[2, 2])
    return pitch, roll, yaw


def _rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _yaw_only(rot: np.ndarray) -> np.ndarray:
    """只保留绕 z 的分量。标定姿态偏置时用：人面朝哪边不能被当成偏置扣掉。"""
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _fallback_hinge_axis(rot_root: np.ndarray, limb: np.ndarray) -> np.ndarray:
    """肢体伸直、叉乘退化时的铰链轴。

    必须继续是一根**语义上的铰链轴**，也就是躯干的侧向轴；随便换一根会让球窝在
    伸直的那一瞬间凭空转一个直角。只有当肢体恰好沿侧向轴（侧平举且伸直）时，
    才改用前向轴——那种位形下自转本来就不可观测。
    """
    lateral = rot_root[:, 1]
    return lateral if abs(float(_unit(limb) @ lateral)) < 0.99 else rot_root[:, 0]


def _angle_between(u: np.ndarray, v: np.ndarray) -> float:
    return math.acos(_clamp(float(_unit(u) @ _unit(v)), -1.0, 1.0))


def _smoothstep(value: float, low: float, high: float) -> float:
    t = _clamp((value - low) / max(high - low, _EPS), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


@dataclass(frozen=True)
class RetargetCalibration:
    """人机差异的对齐，由一段**站立**姿态标出来。

    为什么不能只算几何就完事：人站直时直接解出来的位形有两处**恒定偏置**，
    实测都落在策略训练分布之外，而且两处都是**结构差异**，不是算得不够准：

    * 高 +19.8 度（训练数据 max 才 +10.3）——拿 ``PELVIS->SPINE1`` 当盆骨竖直轴，
      而人的盆骨本来就有前倾，这个向量不是盆骨真正的 z 轴；
    * 高偏航 ±35 度（p95 才 ±14）——人站立自然外八，而 G1 的踝**没有偏航自由度**，
      这个外旋无处可去，全压到高上了。

    所以标定分两层：姿态偏置（``*_fix``）修局部系的定义，它还会污染
    ``projected_gravity``；关节零位（``joint_bias`` -> ``joint_target``）把整个站立位形
    搬到 G1 的 ``default_joint_pos`` 上，之后按**增量**走。

    代价是**绝对姿态会丢**：人站直不再对应“G1 腿伸直”，而是对应 default 姿态。
    这个交换是值的——策略只在训练分布内可靠。

    Attributes:
        scale: 人 -> G1 的长度缩放，按腿长比取。它同时决定步幅和蹲起幅度。
        pelvis_ref_z: 校准帧里人的骨盆离地高度，作为高度变化的零点。
        stand_height: 对应的 G1 骨盆离地高度。
        pelvis_fix / torso_fix: 局部系修正，右乘在解出来的朝向上。
        joint_bias: 校准帧解出的 29 轴位形。
        joint_target: 它该被映射到哪里，也就是 G1 的 ``default_joint_pos``。
    """

    scale: float
    pelvis_ref_z: float
    stand_height: float
    pelvis_fix: np.ndarray
    torso_fix: np.ndarray
    joint_bias: np.ndarray
    joint_target: np.ndarray

    @staticmethod
    def identity(stand_height: float, n_joints: int = 29) -> RetargetCalibration:
        """不做任何人机对齐。只在单测里用——实机上一定要 :meth:`Retargeter.calibrate`。"""
        return RetargetCalibration(
            scale=1.0, pelvis_ref_z=stand_height, stand_height=stand_height,
            pelvis_fix=np.eye(3), torso_fix=np.eye(3),
            joint_bias=np.zeros(n_joints), joint_target=np.zeros(n_joints))


@dataclass(frozen=True)
class RetargetResult:
    """一帧重定向的产物，字段与瘦身 NPZ 的每帧切片逐个对应。"""

    t: float
    joint_pos: np.ndarray
    root_pos: np.ndarray
    root_quat: np.ndarray
    anchor_pos: np.ndarray
    anchor_quat: np.ndarray
    key_pos: np.ndarray


class Retargeter:
    """把 :class:`~.skeleton.BodyFrame` 解成 G1 的位形。

    Args:
        kin: G1 的运动学，必须用**同一份** 29 轴顺序构建。
        key_bodies: 参考窗口要的 key body，顺序必须与策略契约一致。
        anchor_body: 锚刚体（``torso_link``）。
        default_joint_pos: 29 轴默认位形。两个用途：量 G1 的站立高度，
            以及当作站立零位映射的目标（它是策略训练分布的中心）。
    """

    def __init__(self, kin: G1Kinematics, *, key_bodies: Sequence[str], anchor_body: str,
                 default_joint_pos: np.ndarray,
                 foot_ground_clearance_m: float = 0.03) -> None:
        self._kin = kin
        self._names = list(kin.joint_names)
        self._slot = {name: i for i, name in enumerate(self._names)}
        self._key_bodies = tuple(key_bodies)
        self._anchor = anchor_body
        self._lower, self._upper = kin.limits()
        self._legs = {side: kin.calibrate_limb(
            root=spec.root_body, proximal=spec.kin_links[0], mid=spec.kin_links[1],
            distal=spec.kin_links[2], hinge_joint=spec.hinge_joint,
            ball_joints=spec.ball_joints, ball_frame=spec.ball_frame)
            for side, spec in LEGS.items()}
        self._arms = {side: kin.calibrate_limb(
            root=spec.root_body, proximal=spec.kin_links[0], mid=spec.kin_links[1],
            distal=spec.kin_links[2], hinge_joint=spec.hinge_joint,
            ball_joints=spec.ball_joints, ball_frame=spec.ball_frame)
            for side, spec in ARMS.items()}
        self._stand_height = kin.pelvis_height(
            np.asarray(default_joint_pos, dtype=np.float64), ANKLE_LINKS
        ) + float(foot_ground_clearance_m)
        self._default_joint_pos = np.asarray(default_joint_pos, dtype=np.float64).copy()
        if len(self._default_joint_pos) != len(self._names):
            raise ValueError(
                f'default_joint_pos 有 {len(self._default_joint_pos)} 项，'
                f'与 {len(self._names)} 轴对不上')
        self._leg_length = 0.5 * sum(g.proximal_len + g.distal_len for g in self._legs.values())

    @property
    def stand_height(self) -> float:
        return self._stand_height

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(self._names)

    @property
    def key_bodies(self) -> tuple[str, ...]:
        return self._key_bodies

    @property
    def limits(self) -> tuple[np.ndarray, np.ndarray]:
        return self._lower, self._upper

    ##
    # 校准
    ##

    def calibrate(self, frames: Sequence[BodyFrame]) -> RetargetCalibration:
        """用一段**站立**姿态标出人机差异。人得站直、双脚平放，其余不要求。

        两遍：第一遍拿未修正的朝向量出局部系偏置，第二遍在修正后的朝向下量关节零位。
        必须分两遍——修了朝向，髋和肩解出来的角就变了，一遍量不全。
        """
        if not frames:
            raise ValueError('校准至少要一帧')
        positions = np.mean([f.positions for f in frames], axis=0)
        human_leg = 0.5 * sum(
            float(np.linalg.norm(positions[JOINT_INDEX[spec.smpl[1]]]
                                 - positions[JOINT_INDEX[spec.smpl[0]]]))
            + float(np.linalg.norm(positions[JOINT_INDEX[spec.smpl[2]]]
                                   - positions[JOINT_INDEX[spec.smpl[1]]]))
            for spec in LEGS.values())
        if human_leg < 0.2:
            raise ValueError(f'量出来的人腿长只有 {human_leg:.3f} m，这一段不是有效的站立姿态')

        sample = BodyFrame(t=0.0, seq=0, positions=positions, status=1, message=0)
        rough = RetargetCalibration(
            scale=self._leg_length / human_leg,
            pelvis_ref_z=float(positions[JOINT_INDEX['PELVIS']][2])
            * self._leg_length / human_leg,
            stand_height=self._stand_height,
            pelvis_fix=np.eye(3), torso_fix=np.eye(3),
            joint_bias=np.zeros(len(self._names)),
            joint_target=np.zeros(len(self._names)))

        pelvis, torso = self._body_frames(positions)
        posed = replace(rough,
                        pelvis_fix=pelvis.T @ _yaw_only(pelvis),
                        torso_fix=torso.T @ _yaw_only(torso))
        return replace(posed, joint_bias=self.solve(sample, posed).joint_pos,
                       joint_target=self._default_joint_pos)

    ##
    # 单帧求解
    ##

    def _body_frames(self, positions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        def point(name: str) -> np.ndarray:
            return positions[JOINT_INDEX[name]]

        # 竖直方向只用来定平面，侧向那根（左右成对的连线）才是精确保留的那一路。
        return (_frame_zy(point('SPINE1') - point('PELVIS'),
                          point('LEFT_HIP') - point('RIGHT_HIP')),
                _frame_zy(point('NECK') - point('SPINE3'),
                          point('LEFT_SHOULDER') - point('RIGHT_SHOULDER')))

    def solve(self, frame: BodyFrame, calib: RetargetCalibration) -> RetargetResult:
        positions = frame.positions * calib.scale

        def point(name: str) -> np.ndarray:
            return positions[JOINT_INDEX[name]]

        rot_pelvis, rot_torso = self._body_frames(positions)
        rot_pelvis = rot_pelvis @ calib.pelvis_fix
        rot_torso = rot_torso @ calib.torso_fix

        angles = np.zeros(len(self._names))
        yaw, roll, pitch = _decompose_zxy(rot_pelvis.T @ rot_torso)
        for name, value in zip(WAIST_JOINTS, (yaw, roll, pitch)):
            angles[self._slot[name]] = value

        for side, spec in LEGS.items():
            self._solve_limb(angles, spec, self._legs[side], point, rot_pelvis)
        for side, spec in ARMS.items():
            self._solve_limb(angles, spec, self._arms[side], point, rot_torso)

        # 站立零位映射：把校准帧的位形整体搬到 G1 的 default 上，之后按增量走。
        angles = np.clip(angles - calib.joint_bias + calib.joint_target,
                         self._lower, self._upper)

        pelvis = point('PELVIS')
        root_pos = np.array([pelvis[0], pelvis[1],
                             pelvis[2] - calib.pelvis_ref_z + calib.stand_height])
        key_local = self._kin.key_body_pos(angles, self._key_bodies)
        anchor_local = self._kin.frame_pos(self._anchor)
        anchor_rot = self._kin.frame_rot(self._anchor)
        return RetargetResult(
            t=frame.t,
            joint_pos=angles,
            root_pos=root_pos,
            root_quat=quat_from_mat(rot_pelvis),
            anchor_pos=root_pos + rot_pelvis @ anchor_local,
            anchor_quat=quat_from_mat(rot_pelvis @ anchor_rot),
            key_pos=root_pos + key_local @ rot_pelvis.T,
        )

    def _solve_limb(self, angles: np.ndarray, spec: LimbSpec, geom: LimbGeometry,
                    point, rot_root: np.ndarray) -> None:
        proximal, mid, distal, tip = (point(n) for n in spec.smpl)
        upper = mid - proximal
        lower = distal - mid
        tip_dir = _unit(tip - distal)

        hinge = geom.hinge.angle_for(_angle_between(upper, lower))
        angles[self._slot[spec.hinge_joint]] = hinge
        # 远端段实际转过的角还要带上铰链 origin 自己的那一段。
        turned = hinge + geom.hinge.placement_offset

        axis = self._hinge_axis(spec, geom, upper, lower, tip_dir, rot_root)
        rot_ball = _rotation_between(geom.proximal_dir, self._rest_axis(spec, geom, turned),
                                     _unit(upper), axis)
        local = geom.ball.pre.T @ rot_root.T @ rot_ball
        ball = np.array(_decompose_yxz(local)) - geom.ball.offsets
        for name, value in zip(spec.ball_joints, ball):
            angles[self._slot[name]] = value

        # 末端段（脚 / 手）只有一个方向可用，能定的自由度就只有两个；
        # 绕它自身的自转（踝内外翻、腕 roll）留 0。末端 link 的零位 x 轴就是脚尖 / 手尖方向，
        # 所以 kin_links 的最后一项必须取到**末端** link，取中间那个会凭空多出一段偏移角。
        local = (rot_ball @ _rot_y(turned)).T @ tip_dir
        if len(spec.tip_joints) == 2:
            angles[self._slot[spec.tip_joints[0]]] = math.atan2(-local[2], local[0])
        else:
            # Rx(roll) Ry(pitch) Rz(yaw) @ x 与 roll 无关，剩下两轴可解：
            #   local = (cos(pitch)cos(yaw), sin(yaw), -sin(pitch)cos(yaw))
            angles[self._slot[spec.tip_joints[2]]] = math.asin(
                _clamp(float(local[1]), -1.0, 1.0))
            angles[self._slot[spec.tip_joints[1]]] = math.atan2(-local[2], local[0])

    @staticmethod
    def _rest_axis(spec: LimbSpec, geom: LimbGeometry, turned: float) -> np.ndarray:
        """零位侧的铰链轴，要和实测侧用**同一个公式**算。

        G1 的上臂与前臂并不严格共面于肘轴，``cross`` 的方向会随肘角漂。两边都照
        当前肘角算，这个漂就相消了。腿不行：它零位就是直的，这个 ``cross`` 退化，
        只能回到名义上的 y 轴。
        """
        lateral = np.array([0.0, 1.0, 0.0])
        if spec.tip_defines_hinge:
            return lateral
        bent = _rot_y(turned) @ geom.distal_dir
        cross = geom.hinge.axis_sign * _cross(geom.proximal_dir, bent)
        return _unit(cross) if _norm(cross) > 1e-4 else lateral

    @staticmethod
    def _hinge_axis(spec: LimbSpec, geom: LimbGeometry, upper: np.ndarray,
                    lower: np.ndarray, tip_dir: np.ndarray, rot_root: np.ndarray) -> np.ndarray:
        """铰链转轴。它同时也是球窝那三轴里"自转"那一路的唯一约束，取错就是整条肢体拧着。

        ``cross(近端, 远端)`` 在肢体伸直时退化：G1 的大腿本身有 0.4 度侧倾，伸直时这个叉乘
        剩下的全是那点侧倾，方向直接横过来 90 度。腿上改用 ``cross(脚尖, 大腿)`` 兜住——
        它在伸直时最准，只在膝弯超过 90 度后会翻号，而那正是膝弯叉乘最可信的区间。
        两者按弯曲程度平滑过渡，不做硬切换：硬切换会在切换点上让参考抖一下。
        """
        scale = _norm(upper) * _norm(lower)
        bend_cross = geom.hinge.axis_sign * _cross(upper, lower)
        bend_norm = _norm(bend_cross) / max(scale, _EPS)
        if not spec.tip_defines_hinge:
            return (bend_cross / (bend_norm * scale)) if bend_norm > 0.05 \
                else _fallback_hinge_axis(rot_root, upper)

        tip_cross = _cross(tip_dir, upper)
        if _norm(tip_cross) < 1e-6:
            return (bend_cross / (bend_norm * scale)) if bend_norm > 1e-6 \
                else _fallback_hinge_axis(rot_root, upper)
        weight = _smoothstep(bend_norm, math.sin(0.10), math.sin(0.40))
        blended = weight * (bend_cross / max(bend_norm * scale, _EPS)) \
            + (1.0 - weight) * _unit(tip_cross)
        return _unit(blended) if _norm(blended) > 1e-6 else _unit(tip_cross)
