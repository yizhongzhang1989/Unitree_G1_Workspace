#!/usr/bin/env python3
"""URDF -> 正运动学。**只用 stdlib + numpy**。

导出机没有 ROS，也就没有 pinocchio，而 `end_space` 的末端位姿和腕相机的外参都得
从关节角推。这里只做导出用得到的那一件事：给一条 root -> leaf 的链，按时间批量
求 4x4 位姿。不算雅可比、不算动力学，别拿它当运动学库用。

标定出来的 origin 不在 URDF 里 —— `unitree_g1_description/model/final.urdf` 是
submodule，改了会和上游分叉。控制栈是在 launch 里把 `calibration.yaml` 的
`urdf_overrides` 叠到内存中的 URDF 上，这里必须做同一件事，否则导出的外参和机器人
当时实际用的差一截::

    model = RobotModel.from_urdf(urdf_path)
    model.apply_overrides(calibration['urdf_overrides'])
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

MOVING = ('revolute', 'continuous', 'prismatic')


@dataclass(frozen=True)
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray                       # 4x4，T_parent<-child 在零位时的值
    axis: np.ndarray                         # 3，fixed 关节忽略
    mimic: tuple[str, float, float] | None = None   # (源关节, multiplier, offset)


def rpy_to_matrix(rpy) -> np.ndarray:
    """URDF 的 rpy 是固定轴 XYZ，等价于 Rz(y) @ Ry(p) @ Rx(r)"""
    r, p, y = (float(v) for v in rpy)
    cr, sr, cp, sp, cy, sy = (np.cos(r), np.sin(r), np.cos(p),
                              np.sin(p), np.cos(y), np.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def origin_matrix(xyz, rpy) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = rpy_to_matrix(rpy)
    matrix[:3, 3] = [float(v) for v in xyz]
    return matrix


def rotate(axis, angle) -> np.ndarray:
    """绕 axis 转 angle 的 (N,4,4)。angle 可以是标量或一维数组。"""
    angle = np.atleast_1d(np.asarray(angle, float))
    axis = np.asarray(axis, float)
    norm = np.linalg.norm(axis)
    axis = axis / norm if norm > 0 else np.array([0.0, 0.0, 1.0])
    skew = np.array([[0.0, -axis[2], axis[1]],
                     [axis[2], 0.0, -axis[0]],
                     [-axis[1], axis[0], 0.0]])
    sin = np.sin(angle)[:, None, None]
    cos = np.cos(angle)[:, None, None]
    out = np.tile(np.eye(4), (angle.size, 1, 1))
    out[:, :3, :3] = np.eye(3) + sin * skew + (1.0 - cos) * (skew @ skew)
    return out


def translate(axis, distance) -> np.ndarray:
    distance = np.atleast_1d(np.asarray(distance, float))
    axis = np.asarray(axis, float)
    norm = np.linalg.norm(axis)
    axis = axis / norm if norm > 0 else np.array([0.0, 0.0, 1.0])
    out = np.tile(np.eye(4), (distance.size, 1, 1))
    out[:, :3, 3] = distance[:, None] * axis
    return out


def invert(matrix) -> np.ndarray:
    """刚体变换的逆。比 np.linalg.inv 稳，也不会在退化时给出离谱结果。"""
    matrix = np.asarray(matrix, float)
    rotation = np.swapaxes(matrix[..., :3, :3], -1, -2)
    out = np.zeros_like(matrix)
    out[..., :3, :3] = rotation
    out[..., :3, 3] = -np.einsum('...ij,...j->...i', rotation, matrix[..., :3, 3])
    out[..., 3, 3] = 1.0
    return out


def matrix_to_quat(matrix) -> np.ndarray:
    """(...,4,4) -> (...,4) 的 [qx, qy, qz, qw]，w 恒非负。

    四个候选式分别以 w/x/y/z 为主元，取被开方项最大的那个 —— 单主元公式在对应
    分量接近零时要除以一个接近零的数，180° 附近直接崩。
    """
    rot = np.asarray(matrix, float)[..., :3, :3]
    m = [[rot[..., i, j] for j in range(3)] for i in range(3)]
    trace = m[0][0] + m[1][1] + m[2][2]
    pivots = np.stack([1.0 + trace,
                       1.0 + m[0][0] - m[1][1] - m[2][2],
                       1.0 - m[0][0] + m[1][1] - m[2][2],
                       1.0 - m[0][0] - m[1][1] + m[2][2]], axis=-1)
    candidates = np.stack([
        np.stack([m[2][1] - m[1][2], m[0][2] - m[2][0],
                  m[1][0] - m[0][1], pivots[..., 0]], axis=-1),
        np.stack([pivots[..., 1], m[0][1] + m[1][0],
                  m[0][2] + m[2][0], m[2][1] - m[1][2]], axis=-1),
        np.stack([m[0][1] + m[1][0], pivots[..., 2],
                  m[1][2] + m[2][1], m[0][2] - m[2][0]], axis=-1),
        np.stack([m[0][2] + m[2][0], m[1][2] + m[2][1],
                  pivots[..., 3], m[1][0] - m[0][1]], axis=-1),
    ], axis=-2)
    pick = np.argmax(pivots, axis=-1)
    quat = np.take_along_axis(candidates, pick[..., None, None], axis=-2)[..., 0, :]
    quat = quat / np.linalg.norm(quat, axis=-1, keepdims=True)
    return np.where(quat[..., 3:4] < 0.0, -quat, quat)


def matrix_to_pose(matrix) -> np.ndarray:
    """(...,4,4) -> (...,7) 的 [tx, ty, tz, qx, qy, qz, qw]，即数据格式要的那 7 个"""
    matrix = np.asarray(matrix, float)
    return np.concatenate([matrix[..., :3, 3], matrix_to_quat(matrix)], axis=-1)


def quat_multiply(a, b) -> np.ndarray:
    """Hamilton 积，xyzw 存法。等价于旋转矩阵相乘 R(a) @ R(b)。"""
    a, b = np.asarray(a, float), np.asarray(b, float)
    ax, ay, az, aw = (a[..., i] for i in range(4))
    bx, by, bz, bw = (b[..., i] for i in range(4))
    return np.stack([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ], axis=-1)


class RobotModel:
    def __init__(self, joints) -> None:
        self.joints = {j.name: j for j in joints}
        self._by_child = {j.child: j for j in joints}

    @classmethod
    def from_urdf(cls, source) -> 'RobotModel':
        text = source if '<' in str(source)[:200] else Path(source).read_text(encoding='utf-8')
        root = ET.fromstring(text)
        return cls([_parse_joint(node) for node in root.iter('joint')])

    def apply_overrides(self, overrides: dict) -> list[str]:
        """把 calibration.yaml 的 urdf_overrides 叠上去，返回生效的关节名。

        和 `unitree_g1_ros2_control/launch/control.launch.py` 是同一套判据：存的是
        T_parent<-child，parent/child 和标定时对不上就整条跳过，不硬套。
        """
        applied = []
        for name, entry in (overrides or {}).items():
            if 'xyz' not in entry or 'rpy' not in entry:
                continue
            origin = origin_matrix(entry['xyz'], entry['rpy'])
            existing = self.joints.get(name)
            if existing is None:
                if not entry.get('create') or not entry.get('parent') or not entry.get('child'):
                    continue
                joint = Joint(name, 'fixed', entry['parent'], entry['child'],
                              origin, np.array([0.0, 0.0, 1.0]))
            else:
                if any(entry.get(k) and entry[k] != getattr(existing, k)
                       for k in ('parent', 'child')):
                    continue
                joint = replace(existing, origin=origin)
            self.joints[name] = joint
            self._by_child[joint.child] = joint
            applied.append(name)
        return applied

    def chain(self, root: str, leaf: str) -> list[Joint]:
        """root -> leaf 的关节序列。root 不是 leaf 的祖先就抛错，不返回半条链。"""
        out, link = [], leaf
        while link != root:
            joint = self._by_child.get(link)
            if joint is None:
                raise ValueError(f'{root} 不是 {leaf} 的祖先（在 {link} 处到顶）')
            out.append(joint)
            link = joint.parent
        return out[::-1]

    def moving_joints(self, root: str, leaf: str) -> list[str]:
        """这条链上真正要给角度的关节名。

        mimic 关节本身不算，但它的**源关节可能不在这条链上**（夹爪就是这样：
        指节 mimic 的是 `*_eccentric_joint`，而那根轴挂在 gripper_base 的另一支）。
        漏了源关节，`poses` 会在中途抛 KeyError。
        """
        out = []
        for joint in self.chain(root, leaf):
            if joint.kind not in MOVING:
                continue
            name = joint.mimic[0] if joint.mimic else joint.name
            if name not in out:
                out.append(name)
        return out

    def poses(self, root: str, leaf: str, values: dict) -> np.ndarray:
        """(N,4,4) 的 T_root<-leaf。values 里每个关节给标量或长度 N 的数组。"""
        chain = self.chain(root, leaf)
        count = max((np.atleast_1d(np.asarray(v, float)).size
                     for j in chain for v in [values.get(j.name)] if v is not None),
                    default=1)
        out = np.tile(np.eye(4), (count, 1, 1))
        for joint in chain:
            out = out @ self._transform(joint, values, count)
        return out

    def _transform(self, joint: Joint, values: dict, count: int) -> np.ndarray:
        if joint.kind not in MOVING:
            return joint.origin[None]
        if joint.mimic is not None:
            source, multiplier, offset = joint.mimic
            angle = _value(values, source, joint.name, count) * multiplier + offset
        else:
            angle = _value(values, joint.name, joint.name, count)
        step = (translate if joint.kind == 'prismatic' else rotate)(joint.axis, angle)
        return joint.origin[None] @ step


def _value(values: dict, key: str, requester: str, count: int) -> np.ndarray:
    """缺关节角必须抛错。补 0 会给出一个看着完全正常、但整条链都错的位姿。"""
    if key not in values:
        raise KeyError(f'{requester} 需要关节 {key} 的角度，但 values 里没有')
    return np.broadcast_to(np.atleast_1d(np.asarray(values[key], float)), (count,))


def _parse_joint(node) -> Joint:
    origin = node.find('origin')
    xyz = (origin.get('xyz') if origin is not None else None) or '0 0 0'
    rpy = (origin.get('rpy') if origin is not None else None) or '0 0 0'
    axis = node.find('axis')
    mimic = node.find('mimic')
    return Joint(
        name=node.get('name'),
        kind=node.get('type'),
        parent=node.find('parent').get('link'),
        child=node.find('child').get('link'),
        origin=origin_matrix(xyz.split(), rpy.split()),
        axis=np.array([float(v) for v in ((axis.get('xyz') if axis is not None
                                           else None) or '0 0 1').split()]),
        mimic=None if mimic is None else (
            mimic.get('joint'), float(mimic.get('multiplier', 1.0)),
            float(mimic.get('offset', 0.0))),
    )
