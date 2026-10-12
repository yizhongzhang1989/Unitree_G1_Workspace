"""旋转表示互转，不依赖 ROS。

VLA 服务用 3x3 旋转矩阵，``/motion_control/command`` 和 TF 都用 xyzw 四元数，
所以两个方向都要。服务返回的矩阵不保证严格正交，转四元数前一律先投影。
"""

from __future__ import annotations

import math

import numpy as np


def normalize_quat(quat) -> np.ndarray:
    """xyzw 四元数单位化；模长异常时抛 ``ValueError``。"""
    q = np.asarray(quat, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(q))
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError(f'四元数模长 {norm} 无效')
    return q / norm


def orthonormalize(mat) -> np.ndarray:
    """SVD 投影到最近的旋转矩阵（det=+1）。"""
    m = np.asarray(mat, dtype=np.float64).reshape(3, 3)
    if not np.all(np.isfinite(m)):
        raise ValueError('旋转矩阵含非有限值')
    u, _, vt = np.linalg.svd(m)
    if np.linalg.det(u) * np.linalg.det(vt) < 0.0:
        u[:, -1] *= -1.0
    return u @ vt


def quat_to_mat(quat) -> np.ndarray:
    """xyzw 四元数 -> 3x3 旋转矩阵。"""
    x, y, z, w = normalize_quat(quat)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(mat) -> np.ndarray:
    """3x3 旋转矩阵 -> xyzw 四元数。"""
    # Shepperd 分支法：始终从最大的那一项开方，避免 trace≈-1 时除以接近 0 的数。
    m = orthonormalize(mat)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        q = 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        q = (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        q = (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s
    return normalize_quat(q)


def quat_angle(a, b) -> float:
    """两个 xyzw 四元数之间的最小夹角，rad。"""
    dot = abs(float(np.dot(normalize_quat(a), normalize_quat(b))))
    return 2.0 * math.acos(min(1.0, dot))


def quat_slerp(a, b, t: float) -> np.ndarray:
    """球面插值，用来把单帧姿态步长夹到限幅内。"""
    q0 = normalize_quat(a)
    q1 = normalize_quat(b)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:  # 取近的那条弧，否则会绕远路转 360°-θ。
        q1, dot = -q1, -dot
    if dot > 0.9995:
        return normalize_quat(q0 + t * (q1 - q0))
    theta = math.acos(min(1.0, dot))
    sin_theta = math.sin(theta)
    return (math.sin((1.0 - t) * theta) / sin_theta) * q0 + (math.sin(t * theta) / sin_theta) * q1


def pose_matrix(quat, trans) -> np.ndarray:
    """xyzw 四元数 + 平移 -> 4x4 齐次矩阵。"""
    out = np.eye(4)
    out[:3, :3] = quat_to_mat(quat)
    out[:3, 3] = np.asarray(trans, dtype=np.float64).reshape(3)
    return out


def rpy_to_mat(rpy) -> np.ndarray:
    """URDF 惯例的 rpy（绕固定轴依次 x-y-z，即 Rz·Ry·Rx）-> 3x3 旋转矩阵。"""
    roll, pitch, yaw = (float(v) for v in np.asarray(rpy, dtype=np.float64).reshape(3))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def mat_to_rpy(mat) -> np.ndarray:
    """``rpy_to_mat`` 的逆。万向锁（|pitch|≈90°）时把自由度全归到 yaw。"""
    m = orthonormalize(mat)
    if abs(m[2, 0]) > 1.0 - 1e-9:
        pitch = math.copysign(math.pi / 2.0, -m[2, 0])
        return np.array([0.0, pitch, math.atan2(-m[0, 1], m[1, 1])])
    return np.array([math.atan2(m[2, 1], m[2, 2]),
                     math.asin(-m[2, 0]),
                     math.atan2(m[1, 0], m[0, 0])])


def invert_pose(matrix) -> np.ndarray:
    """4x4 齐次矩阵求逆（刚体，不做通用求逆）。"""
    m = np.asarray(matrix, dtype=np.float64).reshape(4, 4)
    rot = m[:3, :3].T
    out = np.eye(4)
    out[:3, :3] = rot
    out[:3, 3] = -rot @ m[:3, 3]
    return out


def solve_base_frame(camera_in_model, camera_in_base):
    """由两侧各自的「相机位姿」解出模型系相对 ``base_frame`` 的刚体变换。

    ``camera_in_model``：训练侧的 ``head_camera_in_world``（4x4），即 ``T_model←cam``。
    ``camera_in_base``：我方 TF 的 ``base_frame -> 相机光心``（4x4），即 ``T_base←cam``。

    模型的空间感建立在它自己相机的位姿上，所以相机才是锚点，与两台机器人的手臂长度无关：

        T_model←base = T_model←cam · (T_base←cam)⁻¹

    返回 ``(base_offset, base_rotation_rpy)``，即 ``FrameTransform`` 的内部约定。要拿去填
    ``vla_bridge.yaml`` 先过一道 ``vla_backend.FrameSpec.from_solution()``。
    """
    model_from_base = (np.asarray(camera_in_model, dtype=np.float64).reshape(4, 4)
                       @ invert_pose(camera_in_base))
    return model_from_base[:3, 3].copy(), mat_to_rpy(model_from_base[:3, :3])


def solve_origin_position(camera_in_model, camera_in_base) -> np.ndarray:
    """只对齐**位置**：模型系保持与 ``base_frame`` 同朝向，只把原点挪到两台相机重合。

    直接返回 ``FrameSpec.origin_in_base``。与 :func:`solve_base_frame` 的取舍：

    * 完整解让相机位姿完全重合，但模型系会被掰斜（两台相机朝向差多少就斜多少），
      于是重力方向在模型系里是错的，末端 state 也跟着歪。
    * 只对位置则保住「模型系水平朝前」，代价是相机朝向仍差一个角度。

    模型对**位置**远比对朝向敏感（朝向差可以靠图像重投影和模型自身的鲁棒性吸收），
    所以默认走这一条。
    """
    return (np.asarray(camera_in_base, dtype=np.float64).reshape(4, 4)[:3, 3]
            - np.asarray(camera_in_model, dtype=np.float64).reshape(4, 4)[:3, 3])


class FrameTransform:
    """``base_frame`` 的末端位姿 <-> VLA 模型系的末端位姿。

    ``p_model = R_base · (p + R · tool_offset) + base_offset``
    ``R_model = R_base · R · R_tool``

    ``base_*`` 只描述两台机器人「底座系」之间的固定差。身体高度、俯仰这些会变的量
    **不在这里**——它们走 ``head_camera_in_world`` 外参，训练时也是这么编码的。
    ``tool_*`` 是我方 tip frame 到模型 joint7 frame 的固定变换。
    """

    def __init__(self, base_offset=(0.0, 0.0, 0.0), tool_rotation_rpy=(0.0, 0.0, 0.0),
                 tool_offset=(0.0, 0.0, 0.0), base_rotation_rpy=(0.0, 0.0, 0.0)) -> None:
        self.base_offset = np.asarray(base_offset, dtype=np.float64).reshape(3)
        self.tool_offset = np.asarray(tool_offset, dtype=np.float64).reshape(3)
        self.tool_rot = rpy_to_mat(tool_rotation_rpy)
        self.base_rot = rpy_to_mat(base_rotation_rpy)

    def to_model(self, pose) -> tuple[np.ndarray, np.ndarray]:
        """``[x,y,z,qx,qy,qz,qw]`` -> 模型系的 ``(trans(3,), rot(3,3))``。"""
        pose = np.asarray(pose, dtype=np.float64).reshape(7)
        rot = quat_to_mat(pose[3:])
        trans = self.base_rot @ (pose[:3] + rot @ self.tool_offset) + self.base_offset
        return trans, self.base_rot @ rot @ self.tool_rot

    def from_model(self, trans, rot) -> np.ndarray:
        """模型系的 ``(trans, rot)`` -> ``[x,y,z,qx,qy,qz,qw]``。"""
        rot_base = self.base_rot.T @ orthonormalize(rot) @ self.tool_rot.T
        out = np.empty(7)
        out[:3] = (self.base_rot.T
                   @ (np.asarray(trans, dtype=np.float64).reshape(3) - self.base_offset)
                   - rot_base @ self.tool_offset)
        out[3:] = mat_to_quat(rot_base)
        return out

    def base_to_model(self, matrix) -> np.ndarray:
        """``base_frame`` 里的任意 4x4 位姿 -> 模型系。不带 ``tool_*``，给相机用。"""
        matrix = np.asarray(matrix, dtype=np.float64).reshape(4, 4)
        out = np.eye(4)
        out[:3, :3] = self.base_rot @ matrix[:3, :3]
        out[:3, 3] = self.base_rot @ matrix[:3, 3] + self.base_offset
        return out


def reanchor(poses: np.ndarray, anchor: np.ndarray,
             position: bool = True, rotation: bool = True) -> np.ndarray:
    """把一段绝对轨迹改成「相对首点」的形状，再叠到 ``anchor`` 上。

    位置和姿态可以分开选：位置用增量时 ``base_offset``/``tool_offset`` 完全抵消，
    而姿态的标定（``tool_rotation_rpy`` 那道绕 Z 的 180°）是确定的，没必要一起重锚。
    **位置用增量也不能省 ``base_rotation_rpy``**：Δp_model = R_base · Δp_base，方向不抵消。
    """
    out = poses.copy()
    if position:
        out[:, :3] = anchor[:3] + poses[:, :3] - poses[0, :3]
    if rotation:
        rot0 = quat_to_mat(poses[0, 3:]).T
        anchor_rot = quat_to_mat(anchor[3:])
        for i in range(poses.shape[0]):
            out[i, 3:] = mat_to_quat(quat_to_mat(poses[i, 3:]) @ rot0 @ anchor_rot)
    return out
