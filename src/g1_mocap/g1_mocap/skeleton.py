"""PicoBridge 报文 -> 机器人坐标系下的 24 关节骨架。不依赖 ROS，可离线单测。

**坐标系**：PicoBridge 走 OpenXR 右手系（X 右、Y 上、−Z 前），机器人这边是
X 前、Y 左、Z 上。换算就是 ``(x, y, z) -> (-z, -x, y)``，写成矩阵即 :data:`XR_TO_ROBOT`。
只在这一层换一次，往后全是机器人坐标系。报文里的关节朝向一律不取，原因见
:mod:`~g1_mocap.retarget`。

**时钟**：帧里的 ``t`` 是头显的 ``predictedDisplayTime``，抖动远小于 WiFi 到达时刻。
用它做帧间相对时间、用首帧的本地时刻做锚点，可以把网络抖动挡在参考窗口之外；
头显重启会让 ``t`` 跳变，:class:`ClockAligner` 检测到就重锚。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# 顺序即 XrBodyJointBD 的枚举值 0..23，不能重排：下面的成对下标全按这个顺序取。
SMPL_JOINTS = (
    'PELVIS', 'LEFT_HIP', 'RIGHT_HIP', 'SPINE1',
    'LEFT_KNEE', 'RIGHT_KNEE', 'SPINE2', 'LEFT_ANKLE',
    'RIGHT_ANKLE', 'SPINE3', 'LEFT_FOOT', 'RIGHT_FOOT',
    'NECK', 'LEFT_COLLAR', 'RIGHT_COLLAR', 'HEAD',
    'LEFT_SHOULDER', 'RIGHT_SHOULDER', 'LEFT_ELBOW', 'RIGHT_ELBOW',
    'LEFT_WRIST', 'RIGHT_WRIST', 'LEFT_HAND', 'RIGHT_HAND',
)
JOINT_INDEX = {name: i for i, name in enumerate(SMPL_JOINTS)}

# OpenXR (X 右, Y 上, -Z 前) -> 机器人 (X 前, Y 左, Z 上)。det = +1，是纯旋转。
XR_TO_ROBOT = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])

STATUS_INVALID, STATUS_VALID, STATUS_LIMITED = 0, 1, 2

# XrBodyTrackingMessagePICO，只在日志里给人看。
STATUS_MESSAGES = {
    0: '正常',
    1: 'tracker 未校准',
    2: 'tracker 数量不足',
    3: 'tracker 状态不满足',
    4: 'tracker 长时间不可见',
    5: 'tracker 数据异常',
    6: '用户变更',
    7: '追踪姿态错误（头显没戴上时常见）',
}


@dataclass(frozen=True)
class BodyFrame:
    """一帧全身骨架。``positions`` 是 (24, 3)，机器人坐标系，米，未缩放。

    ``rotations`` 是 (24, 3, 3)，同一坐标系下每个关节的局部系。拿不到时为 ``None``，
    重定向会退化成纯位置解法（绕骨骼自转的那一路就留 0）。
    """

    t: float
    seq: int
    positions: np.ndarray
    status: int
    message: int
    rotations: np.ndarray | None = None

    @property
    def usable(self) -> bool:
        """LIMITED 也放行：精度降级但数值仍然连续，比直接断流强。"""
        return self.status in (STATUS_VALID, STATUS_LIMITED)


def parse_body(payload: dict) -> BodyFrame | None:
    """从一帧 PicoBridge JSON 里取出骨架。任何一处不合规就返回 ``None``。

    报文来自网络，字段缺失、类型错误、非有限值都当作坏帧丢掉，不抛异常——
    收帧线程里抛异常等于断流。
    """
    if not isinstance(payload, dict):
        return None
    body = payload.get('body')
    if not isinstance(body, dict):
        return None
    joints = body.get('joints')
    if not isinstance(joints, dict):
        return None

    rows = []
    quats = []
    for name in SMPL_JOINTS:
        joint = joints.get(name)
        if not isinstance(joint, dict) or not joint.get('position_valid', True):
            return None
        position = joint.get('position')
        if not isinstance(position, (list, tuple)) or len(position) != 3:
            return None
        rows.append(position)
        orientation = joint.get('orientation')
        if (joint.get('orientation_valid', True)
                and isinstance(orientation, (list, tuple)) and len(orientation) == 4):
            quats.append(orientation)
        else:
            quats.append(None)
    try:
        # 一次性建数组；逐行写进预分配的 (24, 3) 要走 24 次 numpy setitem，慢一倍。
        positions = np.array(rows, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if not np.all(np.isfinite(positions)):
        return None
    rotations = _rotations(quats)

    try:
        t = float(payload.get('t', 0.0))
        seq = int(payload.get('seq', 0))
        status = int(body.get('status', STATUS_INVALID))
        message = int(body.get('message', 0))
    except (TypeError, ValueError):
        return None
    if not math.isfinite(t):
        return None

    return BodyFrame(t=t, seq=seq, positions=positions @ XR_TO_ROBOT.T,
                     status=status, message=message, rotations=rotations)


def _rotations(quats: list) -> np.ndarray | None:
    """一批 OpenXR 的 xyzw 四元数 -> 机器人系下的 (24, 3, 3)。缺一个就整帧不用。

    矩阵的列是**关节局部轴在世界系里的分量**，换世界系只需左乘：``M R``。
    写成相似变换 ``M R Mᵀ`` 就把关节自己的局部系也一起转了，绕骨骼轴的分量会落到
    另一根轴上去。

    不用 :func:`~.rotations.quat_to_mat`：那个只吃单个四元数、而且非法时抛异常。
    这里是收帧线程的热路径（每帧 24 个、最高 90 Hz），而且坏报文要返回 None 而不是断流。
    """
    if any(q is None for q in quats):
        return None
    try:
        q = np.array(quats, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if q.shape != (len(SMPL_JOINTS), 4) or not np.all(np.isfinite(q)):
        return None
    norm = np.linalg.norm(q, axis=1, keepdims=True)
    if np.any(norm < 1e-6):
        return None
    x, y, z, w = (q / norm).T
    rot = np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1),
    ], -2)
    return XR_TO_ROBOT @ rot


def both_thumbsticks_pressed(payload: dict) -> bool:
    """双手摇杆是否同时按下。

    戴着头显没法去点网页上的按钮，这个组合键就是「原地校准」的触发器。选双摇杆
    而不是扳机/AB，是因为它需要两只手一起动，误触的代价（重标一次人机比例）
    又只在站姿不对时才有害。
    """
    if not isinstance(payload, dict):
        return False
    for hand in ('left', 'right'):
        controller = payload.get(hand)
        if not isinstance(controller, dict) or not controller.get('connected'):
            return False
        buttons = controller.get('buttons')
        if not isinstance(buttons, dict) or not buttons.get('thumbstick_pressed'):
            return False
    return True


class ClockAligner:
    """把发送端时间戳映射到本地单调时钟。

    直接拿到达时刻当采样时刻，WiFi 的几毫秒抖动会原样进到参考窗口的速度差分里
    （50 Hz 下 5 ms 抖动就是 25% 的速度误差）。这里改成锚点 + 发送端增量，
    只有锚点那一次吃到达抖动。
    """

    def __init__(self, *, resync_threshold_s: float = 0.25) -> None:
        self._threshold = float(resync_threshold_s)
        self._remote0 = 0.0
        self._local0 = 0.0
        self._primed = False

    def stamp(self, remote_t: float, local_t: float) -> float:
        if not self._primed:
            self._remote0, self._local0, self._primed = remote_t, local_t, True
            return local_t
        mapped = self._local0 + (remote_t - self._remote0)
        # 超出阈值只有两种可能：头显重启把 t 归了零，或者本地被长时间挂起。
        # 两种都得重锚——继续外推会让整条时间轴恒定偏移。
        if abs(mapped - local_t) > self._threshold:
            self._remote0, self._local0 = remote_t, local_t
            return local_t
        return mapped
