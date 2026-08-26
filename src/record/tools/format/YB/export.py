#!/usr/bin/env python3
"""session -> **YB** 数据集。一条 episode 一个 h5 + 每路相机一个 mp4。

格式规范看旁边的 `README.md` —— **交数据给别人时连那份一起给**。
本文件只讲怎么跑和为什么这么实现。

在导出机上跑，除 numpy 外只多 h5py 和 PyYAML（导视频还要 ffmpeg）::

    python export.py <session 目录> -o <输出目录> \\
        --urdf final.urdf --calibration calibration.yaml

`--dry-run` 不写文件，只把每条 episode 会写出什么形状印出来，用来核对口径。

## 目录布局

对齐对面给的 `robotwin_samples/`：视频按相机各放一个目录，和 h5 同名。

    meta.json                    数据集级（机器人、夹爪约定、规模）
    data/<name>.h5
    episode/<name>.json          每条的帧区间与指令
    episodes_all.json
    video_head/<name>.mp4
    video_wrist_left/<name>.mp4
    video_wrist_right/<name>.mp4

**h5 的行数就是 mp4 的帧数**，第 k 帧就是第 k 行那一刻看到的画面。
全部统一到 30 Hz，连带视频一起重采样（见 `tools/resample_video.py`）。

## 为什么要 URDF 和 calibration.yaml

`end_space` 的末端位姿、腕相机的外参都得靠 FK 从关节角推，而机器人当时用的 URDF
是「final.urdf 叠上 calibration.yaml 的 urdf_overrides」——
`unitree_g1_ros2_control/launch/control.launch.py` 就是这么拼的，这里必须一致。
两个文件都不在 session 里，得跟着 `tools/` 一起拷到导出机。

> 后续应当在开录时把展开后的 `robot_description` 快照进 session。那样导出不依赖
> 外部文件，也不会因为事后重标定而算出与当时不符的外参。

## extrinsic 的方向

默认写 `base_T_cam`，定义就是这一行：

    world_xyz = extrinsic @ camera_xyz

即把相机系的点变到世界系，也就是相机在世界系下的位姿。对面给的示例也是这个方向。

别用「world2camera」这类叫法当凭据 —— CV 圈和机器人圈把它指向相反的两边，已经
因此弄反过一回。`--extrinsic cam_T_base` 可以翻（那时
`camera_xyz = extrinsic @ world_xyz`）；无论哪种，方向和公式都会写进
`meta/camera_space/extrinsic_direction` 与 `extrinsic_formula`。

两者只差一个逆，弄反了不报错。自查的办法：腕相机是拧在夹爪上的，把它的平移
列直接当相机位置去量到末端的距离，**必须是常数**；实测本机是 7.4 / 7.7 cm。

## 还没定的口径

- **图像不进 h5**，跟示例一样另存 mp4。h5 里留
  `state/camera_space/frame_index`（源 mkv 里的帧号，−1 = 那一刻没帧），万一要
  回溯到原始素材能对上。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from session_reader import Session                        # noqa: E402
import resample_video                                     # noqa: E402
import urdf_fk                                            # noqa: E402

VERSION = '0.1'
#: 这个导出格式的名字。写进 meta.json，拿到数据的人才知道该查哪份文档。
FORMAT = 'YB'
#: 规范正文相对仓库 src/ 的位置，同样写进 meta.json。
FORMAT_DOC = 'record/tools/format/YB/README.md'
DEFAULT_HZ = 30.0
#: 超过这么久没有新样本，该时刻就判为无效而不是继续保持上一个值
DEFAULT_MAX_AGE_S = 0.1

#: 动作前后各保留多少秒空隙。操作者要先回去按「开始」、做完再回去按「成功」，
#: 两头都挂着一大段机器人不动的画面。实测一次 13 条 episode 的采集：
#: 头部空隙中位 2.5 s、尾部 3.2 s，**183 s 里有 77 s 是空的（42%）**。
DEFAULT_KEEP_IDLE_S = 1.0
#: 判「目标在动」的阈值与观察窗。逐点差分会被 50 Hz 的量化噪声淹没，所以看 ±0.1 s。
#: 位姿阈值不敏感：实测静止时目标位姿速度是**精确的 0**（p05 = 0.00 mm/s），
#: 操作时 p95 在 110~230 mm/s，中间隔着四个数量级。
IDLE_POS_M_S = 0.005
IDLE_WINDOW_S = 0.2
#: 夹爪则看**窗口内的幅度**而不是速率：VR 扬机是模拟量，手指碰一下就能
#: 在 0.19 s 里跑出 0.054 rad（行程的 2%）= 0.29 rad/s，按速率判会当成真动作，
#: 实测把一条 episode 的头部少裁了 2.3 s。真的开合一定走完全行程，
#: 峰值窗口幅度 p50 = 1.18 rad，高出一个数量级。
IDLE_GRIP_RAD = 0.28                # ≈ 行程的 10%
#: FK 的根，也是末端位姿与相机外参的参考系
ORIGIN = 'torso_link'
WORLD_AXES = '+X forward, +Y left, +Z up (REP-103)'

#: 夹爪偏心轴的行程（rad），取自 `g1_motion_control/config/motion_control.yaml`
#: 的 `gripper_limits`。**0 是闭合、2.76 是张开**，和对面 meta.json 要的
#: 「0=open、1=closed」正好相反，所以归一化时要取补。
GRIPPER_TRAVEL_RAD = 2.76377472169236
HEAD_OPTICAL_FRAME = 'camera_color_optical_frame'

#: 末端统一约定：**+Z 接近方向（夹爪尾→头）、+X 朝腕相机那侧、+Y 开合**，
#: 右手系所以 Y = Z × X。
#:
#: `*_gripper_base` 实测是另一种摆法（从 URDF 量的，左右一样、没镜像）：
#:   两个指沿 **±X** 开合（张开 ±41.1 mm、夹紧 ±17.3 mm）
#:   指尖在 **+Z** 61 mm，与 `motion_control.yaml` 的 `tool_axis: gripper_base +Z` 一致
#:   标定出的腕相机在 **+Y** 43 mm
#: 所以 Z 已经对了，X 和 Y 要对换：绕 Z 转 +90°（x_new = y_old、y_new = −x_old）。
#: 用法和对面示例的 `ee_convention` 一致：q_unified = q_raw ⊗ R_fix，xyz 不变。
EE_CONVENTION = 'approach_z_closing_y'
EE_FIX_QUAT_XYZW = (0.0, 0.0, 0.7071067811865476, 0.7071067811865476)

#: `extrinsic` 的两个方向及其定义式。存公式而不只存名字：「world2camera」这类叫法
#: CV 圈和机器人圈指向相反，已经因此弄反过一回。
EXTRINSIC_FORMULA = {
    'base_T_cam': 'world_xyz = extrinsic @ camera_xyz',
    'cam_T_base': 'camera_xyz = extrinsic @ world_xyz',
}

#: 三路相机的 frame 已经就是这个约定，导出不需要再转 —— 头部是 ROS 光学帧，
#: 两个腕相机的 frame 是 solvePnP/calibrateHandEye 定义的 OpenCV 相机帧。
#: 零位实测（在 torso_link 下）：+X → 机器人右侧（图像向右）、+Y → 向下、+Z → 向前。
CAMERA_AXES = '+X image right, +Y image down, +Z forward into the scene (OpenCV)'

#: 头部彩色镜头相对 `d435_link` 的位姿。URDF 里没有这一段 —— 它由 realsense-ros
#: 从设备出厂标定里读出来发成 TF。2026-08-25 从跑着的栈上取的全精度值。
#: **换一台 D435i 就要重取**；session 的 meta.json 里有 `head_optical` 时以它为准。
HEAD_OPTICAL = {
    'xyz': [-0.0008463825797662139, 0.015250945463776588, -2.4963012037915178e-05],
    'quat': [-0.4998616034745607, 0.5031675567847118,
             -0.49744304340790907, 0.4995109665825387],
}


@dataclass(frozen=True)
class CameraSpec:
    name: str          # 导出用的名字，也是 video_<name>/ 目录名
    frame: str         # URDF 里的叶子 link
    calib: str         # calibration.yaml 的 intrinsics 键；空 = 用 meta.json
    static: bool       # 外参在一条 episode 里是否恒定


CAMERAS = (
    CameraSpec('head', HEAD_OPTICAL_FRAME, '', True),
    CameraSpec('wrist_left', 'camera_left', 'camera_left', False),
    CameraSpec('wrist_right', 'camera_right', 'camera_right', False),
)

#: 前 29 个是 G1 本体关节，最后两个 eccentric 是夹爪 —— 它们进 actuator_space。
#: 规范把夹爪明确划在 actuator 而不是 joint（例 2/例 4 都是这么分的）。
GRIPPER_JOINTS = ('left_eccentric_joint', 'right_eccentric_joint')
ARMS = ('left', 'right')
END_LINKS = {'left': 'left_gripper_base', 'right': 'right_gripper_base'}

_ROLE_RULES = (
    ('left_leg', ('left_hip', 'left_knee', 'left_ankle')),
    ('right_leg', ('right_hip', 'right_knee', 'right_ankle')),
    ('waist', ('waist_',)),
    ('left_arm', ('left_shoulder', 'left_elbow', 'left_wrist')),
    ('right_arm', ('right_shoulder', 'right_elbow', 'right_wrist')),
)


# --------------------------------------------------------------------- 时间栅格


def build_grid(t0: float, t1: float, hz: float) -> np.ndarray:
    """[t0, t1] 上的等间隔时刻。两端都含，最后一个不超过 t1。"""
    count = int(np.floor((t1 - t0) * hz)) + 1
    return t0 + np.arange(max(count, 1)) / hz


def hold(t_src: np.ndarray, data: np.ndarray, grid: np.ndarray,
         max_age: float) -> tuple[np.ndarray, np.ndarray]:
    """零阶保持采样。返回 (值, 有效位)。

    **不插值。** 关节角插值看着无害，但四元数、离散状态位、指令都不能线性插；
    与其一个字段一个规则，不如统一保持上一个真实样本，超过 `max_age` 就判无效。
    无效处填 NaN 而不是 0 —— 0 是一个合法的关节角，下游看不出是缺的。
    """
    grid = np.asarray(grid, float)
    data = np.atleast_2d(np.asarray(data, float))
    out = np.full((grid.size, data.shape[1]), np.nan)
    if t_src.size == 0:
        return out, np.zeros(grid.size, bool)
    index = np.searchsorted(t_src, grid, side='right') - 1
    valid = index >= 0
    index = np.clip(index, 0, t_src.size - 1)
    valid &= (grid - t_src[index]) <= max_age
    out[valid] = data[index[valid]]
    return out, valid


def arrived(t_src: np.ndarray, grid: np.ndarray) -> np.ndarray:
    """每个栅格点之前一个周期内有没有新消息。

    这就是规范给 action 的 `valid_mask` 的语义：「指令是什么时候发出的」。
    指令不是每个 tick 都发，用保持后的有效位会把它标成一直有效。
    """
    if t_src.size == 0 or grid.size == 0:
        return np.zeros(grid.size, bool)
    step = float(grid[1] - grid[0]) if grid.size > 1 else 0.0
    lo = np.searchsorted(t_src, grid - step, side='right')
    return np.searchsorted(t_src, grid, side='right') > lo


def frame_index(pts: np.ndarray, grid: np.ndarray, max_age: float) -> np.ndarray:
    """每个栅格时刻对应哪一帧。−1 = 那一刻没有帧。

    `pts` 用 `Session.video_pts()`，它已经把相机管线延迟减掉了。

    查表前先 `maximum.accumulate` 而不是排序：`searchsorted` 要求输入有序，而排序会
    打乱下标 —— 返回的必须是 **mkv 里的帧号**。实测头部第一帧的 RealSense 戳比第二帧
    晚一个帧周期，只有开头这一处；夹平后下标仍然指向真实的帧。
    """
    if pts.size == 0:
        return np.full(grid.size, -1, np.int32)
    index = np.searchsorted(np.maximum.accumulate(pts), grid, side='right') - 1
    valid = index >= 0
    index = np.clip(index, 0, pts.size - 1)
    valid &= (grid - pts[index]) <= max_age
    return np.where(valid, index, -1).astype(np.int32)


# ------------------------------------------------------------------------- 模型


def load_model(urdf: Path, calibration: Path | None) -> tuple:
    """URDF 叠标定，再补上 URDF 里没有的头部光心那一段。"""
    model = urdf_fk.RobotModel.from_urdf(urdf)
    calib = _yaml(calibration) if calibration else {}
    applied = model.apply_overrides(calib.get('urdf_overrides'))
    return model, calib, applied


def add_head_optical(model, optical: dict) -> None:
    """把 d435_link -> camera_color_optical_frame 挂进模型。

    这一段不在 URDF 里，是 realsense-ros 从设备出厂标定读出来发的 TF。彩色镜头
    偏离挂载原点 15.3 mm，当成纯旋转会让投影整体横移十几个像素。
    """
    matrix = np.eye(4)
    matrix[:3, :3] = _quat_to_matrix(optical['quat'])
    matrix[:3, 3] = optical['xyz']
    joint = urdf_fk.Joint('head_optical_joint', 'fixed', 'd435_link',
                          HEAD_OPTICAL_FRAME, matrix, np.array([0.0, 0.0, 1.0]))
    model.joints[joint.name] = joint
    model._by_child[joint.child] = joint


def _quat_to_matrix(quat) -> np.ndarray:
    """(...,4) 的 xyzw -> (...,3,3)。传单个四元数就得到一个 3x3。"""
    quat = np.asarray(quat, float)
    x, y, z, w = (quat[..., i] for i in range(4))
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1),
    ], axis=-2)


# ------------------------------------------------------------------- 各 space


def joint_space(session: Session, grid, order, max_age, gripper_index) -> dict:
    """state 用 joint_states 实测，action 用 fpc_commands —— 关节级指令的唯一出口"""
    keep = [i for i in range(len(order)) if i not in gripper_index]
    count = len(order)
    t, data = read_table(session, 'joint_states', 3 * count)
    state = {}
    for name, offset in (('position', 0), ('velocity', 1), ('effort', 2)):
        block = data[:, offset * count:(offset + 1) * count]
        # 三者同表同时刻，有效位是同一个，取最后一次即可
        state[name], state['valid_mask'] = hold(t, block[:, keep], grid, max_age)

    ta, cmd = read_table(session, 'fpc_commands', 31 + 14 + 14)
    position, _ = hold(ta, cmd[:, keep], grid, max_age)
    return {
        'names': [order[i] for i in keep],
        'roles': _roles([order[i] for i in keep]),
        'state': state,
        'action': {'position': position, 'valid_mask': arrived(ta, grid)},
    }


def actuator_space(session: Session, grid, order, max_age, gripper_index) -> dict:
    """夹爪。state 走 joint_states 里的 eccentric 轴，和关节同一个时间基。"""
    t, data = read_table(session, 'joint_states', 3 * len(order))
    value, valid = hold(t, data[:, list(gripper_index)], grid, max_age)

    tc, cmd = read_table(session, 'motion_control_command', 20)
    command, _ = hold(tc, cmd[:, 18:20], grid, max_age)      # base4+left7+right7+grip2
    return {
        'names': [f'{side}_gripper' for side in ARMS],
        'types': ['gripper'] * len(ARMS),
        'source_joints': [order[i] for i in gripper_index],
        'state': {'value': normalize_gripper(value), 'valid_mask': valid},
        'action': {'value': normalize_gripper(command),
                   'valid_mask': arrived(tc, grid)},
    }


def normalize_gripper(radians) -> np.ndarray:
    """偏心轴角度 -> [0, 1]，**0 = 张开、1 = 夹紧**。

    这是对面 meta.json 声明的约定（`normalized: open 0, closed 1`）。机器人这边
    eccentric 是 0 闭合、2.76 张开，**正好相反**，所以取补。实测值会略超行程
    （量到过 2.889），所以要裁；否则会出现负的开合度。
    """
    return np.clip(1.0 - np.asarray(radians, float) / GRIPPER_TRAVEL_RAD, 0.0, 1.0)


def read_table(session: Session, key: str, ncol: int) -> tuple[np.ndarray, np.ndarray]:
    """读一张表，**按时间排好序**。没录那一路就给空表。

    营业中的落盘是按**到达顺序**追写的，而读的时候优先用源端 header 戳，
    两者不一定同序 —— 实测一个 10505 行的 joint_states 里有 1 行回退 990 ms。
    下面全靠 `searchsorted`，它对未排序输入不报错，只是静静给错答案。

    `fpc_commands` 在 FPC inactive 时根本没有发布者，空表是正常情况，
    下游会走 hold 的「无效填 NaN」而不是整个导出崩掉。
    """
    if key not in session.tables():
        return np.empty(0), np.empty((0, ncol))
    t, data = session.table(key)
    if t.size and not np.all(np.diff(t) >= 0):
        order = np.argsort(t, kind='stable')
        t, data = t[order], data[order]
    return t, data


def active_span(session: Session, t0: float, t1: float, keep: float) -> tuple[float, float]:
    """把 episode 收缩到「下发的目标真的在动」的那一段，前后各留 ``keep`` 秒。

    夹爪和位姿一起看，因为「手臂停着只合爪」在物理上是真动作。实测一次 13 条
    的采集里它一次也没改变过边界（夹爪动的时候手臂也在动），留着是保险。

    判据取 ``motion_control_status``：它一张表里同时有 ``limited_pose`` 和 ``grip``。
    换成 ``motion_control_command`` 的原始 VR 目标结果几乎一样（13 条 episode 上
    总裁剪量 77.6 s vs 77.2 s），不值得为此多读一张表。

    这一路缺失、或者整条 episode 里目标一动没动时，原样返回不裁。
    """
    columns = session.columns('motion_control_status')
    t, data = read_table(session, 'motion_control_status', len(columns))
    inside = (t >= t0) & (t <= t1)
    if inside.sum() < 3:
        return t0, t1
    t, data = t[inside], data[inside]

    pos = np.stack([data[:, [columns.index(f'limited.{side}.{k}') for k in 'xyz']]
                    for side in ARMS], axis=1)                      # (N, 2, 3)
    grip = data[:, [columns.index(f'grip.{side}') for side in ARMS]]

    lo = np.searchsorted(t, t - IDLE_WINDOW_S / 2, 'left')
    hi = np.clip(np.searchsorted(t, t + IDLE_WINDOW_S / 2, 'right') - 1, 0, t.size - 1)
    span = np.maximum(t[hi] - t[lo], 1e-6)
    speed = np.linalg.norm(pos[hi] - pos[lo], axis=-1).max(axis=1) / span
    grip_travel = np.abs(grip[hi] - grip[lo]).max(axis=1)

    moving = np.flatnonzero((speed > IDLE_POS_M_S) | (grip_travel > IDLE_GRIP_RAD))
    if moving.size == 0:
        return t0, t1
    return (max(t0, float(t[moving[0]]) - keep),
            min(t1, float(t[moving[-1]]) + keep))


def trim_episode(session: Session, episode: dict, keep: float) -> dict:
    """就地把 episode 的时间窗收紧，原始窗口留在 ``trim`` 里备查。"""
    t0, t1 = active_span(session, episode['t0'], episode['t1'], keep)
    if t1 - t0 < 1.0 / DEFAULT_HZ:       # 收得只剩一帧就别收了
        return episode
    episode['trim'] = {
        'raw_t0': episode['t0'], 'raw_t1': episode['t1'],
        'head_s': round(t0 - episode['t0'], 3),
        'tail_s': round(episode['t1'] - t1, 3),
        'keep_idle_s': keep,
    }
    episode['t0'], episode['t1'] = t0, t1
    episode['duration'] = t1 - t0
    return episode


def end_space(model, joints: dict, session: Session, grid, max_age,
              action_source: str, provenance: dict) -> dict:
    """末端位姿。state 是实测关节角的 FK，action 是当时下发的目标位姿。

    两者都在 `torso_link` 下（IK 的 base_frame 就是它），原样写出去。
    """
    matrices = np.stack([model.poses(ORIGIN, END_LINKS[side], joints)
                         for side in ARMS], axis=1)              # (N,2,4,4)
    pose = urdf_fk.matrix_to_pose(matrices)

    if action_source == 'target':
        t, data = read_table(session, 'motion_control_command', 20)
        blocks = [data[:, 4:11], data[:, 11:18]]             # base4 + left7 + right7
    else:
        columns = session.columns('motion_control_status')
        t, data = read_table(session, 'motion_control_status', len(columns))
        blocks = [data[:, [columns.index(f'limited.{side}.{k}')
                           for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')]]
                  for side in ARMS]
    held = [hold(t, block, grid, max_age)[0] for block in blocks]
    action = np.stack(held, axis=1)
    return {
        'names': [f'{side}_arm_end' for side in ARMS],
        'links': [END_LINKS[side] for side in ARMS],
        'state': {'pose': pose, 'pose_unified': unify_pose(pose),
                  'valid_mask': np.isfinite(pose).all(axis=(1, 2))},
        'action': {'pose': action, 'pose_unified': unify_pose(action),
                   'valid_mask': arrived(t, grid)},
        'fk_provenance': provenance,
    }


def unify_pose(pose) -> np.ndarray:
    """`gripper_base` 原生位姿 -> 统一约定。平移不变，只右乘一个定向 R_fix。"""
    pose = np.asarray(pose, float)
    quat = urdf_fk.quat_multiply(pose[..., 3:], np.array(EE_FIX_QUAT_XYZW))
    return np.concatenate([pose[..., :3], quat], axis=-1)


def camera_space(model, joints: dict, session: Session, grid, max_age,
                 intrinsics: dict, direction: str) -> dict:
    """内参逐档取标定值，外参靠 FK。图像另存 mp4，h5 里只留帧号。"""
    matrices, extrinsic, frames, warnings = [], [], [], []
    sizes = []
    for camera in CAMERAS:
        entry = intrinsics.get(camera.name)
        if entry is None:
            warnings.append(f'{camera.name}：没内参，该路写 NaN')
        matrices.append(np.full((3, 3), np.nan) if entry is None
                        else np.asarray(entry['k'], float).reshape(3, 3))
        # K 是那个分辨率下的值，而导出的 mp4 已经降过（实测 1920x1080 的 K
        # 配 640x360 的画面）。不把尺寸一起写下去，拿到数据的人无法自行换算。
        sizes.append([int(entry['width'] or 0), int(entry['height'] or 0)]
                     if entry else [0, 0])
        pose = model.poses(ORIGIN, camera.frame, {} if camera.static else joints)
        pose = np.broadcast_to(pose, (grid.size, 4, 4))
        if direction == 'cam_T_base':
            pose = urdf_fk.invert(pose)
        extrinsic.append(pose[:, :3, :])
        pts = (session.video_pts(camera.name)
               if session.video_path(camera.name).is_file() else np.empty(0))
        back = int((np.diff(pts) < 0).sum()) if pts.size else 0
        if back:
            warnings.append(f'{camera.name}：pts 有 {back} 处回退，查表时已夹平')
        frames.append(frame_index(pts, grid, max_age))
    return {
        'names': [c.name for c in CAMERAS],
        'warnings': warnings,
        'extrinsic_direction': direction,
        'intrinsic_size': sizes,
        'static_intrinsic': [1] * len(CAMERAS),
        'static_extrinsic': [int(c.static) for c in CAMERAS],
        'state': {
            'intrinsic': np.broadcast_to(np.stack(matrices),
                                         (grid.size, len(CAMERAS), 3, 3)),
            'extrinsic': np.stack(extrinsic, axis=1),
            'frame_index': np.stack(frames, axis=1),
        },
    }


def _roles(names) -> dict:
    roles = {}
    for role, prefixes in _ROLE_RULES:
        picked = [i for i, n in enumerate(names) if n.startswith(prefixes)]
        if picked:
            roles[role] = picked
    return roles


# --------------------------------------------------------------------- 内参装配


def collect_intrinsics(session: Session, calibration: dict) -> dict:
    """每路相机在**它当时的分辨率下**的内参。取不到就留空，绝不缩放凑数。

    腕相机的两路 RTSP 流是独立的流不是同一路的缩放，头部的档位里既有缩放也有裁剪
    （848x480 -> 640x480 是裁剪，fx 根本不变）。按比例换算出来的 K 看着正常但是错的，
    所以这里只做精确匹配。
    """
    out = {}
    head = session.meta.get('head_camera_info') or {}
    if head.get('k'):
        out['head'] = {'k': head['k'], 'd': head.get('d', [0.0] * 5),
                       'width': head.get('width'), 'height': head.get('height'),
                       'source': 'meta.json head_camera_info（D435i 出厂值）'}
    table = (calibration.get('intrinsics') or {})
    for camera in CAMERAS:
        size = _video_size(session, camera.name)
        for entry in table.get(camera.calib) or []:
            if size and (entry['width'], entry['height']) != size:
                continue
            note = '' if size else '（session 没记这路的分辨率，取标定表第一档）'
            out[camera.name] = {
                'k': list(entry['camera_matrix']), 'd': list(entry['distortion_coefficients']),
                'width': entry['width'], 'height': entry['height'],
                'source': (f'calibration.yaml {camera.calib} '
                           f'{entry["width"]}x{entry["height"]}{note}'),
            }
            break
    return out


def _video_size(session: Session, name: str) -> tuple | None:
    """录制时的分辨率。腕部现在没记，返回 None 表示「按标定表里唯一/第一档算」。"""
    stream = session.meta.get(f'{name}_stream') or {}
    if stream.get('width') and stream.get('height'):
        return int(stream['width']), int(stream['height'])
    return None


# ------------------------------------------------------------------------- 写盘


def export_episode(handle, episode: dict, spaces: dict,
                   grid: np.ndarray, notes: dict) -> None:
    import h5py
    text = h5py.string_dtype('utf-8')

    def strings(group, name, values):
        group.create_dataset(name, data=np.array(list(values), dtype=text))

    def scalar(group, name, value):
        """标量字符串写成 shape=() 的 dataset —— 示例里这些字段都是这么存的，
        写成 attrs 的话对面的读法取不到。"""
        group.create_dataset(name, data=np.array(value, dtype=text))

    meta = handle.create_group('meta')
    meta.attrs['version'] = VERSION
    for key, value in notes.items():
        meta.attrs[key] = value
    handle.create_dataset('timestamp', data=grid)

    camera = spaces['camera']
    group = meta.create_group('camera_space')
    strings(group, 'names', camera['names'])
    group.create_dataset('static_intrinsic', data=np.array(camera['static_intrinsic'], np.int64))
    group.create_dataset('static_extrinsic', data=np.array(camera['static_extrinsic'], np.int64))
    strings(group, 'state_fields', ['intrinsic', 'extrinsic'])
    group.create_dataset('intrinsic_size', data=np.array(camera['intrinsic_size'], np.int64))
    scalar(group, 'intrinsic_size_note',
           'K 对应的是这个分辨率，不是 video_*/ 里 mp4 的分辨率；'
           '要用在 mp4 像素上先按 mp4_w/intrinsic_w 缩放 fx,cx（fy,cy 同理）')
    scalar(group, 'extrinsic_direction', camera['extrinsic_direction'])
    scalar(group, 'extrinsic_formula', EXTRINSIC_FORMULA[camera['extrinsic_direction']])
    scalar(group, 'extrinsic_reference', 'robot')
    scalar(group, 'optical_axes', CAMERA_AXES)

    joint = spaces['joint']
    group = meta.create_group('joint_space')
    group.create_dataset('dof', data=np.int64(len(joint['names'])))
    strings(group, 'names', joint['names'])
    roles = group.create_group('roles')
    for role, index in joint['roles'].items():
        roles.create_dataset(role, data=np.array(index, np.int64))
    strings(group, 'state_fields', ['position', 'velocity', 'effort'])
    strings(group, 'action_fields', ['position'])
    scalar(group, 'state_position_type', 'absolute')
    scalar(group, 'action_position_type', 'absolute')

    end = spaces['end']
    group = meta.create_group('end_space')
    strings(group, 'names', end['names'])
    strings(group, 'links', end['links'])
    group.create_group('roles').create_dataset(
        'end', data=np.arange(len(end['names']), dtype=np.int64))
    strings(group, 'state_fields', ['pose'])
    strings(group, 'action_fields', ['pose'])
    scalar(group, 'state_pose_reference', 'robot')
    scalar(group, 'action_pose_reference', 'robot')
    scalar(group, 'action_pose_type', 'absolute')
    scalar(group, 'pose_frame', ORIGIN)
    # pose 是 gripper_base 原生的摆法，pose_unified 是右乘 R_fix 之后的统一约定。
    # 两份都留，和对面示例一样 —— 换算是纯函数，反过来推不回原生 frame 的语义。
    scalar(group, 'unified_convention', EE_CONVENTION)
    group.create_dataset('unified_fix_quat_xyzw',
                         data=np.array(EE_FIX_QUAT_XYZW, float))
    scalar(group, 'fk_provenance', json.dumps(end['fk_provenance'],
                                              ensure_ascii=False, sort_keys=True))

    actuator = spaces['actuator']
    group = meta.create_group('actuator_space')
    strings(group, 'names', actuator['names'])
    strings(group, 'types', actuator['types'])
    strings(group, 'state_fields', ['value'])
    strings(group, 'action_fields', ['value'])
    scalar(group, 'value_convention', '0=open, 1=closed')

    for section in ('state', 'action'):
        root = handle.create_group(section)
        for key in ('camera', 'joint', 'end', 'actuator'):
            payload = spaces[key].get(section)
            if not payload:
                continue
            group = root.create_group(f'{key}_space')
            for field, array in payload.items():
                group.create_dataset(
                    field,
                    data=np.asarray(array).astype(np.uint8) if field == 'valid_mask'
                    else np.asarray(array),
                    compression='gzip', compression_opts=4)
    # 结构化指令字段有嵌套（obj/target），逐个转字符串会丢结构，整条存 JSON
    handle.attrs['episode_json'] = json.dumps(episode, ensure_ascii=False)
    for key in ('round', 'episode', 'outcome', 'instruction_en', 'instruction_zh'):
        if key in episode:
            handle.attrs[key] = episode[key]


def build_spaces(session: Session, model, grid, args, intrinsics,
                 provenance: dict) -> dict:
    order = list(session.meta.get('joint_order') or [])
    gripper_index = tuple(order.index(n) for n in GRIPPER_JOINTS if n in order)
    joint = joint_space(session, grid, order, args.max_age, gripper_index)
    joints = dict(zip(joint['names'], joint['state']['position'].T))
    return {
        'joint': joint,
        'actuator': actuator_space(session, grid, order, args.max_age, gripper_index),
        'end': end_space(model, joints, session, grid, args.max_age,
                         args.end_action, provenance),
        'camera': camera_space(model, joints, session, grid, args.max_age,
                               intrinsics, args.extrinsic),
    }


def episode_name(serial: int, session_id: str, episode: dict) -> str:
    """h5 / mp4 / episode json 共用这一个基名，示例里就是靠同名把三者对起来的。"""
    return (f'{serial:08d}-{session_id}__g1__'
            f'round{episode["round"]}_episode{episode["episode"]}')


def episode_id(name: str) -> str:
    """索引里的 id。**前半段就是文件名那个序号**（对面示例里两者逐字相同，
    对不上就没法从 id 找回文件）；后半段是「该文件内第几段」，
    我们一个文件只放一条 episode，恒为 0000。
    """
    return f'{name.split("-", 1)[0]}-0000'


def episode_label(episode: dict, session_id: str) -> list:
    """检索用的标签串，粒度照对面示例：任务、机器人、批次、轮次、条次、成败。"""
    task = _slug(f'{episode.get("verb") or ""} {(episode.get("obj") or {}).get("en", "")}')
    return [tag for tag in (task, 'g1', session_id,
                            f'round{episode["round"]}',
                            f'episode{episode["episode"]}',
                            episode.get('outcome') or '') if tag]


def _slug(text: str) -> str:
    return re.sub(r'[^a-z0-9]+', '_', str(text).lower()).strip('_')


def dataset_meta(session: Session, episodes: list, args, intrinsics: dict) -> dict:
    """数据集级 meta.json。"""
    return {
        'format': FORMAT,
        'format_version': VERSION,
        'format_doc': FORMAT_DOC,
        'dataset_name': f'G1 upper-body VR teleop / {session.manifest["session_id"]}',
        'year': 2026,
        'environment': {'type': 'Real'},
        'robot': {
            'eef_type': 'Gripper', 'arm_type': 'Dual',
            'view_num': len(CAMERAS),
            'embodiment_count': 1,
            'gripper': {
                'normalized': {
                    'open': 0, 'closed': 1,
                    'note': 'h5 state & action stored as continuous [0,1], 0=open 1=closed',
                },
                'stored_raw': {
                    'open': GRIPPER_TRAVEL_RAD, 'closed': 0.0, 'unit': 'rad',
                    'source': 'left/right_eccentric_joint, gripper_limits in '
                              'g1_motion_control/config/motion_control.yaml',
                },
            },
        },
        'tasks': {'task_type': 'Bimanual Manipulation'},
        'scale': {'episodes': len(episodes),
                  'hours': round(sum(e['duration'] for e in episodes) / 3600, 4)},
        # 模板里的三个发布字段。自己导的数据没有公开链接，给空表而不是不给：
        # 对面的读取代码按模板写，字段缺了会 KeyError
        'data_links': [],
        'other_links': [],
        'papers': [],
        'sampling': {'hz': args.hz, 'video_fps': args.hz,
                     'note': 'h5 行数 == mp4 帧数，第 k 帧就是第 k 行那一刻的画面'},
        'world_frame': {
            'link': ORIGIN,
            'axes': WORLD_AXES,
            'fk_root': ORIGIN,
            'definition': 'torso_link 本身，跟着机器人一起动',
            'why': ('torso_link 是两条手臂链的共同根，也是 IK 的 base_frame，'
                    '所以末端位姿是纯手臂的量，不随腰/腿姿态漂。'),
            'measured': ('左肩 +Y 100 mm、右肩 −Y 100 mm、'
                         '双夹爪 +X 303 mm、头相机 +Z 428 mm'),
        },
        'camera_space': {
            'names': [c.name for c in CAMERAS],
            'extrinsic_direction': args.extrinsic,
            'extrinsic_formula': EXTRINSIC_FORMULA[args.extrinsic],
            'extrinsic_reference': 'robot',
            'optical_axes': CAMERA_AXES,
            'intrinsic_source': {c.name: (intrinsics.get(c.name) or {}).get('source', '缺')
                                 for c in CAMERAS},
        },
        'ee_convention': {
            '_desc': 'Per-arm R_fix mapping raw EEF frame to the unified convention. '
                     'q_unified = q_raw (x) R_fix_quat_xyzw (Hamilton, xyzw); xyz unchanged. '
                     '已写进 state/action end_space 的 pose_unified。',
            'target_convention': EE_CONVENTION,
            '_target_convention_desc': '+Z=approach（夹爪尾→头）, +X=朝腕相机那侧, '
                                       '+Y=closing（右手系，Y = Z × X）',
            'quat_format': 'xyzw',
            'raw_frame': {
                'links': [END_LINKS[side] for side in ARMS],
                'measured': '两指沿 ±X 开合（张开 ±41.1 mm、夹紧 ±17.3 mm），'
                            '指尖在 +Z 61 mm，标定出的腕相机在 +Y 43 mm',
            },
            'arms': {f'{side}_arm_end': {'R_fix_quat_xyzw': list(EE_FIX_QUAT_XYZW),
                                         'source': 'urdf_measured'}
                     for side in ARMS},
        },
        'license': 'proprietary',
    }


def fk_provenance(model, urdf: Path, calibration: Path | None,
                  applied: list) -> dict:
    return {
        'method': 'forward kinematics of state joint positions',
        'engine': 'urdf_fk.py (float64 numpy), verified against pinocchio to 5.6e-16',
        'root_link': ORIGIN,
        'ee_link': [END_LINKS[side] for side in ARMS],
        'urdf': str(urdf),
        'urdf_sha256': _sha256(urdf),
        'calibration': str(calibration) if calibration else '',
        'calibration_sha256': _sha256(calibration) if calibration else '',
        'calibrated_joints': applied,
        'head_optical_joint': 'd435_link -> camera_color_optical_frame '
                              '(realsense-ros TF, 不在 URDF 里)',
    }


def _sha256(path) -> str:
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for block in iter(lambda: handle.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session')
    ap.add_argument('-o', '--out', help='输出目录，--dry-run 时可省')
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--calibration')
    ap.add_argument('--hz', type=float, default=DEFAULT_HZ,
                    help='统一时间栅格，视频也按它重采样')
    ap.add_argument('--max-age', type=float, default=DEFAULT_MAX_AGE_S,
                    help='超过这么久没新样本就判无效（秒）')
    ap.add_argument('--keep-idle', type=float, default=DEFAULT_KEEP_IDLE_S,
                    help='把 episode 收紧到目标真在动的那一段，前后各留这么多秒空隙')
    ap.add_argument('--no-trim', action='store_true',
                    help='不裁，原样用 events.jsonl 里的起止时刻')
    ap.add_argument('--end-action', choices=('limited', 'target'), default='limited',
                    help='limited=实际下发的目标（IK 限位后），target=VR 原始目标')
    ap.add_argument('--extrinsic', choices=tuple(EXTRINSIC_FORMULA),
                    default='base_T_cam',
                    help='base_T_cam（默认）: world_xyz = extrinsic @ camera_xyz；'
                         'cam_T_base: camera_xyz = extrinsic @ world_xyz')
    ap.add_argument('--video-height', type=int, default=0,
                    help='导出视频的高度，0 = 保持源分辨率')
    ap.add_argument('--no-video', action='store_true')
    ap.add_argument('--include-discarded', action='store_true')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--progress', action='store_true',
                    help='往 stdout 多吐 "@progress <0..1>" 行，给面板的进度条用')
    args = ap.parse_args()

    session = Session(args.session)
    if not session.sealed:
        print('！session 没有 DONE，未正常收尾，数据可能不完整', file=sys.stderr)
    urdf = Path(args.urdf)
    calibration_path = Path(args.calibration) if args.calibration else None
    model, calibration, applied = load_model(urdf, calibration_path)
    add_head_optical(model, session.meta.get('head_optical') or HEAD_OPTICAL)
    intrinsics = collect_intrinsics(session, calibration)
    provenance = fk_provenance(model, urdf, calibration_path, applied)

    print(f'session   {session.manifest["session_id"]}')
    print(f'标定叠加  {", ".join(applied) or "无 —— 外参会是 URDF 名义值"}')
    print(f'外参方向  {args.extrinsic}（{EXTRINSIC_FORMULA[args.extrinsic]}）')
    print(f'参考系    {ORIGIN}')
    for camera in CAMERAS:
        entry = intrinsics.get(camera.name)
        print(f'内参 {camera.name:<12} {entry["source"] if entry else "缺 —— 该路写 NaN"}')

    episodes = session.episodes(include_discarded=args.include_discarded)
    if not episodes:
        print('没有可导出的 episode', file=sys.stderr)
        return 1
    out = Path(args.out) if args.out else None
    session_id = session.manifest['session_id']
    plans, records = {c.name: [] for c in CAMERAS}, []
    for serial, episode in enumerate(episodes, 1):
        if not args.no_trim:
            trim_episode(session, episode, args.keep_idle)
        grid = build_grid(episode['t0'], episode['t1'], args.hz)
        spaces = build_spaces(session, model, grid, args, intrinsics, provenance)
        name = episode_name(serial, session_id, episode)
        cut = episode.get('trim')
        cut_note = (f'  裁掉 头{cut["head_s"]:.1f}+尾{cut["tail_s"]:.1f}s'
                    if cut and cut['head_s'] + cut['tail_s'] > 0.05 else '')
        print(f'{name}  N={grid.size:<5d} {episode["duration"]:.1f}s{cut_note}  '
              f'{episode["outcome"]:<8} {episode.get("instruction_en", "")}')
        for line in _report(spaces, verbose=args.dry_run):
            print(f'    {line}')
        records.append({'name': name, 'episode': episode, 'frames': grid.size})
        if args.dry_run or out is None:
            continue

        import h5py
        notes = {'origin': ORIGIN, 'fk_root': ORIGIN, 'session': session_id,
                 'end_action_source': args.end_action,
                 'grid_hz': args.hz, 'max_age_s': args.max_age,
                 'keep_idle_s': -1.0 if args.no_trim else args.keep_idle}
        (out / 'data').mkdir(parents=True, exist_ok=True)
        with h5py.File(out / 'data' / f'{name}.h5', 'w') as handle:
            export_episode(handle, episode, spaces, grid, notes)
        frames = spaces['camera']['state']['frame_index']
        for column, camera in enumerate(CAMERAS):
            plans[camera.name].append(resample_video.Plan(
                out / f'video_{camera.name}' / f'{name}.mp4',
                frames[:, column].tolist()))

    if args.dry_run or out is None:
        return 0
    videos = {} if args.no_video else export_videos(session, plans, args)
    write_sidecars(out, session, episodes, args, intrinsics, records, videos)
    print(f'写到 {out}')
    return 0


def export_videos(session: Session, plans: dict, args) -> dict:
    if not resample_video.available():
        print('！找不到 ffmpeg/ffprobe，跳过视频', file=sys.stderr)
        return {}
    # 实测视频占总耗时 97%（h5 0.5s / 全程 15s），进度条只看视频就够准了。
    # 分母用源帧总数而不是输出帧数：只导一小段时解码也得从头拉到那里。
    totals = {c.name: max(len(session.video_pts(c.name)), 1) for c in CAMERAS}
    done = {c.name: 0 for c in CAMERAS}
    span = len(CAMERAS)

    def report() -> None:
        if args.progress:
            frac = sum(min(done[n] / totals[n], 1.0) for n in done) / span
            print(f'@progress {frac:.4f}', flush=True)

    out = {}
    for camera in CAMERAS:
        source = session.video_path(camera.name)
        if not source.is_file() or not plans[camera.name]:
            done[camera.name] = totals[camera.name]
            report()
            continue

        def tick(index, name=camera.name):
            done[name] = index
            report()

        stats = resample_video.resample(source, plans[camera.name],
                                        height=args.video_height, fps=args.hz,
                                        on_progress=tick if args.progress else None)
        done[camera.name] = totals[camera.name]
        report()
        for item in stats:
            out.setdefault(item['file'], {})[camera.name] = item
        held = sum(s['held'] for s in stats)
        past = sum(s['past_end'] for s in stats)
        print(f'视频 {camera.name:<12} {len(stats)} 段  '
              f'{stats[0]["width"]}x{stats[0]["height"]}@{args.hz:g}  重复帧 {held}'
              + (f'  ！源提前解完 {past} 帧' if past else ''))
    return out


def write_sidecars(out: Path, session: Session, episodes: list, args,
                   intrinsics: dict, records: list, videos: dict) -> None:
    """meta.json / episode/*.json / episodes_all.json，布局照对面示例。"""
    (out / 'episode').mkdir(parents=True, exist_ok=True)
    _dump(out / 'meta.json', dataset_meta(session, episodes, args, intrinsics))
    everything = []
    for record in records:
        episode = record['episode']
        entry = {
            'episode_id': episode_id(record['name']),
            'file': f'{record["name"]}.h5',
            'start_frame': 0,
            'end_frame': record['frames'] - 1,
            # 规范要的是一串候选说法；我们只有一条，先按单元素列表给
            'instruction': [episode['instruction_en']] if episode.get('instruction_en') else [],
            'instruction_zh': [episode['instruction_zh']] if episode.get('instruction_zh') else [],
            'label': episode_label(episode, session.manifest['session_id']),
            'outcome': episode['outcome'],
            'video': {name: item['file'] for name, item in
                      (videos.get(f'{record["name"]}.mp4') or {}).items()},
            # 内参是源分辨率下的，mp4 已经降过，两边尺寸都给出来才好换算
            'video_size': {name: [item['width'], item['height']] for name, item in
                           (videos.get(f'{record["name"]}.mp4') or {}).items()},
            'source': {k: v for k, v in episode.items()
                       if k not in ('instruction_en', 'instruction_zh', 'outcome')},
        }
        _dump(out / 'episode' / f'{record["name"]}.json', {'episodes': [entry]})
        everything.append(entry)
    _dump(out / 'episodes_all.json', {'episodes': everything})


def _dump(path: Path, payload) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=1),
                    encoding='utf-8')


def _report(spaces: dict, verbose: bool) -> list[str]:
    """每条 episode 的健康度。**有效率必须印出来** —— 某路整段没数据（比如
    FPC inactive 时 `fpc_commands` 一行都没）会得到一个形状完全正常、内容全 NaN
    的数据集，不看这一行就发现不了。
    """
    lines = list(spaces['camera'].get('warnings') or [])
    coverage = []
    for key in ('joint', 'end', 'actuator'):
        for section in ('state', 'action'):
            mask = (spaces[key].get(section) or {}).get('valid_mask')
            if mask is not None:
                coverage.append(f'{section}/{key} {np.mean(mask):.0%}')
    frames = spaces['camera']['state']['frame_index']
    coverage += [f'帧 {n} {(frames[:, i] >= 0).mean():.0%}'
                 for i, n in enumerate(spaces['camera']['names'])]
    lines.append('有效率  ' + '  '.join(coverage))
    if verbose:
        for key in ('camera', 'joint', 'end', 'actuator', 'base'):
            for section in ('state', 'action'):
                payload = spaces[key].get(section) or {}
                shapes = '  '.join(f'{f}{np.shape(a)}' for f, a in payload.items())
                if shapes:
                    lines.append(f'{section}/{key}_space  {shapes}')
    return lines


def _yaml(path: Path) -> dict:
    import yaml
    return yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}


if __name__ == '__main__':
    raise SystemExit(main())
