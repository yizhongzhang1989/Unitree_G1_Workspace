"""ROS 话题 -> 定长 float64 表。

两条硬约束：

**关节顺序必须按 ``msg.name`` 重排。** ``joint_state_broadcaster`` 的 ``joints`` 参数是
空数组，上游遍历 ``unordered_map``，顺序每次启动都可能不同（实测出来是
``left_hip_roll, left_hip_pitch, ..., left_eccentric(第10位)``，完全不是 URDF 顺序）。
按位置索引存会静默错列，而且不同 session 之间还不一致，事后无从发现。重排后的顺序
写进 ``schema.json``。

**列宽必须固定。** ``/motion_control/status`` 是 JSON，``limited_pose`` 只在 engage 之后
出现；列数随内容变化会让 ``np.fromfile().reshape(-1, ncol)`` 直接错位。缺的填 NaN。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Callable

#: 31 轴的规范顺序，取自 unitree_g1_ros2_control/config/default_31dof_param.yaml，
#: README 称它是「31 轴顺序的唯一来源」。
CANONICAL_JOINTS = (
    'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
    'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
    'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
    'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
    'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_roll_joint',
    'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_roll_joint',
    'right_wrist_pitch_joint', 'right_wrist_yaw_joint',
    'left_eccentric_joint', 'right_eccentric_joint',
)

#: /motion_control/status 的定宽布局。顺序即列序，改了就是破坏兼容。
STATUS_SCALARS = ('ready_to_start', 'ik_ready', 'arms_live',
                  'ik_pos_err', 'ik_ms', 'ik_rotation_weight')
STATUS_STATES = ('idle', 'stand', 'run', 'estop', 'error')

#: `g1_localization` 借 `pose.covariance[0]` 标「世界原点还没钉」（REP-145 的惯例）。
#: 值必须与 `g1_localization/localization_node.py` 的 `ORIGIN_UNSET_COV` 一致，
#: 那是它对外契约的一部分；`test_signals` 会去那个文件里核对。
ORIGIN_UNSET_COV = -1.0

NAN = float('nan')


def header_stamp(msg) -> float:
    h = getattr(msg, 'header', None)
    if h is None:
        return NAN
    return h.stamp.sec + h.stamp.nanosec * 1e-9


# ------------------------------------------------------------------ 提取函数
# 全部是纯函数：输入一条消息、输出一行浮点，不碰 ROS API，可以拿假消息单测。


def joint_state_columns(names) -> list[str]:
    return ([f'pos.{n}' for n in names] + [f'vel.{n}' for n in names]
            + [f'eff.{n}' for n in names])


def joint_state_row(msg, order: list[str]) -> list[float]:
    """按给定顺序重排。缺的关节填 NaN，多出来的忽略。"""
    index = {n: i for i, n in enumerate(msg.name)}

    def pick(seq, name):
        i = index.get(name)
        return float(seq[i]) if i is not None and i < len(seq) else NAN

    return ([pick(msg.position, n) for n in order]
            + [pick(msg.velocity, n) for n in order]
            + [pick(msg.effort, n) for n in order])


def float_array_row(msg, width: int) -> list[float]:
    data = msg.data[:width]
    return [float(v) for v in data] + [NAN] * (width - len(data))


def wrench_row(msg) -> list[float]:
    w = msg.wrench
    return [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]


def imu_row(msg) -> list[float]:
    o, g, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
    return [o.x, o.y, o.z, o.w, g.x, g.y, g.z, a.x, a.y, a.z]


def imu_state_row(msg) -> list[float]:
    rpy = getattr(msg, 'rpy', (NAN, NAN, NAN))
    return [*msg.quaternion[:4], *msg.gyroscope[:3],
            *msg.accelerometer[:3], *rpy[:3]]


def pose_twist_row(msg) -> list[float]:
    """``Odometry`` 的位姿 + 速度共 13 列。协方差不存：两个来源实测都是全零。"""
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    v, w = msg.twist.twist.linear, msg.twist.twist.angular
    return [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
            v.x, v.y, v.z, w.x, w.y, w.z]


def odometry_row(msg) -> list[float]:
    """位姿 + 速度 + 一个有效位。

    ``origin_set`` **必须逐帧记**：调过 ``set_origin`` 之后队列里的残留帧仍然带
    ``covariance[0] = -1``，按「调过服务没有」判会把那几帧算成有效，而它们的位姿是
    单位阵，看着完全正常。
    """
    unset = float(msg.pose.covariance[0]) == ORIGIN_UNSET_COV
    return pose_twist_row(msg) + [0.0 if unset else 1.0]


def torso_pose_health(msg) -> str:
    """原点没钉时位姿恒为单位阵 —— 频率、行数、面板上的绿灯全都正常，
    录完一小时才发现这一路记的全是原点。"""
    if float(msg.pose.covariance[0]) == ORIGIN_UNSET_COV:
        return ('世界原点还没钉，这一路现在录下来是废的：'
                'ros2 service call /g1_localization/set_origin std_srvs/srv/Trigger')
    return ''


#: ``/motion_control/status`` 的完整列序。算一次就好 —— status 解不出来时要拿它的长度
#: 填 NAN，而那是逐消息走的路径。
_STATUS_COLUMNS = tuple(
    [f'state.{s}' for s in STATUS_STATES] + list(STATUS_SCALARS)
    + ['grip.left', 'grip.right']
    + [f'command.{i}' for i in range(20)]
    + [f'limited.{side}.{k}'
       for side in ('left', 'right')
       for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')])


def status_columns() -> list[str]:
    return list(_STATUS_COLUMNS)


def status_row(msg) -> list[float]:
    try:
        d = json.loads(msg.data)
    except (json.JSONDecodeError, AttributeError):
        return [NAN] * len(_STATUS_COLUMNS)
    state = str(d.get('state', '')).lower()
    row = [1.0 if state == s else 0.0 for s in STATUS_STATES]
    row += [_num(d.get(k)) for k in STATUS_SCALARS]
    grip = list(d.get('grip') or [])
    row += [_num(grip[0] if len(grip) > 0 else None),
            _num(grip[1] if len(grip) > 1 else None)]
    cmd = list(d.get('command') or [])
    row += [_num(cmd[i]) if i < len(cmd) else NAN for i in range(20)]
    limited = d.get('limited_pose') or {}
    for side in ('left', 'right'):
        pose = list(limited.get(side) or [])
        row += [_num(pose[i]) if i < len(pose) else NAN for i in range(7)]
    return row


def _num(v) -> float:
    if isinstance(v, bool):
        return 1.0 if v else 0.0
    try:
        f = float(v)
    except (TypeError, ValueError):
        return NAN
    return f if math.isfinite(f) else NAN


# ------------------------------------------------------------------- 话题清单


@dataclass(frozen=True)
class SignalSpec:
    """一路信号的完整定义。``max_hz`` > 0 时按时间抽稀。"""

    key: str
    topic: str
    type_name: str
    columns: list[str]
    row: Callable
    best_effort: bool = False
    max_hz: float = 0.0
    note: str = ''
    default_on: bool = True
    #: 订阅的 KEEP_LAST 深度。**高频话题必须调到 1**：深度就是积压上限，实测
    #: ``/dog_odom``（500 Hz）在 depth=50 下 ``t_recv`` 恒定滞后 48 ms，depth=1 只有 0.9 ms。
    depth: int = 50
    #: 逐帧体检，返回一句要摆到面板上的警告或 ''。给那些「消息在来、内容却是废的」
    #: 的信号用 —— 光看频率发现不了。
    health: Callable | None = None


def _wrench(key: str, topic: str, note: str, max_hz: float) -> SignalSpec:
    return SignalSpec(key, topic, 'geometry_msgs/msg/WrenchStamped',
                      ['fx', 'fy', 'fz', 'tx', 'ty', 'tz'], wrench_row,
                      best_effort=True, max_hz=max_hz, note=note)


def default_specs(joint_order=CANONICAL_JOINTS) -> list[SignalSpec]:
    order = list(joint_order)
    return [
        SignalSpec('joint_states', '/joint_states', 'sensor_msgs/msg/JointState',
                   joint_state_columns(order),
                   lambda m: joint_state_row(m, order),
                   note='实际末端位姿靠它离线 FK；顺序已重排为规范序'),
        # QoS 必须 BEST_EFFORT：发布方是 depth=1 的 BEST_EFFORT，RELIABLE 匹配不上。
        SignalSpec('motion_control_command', '/motion_control/command',
                   'std_msgs/msg/Float64MultiArray',
                   [f'cmd.{i}' for i in range(20)],
                   lambda m: float_array_row(m, 20), best_effort=True,
                   note='VR 目标位姿 base4+left7+right7+grip2'),
        SignalSpec('motion_control_status', '/motion_control/status',
                   'std_msgs/msg/String', status_columns(), status_row,
                   note='limited_pose 是关节指令的正解，不是实测末端'),
        # QoS 必须 BEST_EFFORT：发布方是 policy_node 的 depth=1 BEST_EFFORT。
        SignalSpec('fpc_commands', '/forward_position_controller/commands',
                   'std_msgs/msg/Float64MultiArray',
                   [f'pos.{n}' for n in order] + [f'kp.{i}' for i in range(14)]
                   + [f'kd.{i}' for i in range(14)],
                   lambda m: float_array_row(m, 31 + 14 + 14), best_effort=True,
                   note='关节级指令的唯一出口；发布者是 /motion_control，'
                        '它只在 stand/run 状态下发，idle/estop 时这一路本来就空'),
        _wrench('arm0_wrench_raw', '/arm0/wrench_raw', 'KWR57 原始读数', 200.0),
        _wrench('arm1_wrench_raw', '/arm1/wrench_raw', 'KWR57 原始读数', 200.0),
        _wrench('arm0_wrench_net', '/arm0/wrench_net', '扣掉重力的净力', 0.0),
        _wrench('arm1_wrench_net', '/arm1/wrench_net', '扣掉重力的净力', 0.0),
        # QoS 必须 BEST_EFFORT：发布方是 gripper_node 的 depth=5 BEST_EFFORT。
        # 写成 RELIABLE 时 DDS 直接不投递，文件建出来但一条都不写（实测两个 session 全 0 字节）。
        SignalSpec('grip_arm0', '/grip_arm0/joint_states',
                   'sensor_msgs/msg/JointState',
                   ['pos', 'vel', 'eff'],
                   lambda m: [_first(m.position), _first(m.velocity), _first(m.effort)],
                   best_effort=True, note='夹爪实测'),
        SignalSpec('grip_arm1', '/grip_arm1/joint_states',
                   'sensor_msgs/msg/JointState',
                   ['pos', 'vel', 'eff'],
                   lambda m: [_first(m.position), _first(m.velocity), _first(m.effort)],
                   best_effort=True),
        SignalSpec('pelvis_imu', '/pelvis_imu_broadcaster/imu', 'sensor_msgs/msg/Imu',
                   ['qx', 'qy', 'qz', 'qw', 'wx', 'wy', 'wz', 'ax', 'ay', 'az'],
                   imu_row, best_effort=True),
        SignalSpec('secondary_imu', '/secondary_imu', 'unitree_hg/msg/IMUState',
                   ['qw', 'qx', 'qy', 'qz', 'wx', 'wy', 'wz',
                    'ax', 'ay', 'az', 'roll', 'pitch', 'yaw'],
                   imu_state_row, best_effort=True, max_hz=100.0,
                   note='原始 755 Hz，抽到 100 Hz'),
        # QoS 必须 BEST_EFFORT：发布方是 depth=20 的 BEST_EFFORT。
        SignalSpec('torso_pose', '/g1_localization/torso_pose',
                   'nav_msgs/msg/Odometry',
                   ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
                    'vx', 'vy', 'vz', 'wx', 'wy', 'wz', 'origin_set'],
                   odometry_row, best_effort=True, health=torso_pose_health,
                   note='雷达世界定位，10 Hz。'
                        'origin_set=0 表示原点还没钉，该帧位姿无意义'),
        # 发布者是固件里的 bare DDS app，控制栈起不起都在发（RELIABLE depth=1）。
        SignalSpec('dog_odom', '/dog_odom', 'nav_msgs/msg/Odometry',
                   ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
                    'vx', 'vy', 'vz', 'wx', 'wy', 'wz'],
                   pose_twist_row, best_effort=True, depth=1, max_hz=100.0,
                   note='固件腿部里程计，odom -> robot_center，原始 500 Hz 抽到 100 Hz。'
                        '短时相对量比雷达准一个量级，但无绝对参考、随行走无界漂移；'
                        'vx/vy/vz 静止时精确为 0，不是位置的微分，别当速度用'),
        # 不默认录：665 列/条、96 KB/s，且能从 joint_states + URDF 离线重算
        SignalSpec('tf', '/tf', 'tf2_msgs/msg/TFMessage', [], lambda m: [],
                   default_on=False, note='能离线重算，默认不录'),
    ]


def _first(seq) -> float:
    return float(seq[0]) if len(seq) else NAN
