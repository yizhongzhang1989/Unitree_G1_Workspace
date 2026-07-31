"""VR 头显遥操作桥。

前置条件（缺一不可）：

1. 控制栈已经起来（``all_data.launch.py`` + ``lower_body_policy.launch.py``）。
   不需要先手动 engage/start——戴上头显后用 B/Y 推就行。
2. VR 链路已按 ``VR/README.md`` 跑通：``adb reverse`` -> ``python3 VR/server.py``
   -> 头显里打开采集页并点 Enter VR。``curl localhost:8000/state`` 里 ``seq`` 在涨。

启动：

    ros2 launch g1_lower_body_policy vr_teleop.launch.py
    #   换地址：      vr_url:=ws://localhost:8000/ws/subscribe
    #   改速度上限：  vx_max:=0.5 vy_max:=0.4 wz_max:=1.5
    #   手部位移缩放：arm_scale:=1.0

手柄分工：**左摇杆**水平速度、**右摇杆**转向与高度（限幅同 ``teleop_keyboard.py``），
**双手同时按 B/Y** 推进状态机：站立 -> 启动策略 -> 急停，不用回终端敲
``ros2 service call``，也不需要另起 ``teleop_keyboard``。

关节名、末端帧这些和策略层必须是同一份，所以直接从 lower_body_policy.yaml 读，
不在这里抄第二份。
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 从策略层配置里继承的键：两边必须一致，否则槽位或末端帧对不上。
_SHARED = ('arm_joints', 'gripper_joints', 'base_frame',
           'left_tip_frame', 'right_tip_frame')

_ARGUMENTS = {
    'vr_url': 'ws://localhost:8000/ws/subscribe',
    'vx_max': '0.5',
    'vy_max': '0.4',
    'wz_max': '1.5',
    'height': '0.78',
    'height_min': '0.50',
    'height_max': '0.78',
    'height_rate': '0.15',
    'stick_deadzone': '0.08',
    'squeeze_threshold': '0.5',
    'arm_scale': '1.0',
    'frame_timeout_s': '0.3',
    'button_cooldown_s': '1.0',
    'policy_node': '/lower_body_policy',
    'command_topic': '/lower_body_policy/command',
    'status_topic': '/lower_body_policy/status',
    'robot_description_topic': '/robot_description',
}

_FLOATS = ('vx_max', 'vy_max', 'wz_max', 'height', 'height_min', 'height_max',
           'height_rate', 'stick_deadzone', 'squeeze_threshold', 'arm_scale',
           'frame_timeout_s', 'button_cooldown_s')

# 同 lower_body_policy.launch.py：小矩阵上 OpenBLAS 多线程是纯开销，还会多出
# 一堆自旋线程和实时链路抢 CPU。
_SINGLE_THREADED_BLAS = {'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}


def _nodes(context):
    config = Path(get_package_share_directory('g1_lower_body_policy')) \
        / 'config' / 'lower_body_policy.yaml'
    document = yaml.safe_load(config.read_text(encoding='utf-8'))
    policy = next(iter(document.values()))['ros__parameters']
    overrides = {key: policy[key] for key in _SHARED}
    for name in _ARGUMENTS:
        value = LaunchConfiguration(name).perform(context)
        overrides[name] = float(value) if name in _FLOATS else value

    return [Node(
        package='g1_lower_body_policy',
        executable='vr_teleop',
        name='vr_teleop',
        output='screen',
        parameters=[overrides],
        additional_env=_SINGLE_THREADED_BLAS,
        emulate_tty=True,
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default)
         for name, default in _ARGUMENTS.items()]
        + [OpaqueFunction(function=_nodes)])
