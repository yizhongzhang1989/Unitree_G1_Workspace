"""G1 头部 RealSense D435i（realsense-ros 官方驱动）。

相机型号 realsense D435i，位于头顶，经 USB 接在 G1 的 NX 计算单元上
（官方文档：https://support.unitree.com/home/zh/G1_developer/depth_camera_instruction）。
本工作区不自己写驱动，直接用官方 `realsense2_camera`，这里只负责两件事：

1. 把话题收进 `/head/camera/*`，与 `/head/lidar/*` 对齐；
2. 补一条 `d435_link -> camera_link` 静态 TF —— 驱动只发相机自己那棵子树，
   不知道相机装在机器人哪里。`d435_link` 是 URDF 里的安装点
   （相对 `torso_link`：xyz 0.0576235 0.01753 0.42987，pitch 0.8307767 rad，即低头 47.6°）。

    ros2 launch head_sensors head_camera.launch.py
    ros2 launch head_sensors head_camera.launch.py pointcloud:=true align_depth:=true
"""

import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _reject_second_instance(context):
    """D435i 只能被一个进程打开，第二个实例的报错完全看不出根因。

    实测症状是 `xioctl(UVCIOC_CTRL_QUERY) failed`、`Cannot open '/dev/video6'`，
    以及退出时 SIGSEGV —— 没一条指向「设备已被占用」。
    """
    try:
        running = subprocess.run(['pgrep', '-f', 'realsense2_camera_node'],
                                 capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return []
    pids = [p for p in running.stdout.split() if p != str(os.getpid())]
    if pids:
        raise RuntimeError(
            'realsense2_camera_node 已经在跑（pid %s），D435i 不能被两个进程同时打开。\n'
            '先停掉它：pgrep -f head_camera.launch | xargs -r kill -INT'
            % ', '.join(pids))
    return []


_ARGS = {
    'camera_namespace': 'head',
    'camera_name': 'camera',
    # 424x240 是 848x480 的恰好一半：内参整除（fx 608.451 → 304.226）、FOV 不变（仍 69.74°x43.03°）。
    # 别改成 320x240：那是从 16:9 横向裁出来的，水平 FOV 会掍到 55.48°。
    'color_profile': '424x240x30',
    # YUYV 是相机在 USB 上的原生格式，选它就少一道 YUYV->RGB8 转换、DDS 字节少 1/3，
    # 720p 录制实测 150%->114% 单核且不再丢帧；默认仍为 RGB8，不动现有下游。
    'color_format': 'RGB8',
    'enable_depth': 'true',
    'depth_profile': '424x240x30',
    'align_depth': 'false',
    'pointcloud': 'false',
    'enable_gyro': 'false',
    'enable_accel': 'false',
    # 机器人自带盆骨/躯干两个 IMU，相机 IMU 默认不开。
    'mount_frame': 'd435_link',
    'camera_base_frame': 'camera_link',
    'initial_reset': 'false',
    'wait_for_device_timeout': '-1.0',
    'reconnect_timeout': '2.0',
    'respawn': 'true',
    'respawn_delay': '2.0',
}


def _launch_realsense_node(context):
    respawn = LaunchConfiguration('respawn').perform(context).lower() in ('true', '1', 'yes')
    respawn_delay = float(LaunchConfiguration('respawn_delay').perform(context))
    return [
        Node(
            package='realsense2_camera',
            namespace=LaunchConfiguration('camera_namespace'),
            name=LaunchConfiguration('camera_name'),
            executable='realsense2_camera_node',
            output='screen',
            emulate_tty=True,
            respawn=respawn,
            respawn_delay=respawn_delay,
            parameters=[{
                'enable_color': True,
                'enable_depth': ParameterValue(LaunchConfiguration('enable_depth'), value_type=bool),
                'rgb_camera.color_profile': ParameterValue(
                    LaunchConfiguration('color_profile'), value_type=str
                ),
                'rgb_camera.color_format': ParameterValue(
                    LaunchConfiguration('color_format'), value_type=str
                ),
                'depth_module.depth_profile': ParameterValue(
                    LaunchConfiguration('depth_profile'), value_type=str
                ),
                'align_depth.enable': ParameterValue(
                    LaunchConfiguration('align_depth'), value_type=bool
                ),
                'pointcloud.enable': ParameterValue(
                    LaunchConfiguration('pointcloud'), value_type=bool
                ),
                'enable_gyro': ParameterValue(LaunchConfiguration('enable_gyro'), value_type=bool),
                'enable_accel': ParameterValue(LaunchConfiguration('enable_accel'), value_type=bool),
                'initial_reset': ParameterValue(
                    LaunchConfiguration('initial_reset'), value_type=bool
                ),
                'wait_for_device_timeout': ParameterValue(
                    LaunchConfiguration('wait_for_device_timeout'), value_type=float
                ),
                'reconnect_timeout': ParameterValue(
                    LaunchConfiguration('reconnect_timeout'), value_type=float
                ),
            }],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in _ARGS.items()],
        OpaqueFunction(function=_reject_second_instance),
        OpaqueFunction(function=_launch_realsense_node),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='d435_mount_tf',
            output='screen',
            arguments=[
                '--frame-id', LaunchConfiguration('mount_frame'),
                '--child-frame-id', LaunchConfiguration('camera_base_frame'),
            ],
        ),
    ])
