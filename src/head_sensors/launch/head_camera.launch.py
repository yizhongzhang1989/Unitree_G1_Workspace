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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    ResetLaunchConfigurations,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_ARGS = {
    'camera_namespace': 'head',
    'camera_name': 'camera',
    # 848x480x30 需要 USB3 链路（实测 5000 Mbps 下双流稳定 30 fps）。相机接在 USB2 口时
    # 这一档会被驱动判为 invalid 并回落，那种接法要显式降到 640x480x15。
    'color_profile': '848x480x30',
    'depth_profile': '848x480x30',
    'align_depth': 'false',
    'pointcloud': 'false',
    'enable_gyro': 'false',
    'enable_accel': 'false',
    # 机器人自带盆骨/躯干两个 IMU，相机 IMU 默认不开。
    'mount_frame': 'd435_link',
    'camera_base_frame': 'camera_link',
    'wait_for_device_timeout': '-1.0',
    'reconnect_timeout': '6.0',
}


def generate_launch_description() -> LaunchDescription:
    rs_launch = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')

    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in _ARGS.items()],
        # rs_launch.py 会遍历整个上下文，对每个它不认识的 launch configuration 打一条
        # 警告并附上完整参数表。ResetLaunchConfigurations 先求值再清空，只把它认得的
        # 键传进去，日志才干净。
        GroupAction([
            ResetLaunchConfigurations({
                'camera_namespace': LaunchConfiguration('camera_namespace'),
                'camera_name': LaunchConfiguration('camera_name'),
                'enable_color': 'true',
                'enable_depth': 'true',
                'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
                'depth_module.depth_profile': LaunchConfiguration('depth_profile'),
                'align_depth.enable': LaunchConfiguration('align_depth'),
                'pointcloud.enable': LaunchConfiguration('pointcloud'),
                'enable_gyro': LaunchConfiguration('enable_gyro'),
                'enable_accel': LaunchConfiguration('enable_accel'),
                'wait_for_device_timeout': LaunchConfiguration('wait_for_device_timeout'),
                'reconnect_timeout': LaunchConfiguration('reconnect_timeout'),
            }),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(rs_launch)),
        ]),
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
