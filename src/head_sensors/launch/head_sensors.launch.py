"""启动 G1 头部传感器：Livox MID-360 雷达 + RealSense D435i。

    ros2 launch head_sensors head_sensors.launch.py
    ros2 launch head_sensors head_sensors.launch.py camera:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _f(name: str) -> ParameterValue:
    """launch 参数默认是字符串，节点里声明的是 double，必须显式定型。"""
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description() -> LaunchDescription:
    camera_launch = os.path.join(
        get_package_share_directory('head_sensors'), 'launch', 'head_camera.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('min_range', default_value='0.1'),
        DeclareLaunchArgument('max_range', default_value='70.0'),
        # 默认不转发：唯一的消费者 Point-LIO 已改成直接订原始话题。
        # 代价见 head_lidar_node.py 里 `forward_imu` 参数处的注释。
        DeclareLaunchArgument('forward_imu', default_value='false'),
        DeclareLaunchArgument('align_depth', default_value='false'),
        DeclareLaunchArgument('pointcloud', default_value='false'),
        # 默认值与 head_camera.launch.py 一致，转发只是让这一条命令也能改档 ——
        # 缺了它，想要 720p YUYV 就只能绕开本文件单独起相机，雷达那半边容易漏掉。
        DeclareLaunchArgument('color_profile', default_value='424x240x30'),
        DeclareLaunchArgument('color_format', default_value='RGB8'),
        Node(
            package='head_sensors',
            executable='head_lidar_node',
            name='head_lidar',
            output='screen',
            condition=IfCondition(LaunchConfiguration('lidar')),
            parameters=[{
                'min_range': _f('min_range'),
                'max_range': _f('max_range'),
                'forward_imu': ParameterValue(
                    LaunchConfiguration('forward_imu'), value_type=bool),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            condition=IfCondition(LaunchConfiguration('camera')),
            launch_arguments={
                'align_depth': LaunchConfiguration('align_depth'),
                'pointcloud': LaunchConfiguration('pointcloud'),
                'color_profile': LaunchConfiguration('color_profile'),
                'color_format': LaunchConfiguration('color_format'),
            }.items(),
        ),
    ])
