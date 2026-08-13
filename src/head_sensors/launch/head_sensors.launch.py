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
        DeclareLaunchArgument('align_depth', default_value='false'),
        DeclareLaunchArgument('pointcloud', default_value='false'),
        Node(
            package='head_sensors',
            executable='head_lidar_node',
            name='head_lidar',
            output='screen',
            condition=IfCondition(LaunchConfiguration('lidar')),
            parameters=[{
                'min_range': _f('min_range'),
                'max_range': _f('max_range'),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            condition=IfCondition(LaunchConfiguration('camera')),
            launch_arguments={
                'align_depth': LaunchConfiguration('align_depth'),
                'pointcloud': LaunchConfiguration('pointcloud'),
            }.items(),
        ),
    ])
