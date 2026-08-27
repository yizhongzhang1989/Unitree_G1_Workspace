"""起全套世界定位：Point-LIO + 接口层。

雷达接入（`head_lidar_node`）由 `head_sensors` 负责，这里默认不代起 —— 它同时供
相机、避障等别的用途，归属在传感器层。`with_lidar:=true` 时顺带起一个，方便单独调试。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory('g1_localization')
    default_config = os.path.join(share, 'config', 'point_lio_g1.yaml')

    args = [
        DeclareLaunchArgument('config', default_value=default_config,
                              description='Point-LIO 参数文件'),
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='是否广播 world -> pelvis'),
        DeclareLaunchArgument('with_lidar', default_value='false',
                              description='顺带起 head_lidar_node（调试用）'),
    ]

    head_lidar = Node(
        package='head_sensors', executable='head_lidar_node', name='head_lidar',
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_lidar')))

    point_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'point_lio.launch.py')),
        launch_arguments={'config': LaunchConfiguration('config')}.items())

    localization = Node(
        package='g1_localization', executable='localization_node',
        name='g1_localization', output='screen',
        parameters=[{'publish_tf': LaunchConfiguration('publish_tf')}])

    return LaunchDescription([*args, head_lidar, point_lio, localization])
