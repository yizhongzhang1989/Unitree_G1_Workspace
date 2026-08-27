"""只起 Point-LIO 本体，用 G1 的配置。定位接口层另见 localization.launch.py。

Point-LIO 无条件广播 `lio_odom -> lio_body` 的 TF 且没有开关，这里把 `/tf` 重映射到一个
没人订的话题上：机器人的 TF 树由 robot_state_publisher 管，`world` 那一段由
localization_node 挂在 `pelvis` 上，不能让里程计再往主树里塞一棵孤儿子树。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory('g1_localization'), 'config', 'point_lio_g1.yaml')

    config_arg = DeclareLaunchArgument(
        'config', default_value=default_config,
        description='Point-LIO 参数文件')
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='Point-LIO 的日志级别')

    point_lio = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='point_lio',
        output='screen',
        parameters=[LaunchConfiguration('config')],
        remappings=[('/tf', '/point_lio/tf_unused')],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([config_arg, log_level_arg, point_lio])
