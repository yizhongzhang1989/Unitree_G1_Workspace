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
        # 主循环是 `rclcpp::Rate rate(5000)` 的空转轮询：实测被调度 **9683 次/秒**
        # （全栈最高，占总调度事件的 19%），而它真正有活干的只有 10 Hz 一帧点云。
        # 降权重不减少唤醒次数，但让这些空转抢不过 50 Hz 的控制环。每帧有 100 ms
        # 预算而只用 50 ms，让一让不会掉频。上游自己的 mapping_*.launch.py 同样写法。
        prefix='nice -n 10',
        parameters=[LaunchConfiguration('config')],
        remappings=[('/tf', '/point_lio/tf_unused')],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([config_arg, log_level_arg, point_lio])
