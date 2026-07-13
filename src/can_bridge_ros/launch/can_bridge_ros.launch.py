"""Launch the generic ROS CAN bridge from a YAML configuration file.

Examples:
  ros2 launch can_bridge_ros can_bridge_ros.launch.py
  ros2 launch can_bridge_ros can_bridge_ros.launch.py config:=dual_bus.yaml
  ros2 launch can_bridge_ros can_bridge_ros.launch.py params_file:=/abs/path/my.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_dir = os.path.join(
        get_package_share_directory("can_bridge_ros"), "config")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value="single_bus.yaml",
            description="config/ 下的文件名：single_bus.yaml / dual_bus.yaml",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                config_dir, LaunchConfiguration("config")]),
            description="参数 YAML 的绝对路径（给定则忽略 config）",
        ),
        Node(
            package="can_bridge_ros",
            executable="bridge_node",
            name="can_bridge_ros",
            output="screen",
            emulate_tty=True,
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
