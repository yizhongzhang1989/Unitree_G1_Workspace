"""Launch the generic CAN bridge (single or multi channel) from a YAML file.

单总线（默认）：CANalyst-II 只用 CAN1 -> /can0/rx、/can0/tx。
双总线：CANalyst-II 同时用 CAN1+CAN2 -> /can0/*、/can1/*（一个进程独占设备）。

Examples:
  ros2 launch can_bridge can_bridge.launch.py
  ros2 launch can_bridge can_bridge.launch.py config:=dual_bus.yaml
  ros2 launch can_bridge can_bridge.launch.py params_file:=/abs/path/my.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    cfg_dir = os.path.join(get_package_share_directory("can_bridge"), "config")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config", default_value="single_bus.yaml",
            description="config/ 下的文件名：single_bus.yaml / dual_bus.yaml"),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([cfg_dir, LaunchConfiguration("config")]),
            description="参数 YAML 的绝对路径（给定则忽略 config）"),
        Node(
            package="can_bridge",
            executable="bridge_node",
            name="can_bridge",
            output="screen",
            emulate_tty=True,
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
