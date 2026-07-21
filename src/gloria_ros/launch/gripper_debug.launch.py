"""Launch one standalone Gloria-M hardware path for isolated debugging.

This launch owns the CANalyst-II adapter and starts its own can_bridge_ros.
Use gripper.launch.py instead when a shared bridge is already running.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context):
    command_id = int(LaunchConfiguration("command_id").perform(context), 0)
    feedback_id = int(LaunchConfiguration("feedback_id").perform(context), 0)
    node_name = LaunchConfiguration("node_name").perform(context).strip("/")
    if not node_name:
        raise ValueError("node_name must not be empty")
    if not 0 <= command_id <= 0x7FF or not 0 <= feedback_id <= 0x7FF:
        raise ValueError("command_id and feedback_id must be standard CAN IDs")

    rx_topic = f"/can0/{node_name}/rx"
    config_path = os.path.join(
        get_package_share_directory("can_bridge_ros"),
        "config",
        "single_bus.yaml",
    )
    routes = [
        f"0:0x{can_id:X}:{rx_topic}"
        for can_id in dict.fromkeys((feedback_id, command_id, 0x00))
    ]
    bridge = Node(
        package="can_bridge_ros",
        executable="bridge_node",
        name="can_bridge_ros",
        output="screen",
        emulate_tty=True,
        parameters=[config_path, {
            "channel_ids": [0],
            "bus_names": ["can0"],
            "rx_routes": routes,
            "frame_handler_specs": [""],
        }],
    )
    gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("gloria_ros"),
            "launch",
            "gripper.launch.py",
        )),
        launch_arguments={
            "rx_topic": rx_topic,
            "tx_topic": "/can0/tx",
            "command_id": str(command_id),
            "feedback_id": str(feedback_id),
            "safe_position_min": LaunchConfiguration("safe_position_min"),
            "safe_position_max": LaunchConfiguration("safe_position_max"),
            "enable_on_start": "false",
            "diagnostic_period_s": "1.0",
            "joint_name": node_name,
            "node_name": node_name,
        }.items(),
    )
    return [bridge, gripper]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("node_name", default_value="grip_left"),
        DeclareLaunchArgument("command_id", default_value="0x01"),
        DeclareLaunchArgument("feedback_id", default_value="0x101"),
        DeclareLaunchArgument("safe_position_min", default_value="0.0"),
        DeclareLaunchArgument("safe_position_max", default_value="2.77"),
        OpaqueFunction(function=_launch_nodes),
    ])