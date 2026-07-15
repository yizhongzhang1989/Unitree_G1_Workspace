"""启动 CAN0 bridge、单个 Gloria-M 夹爪与浏览器控制台"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context):
    # OpaqueFunction 在运行期解析十六进制 ID 并据此生成专属接收路由
    command_id = int(LaunchConfiguration("command_id").perform(context), 0)
    feedback_id = int(LaunchConfiguration("feedback_id").perform(context), 0)
    node_name = LaunchConfiguration("node_name").perform(context).strip("/")
    if not node_name:
        raise ValueError("node_name must not be empty")
    if not 0 <= command_id <= 0x7FF or not 0 <= feedback_id <= 0x7FF:
        raise ValueError("command_id and feedback_id must be standard CAN IDs")

    rx_topic = f"/can0/{node_name}/rx"
    target_node = f"/{node_name}"
    safe_position_min = float(
        LaunchConfiguration("safe_position_min").perform(context))
    safe_position_max = float(
        LaunchConfiguration("safe_position_max").perform(context))
    config_path = os.path.join(
        get_package_share_directory("can_bridge_ros"),
        "config",
        "single_bus.yaml",
    )
    # 去重可避免命令 ID 与反馈 ID 相同时重复创建同一条路由
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
    gripper = Node(
        package="gloria_ros",
        executable="gripper_node",
        name=node_name,
        output="screen",
        emulate_tty=True,
        parameters=[{
            "rx_topic": rx_topic,
            "tx_topic": "/can0/tx",
            "command_id": command_id,
            "feedback_id": feedback_id,
            "joint_name": node_name,
            "safe_position_min": safe_position_min,
            "safe_position_max": safe_position_max,
            "enable_on_start": False,
            "diagnostic_period_s": 1.0,
        }],
    )
    web = Node(
        package="gloria_ros",
        executable="web_gripper",
        name="gloria_web_gripper",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "target_node": target_node,
            "host": LaunchConfiguration("web_host").perform(context),
            "port": int(LaunchConfiguration("web_port").perform(context)),
            "request_timeout_s": float(
                LaunchConfiguration("request_timeout_s").perform(context)),
            "state_stale_s": float(
                LaunchConfiguration("state_stale_s").perform(context)),
            "safe_position_min": safe_position_min,
            "safe_position_max": safe_position_max,
        }],
    )
    return [bridge, gripper, web]


def generate_launch_description() -> LaunchDescription:
    # Web 调试入口只暴露单设备调试所需的最小参数集合
    arguments = [
        DeclareLaunchArgument("node_name", default_value="grip_left"),
        DeclareLaunchArgument("command_id", default_value="0x01"),
        DeclareLaunchArgument("feedback_id", default_value="0x101"),
        DeclareLaunchArgument("safe_position_min", default_value="0.0"),
        DeclareLaunchArgument("safe_position_max", default_value="2.77"),
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8766"),
        DeclareLaunchArgument("request_timeout_s", default_value="3.0"),
        DeclareLaunchArgument("state_stale_s", default_value="1.0"),
    ]
    return LaunchDescription([
        *arguments,
        OpaqueFunction(function=_launch_nodes),
    ])