"""构造 can_bridge / 力传感器 / 夹爪 launch 节点的辅助函数。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def bridge(config: str):
    """包含通用 can_bridge，用给定 config（single_bus.yaml / dual_bus.yaml）。"""
    src = os.path.join(
        get_package_share_directory("can_bridge"), "launch", "can_bridge.launch.py")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(src),
        launch_arguments={"config": config}.items())


def ft_sensor(name: str, bus: str, cmd_id: int, data_base_id: int,
              topic: str, frame_id: str) -> Node:
    """一个 KWR57 力传感器设备节点，挂在 bus（如 can0）上。"""
    return Node(
        package="kwr57_ros", executable="ft_sensor_node",
        name=name, output="screen", emulate_tty=True,
        parameters=[{
            "rx_topic": f"/{bus}/rx",
            "tx_topic": f"/{bus}/tx",
            "cmd_id": cmd_id,
            "data_base_id": data_base_id,
            "topic": topic,
            "frame_id": frame_id,
        }])


def gripper(name: str, bus: str, command_id: int, feedback_id: int,
            joint_name: str) -> Node:
    """一个 Gloria-M 夹爪设备节点，挂在 bus（如 can0）上。"""
    return Node(
        package="gloria_ros", executable="gripper_node",
        name=name, output="screen", emulate_tty=True,
        parameters=[{
            "rx_topic": f"/{bus}/rx",
            "tx_topic": f"/{bus}/tx",
            "command_id": command_id,
            "feedback_id": feedback_id,
            "joint_name": joint_name,
        }])
