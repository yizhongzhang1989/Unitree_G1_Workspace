"""构造 can_bridge_ros / 力传感器 / 夹爪 launch 节点的辅助函数。"""

import os
from typing import Sequence

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from robot_bringup.topology import (
    CanBus,
    GloriaDevice,
    Kwr57Device,
    build_bridge_parameters,
)


def bridge(config: str, buses: Sequence[CanBus],
           kwr57_devices: Sequence[Kwr57Device],
           gloria_devices: Sequence[GloriaDevice]) -> Node:
    """启动 bridge；物理参数来自 YAML，设备路由由本次 bringup 生成。"""
    config_path = os.path.join(
        get_package_share_directory("can_bridge_ros"),
        "config", config)
    return Node(
        package="can_bridge_ros", executable="bridge_node",
        name="can_bridge_ros", output="screen", emulate_tty=True,
        parameters=[
            config_path,
            build_bridge_parameters(buses, kwr57_devices, gloria_devices),
        ],
    )


def ft_sensor(device: Kwr57Device) -> Node:
    """由部署清单构造一个 KWR57 力传感器节点。"""
    return Node(
        package="kwr57_ros", executable="ft_sensor_node",
        name=device.name, output="screen", emulate_tty=True,
        parameters=[{
            "rx_topic": device.rx_topic,
            "tx_topic": device.bus.tx_topic,
            "cmd_id": device.cmd_id,
            "data_base_id": device.data_base_id,
            "topic": device.wrench_topic,
            "frame_id": device.frame_id,
        }])


def gripper(device: GloriaDevice) -> Node:
    """由部署清单构造一个 Gloria-M 夹爪设备节点。"""
    return Node(
        package="gloria_ros", executable="gripper_node",
        name=device.name, output="screen", emulate_tty=True,
        parameters=[{
            "rx_topic": device.rx_topic,
            "tx_topic": device.bus.tx_topic,
            "command_id": device.command_id,
            "feedback_id": device.feedback_id,
            "joint_name": device.joint_name,
            "control_mode": device.control_mode,
            "safe_position_min": device.safe_position_min,
            "safe_position_max": device.safe_position_max,
            "enable_on_start": device.enable_on_start,
        }])
