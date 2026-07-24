"""构造末端设备 CAN bridge、力传感器、夹爪和相机节点。"""

import os
from typing import Sequence, Union

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitution import Substitution
from launch_ros.actions import Node

from robot_bringup.end_effectors.topology import (
    CanBus,
    GloriaDevice,
    Kwr57Device,
    build_bridge_parameters,
)


def build_bridge_node_parameters(
        buses: Sequence[CanBus],
        kwr57_devices: Sequence[Kwr57Device],
        gloria_devices: Sequence[GloriaDevice]):
    """Build all topology-derived bridge parameters."""
    parameters = build_bridge_parameters(
        buses, kwr57_devices, gloria_devices)
    parameters["kwr57_device_specs"] = [
        device.native_spec
        for device in kwr57_devices
    ] or [""]
    return parameters


def bridge(buses: Sequence[CanBus], kwr57_devices: Sequence[Kwr57Device],
              gloria_devices: Sequence[GloriaDevice]) -> Node:
    """从末端拓扑直接构造原生 bridge。"""
    return Node(
        package="canalystii_native_bridge", executable="native_bridge_node",
        name="can_bridge_ros", output="screen", emulate_tty=True,
        parameters=[build_bridge_node_parameters(
            buses, kwr57_devices, gloria_devices)],
        on_exit=Shutdown(reason="native CANalyst-II bridge exited"),
    )


def gripper(
    device: GloriaDevice,
    enable_on_start: Union[str, Substitution]
    ) -> IncludeLaunchDescription:
    """将部署清单参数传给 gloria_ros 的单节点 launch。"""
    launch_path = os.path.join(
        get_package_share_directory("gloria_ros"),
        "launch",
        "gripper.launch.py",
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={
            "rx_topic": device.rx_topic,
            "tx_topic": device.bus.tx_topic,
            "command_id": str(device.command_id),
            "feedback_id": str(device.feedback_id),
            "joint_name": device.joint_name,
            "control_mode": device.control_mode,
            "safe_position_min": str(device.safe_position_min),
            "safe_position_max": str(device.safe_position_max),
            "enable_on_start": enable_on_start,
            "node_name": device.name,
        }.items(),
    )


def camera(side: str, ip_address: str, server_port: int) -> Node:
    """由左右手部署参数构造一个 IP 相机节点。"""
    camera_name = f"camera_{side}"
    return Node(
        package="camera_node", executable="camera_node",
        name=camera_name, output="screen", emulate_tty=True,
        parameters=[{
            "camera_name": camera_name,
            "rtsp_url_main": (
                f"rtsp://admin:123456@{ip_address}/stream0"),
            "camera_ip": ip_address,
            "server_port": server_port,
            "stream_fps": 25,
            "jpeg_quality": 75,
            "max_width": 800,
            "publish_ros_image": True,
            "ros_topic_name": f"/{camera_name}/image_raw",
            "auto_reconnect": True,
            "reconnect_interval_s": 5.0,
        }])


def end_effector_actions(
        buses: Sequence[CanBus],
        kwr57_devices: Sequence[Kwr57Device],
    gloria_devices: Sequence[GloriaDevice],
    enable_grippers_on_start: Union[str, Substitution]):
    """Build all end-effector actions with KWR57 in the bridge process."""
    return [
                bridge(buses, kwr57_devices, gloria_devices),
                *(gripper(device, enable_grippers_on_start)
                    for device in gloria_devices),
                camera("left", "192.168.123.97", 8010),
                camera("right", "192.168.123.98", 8011),
    ]
