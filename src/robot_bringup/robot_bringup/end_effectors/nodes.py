"""构造末端设备 CAN bridge、力传感器、夹爪和相机节点"""

import os
from typing import Optional, Sequence, Union

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, Shutdown
from launch.events.process import ProcessExited
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitution import Substitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from robot_bringup.end_effectors.topology import (
    CanBus,
    GloriaDevice,
    Kwr57Device,
    build_bridge_parameters,
)

_BRIDGE_RECONNECT_LIMIT = 3
_BRIDGE_RECONNECT_DELAY = 1.0


def build_bridge_node_parameters(
        buses: Sequence[CanBus],
        kwr57_devices: Sequence[Kwr57Device],
        gloria_devices: Sequence[GloriaDevice]):
    """Build all topology-derived bridge parameters"""
    parameters = build_bridge_parameters(
        buses, kwr57_devices, gloria_devices)
    parameters["kwr57_device_specs"] = [
        device.native_spec
        for device in kwr57_devices
    ] or [""]
    return parameters


def bridge(buses: Sequence[CanBus], kwr57_devices: Sequence[Kwr57Device],
              gloria_devices: Sequence[GloriaDevice]) -> Node:
    """从末端拓扑直接构造原生 bridge"""
    reconnect_attempts = 0

    def on_exit(event: ProcessExited, context: LaunchContext) -> Optional[Shutdown]:
        nonlocal reconnect_attempts
        if context.is_shutdown:
            return None
        if event.returncode != 1:
            return Shutdown(reason=(
                f"native CANalyst-II bridge exited with code {event.returncode}"))
        if reconnect_attempts >= _BRIDGE_RECONNECT_LIMIT:
            reason = (
                "native CANalyst-II bridge exhausted "
                f"{_BRIDGE_RECONNECT_LIMIT} reconnect attempts")
            get_logger("canalystii_reconnect").error(
                reason + "; shutting down launched system")
            return Shutdown(reason=reason)
        reconnect_attempts += 1
        get_logger("canalystii_reconnect").warning(
            "CANalyst-II bridge failed; reconnect attempt "
            f"{reconnect_attempts}/{_BRIDGE_RECONNECT_LIMIT} "
            f"in {_BRIDGE_RECONNECT_DELAY:.1f} s. "
            "Feedback watchdogs remain active; timed-out grippers "
            "must be explicitly re-enabled. "
            "KWR57 runtime software tare is not retained across process restarts.")
        return None

    return Node(
        package="canalystii_native_bridge", executable="native_bridge_node",
        name="can_bridge_ros", output="screen", emulate_tty=True,
        parameters=[build_bridge_node_parameters(
            buses, kwr57_devices, gloria_devices)],
        respawn=True, respawn_delay=_BRIDGE_RECONNECT_DELAY,
        on_exit=on_exit,
    )


def gripper(
    device: GloriaDevice,
    enable_on_start: Union[str, Substitution]
    ) -> IncludeLaunchDescription:
    """将部署清单参数传给 gloria_ros 的单节点 launch"""
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


def _parameter(value, value_type):
    return ParameterValue(value, value_type=value_type) \
        if isinstance(value, Substitution) else value


def camera(side: str, url, server_port: int, image_width=0,
           image_height=240, fps=15, calib_file='') -> Node:
    """由左右手部署参数构造一个 IP 相机节点"""
    camera_name = f"camera_{side}"
    return Node(
        package="camera_node", executable="camera_node",
        name=camera_name, output="screen", emulate_tty=True,
        parameters=[{
            "rtsp_url": _parameter(url, str),
            "image_topic": f"/{camera_name}/image_raw",
            "calib_file": _parameter(calib_file, str),
            "image_width": _parameter(image_width, int),
            "image_height": _parameter(image_height, int),
            "fps": _parameter(fps, int),
            "server_port": server_port,
        }])


def end_effector_actions(
        buses: Sequence[CanBus],
        kwr57_devices: Sequence[Kwr57Device],
        gloria_devices: Sequence[GloriaDevice],
        enable_grippers_on_start: Union[str, Substitution],
        wrist_left_url='rtsp://admin:123456@192.168.123.97/stream1',
        wrist_right_url='rtsp://admin:123456@192.168.123.98/stream1',
        wrist_image_width=0, wrist_image_height=240, wrist_fps=15,
        wrist_calib_file=None):
    """Build all end-effector actions with KWR57 in the bridge process"""
    if wrist_calib_file is None:
        wrist_calib_file = os.path.join(
            get_package_share_directory("camera_calibration"),
            "config", "calibration.yaml")
    return [
        bridge(buses, kwr57_devices, gloria_devices),
        *(gripper(device, enable_grippers_on_start) for device in gloria_devices),
        camera("left", wrist_left_url, 8010,
             wrist_image_width, wrist_image_height, wrist_fps,
             wrist_calib_file),
        camera("right", wrist_right_url, 8011,
             wrist_image_width, wrist_image_height, wrist_fps,
             wrist_calib_file),
    ]
