"""Launch one standalone KWR57 hardware path for isolated debugging.

This launch owns the CANalyst-II adapter and starts its own can_bridge_ros.
Use ft_sensor.launch.py instead when an external bridge already publishes ROS
Frame messages. The default uses the same in-process handler architecture as
production, but this standalone launch itself is only for isolated debugging.
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
from kwr57_ros.bridge_handler import build_frame_handler_spec


def _launch_nodes(context):
    use_frame_handler = (
        LaunchConfiguration("use_frame_handler").perform(context).lower()
        in ("true", "1", "yes"))
    cmd_id = int(LaunchConfiguration("cmd_id").perform(context), 0)
    data_base_id = int(LaunchConfiguration("data_base_id").perform(context), 0)
    if not 0 <= cmd_id <= 0x7FF:
        raise ValueError(f"cmd_id must be a standard CAN ID, got 0x{cmd_id:X}")
    if not 0 <= data_base_id <= 0x7FD:
        raise ValueError(
            f"data_base_id must be between 0 and 0x7FD, got 0x{data_base_id:X}")
    if cmd_id in range(data_base_id, data_base_id + 3):
        raise ValueError("cmd_id conflicts with the KWR57 data IDs")

    channel_id = int(LaunchConfiguration("channel_id").perform(context), 0)
    bus_name = LaunchConfiguration("bus_name").perform(context).strip("/")
    if not bus_name:
        raise ValueError("bus_name must not be empty")

    rx_topic = f"/{bus_name}/kwr57/rx"
    tx_topic = f"/{bus_name}/tx"
    sensor_parameters = {
        "cmd_id": cmd_id,
        "data_base_id": data_base_id,
        "topic": LaunchConfiguration("topic").perform(context),
        "frame_id": LaunchConfiguration("frame_id").perform(context),
        "period_ms": int(LaunchConfiguration("period_ms").perform(context)),
        "sample_rate_hz": int(
            LaunchConfiguration("sample_rate_hz").perform(context)),
        "publish_rate": float(
            LaunchConfiguration("publish_rate").perform(context)),
        "use_si": LaunchConfiguration("use_si").perform(context).lower()
        in ("true", "1", "yes"),
        "autostart": True,
        "tare_on_start": LaunchConfiguration(
            "tare_on_start").perform(context).lower() in ("true", "1", "yes"),
    }
    config_path = os.path.join(
        get_package_share_directory("can_bridge_ros"),
        "config",
        LaunchConfiguration("bridge_config").perform(context),
    )

    routes = [
        f"{channel_id}:0x{can_id:X}:{rx_topic}"
        for can_id in range(data_base_id, data_base_id + 3)
    ]
    handler_specs = [""]
    if use_frame_handler:
        handler_specs = [build_frame_handler_spec({
            "channel_id": channel_id,
            "node_name": "kwr57_ft_sensor",
            **sensor_parameters,
        })]
    bridge = Node(
        package="can_bridge_ros",
        executable="bridge_node",
        name="can_bridge_ros",
        output="screen",
        emulate_tty=True,
        parameters=[config_path, {
            "channel_ids": [channel_id],
            "bus_names": [bus_name],
            "rx_routes": routes,
            "frame_handler_specs": handler_specs,
        }],
    )

    if use_frame_handler:
        return [bridge]
    sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("kwr57_ros"),
            "launch",
            "ft_sensor.launch.py",
        )),
        launch_arguments={
            "rx_topic": rx_topic,
            "tx_topic": tx_topic,
            "cmd_id": str(cmd_id),
            "data_base_id": str(data_base_id),
            "topic": sensor_parameters["topic"],
            "frame_id": sensor_parameters["frame_id"],
            "period_ms": str(sensor_parameters["period_ms"]),
            "sample_rate_hz": str(sensor_parameters["sample_rate_hz"]),
            "publish_rate": str(sensor_parameters["publish_rate"]),
            "use_si": str(sensor_parameters["use_si"]).lower(),
            "autostart": "true",
            "tare_on_start": str(sensor_parameters["tare_on_start"]).lower(),
        }.items(),
    )
    return [bridge, sensor]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "bridge_config", default_value="single_bus.yaml"),
        DeclareLaunchArgument("use_frame_handler", default_value="true"),
        DeclareLaunchArgument("channel_id", default_value="0"),
        DeclareLaunchArgument("bus_name", default_value="can0"),
        DeclareLaunchArgument("cmd_id", default_value="16"),
        DeclareLaunchArgument("data_base_id", default_value="21"),
        DeclareLaunchArgument(
            "topic", default_value="/kwr57_ft_sensor/wrench_raw"),
        DeclareLaunchArgument(
            "frame_id", default_value="kwr57_ft_sensor_link"),
        DeclareLaunchArgument("period_ms", default_value="1"),
        DeclareLaunchArgument("sample_rate_hz", default_value="1000"),
        DeclareLaunchArgument("publish_rate", default_value="0.0"),
        DeclareLaunchArgument("use_si", default_value="false"),
        DeclareLaunchArgument("tare_on_start", default_value="false"),
        OpaqueFunction(function=_launch_nodes),
    ])