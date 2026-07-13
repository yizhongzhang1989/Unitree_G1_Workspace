"""Launch one routed KWR57 device with its CAN bridge and web visualizer."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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

    rx_topic = f"/{bus_name}/kwr57_web/rx"
    tx_topic = f"/{bus_name}/tx"
    wrench_topic = LaunchConfiguration("wrench_topic").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)
    period_ms = int(LaunchConfiguration("period_ms").perform(context))
    sample_rate_hz = int(
        LaunchConfiguration("sample_rate_hz").perform(context))
    publish_rate = float(LaunchConfiguration("publish_rate").perform(context))
    use_si = LaunchConfiguration("use_si").perform(context).lower() in (
        "true", "1", "yes")
    tare_on_start = (
        LaunchConfiguration("tare_on_start").perform(context).lower()
        in ("true", "1", "yes"))
    bridge_config = LaunchConfiguration("bridge_config").perform(context)
    config_path = os.path.join(
        get_package_share_directory("can_bridge_ros"),
        "config",
        bridge_config,
    )
    handler_parameters = {
        "cmd_id": cmd_id,
        "data_base_id": data_base_id,
        "topic": wrench_topic,
        "frame_id": frame_id,
        "period_ms": period_ms,
        "sample_rate_hz": sample_rate_hz,
        "publish_rate": publish_rate,
        "use_si": use_si,
        "autostart": True,
        "tare_on_start": tare_on_start,
    }
    sensor_parameters = {
        "rx_topic": rx_topic,
        "tx_topic": tx_topic,
        **handler_parameters,
    }

    routes = [
        f"{channel_id}:0x{can_id:X}:{rx_topic}"
        for can_id in range(data_base_id, data_base_id + 3)
    ]
    handler_specs = [""]
    if use_frame_handler:
        handler_specs = [build_frame_handler_spec({
            "channel_id": channel_id,
            "node_name": "kwr57_ft_sensor",
            **handler_parameters,
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

    sensor = None
    if not use_frame_handler:
        sensor = Node(
            package="kwr57_ros",
            executable="ft_sensor_node",
            name="kwr57_ft_sensor",
            output="screen",
            emulate_tty=True,
            parameters=[sensor_parameters],
        )

    web = Node(
        package="kwr57_ros",
        executable="web_wrench",
        name="kwr57_web_wrench",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "topic": wrench_topic,
            "host": LaunchConfiguration("web_host").perform(context),
            "port": int(LaunchConfiguration("web_port").perform(context)),
            "force_scale": float(
                LaunchConfiguration("force_scale").perform(context)),
            "torque_scale": float(
                LaunchConfiguration("torque_scale").perform(context)),
            "ui_rate": float(LaunchConfiguration("ui_rate").perform(context)),
        }],
    )
    return [bridge, web] if sensor is None else [bridge, sensor, web]


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument(
            "bridge_config",
            default_value="single_bus.yaml",
            description="can_bridge_ros config file for the physical CAN adapter",
        ),
        DeclareLaunchArgument(
            "use_frame_handler",
            default_value="true",
            description="Assemble KWR57 frames in-process; false restores ROS Frame routing",
        ),
        DeclareLaunchArgument("channel_id", default_value="0"),
        DeclareLaunchArgument("bus_name", default_value="can0"),
        DeclareLaunchArgument("cmd_id", default_value="16"),
        DeclareLaunchArgument("data_base_id", default_value="21"),
        DeclareLaunchArgument(
            "wrench_topic", default_value="/kwr57_ft_sensor/wrench_raw"),
        DeclareLaunchArgument("frame_id", default_value="kwr57_ft_sensor_link"),
        DeclareLaunchArgument("period_ms", default_value="1"),
        DeclareLaunchArgument("sample_rate_hz", default_value="1000"),
        DeclareLaunchArgument("publish_rate", default_value="0.0"),
        DeclareLaunchArgument("use_si", default_value="false"),
        DeclareLaunchArgument("tare_on_start", default_value="false"),
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8765"),
        DeclareLaunchArgument("force_scale", default_value="10.0"),
        DeclareLaunchArgument("torque_scale", default_value="0.25"),
        DeclareLaunchArgument("ui_rate", default_value="20.0"),
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=_launch_nodes)])