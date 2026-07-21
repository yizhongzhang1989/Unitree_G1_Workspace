"""Launch ONE KWR57 device node through ``can_bridge_ros``.

Examples:
  ros2 launch kwr57_ros ft_sensor.launch.py
  ros2 launch kwr57_ros ft_sensor.launch.py rx_topic:=/can0/rx tx_topic:=/can0/tx
    ros2 launch kwr57_ros ft_sensor.launch.py data_base_id:=24 cmd_id:=17 topic:=/right/wrench
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("rx_topic", default_value="/can0/rx",
                      description="bridge RX；bringup 会改为设备专属话题"),
        DeclareLaunchArgument("tx_topic", default_value="/can0/tx",
                              description="bridge 订阅的命令帧话题"),
        DeclareLaunchArgument("cmd_id", default_value="16",
                              description="host->sensor command CAN ID (decimal; 16 = 0x10)"),
        DeclareLaunchArgument("data_base_id", default_value="21",
                              description="sensor data start CAN ID (decimal; 21 = 0x15)"),
        DeclareLaunchArgument("topic", default_value="~/wrench_raw"),
        DeclareLaunchArgument("frame_id", default_value="kwr57_ft_sensor_link"),
        DeclareLaunchArgument("period_ms", default_value="1",
                              description="upload period in ms (1 -> ~1000 Hz)"),
        DeclareLaunchArgument("sample_rate_hz", default_value="1000",
                              description="internal sample rate: 100/200/400/500/600/1000"),
        DeclareLaunchArgument("publish_rate", default_value="0.0",
                              description="0.0 = publish every sample (~1 kHz)"),
        DeclareLaunchArgument("use_si", default_value="false",
                              description="false=raw(与非ROS一致); true=换算 N/N*m"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("tare_on_start", default_value="false"),
        DeclareLaunchArgument("node_name", default_value="kwr57_ft_sensor"),
        DeclareLaunchArgument("namespace", default_value=""),
    ]

    node = Node(
        package="kwr57_ros",
        executable="ft_sensor_node",
        name=LaunchConfiguration("node_name"),
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        emulate_tty=True,
        parameters=[{
            "rx_topic": ParameterValue(LaunchConfiguration("rx_topic"), value_type=str),
            "tx_topic": ParameterValue(LaunchConfiguration("tx_topic"), value_type=str),
            "cmd_id": LaunchConfiguration("cmd_id"),
            "data_base_id": LaunchConfiguration("data_base_id"),
            "topic": ParameterValue(LaunchConfiguration("topic"), value_type=str),
            "frame_id": ParameterValue(LaunchConfiguration("frame_id"), value_type=str),
            "period_ms": LaunchConfiguration("period_ms"),
            "sample_rate_hz": LaunchConfiguration("sample_rate_hz"),
            "publish_rate": LaunchConfiguration("publish_rate"),
            "use_si": LaunchConfiguration("use_si"),
            "autostart": LaunchConfiguration("autostart"),
            "tare_on_start": LaunchConfiguration("tare_on_start"),
        }],
    )

    return LaunchDescription([*args, node])
