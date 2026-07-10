"""Launch ONE KWR57 device node

两种模式：
  - bridge 模式（默认）：需先跑 ``can_bridge`` 独占总线，本节点只订阅
    ``rx_topic`` / 发 ``tx_topic``。适合多设备共享总线/中低频。
  - direct 模式（``direct_bus:=true``）：本节点用 SDK 紧循环直接开总线+组包，
    只发 ~1000Hz 的 WrenchStamped（可跑满）。direct 模式独占物理 CAN 设备，
    同一设备上不能再跑 bridge 或其它节点。

Examples:
  ros2 launch kwr57_ros ft_sensor.launch.py                       # bridge 模式
  ros2 launch kwr57_ros ft_sensor.launch.py direct_bus:=true      # direct 1000Hz
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
                              description="bridge 发布的总线帧话题"),
        DeclareLaunchArgument("tx_topic", default_value="/can0/tx",
                              description="bridge 订阅的命令帧话题"),
        DeclareLaunchArgument("direct_bus", default_value="false",
                              description="true=直接开总线(1000Hz,独占设备); false=经 bridge"),
        DeclareLaunchArgument("interface", default_value="canalystii",
                              description="direct 模式 python-can 后端"),
        DeclareLaunchArgument("channel", default_value="0",
                              description="direct 模式 CAN 通道"),
        DeclareLaunchArgument("bitrate", default_value="1000000",
                              description="direct 模式 波特率"),
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
            "direct_bus": ParameterValue(LaunchConfiguration("direct_bus"), value_type=bool),
            "interface": ParameterValue(LaunchConfiguration("interface"), value_type=str),
            "channel": ParameterValue(LaunchConfiguration("channel"), value_type=str),
            "bitrate": LaunchConfiguration("bitrate"),
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
