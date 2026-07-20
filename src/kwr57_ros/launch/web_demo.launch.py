"""Launch only the KWR57 web visualizer.

Start a KWR57 data source or another Wrench publisher first.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument(
            "wrench_topic", default_value="/kwr57_ft_sensor/wrench_raw"),
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8765"),
        DeclareLaunchArgument("force_scale", default_value="10.0"),
        DeclareLaunchArgument("torque_scale", default_value="0.25"),
        DeclareLaunchArgument("ui_rate", default_value="20.0"),
    ]
    web = Node(
        package="kwr57_ros",
        executable="web_wrench",
        name="kwr57_web_wrench",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "topic": LaunchConfiguration("wrench_topic"),
            "host": LaunchConfiguration("web_host"),
            "port": LaunchConfiguration("web_port"),
            "force_scale": LaunchConfiguration("force_scale"),
            "torque_scale": LaunchConfiguration("torque_scale"),
            "ui_rate": LaunchConfiguration("ui_rate"),
        }],
    )
    return LaunchDescription([*arguments, web])