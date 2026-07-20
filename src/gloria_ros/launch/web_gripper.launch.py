"""只启动 Gloria-M Web 控制台；请先启动夹爪数据节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("target_node", default_value="/grip_left"),
        DeclareLaunchArgument("safe_position_min", default_value="0.0"),
        DeclareLaunchArgument("safe_position_max", default_value="2.77"),
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8766"),
        DeclareLaunchArgument("request_timeout_s", default_value="3.0"),
        DeclareLaunchArgument("state_stale_s", default_value="1.0"),
    ]
    web = Node(
        package="gloria_ros",
        executable="web_gripper",
        name="gloria_web_gripper",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "target_node": LaunchConfiguration("target_node"),
            "host": LaunchConfiguration("web_host"),
            "port": LaunchConfiguration("web_port"),
            "request_timeout_s": LaunchConfiguration("request_timeout_s"),
            "state_stale_s": LaunchConfiguration("state_stale_s"),
            "safe_position_min": LaunchConfiguration("safe_position_min"),
            "safe_position_max": LaunchConfiguration("safe_position_max"),
        }],
    )
    return LaunchDescription([*arguments, web])