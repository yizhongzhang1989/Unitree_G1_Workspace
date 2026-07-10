"""Launch one Gloria-M gripper device node (bridge mode).

需要先启动 can_bridge（独占物理总线）。

Examples:
  ros2 launch gloria_ros gripper.launch.py rx_topic:=/can0/rx tx_topic:=/can0/tx
  ros2 launch gloria_ros gripper.launch.py command_id:=1 feedback_id:=257 node_name:=gripper_left
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("rx_topic", default_value="/can0/rx"),
        DeclareLaunchArgument("tx_topic", default_value="/can0/tx"),
        DeclareLaunchArgument("command_id", default_value="1",
                              description="CAN ID for commands (decimal; 1 = 0x01)"),
        DeclareLaunchArgument("feedback_id", default_value="257",
                              description="CAN ID for feedback (decimal; 257 = 0x101)"),
        DeclareLaunchArgument("joint_name", default_value="gripper"),
        DeclareLaunchArgument("node_name", default_value="gloria_gripper"),
        DeclareLaunchArgument("namespace", default_value=""),
    ]
    node = Node(
        package="gloria_ros",
        executable="gripper_node",
        name=LaunchConfiguration("node_name"),
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        emulate_tty=True,
        parameters=[{
            "rx_topic": ParameterValue(LaunchConfiguration("rx_topic"), value_type=str),
            "tx_topic": ParameterValue(LaunchConfiguration("tx_topic"), value_type=str),
            "command_id": LaunchConfiguration("command_id"),
            "feedback_id": LaunchConfiguration("feedback_id"),
            "joint_name": ParameterValue(LaunchConfiguration("joint_name"), value_type=str),
        }],
    )
    return LaunchDescription([*args, node])
