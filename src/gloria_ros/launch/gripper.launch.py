"""Launch one Gloria-M gripper device node (bridge mode)

需要先启动 can_bridge_ros（独占物理总线）

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
        DeclareLaunchArgument("control_mode", default_value="mit",
                      description="mit or pos_vel"),
        DeclareLaunchArgument("pmax", default_value="3.14"),
        DeclareLaunchArgument("vmax", default_value="10.0"),
        DeclareLaunchArgument("tmax", default_value="12.0"),
        DeclareLaunchArgument("safe_position_min", default_value="0.0"),
        DeclareLaunchArgument("safe_position_max", default_value="2.77"),
        DeclareLaunchArgument("kp", default_value="10.0"),
        DeclareLaunchArgument("kd", default_value="1.0"),
        DeclareLaunchArgument("pv_velocity", default_value="1.0"),
        DeclareLaunchArgument("enable_on_start", default_value="false"),
        DeclareLaunchArgument("verify_limits_on_configure", default_value="true"),
        DeclareLaunchArgument("allow_set_zero", default_value="false"),
        DeclareLaunchArgument("feedback_timeout_s", default_value="0.5"),
        DeclareLaunchArgument("response_timeout_s", default_value="0.5"),
        DeclareLaunchArgument("state_poll_period_s", default_value="0.1"),
        DeclareLaunchArgument("disable_on_feedback_timeout", default_value="true"),
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
            "control_mode": ParameterValue(
                LaunchConfiguration("control_mode"), value_type=str),
            "pmax": ParameterValue(LaunchConfiguration("pmax"), value_type=float),
            "vmax": ParameterValue(LaunchConfiguration("vmax"), value_type=float),
            "tmax": ParameterValue(LaunchConfiguration("tmax"), value_type=float),
            "safe_position_min": ParameterValue(
                LaunchConfiguration("safe_position_min"), value_type=float),
            "safe_position_max": ParameterValue(
                LaunchConfiguration("safe_position_max"), value_type=float),
            "kp": ParameterValue(LaunchConfiguration("kp"), value_type=float),
            "kd": ParameterValue(LaunchConfiguration("kd"), value_type=float),
            "pv_velocity": ParameterValue(
                LaunchConfiguration("pv_velocity"), value_type=float),
            "enable_on_start": ParameterValue(
                LaunchConfiguration("enable_on_start"), value_type=bool),
            "verify_limits_on_configure": ParameterValue(
                LaunchConfiguration("verify_limits_on_configure"), value_type=bool),
            "allow_set_zero": ParameterValue(
                LaunchConfiguration("allow_set_zero"), value_type=bool),
            "feedback_timeout_s": ParameterValue(
                LaunchConfiguration("feedback_timeout_s"), value_type=float),
            "response_timeout_s": ParameterValue(
                LaunchConfiguration("response_timeout_s"), value_type=float),
            "state_poll_period_s": ParameterValue(
                LaunchConfiguration("state_poll_period_s"), value_type=float),
            "disable_on_feedback_timeout": ParameterValue(
                LaunchConfiguration("disable_on_feedback_timeout"), value_type=bool),
            "joint_name": ParameterValue(LaunchConfiguration("joint_name"), value_type=str),
        }],
    )
    return LaunchDescription([*args, node])
