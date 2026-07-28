"""Connect ikt_pose_commander to the existing G1 whole-body controller.

Start ``all_data.launch.py scope:=whole_body`` first. The controller manager
owns mutual exclusion between the forward-position and trajectory controllers.

The whole-body position stream goes straight into
``/forward_position_controller/commands``: that controller applies the
calibrated arm gravity itself, so no relay sits in between. Set its
``compensation_scale`` parameter to 0.0 to command raw positions instead.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_DEFAULTS = {
    "controlled_frame": "right_gripper_base",
    "base_frame": "torso_link",
    "controller_manager": "/controller_manager",
    "robot_description_topic": "/robot_description",
    "joint_states_topic": "/joint_states",
    "dashboard_port": "8180",

    "max_joint_speed": "2.0",
    "max_iters": "20",
    "control_rate_hz": "200.0",
    "stream_rate_hz": "100.0",
}


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in _DEFAULTS.items()
    ]
    arguments.append(
        DeclareLaunchArgument("enable_dashboard", default_value="true"))

    commander = Node(
        package="robot_bringup",
        executable="ikt_pose_commander",
        name="ikt_pose_commander",
        output="screen",
        parameters=[{
            "controlled_frame": LaunchConfiguration("controlled_frame"),
            "base_frame": LaunchConfiguration("base_frame"),
            "controller_manager": LaunchConfiguration("controller_manager"),
            "robot_description_topic": LaunchConfiguration("robot_description_topic"),
            "joint_states_topic": LaunchConfiguration("joint_states_topic"),
            "command_mode": "fpc",
            "fpc_controller": "forward_position_controller",
            "jtc_controller": "joint_trajectory_controller",
            "switch_controllers": True,
            "start_enabled": False,
            "max_joint_speed": ParameterValue(LaunchConfiguration("max_joint_speed"), value_type=float),
            "max_iters": ParameterValue(LaunchConfiguration("max_iters"), value_type=int),
            "control_rate_hz": ParameterValue(LaunchConfiguration("control_rate_hz"), value_type=float),
        }],
    )

    dashboard = Node(
        package="robot_bringup",
        executable="ikt_pose_commander_dashboard",
        name="ikt_pose_commander_dashboard",
        output="screen",
        parameters=[{
            "port": ParameterValue(LaunchConfiguration("dashboard_port"), value_type=int),
            "commander_ns": "/ikt_pose_commander",
            "base_frame": LaunchConfiguration("base_frame"),
            "stream_rate_hz": ParameterValue(LaunchConfiguration("stream_rate_hz"), value_type=float),
        }],
        condition=IfCondition(LaunchConfiguration("enable_dashboard")),
    )

    return LaunchDescription([*arguments, commander, dashboard])