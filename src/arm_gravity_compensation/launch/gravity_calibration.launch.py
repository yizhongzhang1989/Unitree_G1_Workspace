from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("port", default_value="8310"),
        DeclareLaunchArgument("lowstate_topic", default_value="/lowstate"),
        DeclareLaunchArgument("lowcmd_topic", default_value="/lowcmd"),
        DeclareLaunchArgument("allow_torque_output", default_value="false"),
        DeclareLaunchArgument(
            "urdf_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("unitree_g1_description"),
                "model", "final.urdf",
            ])),
        DeclareLaunchArgument(
            "parameter_file",
            default_value=PathJoinSubstitution([
                EnvironmentVariable("HOME"), ".ros",
                "arm_gravity_compensation", "parameters.json",
            ])),
        DeclareLaunchArgument(
            "calibrated_urdf",
            default_value=PathJoinSubstitution([
                EnvironmentVariable("HOME"), ".ros",
                "arm_gravity_compensation", "calibrated.urdf",
            ])),
    ]
    node = Node(
        package="arm_gravity_compensation",
        executable="gravity_calibration",
        name="arm_gravity_compensation",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("port"),
            "lowstate_topic": LaunchConfiguration("lowstate_topic"),
            "lowcmd_topic": LaunchConfiguration("lowcmd_topic"),
            "allow_torque_output": LaunchConfiguration("allow_torque_output"),
            "urdf_path": LaunchConfiguration("urdf_path"),
            "parameter_file": LaunchConfiguration("parameter_file"),
            "calibrated_urdf": LaunchConfiguration("calibrated_urdf"),
        }],
    )
    return LaunchDescription([*arguments, node])