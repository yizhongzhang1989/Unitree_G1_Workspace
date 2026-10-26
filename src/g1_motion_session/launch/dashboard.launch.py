"""Independent arm dashboard using the shared task defaults."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node(context):
    shared = Path(get_package_share_directory('g1_motion_session')) / 'config'
    hardware = Path(get_package_share_directory('unitree_g1_ros2_control')) / 'config/default_31dof_param.yaml'
    return [Node(
        package='g1_motion_session', executable='dashboard_node',
        name='motion_control_dashboard', output='screen',
        parameters=[str(shared / 'session.yaml'), str(hardware), {
            'bind_host': LaunchConfiguration('bind_host'),
            'bind_port': LaunchConfiguration('bind_port'),
            'base_frame': LaunchConfiguration('base_frame'),
            'status_topic': LaunchConfiguration('status_topic'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'robot_description_topic': LaunchConfiguration('robot_description_topic'),
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bind_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('bind_port', default_value='8181'),
        DeclareLaunchArgument('base_frame', default_value='torso_link'),
        DeclareLaunchArgument('status_topic', default_value='/motion_control/status'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('robot_description_topic', default_value='/robot_description'),
        OpaqueFunction(function=_node),
    ])
