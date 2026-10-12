from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    directory = Path(get_package_share_directory('upper_force_feedback')) / 'config'
    shared = Path(get_package_share_directory('g1_motion_session')) / 'config'
    hardware = Path(get_package_share_directory('unitree_g1_ros2_control')) / 'config/default_31dof_param.yaml'
    files = [str(shared / 'session.yaml'), str(directory / 'upper_force_feedback.yaml')]
    config = str(directory / LaunchConfiguration('config').perform(context))
    if config not in files:
        files.append(config)
    return [Node(package='upper_force_feedback', executable='force_feedback_node',
                 name='upper_force_feedback', output='screen',
                 parameters=[*files, str(hardware)])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='upper_force_feedback.yaml'),
        OpaqueFunction(function=_nodes),
    ])
