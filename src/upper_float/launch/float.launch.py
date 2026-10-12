from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    directory = Path(get_package_share_directory('upper_float')) / 'config'
    shared = Path(get_package_share_directory('g1_motion_session')) / 'config'
    hardware = Path(get_package_share_directory('unitree_g1_ros2_control')) / 'config/default_31dof_param.yaml'
    files = [str(shared / 'session.yaml'), str(directory / 'upper_float.yaml')]
    config = str(directory / LaunchConfiguration('config').perform(context))
    if config not in files:
        files.append(config)
    overrides = {'gripper_source': LaunchConfiguration('gripper_source').perform(context)}
    topic = LaunchConfiguration('gripper_command_topic').perform(context)
    if topic:
        overrides['gripper_command_topic'] = topic
    return [Node(package='upper_float', executable='float_node', name='upper_float',
                 output='screen', parameters=[*files, str(hardware), overrides])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='upper_float.yaml'),
        DeclareLaunchArgument('gripper_source', default_value='keyboard'),
        DeclareLaunchArgument('gripper_command_topic', default_value=''),
        OpaqueFunction(function=_nodes),
    ])
