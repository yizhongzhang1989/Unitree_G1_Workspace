from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from g1_motion_session.launch_support import task_parameters


def _nodes(context):
    config = LaunchConfiguration('config').perform(context)
    overrides = {'gripper_source': LaunchConfiguration('gripper_source').perform(context)}
    topic = LaunchConfiguration('gripper_command_topic').perform(context)
    if topic:
        overrides['gripper_command_topic'] = topic
    return [Node(package='upper_float', executable='float_node', name='upper_float',
                 output='screen', parameters=task_parameters('upper_float', config) + [overrides])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='upper_float.yaml'),
        DeclareLaunchArgument('gripper_source', default_value='keyboard'),
        DeclareLaunchArgument('gripper_command_topic', default_value=''),
        OpaqueFunction(function=_nodes),
    ])
