"""Launch only the motion session coordinator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from g1_motion_session.launch_support import task_parameters


def _nodes(context):
    config = LaunchConfiguration('config').perform(context)
    return [Node(package='g1_motion_session', executable='motion_session',
                 name='motion_control', output='screen',
                 parameters=task_parameters('g1_motion_session', config))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='session.yaml'),
        OpaqueFunction(function=_nodes),
    ])
