from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from g1_motion_session.launch_support import task_parameters


def _nodes(context):
    config = LaunchConfiguration('config').perform(context)
    return [Node(
        package='lower_velocity_tracking', executable='velocity_task',
        name='lower_velocity_tracking', output='screen',
        parameters=task_parameters('lower_velocity_tracking', config),
        additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='lower_body.yaml'),
        OpaqueFunction(function=_nodes),
    ])
