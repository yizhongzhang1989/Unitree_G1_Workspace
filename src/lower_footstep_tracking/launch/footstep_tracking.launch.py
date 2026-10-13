from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    directory = Path(get_package_share_directory('lower_footstep_tracking')) / 'config'
    shared = Path(get_package_share_directory('g1_motion_session')) / 'config'
    hardware = Path(get_package_share_directory('unitree_g1_ros2_control')) / 'config/default_31dof_param.yaml'
    files = [str(shared / 'session.yaml'), str(directory / 'footstep.yaml')]
    config = str(directory / LaunchConfiguration('config').perform(context))
    if config not in files:
        files.append(config)
    web = LaunchConfiguration('web', default='false').perform(context).lower() == 'true'
    extra = [{'footstep_web_port': int(LaunchConfiguration('port', default='8210').perform(context)),
              'footstep_localization_mode': LaunchConfiguration('localization', default='fused').perform(context)}] if web else []
    return [Node(
        package='lower_footstep_tracking', executable='footstep_web' if web else 'footstep_task',
        name='lower_footstep_tracking', output='screen',
        parameters=[*files, str(hardware), *extra],
        additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='footstep.yaml'),
        DeclareLaunchArgument('web', default_value='false'),
        DeclareLaunchArgument('port', default_value='8210'),
        DeclareLaunchArgument('localization', default_value='fused'),
        OpaqueFunction(function=_nodes),
    ])
