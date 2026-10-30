"""启动 upper_insert 50Hz 控制、独立限频视觉与力觉 ONNX 插孔任务节点."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_OVERRIDABLE = {
    'active_arm': str,
    'default_mode': str,
    'target_hole': str,
    'insert_depth_mm': float,
    'align_orientation': lambda v: v.lower() in ('true', '1', 'yes'),
    'wrench_order': str,
    'left_wrench_topic': str,
    'right_wrench_topic': str,
    'open_camera': lambda v: v.lower() in ('true', '1', 'yes'),
    'vision_preprocess': str,
    'vision_rate_hz': float,
    'camera_index': int,
    'action_scale': float,
    'ik_max_iters': int,
    'ik_tol_pos': float,
    'ik_tol_ori': float,
    'max_step_m': float,
    'max_step_rad': float,
    'max_policy_rot_deg': float,
    'vision_fk_alpha_pos': float,
    'vision_fk_alpha_rot': float,
    'vision_fk_coast_s': float,
    'policy_step_scale': float,
    'max_tip_lead_m': float,
    'max_tip_lead_deg': float,
    'web_host': str,
    'web_port': int,
    'policy_node': str,
    'control_rate_hz': float,
}

_SINGLE_THREADED_BLAS = {'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}


def _nodes(context):
    directory = Path(get_package_share_directory('upper_insert')) / 'config'
    shared = Path(get_package_share_directory('g1_motion_session')) / 'config'
    hardware = (
        Path(get_package_share_directory('unitree_g1_ros2_control'))
        / 'config/default_31dof_param.yaml'
    )
    files = [
        str(shared / 'session.yaml'),
        str(shared / 'arm_ik.yaml'),
        str(directory / 'upper_insert.yaml'),
    ]
    config = str(directory / LaunchConfiguration('config').perform(context))
    if config not in files:
        files.append(config)
    overrides = {}
    for name, cast_fn in _OVERRIDABLE.items():
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = cast_fn(value)

    return [
        Node(
            package='upper_insert',
            executable='insert_node',
            name='upper_insert',
            output='screen',
            parameters=[*files, str(hardware), overrides],
            additional_env=_SINGLE_THREADED_BLAS,
            emulate_tty=True,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument('config', default_value='upper_insert.yaml'),
            *(DeclareLaunchArgument(name, default_value='') for name in _OVERRIDABLE),
            OpaqueFunction(function=_nodes),
        ]
    )
