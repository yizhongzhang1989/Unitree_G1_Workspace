"""Ordered common and per-task ROS parameter sources."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def task_parameters(package, config):
    session_share = Path(get_package_share_directory('g1_motion_session'))
    hardware_share = Path(get_package_share_directory('unitree_g1_ros2_control'))
    document = yaml.safe_load((hardware_share / 'config/default_31dof_param.yaml').read_text())
    parameters: list[str | dict[str, list[str]]] = [str(session_share / 'config/session.yaml')]
    if package != 'g1_motion_session':
        task_share = Path(get_package_share_directory(package))
        parameters.append(str(session_share / 'config/arm_ik.yaml'))
        defaults = {'lower_velocity_tracking': 'lower_body.yaml',
                    'lower_footstep_tracking': 'lower_body.yaml',
                    'upper_vr': 'upper_vr.yaml',
                    'upper_float': 'upper_float.yaml',
                    'upper_vla': 'vla_bridge.yaml'}
        default_path = task_share / 'config' / defaults.get(package, str(config))
        if default_path.is_file():
            parameters.append(str(default_path))
        override = Path(config)
        if not override.is_absolute():
            override = task_share / 'config' / override
        if str(override) not in parameters:
            parameters.append(str(override))
    elif config:
        override = Path(config)
        if not override.is_absolute():
            override = session_share / 'config' / override
        if str(override) not in parameters:
            parameters.append(str(override))
    parameters.append({'joints': document['/**']['ros__parameters']['joints']})
    return parameters
