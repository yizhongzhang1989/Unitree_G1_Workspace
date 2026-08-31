"""只起可视化面板，不碰机器人。

    ros2 launch g1_mocap dashboard.launch.py
    # 浏览器打开 http://<机器人IP>:8080

头显在 PicoBridge 面板里填 `<机器人IP>:8000`。上真机之前用它看：人摆一个姿势，
G1 会变成什么样。

**这个节点、``mocap_node``、``g1_rgmt_tracking_global`` 的跟踪层三选一**：
头显同一时刻只连一个上行地址。
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_SINGLE_THREADED_BLAS = {'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}


def _nodes(context):
    config = Path(get_package_share_directory('g1_mocap')) / 'config' / 'mocap.yaml'
    # config 里的参数挂在 /mocap 名下，本节点叫别的名字，只能读出来当 dict 传。
    document = yaml.safe_load(config.read_text(encoding='utf-8'))
    parameters = dict(next(iter(document.values()))['ros__parameters'])
    for name in ('host', 'token'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            parameters[name] = value
    for name in ('port', 'dashboard_port'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            parameters[name] = int(value)

    return [Node(
        package='g1_mocap',
        executable='dashboard_node',
        name='mocap_dashboard',
        output='screen',
        parameters=[parameters],
        additional_env=_SINGLE_THREADED_BLAS,
        emulate_tty=True,
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value=''),
        DeclareLaunchArgument('port', default_value='',
                              description='动捕上行端口，头显连这个（默认 18000）'),
        DeclareLaunchArgument('token', default_value=''),
        DeclareLaunchArgument('dashboard_port', default_value='',
                              description='网页端口，浏览器开这个（默认 18080）'),
        OpaqueFunction(function=_nodes),
    ])
