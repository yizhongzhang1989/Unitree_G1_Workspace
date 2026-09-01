"""只起可视化面板。**不连头显，不碰机器人。**

它是 ``/mocap/frame`` 的纯消费者，所以要**先起数据源**：

    ros2 launch g1_mocap mocap.launch.py
    ros2 launch g1_mocap dashboard.launch.py
    # 浏览器打开 http://<机器人IP>:18080

因为不抢头显的上行端口，这两个可以同时跑；头显始终只连 ``mocap_node`` 一个地址。
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    config = Path(get_package_share_directory('g1_mocap')) / 'config' / 'mocap.yaml'
    # config 里的参数挂在 /mocap 名下，本节点叫别的名字，只能读出来当 dict 传。
    document = yaml.safe_load(config.read_text(encoding='utf-8'))
    parameters = dict(next(iter(document.values()))['ros__parameters'])
    for name in ('frame_topic', 'status_topic', 'calibrate_service'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            parameters[name] = value
    port = LaunchConfiguration('dashboard_port').perform(context)
    if port:
        parameters['dashboard_port'] = int(port)

    return [Node(
        package='g1_mocap',
        executable='dashboard_node',
        name='mocap_dashboard',
        output='screen',
        parameters=[parameters],
        emulate_tty=True,
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('frame_topic', default_value='',
                              description='数据源，默认 /mocap/frame'),
        DeclareLaunchArgument('status_topic', default_value=''),
        DeclareLaunchArgument('calibrate_service', default_value=''),
        DeclareLaunchArgument('dashboard_port', default_value='',
                              description='网页端口，浏览器开这个（默认 18080）'),
        OpaqueFunction(function=_nodes),
    ])
