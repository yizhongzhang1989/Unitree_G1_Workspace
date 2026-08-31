"""只起动捕节点，不碰机器人。

用来单独验证这条链路：头显连得上吗、``body.status`` 是不是 VALID、重定向出来的
关节角像不像话。确认没问题再去起 ``g1_rgmt_tracking_global``。

    ros2 launch g1_mocap mocap.launch.py
    ros2 topic echo /mocap/status

头显那边在配置面板里填**本机的局域网 IP** 加 ``:18000``，点连接。全程 WiFi，不用 adb。

**这个节点和跟踪层不要同时起**：头显同一时刻只连一个上行地址。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# pinocchio 的 FK 在 90 Hz 下是微秒级负载，多线程 BLAS 只会引入调度抖动。
_SINGLE_THREADED_BLAS = {'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}


def _nodes(context):
    config = Path(get_package_share_directory('g1_mocap')) / 'config' / 'mocap.yaml'
    overrides = {}
    for name in ('host', 'token'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = value
    port = LaunchConfiguration('port').perform(context)
    if port:
        overrides['port'] = int(port)

    return [Node(
        package='g1_mocap',
        executable='mocap_node',
        name='mocap',
        output='screen',
        parameters=[str(config), overrides],
        additional_env=_SINGLE_THREADED_BLAS,
        emulate_tty=True,
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # 留空则用 config/mocap.yaml 里的值。
        DeclareLaunchArgument('host', default_value=''),
        DeclareLaunchArgument('port', default_value='',
                              description='动捕上行端口，头显连这个（默认 18000）'),
        DeclareLaunchArgument('token', default_value=''),
        OpaqueFunction(function=_nodes),
    ])
