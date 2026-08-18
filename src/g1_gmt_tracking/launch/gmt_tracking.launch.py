"""启动全身动作跟踪层。

只启这一层。控制栈要先起来：

    ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual

那条 launch 会把 forward_position_controller 加载成 inactive；本层在 ``~/engage``
的时候才去激活它，急停时再反激活。

**不要和 g1_motion_control 同时启动**：两者都往
``/forward_position_controller/commands`` 写，同时跑就是两个策略抢同一组电机。
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parameters(path: Path) -> dict:
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    return next(iter(document.values()))['ros__parameters']


# 866 维观测 -> GRU(64) -> 29 维动作，50 Hz 下是微秒级负载，OpenBLAS 多线程只会引入
# 调度抖动，还要和 100 Hz 的状态回调抢 CPU。
_SINGLE_THREADED_BLAS = {'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}


def _nodes(context):
    share = Path(get_package_share_directory('g1_gmt_tracking'))
    config = share / 'config' / 'gmt_tracking.yaml'
    # 31 轴的顺序只能有一个来源，就是控制栈的公共参数文件。
    # forward_position_controller.yaml 里只有重力补偿的参数，没有 joints。
    controller = _parameters(
        Path(get_package_share_directory('unitree_g1_ros2_control')) /
        'config' / 'default_31dof_param.yaml')
    overrides = {'joints': controller['joints']}

    for name in ('policy_path', 'motion_dir', 'motion'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = value
    loop = LaunchConfiguration('loop').perform(context)
    if loop:
        overrides['loop'] = loop.lower() in ('1', 'true', 'yes')

    return [Node(
        package='g1_gmt_tracking',
        executable='tracking_node',
        name='gmt_tracking',
        output='screen',
        parameters=[str(config), overrides],
        additional_env=_SINGLE_THREADED_BLAS,
        emulate_tty=True,
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # 留空则用 config/gmt_tracking.yaml 里的值。换策略/动作时可以直接指到
        # 训练产物的绝对路径试跑，不必重新 colcon build。
        DeclareLaunchArgument('policy_path', default_value=''),
        DeclareLaunchArgument('motion_dir', default_value=''),
        DeclareLaunchArgument('motion', default_value=''),
        DeclareLaunchArgument('loop', default_value=''),
        OpaqueFunction(function=_nodes),
    ])
