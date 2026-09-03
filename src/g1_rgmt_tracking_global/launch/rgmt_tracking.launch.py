"""启动全身动作跟踪层。

只启这一层。控制栈要先起来：

    ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual

那条 launch 会把 forward_position_controller 加载成 inactive；本层在 ``~/engage``
的时候才去激活它，急停时再反激活。

``odometry_mode`` 默认 fused，只**订阅** ``/g1_localization/torso_pose``，不会把雷达栈
拉起来。要么先把下面两层都起全（head_sensors 不在控制栈里，漏了就没有
``/head/lidar/points_full``，Point-LIO 无输入），要么换成 odom_only：

    ros2 launch head_sensors head_sensors.launch.py camera:=false
    ros2 launch g1_localization localization.launch.py

本层每次 ``~/engage`` 都会自动调用 ``/g1_localization/set_origin``；服务不可用或
定位未就绪时拒绝激活 FPC。``odom_only`` 不依赖定位栈，因此跳过这一步。

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


# 带注意力的 RGMT 在 50 Hz 下仍是微秒级负载，OpenBLAS 多线程只会引入调度抖动，
# 还要和 100 Hz 的状态回调、500 Hz 的里程计回调抢 CPU。
_SINGLE_THREADED_BLAS = {'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}


def _nodes(context):
    share = Path(get_package_share_directory('g1_rgmt_tracking_global'))
    config = share / 'config' / 'rgmt_tracking.yaml'
    # 31 轴的顺序只能有一个来源，就是控制栈的公共参数文件。
    common = _parameters(
        Path(get_package_share_directory('unitree_g1_ros2_control')) /
        'config' / 'default_31dof_param.yaml')
    overrides = {'joints': common['joints']}

    for name in ('policy_path', 'motion_dir', 'motion', 'odometry_mode',
                 'reference_source', 'mocap_frame_topic'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = value
    loop = LaunchConfiguration('loop').perform(context)
    if loop:
        overrides['loop'] = loop.lower() in ('1', 'true', 'yes')

    return [Node(
        package='g1_rgmt_tracking_global',
        executable='tracking_node',
        name='rgmt_tracking',
        output='screen',
        parameters=[str(config), overrides],
        additional_env=_SINGLE_THREADED_BLAS,
        emulate_tty=True,
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # 留空则用 config/rgmt_tracking.yaml 里的值。换策略/动作时可以直接指到
        # 训练产物的绝对路径试跑，不必重新 colcon build。
        DeclareLaunchArgument('policy_path', default_value=''),
        DeclareLaunchArgument('motion_dir', default_value=''),
        DeclareLaunchArgument('motion', default_value=''),
        DeclareLaunchArgument('loop', default_value=''),
        # fused / odom_only / lidar_only。没起雷达定位栈时用 odom_only，
        # 但只适合短动作：/dog_odom 随行走距离无界漂移。
        DeclareLaunchArgument('odometry_mode', default_value=''),
        # motion = 放录好的 NPZ；mocap = 接 g1_mocap 的 /mocap/frame 实时跟人。
        # 用 mocap 时先起 `ros2 launch g1_mocap mocap.launch.py`，
        # 并拿 `ros2 launch g1_mocap dashboard.launch.py` 看一遍重定向结果。
        DeclareLaunchArgument('reference_source', default_value=''),
        DeclareLaunchArgument('mocap_frame_topic', default_value='',
                              description='动捕数据源，默认 /mocap/frame'),
        OpaqueFunction(function=_nodes),
    ])
