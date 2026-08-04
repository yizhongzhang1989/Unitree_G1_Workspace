"""VR 头显遥操作桥。

本节点**自己托管** WebXR 采集页（默认 ``0.0.0.0:8000``），头显直连过来，
不需要另外起桥接进程。

前置条件：

1. 控制栈已经起来（``all_data.launch.py`` + ``motion_control.launch.py``）。
   不需要先手动 engage/start——戴上头显后用 B/Y 推就行。
2. ``adb reverse tcp:8000 tcp:8000`` 已建（见 ``vr/README.md``），头显里打开
   ``http://localhost:8000`` 点 Enter VR。``curl localhost:8000/state`` 里 ``seq`` 在涨。

启动：

    ros2 launch g1_motion_control vr_teleop.launch.py
    #   换端口：      bind_port:=8001
    #   只收本机：    bind_host:=127.0.0.1     （不开 /monitor 时更安全）
    #   加共享密钥：  token:=xxxx             （所有接口都要带 ?token=）
    #   改速度上限：  vx_max:=0.5 vy_max:=0.4 wz_max:=1.5
    #   手部位移缩放：arm_scale:=1.0

手柄分工：**左摇杆**水平速度、**右摇杆**转向与高度（限幅同 ``teleop_keyboard.py``），
**双手同时按 B/Y** 推进状态机：站立 -> 启动策略 -> 急停，不用回终端敲
``ros2 service call``，也不需要另起 ``teleop_keyboard``。

本节点不做 IK：接管原点直接取策略层 ``~/status`` 里发布的末端位姿，所以不需要
从 ``motion_control.yaml`` 里继承关节名与末端帧。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from g1_motion_control.make_vr_cert import DEFAULT_DIR, DEFAULT_TLS_PORT

_ARGUMENTS = {
    'bind_host': '0.0.0.0',
    'bind_port': '8000',
    'tls_port': str(DEFAULT_TLS_PORT),
    'token': '',
    'tls_cert': str(DEFAULT_DIR / 'cert.pem'),
    'tls_key': str(DEFAULT_DIR / 'key.pem'),
    'vx_max': '0.5',
    'vy_max': '0.4',
    'wz_max': '1.5',
    'height': '0.78',
    'height_min': '0.50',
    'height_max': '0.78',
    'height_rate': '0.15',
    'stick_deadzone': '0.08',
    'squeeze_threshold': '0.5',
    'arm_scale': '1.0',
    'frame_timeout_s': '0.3',
    'button_cooldown_s': '1.0',
    'policy_node': '/motion_control',
    'command_topic': '/motion_control/command',
    'status_topic': '/motion_control/status',
}

_FLOATS = ('vx_max', 'vy_max', 'wz_max', 'height', 'height_min', 'height_max',
           'height_rate', 'stick_deadzone', 'squeeze_threshold', 'arm_scale',
           'frame_timeout_s', 'button_cooldown_s')

_INTS = ('bind_port', 'tls_port')

# 同 motion_control.launch.py：小矩阵上 OpenBLAS 多线程是纯开销，还会多出
# 一堆自旋线程和实时链路抢 CPU。
_SINGLE_THREADED_BLAS = {'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}


def _nodes(context):
    overrides = {}
    for name in _ARGUMENTS:
        value = LaunchConfiguration(name).perform(context)
        if name in _FLOATS:
            overrides[name] = float(value)
        elif name in _INTS:
            overrides[name] = int(value)
        else:
            overrides[name] = value

    return [Node(
        package='g1_motion_control',
        executable='vr_teleop',
        name='vr_teleop',
        output='screen',
        parameters=[overrides],
        additional_env=_SINGLE_THREADED_BLAS,
        emulate_tty=True,
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default)
         for name, default in _ARGUMENTS.items()]
        + [OpaqueFunction(function=_nodes)])
