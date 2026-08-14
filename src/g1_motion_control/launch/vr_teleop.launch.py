"""VR 头显遥操作桥。

本节点**自己托管** WebXR 采集页，头显直连过来，不需要另外起桥接进程。明文口
（默认 ``0.0.0.0:8000``）与 TLS 口（默认 8443）**同时**开着，两条路服务同一份页面：
``https://<机器人IP>:8443`` 直连，或 ``adb reverse`` 之后开 ``http://localhost:8000``。
WebXR 只在安全上下文里可用，所以只有这两种进法；证书与 adb 见 ``vr/README.md``。

前置：控制栈已经起来（``all_data.launch.py`` + ``motion_control.launch.py``）。
不需要先手动 engage/start——戴上头显后用 B/Y 推就行。

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

# 可以从命令行覆盖的参数名与类型。**默认值只写在 vr_teleop.py 的 declare_parameter 里**，
# 这里一律留空、留空就不覆盖：抄第二份早晚会像以前的 vx_max / height 那样悄悄跑偏。
_OVERRIDABLE = {
    'bind_host': str, 'bind_port': int, 'tls_port': int, 'token': str,
    'tls_cert': str, 'tls_key': str, 'rate_hz': float,
    'vx_max': float, 'vy_max': float, 'wz_max': float,
    'height': float, 'height_min': float, 'height_max': float, 'height_rate': float,
    'stick_deadzone': float, 'squeeze_threshold': float,
    'arm_scale': float,
    'gripper_open': float, 'gripper_closed': float,
    'frame_timeout_s': float, 'button_cooldown_s': float,
    # 和键盘 / VLA 共用 motion_control 的标准分块命令总线。VR 发全量 20 值。
    'policy_node': str, 'command_topic': str, 'status_topic': str,
}

# 同 motion_control.launch.py：小矩阵上 OpenBLAS 多线程是纯开销，还会多出
# 一堆自旋线程和实时链路抢 CPU。
_SINGLE_THREADED_BLAS = {'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}


def _nodes(context):
    overrides = {}
    for name, cast in _OVERRIDABLE.items():
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = cast(value)

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
        [DeclareLaunchArgument(name, default_value='') for name in _OVERRIDABLE]
        + [OpaqueFunction(function=_nodes)])
