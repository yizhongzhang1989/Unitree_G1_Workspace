"""单总线接线：所有设备都在 CANalyst-II 的 CAN1（/can0）。

一个 bridge + 2 力传感器 + 2 夹爪，全部在 /can0。
设备清单同时生成 bridge 的 CAN ID 路由和各设备节点参数。
⚠️ 同一条总线上的非共享活动 CAN ID 必须互不冲突：
  - 力传感器用 examples/set_id.py 改（ft_left 0x15、ft_right 0x18）；
  - 夹爪用其上位机/手册改（grip_left 0x01/0x101、grip_right 0x02/0x102）。
        夹爪可共享状态兼容 ID 0x000，但 command_id 低 4 位必须不同；0x7FF 为固定请求 ID。

    ros2 launch robot_bringup end_effectors_single_bus.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from robot_bringup.end_effectors.nodes import end_effector_actions
from robot_bringup.end_effectors.topology import (
    CanBus,
    GloriaDevice,
    Kwr57Device,
)


def generate_launch_description() -> LaunchDescription:
    can0 = CanBus(name="can0", channel_id=0)
    kwr57_devices = [
        Kwr57Device(
            name="ft_left", bus=can0, cmd_id=0x10, data_base_id=0x15,
            wrench_topic="/ft_left/wrench_raw", frame_id="ft_left_link"),
        Kwr57Device(
            name="ft_right", bus=can0, cmd_id=0x11, data_base_id=0x18,
            wrench_topic="/ft_right/wrench_raw", frame_id="ft_right_link"),
    ]
    gloria_devices = [
        GloriaDevice(
            name="grip_left", bus=can0, command_id=0x01,
            feedback_id=0x101, rx_topic="/can0/grip_left/rx",
            joint_name="grip_left"),
        GloriaDevice(
            name="grip_right", bus=can0, command_id=0x02,
            feedback_id=0x102, rx_topic="/can0/grip_right/rx",
            joint_name="grip_right"),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_grippers_on_start", default_value="true"),
        *end_effector_actions(
            "single_bus.yaml",
            [can0],
            kwr57_devices,
            gloria_devices,
            LaunchConfiguration("enable_grippers_on_start"),
        ),
    ])
