"""双总线接线：一个力传感器 + 一个夹爪为一组（一个手臂），分别接两条总线。

CANalyst-II 同时用 CAN1+CAN2（一个 bridge 进程独占设备、桥接两通道）：
  - 臂0 在 /can0（CAN1）：力传感器 + 夹爪
  - 臂1 在 /can1（CAN2）：力传感器 + 夹爪
不同总线相互独立，两臂设备的 CAN ID **可以相同**（无需改 ID）。
设备清单同时生成 bridge 的 CAN ID 路由和各设备节点参数。

    ros2 launch robot_bringup end_effectors_dual_bus.launch.py
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
    can1 = CanBus(name="can1", channel_id=1)
    buses = [can0, can1]
    kwr57_devices = [
        Kwr57Device(
            name="ft_arm0", bus=can0, cmd_id=0x10, data_base_id=0x15,
            wrench_topic="/arm0/wrench_raw", frame_id="arm0_ft_link"),
        Kwr57Device(
            name="ft_arm1", bus=can1, cmd_id=0x10, data_base_id=0x15,
            wrench_topic="/arm1/wrench_raw", frame_id="arm1_ft_link"),
    ]
    gloria_devices = [
        GloriaDevice(
            name="grip_arm0", bus=can0, command_id=0x01,
            feedback_id=0x101, rx_topic="/can0/grip_arm0/rx",
            joint_name="arm0_gripper"),
        GloriaDevice(
            name="grip_arm1", bus=can1, command_id=0x01,
            feedback_id=0x101, rx_topic="/can1/grip_arm1/rx",
            joint_name="arm1_gripper"),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_grippers_on_start", default_value="true"),
        *end_effector_actions(
            "dual_bus.yaml",
            buses,
            kwr57_devices,
            gloria_devices,
            LaunchConfiguration("enable_grippers_on_start"),
        ),
    ])
