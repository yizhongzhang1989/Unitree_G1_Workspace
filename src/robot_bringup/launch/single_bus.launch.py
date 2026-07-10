"""单总线接线：所有设备都在 CANalyst-II 的 CAN1（/can0）。

一个 bridge + 2 力传感器 + 2 夹爪，全部在 /can0。
⚠️ 同一条总线上各设备 CAN ID 必须互不相同：
  - 力传感器用 examples/set_id.py 改（ft_left 0x15、ft_right 0x18）；
  - 夹爪用其上位机/手册改（grip_left 0x01/0x101、grip_right 0x02/0x102）。

  ros2 launch robot_bringup single_bus.launch.py
"""

from launch import LaunchDescription

from robot_bringup.nodes import bridge, ft_sensor, gripper


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        bridge("single_bus.yaml"),
        ft_sensor("ft_left", "can0", cmd_id=0x10, data_base_id=0x15,
                  topic="/ft_left/wrench_raw", frame_id="ft_left_link"),
        ft_sensor("ft_right", "can0", cmd_id=0x11, data_base_id=0x18,
                  topic="/ft_right/wrench_raw", frame_id="ft_right_link"),
        gripper("grip_left", "can0", command_id=0x01, feedback_id=0x101,
                joint_name="grip_left"),
        gripper("grip_right", "can0", command_id=0x02, feedback_id=0x102,
                joint_name="grip_right"),
    ])
