"""末端负载：净力旋量 + 负载参数。

两个节点一条单向数据流：C++ 的 ``ft_wrench_compensator`` 把 KWR57 的原始读数扣成
负载净力，Python 的 ``payload_estimator`` 再把净力压成缓变的 (质量, 质心)，喂给
手臂重力补偿。**只在整机 scope 下有意义**：两者都要关节角和躯干 IMU。

    ros2 launch robot_bringup end_effector_load.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _config(name: str) -> str:
    return os.path.join(
        get_package_share_directory("arm_gravity_compensation"), "config", name)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "gravity_table", default_value=_config("gravity_table.yaml")),
        DeclareLaunchArgument(
            "ft_calibration", default_value=_config("ft_calibration.yaml")),
        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
        DeclareLaunchArgument("imu_topic", default_value="/secondary_imu"),
        # 原始流是 1 kHz，而力控和补偿都用不到那个相位，限流在这里省下的是
        # 一整条 1 kHz 的 Python 回调。
        DeclareLaunchArgument("publish_rate", default_value="200.0"),
        Node(
            package="unitree_g1_ros2_control", executable="ft_wrench_compensator",
            name="ft_wrench_compensator", output="screen", emulate_tty=True,
            parameters=[{
                "gravity_table": LaunchConfiguration("gravity_table"),
                "ft_calibration": LaunchConfiguration("ft_calibration"),
                "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "publish_rate": ParameterValue(
                    LaunchConfiguration("publish_rate"), value_type=float),
            }],
        ),
        Node(
            package="arm_gravity_compensation", executable="payload_estimator",
            name="payload_estimator", output="screen", emulate_tty=True,
            parameters=[{
                "ft_calibration": LaunchConfiguration("ft_calibration"),
            }],
        ),
    ])
