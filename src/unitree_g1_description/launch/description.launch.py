"""Publish the assembled Unitree G1 model and its TF tree."""

from pathlib import Path
from typing import cast
from urllib.parse import urlparse
from xml.dom.minidom import Document
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


_PACKAGE_NAME = "unitree_g1_description"
_MODEL_XACRO = "G1-with-dual-Gloria-M.urdf.xacro"


def _load_robot_description(model_dir: Path) -> str:
    document = cast(
        Document, xacro.process_file(str(model_dir / _MODEL_XACRO)))
    root = ElementTree.fromstring(document.toxml())
    package_prefix = f"package://{_PACKAGE_NAME}/model/"

    for mesh in root.iter("mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue
        if not urlparse(filename).scheme and not Path(filename).is_absolute():
            mesh.set("filename", package_prefix + filename)

    return ElementTree.tostring(root, encoding="unicode")


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory(_PACKAGE_NAME))
    robot_description = _load_robot_description(package_share / "model")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    robot_description_topic = LaunchConfiguration("robot_description_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "joint_states_topic", default_value="/joint_states"),
        DeclareLaunchArgument(
            "robot_description_topic", default_value="/robot_description"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="robot_state_publisher",    # 根据 URDF 和当前关节角，计算机器人各个 Link 的坐标关系（TF），并发布出去
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(
                    robot_description, value_type=str),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
            remappings=[
                ("joint_states", joint_states_topic),
                ("robot_description", robot_description_topic),
            ],
        ),
    ])