import unittest
from unittest.mock import call, patch

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.utilities import (
    normalize_to_list_of_substitutions,
    perform_substitutions,
)

from robot_bringup.end_effectors.nodes import (
    camera,
    end_effector_actions,
    gripper,
)
from robot_bringup.end_effectors.topology import CanBus, GloriaDevice


class EndEffectorsNodesTest(unittest.TestCase):
    def test_gripper_reuses_device_launch_with_topology_parameters(self) -> None:
        can1 = CanBus(name="can1", channel_id=1)
        device = GloriaDevice(
            name="grip_arm1",
            bus=can1,
            command_id=0x02,
            feedback_id=0x102,
            rx_topic="/can1/grip_arm1/rx",
            joint_name="arm1_gripper",
            control_mode="pos_vel",
            safe_position_min=0.1,
            safe_position_max=2.5,
        )

        action = gripper(device, "true")

        self.assertIsInstance(action, IncludeLaunchDescription)
        context = LaunchContext()
        action.launch_description_source.get_launch_description(context)
        launch_path = action.launch_description_source.location
        self.assertTrue(launch_path.endswith("/gloria_ros/launch/gripper.launch.py"))
        self.assertEqual(dict(action.launch_arguments), {
            "rx_topic": "/can1/grip_arm1/rx",
            "tx_topic": "/can1/tx",
            "command_id": "2",
            "feedback_id": "258",
            "joint_name": "arm1_gripper",
            "control_mode": "pos_vel",
            "safe_position_min": "0.1",
            "safe_position_max": "2.5",
            "enable_on_start": "true",
            "node_name": "grip_arm1",
        })

    def test_camera_builds_side_specific_node(self) -> None:
        with patch("robot_bringup.end_effectors.nodes.Node") as node_type:
            action = camera("left", "192.168.123.97", 8010)

        self.assertIs(action, node_type.return_value)
        node_type.assert_called_once_with(
            package="camera_node", executable="camera_node",
            name="camera_left", output="screen", emulate_tty=True,
            parameters=[{
                "camera_name": "camera_left",
                "rtsp_url_main": (
                    "rtsp://admin:123456@192.168.123.97/stream0"),
                "camera_ip": "192.168.123.97",
                "server_port": 8010,
                "stream_fps": 25,
                "jpeg_quality": 75,
                "max_width": 800,
                "publish_ros_image": True,
                "ros_topic_name": "/camera_left/image_raw",
                "auto_reconnect": True,
                "reconnect_interval_s": 5.0,
            }])

    def test_gripper_accepts_bringup_enable_override(self) -> None:
        device = GloriaDevice(
            name="grip_arm0",
            bus=CanBus(name="can0", channel_id=0),
            command_id=0x01,
            feedback_id=0x101,
            rx_topic="/can0/grip_arm0/rx",
            joint_name="arm0_gripper",
        )
        context = LaunchContext()
        context.launch_configurations["enable_grippers_on_start"] = "true"

        action = gripper(
            device, LaunchConfiguration("enable_grippers_on_start"))

        enable_value = dict(action.launch_arguments)["enable_on_start"]
        self.assertEqual(perform_substitutions(
            context, normalize_to_list_of_substitutions(enable_value)), "true")

    def test_bringup_adds_left_and_right_cameras(self) -> None:
        with patch(
                "robot_bringup.end_effectors.nodes.bridge",
                return_value="bridge"), \
                patch(
                    "robot_bringup.end_effectors.nodes.camera", side_effect=[
                    "left_camera", "right_camera"]) as camera_factory:
            actions = end_effector_actions(
                "unused.yaml", [], [], [], "false")

        self.assertEqual(actions, ["bridge", "left_camera", "right_camera"])
        self.assertEqual(camera_factory.call_args_list, [
            call("left", "192.168.123.97", 8010),
            call("right", "192.168.123.98", 8011),
        ])


if __name__ == "__main__":
    unittest.main()