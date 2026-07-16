import unittest
from unittest.mock import call, patch

from robot_bringup.end_effectors.nodes import end_effector_actions, camera


class EndEffectorsNodesTest(unittest.TestCase):
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

    def test_bringup_adds_left_and_right_cameras(self) -> None:
        with patch(
                "robot_bringup.end_effectors.nodes.bridge",
                return_value="bridge"), \
                patch(
                    "robot_bringup.end_effectors.nodes.camera", side_effect=[
                    "left_camera", "right_camera"]) as camera_factory:
            actions = end_effector_actions("unused.yaml", [], [], [])

        self.assertEqual(actions, ["bridge", "left_camera", "right_camera"])
        self.assertEqual(camera_factory.call_args_list, [
            call("left", "192.168.123.97", 8010),
            call("right", "192.168.123.98", 8011),
        ])


if __name__ == "__main__":
    unittest.main()