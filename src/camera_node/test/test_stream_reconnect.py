import threading
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from camera_node.camera_node import CameraNode, RTSPStream


def _node_state():
    return SimpleNamespace(
        _stream_state_lock=threading.Lock(),
        _stream_shutdown=threading.Event(),
        _stream_wake=threading.Event(),
        _stream_expected=True,
        _stream_generation=0,
        stream=MagicMock(),
        camera_name="camera_left",
        rtsp_url="rtsp://camera/stream0",
        publish_ros_image=True,
        camera_image_publisher=MagicMock(),
        bridge=MagicMock(),
        get_logger=MagicMock(return_value=MagicMock()),
    )


class StreamReconnectTest(unittest.TestCase):
    def test_stopped_stream_ends_existing_mjpeg_generator(self) -> None:
        stream = RTSPStream.__new__(RTSPStream)
        stream.viewer_lock = threading.Lock()
        stream.viewer_count = 0
        stream.running = False

        frames = stream.generate_frames_for_streaming()

        with self.assertRaises(StopIteration):
            next(frames)
        self.assertEqual(stream.viewer_count, 0)

    def test_replacement_preserves_ros_publishing(self) -> None:
        node = _node_state()
        previous = node.stream

        with patch("camera_node.camera_node.RTSPStream") as stream_type:
            replacement = stream_type.return_value
            CameraNode._replace_stream(node)

        previous.stop.assert_called_once_with()
        stream_type.assert_called_once_with(
            "camera_left", "rtsp://camera/stream0", node)
        replacement.enable_ros_publishing.assert_called_once_with(
            node.camera_image_publisher, node.bridge)
        self.assertIs(node.stream, replacement)

    def test_manual_stop_prevents_automatic_reconnect(self) -> None:
        node = _node_state()
        previous = node.stream

        CameraNode._stop_stream(node)

        previous.stop.assert_called_once_with()
        self.assertIsNone(node.stream)
        self.assertFalse(node._stream_expected)

    def test_restart_wakes_monitor_and_reenables_stream(self) -> None:
        node = _node_state()
        node._stream_expected = False
        previous = node.stream

        CameraNode._request_stream_restart(node)

        previous.stop.assert_called_once_with()
        self.assertIsNone(node.stream)
        self.assertTrue(node._stream_expected)
        self.assertTrue(node._stream_wake.is_set())


if __name__ == "__main__":
    unittest.main()