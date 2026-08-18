"""不依赖 ROS 运行时和相机的部分：ffmpeg 命令拼装、按需拉流判据、读帧线程。"""

import array
import socket
import subprocess
import threading
import time
import unittest
from types import MethodType, SimpleNamespace
from unittest.mock import MagicMock, patch

from builtin_interfaces.msg import Time
from rclpy.parameter import Parameter

from camera_node.camera_node import (CameraNode, RtspReader, ffmpeg_command,
                                     fit_size, probe_size)
from camera_node.preview import connected

_BOUND = ('_step', '_open_reader', '_close_reader', '_fail', '_on_frame',
          '_setting', '_stream_settings', '_native_size', '_check_parameters')
_DEFAULTS = {'rtsp_url': 'rtsp://camera/stream1', 'fps': 15,
             'image_width': 0, 'image_height': 0, 'jpeg_quality': 60}


def _node(subscribers=0, viewers=0, reader=None, **overrides):
    """够 _step / _on_frame 跑起来的最小 CameraNode 替身"""
    params = dict(_DEFAULTS, **overrides)
    clock = MagicMock()
    clock.return_value.now.return_value.to_msg.return_value = Time()
    node = SimpleNamespace(
        params=params,
        get_parameter=lambda name: SimpleNamespace(value=params[name]),
        _publisher=SimpleNamespace(
            get_subscription_count=MagicMock(return_value=subscribers),
            publish=MagicMock()),
        _frames=threading.Condition(),
        _viewers=viewers,
        _publish_frames=False,
        _preview=None,
        _reader=reader,
        _applied=None,
        _probed={},
        _error=None,
        _seq=0,
        _frame=None,
        _stale_timeout=5.0,
        _frame_id='camera_left',
        get_clock=clock,
        get_logger=MagicMock(return_value=MagicMock()),
    )
    for name in _BOUND:
        setattr(node, name, MethodType(getattr(CameraNode, name), node))
    return node


def _healthy(reader):
    """让替身 reader 看起来活着且在出帧"""
    reader.alive.return_value = True
    reader.frame_age.return_value = 0.0
    return reader


class FitSizeTest(unittest.TestCase):
    def test_height_only_keeps_aspect_ratio(self) -> None:
        self.assertEqual(fit_size((640, 360), 0, 240), (426, 240))
        self.assertEqual(fit_size((1920, 1080), 0, 240), (426, 240))

    def test_width_only_keeps_aspect_ratio(self) -> None:
        self.assertEqual(fit_size((640, 360), 320, 0), (320, 180))

    def test_both_sides_win_over_aspect_ratio(self) -> None:
        self.assertEqual(fit_size((640, 360), 320, 240), (320, 240))

    def test_neither_side_falls_back_to_native(self) -> None:
        self.assertEqual(fit_size((640, 360), 0, 0), (640, 360))


class FfmpegCommandTest(unittest.TestCase):
    def test_native_stream_has_no_filter(self) -> None:
        command = ffmpeg_command('rtsp://camera/stream1')

        self.assertNotIn('-vf', command)
        self.assertEqual(command[-7:], [
            '-i', 'rtsp://camera/stream1',
            '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-'])

    def test_keeps_low_delay_but_never_nobuffer(self) -> None:
        """nobuffer 会丢掉入场 IDR，之后一整个 GOP 都在输出灰帧"""
        command = ffmpeg_command('rtsp://camera/stream1')

        self.assertIn('low_delay', command)
        self.assertNotIn('nobuffer', command)

    def test_fps_and_scale_run_inside_ffmpeg(self) -> None:
        command = ffmpeg_command(
            'rtsp://camera/stream0', fps=15, scale=(640, 360))

        self.assertEqual(
            command[command.index('-vf') + 1],
            'fps=15,scale=640:360:flags=area')


class DemandGateTest(unittest.TestCase):
    def test_idle_stream_never_starts(self) -> None:
        node = _node()

        with patch('camera_node.camera_node.RtspReader') as reader_type:
            node._step()

        reader_type.assert_not_called()
        self.assertIsNone(node._reader)
        self.assertFalse(node._publish_frames)

    def test_last_consumer_leaving_stops_ffmpeg(self) -> None:
        reader = MagicMock()
        node = _node(reader=reader)

        node._step()

        reader.stop.assert_called_once_with()
        self.assertIsNone(node._reader)

    def test_web_viewer_starts_stream_without_publishing(self) -> None:
        node = _node(viewers=1, image_width=640, image_height=360)

        with patch('camera_node.camera_node.RtspReader') as reader_type:
            node._step()

        self.assertIs(node._reader, reader_type.return_value)
        self.assertFalse(node._publish_frames)

    def test_subscriber_starts_stream_and_enables_publishing(self) -> None:
        node = _node(subscribers=1, image_width=640, image_height=360)

        with patch('camera_node.camera_node.RtspReader') as reader_type:
            node._step()

        self.assertTrue(node._publish_frames)
        command, width, height, _ = reader_type.call_args.args
        self.assertEqual((width, height), (640, 360))
        self.assertIn('scale=640:360', command[command.index('-vf') + 1])

    def test_both_sides_given_skips_ffprobe(self) -> None:
        node = _node(subscribers=1, image_width=640, image_height=360)

        with patch('camera_node.camera_node.RtspReader'), \
                patch('camera_node.camera_node.probe_size') as probe:
            node._step()

        probe.assert_not_called()

    def test_height_only_probes_native_and_derives_width(self) -> None:
        node = _node(subscribers=1, image_height=240)

        with patch('camera_node.camera_node.RtspReader') as reader_type, \
                patch('camera_node.camera_node.probe_size',
                      return_value=(640, 360)):
            node._step()

        command, width, height, _ = reader_type.call_args.args
        self.assertEqual((width, height), (426, 240))
        self.assertIn('scale=426:240', command[command.index('-vf') + 1])

    def test_native_size_is_probed_once_and_cached(self) -> None:
        node = _node(subscribers=1)

        with patch('camera_node.camera_node.RtspReader') as reader_type, \
                patch('camera_node.camera_node.probe_size',
                      return_value=(1920, 1080)) as probe:
            node._step()
            node._reader = None
            node._step()

        probe.assert_called_once_with('rtsp://camera/stream1')
        command = reader_type.call_args.args[0]
        self.assertNotIn('scale', command[command.index('-vf') + 1])

    def test_stale_stream_is_restarted(self) -> None:
        stale = MagicMock()
        stale.alive.return_value = True
        stale.frame_age.return_value = 30.0
        node = _node(subscribers=1, reader=stale,
                     image_width=640, image_height=360)

        with patch('camera_node.camera_node.RtspReader') as reader_type:
            node._step()

        stale.stop.assert_called_once_with()
        self.assertIs(node._reader, reader_type.return_value)

    def test_unreadable_stream_reports_error(self) -> None:
        node = _node(subscribers=1)

        with patch('camera_node.camera_node.probe_size', return_value=None):
            node._step()

        self.assertIsNone(node._reader)
        self.assertIn('ffprobe', node._error)


class RuntimeReconfigureTest(unittest.TestCase):
    def test_changed_size_reopens_the_stream(self) -> None:
        node = _node(subscribers=1, image_width=640, image_height=360)

        with patch('camera_node.camera_node.RtspReader') as reader_type:
            _healthy(reader_type.return_value)
            node._step()
            first = node._reader
            node.params['image_height'] = 240
            node.params['image_width'] = 0
            with patch('camera_node.camera_node.probe_size',
                       return_value=(640, 360)):
                node._step()

        first.stop.assert_called_once_with()
        self.assertEqual(reader_type.call_count, 2)
        self.assertEqual(reader_type.call_args.args[1:3], (426, 240))

    def test_unchanged_settings_leave_the_stream_alone(self) -> None:
        node = _node(subscribers=1, image_width=640, image_height=360)

        with patch('camera_node.camera_node.RtspReader') as reader_type:
            _healthy(reader_type.return_value)
            node._step()
            node._step()

        self.assertEqual(reader_type.call_count, 1)
        reader_type.return_value.stop.assert_not_called()

    def test_negative_size_is_rejected(self) -> None:
        node = _node()

        result = node._check_parameters(
            [Parameter('image_height', Parameter.Type.INTEGER, -1)])

        self.assertFalse(result.successful)

    def test_empty_url_is_rejected(self) -> None:
        node = _node()

        result = node._check_parameters(
            [Parameter('rtsp_url', Parameter.Type.STRING, '')])

        self.assertFalse(result.successful)

    def test_valid_change_is_accepted(self) -> None:
        node = _node()

        result = node._check_parameters(
            [Parameter('image_height', Parameter.Type.INTEGER, 240)])

        self.assertTrue(result.successful)


class RtspReaderTest(unittest.TestCase):
    def test_reads_whole_frames_and_stops_on_short_read(self) -> None:
        frames = []
        command = ['python3', '-c',
                   'import sys; sys.stdout.buffer.write(b"\\x01" * 15)']

        reader = RtspReader(command, 2, 2, lambda raw, w, h: frames.append(raw))
        deadline = time.monotonic() + 5.0
        while reader.alive() and time.monotonic() < deadline:
            time.sleep(0.02)
        reader.stop()

        self.assertEqual(frames, [b'\x01' * 12])

    def test_stop_kills_a_reader_blocked_on_read(self) -> None:
        reader = RtspReader(['sleep', '30'], 4, 4, MagicMock())

        reader.stop()

        self.assertFalse(reader.alive())
        self.assertFalse(reader._thread.is_alive())


class PublishTest(unittest.TestCase):
    def test_payload_is_array_not_bytes(self) -> None:
        """赋 bytes 会触发 rclpy 的逐元素断言，一帧要几百毫秒"""
        node = _node()
        node._publish_frames = True

        node._on_frame(b'\x00' * 12, 2, 2)

        message = node._publisher.publish.call_args.args[0]
        self.assertIsInstance(message.data, array.array)
        self.assertEqual(message.data.typecode, 'B')
        self.assertEqual(
            (message.width, message.height, message.step), (2, 2, 6))
        self.assertEqual(message.encoding, 'bgr8')
        self.assertEqual(message.header.frame_id, 'camera_left')

    def test_frames_are_dropped_when_nobody_wants_them(self) -> None:
        node = _node()

        node._on_frame(b'\x00' * 12, 2, 2)

        node._publisher.publish.assert_not_called()
        self.assertIsNone(node._frame)


class ProbeTest(unittest.TestCase):
    def test_probe_failure_returns_none(self) -> None:
        with patch('subprocess.run',
                   side_effect=subprocess.TimeoutExpired('ffprobe', 8.0)):
            self.assertIsNone(probe_size('rtsp://camera/stream1'))


class ViewerDisconnectTest(unittest.TestCase):
    """相机断线时 MJPEG 循环没有帧可写，只能主动探对端，否则观众数永远挂着"""

    def setUp(self) -> None:
        self.here, self.there = socket.socketpair()
        self.addCleanup(self.here.close)
        self.addCleanup(self.there.close)

    def test_open_connection_counts_as_connected(self) -> None:
        self.assertTrue(connected(self.here))

    def test_closed_peer_is_detected_without_writing(self) -> None:
        self.there.close()

        self.assertFalse(connected(self.here))

    def test_pending_client_bytes_are_left_in_the_buffer(self) -> None:
        self.there.send(b'x')

        self.assertTrue(connected(self.here))
        self.assertEqual(self.here.recv(1), b'x')


if __name__ == '__main__':
    unittest.main()
