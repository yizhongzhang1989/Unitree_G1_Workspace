"""Offline capture services and controller commands; never sends robot commands."""

from __future__ import annotations

import queue
import time

import numpy as np
import rclpy
from g1_mocap_msgs.msg import MocapFrame, MocapStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from .capture_stream import CaptureStream
from .mocap_node import fill_pose, to_points
from .motion_capture import JOINT_NAMES, save_motion, validate_label
from .motion_model import MotionModel
from .retarget import Retargeter
from .urdf import DEFAULT_URDF, resolve_package_path


class MotionCaptureNode(Node):
    def __init__(self):
        super().__init__('motion_capture')

        def parameter(name, default):
            return self.declare_parameter(name, default).value

        configured_joints = parameter('joints', Parameter.Type.STRING_ARRAY)
        if tuple(configured_joints) != JOINT_NAMES:
            raise ValueError('Configured joints differ from the motion CSV contract')
        urdf = resolve_package_path(parameter('urdf_path', DEFAULT_URDF))
        kinematics = MotionModel(urdf).kin
        self.key_bodies = list(parameter('key_bodies', Parameter.Type.STRING_ARRAY))
        retargeter = Retargeter(
            kinematics, key_bodies=self.key_bodies,
            anchor_body=parameter('anchor_body', 'torso_link'),
            default_joint_pos=np.asarray(parameter('default_joint_pos', Parameter.Type.DOUBLE_ARRAY)),
            foot_ground_clearance_m=parameter('foot_ground_clearance_m', 0.03))
        self.directory = parameter('output_dir', '~/motions_dataset')
        self.category = validate_label(parameter('category', 'locomotion'))
        self.action = validate_label(parameter('action', 'walk_forward'))
        self.model_confirmed = parameter('model_confirmed', False)
        self.max_duration = float(parameter('max_duration_s', 60.0))
        if not 2.0 <= self.max_duration <= 60.0:
            raise ValueError('max_duration_s must be between 2 and 60')
        self.quality = dict(
            max_gap=parameter('max_gap_s', 0.1),
            max_root_speed=parameter('max_root_speed_m_s', 8.0),
            max_root_angular_speed=parameter('max_root_angular_speed_rad_s', 15.0))
        self.controller_buttons = parameter('controller_buttons', True)
        self.commands = queue.SimpleQueue()
        self.buttons = (False, False)
        self.last_outcome = 'Idle; calibrate before starting'
        self.frame_publisher = self.create_publisher(
            MocapFrame, '~/frame', QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.status_publisher = self.create_publisher(MocapStatus, '~/status', 10)
        self.stream = CaptureStream(
            retargeter, limits=kinematics.limits(),
            host=parameter('host', '0.0.0.0'), port=parameter('port', 18001),
            token=parameter('token', ''), log=self.get_logger().info)
        self.stream.on_preview = self._preview
        self.stream.on_controllers = self._controllers
        for name, callback in (('calibrate', self._calibrate), ('start', self._start),
                               ('stop', self._stop), ('discard', self._discard),
                               ('capture_status', self._capture_status)):
            self.create_service(Trigger, f'~/{name}', callback)
        self.create_timer(0.02, self._tick)
        self.create_timer(1.0, self._status)
        self.stream.start()
        self.get_logger().info(
            f'Capture source ready, model={urdf}, output={self.directory}, '
            f'action={self.category}_{self.action}. '
            'Calibrate standing still, then ~/start; ~/stop validates and saves. '
            'Right A: start/stop; right B: discard. No robot control is started.')

    def _preview(self, stamped, raw, result):
        message = MocapFrame()
        offset = self.get_clock().now().nanoseconds * 1e-9 - time.monotonic()
        message.header.stamp = rclpy.time.Time(seconds=stamped + offset).to_msg()
        message.header.frame_id = 'mocap_capture_world'
        message.seq = raw.seq & 0xFFFFFFFF
        message.body_status, message.body_message = raw.status, raw.message
        message.joint_names = list(JOINT_NAMES)
        message.joint_positions = result.joint_pos.tolist()
        fill_pose(message.root, result.root_pos, result.root_quat)
        fill_pose(message.anchor, result.anchor_pos, result.anchor_quat)
        message.key_body_names = self.key_bodies
        message.key_body_positions = to_points(result.key_pos)
        message.human_joints = to_points(raw.positions)
        self.frame_publisher.publish(message)

    def _controllers(self, controllers):
        right = controllers[1]
        buttons = (right.connected and right.a_x, right.connected and right.b_y)
        if self.controller_buttons:
            with self.stream.capture_lock:
                clip = self.stream.clip
                if buttons[1] and not self.buttons[1]:
                    self.stream.seal()
                    self.commands.put(('discard', clip))
                elif buttons[0] and not self.buttons[0]:
                    self.stream.seal()
                    self.commands.put(('stop' if clip is not None else 'start', clip))
        self.buttons = buttons

    def _calibrate(self, request, response):
        try:
            calibration = self.stream.calibrate()
            response.success = True
            response.message = f'Calibrated: scale={calibration.scale:.4f}'
        except (ValueError, RuntimeError) as exc:
            response.success, response.message = False, str(exc)
        return response

    def _start(self, request, response):
        try:
            if not self.model_confirmed:
                raise RuntimeError('Confirm the client model with model_confirmed:=true at launch')
            self.stream.begin(duration_limit=self.max_duration, **self.quality)
            self.last_outcome = f'Recording {self.category}_{self.action}'
            response.success, response.message = True, self.last_outcome
            self.get_logger().info(self.last_outcome)
        except (ValueError, RuntimeError) as exc:
            response.success, response.message = False, str(exc)
        return response

    def _stop(self, request, response):
        try:
            with self.stream.capture_lock:
                self.stream.seal()
                clip = self.stream.finish()
            rows = clip.resample()
            path = save_motion(self.directory, rows, category=self.category, action=self.action)
            self.last_outcome = f'Saved {path} ({len(rows)} frames, 50 Hz)'
            if len(rows) < 200:
                self.get_logger().warning('Take is valid but shorter than the recommended 4 seconds')
            response.success = True
            self.stream.haptic('right', 0.8, 150)
        except (ValueError, RuntimeError, OSError) as exc:
            self.last_outcome = f'Not saved: {exc}'
            response.success = False
        response.message = self.last_outcome
        self.get_logger().info(self.last_outcome)
        return response

    def _discard(self, request, response):
        try:
            self.stream.finish()
            self.last_outcome = 'Take discarded; nothing written'
            response.success = True
        except RuntimeError as exc:
            self.last_outcome = str(exc)
            response.success = False
        response.message = self.last_outcome
        self.get_logger().info(self.last_outcome)
        return response

    def _capture_status(self, request, response):
        with self.stream.capture_lock:
            clip = self.stream.clip
            if clip is None:
                response.message = self.last_outcome
            elif self.stream.complete:
                response.message = f'Recording stopped; pending save/discard; error={clip.reason or "none"}'
            else:
                response.message = f'Recording: {len(clip.rows)} source frames; error={clip.reason or "none"}'
        response.success = True
        return response

    def _tick(self):
        while not self.commands.empty():
            command, expected_clip = self.commands.get()
            if self.stream.clip is not expected_clip:
                continue
            callback = {'discard': self._discard, 'stop': self._stop, 'start': self._start}[command]
            response = callback(None, Trigger.Response())
            if not response.success:
                self.get_logger().warning(response.message)
        with self.stream.capture_lock:
            clip = self.stream.clip
            if clip is None:
                return
            if not self.stream.complete and time.monotonic() - self.stream.last_valid_arrival > self.quality['max_gap']:
                clip.reject('Tracking timeout')
            if clip.reason:
                self.last_outcome = f'Discarded: {clip.reason}'
                self.stream.finish()
                self.get_logger().error(self.last_outcome)
                self.stream.haptic('right', 0.3, 500)
                return
            complete = self.stream.complete
        if complete:
            self._stop(None, Trigger.Response())

    def _status(self):
        stats = self.stream.stats()
        message = MocapStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.connected, message.calibrated = stats.connected, self.stream.calibrated
        message.frames, message.dropped = stats.frames, stats.dropped
        message.body_status, message.body_message = stats.status, stats.message
        message.last_error = self.last_outcome
        calibration = self.stream.calibration
        if calibration is not None:
            message.scale, message.stand_height = calibration.scale, calibration.stand_height
        self.status_publisher.publish(message)

    def destroy_node(self):
        self.stream.stop()
        with self.stream.capture_lock:
            if self.stream.clip is not None:
                self.stream.finish()
                self.get_logger().warning('Shutdown: unfinished take discarded')
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = MotionCaptureNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
