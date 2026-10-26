"""独立末端轨迹回放，复用 ArmIKMixin 后直接发布 JointTargets"""

import json
import math
from pathlib import Path
import threading
from time import monotonic

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from g1_motion_session.arm_ik_mixin import ArmIKMixin
from g1_motion_session.joint_output import JointOutput
from record.replay import Playback, lerp_arm, load_commands
from record.replay_source import open_session


class ReplayTask(ArmIKMixin, Node):
    def __init__(self, *, config='', request_topic='/replay/request',
                 status_topic='/replay/status', stop_service='/replay/stop', namespace=None):
        shared = Path(get_package_share_directory('g1_motion_session')) / 'config/arm_ik.yaml'
        parameters = yaml.safe_load(shared.read_text())['/**']['ros__parameters']
        if config:
            document = yaml.safe_load(Path(config).expanduser().read_text())
            parameters.update(document['/**']['ros__parameters'])
        super().__init__('replay', namespace=namespace, use_global_arguments=False,
                         parameter_overrides=[Parameter(name, value=value)
                                              for name, value in parameters.items()])
        hardware = Path(get_package_share_directory('unitree_g1_ros2_control'))
        defaults = yaml.safe_load((hardware / 'config/default_31dof_param.yaml').read_text())
        p = self.declare_parameter
        self.joints31 = list(p('joints', defaults['/**']['ros__parameters']['joints'])
                             .get_parameter_value().string_array_value)
        self.dt = 1.0 / p('control_rate_hz', 50.0).get_parameter_value().double_value
        self.target_topic = p('target_topic', '/forward_position_controller/joint_targets')\
            .get_parameter_value().string_value
        p('robot_description_topic', '/robot_description')
        p('passive_targets', [0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, -0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.request_topic = request_topic
        self.ramp_s = self.declare_parameter('ramp_s', 2.0)\
            .get_parameter_value().double_value
        if not math.isfinite(self.ramp_s) or self.ramp_s < 0.0:
            raise ValueError('ramp_s must be finite and nonnegative')
        self.lock = threading.RLock()
        # 停止、播种与发布串行，停止之后不能提交迟到目标
        self.execution_lock = threading.RLock()
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.output = JointOutput(self, self.target_topic, range(15, 31))
        self._q = np.zeros(31)
        self._joint_received = False
        self._joint_names, self._joint_indices = [], []
        self._playback = None
        self._play_generation = 0
        self._play_start = 0.0
        self._ramp_start = np.zeros(14)
        self._ramp_grip = np.zeros(2)
        self._elapsed = 0.0
        self._duration = 0.0
        self._phase = 'idle'
        self._play_error = ''
        self._label = ''
        self._session_path = ''
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        requests = MutuallyExclusiveCallbackGroup()
        self._status_publisher = self.create_publisher(String, status_topic, qos)
        self.create_subscription(String, self.request_topic, self._on_request, qos,
                                 callback_group=requests)
        self.create_subscription(
            JointState, p('joint_states_topic', '/joint_states').get_parameter_value().string_value,
            self._on_joints, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=MutuallyExclusiveCallbackGroup())
        self.create_service(Trigger, stop_service, self._on_stop,
                            callback_group=self.control_group)
        self.init_arm_ik()
        self.create_timer(self.dt, self._tick_replay, callback_group=self.control_group)
        self.create_timer(0.1, self._publish_status, callback_group=self.control_group)

    def declare(self, name, default):
        return (self.get_parameter(name) if self.has_parameter(name)
                else self.declare_parameter(name, default)).value

    @property
    def ready(self):
        return self.ik_ready and self._joint_received

    def _on_joints(self, message):
        if len(message.position) != len(message.name):
            return
        if list(message.name) != self._joint_names:
            if any(name not in message.name for name in self.joints31):
                return
            self._joint_names = list(message.name)
            self._joint_indices = [self._joint_names.index(name) for name in self.joints31]
        positions = np.asarray(message.position)[self._joint_indices]
        if np.isfinite(positions).all():
            with self.lock:
                self._q[:] = positions
                self._joint_received = True

    def stop(self):
        with self.execution_lock, self.lock:
            self._play_generation += 1
            self._playback = None
            self._phase = 'idle'

    def _on_stop(self, request, response):
        self.stop()
        self._publish_status()
        response.success = True
        response.message = 'Playback stopped'
        return response

    def status(self):
        with self.lock:
            return dict(
                ready=self.ready,
                playing=self._phase == 'loading' or self._playback is not None, phase=self._phase,
                progress=min(self._elapsed / self._duration, 1.0) if self._duration else 0.0,
                elapsed=self._elapsed, duration=self._duration,
                error=self._play_error,
                label=self._label, session_path=self._session_path,
                joint_positions=self.output.positions[15:29].tolist(),
                grip=self.output.positions[29:31].tolist())

    def _publish_status(self):
        self._status_publisher.publish(String(data=json.dumps(self.status())))

    def _on_request(self, message):
        with self.execution_lock, self.lock:
            self.stop()
            token = self._play_generation
            self._elapsed = self._duration = 0.0
            self._phase = 'loading'
            self._play_error = ''
        try:
            request = json.loads(message.data)
            if not isinstance(request, dict):
                raise ValueError('request must be a JSON object')
            path = request['session_path']
            label = request.get('label', '')
            t0, t1, speed = request['t0'], request['t1'], request.get('speed', 1.0)
            if not isinstance(path, str) or not path or not isinstance(label, str):
                raise ValueError('session_path and label must be strings')
            if any(isinstance(value, bool) or not isinstance(value, (int, float))
                   or not math.isfinite(value) for value in (t0, t1, speed)):
                raise ValueError('t0, t1 and speed must be finite numbers')
            with self.lock:
                self._label, self._session_path = label, path
            self._publish_status()
            session = open_session(path)
            timestamps, arms, grips = load_commands(session)
            playback = Playback(timestamps, arms, grips, t0, t1, speed)
        except Exception as error:
            with self.execution_lock, self.lock:
                if token != self._play_generation:
                    return
                self._phase = 'error'
                self._play_error = str(error)
            self._publish_status()
            return
        with self.execution_lock, self.lock:
            if token != self._play_generation:
                return
            if not self.ready:
                self._phase, self._play_error = 'error', '等待 URDF 与关节反馈'
                self._publish_status()
                return
            # 每次从最新实测姿态播种，求解和限速全部复用 mixin
            self.output.positions[:] = self._q
            self.reset_ik(self._q)
            reached = self._ik_status['limited_pose']
            self._ramp_start = np.concatenate([reached['left'], reached['right']])
            self._ramp_grip = self._q[29:31].copy()
            self._playback = playback
            self._play_start = monotonic()
            self._duration = self.ramp_s + playback.duration
            self._phase = 'ramp' if self.ramp_s else 'play'
        self._publish_status()

    def _tick_replay(self):
        with self.execution_lock, self.lock:
            playback = self._playback
            if playback is None:
                return
            elapsed = max(0.0, monotonic() - self._play_start)
            token = self._play_generation
            try:
                if elapsed < self.ramp_s:
                    fraction = 0.5 - 0.5 * math.cos(math.pi * elapsed / self.ramp_s)
                    arms = lerp_arm(self._ramp_start, playback.arm[0], fraction)
                    grips = self._ramp_grip + fraction * (playback.grip[0] - self._ramp_grip)
                    done, phase = False, 'ramp'
                else:
                    arms, grips, done = playback.sample(elapsed - self.ramp_s)
                    phase = 'play'
                if token != self._play_generation:
                    return
                solved, _, _, _ = self.solve_ik(left=arms[:7], right=arms[7:])
                if token != self._play_generation:
                    return
                previous = self.output.positions.copy()
                try:
                    self.output.positions[self.arm_slots] = solved
                    self.output.positions[29:31] = grips
                    self.output.publish()
                except Exception:
                    self.output.positions[:] = previous
                    raise
                self.commit_ik(self.output.positions)
                self._elapsed = min(elapsed, self._duration)
                self._phase = phase
                if done:
                    self._playback = None
                    self._phase = 'done'
                    self._publish_status()
            except Exception as error:
                self.stop()
                self._phase, self._play_error = 'error', str(error)
                self._publish_status()
