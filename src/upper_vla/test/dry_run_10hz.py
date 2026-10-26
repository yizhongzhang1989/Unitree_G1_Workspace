"""In-memory live observation and HTTP probe; no command publisher or data dumps."""


import json
from pathlib import Path
import time
from unittest.mock import Mock

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from unitree_g1_ros2_control.msg import JointTargets

from upper_vla.backends.cogact_unitree import CogACTUnitreeBackend, build_payload
from upper_vla.timed_actions import TimedActions
from upper_vla.vla_node import VlaBridgeNode


class ObservationProbe(VlaBridgeNode):
    def create_publisher(self, msg_type, *args, **kwargs):
        if msg_type is JointTargets:
            return Mock(publish=Mock(side_effect=AssertionError('probe cannot publish commands')))
        return super().create_publisher(msg_type, *args, **kwargs)

    def _engage_executor(self, positions):
        raise RuntimeError('observation probe cannot engage')

    def _on_start(self, request, response):
        _ = request
        response.success, response.message = False, 'observation probe only'
        return response

    def _on_home(self, request, response):
        _ = request
        response.success, response.message = False, 'observation probe only'
        return response


def main():
    root = Path(__file__).resolve().parents[1]
    rclpy.init(args=[
        '--ros-args', '--params-file', str(root / 'config/vla_bridge.yaml'),
        '--params-file', str(root / 'config/backends/cogact_unitree.yaml'),
        '-r', '__ns:=/vla_http_probe',
    ], signal_handler_options=SignalHandlerOptions.NO)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = ObservationProbe()
        executor.add_node(node)
        backend = node._backend
        if not isinstance(backend, CogACTUnitreeBackend):
            raise TypeError('probe requires CogACTUnitreeBackend')
        print(json.dumps({'config': backend._config, 'probe_only': True}), flush=True)
        deadline = time.monotonic() + 30.
        observation, failure = None, ''
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            try:
                observation = node._observe()
                break
            except Exception as error:
                failure = str(error)
        if observation is None:
            raise RuntimeError(f'no valid live observation: {failure}')
        assert observation.history == ()
        acquired = observation.acquired_monotonic
        if acquired is None:
            raise RuntimeError('observation acquisition time unavailable')
        payload = build_payload(observation, backend._frame)
        print(json.dumps({
            'state_shapes': {key: list(np.shape(value)) for key, value in payload['state'].items()},
            'source_images': {key: list(value.shape) for key, value in observation.images.items()},
            'wire_images': [640, 360], 'history_state': None, 'history_action': None,
            'observation_timing': vars(node._observation_timing),
            'calibration_sources': ['head CameraInfo', 'camera_calibration/config/calibration.yaml',
                                    'live robot_description FK and observation-time head TF'],
        }), flush=True)
        started = time.monotonic()
        chunk = backend.infer(observation)
        finished = time.monotonic()
        queue = TimedActions(
            node._action_rate, acquired,
            first_offset_steps=1, execution_rate=node._execution_rate)
        accepted = queue.merge(chunk, acquired, finished)
        print(json.dumps({
            'empty_history_http': 'PASS', 'horizon': chunk.horizon,
            'elapsed_s': finished - started, 'accepted_future_steps': accepted,
            'timing': queue.last_merge,
            'poses': {side: list(value.shape) for side, value in chunk.poses.items()},
            'grippers': {side: list(value.shape) for side, value in chunk.grippers.items()},
            'short_full_history_http': 'NOT RUN: no genuinely executed episode commands',
            'robot_commands_published': 0,
        }), flush=True)
    finally:
        if node is not None:
            node.shutdown()
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
