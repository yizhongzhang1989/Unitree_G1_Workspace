"""Read-only live observation check: no inference calls or motion publishers."""


import json
import time
from collections import Counter

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

from dry_run_10hz import ObservationProbe


def main():
    rclpy.init(args=['--ros-args', '-r', '__node:=vla_observation_preflight',
                     '-r', '/motion_control/command:=/vla_preflight/unused_command'])
    node = ObservationProbe()

    def infer(observation):
        _ = observation
        raise AssertionError('preflight must never infer')

    node._backend.infer = infer
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    started = time.monotonic()
    next_observation = started + 2.
    failures, ages, skews, process_ms = Counter(), [], [], []
    observation = None
    try:
        while time.monotonic() - started < 15.:
            executor.spin_once(timeout_sec=.02)
            if time.monotonic() < next_observation:
                continue
            next_observation = time.monotonic() + .2
            before = time.monotonic()
            try:
                observation = node._observe()
                acquired = observation.acquired_monotonic
                timing = node._observation_timing
                if acquired is None or timing is None:
                    raise RuntimeError('observation timing unavailable')
                ages.append(time.monotonic() - acquired)
                skews.append(timing.skew_s)
                process_ms.append((time.monotonic() - before) * 1000)
            except Exception as error:
                failures[str(error)] += 1
        print(json.dumps({
            'observations': len(ages), 'failures': dict(failures),
            'age_s_p50_p95_max': np.percentile(ages, [50, 95, 100]).tolist() if ages else [],
            'skew_s_p50_p95_max': np.percentile(skews, [50, 95, 100]).tolist() if skews else [],
            'processing_ms_p50_p95_max': np.percentile(process_ms, [50, 95, 100]).tolist() if ages else [],
            'image_shapes': {key: value.shape for key, value in observation.images.items()} if observation else {},
            'measured_grippers': observation.grippers if observation else {},
            'stream_errors': {slot: reader.error for slot, reader in node._readers.items()},
        }, indent=2))
        assert ages, 'no usable observations'
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
