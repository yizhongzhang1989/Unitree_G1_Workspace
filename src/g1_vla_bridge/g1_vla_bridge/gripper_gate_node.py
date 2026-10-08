"""Manual chunk experiment with gripper-transition truncation."""

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

from g1_vla_bridge.vla_backend import ActionChunk, SIDES
from g1_vla_bridge.vla_node import VlaBridgeNode


def retained_prefix(chunk, gripper_spec, active, current_grippers):
    first = max(0, chunk.horizon - 15)
    limit = chunk.horizon
    high_threshold = float(gripper_spec.to_robot(0.75))
    low_threshold = float(gripper_spec.to_robot(0.25))
    increasing = ((gripper_spec.robot_closed_rad - gripper_spec.robot_open_rad)
                  * (gripper_spec.model_closed - gripper_spec.model_open) > 0)
    above = np.greater_equal if increasing else np.less_equal
    below = np.less_equal if increasing else np.greater_equal
    for side in SIDES:
        if not active[side]:
            continue
        values = np.concatenate(([current_grippers[side]], chunk.grippers[side]))
        high = above(values, high_threshold)
        low = below(values, low_threshold)
        transitions = (high[:-1] & ~high[1:]) | (low[:-1] & ~low[1:])
        changes = np.flatnonzero(transitions[first:])
        if changes.size:
            limit = min(limit, first + int(changes[0]))
    return limit


class GripperGateNode(VlaBridgeNode):
    def _mode_error(self, mode):
        if mode != 'manual':
            return 'Gripper gate supports manual execution only'
        if self._skip_intermediate:
            return 'Gripper gate requires sequential waypoint execution'
        return super()._mode_error(mode)

    def _on_set_skip_intermediate(self, request, response):
        if request.data:
            response.success = False
            response.message = 'Gripper gate requires sequential waypoint execution'
            return response
        return super()._on_set_skip_intermediate(request, response)

    def _accept(self, chunk, elapsed_ms, generation, requested_at=None):
        with self._lock:
            if generation != self._generation or not self._running.is_set():
                return
            limit = retained_prefix(chunk, self._spec.gripper,
                                    self._active, self._grip_command)
            if limit == 0:
                self._chunk = None
                self._cursor = 0
                self._inference_active = False
                self._infer_ms = elapsed_ms
                self._error = 'Gripper transition at first action; waiting for next'
                return
        prediction = ActionChunk(
            poses={side: chunk.poses[side][:limit] for side in SIDES},
            grippers={side: chunk.grippers[side][:limit] for side in SIDES})
        super()._accept(prediction, elapsed_ms, generation, requested_at=requested_at)
        if limit < chunk.horizon:
            self.get_logger().info(
                f'Gripper transition at action {limit + 1}/{chunk.horizon}; '
                f'executing first {limit} actions only')


def main(args=None):
    rclpy.init(args=args)
    node = GripperGateNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
