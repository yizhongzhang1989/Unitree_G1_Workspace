#!/usr/bin/env python3
"""Publish one latest-state JointState stream for the assembled G1 model."""

import math
from pathlib import Path
import time
from typing import List, Optional, Sequence, cast

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from unitree_hg.msg import LowState

from unitree_g1_description.joint_state_mapping import (
    G1_JOINT_NAMES,
    MotorState,
    gripper_state_to_model_fields,
    load_gripper_model_spec,
    motor_states_to_joint_fields,
)


_GRIPPER_PREFIXES = ("left_", "right_")


class LowStateToJointStates(Node):
    """Cache asynchronous inputs and publish one coherent latest snapshot."""

    def __init__(self) -> None:
        super().__init__("lowstate_to_joint_states")
        package_share = Path(get_package_share_directory(
            "unitree_g1_description"))
        model_path = package_share / "model" / "final.urdf"
        self._gripper_models = {
            prefix: load_gripper_model_spec(model_path, prefix)
            for prefix in _GRIPPER_PREFIXES
        }
        self._joint_state_order = (
            *G1_JOINT_NAMES,
            *(name for prefix in _GRIPPER_PREFIXES
              for name in self._gripper_models[prefix].joint_names),
        )
        self._gripper_state_offsets = {
            prefix: self._joint_state_order.index(
                self._gripper_models[prefix].source_name)
            for prefix in _GRIPPER_PREFIXES
        }
        self.declare_parameter("lowstate_topic", "/lowstate")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("require_pr_mode", True)
        self.declare_parameter("joint_state_publish_rate_hz", 100.0)
        self.declare_parameter(
            "left_gripper_joint_states_topic", "/grip_arm0/joint_states")
        self.declare_parameter(
            "right_gripper_joint_states_topic", "/grip_arm1/joint_states")

        lowstate_topic = str(
            self.get_parameter("lowstate_topic").value)
        joint_states_topic = str(
            self.get_parameter("joint_states_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._require_pr_mode = bool(
            self.get_parameter("require_pr_mode").value)
        publish_rate_hz = self.get_parameter(
            "joint_state_publish_rate_hz").get_parameter_value().double_value
        if not math.isfinite(publish_rate_hz) or publish_rate_hz <= 0.0:
            raise ValueError("joint_state_publish_rate_hz must be finite and > 0")
        gripper_sources = (
            ("left_", str(self.get_parameter(
                "left_gripper_joint_states_topic").value)),
            ("right_", str(self.get_parameter(
                "right_gripper_joint_states_topic").value)),
        )
        if not lowstate_topic or not joint_states_topic:
            raise ValueError("lowstate_topic and joint_states_topic must not be empty")

        lowstate_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        joint_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._publisher = self.create_publisher(
            JointState, joint_states_topic, joint_state_qos)
        # 关节集合固定，因此输入按预计算偏移覆盖数组即可，避免高频 LowState
        # 回调反复分配按名称索引的字典。默认互斥回调组保证更新和发布不并发。
        state_size = len(self._joint_state_order)
        self._positions: List[Optional[float]] = [None] * state_size
        self._velocities: List[Optional[float]] = [None] * state_size
        self._efforts: List[Optional[float]] = [None] * state_size
        self._subscription = self.create_subscription(
            LowState, lowstate_topic, self._on_lowstate, lowstate_qos)
        self._gripper_subscriptions = []
        for joint_prefix, source_topic in gripper_sources:
            if not source_topic:
                continue
            if source_topic == joint_states_topic:
                raise ValueError("gripper source topics must differ from joint_states_topic")
            self._gripper_subscriptions.append(self.create_subscription(
                JointState,
                source_topic,
                lambda message, mapped_prefix=joint_prefix:
                    self._on_gripper_state(
                        mapped_prefix, cast(JointState, message)),
                lowstate_qos,
            ))
        self._publish_timer = self.create_timer(
            1.0 / publish_rate_hz, self._publish_cached_state)
        self._last_warning = 0.0

        self.get_logger().info(
            f"publishing latest {lowstate_topic} and gripper state to "
            f"{joint_states_topic} at {publish_rate_hz:g} Hz "
            f"({len(self._joint_state_order)} assembled joints, including "
            "limit-clamped gripper linkage joints)")

    def _on_lowstate(self, lowstate: LowState) -> None:
        if self._require_pr_mode and lowstate.mode_pr != 0:
            self._warn_invalid(f"mode_pr={lowstate.mode_pr}; mode15 URDF requires PR mode 0")
            return
        try:
            positions, velocities, efforts = motor_states_to_joint_fields(
                cast(Sequence[MotorState], lowstate.motor_state))
        except (AttributeError, TypeError, ValueError) as exc:
            self._warn_invalid(str(exc))
            return

        self._update_cache(0, positions, velocities, efforts)

    def _on_gripper_state(
            self, mapped_prefix: str, source: JointState) -> None:
        try:
            positions, velocities, efforts = gripper_state_to_model_fields(
                source.position,
                source.velocity,
                source.effort,
                self._gripper_models[mapped_prefix],
            )
        except ValueError as exc:
            self._warn_invalid(f"{mapped_prefix}gripper: {exc}")
            return

        offset = self._gripper_state_offsets[mapped_prefix]
        self._update_cache(
            offset,
            positions,
            velocities,
            efforts,
        )

    def _update_cache(
            self, offset: int, positions: Sequence[float],
            velocities: Sequence[float], efforts: Sequence[float]) -> None:
        end = offset + len(positions)
        velocity_values = velocities if velocities else [None] * len(positions)
        effort_values = efforts if efforts else [None] * len(positions)
        self._positions[offset:end] = positions
        self._velocities[offset:end] = velocity_values
        self._efforts[offset:end] = effort_values

    def _publish_cached_state(self) -> None:
        # 只发布已收到过的关节；各输入频率不同，但每帧使用同一发布时间戳。
        active = [
            index for index, position in enumerate(self._positions)
            if position is not None
        ]
        if not active:
            return

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.name = [self._joint_state_order[index] for index in active]
        message.position = [
            cast(float, self._positions[index]) for index in active]
        if all(self._velocities[index] is not None for index in active):
            message.velocity = [
                cast(float, self._velocities[index]) for index in active]
        if all(self._efforts[index] is not None for index in active):
            message.effort = [
                cast(float, self._efforts[index]) for index in active]
        self._publisher.publish(message)

    def _warn_invalid(self, reason: str) -> None:
        now = time.monotonic()
        if now - self._last_warning >= 5.0:
            self.get_logger().warning(f"discarding invalid state: {reason}")
            self._last_warning = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LowStateToJointStates()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()