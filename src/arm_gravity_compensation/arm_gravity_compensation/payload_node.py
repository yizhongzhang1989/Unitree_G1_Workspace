#!/usr/bin/env python3
"""从净力旋量估计末端负载，喂给手臂重力补偿。

**不把净力直接喂回补偿**是这里唯一重要的设计决定。净力经位置命令改变手臂受力，再从
传感器回来，就是一条增益 1/kp 的导纳环，而且分不清"一直拎着的负载"和"顶到桌子上的
接触力"。把它压缩成缓变的 (质量, 质心) 参数则天然稳定。

运动学不在这里：``ft_wrench_compensator`` 为了扣掉工具自重本来就要算传感器系的重力
方向，它把那个方向一并发出来。养第二份 joint_states 订阅和第二个运动学模型除了多一份
CPU，还会在两次 FK 落在不同的 joint_states 采样上时悄悄错开。
"""

from pathlib import Path
import time
from typing import Dict, Optional

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import InertiaStamped, Vector3Stamped, WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from .constants import SIDES
from .ft_model import PayloadEstimator, gravity_aligned, instantaneous_mass


_DEFAULTS = {"left": ("/arm0/wrench_net", "/arm0/gravity", "/arm0/payload"),
             "right": ("/arm1/wrench_net", "/arm1/gravity", "/arm1/payload")}


class PayloadEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__("payload_estimator")
        self._tool_com = self._load_tool_com(self.declare_parameter(
            "ft_calibration",
            str(Path(get_package_share_directory("arm_gravity_compensation")) /
                "config" / "ft_calibration.yaml")
        ).get_parameter_value().string_value)
        self._period = 1.0 / max(self.declare_parameter(
            "publish_rate", 10.0).get_parameter_value().double_value, 1e-9)
        self._maximum_mass = self.declare_parameter(
            "maximum_mass", 3.0).get_parameter_value().double_value
        self._minimum_mass = self.declare_parameter(
            "minimum_mass", 0.05).get_parameter_value().double_value
        # 净力必须基本平行于重力才算负载；横向分量超过这个比例就当成接触。
        self._parallel_tolerance = self.declare_parameter(
            "parallel_tolerance", 0.25).get_parameter_value().double_value
        # 重力方向两拍之间动了这么多就说明手臂在转，这一拍不进估计器。
        self._motion_tolerance = self.declare_parameter(
            "motion_tolerance", 0.1).get_parameter_value().double_value
        self._observability_target = self.declare_parameter(
            "observability_target", 0.05).get_parameter_value().double_value
        self._state_timeout = self.declare_parameter(
            "state_timeout_s", 0.5).get_parameter_value().double_value
        forgetting = self.declare_parameter(
            "forgetting", 0.995).get_parameter_value().double_value

        self._gravity: Dict[str, Optional[np.ndarray]] = {
            side: None for side in SIDES}
        self._gravity_stamp = {side: 0.0 for side in SIDES}
        self._previous = {side: None for side in SIDES}
        self._sampled = {side: 0.0 for side in SIDES}
        self._estimators = {
            side: PayloadEstimator(forgetting=forgetting) for side in SIDES}

        sensor_qos = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT)
        # 不能叫 _publishers / _subscriptions：rclpy 的 Node 自己在用这两个名字。
        self._outputs = {}
        self._inputs = []
        for side in SIDES:
            if side not in self._tool_com:
                self.get_logger().warning(
                    "%s has no force sensor calibration; it stays quiet" % side)
                continue
            net, gravity, payload = _DEFAULTS[side]
            self._outputs[side] = self.create_publisher(
                InertiaStamped, self.declare_parameter(
                    "%s_payload_topic" % side, payload
                ).get_parameter_value().string_value, sensor_qos)
            self._inputs.append(self.create_subscription(
                Vector3Stamped, self.declare_parameter(
                    "%s_gravity_topic" % side, gravity
                ).get_parameter_value().string_value,
                (lambda message, side=side: self._on_gravity(side, message)),
                sensor_qos))
            self._inputs.append(self.create_subscription(
                WrenchStamped, self.declare_parameter(
                    "%s_net_topic" % side, net
                ).get_parameter_value().string_value,
                (lambda message, side=side: self._on_wrench(side, message)),
                sensor_qos))
        if not self._outputs:
            raise RuntimeError("no side is calibrated; nothing to estimate")
        self.create_service(Trigger, "~/reset", self._on_reset)

    def _load_tool_com(self, path: str) -> Dict[str, np.ndarray]:
        """工具自己的质心，在质心可辨识之前当作负载质心的先验。"""
        with open(Path(path).expanduser(), "r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if len(document) == 1:
            only = next(iter(document.values()))
            if isinstance(only, dict) and "ros__parameters" in only:
                document = only["ros__parameters"]
        return {side: np.asarray(document[side]["tool_com"], dtype=float)
                for side in SIDES if side in document}

    def _on_gravity(self, side: str, message: Vector3Stamped) -> None:
        # 这条回调按净力的速率来，别在这里碰 numpy：真正要用时才组装。
        self._gravity[side] = (message.vector.x, message.vector.y,
                               message.vector.z)
        self._gravity_stamp[side] = time.monotonic()

    def _on_wrench(self, side: str, message: WrenchStamped) -> None:
        now = time.monotonic()
        if now - self._sampled[side] < self._period:
            return
        gravity = self._gravity[side]
        if (gravity is None or
                now - self._gravity_stamp[side] > self._state_timeout):
            return
        self._sampled[side] = now
        gravity = np.asarray(self._gravity[side], dtype=float)
        force = message.wrench.force
        torque = message.wrench.torque
        wrench = np.array([force.x, force.y, force.z,
                           torque.x, torque.y, torque.z], dtype=float)
        if not (np.all(np.isfinite(wrench)) and np.all(np.isfinite(gravity))
                and np.linalg.norm(gravity) > 1.0):
            return

        settled = self._previous[side] is not None and float(np.linalg.norm(
            gravity - self._previous[side])) < self._motion_tolerance
        self._previous[side] = gravity
        if settled and self._accepts(wrench, gravity):
            self._estimators[side].add(gravity, wrench)
        self._publish(side, message.header.frame_id, message.header.stamp)

    def _accepts(self, wrench: np.ndarray, gravity: np.ndarray) -> bool:
        mass = instantaneous_mass(wrench, gravity)
        if not 0.0 <= mass <= self._maximum_mass:
            return False
        return mass < self._minimum_mass or gravity_aligned(
            wrench, gravity, tolerance=self._parallel_tolerance)

    def _publish(self, side: str, frame: str, stamp) -> None:
        estimate = self._estimators[side].estimate()
        # 质心要多个朝向才可辨识；在那之前用工具自己的质心当先验，总比放在传感器
        # 原点上强。
        weight = float(np.clip(
            estimate.observability / max(self._observability_target, 1e-9),
            0.0, 1.0))
        centre = (1.0 - weight) * self._tool_com[side] + weight * estimate.com
        mass = float(np.clip(estimate.mass, 0.0, self._maximum_mass))
        if mass < self._minimum_mass:
            mass, centre = 0.0, np.zeros(3)

        message = InertiaStamped()
        message.header.stamp = stamp
        message.header.frame_id = frame
        message.inertia.m = mass
        message.inertia.com.x = float(centre[0])
        message.inertia.com.y = float(centre[1])
        message.inertia.com.z = float(centre[2])
        self._outputs[side].publish(message)

    def _on_reset(self, _request, response):
        for estimator in self._estimators.values():
            estimator.reset()
        response.success = True
        response.message = "payload estimate cleared"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PayloadEstimatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
