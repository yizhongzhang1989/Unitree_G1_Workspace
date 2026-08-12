"""无真机联调：把假的关节角、IMU 和 KWR57 原始读数喂进整条末端负载链路。

用独立的 ROS_DOMAIN_ID 跑，不会碰到机器人上正在跑的任何东西。
"""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import InertiaStamped, WrenchStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from unitree_hg.msg import IMUState

from arm_gravity_compensation.constants import ALL_ARM_JOINTS
from arm_gravity_compensation.ft_model import (
    FtCalibration, expected_raw, tool_wrench)
from arm_gravity_compensation.gravity_model import TorsoArmGravityModel

URDF = "/workspace/src/unitree_g1_description/model/final.urdf"
TABLE = "/tmp/smoke_gravity_table.yaml"
CALIBRATION = "/tmp/smoke_ft_calibration.yaml"

TOOL = FtCalibration(
    force_bias=[11.0, -27.5, 6.25], torque_bias=[0.75, -1.125, 0.25],
    mass=0.72, com=[0.006, -0.009, 0.128], polarity=-1.0,
    origin=[0.0, 0.0, 0.053])
PAYLOAD = FtCalibration(mass=0.85, com=[0.01, -0.02, 0.19])
POSE = np.array([0.35, 0.2, -0.15, 0.9, 0.3, -0.25, 0.4,
                 -0.35, -0.2, 0.15, 0.9, -0.3, 0.25, -0.4])


def _at_sensor(wrench):
    """把对 link 原点取的力矩搬回传感器自己的取矩点。"""
    return np.concatenate([
        wrench[:3], wrench[3:] + np.cross(-np.asarray(TOOL.origin), wrench[:3])])


def prepare():
    model = TorsoArmGravityModel.from_urdf_file(URDF)
    with open(TABLE, "w", encoding="utf-8") as stream:
        yaml.safe_dump({"arm_gravity_compensation": {
            "ros__parameters": model.gravity_table()}}, stream,
            default_flow_style=None, sort_keys=False)
    with open(CALIBRATION, "w", encoding="utf-8") as stream:
        yaml.safe_dump({"ft_wrench_compensator": {"ros__parameters": {
            side: dict(TOOL.to_dict(), frame="%s_kwr57b_link" % side)
            for side in ("left", "right")}}}, stream,
            default_flow_style=None, sort_keys=False)
    return model


class Harness(Node):
    def __init__(self, model):
        super().__init__("end_effector_load_smoke")
        self._model = model
        sensor_qos = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT)
        self._joints = self.create_publisher(JointState, "/joint_states", sensor_qos)
        self._imu = self.create_publisher(IMUState, "/secondary_imu", sensor_qos)
        self._raw = self.create_publisher(
            WrenchStamped, "/arm0/wrench_raw", sensor_qos)
        self.net = None
        self.payload = None
        self.create_subscription(
            WrenchStamped, "/arm0/wrench_net",
            lambda message: setattr(self, "net", message), sensor_qos)
        self.create_subscription(
            InertiaStamped, "/arm0/payload",
            lambda message: setattr(self, "payload", message), sensor_qos)
        self.create_timer(0.01, self._tick)

    def gravity(self):
        """重力在左侧传感器系里的向量，躯干直立。"""
        rotation = self._model.sensor_orientation("left", POSE)
        return rotation.T @ np.array([0.0, 0.0, -9.81])

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = list(ALL_ARM_JOINTS)
        joints.position = POSE.tolist()
        joints.velocity = [0.0] * 14
        self._joints.publish(joints)

        imu = IMUState()
        imu.quaternion = [1.0, 0.0, 0.0, 0.0]
        self._imu.publish(imu)

        gravity = self.gravity()
        reading = expected_raw(TOOL, gravity) + TOOL.polarity * (
            _at_sensor(tool_wrench(PAYLOAD, gravity)).reshape(2, 3) @
            TOOL.rotation.T).reshape(6)
        wrench = WrenchStamped()
        wrench.header.stamp = stamp
        wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z = reading[:3]
        wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z = reading[3:]
        self._raw.publish(wrench)


def main():
    model = prepare()
    log = open("/tmp/smoke_end_effector_load.log", "w", encoding="utf-8")
    # 独立会话 + 独立日志：``ros2 run`` 会再 fork 一层，孙进程继承管道的话，
    # 终止父进程也读不到 EOF。
    nodes = [
        subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                         start_new_session=True)
        for command in (
            ["ros2", "run", "unitree_g1_ros2_control", "ft_wrench_compensator",
             "--ros-args", "-p", "gravity_table:=" + TABLE,
             "-p", "ft_calibration:=" + CALIBRATION],
            ["ros2", "run", "arm_gravity_compensation", "payload_estimator",
             "--ros-args", "-p", "ft_calibration:=" + CALIBRATION],
        )
    ]
    rclpy.init()
    harness = Harness(model)
    deadline = time.monotonic() + 20.0
    try:
        while time.monotonic() < deadline and (
                harness.net is None or harness.payload is None):
            rclpy.spin_once(harness, timeout_sec=0.05)
        if harness.net is None or harness.payload is None:
            print("FAIL: no output (net=%s payload=%s)"
                  % (harness.net, harness.payload))
            return 1
        # 第一帧 payload 必然是零：质量要等估计器攒到第二个静止样本才出来。
        settle = time.monotonic() + 3.0
        while time.monotonic() < settle:
            rclpy.spin_once(harness, timeout_sec=0.05)

        gravity = harness.gravity()
        expected = tool_wrench(PAYLOAD, gravity)
        measured = np.array([
            harness.net.wrench.force.x, harness.net.wrench.force.y,
            harness.net.wrench.force.z, harness.net.wrench.torque.x,
            harness.net.wrench.torque.y, harness.net.wrench.torque.z])
        print("net expected", np.round(expected, 6))
        print("net measured", np.round(measured, 6))
        print("frame", harness.net.header.frame_id)
        print("payload mass", harness.payload.inertia.m, "com", [
            harness.payload.inertia.com.x, harness.payload.inertia.com.y,
            harness.payload.inertia.com.z])
        failures = []
        if np.max(np.abs(measured - expected)) > 1e-9:
            failures.append("net wrench differs by %.3g"
                            % np.max(np.abs(measured - expected)))
        if abs(harness.payload.inertia.m - PAYLOAD.mass) > 1e-6:
            failures.append("payload mass %.4f" % harness.payload.inertia.m)
        if harness.net.header.frame_id != "left_kwr57b_link":
            failures.append("frame %s" % harness.net.header.frame_id)
        for failure in failures:
            print("FAIL:", failure)
        print("OK" if not failures else "FAILED")
        return 1 if failures else 0
    finally:
        harness.destroy_node()
        rclpy.shutdown()
        for process in nodes:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        for process in nodes:
            process.wait(timeout=5)
        log.close()
        print("node log: /tmp/smoke_end_effector_load.log")


if __name__ == "__main__":
    sys.exit(main())
