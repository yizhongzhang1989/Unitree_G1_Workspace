#!/usr/bin/env python3
"""双总线右夹爪的一次性 PMAX 修改工具

本工具只把固件 RID 21（PMAX）从 12.5 rad 改为 3.14 rad，并保存到 Flash

安全限制：
- 仅操作 /can1 上的 /grip_arm1，命令 ID 为 0x001，反馈 ID 为 0x101。
- 要求 ROS pmax=3.14、verify_limits_on_configure=true、enable_on_start=false。
- 读写寄存器前先调用 /grip_arm1/disable。
- 旧值为 12.5 时才写入；已经是 3.14 时跳过 Flash 写入，其他值直接中止。
- 写入后先回读，再保存到 Flash 并再次回读，最后调用 /grip_arm1/configure，
    以驱动原有的 PMAX/VMAX/TMAX 校验作为最终验收。
- 执行结束后夹爪保持失能。不要把本工具加入启动流程或周期任务。

使用前停止其他占用 CANalyst-II 的进程，以
enable_grippers_on_start:=false 启动双总线 bringup，再在另一个终端运行本脚本。
"""

from __future__ import annotations

import math
import struct
import sys
import time
from typing import Optional

import rclpy
from can_msgs.msg import Frame
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger


GRIPPER_NODE = "/grip_arm1"
TX_TOPIC = "/can1/tx"
RX_TOPIC = "/can1/grip_arm1/rx"
COMMAND_ID = 0x001
FEEDBACK_ID = 0x101
RID_PMAX = 21
EXPECTED_OLD_PMAX = 12.5
TARGET_PMAX = 3.14
RESPONSE_TIMEOUT_S = 1.0

_REQUEST = struct.Struct("<HBBI")
_WRITE_FLOAT = struct.Struct("<HBBf")
_FLOAT32 = struct.Struct("<f")


class PmaxMaintenanceNode(Node):
    def __init__(self) -> None:
        super().__init__("set_grip_arm1_pmax_once")
        rx_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        tx_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self._pmax_reply: Optional[float] = None
        self._tx_pub = self.create_publisher(Frame, TX_TOPIC, tx_qos)
        self._rx_sub = self.create_subscription(
            Frame, RX_TOPIC, self._on_frame, rx_qos)
        self._get_parameters_client = self.create_client(
            GetParameters, f"{GRIPPER_NODE}/get_parameters")
        self._disable_client = self.create_client(
            Trigger, f"{GRIPPER_NODE}/disable")
        self._configure_client = self.create_client(
            Trigger, f"{GRIPPER_NODE}/configure")

    def _on_frame(self, frame: Frame) -> None:
        if (frame.is_error or frame.is_rtr or frame.is_extended
                or frame.dlc != 8
                or frame.id not in (COMMAND_ID, FEEDBACK_ID)):
            return
        data = bytes(frame.data)
        address = data[0] | (data[1] << 8)
        if (address not in (0, COMMAND_ID)
                or data[2] != 0x33
                or data[3] != RID_PMAX):
            return
        value = _FLOAT32.unpack_from(data, 4)[0]
        if math.isfinite(value):
            self._pmax_reply = value

    def wait_for_bridge(self, timeout_s: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if (self._tx_pub.get_subscription_count() > 0
                    and self.count_publishers(RX_TOPIC) > 0):
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError(
            f"bridge topics are not connected: {TX_TOPIC}, {RX_TOPIC}")

    def _call(self, client, request, service_name: str):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"service is unavailable: {service_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=5.0)
        if not future.done():
            raise RuntimeError(f"service timed out: {service_name}")
        exception = future.exception()
        if exception is not None:
            raise RuntimeError(f"service failed: {service_name}: {exception}")
        response = future.result()
        if response is None:
            raise RuntimeError(f"service returned no response: {service_name}")
        return response

    def check_ros_configuration(self) -> None:
        request = GetParameters.Request()
        request.names = [
            "pmax", "verify_limits_on_configure", "enable_on_start"]
        response = self._call(
            self._get_parameters_client, request,
            f"{GRIPPER_NODE}/get_parameters")
        if len(response.values) != 3:
            raise RuntimeError("gripper returned an incomplete parameter response")
        pmax, verify_limits, enable_on_start = response.values
        if pmax.type != ParameterType.PARAMETER_DOUBLE:
            raise RuntimeError("ROS pmax is not a double parameter")
        if verify_limits.type != ParameterType.PARAMETER_BOOL:
            raise RuntimeError("verify_limits_on_configure is not a bool parameter")
        if enable_on_start.type != ParameterType.PARAMETER_BOOL:
            raise RuntimeError("enable_on_start is not a bool parameter")
        if not math.isclose(
                pmax.double_value, TARGET_PMAX,
                rel_tol=1e-6, abs_tol=1e-6):
            raise RuntimeError(
                f"ROS pmax must remain {TARGET_PMAX}, got {pmax.double_value}")
        if not verify_limits.bool_value:
            raise RuntimeError("verify_limits_on_configure must remain enabled")
        if enable_on_start.bool_value:
            raise RuntimeError(
                "restart bringup with enable_grippers_on_start:=false")

    def disable(self) -> None:
        response = self._call(
            self._disable_client, Trigger.Request(),
            f"{GRIPPER_NODE}/disable")
        if not response.success:
            raise RuntimeError(f"disable failed: {response.message}")
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def _publish(self, can_id: int, data: bytes) -> None:
        if len(data) != 8:
            raise ValueError("CAN payload must contain exactly 8 bytes")
        frame = Frame()
        frame.id = can_id
        frame.is_extended = False
        frame.is_rtr = False
        frame.is_error = False
        frame.dlc = 8
        frame.data = list(data)
        self._tx_pub.publish(frame)

    def read_pmax(self) -> float:
        for _ in range(3):
            self._pmax_reply = None
            self._publish(
                0x7FF,
                _REQUEST.pack(COMMAND_ID, 0x33, RID_PMAX, 0),
            )
            deadline = time.monotonic() + RESPONSE_TIMEOUT_S
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self._pmax_reply is not None:
                    return self._pmax_reply
        raise RuntimeError("timed out reading firmware PMAX")

    def write_pmax(self, value: float) -> None:
        self._publish(
            0x7FF,
            _WRITE_FLOAT.pack(COMMAND_ID, 0x55, RID_PMAX, value),
        )

    def save_parameters(self) -> None:
        self._publish(0x7FF, _REQUEST.pack(COMMAND_ID, 0xAA, 0, 0))

    def run(self) -> None:
        self.wait_for_bridge()
        self.check_ros_configuration()
        self.disable()

        old_pmax = self.read_pmax()
        print(f"Firmware PMAX before write: {old_pmax:.6g}")
        if math.isclose(
                old_pmax, TARGET_PMAX, rel_tol=1e-4, abs_tol=1e-4):
            print("Firmware PMAX is already correct; no flash write needed.")
        else:
            if not math.isclose(
                    old_pmax, EXPECTED_OLD_PMAX,
                    rel_tol=1e-4, abs_tol=1e-4):
                raise RuntimeError(
                    f"refusing unexpected firmware PMAX {old_pmax}; "
                    f"expected {EXPECTED_OLD_PMAX}")
            self.write_pmax(TARGET_PMAX)
            volatile_pmax = self.read_pmax()
            if not math.isclose(
                    volatile_pmax, TARGET_PMAX,
                    rel_tol=1e-4, abs_tol=1e-4):
                raise RuntimeError(
                    f"PMAX write verification failed: {volatile_pmax}")
            self.save_parameters()
            time.sleep(0.2)
            saved_pmax = self.read_pmax()
            if not math.isclose(
                    saved_pmax, TARGET_PMAX,
                    rel_tol=1e-4, abs_tol=1e-4):
                raise RuntimeError(
                    f"PMAX save verification failed: {saved_pmax}")
            print(f"Firmware PMAX after save: {saved_pmax:.6g}")

        response = self._call(
            self._configure_client, Trigger.Request(),
            f"{GRIPPER_NODE}/configure")
        if not response.success:
            raise RuntimeError(
                f"built-in PMAX/VMAX/TMAX validation failed: "
                f"{response.message}")
        print(f"Built-in limit validation passed: {response.message}")
        print("Done. grip_arm1 remains disabled.")


def main() -> int:
    rclpy.init()
    node = PmaxMaintenanceNode()
    try:
        node.run()
        return 0
    except (KeyboardInterrupt, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())