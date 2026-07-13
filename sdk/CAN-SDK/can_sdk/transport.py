"""适用于单消费者设备 SDK 的轻量 CAN 传输封装

`recv` 从底层队列消费一帧，不提供广播或多订阅语义。需要跨设备/进程共享时，应由上层总线管理器作为唯一接收者并负责分发
"""

from __future__ import annotations

from typing import Optional, Tuple, Union

import can

from .backend import Channel, open_bus

CanPayload = Union[bytes, bytearray, memoryview]


class CanTransport:
    """封装一条 python-can Bus，只负责单帧收发和生命周期管理"""

    def __init__(self, interface: str, channel: Channel, bitrate: int, **bus_kwargs) -> None:
        self._bus = open_bus(
            interface=interface,
            channel=channel,
            bitrate=bitrate,
            **bus_kwargs,
        )
        self._closed = False

    def __enter__(self) -> "CanTransport":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    def send(self, can_id: int, data: bytes, *,
             is_extended_id: bool = False,
             channel: Optional[Channel] = None) -> None:
        """发送一帧 CAN 消息"""
        if self._closed:
            raise RuntimeError("CAN transport is closed")
        message = can.Message(
            arbitration_id=can_id,
            data=bytes(data),
            is_extended_id=is_extended_id,
            channel=channel,
        )
        self._bus.send(message)

    def recv(self, timeout: float = 0.1) -> Optional[Tuple[int, CanPayload]]:
        """消费并返回下一帧 ``(can_id, data)``；超时返回 ``None``。是独占的消费操作，调用方应保证单消费者语义。"""
        if self._closed:
            raise RuntimeError("CAN transport is closed")
        message = self._bus.recv(timeout=timeout)
        if message is None:
            return None
        return message.arbitration_id, message.data

    def drain(self, max_frames: int = 256) -> int:
        """丢弃接收队列中最多 ``max_frames`` 帧，返回实际数量"""
        if max_frames < 0:
            raise ValueError(f"max_frames 不能小于 0，收到 {max_frames}")
        if self._closed:
            raise RuntimeError("CAN transport is closed")

        drained = 0
        while drained < max_frames and self._bus.recv(timeout=0.0) is not None:
            drained += 1
        return drained

    def close(self) -> None:
        """幂等关闭底层总线"""
        if self._closed:
            return
        self._closed = True
        try:
            self._bus.shutdown()
        except Exception:  # noqa: BLE001 - 关闭阶段不掩盖原始业务异常
            pass
