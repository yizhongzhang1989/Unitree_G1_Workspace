"""Stable extension API for in-process CAN frame handlers."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Mapping, Optional, Tuple


class FrameDisposition(Enum):
    """Tell the bridge whether normal ROS frame routing should continue."""

    FORWARD = "forward"
    CONSUME = "consume"


@dataclass(frozen=True)
class FrameKey:
    channel_id: int
    can_id: int

    def __post_init__(self) -> None:
        if self.channel_id < 0:
            raise ValueError("channel_id must not be negative")
        if not 0 <= self.can_id <= 0x7FF:
            raise ValueError("can_id must be a standard CAN identifier")


FrameCallback = Callable[[int, Any], FrameDisposition]
LifecycleCallback = Callable[[], None]
SendFrame = Callable[[int, int, bytes], bool]


@dataclass(frozen=True)
class FrameHandlerContext:
    """Generic services made available to a device-owned handler factory."""

    logger: Any
    send_frame: SendFrame
    ros_context: Any


@dataclass(frozen=True)
class FrameHandlerRegistration:
    """Validated routing and lifecycle returned by a handler factory."""

    name: str
    keys: Tuple[FrameKey, ...]
    callback: FrameCallback
    auxiliary_nodes: Tuple[Any, ...] = ()
    start: Optional[LifecycleCallback] = None
    stop: Optional[LifecycleCallback] = None

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("handler name must not be empty")
        if not self.keys:
            raise ValueError("handler must register at least one frame key")
        if len(set(self.keys)) != len(self.keys):
            raise ValueError(f"handler {self.name!r} contains duplicate frame keys")
        if not callable(self.callback):
            raise TypeError("handler callback must be callable")
        if self.start is not None and not callable(self.start):
            raise TypeError("handler start callback must be callable")
        if self.stop is not None and not callable(self.stop):
            raise TypeError("handler stop callback must be callable")


FrameHandlerFactory = Callable[
    [FrameHandlerContext, Mapping[str, Any]], FrameHandlerRegistration]