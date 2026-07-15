"""KWR57-owned in-process frame handler factory for ``can_bridge_ros``."""

from __future__ import annotations

import json
from typing import Any, Dict, Mapping

from rclpy.parameter import Parameter

from can_bridge_ros.handler_api import (
    FrameDisposition,
    FrameHandlerContext,
    FrameHandlerRegistration,
    FrameKey,
)
from kwr57_ros.ft_sensor_node import KWR57DeviceNode
from kwr57_sensor import protocol


_DEFAULTS: Dict[str, Any] = {
    "channel_id": 0,
    "node_name": "kwr57_ft_sensor",
    "cmd_id": 0x10,
    "data_base_id": 0x15,
    "topic": "/kwr57_ft_sensor/wrench_raw",
    "frame_id": "kwr57_ft_sensor_link",
    "period_ms": 1,
    "sample_rate_hz": 1000,
    "publish_rate": 0.0,
    "use_si": False,
    "autostart": True,
    "tare_on_start": False,
}

_FACTORY_PATH = "kwr57_ros.bridge_handler:create_frame_handler"


def _validated_config(config: Mapping[str, Any]) -> Dict[str, Any]:
    unknown = set(config) - set(_DEFAULTS)
    if unknown:
        raise ValueError(f"unknown KWR57 handler config fields: {sorted(unknown)}")
    values = {**_DEFAULTS, **dict(config)}
    for name in ("channel_id", "cmd_id", "data_base_id", "period_ms",
                 "sample_rate_hz"):
        if isinstance(values[name], bool) or not isinstance(values[name], int):
            raise TypeError(f"KWR57 handler {name} must be an integer")
    for name in ("publish_rate",):
        if isinstance(values[name], bool) or not isinstance(values[name], (int, float)):
            raise TypeError(f"KWR57 handler {name} must be numeric")
        values[name] = float(values[name])
    for name in ("use_si", "autostart", "tare_on_start"):
        if not isinstance(values[name], bool):
            raise TypeError(f"KWR57 handler {name} must be boolean")
    for name in ("node_name", "topic", "frame_id"):
        if not isinstance(values[name], str) or not values[name].strip():
            raise ValueError(f"KWR57 handler {name} must be a non-empty string")

    channel_id = values["channel_id"]
    cmd_id = values["cmd_id"]
    data_base_id = values["data_base_id"]
    if channel_id < 0:
        raise ValueError("KWR57 handler channel_id must not be negative")
    if not 0 <= cmd_id <= 0x7FF:
        raise ValueError("KWR57 handler cmd_id must be a standard CAN ID")
    if not 0 <= data_base_id <= 0x7FD:
        raise ValueError("KWR57 handler data_base_id must be between 0 and 0x7FD")
    if cmd_id in range(data_base_id, data_base_id + 3):
        raise ValueError("KWR57 handler cmd_id conflicts with its data IDs")
    if not 0 <= values["period_ms"] <= 0xFFFF:
        raise ValueError("KWR57 handler period_ms must be between 0 and 65535")
    if values["sample_rate_hz"] not in protocol.SAMPLE_RATE_TABLE:
        allowed = sorted(protocol.SAMPLE_RATE_TABLE)
        raise ValueError(
            f"KWR57 handler sample_rate_hz must be one of {allowed}")
    if values["publish_rate"] < 0.0:
        raise ValueError("KWR57 handler publish_rate must not be negative")
    return values


def build_frame_handler_spec(config: Mapping[str, Any]) -> str:
    """Build one validated bridge handler JSON parameter value."""
    values = _validated_config(config)
    return json.dumps(
        {"factory": _FACTORY_PATH, "config": values},
        separators=(",", ":"),
        sort_keys=True,
    )


def create_frame_handler(
        context: FrameHandlerContext,
        config: Mapping[str, Any]) -> FrameHandlerRegistration:
    """Create one KWR57 ROS node and register its three data CAN IDs."""
    values = _validated_config(config)
    parameter_names = (
        "cmd_id", "data_base_id", "topic", "frame_id", "period_ms",
        "sample_rate_hz", "publish_rate", "use_si", "autostart",
        "tare_on_start",
    )
    overrides = [
        Parameter(name, value=values[name])
        for name in parameter_names
    ]
    channel_id = values["channel_id"]
    node = KWR57DeviceNode(
        node_name=values["node_name"],
        context=context.ros_context,
        parameter_overrides=overrides,
        use_global_arguments=False,
        direct_rx=True,
        direct_tx=lambda can_id, data: context.send_frame(
            channel_id, can_id, data),
        defer_autostart=True,
    )
    data_base_id = values["data_base_id"]

    def handle_frame(_channel_id: int, message: Any) -> FrameDisposition:
        handled = node.handle_can_frame(
            message.arbitration_id,
            message.data,
            is_extended=message.is_extended_id,
            is_rtr=message.is_remote_frame,
            is_error=getattr(message, "is_error_frame", False),
            dlc=message.dlc,
        )
        return FrameDisposition.CONSUME if handled else FrameDisposition.FORWARD

    keys = tuple(
        FrameKey(channel_id, can_id)
        for can_id in range(data_base_id, data_base_id + 3)
    )
    return FrameHandlerRegistration(
        name=values["node_name"],
        keys=keys,
        callback=handle_frame,
        auxiliary_nodes=(node,),
        start=node.activate,
        stop=node.stop_device,
    )