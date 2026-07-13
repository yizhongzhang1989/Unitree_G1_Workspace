"""CAN RX 路由参数解析，不依赖 ROS 或 python-can。"""

from typing import Dict, List, Sequence, Tuple


RouteKey = Tuple[int, int]
RouteTable = Dict[RouteKey, Tuple[str, ...]]


def parse_rx_routes(specs: Sequence[str],
                    channel_ids: Sequence[int]) -> RouteTable:
    """解析 ``channel:can_id:topic``，同一 CAN ID 可扇出到多个话题。"""
    destinations: Dict[RouteKey, List[str]] = {}
    valid_channels = set(channel_ids)
    for raw_spec in specs:
        spec = str(raw_spec).strip()
        if not spec:
            continue
        parts = spec.split(":", 2)
        if len(parts) != 3:
            raise ValueError(
                f"rx_routes 条目格式必须是 channel:can_id:topic，收到 {spec!r}")
        channel_text, can_id_text, topic = parts
        try:
            channel_id = int(channel_text, 0)
            can_id = int(can_id_text, 0)
        except ValueError as exc:
            raise ValueError(
                f"rx_routes 通道和 CAN ID 必须是整数，收到 {spec!r}") from exc
        topic = topic.strip()
        if channel_id not in valid_channels:
            raise ValueError(
                f"rx_routes 通道 {channel_id} 不在 channel_ids={channel_ids} 中")
        if not 0 <= can_id <= 0x7FF:
            raise ValueError(f"rx_routes CAN ID 超出范围，收到 0x{can_id:X}")
        if not topic:
            raise ValueError(f"rx_routes 话题不能为空，收到 {spec!r}")

        key = (channel_id, can_id)
        topics = destinations.setdefault(key, [])
        if topic in topics:
            raise ValueError(f"rx_routes 路由重复: {spec!r}")
        topics.append(topic)

    return {key: tuple(topics) for key, topics in destinations.items()}