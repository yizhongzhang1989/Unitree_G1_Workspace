"""末端设备拓扑：由一份清单生成 bridge 参数和设备节点参数。"""

from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple


@dataclass(frozen=True)
class CanBus:
    """一个 ROS 总线名及其对应的 python-can 消息通道"""

    name: str
    channel_id: int

    def __post_init__(self) -> None:
        if not self.name or self.name.strip("/") != self.name:
            raise ValueError(f"总线名必须是非空相对名称，收到 {self.name!r}")
        if self.channel_id < 0:
            raise ValueError(f"channel_id 不能小于 0，收到 {self.channel_id}")

    @property
    def rx_topic(self) -> str:
        return f"/{self.name}/rx"

    @property
    def tx_topic(self) -> str:
        return f"/{self.name}/tx"


@dataclass(frozen=True)
class Kwr57Device:
    """末端设备 bringup 中一台 KWR57 的完整部署参数。"""

    name: str
    bus: CanBus
    cmd_id: int
    data_base_id: int
    wrench_topic: str
    frame_id: str
    period_ms: int = 1
    sample_rate_hz: int = 1000
    publish_rate: float = 0.0
    use_si: bool = False
    autostart: bool = True
    tare_on_start: bool = False

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("KWR57 节点名不能为空")
        if not 0 <= self.cmd_id <= 0x7FF:
            raise ValueError(f"cmd_id 必须是标准 CAN ID，收到 0x{self.cmd_id:X}")
        if not 0 <= self.data_base_id <= 0x7FD:
            raise ValueError(
                f"data_base_id 必须在 0~0x7FD 之间，收到 0x{self.data_base_id:X}")
        if not self.wrench_topic.startswith("/") or self.wrench_topic == "/":
            raise ValueError(
                f"wrench_topic 必须是绝对 ROS 话题，收到 {self.wrench_topic!r}")
        if not self.frame_id:
            raise ValueError("frame_id 不能为空")
        if self.period_ms < 0:
            raise ValueError("period_ms 不能小于 0")
        if self.publish_rate < 0.0:
            raise ValueError("publish_rate 不能小于 0")

    @property
    def data_ids(self) -> Tuple[int, int, int]:
        return (self.data_base_id, self.data_base_id + 1,
                self.data_base_id + 2)

    @property
    def handler_config(self) -> Dict[str, object]:
        """Return the in-process handler config derived from this device."""
        return {
            "channel_id": self.bus.channel_id,
            "node_name": self.name,
            "cmd_id": self.cmd_id,
            "data_base_id": self.data_base_id,
            "topic": self.wrench_topic,
            "frame_id": self.frame_id,
            "period_ms": self.period_ms,
            "sample_rate_hz": self.sample_rate_hz,
            "publish_rate": self.publish_rate,
            "use_si": self.use_si,
            "autostart": self.autostart,
            "tare_on_start": self.tare_on_start,
        }


@dataclass(frozen=True)
class GloriaDevice:
    """末端设备 bringup 中一台 Gloria-M 夹爪的完整部署参数。"""

    name: str
    bus: CanBus
    command_id: int
    feedback_id: int
    rx_topic: str
    joint_name: str
    control_mode: str = "mit"
    safe_position_min: float = 0.0
    safe_position_max: float = 2.77

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("Gloria-M 节点名不能为空")
        for label, can_id in (("command_id", self.command_id),
                              ("feedback_id", self.feedback_id)):
            if not 0 <= can_id <= 0x7FF:
                raise ValueError(
                    f"{label} 必须是标准 CAN ID，收到 0x{can_id:X}")
        if self.feedback_id == 0:
            raise ValueError(
                "robot_bringup 要求 Gloria-M 使用非零 feedback_id，"
                "以安全区分寄存器回包")
        if self.control_mode not in ("mit", "pos_vel"):
            raise ValueError("control_mode 必须是 'mit' 或 'pos_vel'")
        if self.control_mode == "pos_vel" and self.command_id > 0x6FF:
            raise ValueError("PV 模式要求 command_id <= 0x6FF")
        if (self.command_id & 0x0F) == 0:
            raise ValueError(
                "Gloria-M command_id 低 4 位不能为 0；0 为共享反馈保留设备号")
        if 0x7FF in self.active_ids:
            raise ValueError("Gloria-M 活动 ID 不能占用固定请求 ID 0x7FF")
        if not self.rx_topic.startswith("/") or self.rx_topic == "/":
            raise ValueError(
                f"rx_topic 必须是绝对 ROS 话题，收到 {self.rx_topic!r}")
        if not self.joint_name:
            raise ValueError("joint_name 不能为空")
        if self.safe_position_min >= self.safe_position_max:
            raise ValueError("safe_position_min 必须小于 safe_position_max")

    @property
    def rx_ids(self) -> Tuple[int, ...]:
        """节点兼容的反馈 ID：Master ID、命令 ID 和共享 ID 0"""
        return tuple(dict.fromkeys((self.feedback_id, self.command_id, 0x00)))

    @property
    def active_ids(self) -> Tuple[int, ...]:
        """该设备会主动使用的非共享 ID，用于拓扑冲突检查"""
        ids = [self.command_id, self.feedback_id]
        if self.control_mode == "pos_vel":
            ids.append(0x100 + self.command_id)
        return tuple(dict.fromkeys(ids))


def build_bridge_parameters(
        buses: Sequence[CanBus],
        kwr57_devices: Sequence[Kwr57Device],
        gloria_devices: Sequence[GloriaDevice] = ()) -> Dict[str, object]:
    """由设备清单生成 bridge 总线映射及所有设备的专属 RX 路由"""
    if not buses:
        raise ValueError("至少需要配置一条 CAN 总线")

    buses_by_name: Dict[str, CanBus] = {}
    channels: Dict[int, CanBus] = {}
    for bus in buses:
        if bus.name in buses_by_name:
            raise ValueError(f"总线名重复: {bus.name!r}")
        if bus.channel_id in channels:
            raise ValueError(f"channel_id 重复: {bus.channel_id}")
        buses_by_name[bus.name] = bus
        channels[bus.channel_id] = bus

    routes: List[str] = []
    kwr57_data_owners: Dict[Tuple[int, int], str] = {}
    command_owners: Dict[Tuple[int, int], str] = {}
    device_names = set()
    wrench_topics = set()
    rx_topics = set()
    for device in kwr57_devices:
        configured_bus = buses_by_name.get(device.bus.name)
        if configured_bus != device.bus:
            raise ValueError(
                f"设备 {device.name!r} 使用了未配置的总线 {device.bus!r}")
        if device.name in device_names:
            raise ValueError(f"KWR57 节点名重复: {device.name!r}")
        if device.wrench_topic in wrench_topics:
            raise ValueError(
                f"KWR57 Wrench 话题重复: {device.wrench_topic!r}")
        if device.wrench_topic in {
                topic
                for bus in buses
                for topic in (bus.rx_topic, bus.tx_topic)}:
            raise ValueError(
                f"KWR57 Wrench 话题与 CAN 总线话题冲突: "
                f"{device.wrench_topic!r}")
        device_names.add(device.name)
        wrench_topics.add(device.wrench_topic)

        command_key = (device.bus.channel_id, device.cmd_id)
        previous_command_owner = command_owners.get(command_key)
        if previous_command_owner is not None:
            raise ValueError(
                f"KWR57 {previous_command_owner!r} 与 {device.name!r} "
                f"在通道 {device.bus.channel_id} 上共用 cmd_id=0x{device.cmd_id:X}")
        previous_data_owner = kwr57_data_owners.get(command_key)
        if previous_data_owner is not None:
            raise ValueError(
                f"KWR57 {device.name!r} 的 cmd_id=0x{device.cmd_id:X} 与 "
                f"{previous_data_owner!r} 的数据 ID 冲突")
        command_owners[command_key] = device.name

        for can_id in device.data_ids:
            route_key = (device.bus.channel_id, can_id)
            previous_command_owner = command_owners.get(route_key)
            if previous_command_owner is not None:
                raise ValueError(
                    f"KWR57 {device.name!r} 的数据 ID 0x{can_id:X} 与 "
                    f"{previous_command_owner!r} 的 cmd_id 冲突")
            previous_data_owner = kwr57_data_owners.get(route_key)
            if previous_data_owner is not None:
                raise ValueError(
                    f"KWR57 {previous_data_owner!r} 与 {device.name!r} "
                    f"在通道 {device.bus.channel_id} 上共用数据 ID 0x{can_id:X}")
            kwr57_data_owners[route_key] = device.name

    gloria_active_owners: Dict[Tuple[int, int], str] = {}
    gloria_payload_id_owners: Dict[Tuple[int, int], str] = {}
    for device in gloria_devices:
        configured_bus = buses_by_name.get(device.bus.name)
        if configured_bus != device.bus:
            raise ValueError(
                f"设备 {device.name!r} 使用了未配置的总线 {device.bus!r}")
        if device.name in device_names:
            raise ValueError(f"设备节点名重复: {device.name!r}")
        if device.rx_topic in rx_topics:
            raise ValueError(f"设备专属 RX 话题重复: {device.rx_topic!r}")
        if device.rx_topic in wrench_topics:
            raise ValueError(
                f"设备 RX 话题与 KWR57 Wrench 话题冲突: "
                f"{device.rx_topic!r}")
        device_names.add(device.name)
        rx_topics.add(device.rx_topic)

        broadcast_key = (device.bus.channel_id, 0x7FF)
        reserved_owner = (kwr57_data_owners.get(broadcast_key)
                          or command_owners.get(broadcast_key))
        if reserved_owner is not None:
            raise ValueError(
                f"Gloria-M 固定请求 ID 0x7FF 与 KWR57 "
                f"{reserved_owner!r} 的活动 ID 冲突")

        payload_id_key = (device.bus.channel_id, device.command_id & 0x0F)
        previous_payload_owner = gloria_payload_id_owners.get(payload_id_key)
        if previous_payload_owner is not None:
            raise ValueError(
                f"Gloria-M {previous_payload_owner!r} 与 {device.name!r} "
                f"在通道 {device.bus.channel_id} 上的 Data[0] 设备号相同: "
                f"0x{device.command_id & 0x0F:X}")
        gloria_payload_id_owners[payload_id_key] = device.name

        for can_id in device.active_ids:
            key = (device.bus.channel_id, can_id)
            kwr57_data_owner = kwr57_data_owners.get(key)
            if kwr57_data_owner is not None:
                raise ValueError(
                    f"Gloria-M {device.name!r} 的活动 ID 0x{can_id:X} 与 "
                    f"KWR57 {kwr57_data_owner!r} 的数据 ID 冲突")
            kwr57_command_owner = command_owners.get(key)
            if kwr57_command_owner is not None:
                raise ValueError(
                    f"Gloria-M {device.name!r} 的活动 ID 0x{can_id:X} 与 "
                    f"KWR57 {kwr57_command_owner!r} 的 cmd_id 冲突")
            previous_owner = gloria_active_owners.get(key)
            if previous_owner is not None:
                raise ValueError(
                    f"Gloria-M {previous_owner!r} 与 {device.name!r} "
                    f"在通道 {device.bus.channel_id} 上共用活动 ID 0x{can_id:X}")
            gloria_active_owners[key] = device.name

        for can_id in device.rx_ids:
            key = (device.bus.channel_id, can_id)
            if can_id == 0x00:
                kwr57_owner = (kwr57_data_owners.get(key)
                               or command_owners.get(key))
                if kwr57_owner is not None:
                    raise ValueError(f"Gloria-M 共享反馈 ID 0x0 与 KWR57 {kwr57_owner!r} 的活动 ID 冲突")
            routes.append(
                f"{device.bus.channel_id}:0x{can_id:X}:{device.rx_topic}")

    return {
        "channel_ids": [bus.channel_id for bus in buses],
        "bus_names": [bus.name for bus in buses],
        # Foxy 无法从空列表推断参数类型，空拓扑保留一个可忽略的字符串。
        "rx_routes": routes or [""],
    }