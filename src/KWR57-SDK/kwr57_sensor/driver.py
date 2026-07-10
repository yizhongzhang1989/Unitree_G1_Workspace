"""驱动层：KWR57 六轴力/力矩传感器的高层 API。

组合“协议层(protocol)”与“传输层(transport)”，向应用层提供面向语义的接口：
    - start_stream / stop_stream : 开始 / 停止实时数据上传
    - set_sample_rate            : 设置系统采样率
    - read_wrench                : 读取下一组完整六轴测量值
    - modify_id / factory_reset_id : 修改 / 恢复 CAN ID

典型用法：

    from kwr57_sensor import KWR57Sensor

    with KWR57Sensor.open(interface="slcan", channel="COM5") as sensor:
        sensor.start_stream(period_ms=1, rate_hz=1000)  # 1ms 周期 + 1000Hz 采样，最高频率
        while True:
            w = sensor.read_wrench()
            if w:
                print(w)
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from . import protocol
from .protocol import Wrench, WrenchAssembler
from .transport import CanTransport


# 传感器处理配置命令（停止 / 设采样率）需要时间，紧随其后的命令会被丢弃。
# 实测 CANalyst-II + 1kHz 下背靠背下发约 80% 概率无法起流（只发 1 帧后沉默），
# 故命令之间留出间隔，并在起流后校验重试。
_CMD_SETTLE_S = 0.1


@dataclass
class DeviceSpec:
    """同总线上一个 KWR57 设备的标识

    key:          用户自定义标签（如 "left"），read() 会用它标明数据来源。
    cmd_id:       该设备的命令(接收)ID；出厂默认 0x10。多设备若要单独下发命令，
                  应各自不同（否则一条命令会同时作用于共享该 ID 的所有设备）
    data_base_id: 该设备的数据发送起始 CAN ID；出厂默认 0x15，对应帧 data_base_id / +1 / +2。**同总线各设备必须互不相同**
    data_ids:     可选，直接指定三帧数据 CAN ID（非连续时用）；给出则忽略 data_base_id
    """
    key: str = "default"
    cmd_id: int = protocol.CAN_ID_CMD
    data_base_id: int = protocol.CAN_ID_DATA_FX_FY
    data_ids: Tuple[int, ...] = ()

    def __post_init__(self) -> None:
        if not self.data_ids:
            self.data_ids = protocol.data_ids_from_base(self.data_base_id)
        else:
            self.data_ids = tuple(self.data_ids)
            if len(self.data_ids) != 3:
                raise ValueError("data_ids 必须是 3 个 CAN ID")


class KWR57Sensor:
    """KWR57 CAN 传感器高层驱动（一条总线，1 个或多个设备）

    通过一个已打开的 CanTransport 与传感器通信；
    也可用便捷类方法: `open` 直接根据 python-can 的 interface/channel 打开。
    """

    def __init__(self, transport: CanTransport, *,
                 cmd_id: int = protocol.CAN_ID_CMD,
                 data_ids: Optional[Sequence[int]] = None,
                 devices: Optional[Sequence[DeviceSpec]] = None) -> None:
        """transport: 已打开的传输层实例

        单设备：给 cmd_id / data_ids（省略则用出厂 0x10 / 0x15,0x16,0x17）
        多设备：给 devices=[DeviceSpec, ...]（各设备 CAN ID 必须互不相同）
        """
        self._t = transport
        if devices is None:
            devices = [DeviceSpec(
                key="default", cmd_id=cmd_id,
                data_ids=(() if data_ids is None else tuple(data_ids)))]
        self._devices: List[DeviceSpec] = list(devices)
        if not self._devices:
            raise ValueError("至少需要一个设备")

        self._assemblers: Dict[str, WrenchAssembler] = {}
        self._route: Dict[int, str] = {}   # CAN ID -> device key
        for d in self._devices:
            if d.key in self._assemblers:
                raise ValueError(f"device key 重复: {d.key!r}")
            self._assemblers[d.key] = WrenchAssembler(d.data_ids)
            for cid in d.data_ids:
                if cid in self._route:
                    raise ValueError(
                        f"数据 CAN ID 0x{cid:X} 被多个设备占用；"
                        "请先用 modify_id / examples/set_id.py 给每个设备设不同 ID")
                self._route[cid] = d.key

        self._primary = self._devices[0].key
        self._single = len(self._devices) == 1
        # 命令按“唯一命令 ID”下发，避免共享 cmd_id 的设备被重复发送
        self._cmd_ids: List[int] = list(dict.fromkeys(d.cmd_id for d in self._devices))
        self._ignored = 0   # 不属于任何设备三帧的 CAN 帧数量

    # --- 便捷构造 ----------------------------------------------------------
    @classmethod
    def open(cls, interface: str, channel: str,
             bitrate: int = protocol.CAN_BITRATE,
             *, cmd_id: int = protocol.CAN_ID_CMD,
             data_base_id: int = protocol.CAN_ID_DATA_FX_FY,
             data_ids: Optional[Sequence[int]] = None,
             devices: Optional[Sequence[DeviceSpec]] = None,
             **bus_kwargs) -> "KWR57Sensor":
        """根据 python-can 的 interface/channel 打开总线并返回驱动实例

        单设备用 cmd_id / data_base_id / data_ids；多设备用 devices=[DeviceSpec]
        """
        transport = CanTransport(interface=interface, channel=channel,
                                 bitrate=bitrate, **bus_kwargs)
        if devices is not None:
            return cls(transport, devices=devices)
        resolved_data_ids = (protocol.data_ids_from_base(data_base_id)
                             if data_ids is None else data_ids)
        return cls(transport, cmd_id=cmd_id, data_ids=resolved_data_ids)

    # --- 上下文管理器 ------------------------------------------------------
    def __enter__(self) -> "KWR57Sensor":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    @property
    def keys(self) -> List[str]:
        """所有设备的 key（保持配置顺序）"""
        return [d.key for d in self._devices]

    # --- 指令：数据流 ------------------------------------------------------
    def start_stream(self, period_ms: int = 1,
                     rate_hz: Optional[int] = 1000, *,
                     verify: bool = True) -> None:
        """开始连续上传实时数据

        period_ms: 上传周期(ms)，默认 1ms，为传感器支持的最高上传频率
        rate_hz:   传感器内部采样率(Hz)，默认 1000（最高档）。设为 None 则沿用传感器当前采样率，不主动修改
        verify:    发送实时命令后校验是否真正起流，未起流则自动重发；默认开启。
                   传感器处理配置命令需要时间，背靠背下发会导致实时命令丢失、进而完全收不到数据，本参数用于消除该启动竞态。
        """
        self.stop_stream()
        time.sleep(_CMD_SETTLE_S)          # 等“停止”生效后再清空接收队列
        self._t.drain()
        if rate_hz is not None:
            self.set_sample_rate(rate_hz)
            time.sleep(_CMD_SETTLE_S)      # 采样率切换需时间生效，否则紧随的实时命令被丢弃
        for asm in self._assemblers.values():
            asm.reset()
        for cid in self._cmd_ids:
            self._t.send(cid, protocol.build_realtime_command(period_ms))
        if verify:
            self._ensure_streaming(period_ms)

    def _ensure_streaming(self, period_ms: int, *, attempts: int = 3,
                          probe_s: float = 0.2) -> bool:
        """确认数据流已真正开始；探测窗口内几乎收不到帧则重发实时命令

        传感器偶发丢失实时命令而完全不上传（表现为只发 1 帧后沉默）。
        在 probe_s 内探测：收到 >=2 帧即认为起流成功；否则重发命令重试。
        成功或重试用尽后都会复位组装器，保证随后 read_wrench 从整包边界开始。
        """
        for _ in range(attempts):
            seen = 0
            deadline = time.monotonic() + probe_s
            while time.monotonic() < deadline:
                if self._t.recv(timeout=0.05) is None:
                    continue
                seen += 1
                if seen >= 2:
                    self._reset_all()
                    return True
            # 探测窗口内基本无帧 -> 判定未起流，重发实时命令
            for cid in self._cmd_ids:
                self._t.send(cid, protocol.build_realtime_command(period_ms))
            time.sleep(_CMD_SETTLE_S)
        self._reset_all()
        return False

    def stop_stream(self) -> None:
        """停止所有设备的数据上传（周期设为 0）"""
        for cid in self._cmd_ids:
            self._t.send(cid, protocol.build_stop_command())

    def set_sample_rate(self, rate_hz: int) -> None:
        """对所有设备设置内部采样率(100/200/400/500/600/1000 Hz)"""
        for cid in self._cmd_ids:
            self._t.send(cid, protocol.build_sample_rate_command(rate_hz))

    # --- 指令：ID 管理（会持久化，谨慎使用；单设备语义）--------------------
    def modify_id(self, host_id: int, sensor_id: int) -> None:
        """修改上位机(接收)与下位机(发送)ID（发往主设备当前命令 ID）。改动会持久保存。"""
        self._t.send(self._devices[0].cmd_id,
                     protocol.build_modify_id_command(host_id, sensor_id))

    def factory_reset_id(self) -> None:
        """恢复出厂 ID（接收 0x10 / 发送 0x15）"""
        self._t.send(protocol.CAN_ID_FACTORY_RESET, protocol.build_factory_reset_id_command())

    # --- 读取 --------------------------------------------------------------
    def read(self, timeout: float = 0.1) -> Optional[Tuple[str, Wrench]]:
        """读取下一组集齐的六轴数据，返回 (device_key, Wrench)

        持续接收 CAN 帧并按 CAN ID 分发给对应设备的组装器；哪个设备先集齐
        三帧就先返回它。timeout 秒内没有任何设备集齐则返回 None。
        单设备时 device_key 恒为 "default"（或你在 DeviceSpec 里给的 key）。
        """
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            frame = self._t.recv(timeout=remaining)
            if frame is None:
                self._reset_all(count_drop=True)
                return None
            can_id, data = frame
            key = self._route.get(can_id)
            if key is None:
                self._ignored += 1
                continue
            wrench = self._assemblers[key].push(can_id, data)
            if wrench is not None:
                return key, wrench

    def read_wrench(self, timeout: float = 0.1) -> Optional[Wrench]:
        """单设备便捷读取：读取下一组完整六轴测量值，直接返回 Wrench

        多设备时返回“最先集齐的那一组”的 Wrench（不含 key）；需要区分来源
        请改用 :meth:`read`。
        """
        item = self.read(timeout=timeout)
        return None if item is None else item[1]

    def read_latest_wrench(self, timeout: float = 0.1,
                           max_extra_frames: int = 64) -> Optional[Wrench]:
        """读取一组数据后尽量追到接收队列里的最新完整样本（单设备场景）

        适合发布线程或可视化在处理速度低于上传速度时使用，避免发布旧样本；
        最多额外消费 max_extra_frames 帧，避免长时间霸占线程。
        """
        item = self.read(timeout=timeout)
        if item is None:
            return None
        key, latest = item
        for _ in range(max_extra_frames):
            frame = self._t.recv(timeout=0.0)
            if frame is None:
                break
            can_id, data = frame
            k = self._route.get(can_id)
            if k is None:
                self._ignored += 1
                continue
            wrench = self._assemblers[k].push(can_id, data)
            if wrench is not None and k == key:
                latest = wrench
        return latest

    def read_wrench_si(self, timeout: float = 0.1) -> Optional[Wrench]:
        """读取下一组数据，并按 kgf/kgf*m -> N/N*m 换算"""
        wrench = self.read_wrench(timeout=timeout)
        return None if wrench is None else wrench.to_si()

    def _reset_all(self, count_drop: bool = False) -> None:
        for asm in self._assemblers.values():
            asm.reset(count_drop=count_drop)

    @property
    def ignored_frames(self) -> int:
        """收到但不属于任何设备数据三帧的 CAN 帧数量"""
        return self._ignored

    @property
    def dropped_sequences(self) -> int:
        """因乱序、丢帧或半包超时而丢弃的采样序列数量（所有设备合计）"""
        return sum(a.dropped_sequences for a in self._assemblers.values())

    @property
    def malformed_frames(self) -> int:
        """数据 CAN ID 正确但 DLC 不足 8 字节的异常帧数量（所有设备合计）"""
        return sum(a.malformed_frames for a in self._assemblers.values())

    def close(self) -> None:
        """停止数据流并关闭总线"""
        try:
            self.stop_stream()
        except Exception:  # noqa: BLE001 - 关闭阶段尽量不抛异常
            pass
        self._t.close()
