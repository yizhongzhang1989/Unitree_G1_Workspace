"""KWR57 CAN 协议层（纯逻辑，无任何 I/O）

依据《应变式·六轴力/力矩传感器·KWR57 系列 用户手册》(CAN 通信) 4.3 节实现。
本文件只负责“字节 <-> 语义”的转换：构造下发指令、解码上行数据帧，
不涉及任何串口 / CAN 硬件读写，便于单元测试与替换传输后端。

────────────────────────────────────────────────────────────────────────
数据输出（传感器 -> 上位机）
    单精度浮点(IEEE754)，每通道 4 字节，共 6 通道分 3 帧发送，用 CAN ID 区分：

        CAN ID 0x15 :  data[0:4]=Fx   data[4:8]=Fy
        CAN ID 0x16 :  data[0:4]=Fz   data[4:8]=Mx
        CAN ID 0x17 :  data[0:4]=My   data[4:8]=Mz

    实测 KWR57 数据帧中的 IEEE754 浮点采用小端(little-endian)字节序。

指令（上位机 -> 传感器，默认发往 CAN ID 0x10）
    获取实时数据 : [0x8A, 周期高字节, 周期低字节]
                   周期单位 ms（0x0000 表示停止上传）。
                   例：0x8A 0x00 0x10 -> 0x0010=16ms 周期连续上传。
    设置采样率   : [0x60, 档位]
                   01=100Hz 02=200Hz 03=400Hz 04=500Hz 05=600Hz 06=1000Hz
                   出厂默认 500Hz。
    修改 ID      : [0xDE, 0xAA, 主hi, 主lo, 从hi, 从lo, 0x0D, 0x0A]
                   主 = 上位机(接收)ID，从 = 下位机(发送)ID。
    恢复出厂 ID  : [0xDE, 0xDE, 0xDE, 0x0D, 0x0A]（需发往 CAN ID 0x000）
────────────────────────────────────────────────────────────────────────
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple


# --- CAN 标识符 ------------------------------------------------------------
CAN_ID_CMD = 0x10            # 上位机 -> 传感器：默认命令(接收)ID
CAN_ID_DATA_FX_FY = 0x15     # 传感器 -> 上位机：Fx, Fy
CAN_ID_DATA_FZ_MX = 0x16     # 传感器 -> 上位机：Fz, Mx
CAN_ID_DATA_MY_MZ = 0x17     # 传感器 -> 上位机：My, Mz
CAN_ID_FACTORY_RESET = 0x000  # 恢复出厂 ID 时使用的仲裁 ID

# 一个完整采样由这三帧组成，顺序为 0x15 -> 0x16 -> 0x17
DATA_IDS: Tuple[int, int, int] = (
    CAN_ID_DATA_FX_FY, CAN_ID_DATA_FZ_MX, CAN_ID_DATA_MY_MZ,
)

# 手册基本规格写的是 kg / kgm。需要给 ROS geometry_msgs/WrenchStamped 发布
# SI 单位时，通常按 kgf -> N 换算；原始读取路径保持不换算。
KGF_TO_NEWTON = 9.80665

# --- 指令码 ---------------------------------------------------------------
CMD_REALTIME = 0x8A          # 获取实时采样数据（连续上传）
CMD_SAMPLE_RATE = 0x60       # 设置系统采样率
CMD_MODIFY_ID = 0xDE         # 修改 / 恢复 CAN ID

# 采样率(Hz) -> 指令档位参数
SAMPLE_RATE_TABLE: Dict[int, int] = {
    100: 0x01,
    200: 0x02,
    400: 0x03,
    500: 0x04,
    600: 0x05,
    1000: 0x06,
}

# CAN 总线比特率（手册规定 1Mbps）
CAN_BITRATE = 1_000_000

# IEEE754 单精度小端：每帧含 2 个 float
_FLOATS_LE = struct.Struct("<2f")


# --- 数据结构 -------------------------------------------------------------
@dataclass
class Wrench:
    """六轴力/力矩测量值。

    力(fx/fy/fz) 与 力矩(mx/my/mz) 的物理单位取决于传感器出厂配置：
    手册基本规格表标注“默认数据输出单位 kg, kgm”。若上位机需要 N / N·m，
    请按需自行换算（1 kgf ≈ 9.80665 N）。本层不做单位换算，只做字节解码。
    """
    __slots__ = ("fx", "fy", "fz", "mx", "my", "mz")
    fx: float
    fy: float
    fz: float
    mx: float
    my: float
    mz: float

    def as_tuple(self) -> Tuple[float, float, float, float, float, float]:
        return (self.fx, self.fy, self.fz, self.mx, self.my, self.mz)

    def to_si(self) -> "Wrench":
        """按 kgf/kgf*m -> N/N*m 返回一份 SI 单位的 Wrench。"""
        return Wrench(
            self.fx * KGF_TO_NEWTON,
            self.fy * KGF_TO_NEWTON,
            self.fz * KGF_TO_NEWTON,
            self.mx * KGF_TO_NEWTON,
            self.my * KGF_TO_NEWTON,
            self.mz * KGF_TO_NEWTON,
        )


# --- 指令构造（上位机 -> 传感器）------------------------------------------
def build_realtime_command(period_ms: int) -> bytes:
    """构造“获取实时数据”指令。

    period_ms: 连续上传的时间间隔(ms)，范围 0~65535。
               0 表示停止上传。
    返回：CAN 数据域字节（发往 CAN_ID_CMD）。
    """
    if not 0 <= period_ms <= 0xFFFF:
        raise ValueError(f"period_ms 必须在 0~65535 之间，收到 {period_ms}")
    return bytes([CMD_REALTIME, (period_ms >> 8) & 0xFF, period_ms & 0xFF])


def build_stop_command() -> bytes:
    """构造“停止上传”指令（周期=0）。"""
    return build_realtime_command(0)


def build_sample_rate_command(rate_hz: int) -> bytes:
    """构造“设置系统采样率”指令。

    rate_hz: 必须为 SAMPLE_RATE_TABLE 中的合法档位
             (100/200/400/500/600/1000)。
    """
    if rate_hz not in SAMPLE_RATE_TABLE:
        allowed = "/".join(str(r) for r in sorted(SAMPLE_RATE_TABLE))
        raise ValueError(f"不支持的采样率 {rate_hz}Hz，可选：{allowed}")
    return bytes([CMD_SAMPLE_RATE, SAMPLE_RATE_TABLE[rate_hz]])


def build_modify_id_command(host_id: int, sensor_id: int) -> bytes:
    """构造“修改 ID”指令。

    host_id:   新的上位机(接收)ID，2 字节。
    sensor_id: 新的下位机(发送)ID，2 字节。

    警告：该操作会持久化修改传感器 ID，改动后需同步更新上位机配置。
    """
    for name, value in (("host_id", host_id), ("sensor_id", sensor_id)):
        if not 0 <= value <= 0xFFFF:
            raise ValueError(f"{name} 必须在 0~65535 之间，收到 {value}")
    return bytes([
        CMD_MODIFY_ID, 0xAA,
        (host_id >> 8) & 0xFF, host_id & 0xFF,
        (sensor_id >> 8) & 0xFF, sensor_id & 0xFF,
        0x0D, 0x0A,
    ])


def build_factory_reset_id_command() -> bytes:
    """构造“恢复出厂 ID”指令（出厂：接收 0x10 / 发送 0x15）。

    注意：手册要求该指令在 CAN ID 0x000 下发送，见 CAN_ID_FACTORY_RESET。
    """
    return bytes([CMD_MODIFY_ID, 0xDE, 0xDE, 0x0D, 0x0A])


def data_ids_from_base(base_id: int) -> Tuple[int, int, int]:
    """根据下位机发送起始 ID 构造三帧数据 ID。"""
    if not 0 <= base_id <= 0x7FD:
        raise ValueError(f"base_id 必须在 0~0x7FD 之间，收到 0x{base_id:X}")
    return (base_id, base_id + 1, base_id + 2)


# --- 数据解码（传感器 -> 上位机）------------------------------------------
def decode_data_frame(data: bytes) -> Tuple[float, float]:
    """把一帧 8 字节数据域解码为该帧承载的两个 float。

    调用方需自行根据 CAN ID 判断这两个值对应哪两个通道
    （见 DATA_IDS 说明）。
    """
    if len(data) < 8:
        raise ValueError(f"数据帧需 8 字节，收到 {len(data)}")
    a, b = _FLOATS_LE.unpack_from(data, 0)
    return a, b


class WrenchAssembler:
    """把 0x15 / 0x16 / 0x17 三帧组装成一个完整 Wrench。

    传感器每个采样点发送三帧，本类按 CAN ID 缓存各通道值，
    当集齐三帧后输出一个 Wrench 并清空状态，等待下一采样点。
    """

    def __init__(self, data_ids: Sequence[int] = DATA_IDS) -> None:
        if len(data_ids) != 3:
            raise ValueError("data_ids 必须包含 3 个 CAN ID")
        self._data_ids = tuple(data_ids)
        for value in self._data_ids:
            if not 0 <= value <= 0x7FF:
                raise ValueError(f"CAN ID 必须在 0~0x7FF 之间，收到 0x{value:X}")
        # CAN ID -> 采样内帧序号(0/1/2)。push 用单次 dict 查找同时完成
        # “是否数据帧 + 帧定位”，替代原先的集合成员测试 + 链式 == 比较，
        # 减少热路径上对同一 can_id 的重复哈希/比较。
        self._id_to_pos = {cid: i for i, cid in enumerate(self._data_ids)}
        if len(self._id_to_pos) != 3:
            raise ValueError("data_ids 必须是 3 个互不相同的 CAN ID")
        self._vals = [0.0] * 6
        self._expected_index = 0
        self.ignored_frames = 0
        self.malformed_frames = 0
        self.dropped_sequences = 0

    def push(self, can_id: int, data: bytes) -> Optional[Wrench]:
        """喂入一帧数据。若刚好集齐三帧则返回 Wrench，否则返回 None。

        非数据帧（其它 CAN ID）会被忽略并返回 None。
        """
        pos = self._id_to_pos.get(can_id)
        if pos is None:
            self.ignored_frames += 1
            return None
        if len(data) < 8:
            self.malformed_frames += 1
            self.reset(count_drop=True)
            return None

        if pos != self._expected_index:
            # 乱序：只允许在起始帧(0x15)处重新同步，其余帧一律丢弃当前半包，
            # 避免用不同采样周期的 0x16/0x17 拼出跨采样点的假数据。
            if pos != 0:
                self.dropped_sequences += 1
                self._expected_index = 0
                return None
            if self._expected_index != 0:
                self.dropped_sequences += 1

        # 内联解码：len 已在上方校验，直接 unpack_from 省去 decode_data_frame
        # 的函数调用与重复长度检查。合并上述改动后微基准 push 每帧约 -16% CPU；
        # 但真实链路受 CAN/USB I/O 与 ≤1kHz 上传速率限制，端到端吞吐基本不变。
        vals = self._vals
        base = pos * 2
        vals[base], vals[base + 1] = _FLOATS_LE.unpack_from(data, 0)
        if pos == 2:
            self._expected_index = 0
            return Wrench(*vals)
        self._expected_index = pos + 1
        return None

    def reset(self, count_drop: bool = False) -> None:
        """丢弃已缓存的半个采样点。"""
        if count_drop and self._expected_index != 0:
            self.dropped_sequences += 1
        self._expected_index = 0
