"""KWR57 系列六轴力/力矩传感器 CAN 通信 Python 驱动。

分层结构：
    protocol   协议层：常量、指令构造、数据帧解码（纯逻辑）
    transport  传输兼容层：复用 can_sdk，并提供 KWR57 默认比特率
    driver     驱动层：KWR57Sensor 高层 API
    cli        应用层：命令行实时读取工具
"""

from .protocol import (
    Wrench,
    WrenchAssembler,
    CAN_BITRATE,
    CAN_ID_CMD,
    DATA_IDS,
    KGF_TO_NEWTON,
    SAMPLE_RATE_TABLE,
    data_ids_from_base,
)
from .transport import CanTransport
from .driver import KWR57Sensor, DeviceSpec

__all__ = [
    "KWR57Sensor",
    "DeviceSpec",
    "CanTransport",
    "Wrench",
    "WrenchAssembler",
    "CAN_BITRATE",
    "CAN_ID_CMD",
    "DATA_IDS",
    "KGF_TO_NEWTON",
    "SAMPLE_RATE_TABLE",
    "data_ids_from_base",
]

__version__ = "0.1.0"
