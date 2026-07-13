"""KWR57 传输兼容层

通用 python-can I/O 与 CANalyst-II 环境准备由独立 ``can_sdk`` 包实现；
这里仅保留 KWR57 固定 1 Mbps 的默认值及原有导入路径。
"""

from __future__ import annotations

try:
    # CAN-SDK 已安装或已在 PYTHONPATH 中时
    from can_sdk import CanTransport as _CanTransport
except ModuleNotFoundError as exc:
    import sys
    from pathlib import Path
    if exc.name != "can_sdk":
        raise
    _CAN_SDK_ROOT = Path(__file__).resolve().parents[2] / "CAN-SDK"
    if not _CAN_SDK_ROOT.is_dir():
        raise ModuleNotFoundError(
            "无法导入 can_sdk；请安装 CAN-SDK，或保持 CAN-SDK 与 "
            "KWR57-SDK 位于同一 src 目录"
        ) from exc
    sys.path.insert(0, str(_CAN_SDK_ROOT))
    from can_sdk import CanTransport as _CanTransport

from .protocol import CAN_BITRATE


class CanTransport(_CanTransport):
    """共享传输实现的 KWR57 兼容包装，默认比特率为 1 Mbps。"""

    def __init__(self, interface: str, channel: str,
                 bitrate: int = CAN_BITRATE, **kwargs) -> None:
        """打开 CAN 总线

        interface / channel: python-can 的适配器标识，见本模块文档
        bitrate:             总线比特率，KWR57 固定 1Mbps
        **kwargs:            透传给 can.Bus 的其它参数（如 receive_own_messages）
        """
        super().__init__(
            interface=interface,
            channel=channel,
            bitrate=bitrate,
            **kwargs,
        )
