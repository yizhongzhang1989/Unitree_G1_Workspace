"""传输层：封装 CAN 总线的收发，屏蔽不同 USB-CAN 适配器的差异。

本层基于 python-can (https://python-can.readthedocs.io)，因此几乎支持所有
主流 USB 转 CAN 模块，只需在构造时给出对应的 interface / channel：

    适配器类型              interface        channel 示例
    --------------------    -------------    --------------------------
    CANable / CANtact       "slcan"          "COM5"(Win) / "/dev/ttyACM0"
    候捷/创芯 GC/ZLG (gs_usb) "gs_usb"         "0"
    PEAK PCAN-USB           "pcan"           "PCAN_USBBUS1"
    Kvaser                  "kvaser"         "0"
    Linux SocketCAN         "socketcan"      "can0"

上层驱动只依赖本类暴露的 send / recv / close 三个方法，
因此更换适配器或改用仿真总线时，驱动层代码无需改动。
"""

from __future__ import annotations

import os
import platform
from ctypes.util import find_library
from pathlib import Path
from typing import Optional, Tuple, Union

import can  # python-can

from .protocol import CAN_BITRATE


CanPayload = Union[bytes, bytearray, memoryview]

CANALYSTII_INTERFACE = "canalystii"


def _platform_name() -> str:
    """返回标准化后的平台名（windows/linux/darwin/...）。"""
    return platform.system().lower()


def _libusb_filename_candidates(system: str) -> list[str]:
    """根据平台返回 libusb 文件名候选列表。"""
    if system == "windows":
        return ["libusb-1.0.dll"]
    if system == "darwin":
        return ["libusb-1.0.dylib", "libusb.dylib"]
    # Linux/Unix
    return ["libusb-1.0.so", "libusb-1.0.so.0", "libusb.so"]


def _is_canalystii_interface(interface: str) -> bool:
    """接口名匹配使用大小写不敏感，降低配置拼写差异带来的影响。"""
    return interface.strip().lower() == CANALYSTII_INTERFACE


def _prepare_canalystii_backend() -> None:
    """为 PyUSB 配置 libusb backend（按系统自动选择动态库）。"""
    try:
        import libusb_package
        import usb.backend.libusb1
        import usb.core
    except ImportError as exc:
        raise RuntimeError(
            "CANalyst-II requires extra packages: pip install canalystii libusb-package"
        ) from exc

    if getattr(usb.core.find, "_kwr57_libusb_wrapped", False):
        return

    pkg_dir = Path(libusb_package.__file__).resolve().parent
    system = _platform_name()
    candidate_paths = [
        str(pkg_dir / name) for name in _libusb_filename_candidates(system)
    ]

    def _find_libusb(_candidate: str) -> Optional[str]:
        system_lib = find_library("usb-1.0") or find_library("usb")
        # Linux 上优先系统 libusb（通常与内核/udev 兼容性更好），
        # 其余系统优先使用 libusb-package 随包库。
        if system == "linux" and system_lib:
            return system_lib
        for p in candidate_paths:
            if Path(p).exists():
                return p
        return system_lib

    backend = usb.backend.libusb1.get_backend(find_library=_find_libusb)
    if backend is None:
        raise RuntimeError(
            "Unable to load libusb backend. "
            f"Searched package paths: {candidate_paths} and system libraries."
        )

    original_find = usb.core.find

    def find_with_backend(*args, **kwargs):
        # 统一补上 backend，避免 canalystii 在不同环境下落到错误后端。
        kwargs.setdefault("backend", backend)
        return original_find(*args, **kwargs)

    find_with_backend._kwr57_libusb_wrapped = True
    usb.core.find = find_with_backend


def _ensure_canalystii_usb_access() -> None:
    """Linux 下预检查 USB 设备权限，避免后续底层库崩溃。"""
    if _platform_name() != "linux":
        return

    try:
        import canalystii.protocol as cproto
        import usb.core
    except ImportError:
        return

    dev = usb.core.find(
        idVendor=cproto.USB_ID_VENDOR,
        idProduct=cproto.USB_ID_PRODUCT,
    )
    if dev is None:
        return

    bus = getattr(dev, "bus", None)
    address = getattr(dev, "address", None)
    if bus is None or address is None:
        return

    node = Path(f"/dev/bus/usb/{int(bus):03d}/{int(address):03d}")
    if node.exists() and not os.access(node, os.R_OK | os.W_OK):
        raise PermissionError(
            "CANalyst-II USB permission denied. "
            f"Current user cannot read/write {node}. "
            "Please add a udev rule for VID:PID 04d8:0053 and replug the device."
        )


class CanTransport:
    """对 python-can Bus 的极薄封装：只做“发一帧 / 收一帧 / 关闭”。"""

    def __init__(self, interface: str, channel: str,
                 bitrate: int = CAN_BITRATE, **kwargs) -> None:
        """打开 CAN 总线。

        interface / channel: python-can 的适配器标识，见本模块文档。
        bitrate:             总线比特率，KWR57 固定 1Mbps。
        **kwargs:            透传给 can.Bus 的其它参数（如 receive_own_messages）。
        """
        if _is_canalystii_interface(interface):
            _prepare_canalystii_backend()
            _ensure_canalystii_usb_access()

        self._bus = can.Bus(
            interface=interface,
            channel=channel,
            bitrate=bitrate,
            **kwargs,
        )

    # --- 上下文管理器 ------------------------------------------------------
    def __enter__(self) -> "CanTransport":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    # --- 收发 --------------------------------------------------------------
    def send(self, can_id: int, data: bytes) -> None:
        """发送一帧标准帧(11 位 ID)。"""
        msg = can.Message(
            arbitration_id=can_id,
            data=bytes(data),
            is_extended_id=False,
        )
        self._bus.send(msg)

    def recv(self, timeout: float = 0.1
             ) -> Optional[Tuple[int, CanPayload]]:
        """接收一帧。返回 (can_id, data)，超时返回 None。"""
        msg = self._bus.recv(timeout=timeout)
        if msg is None:
            return None
        return msg.arbitration_id, msg.data

    def drain(self, max_frames: int = 256) -> int:
        """丢弃底层接收队列中的旧帧，返回实际清掉的帧数。"""
        drained = 0
        while drained < max_frames and self._bus.recv(timeout=0.0) is not None:
            drained += 1
        return drained

    def close(self) -> None:
        """关闭总线并释放底层适配器。"""
        try:
            self._bus.shutdown()
        except Exception:  # noqa: BLE001 - 关闭阶段的异常无需向上抛
            pass
