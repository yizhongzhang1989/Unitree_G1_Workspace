"""第1层 CAN Driver：打开一条 python-can 总线（后端无关）。

对 CANalyst-II 需要为 PyUSB 准备 libusb 后端并预检查权限（否则底层库可能报
NoBackendError 或直接崩溃）。本模块与任何具体设备无关，供通用 bridge 使用。
"""

from __future__ import annotations

import os
import platform
from ctypes.util import find_library
from pathlib import Path
from typing import List, Optional

import can  # python-can

CANALYSTII_INTERFACE = "canalystii"


def _platform_name() -> str:
    return platform.system().lower()


def _libusb_filename_candidates(system: str) -> List[str]:
    if system == "windows":
        return ["libusb-1.0.dll"]
    if system == "darwin":
        return ["libusb-1.0.dylib", "libusb.dylib"]
    return ["libusb-1.0.so", "libusb-1.0.so.0", "libusb.so"]


def _is_canalystii(interface: str) -> bool:
    return interface.strip().lower() == CANALYSTII_INTERFACE


def _prepare_canalystii_backend() -> None:
    """为 PyUSB 配置 libusb backend（按系统自动选择动态库）。"""
    try:
        import libusb_package
        import usb.backend.libusb1
        import usb.core
    except ImportError as exc:
        raise RuntimeError(
            "CANalyst-II requires extra packages: "
            "pip install canalystii libusb-package") from exc

    if getattr(usb.core.find, "_canbridge_libusb_wrapped", False):
        return

    pkg_dir = Path(libusb_package.__file__).resolve().parent
    system = _platform_name()
    candidate_paths = [str(pkg_dir / name)
                       for name in _libusb_filename_candidates(system)]

    def _find_libusb(_candidate: str) -> Optional[str]:
        system_lib = find_library("usb-1.0") or find_library("usb")
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
            f"Searched: {candidate_paths} and system libraries.")

    original_find = usb.core.find

    def find_with_backend(*args, **kwargs):
        kwargs.setdefault("backend", backend)
        return original_find(*args, **kwargs)

    find_with_backend._canbridge_libusb_wrapped = True
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

    dev = usb.core.find(idVendor=cproto.USB_ID_VENDOR,
                        idProduct=cproto.USB_ID_PRODUCT)
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
            "Add a udev rule for VID:PID 04d8:0053 and replug the device.")


def open_bus(interface: str, channel: str, bitrate: int, **kwargs) -> "can.BusABC":
    """打开并返回一条 python-can 总线。

    interface/channel: python-can 适配器标识（canalystii/socketcan/slcan/...）。
    bitrate:           总线比特率。
    **kwargs:          透传给 can.Bus（如 receive_own_messages）。
    """
    if _is_canalystii(interface):
        _prepare_canalystii_backend()
        _ensure_canalystii_usb_access()
    return can.Bus(interface=interface, channel=channel, bitrate=bitrate, **kwargs)
