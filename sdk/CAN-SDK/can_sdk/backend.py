"""python-can 总线创建与适配器环境准备

本模块不依赖 ROS 或具体设备协议。物理总线的所有权、接收循环与帧分发由调用方管理。
"""

from __future__ import annotations

import os
import platform
import threading
from ctypes.util import find_library
from pathlib import Path
from typing import List, Optional, Union

import can

CANALYSTII_INTERFACE = "canalystii"
Channel = Union[str, int]

_BACKEND_LOCK = threading.Lock()
_BACKEND_MARKER = "_can_sdk_libusb_wrapped"


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
    """为 PyUSB 配置跨平台 libusb backend；进程内只安装一次包装"""
    try:
        import libusb_package
        import usb.backend.libusb1
        import usb.core
    except ImportError as exc:
        raise RuntimeError(
            "CANalyst-II requires optional dependencies: "
            "pip install 'can-sdk[canalystii]'"
        ) from exc

    with _BACKEND_LOCK:
        if getattr(usb.core.find, _BACKEND_MARKER, False):
            return

        package_dir = Path(libusb_package.__file__).resolve().parent
        system = _platform_name()
        candidate_paths = [
            str(package_dir / name)
            for name in _libusb_filename_candidates(system)
        ]

        def _find_libusb(_candidate: str) -> Optional[str]:
            system_library = find_library("usb-1.0") or find_library("usb")
            # Linux 优先系统库；Windows/macOS 优先使用 wheel 随附的动态库。
            if system == "linux" and system_library:
                return system_library
            for candidate_path in candidate_paths:
                if Path(candidate_path).exists():
                    return candidate_path
            return system_library

        backend = usb.backend.libusb1.get_backend(find_library=_find_libusb)
        if backend is None:
            raise RuntimeError(
                "Unable to load libusb backend. "
                f"Searched package paths: {candidate_paths} and system libraries."
            )

        original_find = usb.core.find

        def find_with_backend(*args, **kwargs):
            kwargs.setdefault("backend", backend)
            return original_find(*args, **kwargs)

        setattr(find_with_backend, _BACKEND_MARKER, True)
        usb.core.find = find_with_backend


def _ensure_canalystii_usb_access() -> None:
    """Linux 下预检查 USB 权限，避免底层驱动以不明确错误失败"""
    if _platform_name() != "linux":
        return

    try:
        import canalystii.protocol as canalystii_protocol
        import usb.core
    except ImportError:
        return

    device = usb.core.find(
        idVendor=canalystii_protocol.USB_ID_VENDOR,
        idProduct=canalystii_protocol.USB_ID_PRODUCT,
    )
    if device is None:
        return

    usb_bus = getattr(device, "bus", None)
    address = getattr(device, "address", None)
    if usb_bus is None or address is None:
        return

    device_node = Path(f"/dev/bus/usb/{int(usb_bus):03d}/{int(address):03d}")
    if device_node.exists() and not os.access(device_node, os.R_OK | os.W_OK):
        raise PermissionError(
            "CANalyst-II USB permission denied. "
            f"Current user cannot read/write {device_node}. "
            "Add a udev rule for VID:PID 04d8:0053 and replug the device."
        )


def open_bus(interface: str, channel: Channel, bitrate: int, **kwargs) -> "can.BusABC":
    """准备适配器环境并打开一条 python-can 总线"""
    if not interface or not interface.strip():
        raise ValueError("interface 不能为空")
    if bitrate <= 0:
        raise ValueError(f"bitrate 必须大于 0，收到 {bitrate}")

    if _is_canalystii(interface):
        _prepare_canalystii_backend()
        _ensure_canalystii_usb_access()

    return can.Bus(
        interface=interface,
        channel=channel,
        bitrate=bitrate,
        **kwargs,
    )
