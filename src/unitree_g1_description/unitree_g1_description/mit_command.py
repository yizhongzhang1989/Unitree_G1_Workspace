"""MIT gain loading and Unitree HG LowCmd CRC helpers."""

import math
import struct
from pathlib import Path
from typing import Dict, Iterable, Protocol, Sequence, Tuple
from xml.etree import ElementTree

import yaml

from unitree_g1_description.joint_state_mapping import G1_JOINT_NAMES


LOW_CMD_MOTOR_COUNT = 35
_LOW_CMD_PAYLOAD_SIZE = 1000
_CRC_POLYNOMIAL = 0x04C11DB7


class MotorCommand(Protocol):
    mode: int
    q: float
    dq: float
    tau: float
    kp: float
    kd: float
    reserve: int


class LowCommand(Protocol):
    mode_pr: int
    mode_machine: int
    motor_cmd: Sequence[MotorCommand]
    reserve: Sequence[int]


def _crc_table() -> Tuple[int, ...]:
    table = []
    for byte in range(256):
        checksum = byte << 24
        for _ in range(8):
            checksum = (
                ((checksum << 1) ^ _CRC_POLYNOMIAL)
                if checksum & 0x80000000
                else checksum << 1
            ) & 0xFFFFFFFF
        table.append(checksum)
    return tuple(table)


_CRC_TABLE = _crc_table()


def load_position_limits(
        path: Path, joint_names: Sequence[str]
        ) -> Dict[str, Tuple[float, float]]:
    """Load finite lower/upper limits for every commanded URDF joint."""
    root = ElementTree.parse(str(path)).getroot()
    requested = set(joint_names)
    limits: Dict[str, Tuple[float, float]] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        if name not in requested:
            continue
        limit = joint.find("limit")
        if limit is None:
            continue
        try:
            lower = float(limit.get("lower", ""))
            upper = float(limit.get("upper", ""))
        except ValueError:
            continue
        if math.isfinite(lower) and math.isfinite(upper) and lower <= upper:
            limits[name] = (lower, upper)

    missing = [name for name in joint_names if name not in limits]
    if missing:
        raise ValueError(
            "URDF has no finite position limits for: " + ", ".join(missing))
    return limits


def load_g1_mit_gains(path: Path) -> Tuple[Tuple[float, ...], Tuple[float, ...]]:
    """Load and validate gains in the physical G1 motor-index order."""
    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    if not isinstance(config, dict):
        raise ValueError("MIT gain file must contain a mapping")
    joint_names = tuple(config.get("joint_names", ()))
    if joint_names != G1_JOINT_NAMES:
        raise ValueError("MIT gain joint_names must exactly match G1 motor order")

    stiffness = _finite_gain_vector(config.get("stiffness"), "stiffness")
    damping = _finite_gain_vector(config.get("damping"), "damping")
    return stiffness, damping


def _finite_gain_vector(values: object, label: str) -> Tuple[float, ...]:
    if not isinstance(values, Sequence) or isinstance(values, (str, bytes)):
        raise ValueError(f"MIT gain {label} must be a sequence")
    result = tuple(float(value) for value in values)
    if len(result) != len(G1_JOINT_NAMES):
        raise ValueError(
            f"MIT gain {label} has {len(result)} values; "
            f"expected {len(G1_JOINT_NAMES)}")
    if not all(math.isfinite(value) and value >= 0.0 for value in result):
        raise ValueError(f"MIT gain {label} values must be finite and non-negative")
    return result


def low_cmd_crc(message: LowCommand) -> int:
    """Return the CRC used by Unitree HG for a ``LowCmd`` message.

    The padding mirrors Unitree's C++ ``LowCmd`` and ``MotorCmd`` structs on
    the little-endian hosts supported by the robot SDK.
    """
    motor_commands = tuple(message.motor_cmd)
    if len(motor_commands) != LOW_CMD_MOTOR_COUNT:
        raise ValueError(
            f"LowCmd has {len(motor_commands)} motors; "
            f"expected {LOW_CMD_MOTOR_COUNT}")

    payload = bytearray(struct.pack(
        "<BB2x", int(message.mode_pr), int(message.mode_machine)))
    for command in motor_commands:
        payload.extend(struct.pack(
            "<B3xfffffI",
            int(command.mode),
            float(command.q),
            float(command.dq),
            float(command.tau),
            float(command.kp),
            float(command.kd),
            int(command.reserve),
        ))

    reserve = tuple(int(value) for value in message.reserve)
    if len(reserve) != 4:
        raise ValueError(f"LowCmd reserve has {len(reserve)} words; expected 4")
    payload.extend(struct.pack("<4I", *reserve))
    if len(payload) != _LOW_CMD_PAYLOAD_SIZE:
        raise AssertionError(f"unexpected LowCmd payload size: {len(payload)}")

    words = struct.unpack(f"<{len(payload) // 4}I", payload)
    return crc32_core(words)


def crc32_core(words: Iterable[int]) -> int:
    """Match Unitree's non-reflected CRC-32 implementation word for word."""
    checksum = 0xFFFFFFFF
    for word in words:
        data = int(word) & 0xFFFFFFFF
        for shift in (24, 16, 8, 0):
            index = ((checksum >> 24) ^ (data >> shift)) & 0xFF
            checksum = ((checksum << 8) ^ _CRC_TABLE[index]) & 0xFFFFFFFF
    return checksum