"""Unitree HG LowCmd arm setpoints and CRC calculation."""

import struct
from dataclasses import dataclass
from typing import Mapping


LOWCMD_MOTOR_COUNT = 35
LOWCMD_PAYLOAD_SIZE = 1000
CRC_POLYNOMIAL = 0x04C11DB7


@dataclass(frozen=True)
class MotorSetpoint:
    """One motor slot of ``tau + kp * (q - q_meas) - kd * dq_meas``."""

    tau: float = 0.0
    q: float = 0.0
    kp: float = 0.0
    kd: float = 0.0


def _crc32_word(checksum: int, data: int) -> int:
    bit = 1 << 31
    for _ in range(32):
        if checksum & 0x80000000:
            checksum = ((checksum << 1) ^ CRC_POLYNOMIAL) & 0xFFFFFFFF
        else:
            checksum = (checksum << 1) & 0xFFFFFFFF
        if data & bit:
            checksum ^= CRC_POLYNOMIAL
        bit >>= 1
    return checksum


def _crc32_tables():
    """Split the per-word update into byte tables.

    Each word maps ``checksum`` to ``M(checksum) ^ N(data)`` and both halves are
    linear over GF(2), so one table per input byte reproduces them exactly.
    """
    checksum_tables = []
    data_tables = []
    for byte in range(4):
        shift = 8 * byte
        checksum_tables.append(tuple(
            _crc32_word(value << shift, 0) for value in range(256)))
        data_tables.append(tuple(
            _crc32_word(0, value << shift) for value in range(256)))
    return tuple(checksum_tables), tuple(data_tables)


_CHECKSUM_TABLE, _DATA_TABLE = _crc32_tables()


def crc32_core(payload: bytes) -> int:
    if len(payload) % 4 != 0:
        raise ValueError("CRC payload length must be a multiple of four")
    checksum = 0xFFFFFFFF
    for (data,) in struct.iter_unpack("<I", payload):
        checksum = (
            _CHECKSUM_TABLE[0][checksum & 0xFF] ^
            _CHECKSUM_TABLE[1][(checksum >> 8) & 0xFF] ^
            _CHECKSUM_TABLE[2][(checksum >> 16) & 0xFF] ^
            _CHECKSUM_TABLE[3][checksum >> 24] ^
            _DATA_TABLE[0][data & 0xFF] ^
            _DATA_TABLE[1][(data >> 8) & 0xFF] ^
            _DATA_TABLE[2][(data >> 16) & 0xFF] ^
            _DATA_TABLE[3][data >> 24]
        )
    return checksum


def lowcmd_payload(message) -> bytes:
    payload = bytearray(struct.pack(
        "<BB2x", int(message.mode_pr), int(message.mode_machine)))
    if len(message.motor_cmd) != LOWCMD_MOTOR_COUNT:
        raise ValueError("LowCmd must contain exactly 35 motor commands")
    for command in message.motor_cmd:
        payload.extend(struct.pack(
            "<B3x5fI",
            int(command.mode),
            float(command.q),
            float(command.dq),
            float(command.tau),
            float(command.kp),
            float(command.kd),
            int(command.reserve),
        ))
    payload.extend(struct.pack("<4I", *(int(value) for value in message.reserve)))
    if len(payload) != LOWCMD_PAYLOAD_SIZE:
        raise ValueError(
            "unexpected LowCmd payload size %d" % len(payload))
    return bytes(payload)


def populate_arm_command(message, mode_machine: int,
                         setpoints: Mapping[int, MotorSetpoint]) -> None:
    """Populate a LowCmd with setpoints for the selected motors only.

    ``kp`` and ``kd`` close the position loop inside the motor at its own rate,
    so tracking is immune to the latency of this node. ``dq`` stays zero, which
    makes ``kd`` pure damping that vanishes once the pose has settled.
    """
    invalid = [index for index in setpoints
               if index < 0 or index >= LOWCMD_MOTOR_COUNT]
    if invalid:
        raise ValueError("invalid motor indices: %s" % sorted(invalid))
    message.mode_pr = 0
    message.mode_machine = int(mode_machine)
    for index, command in enumerate(message.motor_cmd):
        setpoint = setpoints.get(index)
        command.mode = 1 if setpoint is not None else 0
        command.q = 0.0 if setpoint is None else float(setpoint.q)
        command.dq = 0.0
        command.tau = 0.0 if setpoint is None else float(setpoint.tau)
        command.kp = 0.0 if setpoint is None else float(setpoint.kp)
        command.kd = 0.0 if setpoint is None else float(setpoint.kd)
        command.reserve = 0
    message.reserve = [0, 0, 0, 0]
    message.crc = crc32_core(lowcmd_payload(message))