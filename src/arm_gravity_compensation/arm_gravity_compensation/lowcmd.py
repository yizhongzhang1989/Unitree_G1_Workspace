"""Unitree HG LowCmd torque-only population and CRC calculation."""

import struct
from typing import Mapping


LOWCMD_MOTOR_COUNT = 35
LOWCMD_PAYLOAD_SIZE = 1000
CRC_POLYNOMIAL = 0x04C11DB7


def crc32_core(payload: bytes) -> int:
    if len(payload) % 4 != 0:
        raise ValueError("CRC payload length must be a multiple of four")
    checksum = 0xFFFFFFFF
    for offset in range(0, len(payload), 4):
        data = struct.unpack_from("<I", payload, offset)[0]
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


def populate_torque_only(message, mode_machine: int,
                         torques: Mapping[int, float]) -> None:
    """Populate a LowCmd with only selected motor ``tau`` fields enabled."""
    invalid = [index for index in torques
               if index < 0 or index >= LOWCMD_MOTOR_COUNT]
    if invalid:
        raise ValueError("invalid motor indices: %s" % invalid)
    message.mode_pr = 0
    message.mode_machine = int(mode_machine)
    for index, command in enumerate(message.motor_cmd):
        command.mode = 1 if index in torques else 0
        command.q = 0.0
        command.dq = 0.0
        command.tau = float(torques.get(index, 0.0))
        command.kp = 0.0
        command.kd = 0.0
        command.reserve = 0
    message.reserve = [0, 0, 0, 0]
    message.crc = crc32_core(lowcmd_payload(message))