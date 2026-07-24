from dataclasses import dataclass, field

from arm_gravity_compensation.lowcmd import (
    LOWCMD_PAYLOAD_SIZE,
    crc32_core,
    lowcmd_payload,
    populate_torque_only,
)


@dataclass
class MotorCommand:
    mode: int = 0
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    reserve: int = 0


@dataclass
class LowCommand:
    mode_pr: int = 0
    mode_machine: int = 0
    motor_cmd: list = field(
        default_factory=lambda: [MotorCommand() for _ in range(35)])
    reserve: list = field(default_factory=lambda: [0, 0, 0, 0])
    crc: int = 0


def test_only_requested_tau_slots_are_enabled_and_crc_matches_payload():
    message = LowCommand()
    populate_torque_only(message, 9, {15: 1.25, 28: -0.75})

    assert message.mode_pr == 0
    assert message.mode_machine == 9
    assert len(lowcmd_payload(message)) == LOWCMD_PAYLOAD_SIZE
    assert [index for index, command in enumerate(message.motor_cmd)
            if command.mode == 1] == [15, 28]
    assert message.motor_cmd[15].tau == 1.25
    assert message.motor_cmd[28].tau == -0.75
    assert all(command.q == command.dq == command.kp == command.kd == 0.0
               for command in message.motor_cmd)
    assert message.crc == crc32_core(lowcmd_payload(message))