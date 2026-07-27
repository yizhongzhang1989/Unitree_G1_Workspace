from dataclasses import dataclass, field

from arm_gravity_compensation.lowcmd import (
    LOWCMD_PAYLOAD_SIZE,
    MotorSetpoint,
    crc32_core,
    lowcmd_payload,
    populate_arm_command,
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


def test_only_requested_slots_are_enabled_and_crc_matches_payload():
    message = LowCommand()
    populate_arm_command(message, 9, {
        15: MotorSetpoint(tau=1.25, q=0.4, kp=40.0, kd=3.0),
        28: MotorSetpoint(tau=-0.75, q=-0.2, kp=20.0, kd=1.5),
    })

    assert message.mode_pr == 0
    assert message.mode_machine == 9
    assert len(lowcmd_payload(message)) == LOWCMD_PAYLOAD_SIZE
    assert [index for index, command in enumerate(message.motor_cmd)
            if command.mode == 1] == [15, 28]
    assert (message.motor_cmd[15].tau, message.motor_cmd[15].q,
            message.motor_cmd[15].kp, message.motor_cmd[15].kd) == (
                1.25, 0.4, 40.0, 3.0)
    assert (message.motor_cmd[28].tau, message.motor_cmd[28].q,
            message.motor_cmd[28].kp, message.motor_cmd[28].kd) == (
                -0.75, -0.2, 20.0, 1.5)
    # dq 恒为零，kd 因此是纯阻尼，姿态静止后不贡献力矩。
    assert all(command.dq == 0.0 for command in message.motor_cmd)
    assert all(command.q == command.tau == command.kp == command.kd == 0.0
               for index, command in enumerate(message.motor_cmd)
               if index not in (15, 28))
    assert message.crc == crc32_core(lowcmd_payload(message))