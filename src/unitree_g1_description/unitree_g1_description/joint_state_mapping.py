"""Map Unitree G1 LowState motor indices to URDF joint names."""

import math
from typing import Iterable, List, Protocol, Sequence, Tuple


class MotorState(Protocol):
    q: float
    dq: float
    tau_est: float


G1_JOINT_NAMES: Tuple[str, ...] = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)

GRIPPER_JOINT_SUFFIXES: Tuple[str, ...] = (
    "eccentric_joint",
)


def _finite_values(values: Iterable[float]) -> List[float]:
    result = [float(value) for value in values]
    return result if all(math.isfinite(value) for value in result) else []


def motor_states_to_joint_fields(
    motor_states: Sequence[MotorState]) -> Tuple[List[float], List[float],
                             List[float]]:
    """Return position, velocity and effort arrays for the 29 G1 joints.

    Position is required for TF and an invalid position rejects the sample.
    Velocity or effort vectors are omitted as a whole when their source has a
    non-finite value, which follows the optional-array semantics of JointState.
    """
    count = len(G1_JOINT_NAMES)
    if len(motor_states) < count:
        raise ValueError(
            f"LowState has {len(motor_states)} motors; expected at least {count}")

    selected = motor_states[:count]
    positions = _finite_values(state.q for state in selected)
    if len(positions) != count:
        raise ValueError("LowState contains a non-finite joint position")

    velocities = _finite_values(state.dq for state in selected)
    efforts = _finite_values(state.tau_est for state in selected)
    return positions, velocities, efforts


def gripper_joint_names(prefix: str) -> List[str]:
    """Return the driven Gloria-M joint name for one mounted gripper."""
    return [prefix + suffix for suffix in GRIPPER_JOINT_SUFFIXES]


def gripper_state_to_joint_fields(
        positions: Sequence[float], velocities: Sequence[float],
        efforts: Sequence[float]) -> Tuple[List[float], List[float], List[float]]:
    """Return the eccentric state; URDF mimic joints provide the linkage."""
    if not positions:
        raise ValueError("gripper JointState has no position")
    mapped_positions = _finite_values(positions[:1])
    if not mapped_positions:
        raise ValueError("gripper JointState has a non-finite position")

    mapped_velocities = _finite_values(velocities[:1]) if velocities else []
    mapped_efforts = _finite_values(efforts[:1]) if efforts else []
    return mapped_positions, mapped_velocities, mapped_efforts