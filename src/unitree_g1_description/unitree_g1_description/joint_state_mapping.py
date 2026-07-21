"""Map Unitree G1 LowState motor indices to URDF joint names."""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable, List, Protocol, Sequence, Tuple
from xml.etree import ElementTree


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

@dataclass(frozen=True)
class MimicJointSpec:
    name: str
    multiplier: float
    offset: float
    lower: float
    upper: float


@dataclass(frozen=True)
class GripperModelSpec:
    source_name: str
    lower: float
    upper: float
    mimic_joints: Tuple[MimicJointSpec, ...]

    @property
    def joint_names(self) -> Tuple[str, ...]:
        return (
            self.source_name,
            *(joint.name for joint in self.mimic_joints),
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


def _joint_limits(joint: ElementTree.Element) -> Tuple[float, float]:
    limit = joint.find("limit")
    if limit is None or limit.get("lower") is None or limit.get("upper") is None:
        raise ValueError(f"joint {joint.get('name')!r} has no finite limits")
    try:
        lower = float(limit.get("lower", ""))
        upper = float(limit.get("upper", ""))
    except ValueError as exc:
        raise ValueError(
            f"joint {joint.get('name')!r} has invalid limits") from exc
    if not math.isfinite(lower) or not math.isfinite(upper) or lower > upper:
        raise ValueError(f"joint {joint.get('name')!r} has invalid limits")
    return lower, upper


def load_gripper_model_spec(
        urdf_path: Path, prefix: str) -> GripperModelSpec:
    """Load one gripper's driven and direct mimic joints from the model."""
    root = ElementTree.parse(str(urdf_path)).getroot()
    source_name = f"{prefix}eccentric_joint"
    joints = root.findall("joint")
    source_joint = next(
        (joint for joint in joints if joint.get("name") == source_name), None)
    if source_joint is None:
        raise ValueError(f"URDF has no joint {source_name!r}")
    lower, upper = _joint_limits(source_joint)

    mimic_joints = []
    for joint in joints:
        mimic = joint.find("mimic")
        if mimic is None or mimic.get("joint") != source_name:
            continue
        name = joint.get("name")
        if not name:
            raise ValueError(f"mimic of {source_name!r} has no name")
        mimic_lower, mimic_upper = _joint_limits(joint)
        try:
            multiplier = float(mimic.get("multiplier", "1"))
            offset = float(mimic.get("offset", "0"))
        except ValueError as exc:
            raise ValueError(f"joint {name!r} has invalid mimic values") from exc
        if not math.isfinite(multiplier) or not math.isfinite(offset):
            raise ValueError(f"joint {name!r} has invalid mimic values")
        mimic_joints.append(MimicJointSpec(
            name=name,
            multiplier=multiplier,
            offset=offset,
            lower=mimic_lower,
            upper=mimic_upper,
        ))
    if not mimic_joints:
        raise ValueError(f"URDF has no direct mimics of {source_name!r}")
    return GripperModelSpec(
        source_name=source_name,
        lower=lower,
        upper=upper,
        mimic_joints=tuple(mimic_joints),
    )


def gripper_state_to_model_fields(
        positions: Sequence[float], velocities: Sequence[float],
        efforts: Sequence[float], model: GripperModelSpec
        ) -> Tuple[List[float], List[float], List[float]]:
    """Return the driven state plus explicit limit-clamped mimic states."""
    if not positions:
        raise ValueError("gripper JointState has no position")
    source_positions = _finite_values(positions[:1])
    if not source_positions:
        raise ValueError("gripper JointState has a non-finite position")
    source_velocities = _finite_values(velocities[:1]) if velocities else []
    source_efforts = _finite_values(efforts[:1]) if efforts else []

    source_position = min(max(
        source_positions[0], model.lower), model.upper)
    model_positions = [source_position]
    for joint in model.mimic_joints:
        value = joint.multiplier * source_position + joint.offset
        model_positions.append(min(max(value, joint.lower), joint.upper))

    model_velocities: List[float] = []
    if source_velocities:
        source_velocity = source_velocities[0]
        model_velocities.append(source_velocity)
        for joint in model.mimic_joints:
            unconstrained = joint.multiplier * source_position + joint.offset
            if joint.lower < unconstrained < joint.upper:
                model_velocities.append(joint.multiplier * source_velocity)
            else:
                model_velocities.append(0.0)

    model_efforts: List[float] = []
    if source_efforts:
        model_efforts = [source_efforts[0]] + [0.0] * len(model.mimic_joints)
    return model_positions, model_velocities, model_efforts