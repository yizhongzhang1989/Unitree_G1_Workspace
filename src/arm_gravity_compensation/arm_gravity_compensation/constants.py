"""Unitree G1 arm names and LowState motor indices."""

ARM_JOINTS = {
    "left": (
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
    ),
    "right": (
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ),
}

ARM_MOTOR_INDICES = {
    "left": tuple(range(15, 22)),
    "right": tuple(range(22, 29)),
}

ALL_ARM_JOINTS = ARM_JOINTS["left"] + ARM_JOINTS["right"]
ALL_ARM_MOTOR_INDICES = ARM_MOTOR_INDICES["left"] + ARM_MOTOR_INDICES["right"]
SIDES = ("left", "right")