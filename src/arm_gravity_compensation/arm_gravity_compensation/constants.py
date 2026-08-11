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

# 六维力传感器固连在腕 yaw 之后，它远端的一切（夹爪、相机、线缆）都由它称量。
# 这个 link 系就是 ft_model 里的 L 系：净力旋量、工具与负载质心都表达在它里面。
FT_SENSOR_LINKS = {
    "left": "left_kwr57b_link",
    "right": "right_kwr57b_link",
}

# final.urdf 的左右臂关于矢状面严格镜像：每个关节 origin 满足
# ``left_xyz = diag(1, -1, 1) @ right_xyz``，轴向完全相同。镜像是保向性相反的
# 变换，绕 ``a`` 转 theta 映射为绕 ``diag(1,-1,1) @ a`` 转 -theta，于是
#     q_other = MIRROR_SIGNS * q_source
# pitch / elbow / wrist_pitch 保号，roll / yaw 取反。关节限位本身也是镜像的，
# 所以源侧合法的姿态镜像后必定落在对侧限位内。重力力矩按同一符号向量映射。
MIRROR_SIGNS = (1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0)


def opposite_side(side: str) -> str:
    if side not in SIDES:
        raise ValueError("side must be 'left' or 'right'")
    return "right" if side == "left" else "left"


def mirror_arm_values(values) -> tuple:
    """Map seven per-joint values between the two arms."""
    sequence = tuple(float(value) for value in values)
    if len(sequence) != 7:
        raise ValueError("arm values must contain seven entries")
    return tuple(sign * value for sign, value in zip(MIRROR_SIGNS, sequence))