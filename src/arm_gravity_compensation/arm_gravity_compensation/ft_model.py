"""KWR57 六维力传感器的零偏、工具自重与末端负载模型。

标定和运行时补偿共用这一份数学，两边都不依赖 ROS，也不依赖 Pinocchio。

坐标与符号约定
--------------
``gravity`` 一律是重力加速度向量（指向地心、模长约 9.81），表达在传感器安装 link
系 L（URDF 的 ``*_kwr57b_link``）里；它由手臂正运动学与躯干 IMU 给出。传感器自己
的测量系 S 与 L 之间可能差一个常值旋转 ``rotation`` = R(S<-L)，默认单位阵，只有
数据显著支持时标定才采纳它。

"工具"指传感器远端的一切：夹爪、相机、线缆。静力模型是

    raw   = bias + polarity * R @ [ mass * u ; (mass * com) x u ]
    net_L = polarity * R^T (raw - bias) - [ mass * u ; (mass * com) x u ]

``polarity`` 只可能是 ±1，用来吸收厂商的受力方向约定，由标定从数据定出。``net_L``
是**负载或环境施加给传感器工具侧的物理力旋量**：静挂 1 kg 时力就是 9.81 N 指向地面。
"""

from dataclasses import dataclass, field
from typing import Dict, Sequence, Tuple

import numpy as np
from numpy.typing import ArrayLike
from scipy.stats import f as f_distribution

from .em import fit_robust_em

GRAVITY_MAGNITUDE = 9.81
KGF_TO_NEWTON = 9.80665

# 自由力模型比受约束模型多 8 个自由度；只有残差下降显著到这个水平以下才认为
# "传感器姿态或轴增益确实偏离名义值"，否则那点下降是姿态误差喂出来的。
DEFAULT_SIGNIFICANCE = 0.01
# 一阶矩需要三个不共面的朝向，完整 3x3 力标定矩阵需要四个。
MINIMUM_SAMPLES = 3
MINIMUM_ORIENTATION_SAMPLES = 4


def _skew(vector: ArrayLike) -> np.ndarray:
    x, y, z = (float(value) for value in vector)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _validated(values: ArrayLike, size: int, name: str) -> np.ndarray:
    array = np.asarray(values, dtype=float).reshape(-1)
    if array.size != size or not np.all(np.isfinite(array)):
        raise ValueError("%s must be %d finite values" % (name, size))
    return array


@dataclass(frozen=True)
class FtSample:
    """一个静态姿态上的传感器样本。"""

    gravity: np.ndarray
    wrench: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "gravity", _validated(self.gravity, 3, "gravity"))
        object.__setattr__(self, "wrench", _validated(self.wrench, 6, "wrench"))
        if np.linalg.norm(self.gravity) < 1e-6:
            raise ValueError("gravity must not be a zero vector")


@dataclass(frozen=True)
class FtCalibration:
    """一台传感器的标定量。

    ``*_bias`` 在测量系 S，``mass``/``com`` 在 link 系 L 且**对 link 原点取矩**。
    ``origin`` 是传感器力矩参考点在 link 系里的位置：厂家把它放在工具侧法兰面上，
    而不是 URDF link 原点，两者差一整个传感器高度。纯重力标定解不出它（模型里
    ``h`` 和 ``origin`` 是同一项，只有 ``c - origin`` 可辨识），所以它是输入不是输出。
    """

    force_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    torque_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    mass: float = 0.0
    com: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rotation: np.ndarray = field(default_factory=lambda: np.eye(3))
    polarity: float = 1.0
    origin: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "force_bias", _validated(self.force_bias, 3, "force_bias"))
        object.__setattr__(
            self, "torque_bias", _validated(self.torque_bias, 3, "torque_bias"))
        object.__setattr__(self, "com", _validated(self.com, 3, "com"))
        object.__setattr__(self, "origin", _validated(self.origin, 3, "origin"))
        rotation = np.asarray(self.rotation, dtype=float).reshape(3, 3)
        if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-6):
            raise ValueError("rotation must be orthonormal")
        object.__setattr__(self, "rotation", rotation)
        if not np.isfinite(self.mass) or float(self.mass) < 0.0:
            raise ValueError("mass must be finite and non-negative")
        object.__setattr__(self, "mass", float(self.mass))
        if float(self.polarity) not in (1.0, -1.0):
            raise ValueError("polarity must be +1 or -1")
        object.__setattr__(self, "polarity", float(self.polarity))

    @property
    def first_moment(self) -> np.ndarray:
        """质量一阶矩 m*c，对 link 原点取矩。"""
        return self.mass * self.com

    @property
    def bias(self) -> np.ndarray:
        return np.concatenate([self.force_bias, self.torque_bias])

    def to_dict(self) -> Dict[str, object]:
        """展平成 ROS 2 参数文件能直接吃的标量与数组。"""
        return {
            "force_bias": [float(value) for value in self.force_bias],
            "torque_bias": [float(value) for value in self.torque_bias],
            "tool_mass": float(self.mass),
            "tool_com": [float(value) for value in self.com],
            "tool_first_moment": [float(value) for value in self.first_moment],
            "rotation": [float(value) for value in self.rotation.ravel()],
            "polarity": float(self.polarity),
            "measurement_origin": [float(value) for value in self.origin],
        }

    @classmethod
    def from_dict(cls, document: Dict[str, object]) -> "FtCalibration":
        return cls(
            force_bias=document["force_bias"],
            torque_bias=document["torque_bias"],
            mass=document["tool_mass"],
            com=document["tool_com"],
            rotation=np.asarray(
                document.get("rotation", np.eye(3).ravel()),
                dtype=float).reshape(3, 3),
            polarity=document.get("polarity", 1.0),
            origin=document.get("measurement_origin", np.zeros(3)),
        )


def tool_wrench(calibration: FtCalibration, gravity: ArrayLike) -> np.ndarray:
    """工具自重在 link 系 L 里的物理力旋量，对 link 原点取矩。"""
    vector = _validated(gravity, 3, "gravity")
    return np.concatenate([
        calibration.mass * vector,
        np.cross(calibration.first_moment, vector),
    ])


def _shift_to_link(wrench: np.ndarray, origin: np.ndarray) -> np.ndarray:
    """把对测量点取的力矩搬到 link 原点。力与取矩点无关。"""
    return np.concatenate([
        wrench[:3], wrench[3:] + np.cross(origin, wrench[:3])])


def _tool_reading(calibration: FtCalibration, gravity: ArrayLike) -> np.ndarray:
    """工具自重折算到原始读数空间（对测量点取矩、测量系 S、厂商符号约定）。"""
    physical = _shift_to_link(
        tool_wrench(calibration, gravity), -calibration.origin).reshape(2, 3)
    return calibration.polarity * (physical @ calibration.rotation.T).reshape(6)


def expected_raw(calibration: FtCalibration, gravity: ArrayLike) -> np.ndarray:
    """空载时传感器应当读到的原始值，标定残差和自检都用它。"""
    return calibration.bias + _tool_reading(calibration, gravity)


def net_wrench(raw: ArrayLike, calibration: FtCalibration,
               gravity: ArrayLike) -> np.ndarray:
    """扣掉零偏与工具自重，得到负载/环境施加的净力旋量。

    输出在 link 系 L 里、**对 link 原点取矩**，这样它的 frame_id 才名副其实，
    下游把负载挂到手臂上时也不必再知道传感器内部的力矩参考点在哪。
    """
    values = _validated(raw, 6, "raw")
    unbiased = (values - calibration.bias).reshape(2, 3) @ calibration.rotation
    measured = _shift_to_link(
        calibration.polarity * unbiased.reshape(6), calibration.origin)
    return measured - tool_wrench(calibration, gravity)


def rezero(raw: ArrayLike, calibration: FtCalibration,
           gravity: ArrayLike) -> FtCalibration:
    """只重估零偏，保持质量、质心与姿态不变。

    大零偏会随温度漂，重做全标定太重；已知空载且姿态已知时其余项全是已知量，
    单个姿态就能把零偏解出来。
    """
    bias = _validated(raw, 6, "raw") - _tool_reading(calibration, gravity)
    return FtCalibration(
        force_bias=bias[:3], torque_bias=bias[3:], mass=calibration.mass,
        com=calibration.com, rotation=calibration.rotation,
        polarity=calibration.polarity, origin=calibration.origin)


def orientation_coverage(gravities: Sequence[ArrayLike]) -> Dict[str, object]:
    """朝向覆盖度：重力单位向量矩阵的奇异值决定各轴能不能分开。

    三个奇异值都明显非零时，零偏才和质量分得开、质心的三个分量才都可辨识。
    """
    if len(gravities) == 0:
        return {"count": 0, "singular_values": [0.0, 0.0, 0.0], "spread": 0.0}
    directions = np.asarray([
        np.asarray(vector, dtype=float) / np.linalg.norm(vector)
        for vector in gravities], dtype=float)
    # 少于三个样本时 svd 只给出样本数个奇异值，缺的那些方向确实是零。
    singular = np.zeros(3, dtype=float)
    values = np.linalg.svd(
        directions, compute_uv=False) / np.sqrt(directions.shape[0])
    singular[:values.size] = values
    return {
        "count": int(directions.shape[0]),
        "singular_values": [float(value) for value in singular],
        "spread": float(singular[2] / singular[0]) if singular[0] > 0.0 else 0.0,
    }


@dataclass(frozen=True)
class FtSolution:
    calibration: FtCalibration
    diagnostics: Dict[str, object]


def _fit(design: np.ndarray, observed: np.ndarray, blocks: np.ndarray,
         robust: bool) -> Tuple[np.ndarray, float, np.ndarray]:
    """最小二乘，样本数足够时按姿态整块剔除离群（有人扶到工具的那种）。"""
    poses = int(blocks.max()) + 1
    if robust and 3 * poses >= 2 * design.shape[1]:
        result = fit_robust_em(design, observed, blocks=blocks)
        parameters, inlier_fraction = result.parameters, result.inlier_fraction
    else:
        parameters = np.linalg.lstsq(design, observed, rcond=None)[0]
        inlier_fraction = 1.0
    return parameters, float(inlier_fraction), observed - design @ parameters


def _polar_decomposition(
        matrix: np.ndarray) -> Tuple[float, np.ndarray, float, np.ndarray]:
    """A = polarity * mass * R 的分解，外加各向异性诊断。

    极分解把常值姿态偏差（安装角、关节零位）和轴增益/正交性损伤分成两半：正交
    因子是前者，奇异值的离散是后者。
    """
    polarity = -1.0 if np.linalg.det(matrix) < 0.0 else 1.0
    left, singular, right = np.linalg.svd(polarity * matrix)
    if np.linalg.det(left @ right) < 0.0:
        left[:, -1] *= -1.0
    rotation = left @ right
    mass = float(np.mean(singular))
    gains = singular / mass if mass > 0.0 else np.zeros(3)
    return polarity, rotation, mass, gains


def _free_force_fit(gravity: np.ndarray, forces: np.ndarray, blocks: np.ndarray,
                    identity: np.ndarray, robust: bool,
                    constrained_residual: np.ndarray, constrained_columns: int,
                    significance: float) -> Dict[str, object]:
    """解完整的 3x3 力标定矩阵，并对受约束模型做 F 检验。"""
    design = np.hstack([
        np.vstack([np.kron(np.eye(3), vector.reshape(1, 3))
                   for vector in gravity]),
        identity,
    ])
    parameters, _, residual = _fit(design, forces, blocks, robust)
    polarity, rotation, mass, gains = _polar_decomposition(
        parameters[:9].reshape(3, 3))
    extra = design.shape[1] - constrained_columns
    degrees = design.shape[0] - design.shape[1]
    noise = float(np.sum(residual ** 2))
    probability = 1.0
    if degrees > 0 and noise > 0.0:
        improvement = max(
            float(np.sum(constrained_residual ** 2)) - noise, 0.0) / extra
        probability = float(f_distribution.sf(
            improvement / (noise / degrees), extra, degrees))
    return {
        "polarity": polarity,
        "rotation": rotation,
        "mass": mass,
        "gains": gains,
        "bias": parameters[9:],
        "residual": residual,
        "probability": probability,
        "significant": bool(probability < significance),
    }


def solve_ft_calibration(
    samples: Sequence[FtSample],
    *,
    estimate_orientation: bool = True,
    robust: bool = True,
    significance: float = DEFAULT_SIGNIFICANCE,
    origin: ArrayLike = (0.0, 0.0, 0.0),
) -> FtSolution:
    """一次性线性最小二乘解出零偏、工具质量与质心。

    力通道解 ``F = A u + b_F``：A 取完整 3x3 时未知量 12 个，每个朝向给 3 个方程，
    所以至少 4 个朝向；极分解 A 同时得到质量、安装姿态和轴增益诊断。力矩通道在 A
    定下来之后对一阶矩线性，需要至少 3 个不共面的朝向。

    纯重力标定**解不出力矩通道的轴增益**：``M = B (h x u)`` 里 ``B·skew(h)`` 的秩
    最多是 2，B 和 h 分不开，所以那一路只出零偏和一阶矩。同理它也解不出 ``origin``：
    一阶矩只在"质心减取矩点"这一个组合里出现，取矩点必须从外面给。
    """
    if len(samples) < MINIMUM_SAMPLES:
        raise ValueError(
            "至少需要 %d 个朝向，收到 %d 个" % (MINIMUM_SAMPLES, len(samples)))
    gravity = np.asarray([sample.gravity for sample in samples], dtype=float)
    wrench = np.asarray([sample.wrench for sample in samples], dtype=float)
    blocks = np.repeat(np.arange(len(samples)), 3)
    count = len(samples)

    forces = wrench[:, :3].reshape(-1)
    torques = wrench[:, 3:].reshape(-1)
    identity = np.tile(np.eye(3), (count, 1))

    # 受约束模型：R 固定为名义安装姿态，只剩一个带符号的比例系数和三个零偏。
    constrained_design = np.hstack([gravity.reshape(-1, 1), identity])
    constrained, _, constrained_residual = _fit(
        constrained_design, forces, blocks, robust)

    free: Dict[str, object] = {}
    if estimate_orientation and count >= MINIMUM_ORIENTATION_SAMPLES:
        free = _free_force_fit(
            gravity, forces, blocks, identity, robust, constrained_residual,
            constrained_design.shape[1], significance)

    if free.get("significant"):
        polarity = free["polarity"]
        rotation = free["rotation"]
        mass = free["mass"]
        force_bias = free["bias"]
        force_residual = free["residual"]
    else:
        polarity = -1.0 if constrained[0] < 0.0 else 1.0
        rotation = np.eye(3)
        mass = abs(float(constrained[0]))
        force_bias = constrained[1:]
        force_residual = constrained_residual

    torque_design = np.hstack([
        np.vstack([-polarity * rotation @ _skew(vector) for vector in gravity]),
        identity,
    ])
    torque_parameters, inlier_fraction, torque_residual = _fit(
        torque_design, torques, blocks, robust)
    first_moment = torque_parameters[:3]
    reference = _validated(origin, 3, "origin")

    calibration = FtCalibration(
        force_bias=force_bias,
        torque_bias=torque_parameters[3:],
        mass=mass,
        # 辨识出的一阶矩是对取矩点的，质心要搬到 link 原点才能对外用。
        com=first_moment / mass + reference if mass > 1e-9 else reference,
        rotation=rotation,
        polarity=polarity,
        origin=reference,
    )
    diagnostics = {
        "sample_count": count,
        "coverage": orientation_coverage(gravity),
        "force_residual_rms": float(np.sqrt(np.mean(force_residual ** 2))),
        "torque_residual_rms": float(np.sqrt(np.mean(torque_residual ** 2))),
        "force_condition": float(np.linalg.cond(constrained_design)),
        "torque_condition": float(np.linalg.cond(torque_design)),
        "inlier_fraction": inlier_fraction,
        "first_moment": [float(value) for value in first_moment],
        "measurement_origin": [float(value) for value in reference],
        "orientation_estimated": bool(free.get("significant", False)),
        "orientation_probability": float(free.get("probability", 1.0)),
    }
    if free:
        diagnostics.update({
            "free_force_residual_rms": float(
                np.sqrt(np.mean(free["residual"] ** 2))),
            "free_mass": float(free["mass"]),
            "principal_gains": [float(value) for value in free["gains"]],
            "shape_error": float(np.max(np.abs(free["gains"] - 1.0))),
            "misalignment_deg": float(np.degrees(np.arccos(np.clip(
                (np.trace(free["rotation"]) - 1.0) / 2.0, -1.0, 1.0)))),
        })
    return FtSolution(calibration=calibration, diagnostics=diagnostics)


@dataclass(frozen=True)
class PayloadEstimate:
    mass: float
    com: np.ndarray
    first_moment: np.ndarray
    observability: float
    sample_count: int


class PayloadEstimator:
    """从净力旋量递推辨识负载的质量与质心。

    单个姿态只能定出质量：力矩方程 ``M = h x u`` 对 h 的秩恒为 2，质心必须靠多个
    不平行的朝向。用带遗忘因子的正规方程累积，既能一次性批量解，也能挂在运行时里
    持续更新——以后做动态末端标定，只要把加速度项加进 ``add`` 的设计块。
    """

    def __init__(self, *, forgetting: float = 1.0, force_sigma: float = 1.0,
                 torque_sigma: float = 0.05,
                 prior_precision: float = 1e-6) -> None:
        if not 0.0 < forgetting <= 1.0:
            raise ValueError("forgetting must be in (0, 1]")
        if force_sigma <= 0.0 or torque_sigma <= 0.0:
            raise ValueError("noise scales must be positive")
        self._forgetting = float(forgetting)
        # 力是 N、力矩是 N·m，不归一化的话最小二乘等于只看力那三行。
        self._scale = np.concatenate([
            np.full(3, 1.0 / float(force_sigma)),
            np.full(3, 1.0 / float(torque_sigma))])
        self._prior = float(prior_precision)
        self.reset()

    def reset(self) -> None:
        self._information = np.zeros((4, 4))
        self._projection = np.zeros(4)
        self._count = 0

    @property
    def sample_count(self) -> int:
        return self._count

    def add(self, gravity: ArrayLike, wrench: ArrayLike,
            weight: float = 1.0) -> None:
        """累积一个净力旋量样本。``wrench`` 必须已经扣掉零偏与工具自重。"""
        vector = _validated(gravity, 3, "gravity")
        measured = _validated(wrench, 6, "wrench")
        if weight <= 0.0:
            raise ValueError("weight must be positive")
        design = np.zeros((6, 4))
        design[:3, 0] = vector
        design[3:, 1:] = -_skew(vector)
        design *= self._scale[:, None]
        self._information *= self._forgetting
        self._projection *= self._forgetting
        self._information += weight * design.T @ design
        self._projection += weight * design.T @ (measured * self._scale)
        self._count += 1

    def estimate(self) -> PayloadEstimate:
        parameters = np.linalg.solve(
            self._information + self._prior * np.eye(4), self._projection)
        mass = float(parameters[0])
        first_moment = parameters[1:]
        # 质心方向只由力矩块决定，用它的条件数判断朝向够不够。
        eigenvalues = np.linalg.eigvalsh(self._information[1:, 1:])
        return PayloadEstimate(
            mass=mass,
            com=first_moment / mass if abs(mass) > 1e-6 else np.zeros(3),
            first_moment=first_moment,
            observability=float(np.clip(
                eigenvalues[0] / max(eigenvalues[-1], 1e-12), 0.0, 1.0)),
            sample_count=self._count,
        )


def instantaneous_mass(wrench: ArrayLike, gravity: ArrayLike) -> float:
    """单帧质量估计：把净力投影到重力方向。质心估不出来，需要多个朝向。"""
    vector = _validated(gravity, 3, "gravity")
    force = _validated(wrench, 6, "wrench")[:3]
    return float(force @ vector / (vector @ vector))


def suggest_measurement_origin(calibration: FtCalibration,
                               modelled_com: ArrayLike) -> np.ndarray:
    """由 CAD 质心反推力矩参考点。

    重力标定只给出"质心减取矩点"，取矩点必须另有来源。CAD 知道工具质心在哪，而质量
    偏差只改大小不改方向，于是两者之差就是取矩点。给出来让操作者核对量级是不是正好
    等于一个传感器高度——对得上才说明这个假设成立。
    """
    reference = _validated(modelled_com, 3, "modelled_com")
    if calibration.mass <= 1e-9:
        return np.zeros(3)
    return reference - (calibration.com - calibration.origin)


def gravity_aligned(wrench: ArrayLike, gravity: ArrayLike, *,
                    tolerance: float = 0.25) -> bool:
    """净力是不是一个纯重力负载。

    接触力方向任意，静挂的负载必须与重力平行。这是把"一直拎着的东西"和"顶到桌子
    上"分开的判据，没有它，负载估计会把接触力当成质量灌进重力补偿。
    """
    vector = _validated(gravity, 3, "gravity")
    force = _validated(wrench, 6, "wrench")[:3]
    mass = instantaneous_mass(wrench, vector)
    residual = force - mass * vector
    return bool(np.linalg.norm(residual) <=
                tolerance * abs(mass) * np.linalg.norm(vector))
