"""Trajectory and gravity feed-forward for the motor-side position loop."""

from collections import deque
from dataclasses import dataclass
import numpy as np
from numpy.typing import ArrayLike


@dataclass(frozen=True)
class TorqueStep:
    feedforward: np.ndarray
    reference: np.ndarray
    applied: np.ndarray
    target_error: np.ndarray
    trajectory_complete: bool


class PoseStabilityWindow:
    """Detect a stationary measured pose from position variation only."""

    def __init__(
        self,
        *,
        duration: float = 0.6,
        position_range_tolerance: float = 0.02,
    ) -> None:
        if duration <= 0.0:
            raise ValueError("duration must be positive")
        if position_range_tolerance < 0.0:
            raise ValueError("position stability tolerance must be non-negative")
        self.duration = float(duration)
        self.position_range_tolerance = float(position_range_tolerance)
        self._samples = deque()
        self.span = 0.0
        self.max_velocity = float("inf")
        self.max_position_range = float("inf")

    def update(
        self,
        timestamp: float,
        position: ArrayLike,
        velocity: ArrayLike,
    ) -> bool:
        position_array = TorquePoseController._seven(position, "position")
        velocity_array = TorquePoseController._seven(velocity, "velocity")
        now = float(timestamp)
        self._samples.append((now, position_array, velocity_array))
        cutoff = now - self.duration
        while (len(self._samples) > 1 and self._samples[1][0] <= cutoff):
            self._samples.popleft()

        self.span = now - self._samples[0][0]
        positions = np.asarray([sample[1] for sample in self._samples])
        velocities = np.asarray([sample[2] for sample in self._samples])
        self.max_velocity = float(np.max(np.abs(velocities)))
        self.max_position_range = float(np.max(np.ptp(positions, axis=0)))
        return bool(
            self.span >= self.duration and
            self.max_position_range <= self.position_range_tolerance)


class TorquePoseController:
    """Reference trajectory plus gravity feed-forward for one seven-axis arm.

    The position loop itself runs inside the motors, which close it at their
    own rate without the latency and slew limiting of this node. ``stiffness``
    and ``damping`` are therefore the gains published as ``kp`` and ``kd``, and
    this class only uses them to report the torque the motor will produce,
    which is what the static identification regresses on.
    """

    def __init__(
        self,
        *,
        stiffness: ArrayLike = (40.0, 40.0, 40.0, 40.0, 40.0, 20.0, 20.0),
        damping: ArrayLike = (3.0, 3.0, 3.0, 3.0, 3.0, 1.5, 1.5),
        torque_slew_rate: ArrayLike = (30.0,) * 7,
        maximum_speed: float = 0.35,
        minimum_duration: float = 2.0,
    ) -> None:
        self.stiffness = self._seven(stiffness, "stiffness")
        self.damping = self._seven(damping, "damping")
        self.torque_slew_rate = self._seven(
            torque_slew_rate, "torque_slew_rate")
        if np.any(self.stiffness < 0.0) or np.any(self.damping < 0.0):
            raise ValueError("stiffness and damping must be non-negative")
        if np.any(self.torque_slew_rate <= 0.0):
            raise ValueError("torque slew rates must be positive")
        self.maximum_speed = float(maximum_speed)
        if self.maximum_speed <= 0.0:
            raise ValueError("maximum speed must be positive")
        self.minimum_duration = float(minimum_duration)
        self._active = False

    def start(self, timestamp: float, position: ArrayLike,
              target: ArrayLike,
              initial_torque: ArrayLike = (0.0,) * 7) -> float:
        self._start = self._seven(position, "position")
        self._target = self._seven(target, "target")
        self._start_time = float(timestamp)
        excursion = float(np.max(np.abs(self._target - self._start)))
        self._duration = max(
            self.minimum_duration, excursion / self.maximum_speed)
        self._last_time = self._start_time
        self._last_feedforward = self._seven(initial_torque, "initial_torque")
        self._active = True
        return self._duration

    def step(
        self,
        timestamp: float,
        position: ArrayLike,
        velocity: ArrayLike,
        gravity_torque: ArrayLike,
    ) -> TorqueStep:
        """算出这一拍要下发给电机的设定点，并预测电机会产生多大力矩。

        参数
        ----
        timestamp     本拍的单调时钟（秒），只用来算轨迹进度和限速的时间增量。
        position      LowState 实测的 7 个关节角 q（rad），本侧手臂。
        velocity      LowState 实测的 7 个关节角速度 dq（rad/s）。
        gravity_torque
                      Pinocchio 用当前已标定参数算出的重力力矩 G(q)（N·m），
                      即"要让手臂在当前姿态静止不动所需的力矩"。

        本函数不做位置闭环，位置环由电机内部以自身高频率执行：
            电机实际输出 = tau + kp * (q_des - q_实测) - kd * dq_实测
        我们只负责给出 tau（重力前馈）和 q_des（参考轨迹）。
        """
        if not self._active:
            raise RuntimeError("torque pose controller has not been started")
        # 统一校验成 (7,) 的 float 数组，顺序与 ARM_JOINTS[side] 一致。
        position_array = self._seven(position, "position")
        velocity_array = self._seven(velocity, "velocity")
        gravity_array = self._seven(gravity_torque, "gravity_torque")

        # ---- 1. 参考轨迹：从起点平滑走到目标点 ----
        # elapsed 是从 start() 起经过的时间；ratio 是归一化进度 0→1。
        elapsed = max(0.0, float(timestamp) - self._start_time)
        ratio = min(1.0, elapsed / self._duration)
        # smoothstep：3r²-2r³，在两端一阶导为 0，所以起步和到位都没有速度突跳，
        # 避免电机的 kp 项在轨迹拐点处产生冲击。
        smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio)
        # reference 就是要写进 LowCmd 的 q_des。start() 时把 _start 设成当时的
        # 实测位置，所以第一拍 q_des == q_实测，kp 项为 0，接管瞬间不会甩手臂。
        reference = self._start + smooth_ratio * (self._target - self._start)

        # ---- 2. 重力前馈：限制变化率后再下发 ----
        # delta_time 是距上一拍的真实间隔；乘以 torque_slew_rate 得到本拍允许
        # 的最大力矩变化量。刚接管时 _last_feedforward 从上一段轨迹的末值起步，
        # 于是前馈力矩是斜坡上升而不是阶跃，机械上更柔和。
        # 注意：这里只限制前馈，kp/kd 的反馈项由电机自己算，完全不受限速影响
        # ——之前把阻尼项一起限速会让它变成相位滞后，反而激励关节振荡。
        delta_time = max(0.0, float(timestamp) - self._last_time)
        maximum_delta = self.torque_slew_rate * delta_time
        self._last_feedforward = self._last_feedforward + np.clip(
            gravity_array - self._last_feedforward,
            -maximum_delta, maximum_delta)
        self._last_time = float(timestamp)

        # ---- 3. 预测电机的实际输出力矩 ----
        # 复现电机内部的算式。这一项是标定回归的观测量 τ_cmd：静止时它必须等于
        # 真实重力力矩，所以 fit_selected_joints 就是拿它去解 τ = G(q)·s + b。
        # 三项全部已知或可测，因此即使手臂没停在目标点上，观测量依然是精确的。
        # 随着迭代收敛，(reference - position) → 0，观测量趋近于纯前馈，
        # 也就越来越不依赖 kp 的标称精度。
        applied = (
            self._last_feedforward +
            self.stiffness * (reference - position_array) -
            self.damping * velocity_array
        )

        # ---- 4. 状态汇报 ----
        # target_error 是"最终目标"与实测位置之差，只写进记录供诊断，
        # 不参与控制，也不作为静止判据（判据在 PoseStabilityWindow 里看位置量程）。
        # trajectory_complete：参考轨迹本身走完了（不代表手臂到位了）。
        return TorqueStep(
            feedforward=self._last_feedforward.copy(),  # 写入 LowCmd 的 tau
            reference=reference,                        # 写入 LowCmd 的 q
            applied=applied,                            # 回归用的观测力矩
            target_error=self._target - position_array,
            trajectory_complete=ratio >= 1.0,
        )

    @staticmethod
    def _seven(value: ArrayLike, name: str) -> np.ndarray:
        array = np.asarray(value, dtype=float)
        if array.shape != (7,) or not np.all(np.isfinite(array)):
            raise ValueError("%s must contain seven finite values" % name)
        return array.copy()