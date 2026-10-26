"""可选的进程内 IK，复用宿主节点的参数、订阅与锁

宿主提供 joints31、dt、declare、control_group、lock 和 output.positions31
准备结束后 reset_ik，求解结果合入最终目标，成功发布后再 commit_ik
"""

import math
import time
from typing import TYPE_CHECKING, Callable, TypedDict, TypeVar, overload

import numpy as np
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .arm_ik import ArmIK, _pose_errors

if TYPE_CHECKING:
    from threading import RLock
    from rclpy.callback_groups import CallbackGroup, MutuallyExclusiveCallbackGroup
    from rclpy.impl.rcutils_logger import RcutilsLogger
    from rclpy.parameter import Parameter
    from rclpy.subscription import Subscription
    from .joint_output import JointOutput


class IKConfig(TypedDict):
    base_frame: str
    tip_frames: dict[str, str]
    max_iters: int
    damping: float
    tol_pos: float
    tol_ori: float
    max_step_pos: float
    max_step_ori: float
    rotation_weight: float
    joint_limits: dict[str, tuple[float, float]]
    null_gain: dict[str, float]
    null_target: dict[str, float]
    null_gate: tuple[float, ...]
    reference: list[float]
    rate_limit: float
    rescue_err: float
    rescue_giveup: int


class _IKStatusFields(TypedDict):
    upper_ready: bool
    upper_executor: str
    arm_mode: str
    ik_ready: bool
    base_frame: str
    tip_frames: dict[str, str]
    ik_rotation_weight: float
    ik_pos_err: float
    ik_ori_err: float
    ik_iterations: int
    ik_ms: float


class IKStatus(_IKStatusFields):
    limited_pose: dict[str, np.ndarray]


class IKPublicStatus(_IKStatusFields):
    limited_pose: dict[str, list[float]]


MessageT = TypeVar('MessageT')


class ArmIKMixin:
    if TYPE_CHECKING:
        joints31: list[str]
        dt: float
        lock: RLock
        control_group: MutuallyExclusiveCallbackGroup
        output: JointOutput

        @overload
        def declare(self, name: str, default: int) -> int: ...

        @overload
        def declare(self, name: str, default: float) -> float: ...

        @overload
        def declare(self, name: str, default: str) -> str: ...

        @overload
        def declare(self, name: str, default: list[str]) -> list[str]: ...

        @overload
        def declare(self, name: str, default: list[float]) -> list[float]: ...

        def declare(self, name: str, default: int | float | str | list[str] | list[float]) -> int | float | str | list[str] | list[float]: ...

        def get_parameter(self, name: str) -> Parameter: ...

        def get_logger(self) -> RcutilsLogger: ...

        def create_subscription(
            self, msg_type: type[MessageT], topic: str,
            callback: Callable[[MessageT], None], qos_profile: QoSProfile | int,
            *, callback_group: CallbackGroup | None = None,
        ) -> Subscription: ...

        def add_on_set_parameters_callback(self, callback: Callable[[list[Parameter]], SetParametersResult]) -> None: ...

    def init_arm_ik(self):
        """Declare IK-only configuration and bind the first valid description once."""
        if hasattr(self, '_ik'):
            return
        self.arm_joints = list(self.declare('arm_joints', list(self.joints31[15:29])))
        if len(self.arm_joints) != 14 or len(set(self.arm_joints)) != 14:
            raise ValueError('arm_joints must contain 14 unique joints')
        self.arm_slots = np.array([self.joints31.index(name) for name in self.arm_joints], dtype=int)
        self.base_frame = self.declare('base_frame', 'torso_link')
        self.tip_frames = {
            'left': self.declare('left_tip_frame', 'left_gripper_base'),
            'right': self.declare('right_tip_frame', 'right_gripper_base'),
        }
        maximum_iterations = self.declare('ik_max_iters', 10)
        damping = self.declare('ik_damping', 0.05)
        tolerance_position = self.declare('ik_tol_pos', 0.001)
        tolerance_orientation = self.declare('ik_tol_ori', 0.0035)
        maximum_position = self.declare('ik_max_step_pos', 0.1)
        maximum_orientation = self.declare('ik_max_step_ori', 0.5)
        weight = self.declare('ik_rotation_weight', 0.01)
        limits = self.declare('ik_limit_upper', [math.inf] * 14)
        gain = self.declare('ik_null_gain', [0.0] * 14)
        reference = self.declare('ik_null_target', [0.0] * 14)
        gate = self.declare('ik_null_gate', [0.6, 1.2])
        rescue_error = self.declare('ik_rescue_err', 0.01)
        rescue_giveup = self.declare('ik_rescue_giveup', 3)
        rate = self.declare('arm_rate_limit', 10.0)
        for name, values in (('ik_limit_upper', limits), ('ik_null_gain', gain),
                             ('ik_null_target', reference)):
            if len(values) not in (0, 14):
                raise ValueError(f'{name} must be empty or contain 14 values')
        passive = list(self.get_parameter('passive_targets').get_parameter_value().double_array_value)
        if len(passive) != 16:
            raise ValueError('passive_targets must contain 14 arms and 2 grippers')
        passive_by_name = dict(zip(self.joints31[15:], passive))
        self._ik_kwargs = IKConfig(
            base_frame=self.base_frame, tip_frames=self.tip_frames,
            max_iters=maximum_iterations, damping=damping,
            tol_pos=tolerance_position, tol_ori=tolerance_orientation,
            max_step_pos=maximum_position, max_step_ori=maximum_orientation,
            rotation_weight=weight,
            joint_limits={name: (-math.inf, value) for name, value in zip(self.arm_joints, limits)},
            null_gain={name: value for name, value in zip(self.arm_joints, gain) if value},
            null_target={name: value for name, value in zip(self.arm_joints, reference) if value},
            null_gate=tuple(gate), reference=[passive_by_name[name] for name in self.arm_joints],
            rate_limit=rate, rescue_err=rescue_error, rescue_giveup=rescue_giveup)
        self._ik: ArmIK | None = None
        self._ik_seed = None
        self.arm_indices = np.array([], dtype=int)
        self._arm_from_model = np.array([], dtype=int)
        self._ik_targets = {}
        self._ik_status = IKStatus(
            upper_ready=False, upper_executor='ik', arm_mode='ik', ik_ready=False,
            base_frame=self.base_frame, tip_frames=dict(self.tip_frames),
            ik_rotation_weight=weight, ik_pos_err=0.0, ik_ori_err=0.0,
            ik_iterations=0, ik_ms=0.0, limited_pose={})
        topic = self.get_parameter('robot_description_topic').value
        if not isinstance(topic, str):
            raise ValueError('robot_description_topic must be a string')
        self._ik_description_subscription = self.create_subscription(
            String, topic, self._bind_arm_ik,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self.control_group)
        self.add_on_set_parameters_callback(self._set_ik_parameters)

    @property
    def ik_ready(self):
        """Model bound; seed readiness is established separately by reset_ik."""
        return self._ik is not None

    def _bind_arm_ik(self, message: String):
        with self.lock:
            if self.ik_ready:
                return
            try:
                model = ArmIK(message.data, self.arm_joints, **self._ik_kwargs)
                self.arm_indices = np.array([self.joints31.index(name) for name in model.joint_names], dtype=int)
                self._arm_from_model = np.array([model.joint_names.index(name) for name in self.arm_joints], dtype=int)
                self._ik = model
                self._commit_ik_locked(self.output.positions31)
            except Exception as error:
                self._ik = None
                self.get_logger().error(f'IK model binding failed: {error}')

    def _set_ik_parameters(self, parameters: list['Parameter']):
        # 先校验整批参数再提交，后续无效项不能留下部分更新
        for parameter in parameters:
            if parameter.name == 'ik_rotation_weight':
                value = parameter.value
                if not isinstance(value, (float, int)) or not math.isfinite(value) or not 0.0 <= value <= 1.0:
                    return SetParametersResult(successful=False, reason='IK weight must be in [0,1]')
        with self.lock:
            for parameter in parameters:
                if parameter.name == 'ik_rotation_weight':
                    value = parameter.value
                    if isinstance(value, (float, int)):
                        self._ik_kwargs['rotation_weight'] = float(value)
                        if self._ik is not None:
                            self._ik.set_rotation_weight(value)
                        self._ik_status['ik_rotation_weight'] = float(value)
        return SetParametersResult(successful=True)

    def solve_ik(self, left=None, right=None, *, targets=None):
        """Return (14-arm target in arm_joints order, post-limit pos/ori errors, iters).

        Accept solve_ik(left, right), solve_ik(dict), or solve_ik(targets=dict).
        Omitted sides hold the committed seed. This never commits a solved seed.
        """
        if isinstance(left, dict):
            if right is not None or targets is not None:
                raise ValueError('provide either a target dict or left/right targets')
            targets = left
        elif targets is None:
            targets = {}
            if left is not None:
                targets['left'] = left
            if right is not None:
                targets['right'] = right
        elif left is not None or right is not None:
            raise ValueError('provide either a target dict or left/right targets')
        poses = {}
        for side, value in targets.items():
            if side not in self.tip_frames:
                raise ValueError(f'unknown IK side: {side}')
            pose = np.array(value, dtype=np.float64)
            if pose.shape != (7,) or not np.isfinite(pose).all():
                raise ValueError('IK pose must contain seven finite values')
            if not math.isclose(float(np.linalg.norm(pose[3:])), 1.0, abs_tol=1e-5):
                raise ValueError('IK quaternion must be unit xyzw')
            poses[side] = pose
        with self.lock:
            if self._ik is None or self._ik_seed is None:
                raise RuntimeError('IK model and committed seed are required')
            started = time.monotonic()
            solved, position, orientation, iterations = self._ik.step(self._ik_seed, poses, self.dt)
            self._ik_targets = poses
            self._ik_status.update(
                ik_pos_err=round(position, 4), ik_ori_err=round(orientation, 4),
                ik_iterations=iterations, ik_ms=round((time.monotonic() - started) * 1e3, 2))
            return solved[self._arm_from_model], position, orientation, iterations

    def commit_ik(self, positions31):
        """提交实际发布的最终 31 轴目标，再刷新对应 FK"""
        with self.lock:
            self._commit_ik_locked(positions31)

    def _commit_ik_locked(self, positions31):
        if self._ik is None:
            raise RuntimeError('IK model is not bound')
        positions = np.asarray(positions31, dtype=np.float64)
        if positions.shape != (31,) or not np.isfinite(positions).all():
            raise ValueError('positions31 must contain 31 finite values')
        seed = positions[self.arm_indices].copy()
        reached = self._ik.fk(seed, clamp=False)
        position, orientation = _pose_errors(reached, self._ik_targets)
        self._ik_seed = seed
        self._ik_status.update(
            upper_ready=True, ik_ready=True, limited_pose=reached,
            ik_rotation_weight=self._ik.rotation_weight,
            ik_pos_err=round(position, 4), ik_ori_err=round(orientation, 4))

    def reset_ik(self, positions31):
        """Reseed from actual preparation-end/measured joints and re-arm rescue."""
        with self.lock:
            if self._ik is None:
                raise RuntimeError('IK model is not bound')
            self._ik_targets = {}
            self._ik._rescue_miss = 0
            self._commit_ik_locked(positions31)
            self._ik_status.update(ik_iterations=0, ik_ms=0.0)

    def status_ik(self) -> IKPublicStatus:
        """Original upper IK fields; limited_pose is committed command FK, not feedback."""
        with self.lock:
            result: IKPublicStatus = {
                **self._ik_status,
                'upper_ready': self.ik_ready,
                'ik_ready': self.ik_ready,
                'tip_frames': dict(self.tip_frames),
                'limited_pose': {side: pose.tolist() for side, pose in self._ik_status['limited_pose'].items()},
            }
            return result
