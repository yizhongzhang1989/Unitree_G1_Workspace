"""VLA 接入层：把「一个 VLA 是什么样」声明成数据，把「怎么请求」隔离进 backend。

接一个新的 VLA = 在 ``backends/`` 下加一个模块，填一份 :class:`VlaSpec` 并实现
:meth:`VlaBackend.infer`，``vla_node`` 一行都不用改。节点只认这里定义的三样东西：

* :class:`Observation` —— 机器人侧的观测，**全部表示在 ``base_frame``（默认
  ``torso_link``）里**，单位米/弧度。
* :class:`ActionChunk` —— 动作序列，**同样表示在 ``base_frame`` 里**。
* :class:`VlaSpec` —— 这个 VLA 的规格：坐标系原点落在哪、要几张什么图、夹爪怎么换算。

模型系 <-> 机器人系的换算**不由节点做**。每个 VLA 的坐标约定都不同，backend 拿
``spec.frame`` 建一个 ``FrameTransform`` 自己换，节点因此永远只跟 ``base_frame`` 打交道，
换 VLA 不会把坐标系的坑扩散到执行侧。
"""

from __future__ import annotations

import abc
import importlib
import os
from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

import numpy as np

from g1_vla_bridge.transforms import FrameTransform, rpy_to_mat

SIDES = ('left', 'right')
#: 图像槽位的规范名。``VlaSpec.images.slots`` 从这里选，顺序即协议顺序。
IMAGE_SLOTS = ('head', 'left_wrist', 'right_wrist')


# -- 规格 -------------------------------------------------------------------

@dataclass(frozen=True)
class FrameSpec:
    """VLA 的动作/状态所在的坐标系，声明成「相对我方 ``base_frame`` 的固定变换」。

    ``origin_in_base``
        **VLA 坐标系的原点落在 ``base_frame`` 的哪里**，米。这是接一个新 VLA 时第一个
        要问清楚的数，具体值写在各 backend 的 ``SPEC`` 里。
    ``rotation_rpy``
        VLA 坐标系的朝向相对 ``base_frame`` 的 rpy。两边都是「水平朝前的底盘系」时为 0。
        **delta 模式下这一项不会抵消**（Δp_model = R·Δp_base），必须标准。
    ``tool_offset`` / ``tool_rotation_rpy``
        我方 tip frame -> VLA 末端 frame 的固定变换。delta 模式下这两项会抵消。

    会变的量（身体高度、俯仰）**不属于这里**——它们走相机外参，训练时也是这么编码的。
    """

    origin_in_base: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    tool_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    tool_rotation_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def transform(self) -> FrameTransform:
        """建一个可直接换算位姿的 ``FrameTransform``。"""
        origin = np.asarray(self.origin_in_base, dtype=np.float64).reshape(3)
        # FrameTransform 内部用的是「base 原点在模型系里的位置」，与本类的声明互为反向。
        base_offset = -rpy_to_mat(self.rotation_rpy) @ origin
        return FrameTransform(base_offset, self.tool_rotation_rpy,
                              self.tool_offset, self.rotation_rpy)

    @classmethod
    def from_solution(cls, base_offset, rotation_rpy, **rest) -> 'FrameSpec':
        """把 ``transforms.solve_base_frame()`` 的解转成本类的声明式字段。"""
        origin = -rpy_to_mat(rotation_rpy).T @ np.asarray(
            base_offset, dtype=np.float64).reshape(3)
        return cls(origin_in_base=tuple(origin),
                   rotation_rpy=tuple(float(v) for v in rotation_rpy), **rest)


@dataclass(frozen=True)
class ImageSpec:
    """VLA 要的图。``slots`` 的**顺序就是协议顺序**，错了模型不报错，只会变傻。"""

    slots: tuple[str, ...] = IMAGE_SLOTS
    height: int = 240
    jpeg_quality: int = 90

    def __post_init__(self) -> None:
        unknown = [s for s in self.slots if s not in IMAGE_SLOTS]
        if unknown:
            raise ValueError(f'未知的图像槽位 {unknown}，可选 {list(IMAGE_SLOTS)}')


@dataclass(frozen=True)
class GripperSpec:
    """夹爪单位换算：VLA 的开合量 <-> 我方关节弧度。

    **两边的「0」经常是反的**（A2D 模型 0=张开、我们的偏心轴 0 rad=夹紧），写成恒等映射
    的后果是「该松手时夹紧」，实机上很难当场认出来，所以两端都写死在这里。
    """

    model_open: float = 0.0
    model_closed: float = 1.0
    robot_open_rad: float = 0.0
    robot_closed_rad: float = 0.0

    def __post_init__(self) -> None:
        if self.model_open == self.model_closed:
            raise ValueError('model_open 与 model_closed 不能相等')

    @property
    def robot_limits(self) -> tuple[float, float]:
        return (min(self.robot_open_rad, self.robot_closed_rad),
                max(self.robot_open_rad, self.robot_closed_rad))

    def to_robot(self, value) -> np.ndarray:
        """VLA 的开合量 -> 关节弧度，夹到行程内。"""
        t = ((np.asarray(value, dtype=np.float64) - self.model_open)
             / (self.model_closed - self.model_open))
        rad = self.robot_open_rad + t * (self.robot_closed_rad - self.robot_open_rad)
        return np.clip(rad, *self.robot_limits)

    def to_model(self, rad) -> np.ndarray:
        """关节弧度 -> VLA 的开合量（发 state 时用，必须与 :meth:`to_robot` 互逆）。"""
        span = self.robot_closed_rad - self.robot_open_rad
        if span == 0.0:
            raise ValueError('robot_open_rad 与 robot_closed_rad 不能相等')
        t = (np.asarray(rad, dtype=np.float64) - self.robot_open_rad) / span
        return self.model_open + t * (self.model_closed - self.model_open)


@dataclass(frozen=True)
class VlaSpec:
    """一个 VLA 的完整规格。默认值写在 backend 模块里，yaml 只覆盖需要现场标定的项。"""

    name: str
    frame: FrameSpec = field(default_factory=FrameSpec)
    images: ImageSpec = field(default_factory=ImageSpec)
    gripper: GripperSpec = field(default_factory=GripperSpec)
    #: 服务实测返回的 waypoint 数，仅作记录；0 = 不固定。真实长度以 ``ActionChunk.horizon`` 为准。
    horizon: int = 0
    #: ``absolute`` = 输出绝对末端位姿；``delta`` = 输出增量。决定节点能不能开重锚。
    action_semantics: str = 'absolute'

    def summary(self) -> dict[str, Any]:
        """给 ``~/status`` 和启动日志用的一段人读摘要。"""
        return {
            'name': self.name,
            'action_semantics': self.action_semantics,
            'horizon': self.horizon,
            'images': list(self.images.slots),
            'image_height': self.images.height,
            'origin_in_base': [round(float(v), 4) for v in self.frame.origin_in_base],
            'rotation_rpy': [round(float(v), 6) for v in self.frame.rotation_rpy],
        }


# -- 节点与 backend 之间的两个数据结构 ---------------------------------------

@dataclass(frozen=True)
class CameraCalibration:
    """我方头部相机的内参，backend 需要它才能把图重采样到训练相机上。"""

    intrinsics: tuple[float, float, float, float]     # fx, fy, cx, cy
    size: tuple[int, int]                             # width, height
    distortion: tuple[float, ...] = (0.0,) * 5        # plumb_bob [k1,k2,p1,p2,k3]


@dataclass
class Observation:
    """一组观测，全部在 ``base_frame`` 里。"""

    task: str
    #: BGR HxWx3，key 取自 :data:`IMAGE_SLOTS`。
    images: dict[str, np.ndarray]
    #: ``side -> (7,)`` 末端位姿 ``[x,y,z,qx,qy,qz,qw]``。
    poses: dict[str, np.ndarray]
    #: ``side -> float`` 夹爪当前弧度。
    grippers: dict[str, float]
    #: ``side -> bool`` 这一侧是否参与——发给模型的协议字段，不是执行开关。
    enabled: dict[str, bool]
    #: 4x4，``base_frame`` -> 头部相机光心。
    camera_in_base: np.ndarray
    camera: CameraCalibration | None = None


@dataclass
class ActionChunk:
    """一段动作，全部在 ``base_frame`` 里。"""

    #: ``side -> (N,7)`` 末端位姿。
    poses: dict[str, np.ndarray]
    #: ``side -> (N,)`` 夹爪弧度。
    grippers: dict[str, np.ndarray]

    def __post_init__(self) -> None:
        horizons = {self.poses[s].shape[0] for s in SIDES}
        horizons |= {self.grippers[s].shape[0] for s in SIDES}
        if len(horizons) != 1:
            raise ValueError(f'各路的 horizon 不一致: {sorted(horizons)}')
        if horizons == {0}:
            raise ValueError('horizon 为 0')
        for side in SIDES:
            if self.poses[side].shape[1] != 7:
                raise ValueError(f'{side} 位姿必须是 (N,7)，收到 {self.poses[side].shape}')

    @property
    def horizon(self) -> int:
        return int(self.poses[SIDES[0]].shape[0])


# -- backend 基类与加载 -------------------------------------------------------

class VlaBackend(abc.ABC):
    """一个 VLA 服务的封装。**协议细节只能出现在子类里。**

    子类模块必须导出：

    * ``SPEC`` —— :class:`VlaSpec` 默认值；
    * ``PARAMETERS`` —— ``{ROS 参数名: 默认值}``，由节点统一 declare 后原样传进来；
    * ``create(params) -> VlaBackend``。
    """

    def __init__(self, spec: VlaSpec) -> None:
        self.spec = spec
        #: 非空时把**真正发出去的**图像字节落盘，由节点在构造后设置。
        self.debug_dir = ''

    @abc.abstractmethod
    def infer(self, observation: Observation) -> ActionChunk:
        """跑一次推理。网络/服务/返回体任何问题都直接抛，由节点决定怎么退避。"""

    def stats(self) -> dict[str, Any]:
        """给 ``~/status`` 的补充信息，比如重投影填充率。"""
        return {}

    def close(self) -> None:
        """释放连接等资源。"""

    def dump(self, blobs: Mapping[str, bytes]) -> None:
        """把发出去的图落盘，用来核对「模型到底看到了什么」。失败就自动关掉不再重试。"""
        if not self.debug_dir:
            return
        try:
            os.makedirs(self.debug_dir, exist_ok=True)
            for name, blob in blobs.items():
                path = os.path.join(self.debug_dir, f'{name}.jpg')
                with open(path + '.tmp', 'wb') as handle:
                    handle.write(blob)
                os.replace(path + '.tmp', path)      # 原子替换，读的人不会撞到半张
        except OSError:
            self.debug_dir = ''


def _module(name: str):
    # 名字来自 ROS 参数，限死成单个标识符，免得被拿去 import 任意模块。
    if not name.isidentifier():
        raise ValueError(f'backend 名 {name!r} 非法，只能是一个 Python 标识符')
    try:
        return importlib.import_module(f'g1_vla_bridge.backends.{name}')
    except ImportError as error:
        raise ValueError(f'找不到 backend {name!r}: {error}') from error


def backend_parameters(name: str) -> dict[str, Any]:
    """这个 backend 需要节点替它 declare 的 ROS 参数及默认值。"""
    return dict(_module(name).PARAMETERS)


def load_backend(name: str, params: Mapping[str, Any]) -> VlaBackend:
    """按名字实例化 backend。``params`` 是 :func:`backend_parameters` 声明的那些。"""
    backend = _module(name).create(params)
    if not isinstance(backend, VlaBackend):
        raise TypeError(f'backend {name!r} 的 create() 没返回 VlaBackend')
    return backend


def resolve_sequence(value: Sequence[float] | None, default: Sequence[float],
                     length: int, name: str) -> tuple[float, ...]:
    """yaml 里给了就用给的，长度不对直接报错；没给（空数组）就退回默认值。"""
    if value is None or len(value) == 0:
        return tuple(float(v) for v in default)
    if len(value) != length:
        raise ValueError(f'{name} 需要 {length} 个数，收到 {len(value)} 个')
    return tuple(float(v) for v in value)
