"""实时动捕帧的双手离合门。"""

from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np

from .rotations import (
    quat_apply,
    quat_conj,
    quat_from_mat,
    quat_from_xyzw,
    quat_mul,
    quat_normalize,
    quat_to_mat,
    yaw_quat,
)


@dataclass(frozen=True)
class _FramePayload:
    joint_positions: object
    root: object
    anchor: object
    key_body_positions: object

    @classmethod
    def from_message(cls, message):
        return cls(
            joint_positions=message.joint_positions,
            root=message.root,
            anchor=message.anchor,
            key_body_positions=message.key_body_positions,
        )

    def at_stamp(self, header):
        return SimpleNamespace(
            header=header,
            joint_positions=self.joint_positions,
            root=self.root,
            anchor=self.anchor,
            key_body_positions=self.key_body_positions,
        )


def _pose_arrays(pose) -> tuple[np.ndarray, np.ndarray]:
    position = pose.position
    orientation = pose.orientation
    return (
        np.array([position.x, position.y, position.z], dtype=np.float64),
        quat_from_xyzw((orientation.x, orientation.y, orientation.z, orientation.w)),
    )


def _pose(position: np.ndarray, quat_wxyz: np.ndarray):
    return SimpleNamespace(
        position=SimpleNamespace(x=float(position[0]), y=float(position[1]),
                                 z=float(position[2])),
        orientation=SimpleNamespace(x=float(quat_wxyz[1]), y=float(quat_wxyz[2]),
                                    z=float(quat_wxyz[3]), w=float(quat_wxyz[0])),
    )


class ZeroReferenceFactory:
    """以输入 root 位姿承载固定站姿 FK，生成自洽的默认参考。"""

    def __init__(self, joint_positions: np.ndarray, anchor_local: np.ndarray,
                 anchor_rot: np.ndarray, key_local: np.ndarray) -> None:
        self._joint_positions = np.asarray(joint_positions, dtype=np.float64).copy()
        self._anchor_local = np.asarray(anchor_local, dtype=np.float64).copy()
        self._anchor_rot = np.asarray(anchor_rot, dtype=np.float64).copy()
        self._key_local = np.asarray(key_local, dtype=np.float64).copy()

    def __call__(self, message) -> _FramePayload:
        position = message.root.position
        orientation = message.root.orientation
        root_pos = np.array([position.x, position.y, position.z], dtype=np.float64)
        # squeeze 前的参考必须是真正的 canonical 站姿。只借用人体的世界位置和 yaw；
        # 若把第一帧人体 roll/pitch 冻在这里，21 个 token 的重力和 key body 都会倾斜。
        root_quat = yaw_quat(quat_from_xyzw(
            (orientation.x, orientation.y, orientation.z, orientation.w)))
        root_rot = quat_to_mat(root_quat)
        anchor_pos = root_pos + root_rot @ self._anchor_local
        anchor_quat = quat_from_mat(root_rot @ self._anchor_rot)
        key_pos = root_pos + self._key_local @ root_rot.T
        return _FramePayload(
            joint_positions=self._joint_positions,
            root=_pose(root_pos, root_quat),
            anchor=_pose(anchor_pos, anchor_quat),
            key_body_positions=[SimpleNamespace(x=float(point[0]), y=float(point[1]),
                                                z=float(point[2]))
                                for point in key_pos],
        )


class MocapFrameGate:
    """按模式写实时帧、全零默认姿势或最后一次实时姿势。"""

    def __init__(self, sink, default_factory: ZeroReferenceFactory) -> None:
        self._sink = sink
        self._default_factory = default_factory
        self._default: _FramePayload | None = None
        self._last: _FramePayload | None = None
        self._mode: str | None = None
        self._live_quat = np.array([1.0, 0.0, 0.0, 0.0])
        self._live_pos = np.zeros(3)

    @property
    def last_payload(self) -> _FramePayload | None:
        return self._last

    def _live_payload(self, message) -> _FramePayload:
        if self._mode != 'live':
            if self._last is None:
                self._live_quat = np.array([1.0, 0.0, 0.0, 0.0])
                self._live_pos = np.zeros(3)
            else:
                source_pos, source_quat = _pose_arrays(message.root)
                target_pos, target_quat = _pose_arrays(self._last.root)
                self._live_quat = yaw_quat(
                    quat_mul(target_quat, quat_conj(source_quat)))
                self._live_pos = target_pos - quat_apply(self._live_quat, source_pos)

        def transform_pose(pose):
            position, quat = _pose_arrays(pose)
            return _pose(
                self._live_pos + quat_apply(self._live_quat, position),
                quat_normalize(quat_mul(self._live_quat, quat)),
            )

        return _FramePayload(
            joint_positions=message.joint_positions,
            root=transform_pose(message.root),
            anchor=transform_pose(message.anchor),
            key_body_positions=[
                SimpleNamespace(x=float(point[0]), y=float(point[1]), z=float(point[2]))
                for point in (
                    self._live_pos + quat_apply(
                        self._live_quat,
                        np.array([[point.x, point.y, point.z]
                                 for point in message.key_body_positions], dtype=np.float64),
                    )
                )
            ],
        )

    def push(self, message, *, mode: str) -> bool:
        if mode == 'live':
            payload = self._live_payload(message)
            accepted = self._sink.push_frame(payload.at_stamp(message.header))
            if accepted:
                self._last = payload
                self._mode = mode
            return accepted
        if mode == 'default':
            if self._default is None:
                self._default = self._default_factory(message)
            payload = self._default
        elif mode == 'hold':
            payload = self._last or self._default_factory(message)
        else:
            raise ValueError(f'未知动捕门模式: {mode}')
        accepted = self._sink.push_frame(payload.at_stamp(message.header))
        if accepted:
            self._last = payload
            self._mode = mode
        return accepted
