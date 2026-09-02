"""订了 ``/mocap/frame`` 的下游用的缓冲。

按时间插值取样这件事只在这里做：:class:`~.stream.MocapStream` 那边只负责收头显、
跑重定向、算完回调，不攒重定向结果——攒了也没人取，那条路上的消费者全都在话题这
一侧。

时间轴用 ``header.stamp``（ROS 时钟），不是收到的时刻：那个 stamp 是发布端从头显时钟
平移过来的，帧间隔干净；按到达时刻建轴会把 DDS 的调度抖动搅进速度差分里。

.. note::
   :meth:`FrameBuffer.push_frame` **故意不解析** ``human_joints``。那 24 个 ``Point``
   只有可视化才用，而它占了整条消息里对象构造开销的四分之三。跑控制环的下游别去碰它。
"""

from __future__ import annotations

import threading

import numpy as np

from .retarget import RetargetResult
from .stream import SampleBatch, StreamStats, _RingBuffer


def _point_array(points) -> np.ndarray:
    return np.array([[p.x, p.y, p.z] for p in points], dtype=np.float64)


def _pose(pose) -> tuple[np.ndarray, np.ndarray]:
    """``geometry_msgs/Pose`` -> (位置, wxyz 四元数)。ROS 是 xyzw，内部一律 wxyz。"""
    position = pose.position
    quat = pose.orientation
    return (np.array([position.x, position.y, position.z]),
            np.array([quat.w, quat.x, quat.y, quat.z]))


class FrameBuffer:
    """把 ``MocapFrame`` 攒起来，按时间插值取样。线程安全。

    Args:
        n_joints: 关节数，必须与消息里的一致。
        n_keys: key body 数，同上。
        buffer_s: 缓冲时长，要盖住下游窗口的整个跨度加上重同步余量。
    """

    def __init__(self, *, n_joints: int, n_keys: int, buffer_s: float = 2.0) -> None:
        # 头显最高 90 Hz，容量按 120 Hz 折算留出余量。
        self._buffer = _RingBuffer(max(int(buffer_s * 120), 64), n_joints, n_keys)
        self._n_joints, self._n_keys = int(n_joints), int(n_keys)
        self._stats = StreamStats()
        self._calibrated = False
        self._lock = threading.Lock()

    def push_frame(self, message) -> bool:
        """收一帧。返回 False 表示这帧被丢了（乱序、或者维度对不上）。

        跑在订阅回调里，每帧一次、90 Hz 上限，所以只做必要的转换。
        """
        if (len(message.joint_positions) != self._n_joints
                or len(message.key_body_positions) != self._n_keys):
            with self._lock:
                self._stats.dropped += 1
                self._stats.last_error = (
                    f'维度不符: {len(message.joint_positions)} 轴 / '
                    f'{len(message.key_body_positions)} key body')
            return False

        stamp = message.header.stamp
        root_pos, root_quat = _pose(message.root)
        anchor_pos, anchor_quat = _pose(message.anchor)
        result = RetargetResult(
            t=0.0,
            # float64[] 过来就是 array.array，asarray 是零拷贝包装。
            joint_pos=np.asarray(message.joint_positions, dtype=np.float64),
            root_pos=root_pos, root_quat=root_quat,
            anchor_pos=anchor_pos, anchor_quat=anchor_quat,
            key_pos=_point_array(message.key_body_positions),
        )
        return self._buffer.push(stamp.sec + stamp.nanosec * 1e-9, result)

    def push_status(self, message) -> None:
        """收一条状态。下游的断流判据全从这里来。"""
        with self._lock:
            self._stats.connected = bool(message.connected)
            self._stats.frames = int(message.frames)
            self._stats.dropped = int(message.dropped)
            self._stats.status = int(message.body_status)
            self._stats.message = int(message.body_message)
            self._stats.last_error = message.last_error
            self._calibrated = bool(message.calibrated)

    @property
    def calibrated(self) -> bool:
        with self._lock:
            return self._calibrated

    def span(self) -> tuple[float, float] | None:
        return self._buffer.span()

    def sample(self, times: np.ndarray) -> SampleBatch | None:
        return self._buffer.sample(times)

    def stats(self) -> StreamStats:
        with self._lock:
            return StreamStats(**vars(self._stats))
