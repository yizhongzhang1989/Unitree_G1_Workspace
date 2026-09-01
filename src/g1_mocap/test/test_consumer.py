"""消费者侧缓冲的单测。用 stub 消息，不需要 ROS 运行时。"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from g1_mocap.consumer import FrameBuffer

N_JOINTS, N_KEYS = 29, 5


def point(x: float, y: float, z: float) -> SimpleNamespace:
    return SimpleNamespace(x=x, y=y, z=z)


def pose(value: float) -> SimpleNamespace:
    return SimpleNamespace(position=point(value, 0.0, 0.78),
                           orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0))


def frame(stamp: float, value: float = 0.0, *, n_joints: int = N_JOINTS,
          n_keys: int = N_KEYS) -> SimpleNamespace:
    sec = int(stamp)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(
            sec=sec, nanosec=int(round((stamp - sec) * 1e9)))),
        joint_positions=[value] * n_joints,
        root=pose(value),
        anchor=pose(value + 0.1),
        key_body_positions=[point(value, 0.0, 0.0)] * n_keys,
        # 真实消息里有 24 个 Point。push_frame 不该去碰它——这里放个会炸的哨兵。
        human_joints=property(lambda self: 1 / 0),
    )


def status(**overrides) -> SimpleNamespace:
    base = dict(connected=True, calibrated=True, frames=100, dropped=2,
                body_status=1, body_message=0, last_error='')
    base.update(overrides)
    return SimpleNamespace(**base)


def filled(count: int = 20, step: float = 0.1) -> FrameBuffer:
    buffer = FrameBuffer(n_joints=N_JOINTS, n_keys=N_KEYS)
    for i in range(count):
        buffer.push_frame(frame(i * step, i * step))
    return buffer


def test_time_axis_comes_from_the_stamp():
    """时间轴必须用 header.stamp，不是到达时刻。

    那个 stamp 是发布端从头显时钟平移过来的，帧间隔干净；按到达时刻建轴会把 DDS 的
    调度抖动搅进速度差分里。
    """
    buffer = filled()
    oldest, newest = buffer.span()
    assert oldest == pytest.approx(0.0, abs=1e-6)
    assert newest == pytest.approx(1.9, abs=1e-6)


def test_interpolates_between_frames():
    batch = filled().sample(np.array([0.35]))
    assert batch.root_pos[0][0] == pytest.approx(0.35, abs=1e-6)
    assert batch.joint_pos[0][0] == pytest.approx(0.35, abs=1e-6)
    assert batch.key_pos[0][0][0] == pytest.approx(0.35, abs=1e-6)


def test_clamps_instead_of_extrapolating():
    buffer = filled()
    oldest, newest = buffer.span()
    batch = buffer.sample(np.array([oldest - 5.0, newest + 5.0]))
    assert batch.t[0] == pytest.approx(oldest)
    assert batch.t[1] == pytest.approx(newest)


def test_quaternion_order_is_converted():
    """ROS 是 xyzw，内部一律 wxyz。搞反了姿态会整个乱，但不会报错。

    用绕 z 转 90 度的四元数：w 和 z 非零、x 和 y 为零，顺序一错就是 0 和 0.707 换位。
    """
    buffer = FrameBuffer(n_joints=N_JOINTS, n_keys=N_KEYS)
    message = frame(0.0)
    half = np.sqrt(0.5)
    message.root.orientation = SimpleNamespace(w=half, x=0.0, y=0.0, z=half)
    buffer.push_frame(message)
    buffer.push_frame(frame(0.1))
    batch = buffer.sample(np.array([0.0]))
    assert batch.root_quat[0] == pytest.approx([half, 0.0, 0.0, half], abs=1e-9)


def test_out_of_order_frames_are_dropped():
    buffer = FrameBuffer(n_joints=N_JOINTS, n_keys=N_KEYS)
    assert buffer.push_frame(frame(1.0)) is True
    assert buffer.push_frame(frame(2.0)) is True
    assert buffer.push_frame(frame(1.5)) is False


@pytest.mark.parametrize('n_joints,n_keys', [(28, 5), (29, 4), (0, 5)])
def test_dimension_mismatch_is_rejected(n_joints, n_keys):
    """关节数对不上时宁可丢帧也不能塞进去——错位不报错，只是姿态悄悄不对。"""
    buffer = FrameBuffer(n_joints=N_JOINTS, n_keys=N_KEYS)
    assert buffer.push_frame(frame(0.0, n_joints=n_joints, n_keys=n_keys)) is False
    assert buffer.stats().dropped == 1
    assert '维度不符' in buffer.stats().last_error


def test_human_joints_are_not_parsed():
    """那 24 个 Point 占了整条消息四分之三的对象构造开销，跑控制环的下游不该碰。

    stub 里 human_joints 是个一读就抛的哨兵，push_frame 能过就说明真没读。
    """
    buffer = FrameBuffer(n_joints=N_JOINTS, n_keys=N_KEYS)
    assert buffer.push_frame(frame(0.0)) is True


def test_status_feeds_the_stall_detector():
    buffer = FrameBuffer(n_joints=N_JOINTS, n_keys=N_KEYS)
    assert buffer.stats().connected is False
    assert buffer.calibrated is False

    buffer.push_status(status())
    stats = buffer.stats()
    assert stats.connected is True and stats.frames == 100 and stats.dropped == 2
    assert stats.status == 1
    assert buffer.calibrated is True

    buffer.push_status(status(connected=False, calibrated=False))
    assert buffer.stats().connected is False
    assert buffer.calibrated is False


def test_needs_two_frames_before_sampling():
    buffer = FrameBuffer(n_joints=N_JOINTS, n_keys=N_KEYS)
    assert buffer.span() is None
    assert buffer.sample(np.array([0.0])) is None
    buffer.push_frame(frame(0.0))
    assert buffer.sample(np.array([0.0])) is None
