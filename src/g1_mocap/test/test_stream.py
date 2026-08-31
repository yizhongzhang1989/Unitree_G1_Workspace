"""报文解析、坐标系换算、时钟对齐、环形缓冲。不需要头显，也不需要网络。"""

from __future__ import annotations

import numpy as np
import pytest

from g1_mocap.skeleton import (
    SMPL_JOINTS,
    XR_TO_ROBOT,
    ClockAligner,
    parse_body,
)
from g1_mocap.stream import _RingBuffer
from g1_mocap.retarget import RetargetResult


def payload(positions_robot: np.ndarray, **overrides) -> dict:
    # XR_TO_ROBOT 是正交阵，右乘它等于左乘它的逆，正好把机器人坐标转回 XR 坐标。
    xr = positions_robot @ XR_TO_ROBOT
    body = {
        'joint_count': 24,
        'all_tracked': True,
        'status': 1,
        'message': 0,
        'joints': {name: {'position': list(xr[i]), 'orientation': [0.0, 0.0, 0.0, 1.0],
                          'position_valid': True, 'orientation_valid': True}
                   for i, name in enumerate(SMPL_JOINTS)},
    }
    body.update(overrides)
    return {'seq': 7, 't': 12.5, 'body': body}


def test_parse_body_converts_xr_axes():
    """OpenXR 的 Y 上 / -Z 前，换到机器人的 X 前 / Y 左 / Z 上。"""
    positions = np.arange(72, dtype=np.float64).reshape(24, 3) * 0.01
    frame = parse_body(payload(positions))
    assert frame is not None
    assert np.allclose(frame.positions, positions, atol=1e-12)
    assert frame.seq == 7 and frame.t == pytest.approx(12.5)


def test_xr_axis_convention_is_the_documented_one():
    """(x, y, z) -> (-z, -x, y)。写反一个号，整个人就是躺着或者背对着。"""
    assert np.allclose(XR_TO_ROBOT @ np.array([0.0, 0.0, -1.0]), [1.0, 0.0, 0.0])
    assert np.allclose(XR_TO_ROBOT @ np.array([1.0, 0.0, 0.0]), [0.0, -1.0, 0.0])
    assert np.allclose(XR_TO_ROBOT @ np.array([0.0, 1.0, 0.0]), [0.0, 0.0, 1.0])
    assert np.linalg.det(XR_TO_ROBOT) == pytest.approx(1.0)


@pytest.mark.parametrize('mutate', [
    pytest.param(lambda p: p.clear(), id='empty'),
    pytest.param(lambda p: p.update(body=None), id='no-body'),
    pytest.param(lambda p: p['body'].update(joints=[]), id='joints-not-a-dict'),
    pytest.param(lambda p: p['body']['joints'].pop('RIGHT_WRIST'), id='missing-joint'),
    pytest.param(lambda p: p['body']['joints']['LEFT_ANKLE'].update(position_valid=False),
                 id='invalid-position'),
    pytest.param(lambda p: p['body']['joints']['PELVIS'].update(position=[0.0, float('nan'), 0.0]),
                 id='nan'),
    pytest.param(lambda p: p['body']['joints']['PELVIS'].update(position=[0.0, 0.0]),
                 id='short-vector'),
    pytest.param(lambda p: p['body']['joints']['PELVIS'].update(position=['a', 'b', 'c']),
                 id='not-numbers'),
])
def test_parse_body_rejects_bad_payloads(mutate):
    """报文来自网络。坏帧一律丢掉返回 None，不能抛——收帧线程里抛异常等于断流。"""
    bad = payload(np.zeros((24, 3)))
    mutate(bad)
    assert parse_body(bad) is None


def test_body_status_gates_usability():
    assert parse_body(payload(np.zeros((24, 3)), status=0)).usable is False
    assert parse_body(payload(np.zeros((24, 3)), status=1)).usable is True
    # LIMITED 放行：精度降级但数值仍然连续，比直接断流强。
    assert parse_body(payload(np.zeros((24, 3)), status=2)).usable is True


def test_clock_aligner_absorbs_arrival_jitter():
    """WiFi 的几毫秒抖动不能进到参考的速度差分里：50 Hz 下 5 ms 就是 25% 的速度误差。"""
    aligner = ClockAligner()
    aligner.stamp(100.0, 1000.0)
    assert aligner.stamp(100.02, 1000.025) == pytest.approx(1000.02)
    assert aligner.stamp(100.04, 1000.035) == pytest.approx(1000.04)


def test_clock_aligner_resyncs_on_jump():
    aligner = ClockAligner(resync_threshold_s=0.25)
    aligner.stamp(100.0, 1000.0)
    assert aligner.stamp(0.0, 1005.0) == pytest.approx(1005.0)  # 头显重启，t 归零


##
# 环形缓冲
##

def sample_result(value: float) -> RetargetResult:
    return RetargetResult(
        t=0.0,
        joint_pos=np.full(29, value),
        root_pos=np.array([value, 0.0, 0.78]),
        root_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        anchor_pos=np.array([value, 0.0, 0.82]),
        anchor_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        key_pos=np.tile([value, 0.0, 0.0], (5, 1)),
    )


def filled_buffer(count: int = 20, step: float = 0.1) -> _RingBuffer:
    buffer = _RingBuffer(64, 29, 5)
    for i in range(count):
        buffer.push(i * step, sample_result(i * step))
    return buffer


def test_buffer_interpolates_linearly():
    batch = filled_buffer().sample(np.array([0.35]))
    assert batch.root_pos[0][0] == pytest.approx(0.35, abs=1e-9)
    assert batch.joint_pos[0][0] == pytest.approx(0.35, abs=1e-9)


def test_buffer_clamps_instead_of_extrapolating():
    """外推出来的速度会直接喂给策略，宁可钳在端点。"""
    buffer = filled_buffer()
    oldest, newest = buffer.span()
    batch = buffer.sample(np.array([oldest - 10.0, newest + 10.0]))
    assert batch.t[0] == pytest.approx(oldest)
    assert batch.t[1] == pytest.approx(newest)
    assert batch.root_pos[1][0] == pytest.approx(newest, abs=1e-9)


def test_buffer_drops_out_of_order_frames():
    """时间轴必须单调，否则插值会取到反向区间。"""
    buffer = _RingBuffer(16, 29, 5)
    assert buffer.push(1.0, sample_result(1.0)) is True
    assert buffer.push(2.0, sample_result(2.0)) is True
    assert buffer.push(1.5, sample_result(1.5)) is False


def test_buffer_wraps_and_keeps_time_order():
    buffer = filled_buffer(count=200, step=0.01)  # 容量 64，绕了两圈多
    oldest, newest = buffer.span()
    assert newest == pytest.approx(1.99)
    assert oldest == pytest.approx(newest - 63 * 0.01)
    batch = buffer.sample(np.linspace(oldest, newest, 17))
    assert np.all(np.diff(batch.t) > 0)


def test_buffer_needs_two_frames():
    buffer = _RingBuffer(16, 29, 5)
    assert buffer.span() is None
    assert buffer.sample(np.array([0.0])) is None
    buffer.push(0.0, sample_result(0.0))
    assert buffer.sample(np.array([0.0])) is None
