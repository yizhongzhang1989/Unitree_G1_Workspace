"""回放核心的单测：不需要 ROS，也不需要真机。"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from record.replay import (ARM_LEN, Playback, lerp_arm,  # noqa: E402
                           pose_from_status, quat_slerp, ramp)


# 真机上抓的一帧 status：limited_pose 是 dict 不是扁平列表，按扁平写会静默失败
REAL_STATUS = {
    'arm_mode': 'ik', 'arms_live': True, 'state': 'stand',
    'limited_pose': {
        'left': [0.30324, 0.25483, 0.09213, 0.62504, 0.33052, 0.62563, 0.32964],
        'right': [0.30324, -0.25482, 0.09213, 0.33052, 0.62504, 0.32964, 0.62563],
    },
}


def test_pose_from_status_真机样本():
    p = pose_from_status(REAL_STATUS)
    assert p.shape == (ARM_LEN,)
    assert np.isclose(p[1], 0.25483) and np.isclose(p[8], -0.25482)


def test_pose_from_status_拒绝扁平列表():
    """dict 的 len 是 2，按扁平列表判长度会漏过去 —— 必须显式拒绝。"""
    with pytest.raises(ValueError, match='不是 dict'):
        pose_from_status({'limited_pose': [0.0] * 14})


def test_pose_from_status_拒绝缺边():
    with pytest.raises(ValueError, match='right'):
        pose_from_status({'limited_pose': {'left': [0.0] * 7}})


def test_pose_from_status_拒绝长度不对():
    with pytest.raises(ValueError, match='7 个数'):
        pose_from_status({'limited_pose': {'left': [0.0] * 6, 'right': [0.0] * 7}})


def _pose(x, qw=1.0, qz=0.0):
    return np.array([x, 0.0, 0.0, 0.0, 0.0, qz, qw], dtype=float)


def _arm(x=0.0, qz=0.0, qw=1.0):
    return np.concatenate([_pose(x, qw, qz), _pose(-x, qw, qz)])


def test_slerp_端点精确():
    a, b = np.array([0., 0., 0., 1.]), np.array([0., 0., .7071, .7071])
    assert np.allclose(quat_slerp(a, b, 0.0), a)
    assert np.allclose(quat_slerp(a, b, 1.0), b / np.linalg.norm(b))


def test_slerp_保持单位长度():
    a, b = np.array([0., 0., 0., 1.]), np.array([0., .5, .5, .5])
    for u in np.linspace(0, 1, 11):
        assert np.isclose(np.linalg.norm(quat_slerp(a, b, float(u))), 1.0)


def test_slerp_走短弧():
    """点积为负时要取反，否则绕远路转一大圈。"""
    a = np.array([0., 0., 0., 1.])
    b = -np.array([0., 0., .7071, .7071])       # 与 a 同一个姿态，符号相反
    mid = quat_slerp(a, b, 0.5)
    assert np.dot(mid, quat_slerp(a, -b, 0.5)) > 0.99


def test_ramp_端点与长度():
    start, end = _arm(0.0), _arm(0.5)
    out = ramp(start, end, seconds=2.0, hz=50.0)
    assert out.shape == (100, ARM_LEN)
    assert np.allclose(out[-1], end, atol=1e-9)
    assert not np.allclose(out[0], start)        # 第一步已经动了


def test_ramp_两端速度趋零():
    """余弦缓入：头尾的步长应明显小于中段。"""
    out = ramp(_arm(0.0), _arm(1.0), seconds=2.0, hz=50.0)
    step = np.abs(np.diff(out[:, 0]))
    assert step[0] < step[len(step) // 2] / 3
    assert step[-1] < step[len(step) // 2] / 3


def test_playback_按时间取样():
    t = np.array([100.0, 100.5, 101.0])
    arm = np.stack([_arm(0.0), _arm(1.0), _arm(2.0)])
    grip = np.array([[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]])
    p = Playback(t, arm, grip, 100.0, 101.0)
    assert np.isclose(p.duration, 1.0)

    a, g, done = p.sample(0.25)
    assert np.isclose(a[0], 0.5) and np.isclose(g[0], 0.5) and not done
    a, g, done = p.sample(2.0)
    assert done and np.isclose(a[0], 2.0)


def test_playback_速度倍率():
    t = np.array([0.0, 1.0, 2.0])
    arm = np.stack([_arm(0.0), _arm(1.0), _arm(2.0)])
    grip = np.zeros((3, 2))
    p = Playback(t, arm, grip, 0.0, 2.0, speed=2.0)
    assert np.isclose(p.duration, 1.0)
    a, _, _ = p.sample(0.5)                      # 半程实时 = 全轨迹的 1.0
    assert np.isclose(a[0], 1.0)


def test_playback_拒绝离谱的速度():
    t = np.array([0.0, 1.0])
    arm = np.stack([_arm(0.0), _arm(1.0)])
    with pytest.raises(ValueError):
        Playback(t, arm, np.zeros((2, 2)), 0.0, 1.0, speed=10.0)


def test_playback_区间太短就拒绝():
    t = np.array([0.0, 1.0, 2.0])
    arm = np.stack([_arm(0.0), _arm(1.0), _arm(2.0)])
    with pytest.raises(ValueError, match='放不了'):
        Playback(t, arm, np.zeros((3, 2)), 0.4, 0.6)


def test_lerp_arm_两臂互不串():
    start = np.concatenate([_pose(0.0), _pose(10.0)])
    end = np.concatenate([_pose(1.0), _pose(20.0)])
    mid = lerp_arm(start, end, 0.5)
    assert np.isclose(mid[0], 0.5) and np.isclose(mid[7], 15.0)


def _bare_session(root, tables):
    """造一个最小 session 目录。tables 是 schema 里要声明哪些表。"""
    d = root / '20260101_000000'
    (d / 'signals').mkdir(parents=True)
    (d / 'manifest.json').write_text('{"streams": {}}', encoding='utf-8')
    (d / 'meta.json').write_text('{"note": "x"}', encoding='utf-8')
    (d / 'schema.json').write_text(json.dumps({'tables': tables}), encoding='utf-8')
    (d / 'events.jsonl').write_text('', encoding='utf-8')
    (d / 'DONE').write_text('x\n', encoding='utf-8')
    return d


def test_没录指令表时给出干净的错误而不是抛KeyError(tmp_path):
    """勾选是可以不勾 motion_control_command 的，所以这是正常情况。

    这里曾经让 KeyError 一路冒到 HTTP 处理线程，把连接直接打死 —— 浏览器只看到
    ERR_EMPTY_RESPONSE，面板上什么都不显示。
    """
    from record.replay_source import describe
    d = _bare_session(tmp_path, {'joint_states': {'ncol': 3, 'rows': 0,
                                                  'columns': ['t_recv', 't_header', 'q0']}})
    info = describe(d)
    assert 'error' in info and 'motion_control_command' in info['error']
