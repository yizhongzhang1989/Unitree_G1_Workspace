"""delta 模式的重锚：形状保持、偏置抵消、旋转不抵消。"""

import numpy as np
import pytest

from g1_vla_bridge.transforms import FrameTransform, quat_angle, quat_to_mat, reanchor


def _traj(rng, n=8):
    poses = np.empty((n, 7))
    poses[:, :3] = np.cumsum(rng.normal(scale=0.02, size=(n, 3)), axis=0) + [0.3, 0.2, 0.1]
    quats = rng.normal(size=(n, 4))
    poses[:, 3:] = quats / np.linalg.norm(quats, axis=1, keepdims=True)
    return poses


def test_first_waypoint_lands_on_anchor():
    """首点必须恒等于锚点——所以 delta 下拦「首点瞬移」那类准入门根本不会触发。"""
    rng = np.random.default_rng(0)
    poses = _traj(rng)
    anchor = np.array([0.1, -0.2, 0.05, 0.0, 0.0, 0.0, 1.0])
    out = reanchor(poses, anchor)
    assert np.allclose(out[0, :3], anchor[:3], atol=1e-12)
    assert quat_angle(out[0, 3:], anchor[3:]) < 1e-6


def test_shape_is_preserved():
    """相邻点的相对位移与相对转角都不能变。"""
    rng = np.random.default_rng(1)
    poses = _traj(rng)
    quat = rng.normal(size=4)
    anchor = np.concatenate([[0.4, 0.1, -0.3], quat / np.linalg.norm(quat)])
    out = reanchor(poses, anchor)
    assert np.allclose(np.diff(out[:, :3], axis=0), np.diff(poses[:, :3], axis=0), atol=1e-12)
    for i in range(len(poses) - 1):
        assert np.isclose(quat_angle(out[i, 3:], out[i + 1, 3:]),
                          quat_angle(poses[i, 3:], poses[i + 1, 3:]), atol=1e-6)


@pytest.mark.parametrize('offset', ([0.0, 0.0, 0.0], [0.37, -0.04, 0.54], [-2.0, 5.0, 1.0]))
def test_base_offset_cancels(offset):
    """base_offset / tool_offset 标错多少，delta 模式的结果都一样。"""
    rng = np.random.default_rng(2)
    model_trans = rng.normal(scale=0.05, size=(6, 3)) + [0.5, 0.3, 0.7]
    model_rot = [quat_to_mat(q / np.linalg.norm(q)) for q in rng.normal(size=(6, 4))]
    anchor = np.array([0.2, 0.25, 0.06, 0.0, 0.0, 0.0, 1.0])

    frame = FrameTransform(base_offset=offset, tool_offset=[0.0, 0.0, -0.0281],
                           tool_rotation_rpy=[0.0, 0.0, np.pi])
    poses = np.stack([frame.from_model(model_trans[i], model_rot[i]) for i in range(6)])
    out = reanchor(poses, anchor)

    baseline = FrameTransform(tool_offset=[0.0, 0.0, -0.0281],
                              tool_rotation_rpy=[0.0, 0.0, np.pi])
    expect = reanchor(np.stack([baseline.from_model(model_trans[i], model_rot[i])
                                for i in range(6)]), anchor)
    assert np.allclose(out[:, :3], expect[:, :3], atol=1e-12)


def test_base_rotation_does_not_cancel():
    """旋转必须仍然生效，否则「往前 10 cm」会走到别的方向去。"""
    rng = np.random.default_rng(3)
    model_trans = rng.normal(scale=0.05, size=(6, 3)) + [0.5, 0.3, 0.7]
    model_rot = [quat_to_mat(q / np.linalg.norm(q)) for q in rng.normal(size=(6, 4))]
    anchor = np.array([0.2, 0.25, 0.06, 0.0, 0.0, 0.0, 1.0])

    def run(rpy):
        frame = FrameTransform(base_rotation_rpy=rpy)
        return reanchor(np.stack([frame.from_model(model_trans[i], model_rot[i])
                                  for i in range(6)]), anchor)

    spread = float(np.max(np.linalg.norm(run([0.0, -0.84, 0.0])[:, :3] - run([0.0] * 3)[:, :3],
                                         axis=1)))
    assert spread > 0.01


def test_progress_accumulates_only_when_anchored_on_command():
    """锚在实测上会把走过的一截抹掉，机器人只会原地抖——这是 2026-08-17 踩过的坑。

    推理一轮只播得完整段的前几个 waypoint，所以每段的锚点必须是上一段留下的**指令值**。
    """
    poses = np.zeros((10, 7))
    poses[:, 0] = np.linspace(0.0, 0.09, 10)   # 沿 +x 直线推进 9 cm
    poses[:, 6] = 1.0
    start = np.array([0.30, 0.20, 0.10, 0.0, 0.0, 0.0, 1.0])
    played = 3                                  # 一轮推理只走得完前 3 个

    command = reanchor(poses, start)[played]
    on_command = reanchor(poses, command)[played]
    on_measured = reanchor(poses, start)[played]   # 实测几乎没动，等价于锚回起点

    assert np.isclose(command[0] - start[0], 0.03, atol=1e-9)
    assert np.isclose(on_command[0] - start[0], 0.06, atol=1e-9)   # 继续推进
    assert np.isclose(on_measured[0] - start[0], 0.03, atol=1e-9)  # 原地踏步


def test_position_and_rotation_can_be_chosen_independently():
    """位置的标定不确定、姿态的确定，所以两者要能分开用增量。"""
    rng = np.random.default_rng(7)
    poses = _traj(rng)
    quat = rng.normal(size=4)
    anchor = np.concatenate([[0.4, 0.1, -0.3], quat / np.linalg.norm(quat)])

    only_pos = reanchor(poses, anchor, position=True, rotation=False)
    assert np.allclose(only_pos[0, :3], anchor[:3], atol=1e-12)
    assert np.allclose(only_pos[:, 3:], poses[:, 3:], atol=1e-12)   # 姿态原样透传

    only_rot = reanchor(poses, anchor, position=False, rotation=True)
    assert np.allclose(only_rot[:, :3], poses[:, :3], atol=1e-12)   # 位置原样透传
    assert quat_angle(only_rot[0, 3:], anchor[3:]) < 1e-6

    both_off = reanchor(poses, anchor, position=False, rotation=False)
    assert np.allclose(both_off, poses, atol=1e-12)                 # 纯绝对模式
