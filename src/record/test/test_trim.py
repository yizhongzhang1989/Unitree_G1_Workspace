"""`export.active_span` —— 把 episode 收缩到「目标真的在动」的那一段。

夹爪这一项有两个方向的坑，下面都钉了：

* **不能看电平**。「左手把东西递给右手」时操作者全程夹着，按开合与否判会认为
  动作从头到尾都在，一秒都裁不掉。
* **也不能看速率**。VR 扳机是模拟量，手指碰一下能在 0.19 s 里跑出行程的 2%，
  换算成 0.29 rad/s，足以把一条 episode 开头 2.3 s 的空转保下来。看幅度才对。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
HERE = TOOLS / 'format' / 'YB'
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(HERE))

import export as ex                                        # noqa: E402

HZ = 50.0
ARMS = ('left', 'right')
COLUMNS = ([f'limited.{s}.{k}' for s in ARMS
            for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')]
           + [f'grip.{s}' for s in ARMS])


class _Session:
    """只提供 active_span 用到的两个方法。"""

    def __init__(self, t, pose, grip):
        self._t = np.asarray(t, float)
        data = np.zeros((self._t.size, len(COLUMNS)))
        data[:, :14] = pose
        data[:, 14:] = grip
        self._data = data

    def tables(self):
        return {'motion_control_status'}

    def columns(self, key):
        return list(COLUMNS)

    def table(self, key):
        return self._t, self._data


def _session(duration, move_from, move_to, *, speed=0.15, grip_level=0.0,
             grip_pulse=None):
    """静止 -> 以 ``speed`` m/s 沿 x 移动 -> 静止。夹爪默认恒定在 ``grip_level``。"""
    t = np.arange(0.0, duration, 1.0 / HZ)
    pose = np.zeros((t.size, 14))
    pose[:, 3] = pose[:, 10] = 1.0                       # 单位四元数
    ramp = np.clip(t - move_from, 0.0, max(move_to - move_from, 0.0))
    pose[:, 0] = ramp * speed                            # 左手 x
    grip = np.full((t.size, 2), float(grip_level))
    if grip_pulse is not None:
        lo, hi = grip_pulse
        closing = (t >= lo) & (t <= hi)
        grip[closing, 0] = np.linspace(0.0, 2.0, int(closing.sum()))
        grip[t > hi, 0] = 2.0
    return _Session(t, pose, grip)


def test_holding_the_gripper_closed_is_not_motion():
    """「一直夹着」不能算动作 —— 这是 pass 类指令的常态。

    操作者夹着东西点「开始」，隔一会儿才动手臂；按夹爪电平判会一秒都裁不掉。
    """
    ses = _session(20.0, move_from=8.0, move_to=12.0, grip_level=2.0)
    t0, t1 = ex.active_span(ses, 0.0, 20.0, keep=1.0)
    assert 6.5 < t0 < 8.5, t0            # 起点贴着运动开始，前面留 1 s
    assert 11.5 < t1 < 13.5, t1
    assert t1 - t0 < 8.0                 # 20 s 里绝大部分被裁掉


def test_gripper_alone_counts_even_when_the_arm_is_still():
    """手臂停着只合爪也是真动作，不能当成空隙砍掉。"""
    ses = _session(20.0, move_from=12.0, move_to=16.0,
                   grip_pulse=(6.0, 7.0))
    t0, _ = ex.active_span(ses, 0.0, 20.0, keep=1.0)
    assert t0 < 6.5, f'合爪没被算进动作，起点跑到了 {t0}'


def test_a_trigger_flick_is_not_a_grasp():
    """扳机抖一下又弹回去（行程的 2%）不能把前面的空转保下来。

    实测就是它让一条 episode 多留了 2.3 s 的静止画面。
    """
    ses = _session(20.0, move_from=10.0, move_to=14.0, grip_level=2.764)
    flick = (ses._t >= 4.0) & (ses._t <= 4.15)
    ses._data[flick, 14] = 2.71
    t0, _ = ex.active_span(ses, 0.0, 20.0, keep=1.0)
    assert t0 > 8.5, f'扳机抖动被当成了动作，起点停在 {t0}'


def test_nothing_moves_leaves_the_window_alone():
    """一动没动时原样返回，绝不裁成空 episode。"""
    ses = _session(10.0, move_from=99.0, move_to=99.0, grip_level=1.0)
    assert ex.active_span(ses, 0.0, 10.0, keep=1.0) == (0.0, 10.0)


def test_keep_idle_controls_how_much_slack_survives():
    ses = _session(20.0, move_from=8.0, move_to=12.0)
    tight = ex.active_span(ses, 0.0, 20.0, keep=0.0)
    loose = ex.active_span(ses, 0.0, 20.0, keep=3.0)
    assert abs((tight[0] - loose[0]) - 3.0) < 0.1
    assert abs((loose[1] - tight[1]) - 3.0) < 0.1


def test_never_widens_past_the_recorded_episode():
    """keep 再大也不能把别的 episode 的内容拉进来。"""
    ses = _session(20.0, move_from=9.0, move_to=11.0)
    t0, t1 = ex.active_span(ses, 5.0, 15.0, keep=100.0)
    assert (t0, t1) == (5.0, 15.0)


def test_trim_episode_records_what_it_cut():
    """裁掉多少必须留痕，否则事后无法判断 h5 里那段是不是被动过。"""
    ses = _session(20.0, move_from=8.0, move_to=12.0, grip_level=2.0)
    episode = {'t0': 0.0, 't1': 20.0, 'duration': 20.0}
    ex.trim_episode(ses, episode, keep=1.0)
    cut = episode['trim']
    assert cut['raw_t0'] == 0.0 and cut['raw_t1'] == 20.0
    assert cut['head_s'] > 5.0 and cut['tail_s'] > 5.0
    assert episode['duration'] == episode['t1'] - episode['t0']


def test_missing_status_table_is_not_fatal():
    """这一路没录时不能让整个导出崩掉，退回原窗口即可。"""

    class _Empty(_Session):
        def tables(self):
            return set()

    ses = _Empty([0.0], np.zeros((1, 14)), np.zeros((1, 2)))
    assert ex.active_span(ses, 0.0, 10.0, keep=1.0) == (0.0, 10.0)
