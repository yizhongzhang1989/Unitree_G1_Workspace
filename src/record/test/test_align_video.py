"""视频时间对齐估计器的验收。

用合成信号钉住估计器本身的正确性 —— 真实数据要机器人动起来才有意义，
但估计器的逻辑（互相关、亚采样插值、置信度、慢变跟踪）在这里就能验完。
"""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

TOOLS = Path(__file__).resolve().parents[1] / 'tools'


def _load(name):
    """手工加载 tools/ 下的模块，模拟导出机「把目录拷过去直接用」的方式。

    必须先注册进 sys.modules：@dataclass 会去 sys.modules[cls.__module__] 找命名空间。
    """
    spec = importlib.util.spec_from_file_location(name, TOOLS / f'{name}.py')
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='module')
def align():
    return _load('align_video')


def _burst_signal(duration=60.0, hz=200.0, seed=0):
    """造一段「时动时停」的运动，形状接近真实遥操作。"""
    rng = np.random.default_rng(seed)
    t = np.arange(0, duration, 1.0 / hz)
    v = np.zeros_like(t)
    for start in rng.uniform(0, duration - 2, 40):
        w = rng.uniform(0.25, 0.9)
        m = (t >= start) & (t < start + w)
        v[m] += np.sin(np.pi * (t[m] - start) / w) ** 2 * rng.uniform(0.5, 2.0)
    return t, v


def test_no_ros_dependency():
    import re
    src = (TOOLS / 'align_video.py').read_text(encoding='utf-8')
    banned = re.compile(r'^\s*(?:import|from)\s+(rclpy|sensor_msgs|pinocchio)\b', re.M)
    assert not banned.search(src)


@pytest.mark.parametrize('truth', [-0.20, -0.05, 0.0, 0.033, 0.12, 0.25])
def test_recovers_known_delay(align, truth):
    """视频比状态晚 truth 秒，估计器要把它找回来。"""
    t, v = _burst_signal()
    vt = np.arange(0, 60.0, 1 / 25.0)          # 25 fps 的视频采样
    video = np.interp(vt, t + truth, v)        # 视频看到的是延迟后的运动
    est = align.estimate_delay(vt, video, t, v)
    assert abs(est.delay - truth) < 0.012, est
    assert est.trustworthy and est.score > 0.5


def test_subsample_resolution_beats_grid(align):
    """网格是 50 Hz(20 ms)，抛物线插值要能给出比它细的结果。"""
    truth = 0.0137
    t, v = _burst_signal(seed=3)
    vt = np.arange(0, 60.0, 1 / 25.0)
    est = align.estimate_delay(vt, np.interp(vt, t + truth, v), t, v)
    assert abs(est.delay - truth) < 0.008


def test_uncorrelated_input_is_flagged_not_guessed(align):
    """两路无关时必须报低置信，而不是给一个看起来像模像样的数。"""
    t, v = _burst_signal(seed=1)
    _, other = _burst_signal(seed=99)
    vt = np.arange(0, 60.0, 1 / 25.0)
    est = align.estimate_delay(vt, np.interp(vt, t, other), t, v)
    assert not est.trustworthy or est.score < 0.3


def test_static_scene_is_flagged(align):
    """画面不动、手臂不动时没有可对齐的特征，要老实说不知道。"""
    t = np.arange(0, 60.0, 1 / 200.0)
    v = np.zeros_like(t)
    vt = np.arange(0, 60.0, 1 / 25.0)
    est = align.estimate_delay(vt, np.zeros_like(vt), t, v)
    assert not est.trustworthy


def test_too_short_overlap_returns_zero(align):
    t = np.arange(0, 1.0, 0.005)
    est = align.estimate_delay(t, np.sin(t), t, np.sin(t))
    assert est.samples == 0 and est.delay == 0.0


def test_dc_offset_and_scale_do_not_matter(align):
    """包大小和角速度量纲完全不同，必须只对形状敏感。"""
    truth = 0.08
    t, v = _burst_signal(seed=5)
    vt = np.arange(0, 60.0, 1 / 25.0)
    video = np.interp(vt, t + truth, v) * 3000.0 + 45000.0
    est = align.estimate_delay(vt, video, t, v)
    assert abs(est.delay - truth) < 0.012


def test_slow_drift_is_tracked_by_windows(align):
    """偏移慢变时，分窗要能跟上 —— 实测 90 s 内 t0 能漂 138 ms。"""
    t, v = _burst_signal(duration=120.0, seed=7)
    vt = np.arange(0, 120.0, 1 / 25.0)
    drift = 0.05 + 0.10 * (vt / 120.0)          # 延迟从 50 ms 漂到 150 ms
    # 延迟 d 意味着 vt 时刻的画面来自 vt-d 时刻的运动
    video = np.array([np.interp(x - d, t, v) for x, d in zip(vt, drift)])
    first = align.estimate_delay(vt[vt < 30], video[vt < 30], t, v)
    last = align.estimate_delay(vt[vt > 90], video[vt > 90], t, v)
    assert first.trustworthy and last.trustworthy
    assert last.delay - first.delay > 0.05, (first, last)
    assert 0.03 < first.delay < 0.09 and 0.11 < last.delay < 0.17


def test_packet_motion_drops_keyframes(align, tmp_path):
    """关键帧字节比 P 帧大两三个数量级，不剔除会在每个 GOP 边界造假尖峰。"""
    import subprocess
    mkv = tmp_path / 'x.mkv'
    subprocess.run(
        ['ffmpeg', '-nostdin', '-v', 'error', '-f', 'lavfi',
         '-i', 'testsrc2=size=320x180:rate=25:duration=6',
         '-c:v', 'libx264', '-preset', 'ultrafast', '-g', '25', '-y', str(mkv)],
        check=True, stdin=subprocess.DEVNULL)
    idx, motion = align.motion_from_packets(mkv)
    assert idx.size > 100
    assert motion.max() / np.median(motion) < 20, '关键帧尖峰没被压掉'
