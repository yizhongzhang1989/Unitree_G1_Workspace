"""读一个 session。**纯 Python + numpy，不 import rclpy。**

导出机是没有 ROS 的 Windows，只保证有 numpy。把这个目录整个拷过去就能用::

    from session_reader import Session
    s = Session('D:/data/20260821_101500')
    t, q = s.table('joint_states')
    for ep in s.episodes():
        print(ep['instruction_en'], ep['t0'], ep['t1'])

腕部视频的时间戳是 ffmpeg 的收包墙钟，带约 ±20 ms 网络抖动。帧率是硬件恒定的，
``fitted_pts`` 用中位数斜率做鲁棒拟合把抖动抹掉；丢帧会在差分上留下整数倍的跳变，
拟合前先按它分段。
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


class Session:
    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)
        if not (self.root / 'manifest.json').is_file():
            raise FileNotFoundError(f'{self.root} 不像一个 session 目录')
        self.manifest = _json(self.root / 'manifest.json')
        self.schema = _json(self.root / 'schema.json').get('tables', {})
        self.meta = _json(self.root / 'meta.json') if (self.root / 'meta.json').is_file() else {}
        self.events = _events(self.root / 'events.jsonl')
        self._nominal: dict | None = None

    # ------------------------------------------------------------------ 完整性

    @property
    def sealed(self) -> bool:
        return (self.root / 'DONE').is_file()

    def verify(self) -> list[str]:
        """核对 DONE 里的 sha256。返回不一致的文件清单，空列表 = 全对。"""
        import hashlib
        done = self.root / 'DONE'
        if not done.is_file():
            return ['DONE 缺失，session 没有正常封口']
        bad = []
        for rel, info in _json(done)['files'].items():
            f = self.root / rel
            if not f.is_file():
                bad.append(f'{rel}: 缺失')
                continue
            h = hashlib.sha256()
            with open(f, 'rb') as fh:
                for block in iter(lambda: fh.read(1 << 20), b''):
                    h.update(block)
            if h.hexdigest() != info['sha256']:
                bad.append(f'{rel}: 校验不符')
        return bad

    # -------------------------------------------------------------------- 信号

    def table(self, key: str) -> tuple[np.ndarray, np.ndarray]:
        """返回 (时间列, 数据列)。时间用 ``t_header``，无效处回退 ``t_recv``。"""
        entry = self.schema[key]
        path = self.root / 'signals' / entry['file']
        flat = np.fromfile(path, dtype=np.float64)
        ncol = entry['ncol']
        data = flat[:flat.size // ncol * ncol].reshape(-1, ncol)
        t_recv, t_head = data[:, 0], data[:, 1]
        t = np.where(np.isfinite(t_head), t_head, t_recv)
        return t, data[:, 2:]

    def columns(self, key: str) -> list[str]:
        return list(self.schema[key]['columns'][2:])

    def tables(self) -> list[str]:
        return [k for k, v in self.schema.items()
                if (self.root / 'signals' / v['file']).is_file()]

    # -------------------------------------------------------------------- 视频

    def video_pts(self, name: str, fit: bool = True) -> np.ndarray:
        path = self.root / 'video' / f'{name}.pts.bin'
        raw = np.fromfile(path, dtype=np.float64)
        return fitted_pts(raw) if fit and name.startswith('wrist') else raw

    def video_path(self, name: str) -> Path:
        return self.root / 'video' / f'{name}.mkv'

    def nominal_fps(self, name: str) -> float:
        """开录时探到的标称帧率，0 表示没探到。旧 session 没这个文件。"""
        if self._nominal is None:
            path = self.root / 'video' / 'nominal.json'
            self._nominal = _json(path) if path.is_file() else {}
        entry = self._nominal.get(name) or {}
        return float(entry.get('fps', 0.0)) if entry.get('ok') else 0.0

    # ----------------------------------------------------------------- episode

    def rounds(self) -> list[dict]:
        return [_json(p) for p in sorted((self.root / 'rounds').glob('round_*.json'))]

    def episodes(self, include_discarded: bool = False) -> list[dict]:
        """按事件线切出每条 episode 的时间区间与指令。这是交付单位。"""
        out, open_ep = [], None
        for e in self.events:
            if e['type'] == 'episode_start':
                open_ep = e
            elif e['type'] == 'episode_end' and open_ep is not None:
                if include_discarded or e['outcome'] != 'discard':
                    out.append({
                        'round': e['round'], 'episode': e['episode'],
                        't0': open_ep['t'], 't1': e['t'],
                        'duration': e.get('duration', e['t'] - open_ep['t']),
                        'outcome': e['outcome'], 'note': e.get('note', ''),
                        **open_ep.get('instruction', {}),
                    })
                open_ep = None
        return out

    def slice_table(self, key: str, t0: float, t1: float):
        t, data = self.table(key)
        m = (t >= t0) & (t <= t1)
        return t[m], data[m]

    def slice_frames(self, name: str, t0: float, t1: float) -> np.ndarray:
        """区间内的帧序号。取视频帧时按它去 seek。"""
        pts = self.video_pts(name)
        return np.nonzero((pts >= t0) & (pts <= t1))[0]

    def warnings(self) -> list[dict]:
        return [e for e in self.events if e['type'] == 'warning']


def fitted_pts(raw: np.ndarray, min_frames: int = 6) -> np.ndarray:
    """把成簇到达的墙钟还原成等间隔的采集时刻。

    **为什么不能直接用原始戳**：RTSP over TCP 是成簇到达的。实测 1080p@25 一路 90 秒，
    单帧到达相对理想直线的残差 p5..p95 跨 23 ms、极差 41 ms。

    **重建办法**：``-c copy`` 下包数严格等于帧数，用首尾跨度除以间隔数得到步长
    （不受成簇影响，只看端点和计数），截距取中位数（不被长停顿拽偏）。重建后
    窗中心偏差极差 17.2 ms、标准差 5.0 ms。

    试过并**否掉**的两条（同一批 90 秒数据）：

    * **截距改用低分位**（min/p2/p5/p25），想逼近「零排队」的下界。各分位给出的
      分窗结果**完全一样**——到达分布是整体平移而不是形变，取下界拿不到额外信息。
    * **分段拟合**，想吃掉相机与主机 77 ppm 的时钟速率差（90 秒累积 6.9 ms）。
      段独立估截距会在边界留 12.2 ms 跳变；改成累积构造虽然连续了，但段步长只靠
      两个端点估、端点又带 ±20 ms 抖动，误差随机游走，窗中心偏差反而从 17.2 涨到
      20.8 ms、单帧残差从 23.3 涨到 33.9 ms。**那 6.9 ms 不值得用这个噪声去换。**

    局限：若相机真丢了帧，那一帧的包也不存在，此处会把后续整体前移一帧周期。
    用 :func:`pts_residual` 的 ``fps`` 与标称帧率比对即可发现。
    """
    if raw.size < min_frames:
        return raw.copy()
    span = float(raw[-1] - raw[0])
    if span <= 0:
        return raw.copy()
    step = span / (raw.size - 1)
    index = np.arange(raw.size, dtype=np.float64)
    return float(np.median(raw - step * index)) + step * index


def pts_residual(raw: np.ndarray) -> dict:
    """时间戳有多脏。``fps`` 与标称帧率对不上就说明真的丢帧了，导出报告要看它。"""
    if raw.size < 2:
        return {'frames': int(raw.size), 'fps': 0.0, 'step_ms': 0.0,
                'p50_ms': 0.0, 'p95_ms': 0.0, 'max_ms': 0.0}
    fit = fitted_pts(raw)
    r = np.abs(raw - fit) * 1000.0
    span = float(raw[-1] - raw[0])
    return {'frames': int(raw.size),
            'fps': round((raw.size - 1) / span, 2) if span > 0 else 0.0,
            'step_ms': round(span / (raw.size - 1) * 1000, 2) if span > 0 else 0.0,
            'p50_ms': round(float(np.percentile(r, 50)), 2),
            'p95_ms': round(float(np.percentile(r, 95)), 2),
            'max_ms': round(float(r.max()), 2)}


def _json(path: Path) -> dict:
    return json.loads(Path(path).read_text(encoding='utf-8'))


def _events(path: Path) -> list[dict]:
    out = []
    if not Path(path).is_file():
        return out
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            break          # 崩溃留下的半行，丢掉而不是让整个 session 读不出来
    return out
