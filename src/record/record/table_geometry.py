"""桌面几何：可达掩码 + 坐标约定。

坐标一律用机器人 ``torso_link`` 系，不引入他们那套画布坐标：
    +x = 纵深，远离机器人为正（他们的 -y）
    +y = 横向，机器人左手侧为正（他们的 -x）
手区按 ``y`` 的符号分，不是他们的 ``x=600``。

掩码由 `record/data/reach_mask.npz` 提供，生成条件见文件内 ``meta``：
腰锁死、工具轴垂直向下 30° 内、含自碰撞过滤、逐格 IK 热启动洪水填充。
详见 /memories/repo/g1-table-workspace.md。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np

_DATA = Path(__file__).with_name('data') / 'reach_mask.npz'

#: 物品之间、物品与可达边界之间的最小间距。沿用对面的 30 mm。
DEFAULT_CLEARANCE_M = 0.030


@dataclass(frozen=True)
class TableGeometry:
    """桌面可达性查询。所有长度单位是米，角度是弧度。"""

    xs: np.ndarray            # 格心的 x 坐标（升序）
    ys: np.ndarray            # 格心的 y 坐标（升序）
    left: np.ndarray          # (nx, ny) bool，左手可达
    right: np.ndarray
    cell: float
    meta: dict

    @property
    def reachable(self) -> np.ndarray:
        return self.left | self.right

    @property
    def both(self) -> np.ndarray:
        return self.left & self.right

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        """可达区外接矩形 ``(x_min, x_max, y_min, y_max)``，含格子半宽。"""
        rows = np.where(self.reachable.any(axis=1))[0]
        cols = np.where(self.reachable.any(axis=0))[0]
        h = self.cell / 2
        return (float(self.xs[rows[0]] - h), float(self.xs[rows[-1]] + h),
                float(self.ys[cols[0]] - h), float(self.ys[cols[-1]] + h))

    def hand_of(self, y: float) -> str:
        """哪只手负责这个位置。中线两侧硬分，没有模糊带。"""
        return 'left' if y >= 0.0 else 'right'

    def _cell_index(self, x: float, y: float) -> tuple[int, int] | None:
        i = int(round((x - self.xs[0]) / self.cell))
        j = int(round((y - self.ys[0]) / self.cell))
        if 0 <= i < len(self.xs) and 0 <= j < len(self.ys):
            return i, j
        return None

    def is_reachable(self, x: float, y: float, hand: str | None = None) -> bool:
        idx = self._cell_index(x, y)
        if idx is None:
            return False
        i, j = idx
        if hand == 'left':
            return bool(self.left[i, j])
        if hand == 'right':
            return bool(self.right[i, j])
        return bool(self.reachable[i, j])

    def footprint_fits(self, cx: float, cy: float, w: float, d: float,
                       rotation: float = 0.0,
                       clearance: float = DEFAULT_CLEARANCE_M) -> bool:
        """物品放在 (cx,cy) 时，它旋转后的包围盒是否整个落在可达格里。

        必须对**掩码**判而不是对外接矩形判：可达区是个「香蕉形」，用矩形近似会把
        220 mm 的盘子整个判死（实测它其实还有 5 个可放位置）。
        """
        bw, bd = rotated_extent(w, d, rotation)
        half_w, half_d = bw / 2 + clearance, bd / 2 + clearance
        i0 = int(math.floor((cx - half_d - self.xs[0]) / self.cell + 0.5))
        i1 = int(math.ceil((cx + half_d - self.xs[0]) / self.cell - 0.5))
        j0 = int(math.floor((cy - half_w - self.ys[0]) / self.cell + 0.5))
        j1 = int(math.ceil((cy + half_w - self.ys[0]) / self.cell - 0.5))
        if i0 < 0 or j0 < 0 or i1 >= len(self.xs) or j1 >= len(self.ys):
            return False
        return bool(self.reachable[i0:i1 + 1, j0:j1 + 1].all())

    def candidate_centres(self, w: float, d: float, rotation: float = 0.0,
                          clearance: float = DEFAULT_CLEARANCE_M) -> np.ndarray:
        """所有能放下这个尺寸的格心，形状 (n, 2)。布局采样从这里挑，不用盲试。"""
        out = [(float(x), float(y))
               for x in self.xs for y in self.ys
               if self.footprint_fits(x, y, w, d, rotation, clearance)]
        return np.array(out, dtype=float).reshape(-1, 2)


def rotated_extent(w: float, d: float, rotation: float) -> tuple[float, float]:
    """旋转后的轴对齐包围盒尺寸。``w`` 沿 y（横向），``d`` 沿 x（纵深）。"""
    c, s = abs(math.cos(rotation)), abs(math.sin(rotation))
    return w * c + d * s, w * s + d * c


def aabb_overlap(ax: float, ay: float, aw: float, ad: float,
                 bx: float, by: float, bw: float, bd: float,
                 clearance: float = DEFAULT_CLEARANCE_M) -> bool:
    """两个轴对齐包围盒是否重叠（含间距）。两轴都相交才算碰撞。"""
    return (abs(ax - bx) < (ad + bd) / 2 + clearance
            and abs(ay - by) < (aw + bw) / 2 + clearance)


@lru_cache(maxsize=1)
def load_geometry(path: str | None = None) -> TableGeometry:
    src = Path(path) if path else _DATA
    with np.load(src, allow_pickle=False) as z:
        xs, ys = z['xs'], z['ys']
        meta = json.loads(str(z['meta']))
        return TableGeometry(xs=xs, ys=ys, left=z['left'], right=z['right'],
                             cell=float(xs[1] - xs[0]), meta=meta)
