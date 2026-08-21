"""初始布局：在可达域内摆下一组物品，两两不重叠。

结果会渲染成摆放样例图给操作者照着摆，所以必须是**物理可实现**的：
每件物品旋转后的包围盒既要落在可达格里，又要和已放物品保持间距。

和对面的做法有一处关键不同：他们在 1200x600 的整块桌面上盲目 rejection sampling，
我们的可达域只有他们的 1/4 且是「香蕉形」，盲试命中率太低 —— 这里先用
``TableGeometry.candidate_centres`` 把可放格心预筛出来，再在其中采样。
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from record.table_geometry import DEFAULT_CLEARANCE_M, aabb_overlap, rotated_extent

#: 近正方形只试 4 个角度，避免在等价姿态上浪费采样。
SQUARE_TOL_M = 0.005
QUARTER_TURNS = (0.0, math.pi / 2, math.pi, 3 * math.pi / 2)


@dataclass
class Placement:
    """一件物品在桌面上的位置。``cx`` 沿纵深，``cy`` 沿横向，都在 torso_link 系。"""

    item: object
    pose: str
    cx: float
    cy: float
    rotation: float

    @property
    def extent(self) -> tuple[float, float]:
        w, d = self.item.footprint(self.pose)
        return rotated_extent(w, d, self.rotation)

    def as_dict(self) -> dict:
        w, d = self.extent
        return {
            'item_id': self.item.item_id, 'en': self.item.name_en,
            'zh': self.item.name_zh, 'role': self.item.role, 'pose': self.pose,
            'cx': round(self.cx, 4), 'cy': round(self.cy, 4),
            'rotation_deg': round(math.degrees(self.rotation), 1),
            'bbox_w': round(w, 4), 'bbox_d': round(d, 4),
        }


class LayoutError(RuntimeError):
    pass


def _rotations(w: float, d: float, rng: random.Random, tries: int) -> list[float]:
    if abs(w - d) <= SQUARE_TOL_M:
        return list(QUARTER_TURNS)
    return [rng.uniform(0.0, 2 * math.pi) for _ in range(tries)]


def layout_scene(items, geometry, rng: random.Random | None = None,
                 clearance: float = DEFAULT_CLEARANCE_M,
                 rotation_tries: int = 12,
                 centre_tries: int = 60) -> list[Placement]:
    """按占地从大到小摆放，返回全部成功放下的物品。

    放不下的物品**丢弃而不是允许重叠** —— 摆放样例一旦画出重叠的物品，操作者照着摆
    就会摆出一个物理上不存在的场景，后面所有指令都跟着错。
    """
    rng = rng or random.Random()
    ordered = sorted(items, key=lambda it: -(it.width * it.depth))
    placed: list[Placement] = []

    for item in ordered:
        poses = list(item.poses) or ['single']
        rng.shuffle(poses)
        spot = None
        for pose in poses:
            w, d = item.footprint(pose)
            for rot in _rotations(w, d, rng, rotation_tries):
                centres = geometry.candidate_centres(w, d, rot, clearance)
                if not len(centres):
                    continue
                order = rng.sample(range(len(centres)),
                                   min(centre_tries, len(centres)))
                bw, bd = rotated_extent(w, d, rot)
                for k in order:
                    cx, cy = float(centres[k][0]), float(centres[k][1])
                    if any(aabb_overlap(cx, cy, bw, bd, p.cx, p.cy, *p.extent,
                                        clearance=clearance) for p in placed):
                        continue
                    spot = Placement(item, pose, cx, cy, rot)
                    break
                if spot:
                    break
            if spot:
                break
        if spot:
            placed.append(spot)
    if not placed:
        raise LayoutError('一件都没放下：可达域或间距设置有问题')
    return placed


def drop_offset(anchor: Placement, item, pose: str, prep: str,
                clearance: float = DEFAULT_CLEARANCE_M) -> tuple[float, float]:
    """相对放置时落点的中心。``in_front_of`` 是靠近机器人，即 x 减小。"""
    aw, ad = anchor.extent
    w, d = item.footprint(pose)
    dx = (ad + d) / 2 + clearance
    dy = (aw + w) / 2 + clearance
    return {
        'left_of': (anchor.cx, anchor.cy + dy),
        'right_of': (anchor.cx, anchor.cy - dy),
        'in_front_of': (anchor.cx - dx, anchor.cy),
        'behind': (anchor.cx + dx, anchor.cy),
    }[prep]
