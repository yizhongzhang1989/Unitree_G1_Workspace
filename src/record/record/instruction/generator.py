"""桌面 pick&place 指令状态机。

产出的是**原子动作**序列：一次搬运 = ``Pick up`` [+ ``Pass``] + ``Place``，每个原子动作
对应一条 episode。这是训练侧的粒度（上游 skeleton 计数逐项相加恰好等于 episode 数）。

保留对面状态机里真正有价值的不变量 —— 它们是「连着做十几步不出错」的前提：
  occupied     装了东西的容器/承载面不能再被搬动，否则内容物状态失真
  buried       放进容器的物品不再被抓出，避免从深容器底部掏
  used_sides   同一个 Y 的同方向槽位与镜像槽位不复用，防两件物品撞在一起
  used_pairs   同一对 {X,Y} 不重复相对放置，防 A→B→A 的链式漂移
  burial 预算  前 40% 不许 place_in，之后要保证剩余步骤还有货可搬

修掉他们文档 §11.2 自己承认的两个缺陷：
  * **drop 做全局碰撞检测**（他们只查关系约束，不保证不碰到第三个物品）
  * **动作阶段用当前 pose 的占地**（他们退回默认 footprint，多姿态长物体会误判）
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field

from record.instruction.layout import Placement, drop_offset
from record.instruction.templates import Instruction
from record.table_geometry import DEFAULT_CLEARANCE_M, aabb_overlap

#: 动作名义权重。场景越大越偏向 in/on，避免小桌面上全是相对放置。
BASE_WEIGHTS = {
    'place_in': 22, 'place_on': 15, 'left_of': 10, 'right_of': 10,
    'in_front_of': 8, 'behind': 8, 'put_down': 15,
}
RELATIVE = ('left_of', 'right_of', 'in_front_of', 'behind')
MIRROR = {'left_of': 'right_of', 'right_of': 'left_of',
          'in_front_of': 'behind', 'behind': 'in_front_of'}

#: 桌面本身作为 target。它不是库里的物品，但上游数据里 `table` 是最高频的 target。
TABLE_EN, TABLE_ZH = 'table', '桌面'

BURIAL_PROGRESS = 0.4          # 前 40% 的步骤不许 place_in
PLACE_IN_MARGIN = 0.020        # 容器每边要比物品大这么多才装得下
PLACE_ON_RATIO = 0.8           # 承载面至少要有物品的这个比例大


@dataclass
class Move:
    """一次搬运。展开成 2 或 3 条原子指令。

    ``pick_from`` 必须在构造时就拷贝：``obj`` 是活的 ``Placement``，后续步骤
    再搬动同一件物品时会原地改它的坐标，持有引用就会把早先那一步的取件位置篡改。
    """

    action: str
    obj: Placement
    target: Placement | None
    pick_from: tuple[float, float]
    pick_hand: str
    place_hand: str
    drop: tuple[float, float]

    @property
    def passes(self) -> bool:
        return self.pick_hand != self.place_hand

    def to_instructions(self) -> list[Instruction]:
        steps: list[Instruction] = []
        o = self.obj.item
        steps.append(Instruction('Pick up', self.pick_hand, o.name_en, o.name_zh,
                                 o.item_id))
        if self.passes:
            steps.append(Instruction('Pass', self.pick_hand, o.name_en, o.name_zh,
                                     o.item_id, arm_to=self.place_hand))
        if self.action == 'put_down':
            prep, ten, tzh, tid = 'on', TABLE_EN, TABLE_ZH, ''
        elif self.action == 'place_in':
            prep, ten, tzh, tid = ('into', self.target.item.name_en,
                                   self.target.item.name_zh, self.target.item.item_id)
        elif self.action == 'place_on':
            prep, ten, tzh, tid = ('on', self.target.item.name_en,
                                   self.target.item.name_zh, self.target.item.item_id)
        else:
            prep, ten, tzh, tid = (self.action, self.target.item.name_en,
                                   self.target.item.name_zh, self.target.item.item_id)
        steps.append(Instruction('Place', self.place_hand, o.name_en, o.name_zh,
                                 o.item_id, prep=prep, target_en=ten,
                                 target_zh=tzh, target_id=tid))
        total = len(steps)
        return [Instruction(**{**s.__dict__, 'step_index': i, 'step_total': total})
                for i, s in enumerate(steps)]


@dataclass
class SceneState:
    placements: list[Placement]
    occupied: dict[str, list[str]] = field(default_factory=dict)
    buried: set[str] = field(default_factory=set)
    used_sides: set[tuple[str, str]] = field(default_factory=set)
    used_pairs: set[frozenset] = field(default_factory=set)

    def by_id(self, item_id: str) -> Placement | None:
        return next((p for p in self.placements if p.item.item_id == item_id), None)

    def pickable(self) -> list[Placement]:
        return [p for p in self.placements
                if p.item.portable
                and p.item.item_id not in self.buried
                and not self.occupied.get(p.item.item_id)]


def _fits_in(target: Placement, obj: Placement) -> bool:
    tw, td = target.item.footprint(target.pose)
    ow, od = obj.item.footprint(obj.pose)
    return tw + PLACE_IN_MARGIN > ow and td + PLACE_IN_MARGIN > od


def _fits_on(target: Placement, obj: Placement) -> bool:
    tw, td = target.item.footprint(target.pose)
    ow, od = obj.item.footprint(obj.pose)
    return tw > ow * PLACE_ON_RATIO and td > od * PLACE_ON_RATIO


def _drop_is_clear(state: SceneState, obj: Placement, cx: float, cy: float,
                   geometry, clearance: float, ignore: set[str]) -> bool:
    """落点必须在可达域内，且不碰到任何无关物品。

    对面明确说他们不做这一步（文档 §11.2 第 1 条），于是能防「两件物品占同一个
    Y-side」却防不住撞上第三件。我们有掩码和 placement 列表，顺手就能检。
    """
    w, d = obj.item.footprint(obj.pose)
    if not geometry.footprint_fits(cx, cy, w, d, obj.rotation, clearance):
        return False
    bw, bd = obj.extent
    for other in state.placements:
        if other.item.item_id in ignore:
            continue
        if aabb_overlap(cx, cy, bw, bd, other.cx, other.cy, *other.extent,
                        clearance=clearance):
            return False
    return True


def _candidates(state: SceneState, obj: Placement, action: str, geometry,
                clearance: float) -> list[tuple[Placement | None, tuple[float, float]]]:
    out = []
    if action == 'put_down':
        return [(None, (obj.cx, obj.cy))]
    for tgt in state.placements:
        if tgt.item.item_id == obj.item.item_id:
            continue
        if action == 'place_in':
            if not tgt.item.is_container or not _fits_in(tgt, obj):
                continue
            out.append((tgt, (tgt.cx, tgt.cy)))
        elif action == 'place_on':
            if not tgt.item.is_surface or not _fits_on(tgt, obj):
                continue
            out.append((tgt, (tgt.cx, tgt.cy)))
        else:
            key = (tgt.item.item_id, action)
            mirror = (obj.item.item_id, MIRROR[action])
            pair = frozenset({obj.item.item_id, tgt.item.item_id})
            if key in state.used_sides or mirror in state.used_sides \
                    or pair in state.used_pairs:
                continue
            cx, cy = drop_offset(tgt, obj.item, obj.pose, action, clearance)
            if not _drop_is_clear(state, obj, cx, cy, geometry, clearance,
                                  ignore={obj.item.item_id}):
                continue
            out.append((tgt, (cx, cy)))
    return out


def _weights(state: SceneState, step: int, total: int, n_items: int) -> dict:
    w = dict(BASE_WEIGHTS)
    scale = min(1 + 0.3 * max(0, n_items - 5), 2.2)
    w['place_in'] *= scale
    w['place_on'] *= scale
    progress = step / max(total, 1)
    remaining = total - step
    pickable = len(state.pickable())
    if progress < BURIAL_PROGRESS or (pickable - 1) < (remaining - 1):
        w['place_in'] = 0.0
    return {k: v for k, v in w.items() if v > 0}


def simulate(placements, geometry, n_moves: int = 6,
             rng: random.Random | None = None,
             clearance: float = DEFAULT_CLEARANCE_M) -> tuple[list[Move], SceneState]:
    """跑状态机，返回搬运序列与终态。

    **必须在渲染摆放样例之后调用**：状态机会原地改 Placement 的坐标，先跑它再渲染
    画出来的是十几步之后的混合终态，操作者照着摆会摆出一个从没存在过的场景。
    """
    rng = rng or random.Random()
    state = SceneState(placements=list(placements))
    moves: list[Move] = []

    for step in range(n_moves):
        pool = state.pickable()
        if not pool:
            break
        weights = _weights(state, step, n_moves, len(state.placements))
        chosen = None
        for _ in range(20):
            obj = rng.choice(pool)
            action = rng.choices(list(weights), weights=list(weights.values()))[0]
            cands = _candidates(state, obj, action, geometry, clearance)
            if cands:
                chosen = (obj, action, rng.choice(cands))
                break
        if chosen is None:
            obj = rng.choice(pool)
            chosen = (obj, 'put_down', (None, (obj.cx, obj.cy)))
        obj, action, (target, drop) = chosen

        pick_hand = geometry.hand_of(obj.cy)
        place_hand = geometry.hand_of(drop[1])
        moves.append(Move(action, obj, target, (obj.cx, obj.cy),
                          pick_hand, place_hand, drop))

        # 物品一旦移动，所有以它为参照的槽位失效，否则会永久锁死一批合法动作
        state.used_sides = {k for k in state.used_sides if k[0] != obj.item.item_id}
        if action in RELATIVE:
            state.used_sides.add((target.item.item_id, action))
            state.used_pairs.add(frozenset({obj.item.item_id, target.item.item_id}))
        elif action in ('place_in', 'place_on'):
            state.occupied.setdefault(target.item.item_id, []).append(obj.item.item_id)
            if action == 'place_in':
                state.buried.add(obj.item.item_id)
        obj.cx, obj.cy = drop
    return moves, state


def expand(moves) -> list[Instruction]:
    """搬运序列 -> 扁平的原子指令表。一条 = 一个 episode。"""
    return [ins for m in moves for ins in m.to_instructions()]
