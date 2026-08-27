"""桌面 pick&place 指令状态机。

产出的是**原子动作**序列：一次搬运 = ``Pick up`` [+ ``Pass``] + ``Place``，每个原子动作
对应一条 episode。这是训练侧的粒度（上游 skeleton 计数逐项相加恰好等于 episode 数）。

保留对面状态机里真正有价值的不变量 —— 它们是「连着做十几步不出错」的前提：
  occupied     装了东西的容器/承载面不能再被搬动，否则内容物状态失真
  buried       放进容器的物品不再被抓出，避免从深容器底部掏
  used_sides   同一个 Y 的同方向槽位与镜像槽位不复用，防两件物品撞在一起
  used_pairs   同一对 {X,Y} 不重复相对放置，防 A→B→A 的链式漂移
  burial 预算  前 40% 不许 place_in，之后埋一件也得给后面的步骤留下货

再加三条自己的，它们决定了一个 round 里指令**不会重样**：
  真搬运       每一步的落点必须离取件点至少 ``MIN_TRAVEL_M``
  不连搬       不连着两步搬同一件，只剩它一个能搬时宁可收工
  used_moves   同一个 ``(物品, 动作, 目标)`` 一轮只能出一次 —— 指令文本就是它决定的

没合法动作就**收工**，不拿无效步骤凑数；凑不够步数由 ``build_round`` 重摆场景。

修掉他们文档 §11.2 自己承认的两个缺陷：
  * **drop 做全局碰撞检测**（他们只查关系约束，不保证不碰到第三个物品）
  * **动作阶段用当前 pose 的占地**（他们退回默认 footprint，多姿态长物体会误判）
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from record.instruction.layout import Placement, drop_offset
from record.instruction.templates import Instruction
from record.table_geometry import DEFAULT_CLEARANCE_M, aabb_overlap

#: 动作名义权重。场景越大越偏向 in/on，避免小桌面上全是相对放置。
#: ``put_down`` 压得很低但实测占比仍有 ~44%：可达域只有 0.198 m²，相对放置常常
#: 没地方落，只有「放到桌面另一处」永远有解，它天然是兜底。
BASE_WEIGHTS = {
    'place_in': 22, 'place_on': 15, 'left_of': 10, 'right_of': 10,
    'in_front_of': 8, 'behind': 8, 'put_down': 6,
}
RELATIVE = ('left_of', 'right_of', 'in_front_of', 'behind')
MIRROR = {'left_of': 'right_of', 'right_of': 'left_of',
          'in_front_of': 'behind', 'behind': 'in_front_of'}
#: 动作 -> Place 的介词。相对放置的介词就是动作名本身。
PREP = {'put_down': 'on', 'place_in': 'into', 'place_on': 'on'}

#: 桌面本身作为 target。它不是库里的物品，但上游数据里 `table` 是最高频的 target。
TABLE_EN, TABLE_ZH = 'table', '桌面'

BURIAL_PROGRESS = 0.4          # 前 40% 的步骤不许 place_in
KEEP_PICKABLE = 2              # 埋一件之后至少还得剩这么多可搬的
PLACE_IN_MARGIN = 0.020        # 容器每边要比物品大这么多才装得下
PLACE_ON_RATIO = 0.8           # 承载面至少要有物品的这个比例大

#: 落点至少要离取件点这么远（两格）。再往上调会让一大半四件物场景一步都走
#: 不出来（实测 0.10 m 时 200 轮里 20 轮空）。
MIN_TRAVEL_M = 0.05
SPOT_TRIES = 60                # 从可放格心里随机试这么多个
SPOT_KEEP = 12                 # 收够这么多个落点就不再试
SELECT_TRIES = 20              # 按权重随机抽这么多次，不中才去全排


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
        o, t = self.obj.item, self.target
        steps = [Instruction('Pick up', self.pick_hand, o.name_en, o.name_zh,
                             o.item_id)]
        if self.passes:
            steps.append(Instruction('Pass', self.pick_hand, o.name_en, o.name_zh,
                                     o.item_id, arm_to=self.place_hand))
        ten, tzh, tid = ((TABLE_EN, TABLE_ZH, '') if t is None else
                         (t.item.name_en, t.item.name_zh, t.item.item_id))
        steps.append(Instruction('Place', self.place_hand, o.name_en, o.name_zh,
                                 o.item_id, prep=PREP.get(self.action, self.action),
                                 target_en=ten, target_zh=tzh, target_id=tid))
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
    used_moves: set[tuple[str, str, str]] = field(default_factory=set)

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
                   geometry, clearance: float) -> bool:
    """落点必须在可达域内，且不碰到除自己以外的任何物品。

    对面明确说他们不做这一步（文档 §11.2 第 1 条），于是能防「两件物品占同一个
    Y-side」却防不住撞上第三件。我们有掩码和 placement 列表，顺手就能检。
    """
    w, d = obj.item.footprint(obj.pose)
    if not geometry.footprint_fits(cx, cy, w, d, obj.rotation, clearance):
        return False
    bw, bd = obj.extent
    return not any(
        aabb_overlap(cx, cy, bw, bd, o.cx, o.cy, *o.extent, clearance=clearance)
        for o in state.placements if o.item.item_id != obj.item.item_id)


def _travelled(obj: Placement, drop: tuple[float, float]) -> bool:
    return math.hypot(drop[0] - obj.cx, drop[1] - obj.cy) >= MIN_TRAVEL_M


def _free_spots(state: SceneState, obj: Placement, geometry, clearance: float,
                rng: random.Random) -> list[tuple[float, float]]:
    """桌面上放得下这件物品、又离它现在的位置足够远的落点。

    逐格碰撞检测要遍历全部 placement，而这函数每步会被问上二十次 —— 所以随机抽样试、
    收够 ``SPOT_KEEP`` 个就停，和 ``layout_scene`` 一个路数。
    """
    w, d = obj.item.footprint(obj.pose)
    centres = geometry.candidate_centres(w, d, obj.rotation, clearance)
    out: list[tuple[float, float]] = []
    for k in rng.sample(range(len(centres)), min(SPOT_TRIES, len(centres))):
        drop = (float(centres[k][0]), float(centres[k][1]))
        if _travelled(obj, drop) \
                and _drop_is_clear(state, obj, *drop, geometry, clearance):
            out.append(drop)
            if len(out) >= SPOT_KEEP:
                break
    return out


def _candidates(state: SceneState, obj: Placement, action: str, geometry,
                clearance: float,
                rng: random.Random) -> list[tuple[Placement | None, tuple[float, float]]]:
    if action == 'put_down':
        return [(None, s) for s in _free_spots(state, obj, geometry, clearance, rng)]
    out = []
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
            if not _drop_is_clear(state, obj, cx, cy, geometry, clearance):
                continue
            out.append((tgt, (cx, cy)))
    return out


def _signature(obj: Placement, action: str, target: Placement | None) -> tuple:
    return (obj.item.item_id, action, target.item.item_id if target else '')


def _pick_candidate(state: SceneState, obj: Placement, action: str, geometry,
                    clearance: float, rng: random.Random):
    """挑一个本轮还没用过、且真能把物品挪开的落点。返回 ``(target, drop)`` 或 ``None``。

    去重粒度是 ``(物品, 动作, 目标)``，因为指令文本正是这三者决定的。距离那一关
    相对放置也要把：它也会算出物品当前所在的格子（实测 200 轮里 5 次）。
    """
    cands = [c for c in _candidates(state, obj, action, geometry, clearance, rng)
             if _travelled(obj, c[1])
             and _signature(obj, action, c[0]) not in state.used_moves]
    return rng.choice(cands) if cands else None


def _weights(state: SceneState, step: int, total: int, n_items: int) -> dict:
    w = dict(BASE_WEIGHTS)
    scale = min(1 + 0.3 * max(0, n_items - 5), 2.2)
    w['place_in'] *= scale
    w['place_on'] *= scale
    progress = step / max(total, 1)
    pickable = len(state.pickable())
    if progress < BURIAL_PROGRESS or pickable - 1 < KEEP_PICKABLE:
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
    last_id = ''

    for step in range(n_moves):
        # 连着两步搬同一件就是屏幕上连着两行同样的 Pick up。只剩它一个能搬时宁可收工。
        pool = [p for p in state.pickable() if p.item.item_id != last_id]
        if not pool:
            break
        weights = _weights(state, step, n_moves, len(state.placements))
        acts, wts = list(weights), list(weights.values())
        # 先按权重随机试，不中再把 (物品 x 动作) 全排一遍；仍不中就收工。老版本在这里
        # 退回「原地放回」，于是同一条指令能连出五六遍。
        probe = [(rng.choice(pool), rng.choices(acts, wts)[0])
                 for _ in range(SELECT_TRIES)]
        probe += [(o, a) for o in rng.sample(pool, len(pool))
                  for a in rng.sample(acts, len(acts))]
        chosen = next(((o, a, c) for o, a in probe
                       if (c := _pick_candidate(state, o, a, geometry,
                                                clearance, rng))), None)
        if chosen is None:
            break
        obj, action, (target, drop) = chosen

        pick_hand = geometry.hand_of(obj.cy)
        place_hand = geometry.hand_of(drop[1])
        moves.append(Move(action, obj, target, (obj.cx, obj.cy),
                          pick_hand, place_hand, drop))

        # 物品一旦移动，所有以它为参照的槽位失效，否则会永久锁死一批合法动作
        state.used_sides = {k for k in state.used_sides if k[0] != obj.item.item_id}
        state.used_moves.add(_signature(obj, action, target))
        if action in RELATIVE:
            state.used_sides.add((target.item.item_id, action))
            state.used_pairs.add(frozenset({obj.item.item_id, target.item.item_id}))
        elif action in ('place_in', 'place_on'):
            state.occupied.setdefault(target.item.item_id, []).append(obj.item.item_id)
            if action == 'place_in':
                state.buried.add(obj.item.item_id)
        obj.cx, obj.cy = drop
        last_id = obj.item.item_id
    return moves, state


def expand(moves) -> list[Instruction]:
    """搬运序列 -> 扁平的原子指令表。一条 = 一个 episode。"""
    return [ins for m in moves for ins in m.to_instructions()]
