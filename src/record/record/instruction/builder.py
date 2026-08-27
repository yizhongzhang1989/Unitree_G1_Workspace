"""一个 round = 一次摆桌：选物品 -> 布局 -> 摆放样例 -> 指令序列。

**seed 显式写进结果**。对面的实现用模块级 ``random.seed(42)``，连续请求会推进同一个
全局 RNG，且 seed 不进 payload，导致「无法仅凭响应精确复现」（他们文档 §11.2 第 4 条）。
这里每个 round 自带 seed，给同样的 seed 和同样的物品组，布局与指令逐位可复现。
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field, replace

from record.instruction.generator import expand, simulate
from record.instruction.layout import layout_scene
from record.instruction.scene_svg import render_scene
from record.instruction.templates import Instruction, lint


@dataclass
class Round:
    index: int
    seed: int
    items: list
    placements: list
    svg: str
    instructions: list[Instruction]
    moves: list = field(default_factory=list)

    @property
    def vocabulary(self) -> set[str]:
        vocab = {'table'}
        vocab.update(i.name_en for i in self.items)
        return vocab

    def lint_report(self) -> list[tuple[int, str]]:
        """(episode 序号, 命中说明)。运行时只上报，不阻塞落盘。"""
        vocab = self.vocabulary
        return [(k, hit) for k, ins in enumerate(self.instructions)
                for hit in lint(ins.render_en(), vocab)]

    def as_dict(self) -> dict:
        return {
            'round': self.index,
            'seed': self.seed,
            'items': [{'item_id': i.item_id, 'en': i.name_en, 'zh': i.name_zh,
                       'role': i.role} for i in self.items],
            'layout': [p.as_dict() for p in self.placements],
            'episodes': [ins.as_dict() for ins in self.instructions],
            'lint_warnings': [{'episode': k, 'hit': h} for k, h in self.lint_report()],
        }


def build_round(library, geometry, index: int = 0, seed: int | None = None,
                n_items: int = 4, n_moves: int = 6, attempts: int = 8,
                items: list | None = None) -> Round:
    """生成一个完整的 round。

    摆不开的场景就**重摆**：可达域只有 0.198 m²，四件物里卡进两件大的就再也腾挪
    不开，状态机一步都走不出来（实测 200 轮里有 4 轮如此）。重摆用的是同一个 rng，
    所以给定 seed 仍然逐位可复现。

    传 ``items`` 就沿用这组物品，只重摆布局和指令 —— 换物品得起身去桌上换东西，
    只换摆放和指令则手边这几件挪一挪就行。
    """
    seed = random.randrange(2 ** 31) if seed is None else int(seed)
    rng = random.Random(seed)
    best = None
    for _ in range(max(1, attempts)):
        group = list(items) if items else library.choose_group(
            geometry, size=n_items, rng=rng)
        placements = layout_scene(group, geometry, rng=rng)
        # 状态机原地改坐标，给它副本，``placements`` 才一直是待会儿要画的那个初始局面
        moves, _ = simulate([replace(p) for p in placements], geometry,
                            n_moves=n_moves, rng=rng)
        if best is None or len(moves) > len(best[2]):
            best = (group, placements, moves)
        if len(moves) >= n_moves:
            break
    items, placements, moves = best
    return Round(index=index, seed=seed, items=items, placements=placements,
                 svg=render_scene(placements, geometry, title=f'Round {index}'),
                 instructions=expand(moves), moves=moves)
