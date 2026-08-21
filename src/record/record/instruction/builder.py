"""一个 round = 一次摆桌：选物品 -> 布局 -> 摆放样例 -> 指令序列。

**seed 显式写进结果**。对面的实现用模块级 ``random.seed(42)``，连续请求会推进同一个
全局 RNG，且 seed 不进 payload，导致「无法仅凭响应精确复现」（他们文档 §11.2 第 4 条）。
这里每个 round 自带 seed，给同样的 seed 和同样的物品组，布局与指令逐位可复现。
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field

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
                n_items: int = 4, n_moves: int = 6) -> Round:
    """生成一个完整的 round。

    渲染必须在 ``simulate`` **之前** —— 状态机会原地改 Placement 的坐标。
    """
    seed = random.randrange(2 ** 31) if seed is None else int(seed)
    rng = random.Random(seed)
    items = library.choose_group(geometry, size=n_items, rng=rng)
    placements = layout_scene(items, geometry, rng=rng)
    svg = render_scene(placements, geometry, title=f'Round {index}')
    moves, _ = simulate(placements, geometry, n_moves=n_moves, rng=rng)
    return Round(index=index, seed=seed, items=items, placements=placements,
                 svg=svg, instructions=expand(moves), moves=moves)
