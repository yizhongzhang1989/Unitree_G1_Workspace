"""指令生成的规格测试。

Spec §1.5 要求：发布前批量随机模拟，**模板类生成器的 lint 命中数必须为 0**。
``test_bulk_simulation_is_lint_clean`` 就是那道验收。
"""

import random

import pytest

from record.instruction.builder import build_round
from record.instruction.generator import RELATIVE, simulate
from record.instruction.layout import layout_scene
from record.instruction.library import LIBRARY_SUBDIR, ItemLibrary
from record.instruction.templates import (NAMED_POSITIONS, Instruction, join_steps,
                                          lint)
from record.table_geometry import load_geometry

pytestmark = pytest.mark.usefixtures('library')


@pytest.fixture(scope='session')
def geometry():
    return load_geometry()


@pytest.fixture(scope='session')
def library(tmp_path_factory):
    from pathlib import Path
    lib = Path(__file__).resolve().parents[1] / 'items' / LIBRARY_SUBDIR
    if not (lib / 'db' / 'item_library.db').is_file():
        pytest.skip('物品库不在包里')
    return ItemLibrary(lib, usage_path=tmp_path_factory.mktemp('usage') / 'u.json')


# ---------------------------------------------------------------- 模板与 lint

def test_pick_up_matches_spec():
    ins = Instruction('Pick up', 'right', 'silver can', '银色罐子')
    assert ins.render_en() == 'Pick up the silver can with the right arm'
    assert ins.render_zh() == '右手拿起银色罐子'
    assert lint(ins.render_en()) == []


def test_pass_matches_spec():
    ins = Instruction('Pass', 'left', 'pink cup', '粉色杯子', arm_to='right')
    assert ins.render_en() == ('Pass the pink cup to the right arm '
                               'with the left arm')
    assert ins.render_zh() == '左手把粉色杯子交给右手'
    assert lint(ins.render_en()) == []


@pytest.mark.parametrize('prep,expect', [
    ('into', 'Place the toy burger into the brown basket with the left arm'),
    ('on', 'Place the toy burger on the brown basket with the left arm'),
    ('left_of', 'Place the toy burger to the left of the brown basket with the left arm'),
    ('right_of', 'Place the toy burger to the right of the brown basket with the left arm'),
    ('in_front_of', 'Place the toy burger in front of the brown basket with the left arm'),
    ('behind', 'Place the toy burger behind the brown basket with the left arm'),
])
def test_place_branches_match_spec(prep, expect):
    ins = Instruction('Place', 'left', 'toy burger', '汉堡模型', prep=prep,
                      target_en='brown basket', target_zh='棕色篮子')
    assert ins.render_en() == expect
    assert lint(ins.render_en()) == []


def test_canonical_is_in_front_of_not_to_the_front_of():
    """对面实现文档 §9.3 写的是 to the front of，那恰好命中 lint 13，别跟着抄。"""
    good = Instruction('Place', 'left', 'a', 'a', prep='in_front_of',
                       target_en='b', target_zh='b').render_en()
    assert 'in front of' in good and lint(good) == []
    bad = good.replace('in front of the', 'to the front of the')
    assert any(h.startswith('13') for h in lint(bad))


@pytest.mark.parametrize('text,rule', [
    ('Pick up the ball with the right arm.', '1'),
    ('Pick up the ball using the right arm', '2'),
    ('Pick up the ball with the right hand', '3'),
    ('Place the held ball on the table with the left arm', '4'),
    ('Put down the ball with the left arm', '5'),
    ('Hand over the ball to the left arm with the right arm', '6'),
    ('Pour tea into the draining tray with the left arm', '7'),
    ('Place the cup on the tray with the left arm', '8'),
    ('Grab the ball with the right arm', '9'),
    ('Pass the ball to the left arm with the left arm', '10'),
    ('Pass the ball from the left arm to the right arm', '11'),
    ('Place the ball to the back of the box with the left arm', '13'),
    ('Place the ball at the middle of the table with the left arm', '14'),
    ('Place the ball in front of the table with the left arm', '14'),
])
def test_lint_catches_each_rule(text, rule):
    assert any(h.startswith(rule) for h in lint(text)), lint(text)


def test_lint_vocabulary_rule():
    text = 'Pick up the unicorn horn with the right arm'
    assert lint(text, {'silver can'})
    assert lint(text, {'unicorn horn'}) == []


def test_named_positions_whitelist_accepted():
    for pos in NAMED_POSITIONS:
        text = f'Place the spoon at the {pos} of the table with the left arm'
        assert not [h for h in lint(text) if h.startswith('14')]


def test_join_steps_uses_fullwidth_semicolon():
    steps = [Instruction('Pick up', 'left', 'a', 'a'),
             Instruction('Place', 'left', 'a', 'a', prep='on',
                         target_en='table', target_zh='桌面')]
    assert join_steps(steps).count('；') == 1


def test_as_dict_keeps_structured_fields_and_contract_string():
    ins = Instruction('Place', 'right', 'silver can', '银色罐子', 'itm_1',
                      prep='into', target_en='brown basket',
                      target_zh='棕色篮子', target_id='itm_2')
    d = ins.as_dict()
    assert d['instruction_en'].startswith('Place the silver can into')
    assert d['obj']['id'] == 'itm_1' and d['target']['id'] == 'itm_2'
    assert d['prep'] == 'into' and d['lint_warnings'] == []


# ---------------------------------------------------------------- 物品库

def test_library_loads_all_active_items(library):
    assert len(library.items) == 91
    assert not library.skipped, f'有 item 缺必需字段: {library.skipped}'
    assert all(i.name_en for i in library.items.values())


def test_usable_filters_by_reachability_mask(library, geometry):
    usable = library.usable(geometry)
    assert 60 <= len(usable) <= 91
    # 396x220 的锅短边 220 > 可达域能给的连续纵深余量，必须被排除
    assert not any(i.name_en == 'silver metal pot' for i in usable)


def test_usage_is_ours_not_theirs(library, tmp_path):
    library.bump_usage('itm_x', 'as_x')
    library.bump_usage('itm_x', 'as_x')
    assert library.usage('itm_x', 'as_x') == 2
    library.save_usage()
    assert library.usage_path.is_file()


# ---------------------------------------------------------------- 布局

def test_layout_respects_mask_and_spacing(library, geometry):
    rng = random.Random(3)
    items = library.choose_group(geometry, size=4, rng=rng)
    placed = layout_scene(items, geometry, rng=rng)
    assert placed
    for p in placed:
        w, d = p.item.footprint(p.pose)
        assert geometry.footprint_fits(p.cx, p.cy, w, d, p.rotation)
    from record.table_geometry import aabb_overlap
    for a in range(len(placed)):
        for b in range(a + 1, len(placed)):
            pa, pb = placed[a], placed[b]
            assert not aabb_overlap(pa.cx, pa.cy, *pa.extent,
                                    pb.cx, pb.cy, *pb.extent)


# ---------------------------------------------------------------- 状态机

def _round(library, geometry, seed):
    return build_round(library, geometry, index=0, seed=seed, n_items=4, n_moves=6)


def test_round_is_reproducible_from_seed(library, geometry):
    a = _round(library, geometry, 12345)
    b = _round(library, geometry, 12345)
    assert [i.render_en() for i in a.instructions] == [i.render_en() for i in b.instructions]
    assert [p.as_dict() for p in a.placements] == [p.as_dict() for p in b.placements]


def test_atomic_steps_are_pick_then_optional_pass_then_place(library, geometry):
    rnd = _round(library, geometry, 7)
    verbs = [i.verb for i in rnd.instructions]
    k = 0
    while k < len(verbs):
        total = rnd.instructions[k].step_total
        chunk = verbs[k:k + total]
        assert chunk[0] == 'Pick up' and chunk[-1] == 'Place'
        if total == 3:
            assert chunk[1] == 'Pass'
        assert total in (2, 3)
        k += total


def test_pass_only_when_crossing_midline(library, geometry):
    rng = random.Random(11)
    items = library.choose_group(geometry, size=4, rng=rng)
    placed = layout_scene(items, geometry, rng=rng)
    moves, _ = simulate(placed, geometry, n_moves=8, rng=rng)
    for m in moves:
        assert m.pick_hand == geometry.hand_of(m.pick_from[1])
        assert m.place_hand == geometry.hand_of(m.drop[1])
        assert m.passes == (m.pick_hand != m.place_hand)


def test_move_freezes_pick_position(library, geometry):
    """Move 持有活的 Placement，取件位置必须当场拷贝，否则会被后续步骤篡改。"""
    rng = random.Random(11)
    items = library.choose_group(geometry, size=4, rng=rng)
    placed = layout_scene(items, geometry, rng=rng)
    moves, _ = simulate(placed, geometry, n_moves=10, rng=rng)
    seen = {}
    for m in moves:
        key = m.obj.item.item_id
        if key in seen:
            assert m.pick_from == seen[key], '同一物品的第二次取件位置应等于上次落点'
        seen[key] = m.drop


def test_put_down_keeps_position(library, geometry):
    rng = random.Random(11)
    items = library.choose_group(geometry, size=4, rng=rng)
    placed = layout_scene(items, geometry, rng=rng)
    moves, _ = simulate(placed, geometry, n_moves=10, rng=rng)
    for m in moves:
        if m.action == 'put_down':
            assert m.drop == m.pick_from and not m.passes


def test_buried_items_are_never_picked_again(library, geometry):
    rng = random.Random(5)
    items = library.choose_group(geometry, size=5, rng=rng)
    placed = layout_scene(items, geometry, rng=rng)
    moves, state = simulate(placed, geometry, n_moves=12, rng=rng)
    buried_at = {}
    for k, m in enumerate(moves):
        if m.obj.item.item_id in buried_at:
            pytest.fail(f'第 {k} 步又搬动了已埋葬的 {m.obj.item.name_en}')
        if m.action == 'place_in':
            buried_at[m.obj.item.item_id] = k


def test_occupied_containers_are_never_moved(library, geometry):
    rng = random.Random(9)
    items = library.choose_group(geometry, size=5, rng=rng)
    placed = layout_scene(items, geometry, rng=rng)
    moves, _ = simulate(placed, geometry, n_moves=12, rng=rng)
    holding = set()
    for m in moves:
        assert m.obj.item.item_id not in holding, '搬动了装着东西的容器'
        if m.action in ('place_in', 'place_on') and m.target is not None:
            holding.add(m.target.item.item_id)


def test_relative_pair_never_reused(library, geometry):
    rng = random.Random(21)
    items = library.choose_group(geometry, size=5, rng=rng)
    placed = layout_scene(items, geometry, rng=rng)
    moves, _ = simulate(placed, geometry, n_moves=12, rng=rng)
    seen = set()
    for m in moves:
        if m.action in RELATIVE:
            pair = frozenset({m.obj.item.item_id, m.target.item.item_id})
            assert pair not in seen, '同一对物品重复做了相对放置'
            seen.add(pair)


def test_every_drop_stays_reachable_and_clear(library, geometry):
    """对面明说他们不做这一步（文档 §11.2 第 1 条），我们做了就得钉住。"""
    rng = random.Random(33)
    items = library.choose_group(geometry, size=5, rng=rng)
    placed = layout_scene(items, geometry, rng=rng)
    moves, _ = simulate(placed, geometry, n_moves=10, rng=rng)
    for m in moves:
        if m.action in RELATIVE:
            w, d = m.obj.item.footprint(m.obj.pose)
            assert geometry.footprint_fits(m.drop[0], m.drop[1], w, d,
                                           m.obj.rotation), '落点跑出可达域'


def test_place_in_only_after_burial_budget(library, geometry):
    rng = random.Random(45)
    items = library.choose_group(geometry, size=5, rng=rng)
    placed = layout_scene(items, geometry, rng=rng)
    n = 10
    moves, _ = simulate(placed, geometry, n_moves=n, rng=rng)
    for k, m in enumerate(moves):
        if m.action == 'place_in':
            assert k / n >= 0.4 - 1e-9, f'第 {k}/{n} 步就 place_in，早于埋葬预算'


def test_scene_svg_is_wellformed(library, geometry):
    import xml.etree.ElementTree as ET
    rnd = _round(library, geometry, 77)
    root = ET.fromstring(rnd.svg)
    assert root.tag.endswith('svg') and root.get('viewBox')


def test_round_dict_carries_seed_and_scene_binding(library, geometry):
    """他们的 on-disk schema 丢了场景归属和 seed，这条钉住我们没丢。"""
    d = _round(library, geometry, 99).as_dict()
    assert d['seed'] == 99 and d['round'] == 0
    assert d['layout'] and d['items']
    for k, ep in enumerate(d['episodes']):
        assert ep['instruction_en'] and 'step_index' in ep and 'step_total' in ep


# ------------------------------------------------- Spec §1.5 发布前批量验收

def test_bulk_simulation_is_lint_clean(library, geometry):
    """模板类生成器的合格条件是「N 次采样内 lint 命中数 = 0」，不是命中率阈值。"""
    total = 0
    for seed in range(120):
        rnd = build_round(library, geometry, index=seed, seed=seed,
                          n_items=4, n_moves=5)
        report = rnd.lint_report()
        assert not report, f'seed={seed} 命中 lint: {report[:3]}'
        total += len(rnd.instructions)
    assert total >= 1000, f'只生成了 {total} 条，样本不足以做发布前验收'
