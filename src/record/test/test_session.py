"""session 状态机、事件线与封口的契约。"""

import json

import pytest

from record.session import (Session, SessionError, SessionPaths, State,
                            read_events, write_done)

STREAMS = {'joint_states': True, 'head': True, 'head_depth': False}


@pytest.fixture
def session(tmp_path, calibration):
    s = Session.create(tmp_path, STREAMS, meta={'note': 'x'},
                       calibration=calibration)
    yield s
    if s.state is not State.IDLE:
        s.finish({})


def test_layout_created(session):
    p = session.paths
    for d in (p.root, p.video, p.signals, p.rounds):
        assert d.is_dir()
    assert p.manifest.is_file() and p.meta.is_file()


def test_manifest_freezes_stream_selection(session):
    m = json.loads(session.paths.manifest.read_text(encoding='utf-8'))
    assert m['streams'] == {'joint_states': True, 'head': True, 'head_depth': False}
    assert m['schema_version'] == 1 and m['session_id'] == session.session_id


def test_refuses_reuse_of_nonempty_dir(tmp_path):
    s = Session.create(tmp_path, STREAMS, session_id='dup')
    s.finish({})
    with pytest.raises(SessionError, match='已存在'):
        Session.create(tmp_path, STREAMS, session_id='dup')


def test_state_machine_order(session):
    assert session.state is State.SESSION
    with pytest.raises(SessionError, match='要先开 round'):
        session.start_episode({'instruction_en': 'x'})
    session.start_round({'seed': 1, 'items': [], 'episodes': [{}]})
    assert session.state is State.ROUND
    with pytest.raises(SessionError, match='不能开 round'):
        session.start_round({'seed': 2, 'items': [], 'episodes': []})
    session.start_episode({'instruction_en': 'Pick up the a with the left arm'})
    assert session.state is State.EPISODE
    session.end_episode('success')
    assert session.state is State.ROUND
    session.end_round()
    assert session.state is State.SESSION


def test_unknown_outcome_rejected(session):
    session.start_round({'seed': 1, 'items': [], 'episodes': [{}]})
    session.start_episode({})
    with pytest.raises(SessionError, match='未知标注'):
        session.end_episode('maybe')


def test_counts_exclude_discarded(session):
    session.start_round({'seed': 1, 'items': [], 'episodes': [{}]})
    for outcome in ('success', 'success', 'fail', 'discard'):
        session.start_episode({})
        session.end_episode(outcome)
    assert session.counts == {'rounds': 1, 'episodes': 3, 'success': 2,
                              'fail': 1, 'discard': 1}


def test_events_are_the_only_timeline(session):
    session.start_round({'seed': 42, 'items': [{'item_id': 'i1'}],
                         'episodes': [{'instruction_en': 'a'}]})
    session.start_episode({'instruction_en': 'a', 'step_index': 0})
    session.end_episode('success', note='顺利')
    session.end_round()
    session.finish({})
    kinds = [e['type'] for e in read_events(session.paths.events)]
    assert kinds == ['session_start', 'round_start', 'episode_start',
                     'episode_end', 'round_end', 'session_end']
    events = read_events(session.paths.events)
    assert events[1]['seed'] == 42 and events[1]['items'] == ['i1']
    assert events[3]['outcome'] == 'success' and events[3]['duration'] >= 0
    assert all(e['t'] > 0 for e in events)


def test_round_json_and_svg_written(session):
    session.start_round({'seed': 7, 'items': [], 'episodes': []}, svg='<svg/>')
    f = session.paths.rounds / 'round_000.json'
    assert json.loads(f.read_text(encoding='utf-8'))['seed'] == 7
    assert (session.paths.rounds / 'round_000.svg').read_text(encoding='utf-8') == '<svg/>'


def test_warning_never_blocks(session):
    """Spec §1.5：运行时 lint 命中只记录，不阻塞落盘。"""
    session.start_round({'seed': 1, 'items': [], 'episodes': [{}]})
    session.warn('lint', episode=0, hit='13 应使用 canonical')
    session.start_episode({})
    session.end_episode('success')
    kinds = [e['type'] for e in read_events(session.paths.events)]
    assert 'warning' in kinds and 'episode_end' in kinds


def test_episode_index_counts_takes_not_slots(session):
    """`episode_index` 是「本轮第几次录」，**不是指令表的下标**。

    两者只在「一条一次、按顺序走」时碰巧相等，重录一次就永久分家。拿它当
    「面板该高亮哪一行」用过一次：重录之后没有一行匹配得上，于是成功/失败/丢弃
    和开始/重录**两组按钮同时消失**，点了像没反应。现在面板改看 `episode_slot`。
    """
    session.start_round({'seed': 1, 'items': [], 'episodes': [{}, {}]})
    for _ in range(2):
        session.start_episode({})
        session.end_episode('success')
    session.start_episode({})                 # 重录第 0 条
    assert session.episode_index == 2         # 指令表只有 2 条，下标不可能是 2


def test_relabel_appends_instead_of_rewriting(session):
    """改标注只往事件线追加，原来那行 `episode_end` 原样留着。"""
    session.start_round({'seed': 1, 'items': [], 'episodes': [{}]})
    session.start_episode({})
    session.end_episode('success')
    assert session.relabel_episode(0, 'fail') == 'success'
    events = read_events(session.paths.events)
    assert [e['type'] for e in events][-2:] == ['episode_end', 'episode_relabel']
    assert events[-2]['outcome'] == 'success'
    assert events[-1]['outcome'] == 'fail' and events[-1]['was'] == 'success'


def test_relabel_moves_the_counts(session):
    """进出「丢弃」那一档要连总数一起改 —— discard 不算交付的 episode。"""
    session.start_round({'seed': 1, 'items': [], 'episodes': [{}]})
    session.start_episode({})
    session.end_episode('success')
    session.relabel_episode(0, 'discard')
    assert session.counts == {'rounds': 1, 'episodes': 0, 'success': 0,
                              'fail': 0, 'discard': 1}
    session.relabel_episode(0, 'fail')
    assert session.counts == {'rounds': 1, 'episodes': 1, 'success': 0,
                              'fail': 1, 'discard': 0}
    # 改成一样的不该再写一行，否则点两下就多一条假记录
    session.relabel_episode(0, 'fail')
    assert sum(e['type'] == 'episode_relabel'
               for e in read_events(session.paths.events)) == 2


def test_relabel_rejects_unknown_episode(session):
    session.start_round({'seed': 1, 'items': [], 'episodes': [{}]})
    session.start_episode({})
    with pytest.raises(SessionError, match='改不了标注'):
        session.relabel_episode(0, 'fail')          # 还没录完
    with pytest.raises(SessionError, match='改不了标注'):
        session.relabel_episode(7, 'fail')


def test_finish_closes_dangling_episode(session):
    session.start_round({'seed': 1, 'items': [], 'episodes': [{}]})
    session.start_episode({})
    session.finish({})
    events = read_events(session.paths.events)
    end = next(e for e in events if e['type'] == 'episode_end')
    assert end['outcome'] == 'discard'
    assert events[-1]['type'] == 'session_end'


def test_done_lists_every_file_with_digest(session):
    session.start_round({'seed': 1, 'items': [], 'episodes': []})
    (session.paths.signals / 'a.bin').write_bytes(b'\x00' * 24)
    digest = session.finish({'a': {'file': 'a.bin', 'ncol': 3}})
    payload = json.loads(session.paths.done.read_text(encoding='utf-8'))
    assert payload['file_count'] == digest['file_count']
    assert 'signals/a.bin' in payload['files']
    assert payload['files']['signals/a.bin']['bytes'] == 24
    assert len(payload['files']['signals/a.bin']['sha256']) == 64
    assert 'DONE' not in payload['files']


def test_schema_written_on_finish(session):
    session.finish({'joint_states': {'file': 'joint_states.bin', 'ncol': 95}})
    schema = json.loads(session.paths.schema.read_text(encoding='utf-8'))
    assert schema['tables']['joint_states']['ncol'] == 95


def test_read_events_tolerates_truncated_tail(tmp_path):
    """崩溃会留下半行 JSON；丢掉它，别让一条坏尾巴废掉整个 session。"""
    p = tmp_path / 'e.jsonl'
    p.write_text('{"t":1,"type":"a"}\n{"t":2,"type":"b"}\n{"t":3,"ty',
                 encoding='utf-8')
    assert [e['type'] for e in read_events(p)] == ['a', 'b']


def test_write_done_is_atomic(tmp_path):
    paths = SessionPaths(tmp_path / 's')
    paths.make()
    (paths.root / 'x.txt').write_text('hello', encoding='utf-8')
    write_done(paths)
    assert paths.done.is_file()
    assert not list(paths.root.glob('DONE.tmp'))
