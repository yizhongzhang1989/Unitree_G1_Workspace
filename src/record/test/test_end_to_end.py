"""端到端：写一个 session，再用导出机那套工具（纯 numpy）读回来。

这是整个落盘格式的验收 —— 在一台只有 Python + numpy 的 Windows 上必须能把数据取出来。
"""

import importlib.util
import json
import shutil
import time
from pathlib import Path

import numpy as np
import pytest

from record.session import Session
from record.table_writer import TableWriter

TOOLS = Path(__file__).resolve().parents[1] / 'tools'


@pytest.fixture(scope='module')
def reader_module():
    spec = importlib.util.spec_from_file_location('session_reader',
                                                  TOOLS / 'session_reader.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_reader_has_no_ros_dependency():
    """导出机没有 ROS。这条断了整条交付链就断了。"""
    import re
    banned = re.compile(r'^\s*(?:import|from)\s+(rclpy|rosbag2\w*|sensor_msgs|'
                        r'std_msgs|geometry_msgs|ament\w*)\b', re.M)
    for name in ('session_reader.py', 'inspect_session.py'):
        src = (TOOLS / name).read_text(encoding='utf-8')
        assert not banned.search(src), f'{name} 引入了 ROS 依赖'


def _build_session(root: Path, calibration: dict) -> Session:
    streams = {'joint_states': True, 'head': True, 'head_depth': False}
    s = Session.create(root, streams, meta={'note': '端到端'},
                       calibration=calibration)
    cols = ['t_recv', 't_header', 'pos.a', 'pos.b']
    w = TableWriter(s.paths.signals / 'joint_states.bin', cols)

    t0 = time.time()
    for k in range(200):                       # 100 Hz x 2 s
        t = t0 + k * 0.01
        w.append([t, t - 0.001, k * 1.0, -k * 1.0])

    # 腕部墙钟带抖动 + 中间丢两帧；头部是硬件戳，干净
    rng = np.random.default_rng(0)
    wrist = np.array([t0 + i / 30.0 for i in range(60)])
    wrist += rng.normal(0, 0.008, wrist.size)
    wrist = np.delete(wrist, [30, 31])
    wrist.astype(np.float64).tofile(s.paths.video / 'wrist_left.pts.bin')
    (s.paths.video / 'wrist_left.mkv').write_bytes(b'\x1aE\xdf\xa3fake')
    np.array([t0 + i / 30.0 for i in range(60)],
             dtype=np.float64).tofile(s.paths.video / 'head.pts.bin')
    (s.paths.video / 'head.mkv').write_bytes(b'\x1aE\xdf\xa3fake')

    s.start_round({'seed': 4242, 'items': [{'item_id': 'itm_a', 'en': 'red block'}],
                   'episodes': [
                       {'instruction_en': 'Pick up the red block with the left arm',
                        'instruction_zh': '左手拿起红色方块', 'verb': 'Pick up',
                        'arm': 'left', 'step_index': 0, 'step_total': 2},
                       {'instruction_en': 'Place the red block on the table with the left arm',
                        'instruction_zh': '左手把红色方块放在桌面上', 'verb': 'Place',
                        'arm': 'left', 'step_index': 1, 'step_total': 2}]},
                  svg='<svg xmlns="http://www.w3.org/2000/svg"/>')
    detail = json.loads((s.paths.rounds / 'round_000.json').read_text(encoding='utf-8'))
    for k, ep in enumerate(detail['episodes']):
        s.start_episode(ep)
        time.sleep(0.05)
        s.end_episode('success' if k == 0 else 'fail')
    s.start_episode(detail['episodes'][0])
    s.end_episode('discard', note='手滑')
    s.warn('lint', episode=0, hit='测试用')
    s.end_round()

    w.close()
    s.finish({'joint_states': w.schema()})
    return s


@pytest.fixture(scope='module')
def built(tmp_path_factory, calibration):
    return _build_session(tmp_path_factory.mktemp('sessions'), calibration)


def test_session_is_sealed_and_verifies(built, reader_module):
    r = reader_module.Session(built.paths.root)
    assert r.sealed
    assert r.verify() == []


def test_manifest_and_schema_survive_roundtrip(built, reader_module):
    r = reader_module.Session(built.paths.root)
    assert r.manifest['streams']['head_depth'] is False
    assert r.schema['joint_states']['ncol'] == 4
    assert r.meta['note'] == '端到端'


def test_signal_table_reads_back_exactly(built, reader_module):
    r = reader_module.Session(built.paths.root)
    t, data = r.table('joint_states')
    assert data.shape == (200, 2)
    assert np.array_equal(data[:, 0], np.arange(200, dtype=float))
    assert r.columns('joint_states') == ['pos.a', 'pos.b']
    assert np.all(np.diff(t) > 0)


def test_bare_numpy_can_read_without_the_tools(built):
    """连 session_reader 都不要，只有 np.fromfile —— 这是格式的最低保证。"""
    entry = json.loads((built.paths.schema).read_text(encoding='utf-8'))
    ncol = entry['tables']['joint_states']['ncol']
    flat = np.fromfile(built.paths.signals / 'joint_states.bin', dtype=np.float64)
    assert flat.size % ncol == 0
    assert flat.reshape(-1, ncol).shape == (200, 4)


def test_episodes_carry_instruction_and_span(built, reader_module):
    eps = reader_module.Session(built.paths.root).episodes()
    assert len(eps) == 2                       # discard 默认不出现
    assert eps[0]['instruction_en'].startswith('Pick up the red block')
    assert eps[0]['verb'] == 'Pick up' and eps[0]['step_total'] == 2
    assert eps[0]['outcome'] == 'success' and eps[1]['outcome'] == 'fail'
    for e in eps:
        assert e['t1'] > e['t0'] and e['duration'] > 0


def test_discarded_episode_available_on_request(built, reader_module):
    r = reader_module.Session(built.paths.root)
    assert len(r.episodes(include_discarded=True)) == 3


def test_reader_reports_the_relabelled_outcome(tmp_path, reader_module):
    """当场标的结论事后改过，下游拿到的必须是改过的那个。"""
    s = Session.create(tmp_path / 'sessions', {'joint_states': True})
    w = TableWriter(s.paths.signals / 'joint_states.bin', ['t_recv', 't_header', 'a'])
    w.append([time.time(), float('nan'), 1.0])
    s.start_round({'seed': 1, 'items': [], 'episodes': [{'instruction_en': 'a'}]})
    s.start_episode({'instruction_en': 'a'})
    s.end_episode('success')
    s.relabel_episode(0, 'fail')
    s.end_round()
    w.close()
    s.finish({'joint_states': w.schema()})
    eps = reader_module.Session(s.paths.root).episodes(include_discarded=True)
    assert [(e['label'], e['outcome']) for e in eps] == [('r0e0', 'fail')]


def test_deleting_an_episode_keeps_the_seal_intact(built, reader_module, tmp_path):
    """删一条 episode 绝不能去改 `events.jsonl` —— 它的 sha256 写在 `DONE` 里，
    改一个字节整次采集就校验不过了。所以它记在旁挂的 `edits.json` 里。
    """
    root = Path(shutil.copytree(built.paths.root, tmp_path / 'copy'))
    (root / 'edits.json').write_text(json.dumps({'deleted': ['r0e1']}),
                                     encoding='utf-8')
    r = reader_module.Session(root)
    assert r.verify() == []
    assert [e['label'] for e in r.episodes(include_discarded=True)] == ['r0e0', 'r0e2']
    assert len(r.episodes(include_discarded=True, include_deleted=True)) == 3


def test_slicing_lines_up_signals_and_frames(built, reader_module):
    r = reader_module.Session(built.paths.root)
    ep = r.episodes()[0]
    t, data = r.slice_table('joint_states', ep['t0'], ep['t1'])
    assert t.size == data.shape[0] > 0
    assert (t >= ep['t0']).all() and (t <= ep['t1']).all()
    frames = r.slice_frames('head', ep['t0'], ep['t1'])
    assert frames.ndim == 1


def test_round_json_keeps_seed_and_scene_binding(built, reader_module):
    rounds = reader_module.Session(built.paths.root).rounds()
    assert len(rounds) == 1 and rounds[0]['seed'] == 4242
    assert rounds[0]['items'][0]['item_id'] == 'itm_a'


def test_warnings_are_recorded_not_swallowed(built, reader_module):
    warns = reader_module.Session(built.paths.root).warnings()
    assert len(warns) == 1 and warns[0]['code'] == 'lint'


def test_wrist_jitter_is_smoothed(built, reader_module):
    """腕部戳是成簇到达的墙钟，重建后必须严格等间隔。

    RTSP over TCP 实测残差 p95 达 82 ms（约三帧），逐帧到达时刻不可用；
    可用的只有「帧 i 采于 t0 + i/fps」。
    """
    r = reader_module.Session(built.paths.root)
    raw = r.video_pts('wrist_left', fit=False)
    fit = r.video_pts('wrist_left', fit=True)
    assert fit.size == raw.size
    d = np.diff(fit)
    # unix 纪元的 float64 只有约 100 ns 分辨率，1 us 已经远严于 ms 级的用途
    assert float(np.std(d)) < 1e-6, '重建后应严格等间隔'
    assert abs(float(np.median(d)) - 1 / 30) < 2e-3


def test_pts_residual_reports_dirtiness(built, reader_module):
    raw = reader_module.Session(built.paths.root).video_pts('wrist_left', fit=False)
    stat = reader_module.pts_residual(raw)
    assert stat['frames'] == raw.size
    assert 25 < stat['fps'] < 35
    assert stat['p95_ms'] >= stat['p50_ms'] >= 0


def test_head_pts_untouched(built, reader_module):
    """头部是 RealSense 硬件戳，不该被拟合动过。"""
    r = reader_module.Session(built.paths.root)
    assert np.array_equal(r.video_pts('head'), r.video_pts('head', fit=False))


def test_verify_detects_tampering(built, reader_module, tmp_path):
    import shutil
    copy = tmp_path / 'copy'
    shutil.copytree(built.paths.root, copy)
    with open(copy / 'signals' / 'joint_states.bin', 'r+b') as f:
        f.write(b'\xff' * 8)
    assert any('joint_states' in b for b in reader_module.Session(copy).verify())
