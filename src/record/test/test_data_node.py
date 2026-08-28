"""`data_node` 里挡路径穿越的那几个方法。

单挑这几个测是因为它们守着两件不可逆的事：**读任意文件**（`raw_path`）和
**删任意目录**（`drop_bundle` 里的 rmtree）。面板是 0.0.0.0 上开的，
token 和文件名都是外来串。

不起 rclpy —— 这几个方法只碰 `self.root` / `self._bundles` / `self.get_logger()`，
拿个替身绑上去就能测，比拉起整个节点快得多也稳得多。
"""

from __future__ import annotations

import sys
import threading
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


class _Stub:
    """凑齐这几个方法用到的那点东西。"""

    def __init__(self, root: Path, bundles: Path) -> None:
        self.root = root
        self._bundles = bundles
        self._lock = threading.Lock()
        self._convert = {'done': False, 'token': ''}
        self.errors: list[str] = []

    def get_logger(self):
        stub = self

        class _Log:
            def error(self, msg):
                stub.errors.append(msg)

            def warning(self, msg):
                pass

            def info(self, msg):
                pass
        return _Log()


def _stub(tmp_path: Path) -> _Stub:
    from record.data_node import DataManager
    root = tmp_path / 'sessions'
    bundles = tmp_path / 'bundles'
    (root / 's1' / 'video').mkdir(parents=True)
    (root / 's1' / 'video' / 'a.mkv').write_bytes(b'x' * 8)
    (root / 's2').mkdir()
    (bundles / 'tok').mkdir(parents=True)
    (bundles / 'tok' / 'meta.json').write_text('{}')
    (tmp_path / 'secret.txt').write_text('绝密')

    s = _Stub(root, bundles)
    for name in ('_dir', 'raw_files', 'raw_path', 'bundle',
                 '_bundle_path', 'drop_bundle'):
        setattr(_Stub, name, getattr(DataManager, name))
    return s


ESCAPES = ['../secret.txt', '../../secret.txt', '/etc/passwd',
           'video/../../secret.txt', '..', '']


def test_raw_path_allows_nested(tmp_path):
    s = _stub(tmp_path)
    assert s.raw_path('s1', 'video/a.mkv').read_bytes() == b'x' * 8


@pytest.mark.parametrize('bad', ESCAPES)
def test_raw_path_blocks_escape(tmp_path, bad):
    s = _stub(tmp_path)
    with pytest.raises(ValueError):
        s.raw_path('s1', bad)


@pytest.mark.parametrize('bad', ['../s1', 's1/../../etc', '', '..', 'a/b'])
def test_dir_blocks_escape(tmp_path, bad):
    s = _stub(tmp_path)
    with pytest.raises(ValueError):
        s._dir(bad)


def test_raw_files_lists_relative_paths(tmp_path):
    s = _stub(tmp_path)
    got = s.raw_files('s1')
    assert [f['path'] for f in got['files']] == ['video/a.mkv']
    assert got['bytes'] == 8


def test_bundle_resolves(tmp_path):
    s = _stub(tmp_path)
    assert s.bundle('tok') == (tmp_path / 'bundles' / 'tok').resolve()


def test_bundle_rejects_gone(tmp_path):
    s = _stub(tmp_path)
    with pytest.raises(ValueError, match='已经取过'):
        s.bundle('never')


@pytest.mark.parametrize('bad', ['../sessions/s1', '/etc', '', '..', 'a/b'])
def test_bundle_blocks_escape(tmp_path, bad):
    s = _stub(tmp_path)
    with pytest.raises(ValueError, match='非法'):
        s.bundle(bad)


@pytest.mark.parametrize('bad', ['../sessions/s1', '/etc', '', '..', 'a/b'])
def test_drop_bundle_refuses_to_delete_outside(tmp_path, bad):
    """最要命的一条：token 漏进来就等于 rm -rf 采集数据。"""
    s = _stub(tmp_path)
    s.drop_bundle(bad)
    assert (tmp_path / 'sessions' / 's1').is_dir()
    assert (tmp_path / 'secret.txt').is_file()
    assert s.errors, '拒绝了就该留个日志，不然静默吞掉'


def test_drop_bundle_deletes_own(tmp_path):
    s = _stub(tmp_path)
    s._convert = {'done': True, 'token': 'tok'}
    s.drop_bundle('tok')
    assert not (tmp_path / 'bundles' / 'tok').exists()
    # 取件链接是一次性的，不撑回去面板会一直挂着个死链接
    assert s._convert == {'done': False, 'token': ''}
    s.drop_bundle('tok')          # 重复取件不该炸


def test_drop_bundle_keeps_other_tokens_state(tmp_path):
    """掉队的旧 token 不能把当前那一次的取件链接抹了。"""
    s = _stub(tmp_path)
    s._convert = {'done': True, 'token': 'tok'}
    s.drop_bundle('stale')
    assert s._convert == {'done': True, 'token': 'tok'}


def _recorded(root: Path):
    """一次封好口的采集，两条 episode。不写信号表 —— 这几条只碰事件线。"""
    from record.session import Session
    s = Session.create(root, {'joint_states': True}, session_id='s3')
    s.start_round({'seed': 1, 'items': [], 'episodes': [{}, {}]})
    for outcome in ('success', 'fail'):
        s.start_episode({'instruction_en': 'x'})
        s.end_episode(outcome)
    s.end_round()
    s.finish({})
    return s.paths.root


def _editor(tmp_path: Path):
    from record.data_node import DataManager
    s = _stub(tmp_path)
    s.state = {'playing': False, 'session': '', 'label': ''}
    s._render = {'running': False, 'session': '', 'label': '',
                 'done': False, 'bytes': 0, 'error': ''}
    s._summary = {}
    for name in ('delete_episode', 'drop_render'):
        setattr(_Stub, name, getattr(DataManager, name))
    # 这两个是 staticmethod，`getattr` 会退化成普通函数，绑上去就多吃一个 self
    for name in ('_render_path', '_scan'):
        setattr(_Stub, name, staticmethod(getattr(DataManager, name)))
    _recorded(s.root)
    return s


def test_delete_episode_leaves_the_sealed_files_untouched(tmp_path):
    """删的是「这一条算数」这件事。events.jsonl 的 sha256 写在 DONE 里，碰不得。"""
    s = _editor(tmp_path)
    before = (s.root / 's3' / 'events.jsonl').read_bytes()
    got = s.delete_episode('s3', 'r0e1')
    assert got['deleted'] == ['r0e1']
    assert (s.root / 's3' / 'events.jsonl').read_bytes() == before

    from record.replay_source import open_session
    left = open_session(s.root / 's3').episodes(include_discarded=True)
    assert [e['label'] for e in left] == ['r0e0']
    # 概要跟着变，否则面板上的条数一直是删之前那个（它按 session 缓存）
    assert _Stub._scan(s.root / 's3')['episodes'] == 1


def test_delete_episode_rejects_unknown_label(tmp_path):
    s = _editor(tmp_path)
    with pytest.raises(ValueError, match='没有'):
        s.delete_episode('s3', 'r9e9')
    assert not (s.root / 's3' / 'edits.json').exists()


def test_delete_episode_refuses_while_it_is_playing(tmp_path):
    s = _editor(tmp_path)
    s.state = {'playing': True, 'session': 's3', 'label': 'r0e0'}
    with pytest.raises(RuntimeError, match='正在回放'):
        s.delete_episode('s3', 'r0e0')


def _converter_stub(tmp_path: Path):
    from record.data_node import DataManager
    s = _stub(tmp_path)
    (s.root / 's1' / 'DONE').write_text('')
    (s.root / 's2' / 'DONE').write_text('')
    s.state = {'playing': False}
    s._render = {'running': False}
    s._convert = {'running': False, 'session': '', 'sessions': [], 'token': ''}
    for name in ('start_convert', '_sweep_bundles'):
        setattr(_Stub, name, getattr(DataManager, name))
    s.bundle_ttl_s = 1800.0
    return s


def test_start_convert_rejects_an_empty_selection(tmp_path):
    """勾都没勾就点，别让它拿空清单去起子进程。"""
    s = _converter_stub(tmp_path)
    with pytest.raises(ValueError, match='没有勾选'):
        s.start_convert([], 'yb')


def test_start_convert_refuses_unsealed_sessions(tmp_path):
    """一批里只要有一个没封口就整批不转 —— 半份 dataset 比不转更难查。"""
    s = _converter_stub(tmp_path)
    (s.root / 's2' / 'DONE').unlink()
    with pytest.raises(RuntimeError, match='DONE'):
        s.start_convert(['s1', 's2'], 'yb')
    assert not s._convert['running']
