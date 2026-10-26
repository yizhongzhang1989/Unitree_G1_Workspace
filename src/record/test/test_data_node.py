"""数据管理的回放接口、双节点生命周期与文件路径边界测试。"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from unittest.mock import Mock

import pytest
from rclpy.client import Client
from rclpy.publisher import Publisher
from rclpy.task import Future
from std_msgs.msg import String
from std_srvs.srv import Trigger

from record.data_node import DataManager


class _ReplayManager(DataManager):
    sent: list[String]
    calls: list[str | tuple[str, Trigger.Request]]


def _replay_manager(subscribers=1) -> _ReplayManager:
    node = _ReplayManager.__new__(_ReplayManager)
    node._status = {
        'ready': True, 'error': '', 'playing': False, 'phase': 'idle',
    }
    node._status_t = time.time()
    node.status_topic = '/replay/status'
    node.request_topic = '/replay/request'
    node._lock = threading.Lock()
    node._request_path = ''
    node._render = {'running': False}
    node.state = {'playing': False, 'session': '', 'label': '',
                  'phase': 'idle', 'progress': 0.0, 'elapsed': 0.0, 'duration': 0.0,
                  'session_path': '', 'grip': [], 'joint_positions': [], 'error': ''}
    node.sent = []
    node.pub = Mock(spec=Publisher, get_subscription_count=Mock(return_value=subscribers),
                    publish=Mock(side_effect=node.sent.append))
    return node


def test_replay_requires_a_live_input_receiver():
    node = _replay_manager(subscribers=0)
    with pytest.raises(RuntimeError, match='没有回放接收者'):
        node._require_ready()


@pytest.mark.parametrize('ready', [False, 1])
def test_replay_requires_ready_model_and_feedback(ready):
    node = _replay_manager()
    node._status['ready'] = ready
    with pytest.raises(RuntimeError, match='模型和关节反馈'):
        node._require_ready()


def test_replay_observes_own_readiness():
    import json
    from record.data_node import DataManager
    from std_msgs.msg import String
    node = _replay_manager()
    DataManager._on_status(node, String(data=json.dumps({
        **node._status, 'ready': False})))
    with pytest.raises(RuntimeError):
        node._require_ready()
    DataManager._on_status(node, String(data=json.dumps({
        **node._status, 'ready': True})))
    assert node._require_ready() is None


@pytest.mark.parametrize('payload', ['[1]', 'null', '{',
                                     '{"ready": true, "session_path": []}'])
def test_replay_ignores_invalid_status(payload):
    from record.data_node import DataManager
    from std_msgs.msg import String
    node = _replay_manager()
    previous = dict(node._status)
    received_at = node._status_t
    DataManager._on_status(node, String(data=payload))
    assert node._status == previous
    assert node._status_t == received_at


def test_replay_requires_fresh_status():
    node = _replay_manager()
    node._status_t -= 2.0
    with pytest.raises(RuntimeError, match='没有数据'):
        node._require_ready()


def _playable(tmp_path: Path) -> _ReplayManager:
    node = _replay_manager()
    node.root = tmp_path
    directory = tmp_path / 's1'
    directory.mkdir()
    (directory / 'DONE').write_text('')
    return node


def test_replay_publishes_one_request_and_waits_for_status(tmp_path):
    import json
    from record.data_node import DataManager
    from std_msgs.msg import String
    node = _playable(tmp_path)
    result = DataManager.start(node, 's1', 1.0, 3.0, 0.5, 'r0e0')
    assert len(node.sent) == 1
    assert isinstance(node.sent[0], String)
    assert json.loads(node.sent[0].data) == {
        'session_path': str(tmp_path / 's1'), 't0': 1.0, 't1': 3.0,
        'speed': 0.5, 'label': 'r0e0'}
    assert result['playing'] is True
    assert result['phase'] == 'loading'
    assert result['session'] == 's1'
    assert result['label'] == 'r0e0'
    feedback = {
        **node._status, 'playing': True, 'phase': 'play', 'progress': 0.25,
        'elapsed': 1.0, 'duration': 4.0, 'label': 'r0e0',
        'session_path': str(tmp_path / 's1'),
        'grip': [0.4, 0.5], 'joint_positions': [0.1] * 14, 'error': '',
    }
    DataManager._on_status(node, String(data=json.dumps(feedback)))
    for key in ('playing', 'phase', 'progress', 'elapsed', 'duration', 'label',
                'session_path', 'grip', 'joint_positions', 'error'):
        assert node.state[key] == feedback[key]
    assert len(node.sent) == 1


def test_replay_ignores_feedback_for_a_previous_request(tmp_path):
    import json
    from record.data_node import DataManager
    from std_msgs.msg import String
    node = _playable(tmp_path)
    DataManager.start(node, 's1', 1.0, 3.0)
    DataManager._on_status(node, String(data=json.dumps({
        **node._status, 'session_path': str(tmp_path / 'older'),
        'playing': False, 'phase': 'idle', 'error': 'previous request'})))
    assert node.state['playing'] is True
    assert node.state['phase'] == 'loading'
    assert node.state['session'] == 's1'
    assert node.state['error'] == ''


@pytest.mark.parametrize('progress,error', [(0.25, ''), (1.0, ''), (1.0, 'invalid trajectory')])
def test_replay_observes_idle_with_retained_targets(tmp_path, progress, error):
    import json
    from record.data_node import DataManager
    from std_msgs.msg import String
    node = _playable(tmp_path)
    DataManager.start(node, 's1', 1.0, 3.0)
    DataManager._on_status(node, String(data=json.dumps({
        **node._status, 'session_path': str(tmp_path / 's1'),
        'playing': False, 'phase': 'idle', 'joint_positions': [0.2] * 14,
        'grip': [0.4, 0.5], 'progress': progress, 'error': error})))
    assert node.state['playing'] is False
    assert node.state['phase'] == 'idle'
    assert node.state['session'] == 's1'
    assert node.state['joint_positions'] == [0.2] * 14
    assert node.state['grip'] == [0.4, 0.5]
    assert node.state['progress'] == progress
    assert node.state['error'] == error
    assert len(node.sent) == 1


def test_replay_can_retry_a_failed_request_for_the_same_path(tmp_path):
    import json
    from record.data_node import DataManager
    from std_msgs.msg import String
    node = _playable(tmp_path)
    DataManager.start(node, 's1', 1.0, 3.0)
    DataManager._on_status(node, String(data=json.dumps({
        **node._status, 'session_path': str(tmp_path / 's1'),
        'playing': False, 'phase': 'idle', 'error': 'invalid trajectory'})))
    assert node._require_ready() is None
    result = DataManager.start(node, 's1', 1.0, 3.0)
    assert result['playing'] is True
    assert result['phase'] == 'loading'
    assert result['error'] == ''
    assert len(node.sent) == 2


@pytest.mark.parametrize('t0,t1,speed', [
    (2.0, 1.0, 1.0), (1.0, 1.0, 1.0), (-1.0, 1.0, 1.0),
    (float('nan'), 1.0, 1.0), (0.0, float('inf'), 1.0),
    (0.0, 1.0, 0.0), (0.0, 1.0, 2.1), (0.0, 1.0, float('nan')),
])
def test_replay_rejects_invalid_request(tmp_path, t0, t1, speed):
    from record.data_node import DataManager
    node = _playable(tmp_path)
    with pytest.raises(ValueError):
        DataManager.start(node, 's1', t0, t1, speed)
    assert not node.sent
    assert not node.state['playing']


@pytest.mark.parametrize('guard', ['unsealed', 'playing', 'rendering', 'receiver'])
def test_replay_start_preserves_guards(tmp_path, guard):
    from record.data_node import DataManager
    node = _playable(tmp_path)
    if guard == 'unsealed':
        (tmp_path / 's1' / 'DONE').unlink()
    elif guard == 'playing':
        node.state['playing'] = True
    elif guard == 'rendering':
        node._render['running'] = True
    else:
        node.pub.get_subscription_count = lambda: 0
    with pytest.raises(RuntimeError):
        DataManager.start(node, 's1', 0.0, 1.0)
    assert not node.sent


def test_replay_start_rejects_path_traversal(tmp_path):
    from record.data_node import DataManager
    node = _playable(tmp_path)
    with pytest.raises(ValueError):
        DataManager.start(node, '../s1', 0.0, 1.0)
    assert not node.sent


def _service_manager(success: bool = True, available: bool = True, done: bool = True) -> _ReplayManager:
    node = _replay_manager()
    node.calls = []
    future = Mock(spec=Future, done=Mock(return_value=done),
                  result=Mock(return_value=Trigger.Response(success=success, message='reply')),
                  cancel=Mock(side_effect=lambda: node.calls.append('cancel')))
    clients: dict[str, Client] = {
        'replay_stop': Mock(spec=Client,
                            srv_name='/replay/stop', wait_for_service=Mock(return_value=available),
                            call_async=Mock(side_effect=lambda request:
                                            (node.calls.append(('replay_stop', request)), future)[1])),
    }
    node._srv = clients
    return node


def test_replay_stop_calls_its_service():
    from record.data_node import DataManager
    from std_srvs.srv import Trigger
    node = _service_manager()
    node.state.update(playing=True, phase='play')
    result = DataManager.stop(node)
    assert len(node.calls) == 1
    assert node.calls[0][0] == 'replay_stop'
    assert isinstance(node.calls[0][1], Trigger.Request)
    assert result['playing'] is False
    assert result['phase'] == 'idle'
    assert not node.sent


@pytest.mark.parametrize('success,available', [(False, True), (True, False)])
def test_replay_stop_surfaces_service_failures(success, available):
    from record.data_node import DataManager
    node = _service_manager(success=success, available=available)
    node.state['playing'] = True
    with pytest.raises(RuntimeError):
        DataManager.stop(node)
    assert node.state['playing'] is True


def test_replay_stop_wait_is_bounded(monkeypatch):
    from record import data_node
    node = _service_manager(done=False)
    ticks = iter([0.0, 16.0])
    monkeypatch.setattr(data_node.time, 'monotonic', lambda: next(ticks))
    with pytest.raises(RuntimeError, match='超时'):
        data_node.DataManager.stop(node)
    assert node.calls[-1] == 'cancel'


def test_replay_stop_polls_until_response(monkeypatch):
    from record import data_node
    node = _service_manager(done=False)
    polls = iter([False, False, True, True])
    future = node._srv['replay_stop'].call_async(None)
    node.calls.clear()
    future.done = lambda: next(polls)
    waits = []
    monkeypatch.setattr(data_node.time, 'sleep', waits.append)
    assert data_node.DataManager.stop(node)['playing'] is False
    assert waits == [0.02, 0.02]


def test_status_exposes_replay_feedback_and_readiness(tmp_path):
    from record.data_node import DataManager
    node = _playable(tmp_path)
    node.peer_port = 8220
    node._peer_alive = lambda name, ttl=5.0: False
    DataManager.start(node, 's1', 0.0, 1.0)
    result = DataManager.status(node)
    assert result['ready'] is True
    assert result['blocked'] == ''
    assert result['phase'] == 'loading'
    assert result['session'] == 's1'
    assert result['session_path'] == str(tmp_path / 's1')
    node._status['error'] = 'invalid trajectory'
    assert DataManager.status(node)['ready'] is True
    node._status['ready'] = False
    result = DataManager.status(node)
    assert result['ready'] is False
    assert '模型和关节反馈' in result['blocked']


def test_request_interfaces_use_standard_types(tmp_path, monkeypatch):
    from types import SimpleNamespace
    from record import data_node
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    publishers, subscriptions, clients, parameters = [], [], [], {}
    monkeypatch.setattr(data_node.Node, '__init__', lambda self, name: None)

    def parameter(self, name, default):
        parameters[name] = default
        values = {'sessions_root': str(tmp_path / 'sessions'),
                  'bundles_root': str(tmp_path / 'bundles'), 'urdf': 'robot.urdf'}
        return Parameter(name, value=values.get(name, default))

    monkeypatch.setattr(data_node.DataManager, 'declare_parameter', parameter)
    monkeypatch.setattr(data_node.DataManager, 'create_publisher',
                        lambda self, kind, topic, qos: publishers.append((kind, topic)))
    monkeypatch.setattr(data_node.DataManager, 'create_subscription',
                        lambda self, kind, topic, callback, qos, **kwargs:
                        subscriptions.append((kind, topic)))
    monkeypatch.setattr(data_node.DataManager, 'create_client',
                        lambda self, kind, service, **kwargs: clients.append((kind, service)))
    monkeypatch.setattr(data_node.DataManager, '_sweep_bundles', lambda self, ttl: None)
    monkeypatch.setattr(data_node.DataManager, 'get_logger',
                        lambda self: SimpleNamespace(info=lambda message: None))
    node = data_node.DataManager()
    assert node.stop_service == parameters['stop_service'] == '/replay/stop'
    assert node.request_topic == parameters['request_topic'] == '/replay/request'
    assert node.status_topic == parameters['status_topic'] == '/replay/status'
    assert node.replay_config == parameters['replay_config'] == ''
    assert publishers == [(String, '/replay/request')]
    assert subscriptions == [(String, '/replay/status')]
    assert clients == [(Trigger, '/replay/stop')]


def test_main_owns_both_nodes_and_stops_replay_before_ros_shutdown(monkeypatch):
    from types import SimpleNamespace
    from record import data_dashboard, data_node
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    events, added, replay_kwargs, init_kwargs = [], [], {}, {}
    node = data_node.DataManager.__new__(data_node.DataManager)
    node.replay_config = '/test/replay.yaml'
    node.request_topic = '/preview_test/replay/request'
    node.status_topic = '/preview_test/replay/status'
    node.stop_service = '/preview_test/replay/stop'
    node.get_namespace = Mock(return_value='/preview_test')
    node.declare_parameter = Mock(side_effect=lambda name, default: Parameter(name, value=default))
    node.destroy_node = Mock(side_effect=lambda: events.append('data_destroy'))
    replay = data_node.ReplayTask.__new__(data_node.ReplayTask)
    replay.stop = Mock(side_effect=lambda: events.append('replay_stop'))
    replay.destroy_node = Mock(side_effect=lambda: events.append('replay_destroy'))

    def create_replay(**kwargs):
        replay_kwargs.update(kwargs)
        return replay

    def interrupted():
        raise KeyboardInterrupt

    executor = SimpleNamespace(
        add_node=added.append, spin=interrupted,
        shutdown=lambda: events.append('executor_shutdown'))
    dashboard = SimpleNamespace(start=lambda: None,
                                stop=lambda: events.append('dashboard_stop'))
    monkeypatch.setattr(data_node, 'DataManager', lambda: node)
    monkeypatch.setattr(data_node, 'ReplayTask', create_replay)
    monkeypatch.setattr(data_node, 'MultiThreadedExecutor', lambda **kwargs: executor)
    monkeypatch.setattr(data_dashboard, 'panel', lambda node, port: dashboard)
    monkeypatch.setattr(data_node.rclpy, 'init', lambda **kwargs: init_kwargs.update(kwargs))
    monkeypatch.setattr(data_node.rclpy, 'try_shutdown', lambda: events.append('ros_shutdown'))
    monkeypatch.setattr(data_node.signal, 'signal', lambda *args: None)
    data_node.main()
    assert init_kwargs == {'signal_handler_options': SignalHandlerOptions.NO}
    assert replay_kwargs == {
        'config': '/test/replay.yaml',
        'request_topic': '/preview_test/replay/request',
        'status_topic': '/preview_test/replay/status',
        'stop_service': '/preview_test/replay/stop',
        'namespace': '/preview_test',
    }
    assert len(added) == 2 and added[0] is node and added[1] is replay
    assert events == ['dashboard_stop', 'replay_stop', 'executor_shutdown',
                      'replay_destroy', 'data_destroy', 'ros_shutdown']


class _Stub(DataManager):
    """凑齐这几个方法用到的那点东西。"""

    def __init__(self, root: Path, bundles: Path) -> None:
        self.root = root
        self._bundles = bundles
        self._lock = threading.Lock()
        self._convert = {'done': False, 'token': ''}
        self.errors: list[str] = []
        self.logger = Mock()
        self.logger.error.side_effect = self.errors.append

    def get_logger(self):
        return self.logger


def _stub(tmp_path: Path) -> _Stub:
    root = tmp_path / 'sessions'
    bundles = tmp_path / 'bundles'
    (root / 's1' / 'video').mkdir(parents=True)
    (root / 's1' / 'video' / 'a.mkv').write_bytes(b'x' * 8)
    (root / 's2').mkdir()
    (bundles / 'tok').mkdir(parents=True)
    (bundles / 'tok' / 'meta.json').write_text('{}')
    (tmp_path / 'secret.txt').write_text('绝密')

    return _Stub(root, bundles)


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
    s = _stub(tmp_path)
    s.state = {'playing': False, 'session': '', 'label': ''}
    s._render = {'running': False, 'session': '', 'label': '',
                 'done': False, 'bytes': 0, 'error': ''}
    s._summary = {}
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
    s = _stub(tmp_path)
    (s.root / 's1' / 'DONE').write_text('')
    (s.root / 's2' / 'DONE').write_text('')
    s.state = {'playing': False}
    s._render = {'running': False}
    s._convert = {'running': False, 'session': '', 'sessions': [], 'token': ''}
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
