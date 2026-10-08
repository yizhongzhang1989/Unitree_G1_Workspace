# pyright: reportAttributeAccessIssue=false

import json
from unittest.mock import Mock

import pytest
import requests

from g1_vla_bridge.online_rl.session import Journal, score_reward
from g1_vla_bridge.online_rl.transport import ApiError, RetryRequest, Transport


@pytest.mark.parametrize('text,reward', [
    ('', 0.), (' ', 0.), ('1', -1.), ('2', -.5),
    ('3', 0.), ('4', .5), ('5', 1.), ('/null', None)])
def test_score_mapping(text, reward):
    assert score_reward(text) == reward


@pytest.mark.parametrize('text', ['0', '6', '3.0', 'nan', '/home'])
def test_invalid_score(text):
    with pytest.raises(ValueError):
        score_reward(text)


def prepared(journal):
    return journal.prepare({'state': {}, 'return_dict': True}, (b'head', b'left', b'right'), 12.)


def received(journal):
    request_id = prepared(journal)
    journal.receive(request_id, {'id': 'chunk-1', 'policy_version': 0, 'action': {}})
    return request_id


def test_wire_survives_restart_and_excludes_new_observation(tmp_path):
    journal = Journal(tmp_path)
    request_id = prepared(journal)
    original = journal.wire(request_id)
    journal.close()
    journal = Journal(tmp_path)
    assert journal.wire(request_id) == original
    assert json.loads(original[0])['request_id'] == request_id
    with pytest.raises(RuntimeError):
        prepared(journal)
    journal.close()


def test_single_writer(tmp_path):
    journal = Journal(tmp_path)
    with pytest.raises(RuntimeError, match='already in use'):
        Journal(tmp_path)
    journal.close()


def test_resolved_request_keeps_small_ledger_and_response_identity(tmp_path):
    journal = Journal(tmp_path)
    request_id = prepared(journal)
    response = dict(id='chunk-1', policy_version=3, batch_id='batch-1',
                    execution_chunk_size=30, action={'positions': [1.] * 1000})
    record = journal.receive(request_id, response)
    assert record['response'] == dict(policy_version=3, batch_id='batch-1', execution_chunk_size=30)
    assert journal.receive(request_id, response) == record
    with pytest.raises(ValueError, match='conflicting'):
        journal.receive(request_id, dict(response, action={'positions': [2.]}))
    blobs = journal._db.execute(
        'SELECT query, head, left_image, right_image FROM requests WHERE request_id=?',
        (request_id,)).fetchone()
    assert blobs == (b'', b'', b'', b'')
    with pytest.raises(ValueError, match='resolved'):
        journal.wire(request_id)
    journal.close()


def test_existing_ledger_compaction_preserves_retry_and_recovery(tmp_path):
    journal = Journal(tmp_path)
    response = dict(id='chunk-1', policy_version=5, batch_id='batch-2',
                    execution_chunk_size=30, action={'positions': [1.] * 200000})
    completed = dict(request_id='old', id='chunk-1', state='feedback_pending',
                     executed_steps=30, feedback=dict(id='chunk-1', reward=.5, executed_steps=30),
                     feedback_confirmed=False, response=response)
    journal._db.execute('INSERT INTO requests VALUES (?, ?, ?, ?, ?, ?, ?)',
                        ('old', 'chunk-1', json.dumps(completed), b'query',
                         b'image' * 200000, b'image' * 200000, b'image' * 200000))
    journal._db.commit()
    journal.update('old', state='confirmed')
    request_id = prepared(journal)
    original = journal.wire(request_id)
    journal._db.execute("DELETE FROM metadata WHERE key='compact_storage'")
    journal._db.execute("UPDATE requests SET data=?, head=? WHERE request_id='old'",
                        (json.dumps(completed), b'image' * 200000))
    journal._db.commit()
    journal.close()
    path = tmp_path / 'session.sqlite3'
    original_size = path.stat().st_size
    journal = Journal(tmp_path)
    assert journal.wire(request_id) == original
    assert journal.get('old')['feedback'] == completed['feedback']
    assert journal.get('old')['response']['policy_version'] == 5
    assert path.stat().st_size < original_size / 10
    journal.confirm('old', {'duplicate': True})
    assert journal.get('old')['feedback_confirmed']
    journal.close()


@pytest.mark.parametrize('result', ['received', 'discarded'])
def test_request_resolution_reclaims_database_pages(tmp_path, result):
    journal = Journal(tmp_path)
    request_id = journal.prepare({}, (b'image' * 200000,) * 3, 12.)
    path = tmp_path / 'session.sqlite3'
    original_size = path.stat().st_size
    if result == 'received':
        journal.receive(request_id, dict(id='chunk-1', policy_version=1,
                                         batch_id='batch-1', execution_chunk_size=30))
    else:
        journal.update(request_id, state='discarded')
    assert path.stat().st_size < original_size / 10
    assert journal.get(request_id)['state'] == result
    journal.close()


def test_pending_query_reads_only_open_records_after_reopening(tmp_path, monkeypatch):
    journal = Journal(tmp_path)
    for state in ('confirmed', 'expired', 'discarded'):
        journal.update(prepared(journal), state=state)
    request_id = prepared(journal)
    journal._db.execute('DROP INDEX pending_requests')
    journal.close()
    journal = Journal(tmp_path)
    loads = Mock(wraps=json.loads)
    monkeypatch.setattr('g1_vla_bridge.online_rl.session.json.loads', loads)
    statements = []
    journal._db.set_trace_callback(statements.append)
    assert [record['request_id'] for record in journal.pending()] == [request_id]
    assert loads.call_count == 1
    plan = journal._db.execute('EXPLAIN QUERY PLAN ' + statements[0])
    assert any('pending_requests' in row[-1] for row in plan)
    journal.update(request_id, state='discarded')
    assert journal.pending() == []
    journal.close()


def test_claim_is_durable_and_cannot_be_replayed(tmp_path):
    journal = Journal(tmp_path)
    request_id = received(journal)
    journal.claim(request_id, 30)
    with pytest.raises(RuntimeError):
        journal.claim(request_id, 30)
    journal.close()
    journal = Journal(tmp_path)
    assert journal.get(request_id)['state'] == 'uncertain'
    with pytest.raises(ValueError):
        journal.queue_feedback(request_id, 0)
    journal.resolve(request_id, 12)
    assert journal.resolve(request_id, 12)['operator_verified']
    with pytest.raises(ValueError):
        journal.resolve(request_id, 13)
    record = journal.queue_feedback(request_id, None)
    assert record['feedback'] == {'id': 'chunk-1', 'reward': None, 'executed_steps': 12}
    journal.close()


def test_feedback_is_immutable_and_confirmation_survives_restart(tmp_path):
    journal = Journal(tmp_path)
    request_id = received(journal)
    journal.claim(request_id, 30)
    journal.finish(request_id, 30, True)
    record = journal.queue_feedback(request_id, .5)
    assert journal.queue_feedback(request_id, .5) == record
    with pytest.raises(ValueError):
        journal.queue_feedback(request_id, -.5)
    journal.close()
    journal = Journal(tmp_path)
    assert journal.get(request_id)['feedback'] == record['feedback']
    journal.confirm(request_id, {'duplicate': True})
    assert journal.pending() == []
    assert journal.get(request_id)['feedback_confirmed']
    journal.close()


def test_cached_unexecuted_action_is_skipped_after_restart(tmp_path):
    journal = Journal(tmp_path)
    request_id = received(journal)
    journal.close()
    journal = Journal(tmp_path)
    assert journal.get(request_id)['state'] == 'skipped'
    with pytest.raises(ValueError):
        journal.queue_feedback(request_id, 0)
    assert journal.queue_feedback(request_id, None)['feedback']['executed_steps'] == 0
    journal.close()


@pytest.fixture
def transport(tmp_path, monkeypatch):
    from test_cogact_unitree_backend import configured_backend, history_config

    backend, _ = configured_backend(monkeypatch, history_config())
    backend._session.proxies = {}
    journal = Journal(tmp_path)
    http = Mock(proxies={})
    monkeypatch.setattr('g1_vla_bridge.online_rl.transport.requests.Session', lambda: http)
    transport = Transport(backend, journal)
    yield transport, journal, http
    journal.close()


def http_response(status, body):
    return Mock(status_code=status, json=Mock(return_value=body))


def test_timeout_retries_exact_multipart(transport):
    client, journal, http = transport
    request_id = prepared(journal)
    http.request.side_effect = requests.Timeout()
    for _ in range(2):
        with pytest.raises(RetryRequest):
            client.infer(request_id)
    first, second = http.request.call_args_list
    assert first == second
    assert not first.kwargs['allow_redirects']


@pytest.mark.parametrize('status,code,retry', [
    (409, 'pending', True), (409, 'conflict', False), (503, 'updating', False),
    (503, 'stale_observation', False), (429, 'pending_limit', False),
    (410, 'expired', False), (401, 'unknown', False)])
def test_http_errors_are_distinct(transport, status, code, retry):
    client, journal, http = transport
    http.request.return_value = http_response(status, {'error': code})
    with pytest.raises(RetryRequest if retry else ApiError) as caught:
        client.infer(prepared(journal))
    if not retry:
        assert caught.value.status == status
        assert caught.value.code == code


def test_feedback_retries_exact_payload_and_confirms_duplicate(transport):
    client, journal, http = transport
    request_id = received(journal)
    journal.claim(request_id, 30)
    journal.finish(request_id, 30, True)
    journal.queue_feedback(request_id, score_reward(''))
    http.request.side_effect = [requests.ConnectionError(),
                                http_response(200, {'duplicate': True, 'id': 'chunk-1'})]
    with pytest.raises(RetryRequest):
        client.feedback(request_id)
    assert journal.get(request_id)['state'] == 'feedback_pending'
    client.feedback(request_id)
    assert http.request.call_args_list[0] == http.request.call_args_list[1]
    assert http.request.call_args.kwargs['json']['reward'] == 0
    assert journal.get(request_id)['feedback_confirmed']


def test_rl_metadata_preserved_separately_from_actions(transport):
    from test_cogact_unitree_backend import _observation
    import time

    client, journal, http = transport
    observation = _observation()
    observation.acquired_monotonic = time.monotonic()
    request_id = client.prepare(observation)
    body = client.backend._session.post.return_value.json.return_value
    body.update(id='chunk-1', policy_version=7, batch_id='batch-2', execution_chunk_size=16)
    http.request.return_value = http_response(200, body)
    chunk, metadata = client.infer(request_id)
    assert chunk.horizon == 30
    assert metadata['execution_chunk_size'] == 16
    assert journal.get(request_id)['response']['policy_version'] == 7


@pytest.fixture
def rl_bridge(monkeypatch, tmp_path):
    import queue
    import threading

    from g1_vla_bridge.online_rl.bridge import OnlineRLBridge
    import test_async_execution

    original, _ = test_async_execution.bridge.__wrapped__(monkeypatch)
    node = object.__new__(OnlineRLBridge)
    node.__dict__.update(original.__dict__)
    node._execution_mode = 'manual'
    node._action_rate, node._execution_rate = 10., 30.
    node._rl_guard = threading.RLock()
    node._rl_session = Journal(tmp_path)
    node._rl_endings = queue.SimpleQueue()
    node._rl_commands = queue.Queue()
    node._rl_pending = False
    node._rl_active = None
    node._rl_owner = 0
    node._rl_steps = 0
    node._rl_phase = 'collecting'
    node._rl_fault = ''
    node._rl_message = ''
    node._rl_last = None
    node._rl_max_age = 3.
    node._rl_transport = Mock()
    yield node
    node._rl_session.close()


def start_rl_chunk(node):
    import time
    from test_async_execution import prediction

    request_id = prepared(node._rl_session)
    node._rl_session.update(request_id, observation_time=time.time())

    def infer(identity):
        body = {'id': 'chunk-1', 'execution_chunk_size': 30}
        node._rl_session.receive(identity, body)
        return prediction(), body

    node._rl_transport.infer.side_effect = infer
    node._inference_active = True
    node._fetch(request_id)
    return request_id


def test_rl_counts_model_steps_and_blocks_next_until_feedback(rl_bridge):
    node = rl_bridge
    request_id = start_rl_chunk(node)
    assert node._rl_session.get(request_id)['state'] == 'executing'
    assert node._request_inference()
    for _ in range(90):
        node._on_tick()
    node._drain_endings()
    record = node._rl_session.get(request_id)
    assert record['state'] == 'completed'
    assert record['executed_steps'] == 30
    assert node._publisher.publish.call_count == 180
    assert node._request_inference()
    node._rl_session.queue_feedback(request_id, 0)
    node._rl_session.confirm(request_id, {'duplicate': False})
    node._snapshot()
    assert node._request_inference() == ''


def test_partial_execution_keeps_id_and_model_count(rl_bridge):
    node = rl_bridge
    request_id = start_rl_chunk(node)
    for _ in range(10):
        node._on_tick()
    node._stop('Operator stopped')
    node._drain_endings()
    record = node._rl_session.get(request_id)
    assert record['state'] == 'interrupted'
    assert record['executed_steps'] == 4
    assert record['id'] == 'chunk-1'
    assert node._request_inference()


@pytest.mark.parametrize('execution_rate', [5., 10., 30.])
def test_completion_requires_successful_final_publication(rl_bridge, execution_rate):
    node = rl_bridge
    node._execution_rate = execution_rate
    request_id = start_rl_chunk(node)
    for _ in range(int(30 * execution_rate / node._action_rate) - 1):
        node._on_tick()
    assert node._rl_session.get(request_id)['state'] == 'executing'
    node._publisher.publish.side_effect = RuntimeError('final publication failed')
    node._on_tick()
    node._drain_endings()
    assert node._rl_session.get(request_id)['state'] == 'uncertain'


def test_stale_response_never_publishes_and_is_null_eligible(rl_bridge):
    node = rl_bridge
    node._rl_max_age = -1
    request_id = start_rl_chunk(node)
    node._on_tick()
    node._publisher.publish.assert_not_called()
    assert node._rl_session.get(request_id)['state'] == 'skipped'
    assert node._rl_session.queue_feedback(request_id, None)['feedback']['reward'] is None


def test_partial_publication_requires_operator_resolution(rl_bridge):
    node = rl_bridge
    request_id = start_rl_chunk(node)
    node._publisher.publish.side_effect = [None, RuntimeError('gripper publication failed')]
    node._on_tick()
    node._drain_endings()
    assert node._rl_session.get(request_id)['state'] == 'uncertain'
    assert not node._running.is_set()


def test_cli_enter_is_score_only_after_completion():
    from g1_vla_bridge.online_rl.cli import OnlineRLCli, prompt_text

    cli = object.__new__(OnlineRLCli)
    cli._rl = {'pending': False, 'phase': 'collecting'}
    cli._submitted = None
    cli.execute_one = Mock()
    cli.set_task = Mock()
    cli.submit = Mock()
    assert cli.handle_line('pick up the cup')
    cli.set_task.assert_called_once_with('pick up the cup')
    cli.execute_one.assert_not_called()
    cli.handle_line('')
    cli.execute_one.assert_called_once()
    record = dict(id='chunk-A', request_id='request-A', state='completed', executed_steps=30)
    cli._rl = dict(pending=True, record=record)
    assert 'Enter=3' in prompt_text({}, cli._rl)
    cli.handle_line('', dict(record))
    cli.submit.assert_called_once_with(dict(command='score', id='chunk-A',
                                            request_id='request-A', reward=0.))
    cli.execute_one.assert_called_once()


def test_cli_score_binds_displayed_id_and_home_remains_available():
    from g1_vla_bridge.online_rl.cli import OnlineRLCli

    cli = object.__new__(OnlineRLCli)
    cli._rl = {'pending': True, 'record': {'id': 'new-chunk'}}
    cli._submitted = None
    cli._home = object()
    cli.call = Mock()
    cli.submit = Mock()
    shown = dict(id='shown-chunk', request_id='shown-request', state='completed')
    cli.handle_line('5', shown)
    assert cli.submit.call_args.args[0]['id'] == 'shown-chunk'
    cli.handle_line('/home', shown)
    cli.call.assert_called_once_with(cli._home, 'home')


@pytest.mark.parametrize('command,state,confirmed,identity,acknowledged', [
    ('score', 'confirmed', True, 'request-A', True),
    ('score', 'expired', False, 'request-A', True),
    ('resolve', 'interrupted', False, 'request-A', True),
    ('score', 'feedback_pending', False, 'request-A', False),
    ('score', 'confirmed', True, 'other-request', False)])
def test_cli_acknowledgement_matches_pending_submission(command, state, confirmed, identity, acknowledged):
    from types import SimpleNamespace
    from g1_vla_bridge.online_rl.cli import OnlineRLCli

    cli = object.__new__(OnlineRLCli)
    submission = dict(command=command, request_id='request-A', id='chunk-A')
    cli._submitted = submission
    record = dict(request_id=identity, id='chunk-A', state=state,
                  feedback_confirmed=confirmed, feedback={'reward': .5})
    cli._on_rl_status(SimpleNamespace(data=json.dumps({'record': record})))
    assert cli._submitted == (None if acknowledged else submission)


def test_cli_estop_precedes_stopping_the_bridge():
    from g1_vla_bridge.online_rl.cli import OnlineRLCli

    cli = object.__new__(OnlineRLCli)
    cli._estop, cli._stop = object(), object()
    cli.call = Mock()
    assert cli.handle_line('/estop')
    assert [call.args for call in cli.call.call_args_list] == [
        (cli._estop, 'estop'), (cli._stop, 'stop')]


def test_cli_never_defaults_interrupted_chunk_to_zero():
    from g1_vla_bridge.online_rl.cli import score_command

    record = dict(id='chunk-A', request_id='request-A', state='interrupted')
    with pytest.raises(ValueError, match='/null'):
        score_command('', record)
    assert score_command('/null', record)['reward'] is None


def test_launch_uses_existing_settings_with_independent_node(monkeypatch):
    from pathlib import Path
    import runpy
    from launch import LaunchContext

    package = Path(__file__).resolve().parents[1]
    launch = runpy.run_path(str(package / 'launch/online_rl.launch.py'))
    context = LaunchContext()
    context.launch_configurations.update({name: '' for name in launch['OPTIONAL']})
    context.launch_configurations.update(rl_directory='/tmp/rl-test',
                                         rl_max_observation_age_s='3.0', proxy='socks5h://proxy:1080')
    factory = Mock()
    monkeypatch.setitem(launch['_node'].__globals__, 'Node', factory)
    def get_package_share_directory(name):
        _ = name
        return str(package)

    monkeypatch.setitem(launch['_node'].__globals__, 'get_package_share_directory',
                        get_package_share_directory)
    launch['_node'](context)
    arguments = factory.call_args.kwargs
    assert arguments['name'] == 'online_rl_bridge'
    assert arguments['executable'] == 'online_rl_bridge'
    parameters = arguments['parameters'][0]
    assert parameters['task_description'] == ''
    assert parameters['execution_mode'] == 'manual'
    assert not parameters['skip_intermediate_waypoints']
    assert not parameters['cartesian_limit_enabled']
    assert parameters['proxy'] == 'socks5h://proxy:1080'
    assert parameters['action_rate_hz'] == 10.
    assert parameters['execution_rate_hz'] == 30.
    assert parameters['history_length'] == 16


def test_experiment_cannot_send_pending_requests_to_another_server(tmp_path):
    journal = Journal(tmp_path)
    journal.bind_server('http://server-a/api/inference')
    journal.bind_server('http://server-a/api/inference')
    with pytest.raises(ValueError, match='different server'):
        journal.bind_server('http://server-b/api/inference')
    journal.close()


@pytest.mark.parametrize('code', ['updating', 'stale_observation'])
def test_update_discards_reservation_without_reusing_observation(rl_bridge, code):
    node = rl_bridge
    request_id = prepared(node._rl_session)
    node._inference_active = True
    node._handle_api_error(ApiError(503, code))
    assert node._rl_session.get(request_id)['state'] == 'discarded'
    assert not node._inference_active
    node._snapshot()
    assert node._request_inference()
    node._rl_phase = 'collecting'
    assert node._request_inference() == ''


def test_feedback_busy_keeps_payload_without_stopping_or_reexecuting(rl_bridge):
    node = rl_bridge
    request_id = start_rl_chunk(node)
    for _ in range(90):
        node._on_tick()
    node._drain_endings()
    feedback = node._rl_session.queue_feedback(request_id, .5)['feedback']
    node._handle_api_error(ApiError(503, 'updating'))
    assert not node._rl_fault
    assert node._rl_session.get(request_id)['feedback'] == feedback
    assert node._rl_session.get(request_id)['state'] == 'feedback_pending'
    node._on_tick()
    assert node._publisher.publish.call_count == 180


def test_feedback_outbox_runs_even_when_status_is_temporarily_unreachable(rl_bridge):
    import threading

    node = rl_bridge
    node._rl_ready = threading.Event()
    node._rl_ready.set()
    request_id = start_rl_chunk(node)
    for _ in range(90):
        node._on_tick()
    node._drain_endings()
    node._rl_session.queue_feedback(request_id, 0.)
    node._rl_transport.status.side_effect = RetryRequest('offline')

    def feedback(identity):
        node._rl_session.confirm(identity, {'duplicate': True})
        node._alive = False

    node._rl_transport.feedback.side_effect = feedback
    node._infer_loop()
    assert node._rl_session.get(request_id)['feedback_confirmed']
    assert node._rl_phase == 'unreachable'


def test_response_after_stop_is_recorded_but_not_executed(rl_bridge):
    import time
    from test_async_execution import prediction

    node = rl_bridge
    request_id = prepared(node._rl_session)
    node._rl_session.update(request_id, observation_time=time.time())

    def infer(identity):
        node._stop('Operator cancelled during HTTP')
        body = {'id': 'late-chunk', 'execution_chunk_size': 30}
        node._rl_session.receive(identity, body)
        return prediction(), body

    node._rl_transport.infer.side_effect = infer
    node._fetch(request_id)
    node._on_tick()
    assert node._rl_session.get(request_id)['state'] == 'skipped'
    assert node._rl_session.get(request_id)['id'] == 'late-chunk'
    node._publisher.publish.assert_not_called()


def test_real_http_multipart_retry_and_duplicate_feedback(tmp_path):
    from email import policy
    from email.parser import BytesParser
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    import threading
    import time

    from g1_vla_bridge.backends.cogact_unitree import CogACTUnitreeBackend
    from test_cogact_unitree_backend import _observation, action_response

    parts_received = []
    feedback_received = []

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            _ = args
            pass

        def respond(self, status, body):
            data = json.dumps(body).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            self.respond(200, {'phase': 'collecting', 'policy_version': 2})

        def do_POST(self):
            data = self.rfile.read(int(self.headers['Content-Length']))
            if self.path == '/api/feedback':
                feedback_received.append(json.loads(data))
                self.respond(200, {'id': 'http-chunk', 'duplicate': len(feedback_received) > 1})
                return
            message = BytesParser(policy=policy.default).parsebytes(
                b'MIME-Version: 1.0\r\nContent-Type: ' + self.headers['Content-Type'].encode()
                + b'\r\n\r\n' + data)
            parts_received.append({part.get_param('name', header='content-disposition'):
                                   part.get_payload(decode=True) for part in message.iter_parts()})
            if len(parts_received) == 1:
                self.respond(409, {'code': 'pending'})
            else:
                self.respond(200, dict(action_response(), id='http-chunk', policy_version=2,
                                       batch_id='batch-0', execution_chunk_size=30))

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    backend = CogACTUnitreeBackend(f'http://127.0.0.1:{server.server_port}/api/inference')
    backend._history_enabled = True
    journal = Journal(tmp_path)
    client = Transport(backend, journal)
    client.http.trust_env = False
    try:
        assert client.status()['phase'] == 'collecting'
        observation = _observation()
        observation.acquired_monotonic = time.monotonic()
        request_id = client.prepare(observation)
        query, images = journal.wire(request_id)
        with pytest.raises(RetryRequest):
            client.infer(request_id)
        chunk, _ = client.infer(request_id)
        assert chunk.horizon == 30
        assert parts_received[0] == parts_received[1]
        assert parts_received[0]['json'] == query
        assert tuple(parts_received[0][f'image_{index}'] for index in range(3)) == images
        journal.claim(request_id, 30)
        journal.finish(request_id, 30, True)
        journal.queue_feedback(request_id, score_reward('4'))
        client._request('POST', 'feedback', json=journal.get(request_id)['feedback'])
        client.feedback(request_id)
        assert feedback_received[0] == feedback_received[1]
        assert journal.get(request_id)['feedback_response']['duplicate']
    finally:
        client.close()
        backend.close()
        journal.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.)


def test_isolated_ros_initialization_and_shutdown(tmp_path):
    import os
    import subprocess
    import sys

    script = '''
import sys
from unittest.mock import Mock, patch
import rclpy
from rclpy.signals import SignalHandlerOptions
from g1_vla_bridge.backends.cogact_unitree import CogACTUnitreeBackend
from g1_vla_bridge.online_rl.bridge import OnlineRLBridge
from g1_vla_bridge.online_rl.cli import OnlineRLCli
backend = CogACTUnitreeBackend('http://127.0.0.1:1/api/inference')
backend._history_enabled = True
transport = Mock()
transport.status.return_value = {'phase': 'collecting'}
rclpy.init(args=['--ros-args', '-r', '__node:=online_rl_bridge',
                 '-p', 'rl_directory:=' + sys.argv[1],
                 '-p', 'command_topic:=/isolated_rl_test/command'],
           signal_handler_options=SignalHandlerOptions.NO)
try:
    with patch('g1_vla_bridge.vla_node.load_backend', return_value=backend), \\
         patch('g1_vla_bridge.vla_node.WristReader'), \\
         patch('g1_vla_bridge.online_rl.bridge.Transport', return_value=transport):
        node = OnlineRLBridge()
        assert node.get_name() == 'online_rl_bridge'
        assert node._publisher.topic_name == '/isolated_rl_test/command'
        assert not node._running.is_set()
        assert node._action_rate == node._observations.rate == 10.
        assert node._execution_rate == 30.
        node.shutdown()
        assert not node._worker.is_alive()
        node.destroy_node()
    cli = OnlineRLCli()
    cli.destroy_node()
finally:
    rclpy.shutdown()
print('isolated ROS initialization and shutdown passed')
'''
    environment = dict(os.environ, ROS_DOMAIN_ID='218', ROS_LOCALHOST_ONLY='1')
    result = subprocess.run([sys.executable, '-c', script, str(tmp_path)], env=environment,
                            capture_output=True, text=True, timeout=25.)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'isolated ROS initialization and shutdown passed' in result.stdout


def test_cli_consumes_buffered_multiline_input_without_fd_readiness(monkeypatch):
    from types import SimpleNamespace
    from g1_vla_bridge.online_rl import cli as module

    cli = object.__new__(module.OnlineRLCli)
    cli._status = {'task': ''}
    cli._rl = {'phase': 'collecting'}
    cli._submitted = None
    cli.retry_submission = Mock()
    cli.handle_line = Mock(side_effect=[True, True, False])
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    def spin_once(*args, **kwargs):
        _ = args, kwargs

    monkeypatch.setattr(module.rclpy, 'spin_once', spin_once)
    monkeypatch.setattr(module.sys, 'stdin', SimpleNamespace(fileno=lambda: 123))
    monkeypatch.setattr(module.select, 'select', Mock(return_value=([123], [], [])))
    read = Mock(return_value=b'pick up the cup\n\n/quit\n')
    monkeypatch.setattr(module.os, 'read', read)
    cli.run()
    assert [call.args[0] for call in cli.handle_line.call_args_list] == [
        'pick up the cup', '', '/quit']
    read.assert_called_once_with(123, 4096)
