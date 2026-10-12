"""VLA CLI 的输入语义：空行执行，文字只更新任务。"""


from unittest.mock import MagicMock, call

import pytest

from upper_vla.vla_cli import VlaCli, input_action, mode_command, switch_command


def cli_fixture(monkeypatch, status, *, refresh=True, accepted=True):
    cli = VlaCli.__new__(VlaCli)
    cli._status = status
    cli._start = MagicMock(name='start')
    cli._next = MagicMock(name='next')
    cli._set_auto = MagicMock(name='auto')
    cli._set_async = MagicMock(name='async')
    cli._set_skip = MagicMock(name='skip')
    request = MagicMock(return_value=True)
    request_bool = MagicMock(return_value=accepted)
    monkeypatch.setattr(cli, 'call', request)
    monkeypatch.setattr(cli, 'call_bool', request_bool)
    monkeypatch.setattr(cli, 'refresh_status', MagicMock(return_value=refresh))
    return cli, request, request_bool


@pytest.mark.parametrize(('line', 'expected'), (
    ('', ('next', '')),
    ('   ', ('next', '')),
    ('pick up the cup', ('task', 'pick up the cup')),
    ('  pick up the cup  ', ('task', 'pick up the cup')),
    ('/estop', ('command', '/estop')),
    ('/home', ('command', '/home')),
))
def test_input_action(line, expected):
    assert input_action(line) == expected


@pytest.mark.parametrize(('line', 'expected'), (
    ('/auto', ('/auto', None)),
    ('/auto on', ('/auto', True)),
    ('/AUTO OFF', ('/auto', False)),
    ('/skip true', ('/skip', True)),
    ('/skip false', ('/skip', False)),
))
def test_switch_command(line, expected):
    assert switch_command(line) == expected


def test_switch_command_rejects_bad_value():
    with pytest.raises(ValueError, match=r'on\|off'):
        switch_command('/auto maybe')


def test_status_accepts_valid_json():
    cli = object.__new__(VlaCli)
    cli._status = {}
    cli._on_status(MagicMock(data='{"task": "pick cup", "waiting_for_next": true}'))
    assert cli._status == {'task': 'pick cup', 'waiting_for_next': True}


def test_status_ignores_invalid_json():
    cli = object.__new__(VlaCli)
    cli._status = {'task': 'keep'}
    cli._on_status(MagicMock(data='bad'))
    assert cli._status == {'task': 'keep'}


@pytest.mark.parametrize(('running', 'expected'), (
    (False, [call('start', '待命'), call('next', '执行')]),
    (True, [call('next', '执行')]),
))
def test_enter_refreshes_state_before_request(monkeypatch, running, expected):
    cli, request, _ = cli_fixture(monkeypatch, {'running': not running})

    def refresh_status():
        cli._status = {'running': running, 'execution_mode': 'manual'}
        return True

    monkeypatch.setattr(cli, 'refresh_status', refresh_status)
    VlaCli.execute_one(cli)
    services = {'start': cli._start, 'next': cli._next}
    assert request.call_args_list == [call(services[item.args[0]], item.args[1]) for item in expected]


def test_enter_never_starts_continuous_mode(monkeypatch):
    cli, request, _ = cli_fixture(monkeypatch, {'running': False, 'execution_mode': 'continuous'})
    VlaCli.execute_one(cli)
    request.assert_not_called()


def test_enter_requires_status(monkeypatch):
    cli, request, _ = cli_fixture(monkeypatch, {'running': True}, refresh=False)
    VlaCli.execute_one(cli)
    request.assert_not_called()


def test_auto_toggle_starts_stopped_bridge(monkeypatch):
    cli, request, request_bool = cli_fixture(monkeypatch, {'running': False, 'execution_mode': 'manual'})
    VlaCli.set_auto(cli, None)
    request_bool.assert_called_once_with(cli._set_auto, True, '自动模式')
    request.assert_called_once_with(cli._start, '启动')


def test_auto_off_does_not_stop_current_execution(monkeypatch):
    cli, request, request_bool = cli_fixture(monkeypatch, {'running': True, 'execution_mode': 'continuous'})
    VlaCli.set_auto(cli, False)
    request_bool.assert_called_once_with(cli._set_auto, False, '自动模式')
    request.assert_not_called()


def test_skip_toggle_uses_latest_status(monkeypatch):
    cli, _, request_bool = cli_fixture(monkeypatch, {'skip_intermediate_waypoints': True})
    VlaCli.set_skip_intermediate(cli, None)
    request_bool.assert_called_once_with(cli._set_skip, False, '末点直达')


@pytest.mark.parametrize('mode', ['manual', 'continuous', 'async'])
def test_mode_command(mode):
    assert mode_command(f'/MODE {mode.upper()}') == mode


@pytest.mark.parametrize('line', ['/mode', '/mode bogus', '/mode async extra'])
def test_mode_command_rejects_invalid(line):
    with pytest.raises(ValueError, match='用法'):
        mode_command(line)


@pytest.mark.parametrize(('mode', 'client', 'enabled'), [
    ('manual', 'auto', False), ('continuous', 'auto', True), ('async', 'async', True)])
def test_mode_routes_to_service_without_starting(monkeypatch, mode, client, enabled):
    cli, request, request_bool = cli_fixture(monkeypatch, {'running': False})
    VlaCli.set_mode(cli, mode)
    service = cli._set_auto if client == 'auto' else cli._set_async
    request_bool.assert_called_once_with(service, enabled, f'执行模式 {mode}')
    request.assert_not_called()


def test_mode_requires_stop(monkeypatch):
    cli, _, request_bool = cli_fixture(monkeypatch, {'running': True, 'execution_mode': 'manual'})
    VlaCli.set_mode(cli, 'async')
    request_bool.assert_not_called()


def test_auto_rejection_never_starts_bridge(monkeypatch):
    cli, request, _ = cli_fixture(monkeypatch, {'running': False, 'execution_mode': 'async'}, accepted=False)
    VlaCli.set_auto(cli, True)
    request.assert_not_called()


def test_home_cli_calls_service_without_start_or_inference(monkeypatch):
    from upper_vla import vla_cli

    cli = MagicMock()
    inputs = iter(['/home', '/quit'])
    monkeypatch.setattr(vla_cli, 'VlaCli', lambda: cli)

    def input_value(prompt):
        _ = prompt
        return next(inputs)

    monkeypatch.setattr('builtins.input', input_value)
    monkeypatch.setattr(vla_cli.rclpy, 'init', MagicMock())
    monkeypatch.setattr(vla_cli.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(vla_cli.rclpy, 'shutdown', MagicMock())
    vla_cli.main()
    cli.call.assert_called_once_with(cli._home, '复位')
    cli.execute_one.assert_not_called()
    cli.set_task.assert_not_called()
    cli.destroy_node.assert_called_once()
