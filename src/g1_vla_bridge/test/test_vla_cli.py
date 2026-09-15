"""VLA CLI 的输入语义：空行执行，文字只更新任务。"""

from types import SimpleNamespace
from unittest.mock import MagicMock, call

import pytest

from g1_vla_bridge.vla_cli import VlaCli, input_action, mode_command, switch_command


@pytest.mark.parametrize(('line', 'expected'), (
    ('', ('next', '')),
    ('   ', ('next', '')),
    ('pick up the cup', ('task', 'pick up the cup')),
    ('  pick up the cup  ', ('task', 'pick up the cup')),
    ('/estop', ('command', '/estop')),
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
def test_enter_refreshes_state_before_request(running, expected):
    cli = SimpleNamespace(_status={'running': not running},
                          _start='start', _next='next', call=MagicMock(return_value=True))

    def refresh_status():
        cli._status = {'running': running, 'execution_mode': 'manual'}
        return True

    cli.refresh_status = refresh_status
    VlaCli.execute_one(cli)
    assert cli.call.call_args_list == expected


def test_enter_never_starts_continuous_mode():
    cli = SimpleNamespace(_status={}, call=MagicMock(),
                          refresh_status=MagicMock(return_value=True))

    cli._status = {'running': False, 'execution_mode': 'continuous'}
    VlaCli.execute_one(cli)
    cli.call.assert_not_called()


def test_enter_requires_status():
    cli = SimpleNamespace(_status={'running': True}, call=MagicMock(),
                          refresh_status=MagicMock(return_value=False))
    VlaCli.execute_one(cli)
    cli.call.assert_not_called()


def test_auto_toggle_starts_stopped_bridge():
    cli = SimpleNamespace(
        _status={'running': False, 'execution_mode': 'manual'},
        _set_auto='auto', _start='start',
        refresh_status=MagicMock(return_value=True),
        call_bool=MagicMock(return_value=True), call=MagicMock(return_value=True))
    VlaCli.set_auto(cli, None)
    cli.call_bool.assert_called_once_with('auto', True, '自动模式')
    cli.call.assert_called_once_with('start', '启动')


def test_auto_off_does_not_stop_current_execution():
    cli = SimpleNamespace(
        _status={'running': True, 'execution_mode': 'continuous'},
        _set_auto='auto', _start='start',
        refresh_status=MagicMock(return_value=True),
        call_bool=MagicMock(return_value=True), call=MagicMock())
    VlaCli.set_auto(cli, False)
    cli.call_bool.assert_called_once_with('auto', False, '自动模式')
    cli.call.assert_not_called()


def test_skip_toggle_uses_latest_status():
    cli = SimpleNamespace(
        _status={'skip_intermediate_waypoints': True}, _set_skip='skip',
        refresh_status=MagicMock(return_value=True), call_bool=MagicMock())
    VlaCli.set_skip_intermediate(cli, None)
    cli.call_bool.assert_called_once_with('skip', False, '末点直达')


@pytest.mark.parametrize('mode', ['manual', 'continuous', 'async'])
def test_mode_command(mode):
    assert mode_command(f'/MODE {mode.upper()}') == mode


@pytest.mark.parametrize('line', ['/mode', '/mode bogus', '/mode async extra'])
def test_mode_command_rejects_invalid(line):
    with pytest.raises(ValueError, match='用法'):
        mode_command(line)


@pytest.mark.parametrize(('mode', 'client', 'enabled'), [
    ('manual', 'auto', False), ('continuous', 'auto', True), ('async', 'async', True)])
def test_mode_routes_to_service_without_starting(mode, client, enabled):
    cli = SimpleNamespace(
        _status={'running': False}, _set_auto='auto', _set_async='async',
        refresh_status=MagicMock(return_value=True), call_bool=MagicMock(), call=MagicMock())
    VlaCli.set_mode(cli, mode)
    cli.call_bool.assert_called_once_with(client, enabled, f'执行模式 {mode}')
    cli.call.assert_not_called()


def test_mode_requires_stop():
    cli = SimpleNamespace(
        _status={'running': True, 'execution_mode': 'manual'},
        refresh_status=MagicMock(return_value=True), call_bool=MagicMock())
    VlaCli.set_mode(cli, 'async')
    cli.call_bool.assert_not_called()


def test_auto_rejection_never_starts_bridge():
    cli = SimpleNamespace(
        _status={'running': False, 'execution_mode': 'async'},
        _set_auto='auto', _start='start', refresh_status=MagicMock(return_value=True),
        call_bool=MagicMock(return_value=False), call=MagicMock())
    VlaCli.set_auto(cli, True)
    cli.call.assert_not_called()
