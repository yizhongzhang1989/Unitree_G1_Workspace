"""VLA CLI 的输入语义：空行执行，文字只更新任务。"""

from types import SimpleNamespace
from unittest.mock import MagicMock, call, patch

import pytest

from g1_vla_bridge.vla_cli import VlaCli, input_action


@pytest.mark.parametrize(('line', 'expected'), (
    ('', ('next', '')),
    ('   ', ('next', '')),
    ('pick up the cup', ('task', 'pick up the cup')),
    ('  pick up the cup  ', ('task', 'pick up the cup')),
    ('/estop', ('command', '/estop')),
))
def test_input_action(line, expected):
    assert input_action(line) == expected


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

    def spin(node, **kwargs):
        node._status = {'running': running, 'execution_mode': 'manual'}

    with patch('g1_vla_bridge.vla_cli.rclpy.spin_once', side_effect=spin):
        VlaCli.execute_one(cli)
    assert cli.call.call_args_list == expected


def test_enter_never_starts_continuous_mode():
    cli = SimpleNamespace(_status={}, call=MagicMock())

    def spin(node, **kwargs):
        node._status = {'running': False, 'execution_mode': 'continuous'}

    with patch('g1_vla_bridge.vla_cli.rclpy.spin_once', side_effect=spin):
        VlaCli.execute_one(cli)
    cli.call.assert_not_called()


def test_enter_requires_status():
    cli = SimpleNamespace(_status={'running': True}, call=MagicMock())
    with patch('g1_vla_bridge.vla_cli.time.monotonic', side_effect=[0., 3.]):
        VlaCli.execute_one(cli)
    cli.call.assert_not_called()
