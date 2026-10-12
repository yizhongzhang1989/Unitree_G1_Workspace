"""Keyboard task feedback and lower commands without a ROS graph."""

import json
from unittest.mock import Mock

import pytest

from std_msgs.msg import Float64MultiArray, String

from lower_velocity_tracking.teleop_keyboard import Teleop


def _teleop(monkeypatch):
    node = Teleop.__new__(Teleop)
    node._status = ''
    node.notice = ''
    node._vel = [0.0, 0.0, 0.0]
    node._height = 0.74
    node._idle = 0.0
    node._hold_timeout = 0.4
    node._decay_s = 0.5
    node._message = Float64MultiArray(data=[0., 0., 0., .74])
    node._step = (0.1, 0.1, 0.1)
    node._limit = (0.4, 0.3, 1.0)
    node._h_step = 0.01
    node._h_range = (0.5, 0.8)
    sent: list[list[float]] = []

    def publish(message: Float64MultiArray) -> None:
        sent.append(list(message.data))

    node._publisher = Mock(publish=Mock(side_effect=publish))
    request = Mock()
    monkeypatch.setattr(node, 'request', request)
    node._estop_client = Mock()
    return node, sent, request


def _status(node, **fields):
    node._on_status(String(data=json.dumps(
        {'state': 'running', 'upper_executor': 'float', **fields})))


@pytest.mark.parametrize('key,axis,expected', [('w', 0, 0.4), ('a', 1, 0.3), ('j', 2, 1.0)])
def test_velocity_keys_clamp_and_decay(monkeypatch, key, axis, expected):
    node, sent, request = _teleop(monkeypatch)
    for _ in range(20):
        node.key(key)
    node.tick(0.02)
    assert sent[-1][axis] == pytest.approx(expected)
    node.tick(0.5)
    assert sent[-1][axis] == 0.0
    assert sent[-1][3] == 0.74
    request.assert_not_called()


def test_estop_zeros_lower_commands(monkeypatch):
    node, sent, request = _teleop(monkeypatch)
    _status(node)
    node.key('w')
    node.key(' ')
    node.tick(0.02)
    assert sent == [[0.0, 0.0, 0.0, 0.74]]
    request.assert_called_once_with(node._estop_client, '急停')
