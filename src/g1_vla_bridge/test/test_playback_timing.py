"""动作 chunk 必须从第 0 点完整播放到最后一点。"""

import pytest

from g1_vla_bridge.vla_node import inference_request_error, playback_step


@pytest.mark.parametrize(('cursor', 'horizon', 'expected'), (
    (0, 30, (0, 1, False)),
    (1, 30, (1, 2, False)),
    (28, 30, (28, 29, False)),
    (29, 30, (29, 29, True)),
    (30, 30, (29, 29, True)),
    (-1, 30, (0, 1, False)),
))
def test_playback_step(cursor, horizon, expected):
    assert playback_step(cursor, horizon) == expected


def test_playback_step_rejects_empty_horizon():
    with pytest.raises(ValueError, match='horizon'):
        playback_step(0, 0)


def test_playback_visits_every_waypoint_once():
    cursor = 0
    visited = []
    while True:
        index, cursor, finished = playback_step(cursor, 30)
        visited.append(index)
        if finished:
            break
    assert visited == list(range(30))


def test_playback_can_skip_directly_to_final_waypoint():
    assert playback_step(0, 30, skip_intermediate=True) == (29, 29, True)


@pytest.mark.parametrize(('running', 'active', 'has_chunk', 'expected'), (
    (False, False, False, '尚未 start'),
    (True, True, False, '正在推理'),
    (True, False, True, '当前 chunk 尚未执行完'),
    (True, False, False, ''),
))
def test_inference_request_state(running, active, has_chunk, expected):
    assert inference_request_error(running, active, has_chunk) == expected
