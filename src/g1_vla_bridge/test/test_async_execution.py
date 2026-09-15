"""Exercise bridge callbacks with a fake clock and a mock publisher."""

import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from g1_vla_bridge import vla_node
from g1_vla_bridge.backends.cogact_unitree import SPEC
from g1_vla_bridge.timed_actions import TimedActions
from g1_vla_bridge.vla_backend import ActionChunk, SIDES
from g1_vla_bridge.vla_node import VlaBridgeNode


def prediction(position=0.):
    return ActionChunk(
        poses={side: np.tile([position, 0., 0., 0., 0., 0., 1.], (30, 1)) for side in SIDES},
        grippers={side: np.ones(30) for side in SIDES})


def test_ten_hz_execution_preserves_thirty_hz_prediction_time():
    queue = TimedActions(30., .5, 0.)
    chunk = prediction()
    for side in SIDES:
        chunk.poses[side][:, 0] = np.arange(30)
    queue.merge(chunk, 0., .2)
    assert queue.end == 1.
    positions = []
    for now in (.2, .3, .4, .5, .6, .7, .8, .9):
        poses, _ = queue.take(now)
        positions.append(poses['left'][0])
    np.testing.assert_allclose(positions, [6, 9, 12, 15, 18, 21, 24, 27])
    assert queue.take(1.) is None


@pytest.fixture
def bridge(monkeypatch):
    clock = SimpleNamespace(now=0.)
    monkeypatch.setattr(vla_node.time, 'monotonic', lambda: clock.now)
    node = SimpleNamespace(
        _lock=threading.Lock(), _running=threading.Event(), _infer_requested=threading.Event(),
        _inference_active=False, _generation=0, _execution_mode='async', _chunk=None,
        _cursor=0, _timed=TimedActions(30., .5, 0.), _async_timeout=1., _horizon=0,
        _command={side: np.array([0., 0., 0., 0., 0., 0., 1.]) for side in SIDES},
        _grip_command={side: 0. for side in SIDES}, _active={side: True for side in SIDES},
        _publisher=Mock(), _arms_ready=lambda: '', _cartesian_limit_enabled=False,
        _delta=False, _spec=SPEC, _skip_intermediate=False, _error='',
        _max_step_pos=.02, _max_step_ori=.1, get_logger=Mock(return_value=Mock()),
        _retry_delay=0., _action_rate=30., _async_alpha=.5, _task='test',
        _observe=lambda: object(), _alive=True)
    node._running.set()
    node._measured_pose = lambda side: node._command[side].copy()
    node._request_inference = lambda generation=None: VlaBridgeNode._request_inference(node, generation)
    node._accept = lambda *args, **kwargs: VlaBridgeNode._accept(node, *args, **kwargs)
    node._fail = lambda *args: VlaBridgeNode._fail(node, *args)
    node._stop = lambda reason: VlaBridgeNode._stop(node, reason)
    node._on_async_tick = lambda: VlaBridgeNode._on_async_tick(node)
    node._limit = lambda current, target: VlaBridgeNode._limit(node, current, target)
    node._mode_error = lambda mode: VlaBridgeNode._mode_error(node, mode)
    node._set_execution_mode = lambda mode: VlaBridgeNode._set_execution_mode(node, mode)
    return node, clock


def test_accept_requests_again_before_playback_and_tick_uses_ema(bridge):
    node, clock = bridge
    clock.now = .2
    node._accept(prediction(0.), 200., 0, requested_at=0.)
    assert node._inference_active and node._infer_requested.is_set()
    assert node._request_inference() == '正在推理'
    assert len(node._timed.samples) == 24
    node._publisher.publish.assert_not_called()
    clock.now = .4
    node._accept(prediction(4.), 200., 0, requested_at=.2)
    VlaBridgeNode._on_tick(node)
    assert node._command['left'][0] == 2.
    assert node._grip_command['left'] == 1.
    assert node._publisher.publish.call_count == 2
    assert node._inference_active


def test_regular_thirty_hz_ticks_still_stall_with_short_prediction_tail(bridge):
    node, clock = bridge
    published_ticks = []
    overlaps = []
    for tick in range(73):
        clock.now = tick / 30.
        if tick in (27, 48, 69):
            node._accept(prediction(), 700., 0, requested_at=(tick - 27) / 30.)
            overlaps.append(node._async_merge['overlap'])
            assert node._async_merge['accepted'] == 3
        previous_count = node._publisher.publish.call_count
        VlaBridgeNode._on_tick(node)
        if node._publisher.publish.call_count > previous_count:
            published_ticks.append(tick)
    assert published_ticks == [27, 28, 29, 48, 49, 50, 69, 70, 71]
    assert overlaps == [0, 0, 0]
    assert node._running.is_set()
    assert max(np.diff(published_ticks)) / 30. == pytest.approx(19 / 30.)


def test_stop_preserves_last_merge_diagnostics(bridge):
    node, clock = bridge
    clock.now = .2
    node._accept(prediction(), 200., 0, requested_at=0.)
    node._stop('operator stopped')
    assert node._timed is None
    assert node._async_merge['accepted'] == 24
    assert node._async_merge['overlap'] == 0


def test_tick_holds_then_stops_and_discards_late_response(bridge):
    node, clock = bridge
    clock.now = .2
    node._accept(prediction(), 200., 0, requested_at=0.)
    clock.now = 1.2
    VlaBridgeNode._on_tick(node)
    assert node._running.is_set()
    node._publisher.publish.assert_not_called()
    clock.now = 2.
    VlaBridgeNode._on_tick(node)
    assert not node._running.is_set()
    assert '超时' in node._error
    assert node._timed is None
    node._accept(prediction(4.), 1800., 0, requested_at=.2)
    assert not node._infer_requested.is_set()
    assert node._timed is None


def test_response_after_deadline_cannot_revive_queue(bridge):
    node, clock = bridge
    clock.now = 1.1
    with pytest.raises(RuntimeError, match='超时'):
        node._accept(prediction(), 200., 0, requested_at=.9)
    assert node._timed.end == 0.


def test_expired_response_does_not_extend_deadline(bridge):
    node, clock = bridge
    node._async_timeout = 2.
    clock.now = 1.1
    node._accept(prediction(), 1100., 0, requested_at=0.)
    assert not node._timed.samples
    assert node._timed.end == 0.
    assert '过期' in node._error
    assert node._inference_active


def test_async_starts_requests_and_mode_change_invalidates_results(bridge):
    node, clock = bridge
    node._stop('test')
    assert node._set_execution_mode('manual') == ''
    response = VlaBridgeNode._on_set_async(node, SimpleNamespace(data=True), SimpleNamespace())
    assert response.success
    generation = node._generation
    response = VlaBridgeNode._on_start(node, None, SimpleNamespace())
    assert response.success and node._infer_requested.is_set()
    clock.now = .2
    node._accept(prediction(10.), 200., generation, requested_at=0.)
    assert not node._timed.samples
    assert node._inference_active


@pytest.mark.parametrize('mode', ['manual', 'continuous'])
def test_running_mode_change_is_rejected(bridge, mode):
    node, _ = bridge
    assert 'stop' in node._set_execution_mode(mode)
    assert node._execution_mode == 'async'


@pytest.mark.parametrize('invalid', ['delta', 'skip', 'semantics'])
def test_incompatible_mode_configuration_is_rejected(bridge, invalid):
    node, _ = bridge
    node._running.clear()
    node._execution_mode = 'manual'
    if invalid == 'delta':
        node._delta = True
    elif invalid == 'skip':
        node._skip_intermediate = True
    else:
        node._spec = SimpleNamespace(action_semantics='delta')
    assert node._set_execution_mode('async')
    assert node._execution_mode == 'manual'


def test_skip_is_rejected_in_async(bridge):
    node, _ = bridge
    response = VlaBridgeNode._on_set_skip_intermediate(
        node, SimpleNamespace(data=True), SimpleNamespace())
    assert not response.success
    assert not node._skip_intermediate


def test_async_keeps_limits_and_held_arm(bridge):
    node, clock = bridge
    node._cartesian_limit_enabled = True
    node._active['left'] = False
    clock.now = .2
    node._accept(prediction(1.), 200., 0, requested_at=0.)
    VlaBridgeNode._on_tick(node)
    assert node._command['right'][0] == .02
    assert node._command['left'][0] == 0.
    assert node._grip_command['left'] == 0.
    assert node._async_step == {'right': {
        'position_m': 1., 'rotation_rad': 0., 'gripper_rad': 1.}}
    assert node._async_last_publish == .2


def test_worker_immediately_observes_again_without_ticks(bridge):
    node, clock = bridge
    observed = []

    def observe():
        observed.append(clock.now)
        return SimpleNamespace(acquired_monotonic=clock.now)

    node._observe = observe

    def infer(observation):
        clock.now += .2
        if len(observed) == 2:
            node._alive = False
        return prediction(float(len(observed)))

    node._backend = SimpleNamespace(infer=infer)
    node._request_inference()
    VlaBridgeNode._infer_loop(node)
    assert observed == [0., .2]
    assert len(node._timed.samples) == 24
    assert node._timed.samples[12][0]['left'][0] == 1.5
    node._publisher.publish.assert_not_called()


def test_async_failure_preserves_buffer_and_retries(bridge):
    node, clock = bridge
    clock.now = .2
    node._accept(prediction(), 200., 0, requested_at=0.)
    queue = node._timed
    node._infer_requested.clear()
    node._fail('HTTP failed', 0)
    assert node._timed is queue and len(queue.samples) == 24
    assert node._infer_requested.is_set()
    assert node._error == 'HTTP failed'


def test_worker_counts_image_age_as_well_as_inference_latency(bridge):
    node, clock = bridge
    clock.now = .1
    node._observe = lambda: SimpleNamespace(acquired_monotonic=0.)

    def infer(observation):
        clock.now = .3
        node._alive = False
        return prediction()

    node._backend = SimpleNamespace(infer=infer)
    node._request_inference()
    VlaBridgeNode._infer_loop(node)
    assert sorted(node._timed.samples) == list(range(9, 30))
    assert node._infer_ms == pytest.approx(200.)
    assert node._timed.end == pytest.approx(1.)


def test_async_missing_acquisition_time_never_calls_backend(bridge):
    node, _ = bridge
    node._observe = lambda: SimpleNamespace(acquired_monotonic=None)
    node._backend = Mock()

    def fail(reason, generation):
        node._error = reason
        node._alive = False

    node._fail = fail
    node._request_inference()
    VlaBridgeNode._infer_loop(node)
    node._backend.infer.assert_not_called()
    assert '观测获取时间' in node._error
