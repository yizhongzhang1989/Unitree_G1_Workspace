"""Exercise bridge callbacks with a fake clock and a mock publisher."""


from types import SimpleNamespace
from unittest.mock import Mock
from dataclasses import replace
from rclpy.clock import Clock
from rclpy.time import Time
from std_srvs.srv import SetBool, Trigger

import numpy as np
import pytest

from upper_vla import vla_node
from upper_vla.control_history import ControlHistory
from upper_vla.record_observation import ObservationBuffer
from test_history_execution import deliver_state
from test_execution_regression import executor_fixture
from upper_vla.timed_actions import TimedActions
from upper_vla.vla_backend import ActionChunk, Observation, SIDES, VlaBackend
from upper_vla.vla_node import VlaBridgeNode


def prediction(position=0.):
    return ActionChunk(
        poses={side: np.tile([position, 0., 0., 0., 0., 0., 1.], (30, 1)) for side in SIDES},
        grippers={side: np.ones(30) for side in SIDES})


def test_ten_hz_execution_preserves_thirty_hz_prediction_time():
    queue = TimedActions(30., 0.)
    chunk = prediction()
    for side in SIDES:
        chunk.poses[side][:, 0] = np.arange(30)
    queue.merge(chunk, 0., .2)
    assert queue.end == 1.
    positions = []
    for now in (.2, .3, .4, .5, .6, .7, .8, .9):
        sample = queue.take(now)
        assert sample is not None
        poses, _ = sample
        positions.append(poses['left'][0])
    np.testing.assert_allclose(positions, [6, 9, 12, 15, 18, 21, 24, 27])
    assert queue.take(1.) is None


@pytest.fixture
def bridge(monkeypatch):
    clock = SimpleNamespace(now=0.)
    monkeypatch.setattr(vla_node.time, 'monotonic', lambda: clock.now)
    node = executor_fixture('async')
    node._chunk = None
    node._timed = TimedActions(30., 0.)
    node._cartesian_limit_enabled = False
    node._retry_delay = 0.
    node._action_rate = 30.
    node._execution_rate = 30.
    node._task = 'test'
    node._alive = True
    node.observe_spy = Mock(return_value=Observation(
        'test', {}, {}, {'left': .7, 'right': .8}, {}))
    monkeypatch.setattr(node, '_observe', node.observe_spy)
    del node._request_inference
    monkeypatch.setattr(node, '_measured_pose', lambda side: node._command[side].copy())
    return node, clock


@pytest.mark.parametrize('held', SIDES)
def test_start_preserves_inactive_gripper_through_publication(bridge, held):
    node, clock = bridge
    node._stop('setup')
    node._active[held] = False
    measured = node._observe().grippers[held]
    assert VlaBridgeNode._on_start(node, Trigger.Request(), Trigger.Response()).success
    assert node._grip_command[held] == measured
    clock.now = .2
    node._accept(prediction(), 200., node._generation, requested_at=0.)
    VlaBridgeNode._on_tick(node)
    assert node._grip_command[held] == measured


def test_accept_requests_again_before_playback_and_tick_uses_ramp(bridge):
    node, clock = bridge
    clock.now = .2
    node._accept(prediction(0.), 200., 0, requested_at=0.)
    assert node._inference_active and node._infer_requested.is_set()
    assert node._request_inference() == '正在推理'
    assert node._timed is not None
    assert len(node._timed.samples) == 24
    node.publish_control_spy.assert_not_called()
    clock.now = .4
    node._accept(prediction(4.), 200., 0, requested_at=.2)
    VlaBridgeNode._on_tick(node)
    assert node._command['left'][0] == pytest.approx(4. / 19.)
    assert node._grip_command['left'] == 1.
    assert node.publish_control_spy.call_count == 1
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
        previous_count = node.publish_control_spy.call_count
        VlaBridgeNode._on_tick(node)
        if node.publish_control_spy.call_count > previous_count:
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


def test_async_thirty_hz_publications_and_history(bridge):
    node, clock = bridge
    node._stop('test setup')
    node._action_rate, node._execution_rate = 10., 30.
    node._history = ControlHistory()
    pytest.MonkeyPatch().setattr(node, 'get_clock', Mock(return_value=Mock(
        spec=Clock, now=Mock(side_effect=lambda: Time(nanoseconds=round(clock.now * 1e9))))))
    node._observations = ObservationBuffer(())

    def control_measurement(joints):
        _ = joints
        return ({side: node._command[side].copy() for side in SIDES},
                dict(node._grip_command))

    pytest.MonkeyPatch().setattr(node, '_control_measurement', control_measurement)
    assert VlaBridgeNode._on_start(node, Trigger.Request(), Trigger.Response()).success
    assert node._timed is not None
    assert node._timed.execution_rate == 30.
    chunk = prediction()
    for side in SIDES:
        chunk.poses[side][:, 0] = np.arange(1, 31)
        chunk.grippers[side][:] = np.arange(1, 31) / 30.
    clock.now = .2
    node._accept(chunk, 200., node._generation, requested_at=0.)
    expected_actions = TimedActions(
        10., 0., minimum_overlap_actions=7, first_offset_steps=1,
        execution_rate=30.)
    expected_actions.merge(chunk, 0., .2, fallback_poses={
        side: np.array([0., 0., 0., 0., 0., 0., 1.]) for side in SIDES})
    expected = []
    for tick in range(6, 91):
        clock.now = tick / 30.
        VlaBridgeNode._on_tick(node)
        deliver_state(node, clock.now + .001)
        sample = expected_actions.take(clock.now)
        assert sample is not None
        expected.append(sample[0]['left'][0])
    sent = [call.args[0] for call in node.publish_control_spy.call_args_list]
    np.testing.assert_allclose([command['left'][0] for command in sent], expected)
    np.testing.assert_allclose([call.args[1]['left'] for call in node.publish_control_spy.call_args_list],
                               np.arange(6, 91) / 90.)
    rows = node._history.snapshot()
    assert len(rows) == 16
    np.testing.assert_allclose([row.action['left'][0] for row in rows], np.arange(45, 91, 3) / 3.)
    np.testing.assert_allclose(np.diff([row.action_stamp for row in rows]), .1, atol=1e-9)
    clock.now = 10.
    VlaBridgeNode._on_tick(node)
    assert node.publish_control_spy.call_count == 85
    assert node._running.is_set()


@pytest.mark.parametrize('prior_chunk', [False, True])
def test_empty_queue_holds_and_accepts_nonoverlapping_response(bridge, prior_chunk):
    node, clock = bridge
    node._timed = TimedActions(10., 0., first_offset_steps=1)
    if prior_chunk:
        clock.now = .2
        node._accept(prediction(1.), 200., 0, requested_at=0.)
        VlaBridgeNode._on_tick(node)
    held = node._command['left'].copy()
    published = node.publish_control_spy.call_count
    clock.now = 10.
    VlaBridgeNode._on_tick(node)
    assert node._running.is_set()
    assert node.publish_control_spy.call_count == published
    np.testing.assert_array_equal(node._command['left'], held)
    node._accept(prediction(4.), 1000., 0, requested_at=9.)
    assert node._async_merge['overlap'] == 0
    assert node._async_merge['accepted'] > 0
    assert node._inference_active and node._infer_requested.is_set()
    VlaBridgeNode._on_tick(node)
    assert node.publish_control_spy.call_count == published + 1
    clock.now = 10.1
    VlaBridgeNode._on_tick(node)
    assert node.publish_control_spy.call_count == published + 2
    assert node._command['left'][0] == 4.
    node._stop('operator stopped')
    node._accept(prediction(8.), 100., 0, requested_at=10.)
    assert not node._infer_requested.is_set()
    assert node._timed is None


def test_expired_response_keeps_waiting_and_requesting(bridge):
    node, clock = bridge
    clock.now = 1.1
    node._accept(prediction(), 1100., 0, requested_at=0.)
    assert node._timed is not None
    assert not node._timed.samples
    assert node._timed.end == 0.
    assert '过期' in node._error
    assert node._inference_active
    clock.now = 10.
    VlaBridgeNode._on_tick(node)
    assert node._running.is_set()
    node.publish_control_spy.assert_not_called()


def test_async_starts_requests_and_mode_change_invalidates_results(bridge):
    node, clock = bridge
    node._stop('test')
    assert node._set_execution_mode('manual') == ''
    response = VlaBridgeNode._on_set_async(node, SetBool.Request(data=True), SetBool.Response())
    assert response.success
    generation = node._generation
    response = VlaBridgeNode._on_start(node, Trigger.Request(), Trigger.Response())
    assert response.success and node._infer_requested.is_set()
    clock.now = .2
    node._accept(prediction(10.), 200., generation, requested_at=0.)
    assert node._timed is not None
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
        node._spec = replace(node._spec, action_semantics='delta')
    assert node._set_execution_mode('async')
    assert node._execution_mode == 'manual'


def test_skip_is_rejected_in_async(bridge):
    node, _ = bridge
    response = VlaBridgeNode._on_set_skip_intermediate(
        node, SetBool.Request(data=True), SetBool.Response())
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
    observation = node._observe()

    def observe():
        observed.append(clock.now)
        observation.acquired_monotonic = clock.now
        return observation

    node.observe_spy.side_effect = observe

    def infer(observation):
        _ = observation
        clock.now += .2
        if len(observed) == 2:
            node._alive = False
        return prediction(float(len(observed)))

    node._backend = Mock(spec=VlaBackend, infer=Mock(side_effect=infer))
    node._request_inference()
    VlaBridgeNode._infer_loop(node)
    assert observed == [0., .2]
    assert node._timed is not None
    assert len(node._timed.samples) == 24
    assert node._timed.samples[12][0]['left'][0] == pytest.approx(1. + 1. / 19.)
    node.publish_control_spy.assert_not_called()


def test_async_failure_preserves_buffer_and_retries(bridge):
    node, clock = bridge
    clock.now = .2
    node._accept(prediction(), 200., 0, requested_at=0.)
    queue = node._timed
    assert queue is not None
    node._infer_requested.clear()
    node._fail('HTTP failed', 0)
    assert node._timed is queue and len(queue.samples) == 24
    assert node._infer_requested.is_set()
    assert node._error == 'HTTP failed'


def test_worker_counts_image_age_as_well_as_inference_latency(bridge):
    node, clock = bridge
    clock.now = .1
    node.observe_spy.return_value.acquired_monotonic = 0.

    def infer(observation):
        _ = observation
        clock.now = .3
        node._alive = False
        return prediction()

    node._backend = Mock(spec=VlaBackend, infer=Mock(side_effect=infer))
    node._request_inference()
    VlaBridgeNode._infer_loop(node)
    assert node._timed is not None
    assert sorted(node._timed.samples) == list(range(9, 30))
    assert node._infer_ms == pytest.approx(200.)
    assert node._timed.end == pytest.approx(1.)


def test_async_missing_acquisition_time_never_calls_backend(bridge):
    node, _ = bridge
    node.observe_spy.return_value.acquired_monotonic = None
    backend_spy = Mock(spec=VlaBackend)
    node._backend = backend_spy

    def fail(reason, generation):
        _ = generation
        node._error = reason
        node._alive = False

    pytest.MonkeyPatch().setattr(node, '_fail', fail)
    node._request_inference()
    VlaBridgeNode._infer_loop(node)
    backend_spy.infer.assert_not_called()
    assert '观测获取时间' in node._error
