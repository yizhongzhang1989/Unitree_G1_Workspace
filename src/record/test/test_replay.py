"""末端回放的 schema、插值及隔离 ROS 测试"""

import json
import sys
import threading
import time
import uuid
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from record.replay import Playback, lerp_arm, load_commands, quat_slerp, ramp
from record.replay_source import describe, open_session


def _poses(count=3):
    poses = np.zeros((count, 14))
    poses[:, 0] = np.arange(count) * .01
    poses[:, 7] = np.arange(count) * -.01
    poses[:, 6] = poses[:, 13] = 1.
    return poses


def _session(root, columns=None, sealed=True, poses=None):
    if columns is None:
        columns = [f'cmd.{index}' for index in range(20)]
    (root / 'signals').mkdir(parents=True)
    (root / 'manifest.json').write_text('{"streams": {}}')
    (root / 'meta.json').write_text('{"note": "test"}')
    schema = {'tables': {'motion_control_command': {
        'file': 'commands.bin', 'ncol': len(columns) + 2,
        'columns': ['t_recv', 't_header'] + columns}}}
    (root / 'schema.json').write_text(json.dumps(schema))
    (root / 'events.jsonl').write_text('')
    if sealed:
        (root / 'DONE').write_text('{"files": {}}')
    data = np.zeros((3, len(columns) + 2))
    data[:, 0] = [100., 101., 102.]
    data[:, 1] = np.nan
    data[:, 2:6] = np.nan
    upper = np.column_stack((_poses() if poses is None else poses,
                             [[.1, .2], [.2, .3], [.3, .4]]))
    data[:, 6:min(len(columns) + 2, 22)] = upper[:, :max(0, min(len(columns) - 4, 16))]
    data.tofile(root / 'signals/commands.bin')
    return root, data


def test_command_pose_grip_and_base_nan(tmp_path):
    root, data = _session(tmp_path / 'session')
    data[1, 6] = np.nan
    data.tofile(root / 'signals/commands.bin')
    timestamps, arms, grips = load_commands(open_session(root))
    np.testing.assert_array_equal(timestamps, [100., 102.])
    np.testing.assert_array_equal(arms, _poses()[[0, 2]])
    np.testing.assert_array_equal(grips, [[.1, .2], [.3, .4]])


def test_legacy_command_extra_columns(tmp_path):
    root, data = _session(tmp_path / 'session', [f'cmd.{index}' for index in range(39)])
    data[:, 22:] = np.nan
    data.tofile(root / 'signals/commands.bin')
    timestamps, arms, grips = load_commands(open_session(root))
    assert timestamps.size == 3
    np.testing.assert_array_equal(arms, _poses())
    np.testing.assert_array_equal(grips, [[.1, .2], [.2, .3], [.3, .4]])


def test_sealed_and_complete_pose_schema(tmp_path):
    root, _ = _session(tmp_path / 'session', sealed=False)
    with pytest.raises(ValueError, match='DONE'):
        load_commands(open_session(root))
    (root / 'DONE').write_text('{"files": {}}')
    document = json.loads((root / 'schema.json').read_text())
    document['tables']['motion_control_command']['ncol'] = 21
    (root / 'schema.json').write_text(json.dumps(document))
    with pytest.raises(ValueError, match='20 列'):
        load_commands(open_session(root))


@pytest.mark.parametrize('invalid', ['time', 'pose', 'grip'])
def test_command_invalid_rows_and_empty_start(tmp_path, invalid):
    root, data = _session(tmp_path / 'session')
    data[0, 6:22] = np.nan
    data[1, {'time': 0, 'pose': 16, 'grip': 20}[invalid]] = np.inf
    data.tofile(root / 'signals/commands.bin')
    timestamps, arms, grips = load_commands(open_session(root))
    np.testing.assert_array_equal(timestamps, [102.])
    np.testing.assert_array_equal(arms, _poses()[2:])
    np.testing.assert_array_equal(grips, [[.3, .4]])


def test_slerp_endpoints_and_short_arc():
    start, end = np.array([0., 0., 0., 1.]), np.array([0., 0., 1., 0.])
    np.testing.assert_allclose(quat_slerp(start, end, 0.), start)
    np.testing.assert_allclose(quat_slerp(start, end, 1.), end)
    np.testing.assert_allclose(quat_slerp(start, -start, .5), start)
    near = np.array([0., 0., np.sin(.1), np.cos(.1)])
    np.testing.assert_allclose(quat_slerp(start, -near, .5), [0., 0., np.sin(.05), np.cos(.05)])


@pytest.mark.parametrize('angle', [.001, .7, np.pi / 2])
def test_slerp_unit_quaternions(angle):
    start, end = np.array([0., 0., 0., 2.]), np.array([0., np.sin(angle), 0., np.cos(angle)]) * 3
    for fraction in np.linspace(0., 1., 21):
        assert np.linalg.norm(quat_slerp(start, end, fraction)) == pytest.approx(1.)


def test_playback_window_speed_and_pose_interpolation():
    timestamps = np.array([99., 100., 101., 102., 103.])
    arms = _poses(5)
    grips = np.arange(5)[:, None] + np.arange(2)[None, :]
    playback = Playback(timestamps, arms, grips, 100., 102., speed=2.)
    assert playback.duration == 1.
    arm, grip, done = playback.sample(.25)
    np.testing.assert_allclose(arm, lerp_arm(arms[1], arms[2], .5))
    np.testing.assert_array_equal(grip, [1.5, 2.5])
    assert not done
    arm, grip, done = playback.sample(10.)
    np.testing.assert_array_equal(arm, arms[3])
    np.testing.assert_array_equal(grip, grips[3])
    assert done
    np.testing.assert_array_equal(playback.sample(-1.)[0], arms[1])


def test_playback_validation():
    timestamps, arms, grips = np.arange(3), _poses(), np.zeros((3, 2))
    for t0, t1, speed in [(0., 2., 10.), (0.4, 0.6, 1.), (2., 0., 1.),
                          (0., float('nan'), 1.), (float('inf'), 2., 1.),
                          (0., 2., float('nan')), (0., 2., float('inf'))]:
        with pytest.raises(ValueError):
            Playback(timestamps, arms, grips, t0, t1, speed)
    with pytest.raises(ValueError):
        Playback(timestamps[::-1], arms, grips, 0., 2.)
    with pytest.raises(ValueError):
        Playback(timestamps, arms[:, :7], grips, 0., 2.)
    for array in (timestamps, arms, grips):
        invalid = array.copy().astype(float)
        invalid.flat[0] = np.nan
        values = [timestamps, arms, grips]
        values[next(index for index, value in enumerate(values) if value is array)] = invalid
        with pytest.raises(ValueError):
            Playback(values[0], values[1], values[2], 0., 2.)
    with pytest.raises(ValueError, match='四元数'):
        Playback(timestamps, np.zeros((3, 14)), grips, 0., 2.)


def test_playback_duplicate_boundaries_and_normalized_endpoints():
    arms = _poses(4)
    arms[:, 3:7] *= 2.
    arms[:, 10:14] *= 3.
    playback = Playback(np.array([0., 0., 1., 1.]), arms, np.zeros((4, 2)), 0., 1.)
    np.testing.assert_allclose(playback.sample(0.)[0], _poses(4)[1])
    np.testing.assert_allclose(playback.sample(1.)[0], _poses(4)[3])
    assert playback.sample(1.)[2]
    with pytest.raises(ValueError, match='不同时刻'):
        Playback(np.zeros(4), arms, np.zeros((4, 2)), 0., 1.)


def test_cosine_ramp_first_and_last_pose():
    start, goal = _poses(2)
    goal[3:7] = [0., 0., np.sin(.2), np.cos(.2)]
    goal[10:14] = [0., np.sin(.3), 0., np.cos(.3)]
    trajectory = ramp(start, goal, seconds=2., hz=50.)
    assert trajectory.shape == (100, 14)
    fraction = .5 - .5 * np.cos(np.pi / 100)
    np.testing.assert_allclose(trajectory[0], lerp_arm(start, goal, fraction))
    np.testing.assert_allclose(trajectory[-1], goal)
    steps = np.diff(trajectory[:, 0])
    assert steps[0] < steps[50] / 3 and steps[-1] < steps[50] / 3


def test_describe_pose_windows_and_sealing(tmp_path):
    root, _ = _session(tmp_path / 'session')
    events = [{'type': 'episode_start', 'round': 1, 'episode': 2, 't': 100., 'instruction': {}},
              {'type': 'episode_end', 'round': 1, 'episode': 2, 't': 102., 'outcome': 'success'}]
    (root / 'events.jsonl').write_text('\n'.join(json.dumps(event) for event in events))
    info = describe(root)
    assert info['whole']['commands'] == 3
    assert info['episodes'][0]['label'] == 'r1e2'
    assert info['episodes'][0]['commands'] == 3
    (root / 'DONE').unlink()
    assert 'DONE' in describe(root)['error']


def test_describe_empty_pose_commands(tmp_path):
    root, data = _session(tmp_path / 'session')
    data[:, 6:] = np.nan
    data.tofile(root / 'signals/commands.bin')
    timestamps, arms, grips = load_commands(open_session(root))
    assert timestamps.shape == (0,) and arms.shape == (0, 14) and grips.shape == (0, 2)
    assert '没有有效末端目标' in describe(root)['error']


@pytest.fixture
def ros_replay(monkeypatch, tmp_path):
    import rclpy
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from unitree_g1_ros2_control.msg import JointTargets

    from record import replay_node as module
    request_finished = threading.Event()
    request_errors = []
    handler = module.ReplayTask._on_request

    def on_request(self, message):
        request_finished.clear()
        try:
            handler(self, message)
        except Exception as error:
            request_errors.append(error)
        finally:
            request_finished.set()

    monkeypatch.setattr(module.ReplayTask, '_on_request', on_request)
    monkeypatch.setenv('ROS_DOMAIN_ID', '217')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    namespace = f'/replay_test_{uuid.uuid4().hex}'
    target_topic = f'{namespace}/joint_targets'
    joint_topic = f'{namespace}/joint_states'
    description_topic = f'{namespace}/robot_description'
    config = tmp_path / 'replay.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'joint_states_topic': joint_topic,
        'robot_description_topic': description_topic,
        'target_topic': target_topic,
    }}}))
    rclpy.init(args=[], domain_id=217)
    node = peer = executor = None
    try:
        peer = rclpy.create_node('replay_test_peer', use_global_arguments=False,
                                 namespace=namespace)
        targets, feedback = [], []

        def on_status(message: String) -> None:
            feedback.append(json.loads(message.data))

        peer.create_subscription(JointTargets, target_topic,
                                 targets.append, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        peer.create_subscription(String, f'{namespace}/replay/status',
                                 on_status, 10)
        request = peer.create_publisher(String, f'{namespace}/replay/request', 10)
        stop = peer.create_client(Trigger, f'{namespace}/replay/stop')
        description = peer.create_publisher(
            String, description_topic,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        urdf = Path(get_package_share_directory('unitree_g1_description')) / 'model/final.urdf'
        description.publish(String(data=urdf.read_text(encoding='utf-8')))
        joints = peer.create_publisher(
            JointState, joint_topic, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        node = module.ReplayTask(
            config=str(config), namespace=namespace,
            request_topic=f'{namespace}/replay/request',
            status_topic=f'{namespace}/replay/status',
            stop_service=f'{namespace}/replay/stop')
        for timer in node.timers:
            if timer.callback == node._tick_replay:
                timer.cancel()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(peer)
        executor.add_node(node)

        def wait(predicate):
            deadline = time.monotonic() + 10.
            while not predicate() and time.monotonic() < deadline:
                if request_errors:
                    raise request_errors[0]
                executor.spin_once(timeout_sec=.01)
            if request_errors:
                raise request_errors[0]
            assert predicate()

        wait(lambda: node.ik_ready and request.get_subscription_count() == 1
             and node.output.publisher.get_subscription_count() == 1
             and joints.get_subscription_count() == 1 and stop.service_is_ready())
        assert not node.ready
        measured = np.concatenate((np.arange(15) * .01,
                                   [0., .6, 0., -.5, 0., 0., 0.,
                                    0., -.6, 0., -.5, 0., 0., 0., .12, .23]))
        message = JointState(name=list(reversed(node.joints31)), position=measured[::-1].tolist())
        joints.publish(message)
        wait(lambda: node.ready)
        np.testing.assert_array_equal(node._q, measured)
        wait(lambda: any(status['ready'] for status in feedback))
        for timer in node.timers:
            timer.cancel()
        yield SimpleNamespace(node=node, module=module, peer=peer, measured=measured,
                              targets=targets, feedback=feedback, request=request,
                              request_finished=request_finished, stop=stop, wait=wait,
                              joints=joints, joint_message=message)
    finally:
        if node is not None:
            node.stop()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if peer is not None:
            peer.destroy_node()
        rclpy.try_shutdown()


def _model_session(harness, root):
    node = harness.node
    reached = node._ik.fk(harness.measured[node.arm_indices], clamp=False)
    start = np.concatenate((reached['left'], reached['right']))
    poses = np.tile(start, (3, 1))
    poses[:, 0] += [.003, .006, .01]
    poses[:, 7] += [.003, .006, .01]
    root, _ = _session(root, poses=poses)
    return root, start, poses


def _request(harness, root):
    from std_msgs.msg import String
    values = dict(session_path=str(root), t0=100., t1=102., speed=1., label='r1e2')
    harness.request_finished.clear()
    harness.request.publish(String(data=json.dumps(values)))


def test_ros_joint_measurements_and_readiness(ros_replay):
    from sensor_msgs.msg import JointState
    node = ros_replay.node
    assert node.ready and node.ik_ready
    assert node.get_name() == 'replay'
    assert node.get_namespace() == ros_replay.peer.get_namespace()
    assert node.request_topic == f'{node.get_namespace()}/replay/request'
    assert node.output.message.mask == sum(1 << index for index in range(15, 31))
    before = node._q.copy()
    for message in [JointState(name=node.joints31, position=[0.]),
                    JointState(name=node.joints31[:-1], position=before[:-1].tolist()),
                    JointState(name=node.joints31, position=[float('nan')] * 31)]:
        node._on_joints(message)
        np.testing.assert_array_equal(node._q, before)
    changed = before + .001
    node._on_joints(JointState(name=node.joints31, position=changed.tolist()))
    np.testing.assert_array_equal(node._q, changed)


def test_ros_request_ramp_ik_publish_and_completion(ros_replay, tmp_path, monkeypatch):
    harness, node = ros_replay, ros_replay.node
    root, start, poses = _model_session(harness, tmp_path / 'session')
    node.output.positions[:] = -1.
    clock = [time.monotonic()]
    monkeypatch.setattr(harness.module, 'monotonic', lambda: clock[0])
    loader = Mock(wraps=harness.module.load_commands)
    monkeypatch.setattr(harness.module, 'load_commands', loader)
    _request(harness, root)
    harness.wait(lambda: node._playback is not None)
    np.testing.assert_allclose(node._ramp_start, start)
    np.testing.assert_array_equal(node._ramp_grip, harness.measured[29:])
    np.testing.assert_array_equal(node._ik_seed, harness.measured[node.arm_indices])
    harness.wait(lambda: any(status['phase'] == 'ramp' and status['playing'] for status in harness.feedback))
    events, solutions = [], []
    solve, publish, commit = node.solve_ik, node.output.publish, node.commit_ik
    committed = [node._ik_seed.copy()]

    def solve_target(*args, **kwargs):
        result = solve(*args, **kwargs)
        events.append('solve')
        solutions.append(result[0].copy())
        return result

    def publish_target():
        np.testing.assert_array_equal(node._ik_seed, committed[0])
        publish()
        events.append('publish')

    def commit_target(positions):
        assert events[-1] == 'publish'
        commit(positions)
        committed[0] = node._ik_seed.copy()
        events.append('commit')

    monkeypatch.setattr(node, 'solve_ik', solve_target)
    monkeypatch.setattr(node.output, 'publish', publish_target)
    monkeypatch.setattr(node, 'commit_ik', commit_target)
    clock[0] += node.ramp_s / 2
    node._tick_replay()
    expected_pose = lerp_arm(start, poses[0], .5)
    np.testing.assert_allclose(node._ik_targets['left'], expected_pose[:7])
    np.testing.assert_allclose(node._ik_targets['right'], expected_pose[7:])
    np.testing.assert_allclose(node.output.positions[node.arm_slots], solutions[-1])
    np.testing.assert_allclose(node.output.positions[29:], .5 * (harness.measured[29:] + [.1, .2]))
    np.testing.assert_array_equal(node.output.positions[:15], harness.measured[:15])
    np.testing.assert_array_equal(node._ik_seed, node.output.positions[node.arm_indices])
    harness.wait(lambda: bool(harness.targets))
    np.testing.assert_allclose(harness.targets[-1].positions, node.output.positions)
    assert harness.targets[-1].mask == sum(1 << index for index in range(15, 31))
    clock[0] += node.ramp_s / 2 + .5
    node._tick_replay()
    expected_pose = lerp_arm(poses[0], poses[1], .5)
    np.testing.assert_allclose(node._ik_targets['left'], expected_pose[:7])
    np.testing.assert_allclose(node._ik_targets['right'], expected_pose[7:])
    assert node.status()['phase'] == 'play' and node.status()['playing']
    node._publish_status()
    harness.wait(lambda: any(status['phase'] == 'play' and status['playing'] for status in harness.feedback))
    clock[0] = node._play_start + node.ramp_s + 2.
    node._tick_replay()
    assert events == ['solve', 'publish', 'commit'] * 3
    loader.assert_called_once()
    assert loader.call_args.args[0].root == root and node._playback is None
    np.testing.assert_allclose(node.output.positions[node.arm_slots], solutions[-1])
    np.testing.assert_array_equal(node._ik_seed, node.output.positions[node.arm_indices])
    assert np.max(np.abs(node.output.positions[15:29] - harness.measured[15:29])) > .001
    np.testing.assert_allclose(node.output.positions[29:], [.3, .4])
    reached = node._ik.fk(node._ik_seed, clamp=False)
    assert np.linalg.norm(reached['left'][:3] - poses[-1, :3]) < .002
    assert np.linalg.norm(reached['right'][:3] - poses[-1, 7:10]) < .002
    node._publish_status()
    harness.wait(lambda: any(status['phase'] == 'done' for status in harness.feedback))
    status = next(status for status in reversed(harness.feedback) if status['phase'] == 'done')
    assert status['ready'] and not status['playing']
    assert status['elapsed'] == status['duration'] and status['progress'] == 1.
    assert status['label'] == 'r1e2' and status['session_path'] == str(root)
    np.testing.assert_allclose(status['joint_positions'], node.output.positions[15:29])
    np.testing.assert_allclose(status['grip'], [.3, .4])


@pytest.mark.parametrize('interruption', ['publish_failure', 'inflight_stop'])
def test_ros_interruption_preserves_targets_and_seed(ros_replay, tmp_path, monkeypatch, interruption):
    harness, node = ros_replay, ros_replay.node
    root, _, _ = _model_session(harness, tmp_path / 'session')
    _request(harness, root)
    harness.wait(lambda: node._playback is not None)
    before, seed = node.output.positions.copy(), node._ik_seed.copy()

    if interruption == 'publish_failure':
        def failed_publish():
            raise RuntimeError('publication failed')

        monkeypatch.setattr(node.output, 'publish', failed_publish)
    else:
        solve = node.solve_ik

        def late_solve(*args, **kwargs):
            result = solve(*args, **kwargs)
            node.stop()
            return result

        monkeypatch.setattr(node, 'solve_ik', late_solve)
    node._tick_replay()
    np.testing.assert_array_equal(node.output.positions, before)
    np.testing.assert_array_equal(node._ik_seed, seed)
    assert node._playback is None
    if interruption == 'publish_failure':
        phase = 'error'
        assert node.status()['error'] == 'publication failed'
    else:
        phase = 'idle'
        assert node.status()['phase'] == phase
        node._publish_status()
    harness.wait(lambda: any(status['phase'] == phase for status in harness.feedback))
    assert not harness.targets


def test_ros_stop_holds_last_target(ros_replay, tmp_path):
    from std_srvs.srv import Trigger
    harness, node = ros_replay, ros_replay.node
    root, _, _ = _model_session(harness, tmp_path / 'session')
    _request(harness, root)
    harness.wait(lambda: node._playback is not None)
    node._tick_replay()
    harness.wait(lambda: bool(harness.targets))
    before, seed = node.output.positions.copy(), node._ik_seed.copy()
    count = len(harness.targets)
    future = harness.stop.call_async(Trigger.Request())
    harness.wait(future.done)
    assert future.result().success and node._playback is None
    node._tick_replay()
    np.testing.assert_array_equal(node.output.positions, before)
    np.testing.assert_array_equal(node._ik_seed, seed)
    harness.wait(lambda: any(status['phase'] == 'idle' and not status['playing'] for status in harness.feedback))
    assert len(harness.targets) == count and node.ready


def test_ros_stop_cancels_loading(ros_replay, tmp_path, monkeypatch):
    from std_srvs.srv import Trigger
    harness, node = ros_replay, ros_replay.node
    root, _, _ = _model_session(harness, tmp_path / 'session')
    before = node.output.positions.copy()
    entered, release = threading.Event(), threading.Event()
    loader = harness.module.load_commands

    def blocked_load(session):
        entered.set()
        assert release.wait(10.)
        return loader(session)

    monkeypatch.setattr(harness.module, 'load_commands', blocked_load)
    _request(harness, root)
    try:
        harness.wait(entered.is_set)
        harness.wait(lambda: any(status['phase'] == 'loading' and status['playing'] for status in harness.feedback))
        future = harness.stop.call_async(Trigger.Request())
        harness.wait(future.done)
        assert future.result().success and node._playback is None
        stopped = node._play_generation
    finally:
        release.set()
    harness.wait(harness.request_finished.is_set)
    with node.execution_lock:
        assert node._play_generation == stopped
    node._tick_replay()
    np.testing.assert_array_equal(node.output.positions, before)
    harness.wait(lambda: any(status['phase'] == 'idle' and not status['playing'] for status in harness.feedback))
    assert node._playback is None and not harness.targets


def test_ros_stop_waits_for_ik_step(ros_replay, tmp_path, monkeypatch):
    from std_srvs.srv import Trigger
    harness, node = ros_replay, ros_replay.node
    root, _, _ = _model_session(harness, tmp_path / 'session')
    _request(harness, root)
    harness.wait(lambda: node._playback is not None and harness.request_finished.is_set())
    entered, release, stopping = threading.Event(), threading.Event(), threading.Event()
    step, stop = node._ik.step, node.stop
    errors = []

    def blocked_step(*args, **kwargs):
        entered.set()
        assert release.wait(10.)
        return step(*args, **kwargs)

    def stop_playback():
        stopping.set()
        stop()

    def tick():
        try:
            node._tick_replay()
        except Exception as error:
            errors.append(error)

    monkeypatch.setattr(node._ik, 'step', blocked_step)
    monkeypatch.setattr(node, 'stop', stop_playback)
    worker = threading.Thread(target=tick)
    worker.start()
    try:
        assert entered.wait(5.)
        future = harness.stop.call_async(Trigger.Request())
        harness.wait(stopping.is_set)
        assert not future.done()
    finally:
        release.set()
        worker.join(10.)
    assert not worker.is_alive() and not errors
    harness.wait(future.done)
    assert future.result().success and node._playback is None
    harness.wait(lambda: len(harness.targets) == 1)
    before, seed, count = node.output.positions.copy(), node._ik_seed.copy(), len(harness.targets)
    node._tick_replay()
    node._publish_status()
    harness.wait(lambda: any(status['phase'] == 'idle' and not status['playing'] for status in harness.feedback))
    np.testing.assert_array_equal(node.output.positions, before)
    np.testing.assert_array_equal(node._ik_seed, seed)
    assert len(harness.targets) == count


def test_ros_load_failure_reports_requested_session(ros_replay, tmp_path):
    harness = ros_replay
    path = str(tmp_path / 'missing-session')
    _request(harness, path)
    harness.wait(lambda: any(status['phase'] == 'error' for status in harness.feedback))
    status = next(status for status in reversed(harness.feedback) if status['phase'] == 'error')
    assert status['session_path'] == path and status['label'] == 'r1e2'
    assert status['error'] and not status['playing']
    assert status['ready']


def test_data_main_http_replay_and_sigint(ros_replay, tmp_path):
    import os
    import signal
    import subprocess
    import urllib.request

    import yaml
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from unitree_g1_ros2_control.msg import JointTargets

    harness = ros_replay
    namespace = f'{harness.peer.get_namespace()}/preview'
    sessions = tmp_path / 'sessions'
    root, _, poses = _model_session(harness, sessions / 'session')
    config = tmp_path / 'preview_replay.yaml'
    target_topic = f'{namespace}/joint_targets'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'joint_states_topic': harness.joints.topic_name,
        'robot_description_topic': harness.node.get_parameter('robot_description_topic').value,
        'target_topic': target_topic,
        'ramp_s': 0.1,
    }}}))
    parameters = tmp_path / 'preview.yaml'
    parameters.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'sessions_root': str(sessions),
        'bundles_root': str(tmp_path / 'bundles'),
        'request_topic': f'{namespace}/replay/request',
        'status_topic': f'{namespace}/replay/status',
        'stop_service': f'{namespace}/replay/stop',
        'replay_config': str(config),
        'dashboard_port': 0,
    }}}))
    targets, statuses, requests = [], [], []
    harness.peer.create_subscription(
        JointTargets, target_topic, targets.append,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    harness.peer.create_subscription(
        String, f'{namespace}/replay/status',
        lambda message: statuses.append(json.loads(message.data)), 10)
    harness.peer.create_subscription(
        String, f'{namespace}/replay/request',
        lambda message: requests.append(json.loads(message.data)), 10)
    endpoint = tmp_path / 'http_port'
    log = tmp_path / 'preview.log'
    script = '''
import sys
from pathlib import Path
from record import data_dashboard, data_node

endpoint = Path(sys.argv.pop(1))

class LocalPanel(data_dashboard.Panel):
    def start(self):
        self.host = '127.0.0.1'
        super().start()
        endpoint.write_text(str(self._srv.server_address[1]))

data_dashboard.Panel = LocalPanel
data_node.main()
'''
    environment = dict(os.environ, ROS_DOMAIN_ID='217', ROS_LOCALHOST_ONLY='1')
    process = None
    with log.open('w') as output:
        try:
            process = subprocess.Popen(
                [sys.executable, '-c', script, str(endpoint), '--ros-args',
                 '-r', '__node:=data_preview', '-r', f'__ns:={namespace}',
                 '--params-file', str(parameters)],
                env=environment, stdout=output, stderr=subprocess.STDOUT)

            def wait(predicate):
                harness.wait(lambda: process.poll() is not None or predicate())
                assert process.poll() is None, log.read_text()

            wait(endpoint.is_file)
            opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
            base = f'http://127.0.0.1:{int(endpoint.read_text())}'

            def http(path, body=None):
                request = urllib.request.Request(
                    base + path, data=None if body is None else json.dumps(body).encode(),
                    headers={'Content-Type': 'application/json'})
                with opener.open(request, timeout=5.) as response:
                    payload = json.load(response)
                if body is not None:
                    assert payload['ok'], payload
                    return payload['result']
                return payload

            wait(lambda: harness.joints.get_subscription_count() == 2)
            harness.joints.publish(harness.joint_message)
            wait(lambda: any(status['ready'] for status in statuses))
            wait(lambda: http('/api/state')['status']['ready'])
            names = set(harness.peer.get_node_names_and_namespaces())
            assert ('data_preview', namespace) in names
            assert ('replay', namespace) in names
            body = dict(session='session', t0=100., t1=102., speed=1., label='r1e2')
            assert http('/api/replay/start', body)['phase'] == 'loading'
            wait(lambda: bool(targets) and len(requests) == 1)
            assert requests[0] == dict(session_path=str(root), t0=100., t1=102.,
                                       speed=1., label='r1e2')
            wait(lambda: any(status['phase'] == 'done' for status in statuses))
            wait(lambda: http('/api/state')['status']['phase'] == 'done')
            completed = http('/api/state')['status']
            assert not completed['playing'] and completed['progress'] == 1.
            assert completed['session_path'] == str(root) and not completed['error']
            final = targets[-1]
            assert final.mask == sum(1 << index for index in range(15, 31))
            np.testing.assert_array_equal(final.positions[:15], harness.measured[:15])
            np.testing.assert_allclose(final.positions[29:], [.3, .4])
            reached = harness.node._ik.fk(np.asarray(final.positions)[15:29], clamp=False)
            assert np.linalg.norm(reached['left'][:3] - poses[-1, :3]) < .002
            assert np.linalg.norm(reached['right'][:3] - poses[-1, 7:10]) < .002
            assert http('/api/replay/start', body)['playing']
            wait(lambda: len(requests) == 2 and http('/api/state')['status']['phase'] == 'play')
            assert not http('/api/replay/stop', {})['playing']
            wait(lambda: not http('/api/state')['status']['playing'])
            assert http('/api/replay/start', body)['playing']
            wait(lambda: len(requests) == 3 and http('/api/state')['status']['phase'] == 'play')
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=10.) == 0, log.read_text()
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5.)
