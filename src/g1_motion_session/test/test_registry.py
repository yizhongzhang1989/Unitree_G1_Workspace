"""Focused session checks without a controller manager or hardware."""

import json
import sys
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from std_msgs.msg import String

from g1_motion_session.session_node import DefaultJointOutput, FULL_MASK, MotionSession, State
from g1_motion_session.task_base import State as TaskState, TaskBase
from g1_motion_session.joint_output import JointOutput


@pytest.mark.parametrize('override', [False, True])
def test_task_fixed_parameter_defaults_and_yaml_override(tmp_path, override):
    import rclpy
    import yaml
    from ament_index_python.packages import get_package_share_directory

    hardware = Path(get_package_share_directory('unitree_g1_ros2_control'))
    defaults = yaml.safe_load((hardware / 'config/default_31dof_param.yaml').read_text())
    joints = defaults['/**']['ros__parameters']['joints']
    passive = [0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, -0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    args = []
    if override:
        joints = list(reversed(joints))
        passive = [0.1] * 16
        parameters = tmp_path / 'task.yaml'
        parameters.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
            'joints': joints,
            'control_rate_hz': 100.0,
            'stand_s': 3.0,
            'stand_clear_s': 0.5,
            'stand_clear_roll': 0.8,
            'target_topic': '/test/targets',
            'robot_description_topic': '/test/description',
            'passive_targets': passive,
            'state_timeout_s': 0.3,
            'imu_topic': '/test/imu',
        }}}))
        args = ['--ros-args', '--params-file', str(parameters)]
    rclpy.init(args=args)
    node = None
    try:
        node = TaskBase('test_fixed_parameters', slots=range(15))
        assert node.joints31 == joints
        assert node.dt == (0.01 if override else 0.02)
        assert node.stand_s == (3.0 if override else 2.5)
        assert node.clear_s == (0.5 if override else 0.4)
        assert node.clear_roll == (0.8 if override else 0.7)
        assert node._state_timeout == (0.3 if override else 0.2)
        np.testing.assert_array_equal(node._stand_goal[15:], passive)
        assert node.output.publisher.topic_name == (
            '/test/targets' if override else '/forward_position_controller/joint_targets')
        assert node.get_parameter('robot_description_topic').get_parameter_value().string_value == (
            '/test/description' if override else '/robot_description')
        assert any(subscription.topic_name == (
            '/test/imu' if override else '/pelvis_imu_broadcaster/imu')
            for subscription in node.subscriptions)
        assert node.declare('stand_s', 9.0) == node.stand_s
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def task():
    node = TaskBase.__new__(TaskBase)
    node.lock = threading.RLock()
    node.execution_lock = threading.RLock()
    node.state = TaskState.IDLE
    node.generation = 0
    node._epoch = 0
    node._registered = True
    node._preparing = False
    node._error = ''
    node._events = {}
    node.joints31 = [f'joint_{index}' for index in range(31)]
    node.joints31[16] = 'left_shoulder_roll_joint'
    node.joints31[23] = 'right_shoulder_roll_joint'
    node._q = np.arange(31, dtype=float) / 100.0
    node._dq = np.zeros(31)
    node._quat = (0.0, 0.0, 0.0, 1.0)
    node._omega = (0.0, 0.0, 0.0)
    node.clear_roll = 0.7
    node._mask = 1 << 16
    node.output = JointOutput(Mock(), '/test/targets', [16])
    node.output.publish = Mock()
    node.stale_reason = Mock(return_value='')
    node.start = Mock()
    node.stop = Mock()
    node._publish_task_status = Mock()
    return node


def event(node, command, epoch):
    node._on_event(SimpleNamespace(command=command, epoch=epoch, start_time=100.0))


def test_task_lifecycle_reengage_and_running_no_reset(task):
    event(task, 'engage', 1)
    assert task._preparing and task.state is TaskState.STAND
    np.testing.assert_array_equal(task._prepare_start, task._q)
    task._q += 0.1
    event(task, 'engage', 2)
    np.testing.assert_array_equal(task._prepare_start, task._q)
    assert task.generation == 2
    task._preparing = False
    event(task, 'start', 3)
    assert task.state is TaskState.RUNNING
    task.start.assert_called_once()
    previous = task._prepare_start.copy()
    task._q += 0.1
    event(task, 'engage', 4)
    event(task, 'start', 5)
    assert task.state is TaskState.RUNNING and task.generation == 2
    np.testing.assert_array_equal(task._prepare_start, previous)
    task.start.assert_called_once()


def test_task_lifecycle_registration_and_stop_gate_publication(task):
    task._registered = False
    event(task, 'engage', 1)
    assert task.state is TaskState.IDLE
    assert not task.publish_target(np.ones(31))
    task._registered = True
    event(task, 'engage', 1)
    assert task.publish_target(np.ones(31))
    event(task, 'stop', 2)
    assert task.state is TaskState.STOPPED and not task._preparing
    assert task.generation == 2
    task.stop.assert_called_once()
    assert not task.publish_target(np.zeros(31))
    task.output.publish.assert_called_once()


@pytest.fixture
def prepared_task(task):
    task.state = TaskState.RUNNING
    task.slots = list(range(15, 31))
    for side, offset in (('left', 15), ('right', 22)):
        for index, joint in enumerate(('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw',
                                       'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw'), offset):
            task.joints31[index] = f'{side}_{joint}_joint'
    task._mask = sum(1 << index for index in task.slots)
    task.output.message.mask = task._mask
    task.output.positions[:] = task._q
    task.clear_s = 0.4
    task.stand_s = 2.5
    task._stand_goal = np.zeros(31)
    task._stand_goal[16], task._stand_goal[23] = 0.6, -0.6
    task.now = Mock(return_value=100.0)
    return task


@pytest.mark.parametrize('phase', ['prepare', 'prepared'])
def test_task_lifecycle_exception_stops_locally(prepared_task, phase):
    node = prepared_task
    node.stand_s = 2.5
    node._events['stop'] = Mock()
    node.state = TaskState.IDLE
    event(node, 'engage', 1)
    node.now.return_value = 103.0 if phase == 'prepared' else 100.2
    if phase == 'prepared':
        node.prepared = Mock(side_effect=RuntimeError('prepared failed'))
    else:
        node.output.publish.side_effect = RuntimeError('publish failed')
    node._tick_lifecycle()
    assert node.state is TaskState.ERROR
    assert not node._preparing
    assert 'failed' in node._error
    assert node.output.message.mask == node._mask
    node.stop.assert_called_once()
    node._events['stop'].assert_called_once()
    before = node.output.publish.call_count
    node._tick_lifecycle()
    assert node.output.publish.call_count == before


def test_default_preparation_finishes_without_starting(prepared_task):
    node = prepared_task
    node.state = TaskState.IDLE
    node.stand_s = 2.5
    node.prepared = Mock()
    event(node, 'engage', 1)
    node.now.return_value = 100.2
    node._tick_lifecycle()
    assert node._preparing
    node.start.assert_not_called()
    node.now.return_value = 103.0
    node._tick_lifecycle()
    assert not node._preparing and node.state is TaskState.STAND
    np.testing.assert_allclose(node.output.positions, node._stand_goal)
    node.prepared.assert_called_once()
    np.testing.assert_allclose(node.prepared.call_args.args[0].position, node._stand_goal)
    node.start.assert_not_called()
    count = node.output.publish.call_count
    node._tick_lifecycle()
    assert node.output.publish.call_count == count
    event(node, 'start', 2)
    node.start.assert_called_once()
    assert node.state is TaskState.RUNNING


@pytest.mark.parametrize('failure', ['stop', 'status'])
def test_task_lifecycle_cleanup_failure_does_not_escape(prepared_task, failure):
    node = prepared_task
    node.state = TaskState.IDLE
    event(node, 'engage', 1)
    node.now.return_value = 100.2
    node.output.publish.side_effect = RuntimeError('publish failed')
    if failure == 'stop':
        node.stop.side_effect = RuntimeError('stop failed')
    else:
        node._publish_task_status.side_effect = RuntimeError('status failed')
    node._tick_lifecycle()
    assert node.state is TaskState.ERROR
    assert f'{failure} failed' in node._error
    assert node.output.message.mask == node._mask
    node._tick_lifecycle()


@pytest.mark.parametrize('emergency', [False, True])
def test_task_blocking_stop_prevents_late_publication(prepared_task, emergency):
    node = prepared_task
    stopping = threading.Event()
    finish = threading.Event()
    attempting = threading.Event()
    published = []

    def stop():
        stopping.set()
        assert finish.wait(2.0)

    def publish():
        attempting.set()
        published.append(node.publish_target(np.ones(31)))

    node.stop = stop
    stopper = threading.Thread(target=node.estop, args=('test',)) if emergency else threading.Thread(
        target=event, args=(node, 'stop', 1))
    publisher = threading.Thread(target=publish)
    try:
        stopper.start()
        assert stopping.wait(1.0)
        publisher.start()
        assert attempting.wait(1.0)
        node.output.publish.assert_not_called()
        finish.set()
        stopper.join(1.0)
        publisher.join(1.0)
        assert not stopper.is_alive() and not publisher.is_alive()
        assert published == [False]
        node.output.publish.assert_not_called()
    finally:
        finish.set()
        stopper.join(1.0)
        if publisher.ident is not None:
            publisher.join(1.0)


@pytest.fixture
def session():
    node = MotionSession.__new__(MotionSession)
    node.lock = threading.RLock()
    node._switch_lock = threading.Lock()
    node._tasks = {}
    node.state = State.STAND
    node.generation = 0
    node._event_epoch = 0
    node._claimed_epoch = 123
    node._controller_active = True
    node._deactivating = False
    node._default_preparing = False
    node._reason = ''
    node._safety_checks_enabled = True
    node.clear_s = 0.4
    node.stand_s = 2.5
    node.clear_roll = 0.7
    node.joints = [f'joint_{index}' for index in range(31)]
    node.joints[16] = 'left_shoulder_roll_joint'
    node.joints[23] = 'right_shoulder_roll_joint'
    node._q = np.arange(31, dtype=float) / 100.0
    node._quat = (0.0, 0.0, 0.0, 1.0)
    node._tilt_limit = 1.0
    node._default_goal = np.zeros(31)
    node._state_timeout = 0.2
    node.now = lambda: 100.0
    node.ros_now = lambda: 200.0
    node._joint_stamp = node._imu_stamp = 100.0
    node.default_output = DefaultJointOutput(Mock(), '/test/targets')
    node.default_output.publish = Mock()
    node._events_publisher = Mock()
    node._status_publisher = Mock()
    node._switch_controller = Mock(return_value=(True, 'ok'))
    return node


def register(node, task_id, mask, release=False):
    return node._on_register(SimpleNamespace(task_id=task_id, mask=mask, release=release),
                             SimpleNamespace())


def status(node, task_id, state, epoch=None, **fields):
    task = node._tasks[task_id]
    payload = dict(task_id=task_id, mask=task['mask'], state=state,
                   ready=state == 'ready', error='', epoch=task['epoch'] if epoch is None else epoch)
    payload.update(fields)
    node._on_task_status(String(data=json.dumps(payload)))


def test_masks_conflict_and_release(session):
    assert register(session, 'lower', 0b11).accepted
    assert not register(session, 'other', 0b10).accepted
    assert register(session, 'upper', 0b100).accepted
    assert register(session, 'lower', 0, release=True).accepted
    assert register(session, 'other', 0b10).accepted
    assert session.default_output.message.mask == FULL_MASK & ~0b110


def test_mask_bit31_filtered(session):
    assert register(session, 'all', 0xffffffff).accepted
    assert session._tasks['all']['mask'] == FULL_MASK
    assert session.default_output.message.mask == 0


def test_register_running_hands_off_defaults(session):
    session.state = State.RUNNING
    assert register(session, 'new', 1 << 16).accepted
    assert session._tasks['new']['state'] == 'idle'
    assert session.default_output.message.mask & (1 << 16) == 0
    session._on_engage(None, SimpleNamespace())
    assert session.default_output.message.mask & (1 << 16) == 0
    assert session.state is State.RUNNING
    assert session._claimed_epoch == 123
    session._switch_controller.assert_not_called()


def test_reengage_preserves_running_task_and_execution_epoch(session):
    register(session, 'existing', 1)
    status(session, 'existing', 'running', epoch=7, vla_running=True)
    session.state = State.RUNNING
    session._event_epoch = 7
    register(session, 'new', 2)
    response = session._on_engage(None, SimpleNamespace())
    assert response.success
    assert session.state is State.RUNNING
    assert session._tasks['existing']['epoch'] == 7
    assert session._tasks['existing']['state'] == 'running'
    assert session._tasks['new']['state'] == 'preparing'
    event = session._events_publisher.publish.call_args.args[0]
    assert (event.command, event.epoch, event.start_time) == ('engage', 8, 200.05)
    assert session._claimed_epoch == 123
    session._switch_controller.assert_not_called()
    np.testing.assert_array_equal(session._default_start, session._q)
    session._q += 0.1
    session._on_engage(None, SimpleNamespace())
    np.testing.assert_array_equal(session._default_start, session._q)


def test_ready_status_controls_start(session):
    register(session, 'lower', 1)
    assert not session._on_start(None, SimpleNamespace()).success
    session._on_engage(None, SimpleNamespace())
    status(session, 'lower', 'ready', epoch=0)
    assert not session._on_start(None, SimpleNamespace()).success
    status(session, 'lower', 'ready', policy_loaded=True)
    assert not session._on_start(None, SimpleNamespace()).success
    session._default_preparing = False
    assert session._on_start(None, SimpleNamespace()).success
    assert session.state is State.RUNNING
    assert session._events_publisher.publish.call_args.args[0].command == 'start'
    session._publish_status()
    payload = json.loads(session._status_publisher.publish.call_args.args[0].data)
    assert payload['policy_loaded'] is True
    assert payload['execution_epoch'] == 123


def test_running_tasks_do_not_block_start(session):
    register(session, 'upper', 1)
    status(session, 'upper', 'running', ready=False)
    assert session._on_start(None, SimpleNamespace()).success


def test_empty_registry_can_start_after_default_preparation(session):
    session._default_preparing = True
    assert not session._on_start(None, SimpleNamespace()).success
    session._default_preparing = False
    assert session._on_start(None, SimpleNamespace()).success


@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('fault,reason', [
    ('joints', '/joint_states timeout'),
    ('imu', 'IMU timeout'),
    ('tilt', 'tilt limit exceeded'),
])
def test_session_safety_switch_controls_commands_and_monitoring(session, monkeypatch, enabled, fault, reason):
    session._safety_checks_enabled = enabled
    if fault == 'joints':
        session._joint_stamp = 99.0
    elif fault == 'imu':
        session._imu_stamp = 99.0
    else:
        session._quat = (2 ** -0.5, 0.0, 0.0, 2 ** -0.5)
    response = session._on_engage(None, SimpleNamespace())
    assert response.success is not enabled
    session._default_preparing = False
    response = session._on_start(None, SimpleNamespace())
    assert response.success is not enabled
    if enabled:
        assert response.message == reason
    emergency = Mock()
    monkeypatch.setattr(session, 'estop', emergency)
    session._publish_status()
    if enabled:
        emergency.assert_called_once_with(reason)
    else:
        emergency.assert_not_called()
    payload = json.loads(session._status_publisher.publish.call_args.args[0].data)
    assert payload['safety_checks_enabled'] is enabled


@pytest.mark.parametrize('override', [None, False])
def test_session_configures_safety_and_sensor_subscription(override):
    import rclpy

    root = Path(__file__).resolve().parents[2]
    arguments = ['--ros-args', '--params-file', str(root / 'g1_motion_session/config/session.yaml'),
                 '--params-file', str(root / 'unitree_g1_ros2_control/config/default_31dof_param.yaml')]
    if override is not None:
        arguments.extend(['-p', 'safety_checks_enabled:=false'])
    rclpy.init(args=arguments)
    node = None
    try:
        node = MotionSession()
        enabled = override is not False
        assert node._safety_checks_enabled is enabled
        assert node.describe_parameter('safety_checks_enabled').read_only
        topics = {subscription.topic_name for subscription in node.subscriptions}
        assert '/joint_states' in topics
        assert ('/pelvis_imu_broadcaster/imu' in topics) is enabled
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_stop_holds_controller_and_stops_default_publish(session):
    register(session, 'lower', 1)
    session.state = State.RUNNING
    assert session._on_stop(None, SimpleNamespace()).success
    assert session.state is State.IDLE
    assert session._controller_active
    assert session._events_publisher.publish.call_args.args[0].command == 'stop'
    session._switch_controller.assert_not_called()


def test_estop_broadcasts_stop_before_deactivate(session):
    order = []
    session._events_publisher.publish = lambda event: order.append(event.command)
    session._switch_controller = lambda **kwargs: (order.append(kwargs['activate']) or True, 'ok')
    session.get_logger = lambda: SimpleNamespace(error=Mock())
    session.estop('test')
    assert order == ['stop', False]
    assert session.state is State.ESTOP


def test_default_timer_register_handoff_is_serialized(session, monkeypatch):
    monkeypatch.setitem(sys.modules, 'g1_motion_session.joint_output',
                        SimpleNamespace(two_stage=lambda *args: np.zeros(31)))
    publishing = threading.Event()
    finish_publish = threading.Event()
    registering = threading.Event()
    registered = threading.Event()
    masks = []

    def publish():
        masks.append(session.default_output.message.mask)
        if len(masks) == 1:
            publishing.set()
            assert finish_publish.wait(1.0)

    def claim():
        registering.set()
        register(session, 'upper', 1 << 16)
        registered.set()

    session.default_output.publish = publish
    timer = threading.Thread(target=session._tick_defaults)
    claimant = threading.Thread(target=claim)
    try:
        timer.start()
        assert publishing.wait(1.0)
        claimant.start()
        assert registering.wait(1.0)
        assert not registered.is_set()
        finish_publish.set()
        timer.join(1.0)
        claimant.join(1.0)
        assert registered.is_set()
        session._tick_defaults()
        assert masks == [FULL_MASK, FULL_MASK & ~(1 << 16)]
    finally:
        finish_publish.set()
        timer.join(1.0)
        if claimant.ident is not None:
            claimant.join(1.0)


def test_default_preparation_shared_ros_start_and_clearance(session, monkeypatch):
    interpolate = Mock(return_value=np.ones(31))
    monkeypatch.setitem(sys.modules, 'g1_motion_session.joint_output',
                        SimpleNamespace(two_stage=interpolate))
    session._q[16] = 0.9
    session._q[23] = -0.8
    session._on_engage(None, SimpleNamespace())
    assert session._default_via[16] == 0.9
    assert session._default_via[23] == -0.8
    np.testing.assert_array_equal(session._default_via[:15], session._q[:15])
    session._tick_defaults()
    session.default_output.publish.assert_not_called()
    session.ros_now = lambda: session._default_start_time + session.stand_s
    session._tick_defaults()
    assert interpolate.call_args.args[0] == session.stand_s
    assert not session._default_preparing
    np.testing.assert_array_equal(session.default_output.positions, np.ones(31))


def test_stop_during_activation_holds_controller_without_engage(session):
    session.state = State.IDLE
    session._controller_active = False
    entered = threading.Event()
    finish = threading.Event()
    switches = []

    def switch(*, activate):
        switches.append(activate)
        if activate:
            entered.set()
            assert finish.wait(1.0)
        return True, 'ok'

    session._switch_controller = switch
    try:
        assert session._on_engage(None, SimpleNamespace()).success
        assert entered.wait(1.0)
        session._on_stop(None, SimpleNamespace())
        finish.set()
        with session._switch_lock:
            pass
        assert session.state is State.IDLE
        assert session._controller_active
        assert switches == [True]
        assert [call.args[0].command for call in session._events_publisher.publish.call_args_list] == ['stop']
    finally:
        finish.set()


def test_estop_interrupts_pending_activation(session):
    session.state = State.ENGAGING
    session._controller_active = False
    session.get_logger = lambda: SimpleNamespace(error=Mock())
    entered = threading.Event()
    finish = threading.Event()
    stopped = threading.Event()
    order = []

    def switch(*, activate):
        order.append(activate)
        if activate:
            entered.set()
            assert finish.wait(1.0)
        return True, 'ok'

    def publish(event):
        order.append(event.command)
        stopped.set()

    session._switch_controller = switch
    session._events_publisher.publish = publish
    activation = threading.Thread(target=session._activate, args=(session.generation,))
    emergency = threading.Thread(target=session.estop, args=('test',))
    try:
        activation.start()
        assert entered.wait(1.0)
        emergency.start()
        assert stopped.wait(1.0)
        assert session.state is State.ESTOP
        assert not session._on_engage(None, SimpleNamespace()).success
        finish.set()
        activation.join(1.0)
        emergency.join(1.0)
        assert not activation.is_alive() and not emergency.is_alive()
        assert order[:2] == [True, 'stop']
        assert order[2:] and all(value is False for value in order[2:])
        assert not session._controller_active
    finally:
        finish.set()
        activation.join(1.0)
        if emergency.ident is not None:
            emergency.join(1.0)


def test_partial_joint_states_update_known_slots(session):
    session._js_names, session._js_index = [], []
    session._on_joint_states(SimpleNamespace(name=[session.joints[16]], position=[0.45]))
    assert session._q[16] == 0.45
    assert session._joint_stamp == 100.0


def test_task_legacy_fields_cannot_replace_session_epoch(session):
    register(session, 'upper', 1)
    status(session, 'upper', 'running', execution_epoch=999, arms_live=True)
    session._publish_status()
    payload = json.loads(session._status_publisher.publish.call_args.args[0].data)
    assert payload['arms_live']
    assert payload['execution_epoch'] == 123


@pytest.mark.parametrize('config', ['session.yaml', 'custom.yaml', 'absolute'])
def test_session_launch_parameters_and_override(monkeypatch, tmp_path, config):
    import runpy
    from launch import LaunchContext

    root = Path(__file__).resolve().parents[2]
    module = runpy.run_path(str(root / 'g1_motion_session/launch/session.launch.py'))
    shares = {'g1_motion_session': tmp_path,
              'unitree_g1_ros2_control': root / 'unitree_g1_ros2_control'}
    factory = Mock()
    monkeypatch.setitem(module['_nodes'].__globals__, 'Node', factory)
    monkeypatch.setitem(module['_nodes'].__globals__, 'get_package_share_directory',
                        lambda name: str(shares[name]))
    if config == 'absolute':
        config = str(tmp_path / 'absolute.yaml')
    context = LaunchContext()
    context.launch_configurations.update(config=config, safety_checks_enabled='false')
    module['_nodes'](context)
    expected = [str(tmp_path / 'config/session.yaml')]
    override = str(tmp_path / 'config' / config)
    if override not in expected:
        expected.append(override)
    expected.append(str(shares['unitree_g1_ros2_control'] / 'config/default_31dof_param.yaml'))
    parameters = factory.call_args.kwargs['parameters']
    assert parameters[:-1] == expected
    assert parameters[-1]['safety_checks_enabled'].evaluate(context) is False


def test_default_output_exposes_full_positions(monkeypatch):
    positions = np.zeros(31)
    output = SimpleNamespace(message=SimpleNamespace(mask=FULL_MASK), positions=positions,
                             publish=Mock())
    monkeypatch.setitem(sys.modules, 'g1_motion_session.joint_output',
                        SimpleNamespace(JointOutput=lambda *args: output))
    default = DefaultJointOutput(None, '/targets')
    assert default.positions is positions
    assert default.positions31 is positions
    default.publish()
    output.publish.assert_called_once()
