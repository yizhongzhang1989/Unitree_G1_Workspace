import ast
import errno
import importlib.util
import json
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import textwrap
import threading
import time
from typing import Dict, List, cast
from unittest import mock

import pytest
from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.utilities import (
    normalize_to_list_of_substitutions,
    perform_substitutions,
)
from launch_ros.actions import Node


LAUNCH_DIR = Path(__file__).parents[1] / "launch"
WORKSPACE_SRC = Path(__file__).parents[2]


def _load_launch(name, launch_dir=LAUNCH_DIR):
    path = launch_dir / name
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load launch module: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _all_data_context(scope, topology="dual", end_effector_load="true"):
    context = LaunchContext()
    context.launch_configurations.update({
        "scope": scope,
        "topology": topology,
        "enable_grippers_on_start": "true",
        "controller_manager": "/controller_manager",
        "lowstate_topic": "/lowstate",
        "arm_stiffness_scale": "2.5",
        "joint_states_topic": "/joint_states",
        "robot_description_topic": "/robot_description",
        "require_pr_mode": "true",
        "use_sim_time": "false",
        "end_effector_load": end_effector_load,
    })
    return context


def _source_path(include):
    context = LaunchContext()
    include.launch_description_source.get_launch_description(context)
    return include.launch_description_source.location


def _perform(context, value):
    return perform_substitutions(
        context, normalize_to_list_of_substitutions(value))


def _default(argument: DeclareLaunchArgument) -> str:
    value = argument.default_value
    assert value is not None
    return _perform(LaunchContext(), value)


@pytest.mark.parametrize('topology', ['dual', 'single'])
def test_end_effectors_camera_parameters_preserve_defaults_and_overrides(topology):
    from launch_ros.utilities import evaluate_parameters

    module = _load_launch(f'end_effectors_{topology}_bus.launch.py')
    context = LaunchContext()
    description = module.generate_launch_description()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            assert entity.default_value is not None
            context.launch_configurations[entity.name] = _perform(context, entity.default_value)
    factory = next(entity for entity in description.entities if isinstance(entity, OpaqueFunction))
    for overrides, expected in (
        ({}, (0, 240, 15)),
        ({'wrist_left_url': 'rtsp://127.0.0.1/left',
          'wrist_right_url': 'rtsp://127.0.0.1/right',
          'wrist_image_width': '640', 'wrist_image_height': '360', 'wrist_fps': '30'},
         (640, 360, 30)),
    ):
        context.launch_configurations.update(overrides)
        actions = factory.execute(context)
        assert actions is not None
        cameras = [action for action in actions if isinstance(action, Node)
                   and _node_package(action) == 'camera_node']
        assert len(cameras) == 2
        for side, camera in zip(('left', 'right'), cameras):
            parameters, = evaluate_parameters(context, getattr(camera, '_Node__parameters'))
            assert isinstance(parameters, dict)
            assert parameters['rtsp_url'] == context.launch_configurations[f'wrist_{side}_url']
            assert tuple(parameters[key] for key in ('image_width', 'image_height', 'fps')) == expected
            assert all(type(parameters[key]) is int for key in ('image_width', 'image_height', 'fps'))


def _node_package(node: Node) -> str:
    return cast(str, getattr(node, "_Node__package"))


def _node_executable(node: Node) -> str:
    return cast(str, getattr(node, "_Node__node_executable"))


@pytest.mark.parametrize('lower,upper,expected', [
    ('velocity', 'vr', [
        ('g1_motion_session', 'session.launch.py'),
        ('lower_velocity_tracking', 'velocity_tracking.launch.py', ''),
        ('upper_vr', 'vr_teleop.launch.py', ''),
    ]),
    ('footstep', 'vla', [
        ('g1_motion_session', 'session.launch.py'),
        ('lower_footstep_tracking', 'footstep_tracking.launch.py', ''),
        ('upper_vla', 'vla_bridge.launch.py', ''),
    ]),
    ('none', 'float', [
        ('g1_motion_session', 'session.launch.py'),
        ('upper_float', 'float.launch.py', ''),
    ]),
    ('none', 'force_feedback', [
        ('g1_motion_session', 'session.launch.py'),
        ('upper_force_feedback', 'force_feedback.launch.py', ''),
    ]),
    ('none', 'none', [('g1_motion_session', 'session.launch.py')]),
])
def test_motion_tasks_select_independent_launches(lower, upper, expected):
    module = _load_launch('motion_tasks.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(lower_task=lower, upper_task=upper)
    with mock.patch.object(module, '_include', side_effect=lambda *args, **kwargs: args):
        assert module._tasks(context) == expected


@pytest.mark.parametrize('lower,upper', [('bad', 'none'), ('none', 'bad')])
def test_motion_tasks_reject_unknown_selections(lower, upper):
    module = _load_launch('motion_tasks.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(lower_task=lower, upper_task=upper)
    with pytest.raises(ValueError):
        module._tasks(context)


def test_motion_tasks_argument_defaults_and_choices():
    module = _load_launch('motion_tasks.launch.py')
    arguments = {
        entity.name: entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert {name: _default(argument) for name, argument in arguments.items()} == {
        'lower_task': 'velocity',
        'upper_task': 'vr',
        'lower_config': '',
        'upper_config': '',
        'safety_checks_enabled': '',
    }
    assert arguments['lower_task'].choices == ['velocity', 'footstep', 'none']
    assert arguments['upper_task'].choices == ['vr', 'vla', 'float', 'force_feedback', 'none']


@pytest.mark.parametrize('config,data', [('', 'true'), ('/tmp/replay-calibration.yaml', 'false')])
def test_record_launch_data_manager_parameters(config, data):
    from launch_ros.utilities import evaluate_parameters

    module = _load_launch('record.launch.py', WORKSPACE_SRC / 'record/launch')
    description = module.generate_launch_description()
    arguments = {entity.name: _default(entity)
                 for entity in description.entities if isinstance(entity, DeclareLaunchArgument)}
    assert {name: arguments[name] for name in ('data', 'data_port', 'replay_config')} == {
        'data': 'true', 'data_port': '8221', 'replay_config': ''}
    context = LaunchContext()
    context.launch_configurations.update(
        output_root='/tmp/record-sessions', dashboard_port='8220',
        data_port='8221', replay_config=config, data=data)
    recorder, manager = [entity for entity in description.entities if isinstance(entity, Node)]
    assert _node_package(recorder) == 'record'
    assert _node_executable(recorder) == 'recorder'
    assert _node_package(manager) == 'record'
    assert _node_executable(manager) == 'data_manager'
    condition = manager.condition
    assert condition is not None
    assert condition.evaluate(context) == (data == 'true')
    assert evaluate_parameters(context, getattr(manager, '_Node__parameters')) == ({
        'sessions_root': '/tmp/record-sessions',
        'dashboard_port': 8221,
        'peer_port': 8220,
        'replay_config': config,
    },)


@pytest.mark.parametrize('safety', ['', 'false', 'true'])
def test_whole_vr_selects_vr_velocity_source(safety):
    module = _load_launch('whole_vr.launch.py')
    with mock.patch.object(module, 'get_package_share_directory', return_value=str(LAUNCH_DIR.parent)):
        description = module.generate_launch_description()
    include, = [entity for entity in description.entities
                if isinstance(entity, IncludeLaunchDescription)]
    assert isinstance(include, IncludeLaunchDescription)
    assert _source_path(include).endswith('motion_tasks.launch.py')
    arguments = dict(include.launch_arguments)
    override = arguments.pop('safety_checks_enabled')
    context = LaunchContext()
    context.launch_configurations['safety_checks_enabled'] = safety
    assert _perform(context, override) == safety
    assert arguments == {
        'lower_task': 'velocity', 'upper_task': 'vr', 'lower_config': 'lower_vr.yaml'}


@pytest.fixture
def keyboard_launcher_pty(tmp_path, monkeypatch):
    import fcntl
    import struct
    import termios

    import rclpy
    from ament_index_python.packages import get_package_prefix
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node as RosNode
    from rclpy.signals import SignalHandlerOptions
    from std_srvs.srv import Trigger

    entry = Path(get_package_prefix('robot_bringup')) / 'lib/robot_bringup/teleop_keyboard'
    started = tmp_path / 'launch-start.json'
    stopped = tmp_path / 'launch-exit.json'
    failure = tmp_path / 'launch-failure'
    command = tmp_path / 'ros2'
    command.write_text(f'#!{sys.executable}\n' + textwrap.dedent('''\
        import json
        import os
        from pathlib import Path
        import signal
        import sys
        import threading

        stopped = threading.Event()
        signals = []

        def interrupt(signum, frame):
            signals.append(signum)
            stopped.set()

        signal.signal(signal.SIGINT, interrupt)
        Path(os.environ['FAKE_LAUNCH_START']).write_text(json.dumps({
            'pid': os.getpid(), 'argv': sys.argv[1:],
            'domain': os.environ['ROS_DOMAIN_ID'],
            'localhost': os.environ['ROS_LOCALHOST_ONLY'],
        }))
        print('fake launch ready', flush=True)
        failure = Path(os.environ['FAKE_LAUNCH_FAILURE'])
        while not stopped.wait(0.02):
            if failure.exists():
                break
        code = 7 if failure.exists() else 0
        Path(os.environ['FAKE_LAUNCH_EXIT']).write_text(json.dumps({
            'returncode': code, 'signals': signals,
        }))
        print(f'fake launch exit {code}', flush=True)
        raise SystemExit(code)
    '''))
    command.chmod(0o755)
    monkeypatch.setenv('ROS_DOMAIN_ID', '219')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    monkeypatch.setenv('CYCLONEDDS_URI', (
        '<CycloneDDS><Domain><General>'
        '<AllowMulticast>false</AllowMulticast></General>'
        '<Discovery><ParticipantIndex>auto</ParticipantIndex>'
        '<Peers><Peer address="127.0.0.1"/></Peers></Discovery>'
        '</Domain></CycloneDDS>'))
    monkeypatch.setenv('PATH', f'{tmp_path}{os.pathsep}{os.environ["PATH"]}')
    monkeypatch.setenv('PYTHONUNBUFFERED', '1')
    monkeypatch.setenv('TMPDIR', str(tmp_path))
    monkeypatch.setenv('FAKE_LAUNCH_START', str(started))
    monkeypatch.setenv('FAKE_LAUNCH_EXIT', str(stopped))
    monkeypatch.setenv('FAKE_LAUNCH_FAILURE', str(failure))

    context = Context()
    service_node = None
    executor = None
    worker = None
    process = None
    master = slave = None
    shutdown = threading.Event()
    requests = []
    output = bytearray()
    eof = False

    def respond(name, response):
        requests.append(name)
        response.success = True
        response.message = f'fake {name}'
        return response

    def spin_services():
        assert executor is not None
        while not shutdown.is_set():
            executor.spin_once(timeout_sec=0.05)

    def wait_for(predicate, timeout=10.0):
        nonlocal eof
        assert master is not None
        deadline = time.monotonic() + timeout
        while not predicate():
            remaining = deadline - time.monotonic()
            if remaining <= 0 or eof:
                pytest.fail(f'PTY wait failed:\n{output.decode("utf-8", "replace")}')
            if select.select([master], [], [], min(remaining, 0.1))[0]:
                try:
                    chunk = os.read(master, 65536)
                except OSError as exc:
                    if exc.errno != errno.EIO:
                        raise
                    chunk = b''
                output.extend(chunk)
                eof = not chunk
        return bytes(output)

    try:
        rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
        service_node = RosNode('keyboard_boundary_services', context=context)
        service_node.create_service(
            Trigger, '/motion_control/engage',
            lambda request, response: respond('engage', response))
        service_node.create_service(
            Trigger, '/motion_control/start',
            lambda request, response: respond('start', response))
        service_node.create_service(
            Trigger, '/motion_control/estop',
            lambda request, response: respond('estop', response))
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(service_node)
        worker = threading.Thread(target=spin_services)
        worker.start()
        master, slave = pty.openpty()
        fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', 24, 120, 0, 0))
        wrapper = textwrap.dedent('''\
            import fcntl
            import runpy
            import sys
            import termios
            from lower_velocity_tracking import teleop_keyboard

            fcntl.ioctl(sys.stdin.fileno(), termios.TIOCSCTTY, 0)
            original_init = teleop_keyboard.Teleop.__init__

            def init(self):
                original_init(self)
                if not self._estop_client.wait_for_service(timeout_sec=5.0):
                    raise RuntimeError('fake estop service unavailable')

            teleop_keyboard.Teleop.__init__ = init
            entry = sys.argv.pop(1)
            sys.argv[0] = entry
            runpy.run_path(entry, run_name='__main__')
        ''')
        process = subprocess.Popen(
            [sys.executable, '-c', wrapper, str(entry)],
            stdin=slave, stdout=slave, stderr=slave, start_new_session=True,
            cwd=WORKSPACE_SRC.parent)
        os.close(slave)
        slave = None
        yield {
            'process': process, 'master': master, 'output': output,
            'wait_for': wait_for, 'started': started, 'stopped': stopped,
            'failure': failure, 'requests': requests, 'directory': tmp_path,
        }
    finally:
        groups = []
        if process is not None and process.poll() is None:
            groups.append(process.pid)
        if started.exists():
            groups.append(json.loads(started.read_text())['pid'])
        for group in groups:
            try:
                os.killpg(group, signal.SIGINT)
            except ProcessLookupError:
                pass
        if process is not None:
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                for group in groups:
                    try:
                        os.killpg(group, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                process.wait(timeout=5.0)
        shutdown.set()
        if worker is not None:
            worker.join(timeout=2.0)
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if service_node is not None:
            service_node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)
        for descriptor in (master, slave):
            if descriptor is not None:
                os.close(descriptor)


@pytest.mark.parametrize('exit_kind', ['q', 'ctrl_c', 'launch_fail'])
def test_keyboard_launcher_real_pty_lifecycle(keyboard_launcher_pty, exit_kind):
    session = keyboard_launcher_pty
    process = session['process']
    wait_for = session['wait_for']
    wait_for(lambda: b'Q         ' in session['output'])
    wait_for(lambda: b'vx+0.00' in session['output'])
    wait_for(session['started'].exists)
    launch = json.loads(session['started'].read_text())
    assert launch['argv'] == [
        'launch', 'robot_bringup', 'motion_tasks.launch.py',
        'lower_task:=velocity', 'upper_task:=float', 'lower_config:=', 'upper_config:=']
    assert launch['domain'] == '219'
    assert launch['localhost'] == '1'
    assert os.getpgid(launch['pid']) == launch['pid']
    assert os.getpgid(process.pid) == process.pid
    assert session['requests'] == []
    assert process.poll() is None

    if exit_kind == 'launch_fail':
        session['failure'].write_text('fail')
    else:
        os.write(session['master'], b'Q' if exit_kind == 'q' else b'\x03')
    output = wait_for(lambda: b'fake estop' in session['output'])
    wait_for(session['stopped'].exists)
    wait_for(lambda: process.poll() is not None)
    assert process.wait(timeout=1.0) == (1 if exit_kind == 'launch_fail' else 0)
    assert session['requests'] == ['estop']
    result = json.loads(session['stopped'].read_text())
    assert result == {
        'returncode': 7 if exit_kind == 'launch_fail' else 0,
        'signals': [] if exit_kind == 'launch_fail' else [signal.SIGINT],
    }
    assert b'Traceback' not in session['output']
    assert b'KeyboardInterrupt' not in output
    if exit_kind == 'launch_fail':
        assert '任务启动失败'.encode() in session['output']
    logs = list(session['directory'].glob('g1-motion-keyboard-*.log'))
    assert len(logs) == 1
    assert 'fake launch ready' in logs[0].read_text()
    with pytest.raises(ProcessLookupError):
        os.killpg(launch['pid'], 0)


@pytest.mark.parametrize('lower,upper', [
    ('velocity', 'none'),
    ('footstep', 'none'),
    ('none', 'vr'),
    ('none', 'vla'),
    ('none', 'float'),
    ('none', 'force_feedback'),
])
@pytest.mark.parametrize('config', ['', '/tmp/user-calibration.yaml'])
def test_motion_tasks_forward_explicit_config_in_scoped_group(
        lower, upper, config):
    module = _load_launch('motion_tasks.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(
        lower_task=lower, upper_task=upper,
        lower_config=config if lower != 'none' else '',
        upper_config=config if upper != 'none' else '')
    with mock.patch.object(module, 'get_package_share_directory',
                           side_effect=lambda name: str(WORKSPACE_SRC / name)):
        actions = module._tasks(context)
    group = actions[1]
    assert isinstance(group, module.GroupAction)
    assert getattr(group, '_GroupAction__scoped') is True
    include = next(entity for entity in group.get_sub_entities()
                   if isinstance(entity, IncludeLaunchDescription))
    assert dict(include.launch_arguments) == ({'config': config} if config else {})
    package, filename = module._LOWER[lower] if lower != 'none' else module._UPPER[upper]
    assert _source_path(include) == str(WORKSPACE_SRC / package / 'launch' / filename)


@pytest.mark.parametrize('safety', ['', 'false', 'true'])
def test_motion_tasks_forward_session_safety_override(safety):
    module = _load_launch('motion_tasks.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(lower_task='none', upper_task='none', safety_checks_enabled=safety)
    with mock.patch.object(module, 'get_package_share_directory',
                           side_effect=lambda name: str(WORKSPACE_SRC / name)):
        actions = module._tasks(context)
    include = next(entity for entity in actions[0].get_sub_entities()
                   if isinstance(entity, IncludeLaunchDescription))
    assert dict(include.launch_arguments) == ({'safety_checks_enabled': safety} if safety else {})


@pytest.mark.parametrize('package,filename,default,ik,overrides', [
    ('lower_velocity_tracking', 'velocity_tracking.launch.py', 'lower_body.yaml', False, {}),
    ('lower_footstep_tracking', 'footstep_tracking.launch.py', 'footstep.yaml', False, {}),
    ('upper_float', 'float.launch.py', 'upper_float.yaml', False,
     {'gripper_source': 'keyboard', 'gripper_command_topic': '/custom/grip'}),
    ('upper_force_feedback', 'force_feedback.launch.py', 'upper_force_feedback.yaml', False, {}),
    ('upper_vr', 'vr_teleop.launch.py', 'upper_vr.yaml', True, {'arm_scale': 1.25}),
    ('upper_vla', 'vla_bridge.launch.py', 'vla_bridge.yaml', True,
     {'history_length': 8, 'execution_rate_hz': 25.0}),
])
@pytest.mark.parametrize('override', [False, True])
def test_task_launch_parameter_sources_are_explicit(tmp_path, package, filename, default, ik, overrides, override):
    from launch_ros.utilities import evaluate_parameters
    import yaml

    module = _load_launch(filename, WORKSPACE_SRC / package / 'launch')
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = _default(entity)
    context.launch_configurations.update({name: str(value) for name, value in overrides.items()})
    config = WORKSPACE_SRC / package / 'config' / default
    if override:
        config = tmp_path / 'override.yaml'
        config.write_text((WORKSPACE_SRC / package / 'config' / default).read_text())
        context.launch_configurations['config'] = str(config)
    factory = module._node if hasattr(module, '_node') else module._nodes
    node, = factory(context)
    parameters = evaluate_parameters(context, getattr(node, '_Node__parameters'))
    expected = [WORKSPACE_SRC / 'g1_motion_session/config/session.yaml']
    if ik:
        expected.append(WORKSPACE_SRC / 'g1_motion_session/config/arm_ik.yaml')
    expected.append(WORKSPACE_SRC / package / 'config' / default)
    if override:
        expected.append(config)
    files = [source.resolve() for source in parameters if isinstance(source, Path)]
    expected.append(WORKSPACE_SRC / 'unitree_g1_ros2_control/config/default_31dof_param.yaml')
    if package == 'upper_vla':
        expected.append(WORKSPACE_SRC / 'upper_vla/config/backends/cogact_unitree.yaml')
    assert files == [source.resolve() for source in expected]
    hardware = yaml.safe_load(expected[-2 if package == 'upper_vla' else -1].read_text())
    assert len(hardware['/**']['ros__parameters']['joints']) == 31
    if overrides:
        assert parameters[-1] == overrides


def test_record_all_preserves_startup_stages_and_shutdown_timeouts():
    module = _load_launch('record_all.launch.py')
    description = module.generate_launch_description()
    arguments = {
        entity.name: _default(entity)
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert arguments['sigterm_timeout'] == '40'
    assert arguments['sigkill_timeout'] == '10'
    assert arguments['color_profile'] == '1280x720x30'
    assert arguments['color_format'] == 'YUYV'
    assert module._STAGES == (
        ('hardware', 'robot_bringup', 'all_data.launch.py', 0.0),
        ('sensors', 'head_sensors', 'head_sensors.launch.py', 0.0),
        ('motion', 'robot_bringup', 'motion_tasks.launch.py', 6.0),
        ('localization', 'g1_localization', 'localization.launch.py', 8.0),
        ('record', 'record', 'record.launch.py', 12.0),
    )


parametrize = pytest.mark.parametrize


def test_all_data_scope_selects_expected_producers():
    module = _load_launch("all_data.launch.py")

    end_effectors = module._data_launches(
        _all_data_context("end_effectors", "single"))
    assert len(end_effectors) == 1
    assert all(isinstance(action, IncludeLaunchDescription)
               for action in end_effectors)
    assert "end_effectors_single_bus.launch.py" in _source_path(
        end_effectors[0])
    assert dict(end_effectors[0].launch_arguments)[
        "enable_grippers_on_start"].perform(
            _all_data_context("end_effectors", "single")) == "true"

    whole_body = module._data_launches(
        _all_data_context("whole_body", "dual"))
    assert len(whole_body) == 3
    assert all(isinstance(action, IncludeLaunchDescription)
               for action in whole_body)
    paths = [_source_path(action) for action in whole_body]
    assert any("end_effectors_dual_bus.launch.py" in path for path in paths)
    assert any("control.launch.py" in path for path in paths)
    assert any("end_effector_load.launch.py" in path for path in paths)
    assert all("dashboard" not in path.lower() for path in paths)
    control = next(
        action for action in whole_body
        if "control.launch.py" in _source_path(action))
    assert dict(control.launch_arguments)["arm_stiffness_scale"].perform(
        _all_data_context("whole_body", "dual")) == "2.5"


def test_end_effector_load_needs_whole_body_and_can_be_disabled():
    module = _load_launch("all_data.launch.py")

    def paths(*args, **kwargs):
        return [_source_path(action)
                for action in module._data_launches(
                    _all_data_context(*args, **kwargs))]

    # 补偿节点要关节角和躯干 IMU，末端 scope 下两者都不存在。
    assert not any("end_effector_load" in path
                   for path in paths("end_effectors"))
    assert not any("end_effector_load" in path
                   for path in paths("whole_body", end_effector_load="false"))


def test_all_data_rejects_unknown_scope():
    module = _load_launch("all_data.launch.py")
    with pytest.raises(ValueError, match="scope must be"):
        module._data_launches(_all_data_context("bad"))


def test_all_data_rejects_unknown_topology():
    module = _load_launch("all_data.launch.py")
    with pytest.raises(ValueError, match="topology must be"):
        module._data_launches(_all_data_context("end_effectors", "bad"))


def test_end_effectors_dashboard_expands_to_one_web_node():
    module = _load_launch("end_effectors_dashboard.launch.py")
    context = LaunchContext()
    context.launch_configurations.update({
        "topology": "dual",
        "web_host": "0.0.0.0",
        "web_port": "8770",
        "request_timeout_s": "3.0",
        "state_stale_s": "1.0",
        "left_camera_url": "http://127.0.0.1:8010",
        "right_camera_url": "http://127.0.0.1:8011",
        "camera_timeout_s": "1.0",
        "camera_stream_timeout_s": "10.0",
        "camera_poll_period_s": "2.0",
    })
    actions = module._dashboard_node(context)
    assert len(actions) == 1
    assert isinstance(actions[0], Node)
    assert _node_package(actions[0]) == "robot_bringup"
    assert _node_executable(actions[0]) == "end_effectors_dashboard"


def test_whole_body_dashboard_only_creates_compat_web_node():
    module = _load_launch("whole_body_dashboard.launch.py")
    description = module.generate_launch_description()
    includes = [
        entity for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    assert includes == []
    nodes = [
        entity for entity in description.entities
        if isinstance(entity, Node)
    ]
    assert len(nodes) == 1
    assert _node_package(nodes[0]) == "robot_bringup"
    assert _node_executable(nodes[0]) == "whole_body_dashboard"


def test_lowlevel_dashboard_expands_to_one_read_only_web_node():
    module = _load_launch("lowlevel_dashboard.launch.py")
    context = LaunchContext()
    context.launch_configurations.update({
        "web_host": "0.0.0.0",
        "web_port": "8210",
        "lowstate_topic": "/lf/lowstate",
        "secondary_imu_topic": "/lf/secondary_imu",
        "idle_release_s": "3.0",
    })
    actions = module._dashboard_node(context)
    assert len(actions) == 1
    assert isinstance(actions[0], Node)
    assert _node_package(actions[0]) == "robot_bringup"
    assert _node_executable(actions[0]) == "lowlevel_dashboard"


@parametrize(
    "launch_dir,name,package,executable",
    [
        (WORKSPACE_SRC / "kwr57_ros" / "launch",
         "web_demo.launch.py", "kwr57_ros", "web_wrench"),
        (WORKSPACE_SRC / "gloria_ros" / "launch",
         "web_gripper.launch.py", "gloria_ros", "web_gripper"),
        (WORKSPACE_SRC / "kwr57_ros" / "launch",
         "ft_sensor.launch.py", "kwr57_ros", "ft_sensor_node"),
        (WORKSPACE_SRC / "gloria_ros" / "launch",
         "gripper.launch.py", "gloria_ros", "gripper_node"),
    ],
)
def test_single_device_launches_only_create_expected_node(
        launch_dir, name, package, executable):
    module = _load_launch(name, launch_dir)
    description = module.generate_launch_description()
    nodes = [
        entity for entity in description.entities
        if isinstance(entity, Node)
    ]
    includes = [
        entity for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    assert includes == []
    assert len(nodes) == 1
    assert _node_package(nodes[0]) == package
    assert _node_executable(nodes[0]) == executable


def test_gripper_debug_owns_bridge_and_reuses_device_launch():
    module = _load_launch(
        "gripper_debug.launch.py", WORKSPACE_SRC / "gloria_ros" / "launch")
    context = LaunchContext()
    context.launch_configurations.update({
        "node_name": "grip_left",
        "command_id": "0x01",
        "feedback_id": "0x101",
        "safe_position_min": "0.0",
        "safe_position_max": "2.77",
    })
    actions = module._launch_nodes(context)
    assert len(actions) == 2
    assert isinstance(actions[0], Node)
    assert _node_package(actions[0]) == "can_bridge_ros"
    assert isinstance(actions[1], IncludeLaunchDescription)
    assert "gripper.launch.py" in _source_path(actions[1])
    launch_arguments = dict(actions[1].launch_arguments)
    assert launch_arguments == {
        "rx_topic": "/can0/grip_left/rx",
        "tx_topic": "/can0/tx",
        "command_id": "1",
        "feedback_id": "257",
        "safe_position_min": launch_arguments["safe_position_min"],
        "safe_position_max": launch_arguments["safe_position_max"],
        "enable_on_start": "false",
        "diagnostic_period_s": "1.0",
        "joint_name": "grip_left",
        "node_name": "grip_left",
    }
    assert _perform(context, launch_arguments["safe_position_min"]) == "0.0"
    assert _perform(context, launch_arguments["safe_position_max"]) == "2.77"


@parametrize("use_frame_handler,action_count", [
    ("true", 1),
    ("false", 2),
])
def test_ft_sensor_debug_owns_bridge_and_reuses_fallback_launch(
        use_frame_handler, action_count):
    module = _load_launch(
        "ft_sensor_debug.launch.py", WORKSPACE_SRC / "kwr57_ros" / "launch")
    context = LaunchContext()
    context.launch_configurations.update({
        "bridge_config": "single_bus.yaml",
        "use_frame_handler": use_frame_handler,
        "channel_id": "0",
        "bus_name": "can0",
        "cmd_id": "16",
        "data_base_id": "21",
        "topic": "/kwr57_ft_sensor/wrench_raw",
        "frame_id": "kwr57_ft_sensor_link",
        "period_ms": "1",
        "sample_rate_hz": "1000",
        "publish_rate": "0.0",
        "use_si": "false",
        "tare_on_start": "false",
    })
    actions = module._launch_nodes(context)
    assert len(actions) == action_count
    assert isinstance(actions[0], Node)
    assert _node_package(actions[0]) == "can_bridge_ros"
    if use_frame_handler == "false":
        assert isinstance(actions[1], IncludeLaunchDescription)
        assert "ft_sensor.launch.py" in _source_path(actions[1])
        assert dict(actions[1].launch_arguments) == {
            "rx_topic": "/can0/kwr57/rx",
            "tx_topic": "/can0/tx",
            "cmd_id": "16",
            "data_base_id": "21",
            "topic": "/kwr57_ft_sensor/wrench_raw",
            "frame_id": "kwr57_ft_sensor_link",
            "period_ms": "1",
            "sample_rate_hz": "1000",
            "publish_rate": "0.0",
            "use_si": "false",
            "autostart": "true",
            "tare_on_start": "false",
        }


def _numeric_defaults(tree: ast.Module) -> Dict[str, str]:
    """Launch-argument defaults that look like numbers, by argument name.

    Covers both spellings used here: an explicit ``DeclareLaunchArgument`` call
    and a module-level ``{name: default}`` table fed into one in a loop.
    """
    found: Dict[str, str] = {}

    def remember(name: object, default: object) -> None:
        if not isinstance(name, str) or not isinstance(default, str):
            return
        try:
            float(default)
        except ValueError:
            return
        found[name] = default

    for node in ast.walk(tree):
        if isinstance(node, ast.Dict):
            for key, value in zip(node.keys, node.values):
                if isinstance(key, ast.Constant) and isinstance(value, ast.Constant):
                    remember(key.value, value.value)
        elif (isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and
                node.func.id == "DeclareLaunchArgument"):
            name = node.args[0].value if (
                node.args and isinstance(node.args[0], ast.Constant)) else None
            default = next(
                (kw.value.value for kw in node.keywords
                 if kw.arg == "default_value" and isinstance(kw.value, ast.Constant)),
                None)
            remember(name, default)
    return found


def _bare_launch_configuration_parameters(tree: ast.Module) -> List[str]:
    """Node ``parameters=`` entries whose value is a bare ``LaunchConfiguration``.

    Only node parameters are typed by rclpy. ``IncludeLaunchDescription``'s
    ``launch_arguments`` stay strings all the way down, so they must not be
    flagged.
    """
    bare: List[str] = []
    for node in ast.walk(tree):
        if not (isinstance(node, ast.keyword) and node.arg == "parameters"):
            continue
        for inner in ast.walk(node.value):
            if not isinstance(inner, ast.Dict):
                continue
            for key, value in zip(inner.keys, inner.values):
                if not (isinstance(key, ast.Constant) and isinstance(key.value, str)):
                    continue
                if (isinstance(value, ast.Call) and isinstance(value.func, ast.Name) and
                        value.func.id == "LaunchConfiguration"):
                    bare.append(key.value)
    return bare


def test_numeric_parameters_pin_their_type():
    """Numeric node parameters must never be a bare ``LaunchConfiguration``.

    Launch arguments are always strings and rclpy infers the parameter type from
    the literal, so ``max_joint_speed:=3`` becomes an INTEGER and the node dies
    with ``InvalidParameterTypeException`` against a DOUBLE declaration. Foxy
    accepted the mismatch silently; Humble does not. Wrapping the substitution
    in ``ParameterValue(..., value_type=...)`` pins the type whatever the
    operator types.
    """
    offenders = {}
    for launch_file in sorted(LAUNCH_DIR.glob("*.launch.py")):
        tree = ast.parse(launch_file.read_text())
        numeric = _numeric_defaults(tree)
        bare = sorted(name for name in _bare_launch_configuration_parameters(tree)
                      if name in numeric)
        if bare:
            offenders[launch_file.name] = bare
    assert not offenders, f"wrap these in ParameterValue(value_type=...): {offenders}"
