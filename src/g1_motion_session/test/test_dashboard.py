"""离线校验 URDF 几何、路径边界、权重路由和 launch 组合"""

import io
import json
import runpy
import threading
import xml.etree.ElementTree as ET
from email.message import Message
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from g1_motion_session.joint_output import JointOutput

from g1_motion_session.dashboard_node import (
    DashboardNode,
    _handler,
    mesh_url,
    parse_urdf,
    rpy_to_quat,
    under,
)

BASE = 'torso_link'


def _raw() -> str:
    return (Path(get_package_share_directory('unitree_g1_description'))
            / 'model' / 'final.urdf').read_text(encoding='utf-8')


def _urdf() -> str:
    """按 control.launch.py 的发布约定补齐 package:// mesh 路径"""
    return _raw().replace(
        'filename="', 'filename="package://unitree_g1_description/model/')


@pytest.fixture(scope='module')
def model():
    return parse_urdf(_urdf(), BASE)


def test_movable_head_and_arms_survive(model):
    links = {link['name'] for link in model['links']}
    assert not links & {'logo_link', 'd435_link', 'mid360_link',
                        'imu_in_torso', 'pelvis', 'left_hip_pitch_link'}
    assert not any(name.endswith('kwr57b_link') for name in links)
    assert 'head_link' in links
    for side in ('left', 'right'):
        assert f'{side}_shoulder_pitch_link' in links
        assert f'{side}_gripper_base' in links
    names = [joint['name'] for joint in model['joints']]
    head_pitch = next(joint for joint in model['joints'] if joint['name'] == 'head_pitch_joint')
    assert head_pitch['type'] == 'revolute'
    assert head_pitch['parent'] == BASE
    head = next(joint for joint in model['joints'] if joint['name'] == 'head_joint')
    assert head['parent'] == head_pitch['child']
    assert head['child'] == 'head_link'
    assert 'left_elbow_joint' in names and 'right_eccentric_joint' in names
    # 腰腿应在基座之外，用于区分错误的 base_frame
    assert not any('hip' in name or 'waist' in name for name in names)


def test_joints_come_in_parent_before_child_order(model):
    """前端按这个顺序一遍搭树、一遍解 mimic，父在子前是硬前提"""
    seen = {BASE}
    resolved, mimics = set(), 0
    for joint in model['joints']:
        assert joint['parent'] in seen, f'{joint["name"]} 的父 link 还没出现'
        seen.add(joint['child'])
        mimic = joint.get('mimic')
        if mimic is not None:
            mimics += 1
            assert mimic['joint'] in resolved, f'{joint["name"]} 的 mimic 源在它后面'
        resolved.add(joint['name'])
    assert mimics > 0, '这份 URDF 没有 mimic 关节，用例失去意义'


def test_every_visual_has_a_resolvable_url(model):
    assert model['links'], '一个带 mesh 的 link 都没有'
    for link in model['links']:
        assert link['visuals']
        for visual in link['visuals']:
            assert visual['url'].startswith('/mesh?pkg=unitree_g1_description&path=model')


def test_unresolvable_meshes_fail_loudly():
    with pytest.raises(ValueError):
        parse_urdf(_raw(), BASE)


_TINY = """<robot name="t">
  <link name="torso"/><link name="head"/>
  <link name="arm">
    <visual><geometry><mesh filename="package://p/a.stl"/></geometry></visual>
  </link>
  <joint name="j_head" type="fixed">
    <parent link="torso"/><child link="head"/>
  </joint>
  <joint name="j_arm" type="{arm_type}">
    <parent link="torso"/><child link="arm"/><axis xyz="0 1 0"/>
  </joint>
</robot>"""


def test_fixed_only_branches_are_pruned():
    model = parse_urdf(_TINY.format(arm_type='revolute'), 'torso')
    assert [joint['name'] for joint in model['joints']] == ['j_arm']
    with pytest.raises(ValueError):
        parse_urdf(_TINY.format(arm_type='fixed'), 'torso')


def test_rpy_matches_pinocchio(model):
    """以 pinocchio 为基准区分 URDF 固定轴 RPY 与内旋 XYZ"""
    pin = pytest.importorskip('pinocchio')
    origins = {element.get('name'): element.find('origin')
               for element in ET.fromstring(_raw()).findall('joint')}
    checked = 0
    for joint in model['joints']:
        origin = origins[joint['name']]
        rpy = (origin.get('rpy') if origin is not None else None) or '0 0 0'
        expected = pin.rpy.rpyToMatrix(*[float(v) for v in rpy.split()])
        got = pin.Quaternion(np.asarray(joint['quat'], dtype=float)).matrix()
        assert np.allclose(got, expected, atol=1e-12), joint['name']
        checked += 1
    assert checked > 50


def test_rpy_to_quat_is_not_the_intrinsic_convention():
    """双轴同时非零才能区分固定轴与内旋约定"""
    quat = rpy_to_quat('1.5707963267948966 0 1.5707963267948966')
    assert np.allclose(quat, [0.5, 0.5, 0.5, 0.5], atol=1e-12)


def test_bad_base_frame_fails_loudly():
    with pytest.raises(ValueError):
        parse_urdf(_urdf(), 'no_such_link')


def test_mesh_url_only_accepts_package_scheme():
    """``/mesh`` 的参数来自网络，放开任意路径就是一个任意文件读"""
    assert mesh_url('package://a b/c d.stl') == '/mesh?pkg=a%20b&path=c%20d.stl'
    for bad in ('file:///etc/passwd', '/etc/passwd', 'meshes/x.stl',
                'package://onlypkg', 'package:///nopkg.stl'):
        assert mesh_url(bad) == ''


def test_under_rejects_escapes_but_follows_symlinks(tmp_path):
    """相对路径需防穿越，但允许 symlink-install 的 share 链接指回源码"""
    (tmp_path / 'real').mkdir()
    (tmp_path / 'real' / 'page.html').write_text('hi')
    (tmp_path / 'share').mkdir()
    (tmp_path / 'share' / 'page.html').symlink_to(tmp_path / 'real' / 'page.html')
    page = under(tmp_path / 'share', 'page.html')
    assert page is not None
    assert page.read_text() == 'hi'
    for bad in ('', '../setup.py', 'a/../../b', '/etc/passwd'):
        assert under(tmp_path / 'share', bad) is None


def test_dashboard_observes_upper_task_status(dashboard):
    command = {'left': [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
               'right': [0.4, 0.5, 0.6, 0.0, 0.0, 0.0, 1.0]}
    limited = {'left': [0.11, 0.21, 0.31, 0.0, 0.0, 0.0, 1.0]}
    _runtime_status(dashboard, upper_node='/upper_vr', upper_task='vr', arm_mode='ik',
                    state='running', stale=False, command_pose=command,
                    limited_pose=limited, ik_pos_err=0.002, ik_ms=1.5,
                    ik_rotation_weight=0.03)
    assert dashboard.snapshot() == {
        'q': {}, 'state': 'running', 'stale': False,
        'command_pose': command, 'limited_pose': limited,
        'ik_pos_err': 0.002, 'ik_ms': 1.5, 'ik_rotation_weight': 0.03,
    }

    updated = {'right': [0.7, 0.8, 0.9, 0.0, 0.0, 0.0, 1.0]}
    _runtime_status(dashboard, upper_node='/upper_vla', upper_task='vla', arm_mode='ik',
                    command_pose=updated)
    assert dashboard.snapshot()['command_pose'] == updated


def test_dashboard_reports_empty_pose_from_current_status(dashboard):
    _runtime_status(dashboard, upper_node='/upper_vr', upper_task='vr', arm_mode='ik',
                    command_pose={'left': [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]})
    _runtime_status(dashboard, command_pose=None)
    assert dashboard.snapshot()['command_pose'] == {}


@pytest.fixture
def upper_status(request):
    raw = {'left': np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]),
           'right': np.array([0.4, -0.5, 0.6, 0.0, 0.0, 0.0, 1.0])}
    if request.param == 'vr':
        from upper_vr.vr_teleop import VRTeleop
        node = VRTeleop.__new__(VRTeleop)
        node._pose = raw
        node._gripper_slots = [29, 30]
    else:
        from upper_vla.vla_node import VlaBridgeNode
        node = VlaBridgeNode.__new__(VlaBridgeNode)
        node._command = raw
        node._grip_command = {'left': .4, 'right': .5}
    node._lock = threading.Lock()
    node.output = JointOutput(Mock(), '/test/targets', range(15, 31))
    node.arm_slots = np.arange(15, 29)
    node.status_ik = Mock(return_value={
        'arm_mode': 'ik', 'limited_pose': {'left': [0.11, 0.21, 0.31, 0., 0., 0., 1.]},
        'ik_rotation_weight': 0.03,
    })
    return node, raw, request.param


@pytest.mark.parametrize('upper_status', ['vr', 'vla'], indirect=True)
def test_upper_task_status_reaches_dashboard(upper_status, dashboard):
    from g1_motion_session.task_base import State, TaskBase
    node, raw, task = upper_status
    publisher = Mock()
    mask = sum(1 << slot for slot in range(15, 31))
    host = TaskBase.__new__(TaskBase)
    host._registered = True
    host.lock = threading.RLock()
    host.status = node.status
    host._events = {'status': node.status_ik}
    host.slots = list(range(15, 31))
    host.get_fully_qualified_name = Mock(return_value='/upper_' + task)
    host.state = State.RUNNING
    host.joints31 = [f'joint_{index}' for index in range(31)]
    host._preparing = False
    host._mask = mask
    host._epoch = 1
    host._error = ''
    host._task_status_publisher = publisher
    TaskBase._publish_task_status(host)
    detail = json.loads(publisher.publish.call_args.args[0].data)
    _runtime_status(dashboard, **detail,
                    tasks=[{'task_id': detail['task_id'], 'mask': detail['mask']}])
    expected = {side: pose.tolist() for side, pose in raw.items()}
    assert dashboard.snapshot()['command_pose'] == expected
    assert dashboard.snapshot()['limited_pose'] == node.status_ik()['limited_pose']
    assert dashboard._param_target == ('/upper_' + task, '/upper_' + task)
    detail['command_pose']['left'][0] = 99.0
    assert raw['left'][0] == 0.1
    raw['right'][0] = 0.7
    assert node.status()['command_pose']['right'][0] == 0.7
    raw.clear()
    assert node.status()['command_pose'] == {}


@pytest.mark.parametrize('upper_status', ['vla'], indirect=True)
def test_vla_private_status_reports_raw_command(upper_status):
    node, raw, _ = upper_status
    node._chunk, node._cursor = None, 0
    node._spec = SimpleNamespace(name='offline', images=SimpleNamespace(slots=()))
    node._execution_mode = 'manual'
    node._action_rate, node._execution_rate = 10.0, 50.0
    node._skip_intermediate = False
    node._cartesian_limit_enabled = True
    node._running = threading.Event()
    node._inference_active = False
    node._async_min_overlap = 1
    node._timed = None
    node._async_merge, node._async_step = None, {}
    node._async_last_publish = None
    node._task, node._infer_ms = 'offline', 0.0
    node._observation_timing = None
    node._lead, node._jump = 0.0, 0.0
    node._hold, node._error = set(), ''
    node._grip_command = {'left': 0.0, 'right': 0.0}
    node._readers = {}
    node._backend = SimpleNamespace(debug_dir='', stats=lambda: {})
    publisher = Mock()
    node._status_publisher = publisher
    node._publish_status()
    payload = json.loads(publisher.publish.call_args.args[0].data)
    assert payload['command_pose'] == {side: pose.tolist() for side, pose in raw.items()}
    assert payload['limited_pose'] == node.status_ik()['limited_pose']
    raw.clear()
    node._publish_status()
    assert json.loads(publisher.publish.call_args.args[0].data)['command_pose'] == {}


def _post(node: DashboardNode, body: bytes, length: int | None = None) -> dict[str, int | bytes]:
    """直接运行 handler 校验，不建立 HTTP 连接"""
    handler_type = _handler(node)
    handler = handler_type.__new__(handler_type)
    handler.path = '/api/ik_weight'
    handler.headers = Message()
    handler.headers['Content-Length'] = str(len(body) if length is None else length)
    handler.rfile = io.BytesIO(body)
    sent: dict[str, int | bytes] = {}
    handler._send = lambda code, body, kind, cache='no-store': sent.update(
        code=code, body=body)
    handler._json = lambda payload: sent.update(
        code=200, body=json.dumps(payload).encode())
    handler.do_POST()
    return sent


def test_ik_weight_endpoint_accepts_only_sane_values(dashboard):
    node = dashboard

    assert _post(node, b'{"value": 0.03}')['code'] == 200
    assert node._pending_weight == pytest.approx(0.03)

    node._pending_weight = None
    for bad in (b'{"value": 1.5}', b'{"value": -0.1}', b'{"value": NaN}',
                b'{"value": Infinity}', b'{"value": "x"}', b'{}', b'not json'):
        assert _post(node, bad)['code'] == 400, bad
        assert node._pending_weight is None, bad

    # 无效长度须在读取 body 前拒绝
    assert _post(node, b'{"value": 0.5}', length=999)['code'] == 400
    assert _post(node, b'', length=0)['code'] == 400
    assert node._pending_weight is None


@pytest.fixture
def dashboard() -> DashboardNode:
    node = DashboardNode.__new__(DashboardNode)
    node._lock = threading.Lock()
    node._status = {}
    node._joints = None
    node._names = []
    node._index = {}
    node._pending_weight = None
    node._param_client = None
    node._param_target = None
    node.create_client = Mock(side_effect=lambda *args: Mock())
    node.destroy_client = Mock()
    node.get_logger = Mock(return_value=Mock())
    return node


@pytest.fixture
def dashboard_clients(dashboard: DashboardNode) -> tuple[Mock, Mock, list[Mock]]:
    clients = [Mock(), Mock()]
    create = Mock(side_effect=clients)
    destroy = Mock()
    dashboard.create_client = create
    dashboard.destroy_client = destroy
    return create, destroy, clients


def _runtime_status(node: DashboardNode, **fields: object) -> None:
    node._on_status(String(data=json.dumps(fields)))


def test_weight_routes_to_current_upper_task(dashboard: DashboardNode, dashboard_clients):
    node = dashboard
    create, _, clients = dashboard_clients
    _runtime_status(node, upper_node='/upper_vr', upper_task='vr', arm_mode='ik')
    create.assert_called_once_with(create.call_args.args[0], '/upper_vr/set_parameters')
    client = clients[0]
    assert node._param_client is client
    node.request_rotation_weight(0.03)
    node._push_weight()
    parameter = client.call_async.call_args.args[0].parameters[0]
    assert parameter.name == 'ik_rotation_weight'
    assert parameter.value.double_value == pytest.approx(0.03)
    _runtime_status(node, upper_node='/upper_vr', upper_task='vr', arm_mode='ik')
    assert create.call_count == 1


def test_task_change_discards_pending_weight_and_replaces_client(dashboard: DashboardNode, dashboard_clients):
    node = dashboard
    _, destroy, clients = dashboard_clients
    _runtime_status(node, upper_node='/upper_vr', upper_task='vr', arm_mode='ik')
    previous = node._param_client
    node.request_rotation_weight(0.07)
    _runtime_status(node, upper_node='/upper_vr', upper_task='other', arm_mode='ik')
    destroy.assert_called_once_with(previous)
    assert node._pending_weight is None
    node._push_weight()
    assert node._param_client is clients[1]
    clients[1].call_async.assert_not_called()


def test_weight_uses_registered_task_identity(dashboard: DashboardNode, dashboard_clients):
    node = dashboard
    _, _, clients = dashboard_clients
    _runtime_status(node, upper_node='/upper_vr', arm_mode='ik',
                    tasks=[{'task_id': 'vr_runtime', 'mask': 1 << 15},
                           {'task_id': 'velocity', 'mask': 1}])
    assert node._param_target == ('/upper_vr', 'vr_runtime')
    node.request_rotation_weight(0.03)
    node._push_weight()
    assert node._param_client is clients[0]
    clients[0].call_async.assert_called_once()


@pytest.mark.parametrize('fields', [
    {}, {'upper_node': '/motion_control', 'upper_task': 'vr', 'arm_mode': 'ik'},
    {'upper_node': '/upper_vla', 'upper_task': 'vla', 'arm_mode': 'passthrough'},
])
def test_weight_without_an_ik_task_does_not_create_a_client(dashboard: DashboardNode, fields, dashboard_clients):
    node = dashboard
    create, _, _ = dashboard_clients
    _runtime_status(node, **fields)
    node.request_rotation_weight(0.03)
    node._push_weight()
    create.assert_not_called()


def test_dashboard_launch_forwards_monitoring_parameters(monkeypatch):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    import launch_ros.actions

    factory = Mock()
    monkeypatch.setattr(launch_ros.actions, 'Node', factory)
    path = Path(__file__).resolve().parents[1] / 'launch/dashboard.launch.py'
    module = runpy.run_path(str(path))
    context = LaunchContext()
    context.launch_configurations.update({
        action.name: ''.join(item.perform(context) for item in action.default_value or [])
        for action in module['generate_launch_description']().entities
        if isinstance(action, DeclareLaunchArgument)
    })
    module['_node'](context)
    parameters = factory.call_args.kwargs['parameters']
    assert Path(parameters[0]).name == 'session.yaml'
    assert Path(parameters[-2]).name == 'default_31dof_param.yaml'
    assert {name: value.perform(context) for name, value in parameters[-1].items()} == {
        'bind_host': '0.0.0.0', 'bind_port': '8181', 'base_frame': 'torso_link',
        'status_topic': '/motion_control/status', 'joint_states_topic': '/joint_states',
        'robot_description_topic': '/robot_description',
    }
