"""配置分层：通用参数与各家 VLA 的参数必须严格分开。

拆错文件不会报错，只会「改了没生效」或者「换 backend 后拿的还是上一家的值」，
现场很难当场认出来，所以在这里机械核对。
"""

import ast
import glob
import os
import runpy
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml

from g1_vla_bridge.vla_backend import backend_parameters

PACKAGE = os.path.join(os.path.dirname(__file__), '..')
COMMON = os.path.join(PACKAGE, 'config', 'vla_bridge.yaml')
BACKEND_CONFIGS = sorted(glob.glob(os.path.join(PACKAGE, 'config', 'backends', '*.yaml')))
BACKEND_MODULES = sorted(
    os.path.basename(p)[:-3]
    for p in glob.glob(os.path.join(PACKAGE, 'g1_vla_bridge', 'backends', '*.py'))
    if not p.endswith('__init__.py'))


def load(path) -> dict:
    with open(path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle)['/vla_bridge']['ros__parameters']


def name_of(path) -> str:
    return os.path.splitext(os.path.basename(path))[0]


def test_every_backend_module_has_a_config():
    """``backends/<名字>.py`` 与 ``config/backends/<名字>.yaml`` 一一对应。"""
    assert BACKEND_MODULES == [name_of(p) for p in BACKEND_CONFIGS]


@pytest.mark.parametrize('path', BACKEND_CONFIGS, ids=name_of)
def test_backend_config_only_holds_its_own_parameters(path):
    """backend yaml 里的每个键都得是那个 backend 真的会 declare 的。"""
    declared = set(backend_parameters(name_of(path)))
    assert set(load(path)) <= declared


@pytest.mark.parametrize('path', BACKEND_CONFIGS, ids=name_of)
def test_common_config_does_not_shadow_backend_parameters(path):
    """通用 yaml 不许碰 backend 的键——launch 里 backend 那份在后面，会把它盖掉。"""
    assert set(load(COMMON)) & set(backend_parameters(name_of(path))) == set()


@pytest.mark.parametrize('path', BACKEND_CONFIGS, ids=name_of)
def test_backend_config_loads(path):
    """真的能造出 backend 来，顺便挡住写错类型/长度的标定值。"""
    from g1_vla_bridge.vla_backend import load_backend

    params = dict(backend_parameters(name_of(path)))
    params.update(load(path))
    load_backend(name_of(path), params).close()


@pytest.mark.parametrize('override', ['', '15', '30'])
def test_history_length_launch_override(monkeypatch, override):
    import ament_index_python.packages
    from launch import LaunchContext
    import launch_ros.actions

    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory', lambda name: PACKAGE)
    node_factory = Mock()
    monkeypatch.setattr(launch_ros.actions, 'Node', node_factory)
    module = runpy.run_path(os.path.join(PACKAGE, 'launch', 'vla_bridge.launch.py'))
    context = LaunchContext()
    context.launch_configurations.update({name: '' for name in module['_ARGUMENTS']})
    context.launch_configurations['history_length'] = override
    module['_node'](context)
    common, backend, overrides = node_factory.call_args.kwargs['parameters']
    assert common == COMMON
    assert backend == os.path.join(PACKAGE, 'config', 'backends', 'cogact_unitree.yaml')
    assert load(backend)['history_length'] == backend_parameters('cogact_unitree')['history_length']
    if override:
        assert overrides['history_length'] == int(override)
    else:
        assert 'history_length' not in overrides


@pytest.mark.parametrize('history_length', [1, 15, 16, 30])
def test_node_initializes_history_with_backend_length(monkeypatch, history_length):
    from g1_vla_bridge import vla_node

    class HistoryInitialized(Exception):
        pass

    backend = SimpleNamespace(spec=None, history_enabled=True, history_length=history_length)
    monkeypatch.setattr(vla_node.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(vla_node.Node, 'declare_parameter', lambda self, name, default: SimpleNamespace(
        value=default, get_parameter_value=lambda: SimpleNamespace(string_value=default)))
    monkeypatch.setattr(vla_node, 'load_backend', lambda name, params: backend)
    history_factory = Mock(side_effect=HistoryInitialized)
    monkeypatch.setattr(vla_node, 'ControlHistory', history_factory)
    with pytest.raises(HistoryInitialized):
        vla_node.VlaBridgeNode()
    history_factory.assert_called_once_with(history_length=history_length)


def test_common_config_covers_the_node_parameters():
    """``vla_backend`` 必须在通用那份里——launch 要靠它决定加载哪个 backend yaml。"""
    assert load(COMMON)['vla_backend'] in BACKEND_MODULES


def test_default_execution_is_thirty_hz_with_ten_hz_predictions():
    params = load(COMMON)
    assert params['execution_rate_hz'] == 30.0
    assert params['action_rate_hz'] == params['observation_rate_hz'] == 10.0
    assert 'dry_run' not in params
    assert not params['cartesian_limit_enabled']
    assert params['execution_mode'] == 'manual'
    assert params['debug_image_dir'] == ''


def test_python_rate_defaults_match_yaml_without_overrides():
    from g1_vla_bridge.record_observation import ObservationBuffer

    with open(os.path.join(PACKAGE, 'g1_vla_bridge', 'vla_node.py'), encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    defaults = {
        call.args[0].value: call.args[1]
        for call in ast.walk(tree)
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
        and call.func.id == 'p' and len(call.args) >= 2
        and isinstance(call.args[0], ast.Constant)
    }
    params = load(COMMON)
    for name in ('action_rate_hz', 'observation_rate_hz'):
        assert ast.literal_eval(defaults[name]) == params[name] == 10.0
    assert ast.literal_eval(defaults['execution_rate_hz']) == params['execution_rate_hz'] == 30.0
    assert ObservationBuffer(()).rate == 10.0
