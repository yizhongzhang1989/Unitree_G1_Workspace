"""验证配置分层、默认值与 launch 覆盖。"""


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


def test_configuration_layout():
    assert BACKEND_MODULES == [name_of(p) for p in BACKEND_CONFIGS]
    common = load(COMMON)
    assert common['vla_backend'] in BACKEND_MODULES
    for path in BACKEND_CONFIGS:
        declared = set(backend_parameters(name_of(path)))
        assert set(load(path)) <= declared
        assert not set(common) & declared


@pytest.mark.parametrize('path', BACKEND_CONFIGS, ids=name_of)
def test_backend_config_loads(path, monkeypatch):
    """真的能造出 backend 来，顺便挡住写错类型/长度的标定值。"""
    from g1_vla_bridge.backends.cogact_unitree import CogACTUnitreeBackend
    from g1_vla_bridge.vla_backend import load_backend

    def no_configure(self):
        _ = self

    monkeypatch.setattr(CogACTUnitreeBackend, 'configure', no_configure)
    params = dict(backend_parameters(name_of(path)))
    params.update(load(path))
    load_backend(name_of(path), params).close()


@pytest.mark.parametrize('override', ['', '15', '30'])
def test_history_length_launch_override(monkeypatch, override):
    import ament_index_python.packages
    from launch import LaunchContext
    import launch_ros.actions

    def get_package_share_directory(name):
        _ = name
        return PACKAGE

    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory', get_package_share_directory)
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
    def node_init(self, name):
        _ = self, name

    def declare_parameter(self, name, default):
        _ = self, name
        return SimpleNamespace(value=default, get_parameter_value=lambda: SimpleNamespace(string_value=default))

    def fake_load_backend(name, params):
        _ = name, params
        return backend

    monkeypatch.setattr(vla_node.Node, '__init__', node_init)
    monkeypatch.setattr(vla_node.Node, 'declare_parameter', declare_parameter)
    monkeypatch.setattr(vla_node, 'load_backend', fake_load_backend)
    history_factory = Mock(side_effect=HistoryInitialized)
    monkeypatch.setattr(vla_node, 'ControlHistory', history_factory)
    with pytest.raises(HistoryInitialized):
        vla_node.VlaBridgeNode()
    history_factory.assert_called_once_with(history_length=history_length)


def test_default_execution_is_thirty_hz_with_ten_hz_predictions():
    params = load(COMMON)
    assert params['execution_rate_hz'] == 30.0
    assert params['action_rate_hz'] == params['observation_rate_hz'] == 10.0
    assert not params['cartesian_limit_enabled']
    assert params['execution_mode'] == 'manual'
    assert params['debug_image_dir'] == ''
