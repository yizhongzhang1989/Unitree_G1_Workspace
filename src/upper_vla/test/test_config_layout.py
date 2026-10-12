"""验证配置分层、默认值与 launch 覆盖。"""


import glob
import os
import runpy
from unittest.mock import Mock

import pytest
import yaml

from upper_vla.vla_backend import backend_parameters

PACKAGE = os.path.join(os.path.dirname(__file__), '..')
COMMON = os.path.join(PACKAGE, 'config', 'vla_bridge.yaml')
BACKEND_CONFIGS = sorted(glob.glob(os.path.join(PACKAGE, 'config', 'backends', '*.yaml')))
BACKEND_MODULES = sorted(
    os.path.basename(p)[:-3]
    for p in glob.glob(os.path.join(PACKAGE, 'upper_vla', 'backends', '*.py'))
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
    from upper_vla.backends.cogact_unitree import CogACTUnitreeBackend
    from upper_vla.vla_backend import load_backend

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
        return PACKAGE if name == 'upper_vla' else os.path.join(PACKAGE, '..', name)

    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory', get_package_share_directory)
    node_factory = Mock()
    monkeypatch.setattr(launch_ros.actions, 'Node', node_factory)
    module = runpy.run_path(os.path.join(PACKAGE, 'launch', 'vla_bridge.launch.py'))
    context = LaunchContext()
    context.launch_configurations.update({name: '' for name in module['_ARGUMENTS']})
    context.launch_configurations['config'] = 'vla_bridge.yaml'
    context.launch_configurations['history_length'] = override
    module['_node'](context)
    parameters = node_factory.call_args.kwargs['parameters']
    common, _, backend, overrides = parameters[2:]
    assert common == COMMON
    assert parameters[1].endswith('g1_motion_session/config/arm_ik.yaml')
    assert backend == os.path.join(PACKAGE, 'config', 'backends', 'cogact_unitree.yaml')
    assert load(backend)['history_length'] == backend_parameters('cogact_unitree')['history_length']
    if override:
        assert overrides['history_length'] == int(override)
    else:
        assert 'history_length' not in overrides


@pytest.mark.parametrize('cli_backend', [False, True])
def test_launch_uses_selected_common_config(tmp_path, monkeypatch, cli_backend):
    import ament_index_python.packages
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    import launch_ros.actions

    def get_package_share_directory(name):
        return PACKAGE if name == 'upper_vla' else os.path.join(PACKAGE, '..', name)

    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory', get_package_share_directory)
    node_factory = Mock()
    monkeypatch.setattr(launch_ros.actions, 'Node', node_factory)
    module = runpy.run_path(os.path.join(PACKAGE, 'launch', 'vla_bridge.launch.py'))
    default_context = LaunchContext()
    defaults = {}
    for action in module['generate_launch_description']().entities:
        if isinstance(action, DeclareLaunchArgument):
            assert action.default_value is not None
            defaults[action.name] = perform_substitutions(default_context, action.default_value)
    assert defaults['config'] == 'vla_bridge.yaml'
    common = load(COMMON)
    backend = 'calibrated_backend'
    backend_config = os.path.join(PACKAGE, 'config', 'backends', f'{backend}.yaml')
    path_exists = os.path.exists
    monkeypatch.setattr(os.path, 'exists', lambda path: path == backend_config or path_exists(path))
    custom = tmp_path / 'calibrated.yaml'
    custom.write_text(yaml.safe_dump({'/vla_bridge': {'ros__parameters': {
        **common, 'vla_backend': backend, 'execution_rate_hz': 7.0}}}))
    context = LaunchContext()
    context.launch_configurations.update({name: '' for name in module['_ARGUMENTS']})
    context.launch_configurations.update(config=str(custom), execution_rate_hz='12.5')
    if cli_backend:
        context.launch_configurations['vla_backend'] = common['vla_backend']
        backend = common['vla_backend']
    module['_node'](context)
    parameters = node_factory.call_args.kwargs['parameters']
    assert parameters[0].endswith('g1_motion_session/config/session.yaml')
    assert parameters[1].endswith('g1_motion_session/config/arm_ik.yaml')
    assert parameters[2] == COMMON
    assert parameters[3] == str(custom)
    assert parameters[-2] == os.path.join(PACKAGE, 'config', 'backends', f'{backend}.yaml')
    assert parameters[-1]['execution_rate_hz'] == 12.5
