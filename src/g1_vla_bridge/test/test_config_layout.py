"""配置分层：通用参数与各家 VLA 的参数必须严格分开。

拆错文件不会报错，只会「改了没生效」或者「换 backend 后拿的还是上一家的值」，
现场很难当场认出来，所以在这里机械核对。
"""

import glob
import os

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


def test_common_config_covers_the_node_parameters():
    """``vla_backend`` 必须在通用那份里——launch 要靠它决定加载哪个 backend yaml。"""
    assert load(COMMON)['vla_backend'] in BACKEND_MODULES
