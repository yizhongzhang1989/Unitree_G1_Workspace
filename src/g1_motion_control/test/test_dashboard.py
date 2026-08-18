"""dashboard_node 的离线校验。不需要真机、不需要 ROS 图、不起 HTTP。

只测和“画得对不对”直接相关的纯逻辑：URDF 裁剪出来的确实只有两条手臂、
rpy 用的是固定轴约定（写反了夹爪会被甩出手掌）、mimic 能被前端那套**单遍**
解算还原，以及静态文件 / mesh 的路径容纳不会被穿越。

外加一组：姿态权重那个 **写入** 端点的参数校验（它会改变求解行为）。
"""

import io
import json
import threading
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float64MultiArray

from g1_motion_control.dashboard_node import (
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
    """把裸相对 mesh 路径改写成 ``package://``，和 ``control.launch.py`` 一样。

    真机上 ``/robot_description`` 是被那一步处理过才发出来的；拿文件原文测
    等于测了一份 mesh 全解析不了的 URDF。
    """
    return _raw().replace(
        'filename="', 'filename="package://unitree_g1_description/model/')


@pytest.fixture(scope='module')
def model():
    return parse_urdf(_urdf(), BASE)


def test_only_the_two_arms_survive(model):
    """base_frame 之下只留含可动关节的分支——头/相机/雷达全是 fixed，必须剪掉"""
    links = {link['name'] for link in model['links']}
    assert not links & {'head_link', 'logo_link', 'd435_link', 'mid360_link',
                        'imu_in_torso', 'pelvis', 'left_hip_pitch_link'}
    for side in ('left', 'right'):
        assert f'{side}_shoulder_pitch_link' in links
        assert f'{side}_gripper_base' in links
    names = [joint['name'] for joint in model['joints']]
    assert 'left_elbow_joint' in names and 'right_eccentric_joint' in names
    # 腰和腿不在 torso_link 之下，本来就进不来；这一条防的是 base_frame 写错。
    assert not any('hip' in name or 'waist' in name for name in names)


def test_joints_come_in_parent_before_child_order(model):
    """前端按这个顺序一遍搭树、一遍解 mimic，父在子前是硬前提"""
    seen = {BASE}
    for joint in model['joints']:
        assert joint['parent'] in seen, f'{joint["name"]} 的父 link 还没出现'
        seen.add(joint['child'])


def test_mimic_sources_resolve_in_one_pass(model):
    """夹爪的 spline 链全是 mimic；源必须排在从动之前，否则单遍解算会漏"""
    resolved, mimics = set(), 0
    for joint in model['joints']:
        mimic = joint.get('mimic')
        if mimic is not None:
            mimics += 1
            assert mimic['joint'] in resolved, f'{joint["name"]} 的 mimic 源在它后面'
        resolved.add(joint['name'])
    assert mimics > 0, '这份 URDF 没有 mimic 关节，用例失去意义'


def test_every_visual_has_a_resolvable_url(model):
    """只收能翻成 ``/mesh`` 代理 URL 的 mesh。

    注意 ``final.urdf`` 里写的是裸相对路径，``control.launch.py`` 在发
    ``/robot_description`` 之前会统一改写成 ``package://``，这里照做。
    """
    assert model['links'], '一个带 mesh 的 link 都没有'
    for link in model['links']:
        assert link['visuals']
        for visual in link['visuals']:
            assert visual['url'].startswith('/mesh?pkg=unitree_g1_description&path=model')


def test_unresolvable_meshes_fail_loudly():
    """直接发 ``final.urdf`` 原文（裸相对路径）应该报错，而不是默默画个空场景。"""
    with pytest.raises(ValueError):
        parse_urdf(_raw(), BASE)


def test_non_mesh_geometry_is_skipped(model):
    """只画 mesh。手腕到夹爪之间 ``*_kwr57b_link`` 是圆柱基本体，故意不画

    所以那里会有 9.5 cm 空档，看着像夹爪脱开了手臂——**不是位置算错**
    （已拿 pinocchio 的 visual 几何逐个对过）。这一条把它钉住，免得再排查一遍
    """
    assert not any(link['name'].endswith('kwr57b_link') for link in model['links'])


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
    """判据是"分支里有没有可动关节"，不是关节名单——换 URDF 不用改配置"""
    model = parse_urdf(_TINY.format(arm_type='revolute'), 'torso')
    assert [joint['name'] for joint in model['joints']] == ['j_arm']
    with pytest.raises(ValueError):     # 全 fixed = 没什么可看的，报错而不是画空场景
        parse_urdf(_TINY.format(arm_type='fixed'), 'torso')


def test_rpy_matches_pinocchio(model):
    """URDF 的 rpy 是**固定轴**（外旋）X→Y→Z，写反了夹爪会被甩出手掌

    拿 pinocchio 的 ``rpyToMatrix`` 当基准逐个关节比，这一条锁的就是那次踩坑：
    three.js 的 ``Euler`` 默认是内旋 "XYZ"，正好和 URDF 反过来，单轴关节看不出来。
    """
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
    """两轴同时非零时两种约定才分得开——夹爪的 rpy 正是 `1.57 0 1.57`"""
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
    """symlink-install 把 share/ 里每个文件都做成了指回 src/ 的链接

    所以容纳性只能校验**相对路径本身**；拼完再 ``resolve()`` 比前缀会一路
    跟出 base，页面和 mesh 全变 404（已踩）。
    """
    (tmp_path / 'real').mkdir()
    (tmp_path / 'real' / 'page.html').write_text('hi')
    (tmp_path / 'share').mkdir()
    (tmp_path / 'share' / 'page.html').symlink_to(tmp_path / 'real' / 'page.html')
    assert under(tmp_path / 'share', 'page.html').read_text() == 'hi'
    for bad in ('', '../setup.py', 'a/../../b', '/etc/passwd'):
        assert under(tmp_path / 'share', bad) is None


def test_dashboard_observes_arm_blocks_without_clearing_other_fields():
    """2/4 不碰臂；7 只更新右臂；14/20 更新双臂，和 motion_control 契约一致。"""
    node = DashboardNode.__new__(DashboardNode)
    node._lock = threading.Lock()
    node._command_pose = {}

    full = Float64MultiArray()
    full.data = [0.0] * 4 + [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0] \
        + [0.4, 0.5, 0.6, 0.0, 0.0, 0.0, 1.0] + [0.0, 0.0]
    node._on_command(full)
    assert node._command_pose['left'][:3] == [0.1, 0.2, 0.3]
    assert node._command_pose['right'][:3] == [0.4, 0.5, 0.6]

    node._on_command(Float64MultiArray(data=[0.2, 0.0, 0.0, 0.74]))
    assert node._command_pose['left'][:3] == [0.1, 0.2, 0.3]

    node._on_command(Float64MultiArray(data=[0.7, 0.8, 0.9, 0.0, 0.0, 0.0, 1.0]))
    assert node._command_pose['left'][:3] == [0.1, 0.2, 0.3]
    assert node._command_pose['right'][:3] == [0.7, 0.8, 0.9]


def _post(node, body: bytes, length=None):
    """不起 HTTP，直接跑 do_POST：只想验那几条校验，不验 socket。"""
    handler = _handler(node).__new__(_handler(node))
    handler.path = '/api/ik_weight'
    handler.headers = {'Content-Length':
                       str(len(body) if length is None else length)}
    handler.rfile = io.BytesIO(body)
    sent = {}
    handler._send = lambda code, payload, kind, cache='no-store': sent.update(
        code=code, body=payload)
    handler._json = lambda payload: sent.update(
        code=200, body=json.dumps(payload).encode())
    handler.do_POST()
    return sent


def test_ik_weight_endpoint_accepts_only_sane_values():
    """这是页面上唯一会改变求解行为的写入口，越界值必须在进 ROS 之前就被拦住。"""
    node = DashboardNode.__new__(DashboardNode)
    node._lock = threading.Lock()
    node._pending_weight = None

    assert _post(node, b'{"value": 0.03}')['code'] == 200
    assert node._pending_weight == pytest.approx(0.03)

    node._pending_weight = None
    for bad in (b'{"value": 1.5}', b'{"value": -0.1}', b'{"value": NaN}',
                b'{"value": Infinity}', b'{"value": "x"}', b'{}', b'not json'):
        assert _post(node, bad)['code'] == 400, bad
        assert node._pending_weight is None, bad

    # 超大 body 不读；声明长度为 0 也不读。
    assert _post(node, b'{"value": 0.5}', length=999)['code'] == 400
    assert _post(node, b'', length=0)['code'] == 400
    assert node._pending_weight is None
