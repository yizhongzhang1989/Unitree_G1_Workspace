"""URDF 解析与手柄触发的校准快捷键。不需要头显，也不需要网络。"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from g1_mocap.skeleton import both_thumbsticks_pressed, parse_body, parse_controllers
from g1_mocap.urdf import mesh_url, parse, rpy_to_quat, under

URDF_PATH = (Path(__file__).resolve().parents[2] / 'unitree_g1_description' / 'model'
             / 'g1_description' / 'g1_29dof_mode_15.urdf')


@pytest.fixture(scope='module')
def model() -> dict:
    return parse(URDF_PATH.read_text(encoding='utf-8'), 'pelvis')


def test_joints_come_out_parent_before_child(model):
    """前端按这个顺序往 Object3D 树上挂，父节点必须先出现，否则整段肢体挂不上。"""
    seen = {model['base']}
    for joint in model['joints']:
        assert joint['parent'] in seen, joint['name']
        seen.add(joint['child'])
    assert len(model['joints']) >= 29


def test_every_link_with_a_visual_has_a_resolvable_mesh(model):
    """G1 的 URDF 用的是裸相对路径，不是 package://。只认后者会一个 mesh 都出不来。"""
    assert len(model['links']) >= 25
    for link in model['links']:
        for visual in link['visuals']:
            assert len(visual['xyz']) == 3 and len(visual['quat']) == 4
            if visual['kind'] == 'mesh':
                assert visual['url'].startswith('/mesh?path=')
                assert len(visual['scale']) == 3


def test_primitives_survive_parsing():
    """挂在腕上的 KWR57B 是圆柱不是 mesh。只认 mesh 的话那一截会凭空消失且不报错。"""
    urdf = """<robot name="t">
      <link name="pelvis"/>
      <link name="tip">
        <visual><origin xyz="0 0 0.1"/><geometry>
          <cylinder radius="0.03" length="0.08"/></geometry></visual>
      </link>
      <link name="b"><visual><geometry><box size="0.1 0.2 0.3"/></geometry></visual></link>
      <link name="s"><visual><geometry><sphere radius="0.05"/></geometry></visual></link>
      <joint name="j" type="fixed">
        <parent link="pelvis"/><child link="tip"/></joint>
      <joint name="j2" type="fixed"><parent link="tip"/><child link="b"/></joint>
      <joint name="j3" type="fixed"><parent link="b"/><child link="s"/></joint>
    </robot>"""
    shapes = {link['name']: link['visuals'][0] for link in parse(urdf, 'pelvis')['links']}
    assert shapes['tip']['kind'] == 'cylinder'
    assert (shapes['tip']['radius'], shapes['tip']['length']) == (0.03, 0.08)
    assert shapes['tip']['xyz'] == [0.0, 0.0, 0.1]
    assert shapes['b']['kind'] == 'box' and shapes['b']['size'] == [0.1, 0.2, 0.3]
    assert shapes['s']['kind'] == 'sphere' and shapes['s']['radius'] == 0.05


def test_mesh_url_rejects_traversal():
    """/mesh 的参数来自网络而面板默认听 0.0.0.0，放开任意路径就是任意文件读。"""
    assert mesh_url('g1_description/meshes/pelvis.STL') == \
        '/mesh?path=g1_description/meshes/pelvis.STL'
    assert mesh_url('package://unitree_g1_description/model/a.STL') == '/mesh?path=model/a.STL'
    assert mesh_url('/etc/passwd') == ''
    assert mesh_url('../../../etc/passwd') == ''
    assert mesh_url('a/../../b.STL') == ''
    assert mesh_url('') == ''


def test_under_rejects_traversal():
    base = Path('/tmp/base')
    assert under(base, 'a/b.STL') == base / 'a' / 'b.STL'
    assert under(base, '/etc/passwd') is None
    assert under(base, '../escape') is None
    assert under(base, 'a/../../escape') is None
    assert under(base, '') is None


def test_rpy_is_fixed_axis_not_intrinsic():
    """URDF 的 rpy 是固定轴外旋 X->Y->Z，而 three.js 的 Euler 默认内旋，正好反过来。

    单轴关节看不出差别，三个分量都非零的 visual 会整个歪掉——所以这一步放在后端，
    好被这条测试钉住。
    """
    x, y, z, w = rpy_to_quat('0 0 1.5707963')
    assert (x, y) == (0.0, 0.0)
    assert z == pytest.approx(math.sin(0.5707963 / 2 + 0.5), abs=0.01)

    # R = Rz(y)Ry(p)Rx(r)：绕 x 转 90 度再绕 z 转 90 度，等价于绕 (1,1,1)/sqrt(3) 转 120 度
    quat = rpy_to_quat('1.5707963 0 1.5707963')
    expected = 0.5
    assert quat[0] == pytest.approx(expected, abs=1e-6)
    assert quat[1] == pytest.approx(expected, abs=1e-6)
    assert quat[2] == pytest.approx(expected, abs=1e-6)
    assert quat[3] == pytest.approx(expected, abs=1e-6)


def test_missing_base_is_rejected():
    with pytest.raises(ValueError, match='没有子关节'):
        parse(URDF_PATH.read_text(encoding='utf-8'), 'no_such_link')


##
# 手柄快捷键
##

def controllers(left: bool, right: bool, connected: bool = True) -> dict:
    return {hand: {'connected': connected,
                   'buttons': {'thumbstick_pressed': pressed}}
            for hand, pressed in (('left', left), ('right', right))}


def pressed(payload: dict) -> bool:
    return both_thumbsticks_pressed(parse_controllers(payload))


def test_both_thumbsticks_needed():
    assert pressed(controllers(True, True)) is True
    assert pressed(controllers(True, False)) is False
    assert pressed(controllers(False, True)) is False
    assert pressed(controllers(False, False)) is False


def test_disconnected_controller_does_not_trigger():
    """手柄没连的时候 buttons 里的值不可信，不能当成按下。"""
    assert pressed(controllers(True, True, connected=False)) is False


@pytest.mark.parametrize('payload', [
    {}, {'left': None, 'right': None},
    {'left': {'connected': True}, 'right': {'connected': True}},
    {'left': {'connected': True, 'buttons': []},
     'right': {'connected': True, 'buttons': []}},
])
def test_malformed_controller_payloads_are_safe(payload):
    """报文来自网络。缺字段、类型不对都只能是「没按下」，不能抛。"""
    assert pressed(payload) is False


def test_controller_parsing_survives_garbage():
    """这条流服务于急停，宁可读到「没按」也不能断——坏字段一律退化成默认值。"""
    left, right = parse_controllers({
        'left': {'connected': True, 'battery': 'x',
                 'buttons': {'trigger': None, 'b_y': True, 'menu': 1},
                 'thumbstick': [float('nan'), 0.5]},
        'right': {'connected': True,
                  'buttons': {'trigger': 0.75, 'a_x': True},
                  'thumbstick': 'bad'},
    })
    assert left.connected and left.b_y and left.menu
    assert left.trigger == 0.0 and left.battery == 0.0
    assert left.thumbstick == (0.0, 0.5), '非有限值要退化成 0，不能带进下游'
    assert right.trigger == 0.75 and right.a_x and not right.b_y
    assert right.thumbstick == (0.0, 0.0)


def test_controllers_are_independent_of_the_skeleton():
    """骨架坏掉时手柄照样要能解出来：最需要急停的时刻正是数据流出问题的时刻。"""
    payload = {'left': {'connected': True, 'buttons': {'b_y': True}},
               'right': {'connected': True, 'buttons': {'b_y': True}},
               'body': {'status': 0, 'joints': {}}}
    assert parse_body(payload) is None, '骨架确实是坏的'
    left, right = parse_controllers(payload)
    assert left.b_y and right.b_y
