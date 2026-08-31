"""URDF -> 前端要的关节树与 mesh 清单。只解析，不做运动学。

正运动学交给 three.js：``Object3D`` 的嵌套本来每帧就要合成矩阵，后端再用 pinocchio
算一遍纯属白花钱。所以这里只发一次静态结构，运行时只发 29 个关节角。

安全上有两条硬线，都在 :func:`under` 和 :func:`mesh_url` 里：mesh 的路径参数来自
网络，而面板默认听 ``0.0.0.0``，放开任意路径就是一个任意文件读。
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.parse import quote

MOVABLE = ('revolute', 'continuous', 'prismatic')


def _floats(text: str | None, fallback: tuple) -> list:
    if not text:
        return list(fallback)
    values = [float(v) for v in text.split()]
    return values if len(values) == 3 else list(fallback)


def rpy_to_quat(text: str | None) -> list:
    """URDF 的 ``rpy`` -> 四元数 ``[x, y, z, w]``。

    URDF 的 rpy 是**固定轴（外旋）X→Y→Z**，即 ``R = Rz(y)·Ry(p)·Rx(r)``。转换放在
    后端而不是前端，是因为 three.js 的 ``Euler`` 默认是**内旋** ``"XYZ"``，正好反过来：
    单轴关节看不出差别，而 ``rpy`` 三个分量都非零的那些 visual 会整个歪掉。
    """
    roll, pitch, yaw = (v / 2.0 for v in _floats(text, (0.0, 0.0, 0.0)))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy]


def mesh_url(filename: str) -> str:
    """URDF 里的 mesh 路径 -> ``/mesh`` 代理 URL。绝对路径和越界写法一律丢掉。

    G1 的 URDF 用的是**裸相对路径**（``g1_description/meshes/*.STL``），相对于
    描述包的 ``model/`` 目录；``package://`` 也一并认，两种写法都能用。
    """
    prefix = 'package://'
    relative = filename
    if filename.startswith(prefix):
        _, _, relative = filename[len(prefix):].partition('/')
    if not relative or Path(relative).is_absolute() or '..' in Path(relative).parts:
        return ''
    return f'/mesh?path={quote(relative)}'


def under(base: Path, relative: str) -> Path | None:
    """把 ``relative`` 接到 ``base`` 下，越界就返回 None。

    校验的是**相对路径本身**，不是拼出来再 ``resolve()`` 比前缀——
    ``--symlink-install`` 把 share/ 里的每个文件都做成了指回 src/ 的符号链接，
    ``resolve()`` 一路跟过去就跑出了 base，所有 mesh 都会 404。
    """
    path = Path(relative)
    if not relative or path.is_absolute() or '..' in path.parts:
        return None
    return base / path


def parse(urdf: str, base: str) -> dict:
    """URDF -> ``{'base', 'joints', 'links'}``，关节按深度优先排好序。

    前端按这个顺序往 ``Object3D`` 树上挂，父节点保证先于子节点出现。
    """
    root = ET.fromstring(urdf)
    children: dict[str, list] = {}
    for element in root.findall('joint'):
        parent, child = element.find('parent'), element.find('child')
        if parent is None or child is None:
            continue
        origin = element.find('origin')
        axis = element.find('axis')
        children.setdefault(parent.get('link'), []).append({
            'name': element.get('name'),
            'type': element.get('type', 'fixed'),
            'parent': parent.get('link'),
            'child': child.get('link'),
            'xyz': _floats(origin.get('xyz') if origin is not None else None, (0, 0, 0)),
            'quat': rpy_to_quat(origin.get('rpy') if origin is not None else None),
            'axis': _floats(axis.get('xyz') if axis is not None else None, (1, 0, 0)),
        })
    if base not in children:
        raise ValueError(f'URDF 里 {base} 没有子关节，base 写错了？')

    def branch(link: str) -> list:
        out = []
        for joint in children.get(link, ()):
            out.append(joint)
            out.extend(branch(joint['child']))
        return out

    joints = branch(base)
    wanted = {base} | {joint['child'] for joint in joints}
    links = []
    for element in root.findall('link'):
        name = element.get('name')
        if name not in wanted:
            continue
        visuals = []
        for visual in element.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                continue
            url = mesh_url(mesh.get('filename') or '')
            if not url:
                continue
            origin = visual.find('origin')
            # visual 自己的 origin 很容易被漏掉，漏了整个零件就错位。
            visuals.append({
                'url': url,
                'xyz': _floats(origin.get('xyz') if origin is not None else None, (0, 0, 0)),
                'quat': rpy_to_quat(origin.get('rpy') if origin is not None else None),
                'scale': _floats(mesh.get('scale'), (1, 1, 1)),
            })
        if visuals:
            links.append({'name': name, 'visuals': visuals})
    if not links:
        raise ValueError(f'{base} 之下一个 mesh 都解析不了，检查 URDF 里的 filename 写法')
    return {'base': base, 'joints': joints, 'links': links}
