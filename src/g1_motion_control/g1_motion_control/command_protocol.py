"""``/motion_control/command`` 的唯一分块与校验实现，不依赖 ROS。"""

from __future__ import annotations

import numpy as np

BLOCK = {'base': 4, 'left': 7, 'right': 7, 'grip': 2}
LAYOUT = {sum(BLOCK[name] for name in fields): fields for fields in (
    ('grip',), ('base',), ('right',), ('left', 'right'),
    ('base', 'left', 'right', 'grip'))}


def split_command(values, *, arm_poses: bool = True) -> dict[str, np.ndarray]:
    """按长度分块；非法帧抛 ``ValueError``。

    ``arm_poses`` 为真时那两个 7 值块是末端位姿，四元数在这里校验并归一化；
    逐关节透传模式下同一个块是 7 个关节角，没有四元数可言，所以这道校验必须
    跟着模式走，不能写死在协议里。分块长度两种模式完全相同。
    """
    # 一次拷贝后各块都只是视图；既不改 ROS message，也不为 2/4/7/14/20 再逐块分配。
    data = np.array(values, dtype=np.float64, copy=True)
    fields = LAYOUT.get(data.size)
    if fields is None or not np.all(np.isfinite(data)):
        raise ValueError(f'长度 {data.size} 不在 {sorted(LAYOUT)} 里，或含非有限值')
    chunks, offset = {}, 0
    for name in fields:
        chunks[name] = data[offset:offset + BLOCK[name]]
        offset += BLOCK[name]
    if arm_poses:
        for name in ('left', 'right'):
            pose = chunks.get(name)
            if pose is None:
                continue
            norm = float(np.linalg.norm(pose[3:]))
            if not 0.5 < norm < 2.0:
                raise ValueError(f'{name} 四元数模长 {norm:.3f} 异常')
            pose[3:] /= norm
    return chunks


def join_command(**chunks) -> list[float]:
    """把若干块按 ``BLOCK`` 的顺序拼成一帧；发布方由此不必自己数偏移量。"""
    fields = tuple(name for name in BLOCK if name in chunks)
    data = [float(v) for name in fields for v in chunks[name]]
    if LAYOUT.get(len(data)) != fields:
        raise ValueError(f'{fields} 不是 {sorted(LAYOUT)} 里的合法分块组合')
    return data
