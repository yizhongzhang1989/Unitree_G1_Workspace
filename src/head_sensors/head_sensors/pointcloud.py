"""PointCloud2 与 numpy 之间的零拷贝转换。

Livox MID-360 经宇树 `lidar_driver` 发出的 `/utlidar/cloud_livox_mid360` 布局
（2026-08-13 实机读到）：point_step=22，字段
`x,y,z,intensity`(float32) + `ring`(uint16, offset 16) + `time`(float32, offset 18)。
这里不写死该布局，而是按 `msg.fields` 现场构造 numpy 结构化 dtype，
换固件或换雷达都不用改代码。
"""

from typing import Tuple

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

# PointField.datatype -> numpy 类型
_DATATYPE_TO_NP = {
    PointField.INT8: np.int8,
    PointField.UINT8: np.uint8,
    PointField.INT16: np.int16,
    PointField.UINT16: np.uint16,
    PointField.INT32: np.int32,
    PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
}


def dtype_from_fields(msg: PointCloud2) -> np.dtype:
    """由 `msg.fields` 构造与 `point_step` 对齐的结构化 dtype。"""
    names, formats, offsets = [], [], []
    for f in msg.fields:
        np_type = _DATATYPE_TO_NP.get(f.datatype)
        if np_type is None:
            raise ValueError('不支持的 PointField.datatype=%d (%s)' % (f.datatype, f.name))
        base = np.dtype(np_type).newbyteorder('>' if msg.is_bigendian else '<')
        names.append(f.name)
        formats.append(base if f.count == 1 else np.dtype((base, f.count)))
        offsets.append(f.offset)
    return np.dtype({
        'names': names,
        'formats': formats,
        'offsets': offsets,
        'itemsize': msg.point_step,
    })


def cloud_to_structured(msg: PointCloud2) -> np.ndarray:
    """返回 shape=(N,) 的结构化数组，字段名与 `msg.fields` 一致。

    共享 `msg.data` 的内存，只读；msg 被回收后不要再用。
    """
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    return buf.view(dtype_from_fields(msg)).reshape(-1)


def cloud_to_xyzi(msg: PointCloud2) -> np.ndarray:
    """返回 float32 的 `(N, 4)` 数组：x, y, z, intensity。

    没有 intensity 字段时该列填 0。
    """
    rec = cloud_to_structured(msg)
    out = np.empty((rec.shape[0], 4), dtype=np.float32)
    out[:, 0] = rec['x']
    out[:, 1] = rec['y']
    out[:, 2] = rec['z']
    out[:, 3] = rec['intensity'] if 'intensity' in rec.dtype.names else 0.0
    return out


def filter_range(xyzi: np.ndarray, min_range: float,
                 max_range: float) -> Tuple[np.ndarray, np.ndarray]:
    """按到雷达原点的欧氏距离过滤，同时剔除非有限值。

    返回 `(过滤后的点, 距离)`。
    """
    finite = np.isfinite(xyzi[:, :3]).all(axis=1)
    dist = np.full(xyzi.shape[0], np.inf, dtype=np.float32)
    dist[finite] = np.linalg.norm(xyzi[finite, :3], axis=1)
    keep = finite & (dist >= min_range) & (dist <= max_range)
    return xyzi[keep], dist[keep]


_XYZI_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]


def make_xyzi_cloud(header, xyzi: np.ndarray) -> PointCloud2:
    """把 `(N, 4)` float32 数组打包成 16 字节步长的稠密 PointCloud2。"""
    pts = np.ascontiguousarray(xyzi, dtype=np.float32)
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = pts.shape[0]
    msg.fields = _XYZI_FIELDS
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * pts.shape[0]
    msg.is_dense = True
    msg.data = pts.tobytes()
    return msg
