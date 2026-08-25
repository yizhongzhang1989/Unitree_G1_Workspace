"""PointCloud2 与 numpy 之间的零拷贝转换。

Livox MID-360 经宇树 `lidar_driver` 发出的 `/utlidar/cloud_livox_mid360` 布局
（2026-08-13 实机读到）：point_step=22，字段
`x,y,z,intensity`(float32) + `ring`(uint16, offset 16) + `time`(float32, offset 18)。
这里不写死该布局，而是按 `msg.fields` 现场构造 numpy 结构化 dtype，
换固件或换雷达都不用改代码。
"""

from typing import Tuple

import array

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


def xyzi_of(rec: np.ndarray) -> np.ndarray:
    """结构化数组 -> float32 的 `(N, 4)`：x, y, z, intensity。

    没有 intensity 字段时该列填 0。
    """
    out = np.empty((rec.shape[0], 4), dtype=np.float32)
    out[:, 0] = rec['x']
    out[:, 1] = rec['y']
    out[:, 2] = rec['z']
    out[:, 3] = rec['intensity'] if 'intensity' in rec.dtype.names else 0.0
    return out


def cloud_to_xyzi(msg: PointCloud2) -> np.ndarray:
    """返回 float32 的 `(N, 4)` 数组：x, y, z, intensity。

    没有 intensity 字段时该列填 0。
    """
    return xyzi_of(cloud_to_structured(msg))


def _range_keep(xyz: np.ndarray, min_range: float,
                max_range: float) -> Tuple[np.ndarray, np.ndarray]:
    finite = np.isfinite(xyz).all(axis=1)
    dist = np.full(xyz.shape[0], np.inf, dtype=np.float32)
    dist[finite] = np.linalg.norm(xyz[finite], axis=1)
    return finite & (dist >= min_range) & (dist <= max_range), dist


def filter_range(xyzi: np.ndarray, min_range: float,
                 max_range: float) -> Tuple[np.ndarray, np.ndarray]:
    """按到雷达原点的欧氏距离过滤，同时剔除非有限值。

    返回 `(过滤后的点, 距离)`。
    """
    keep, dist = _range_keep(xyzi[:, :3], min_range, max_range)
    return xyzi[keep], dist[keep]


def range_mask(rec: np.ndarray, min_range: float, max_range: float) -> np.ndarray:
    """同 `filter_range` 的判据，但直接吃结构化数组、只返回布尔掩码。"""
    xyz = np.stack([rec['x'], rec['y'], rec['z']], axis=1).astype(np.float32, copy=False)
    return _range_keep(xyz, min_range, max_range)[0]


_XYZI_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]


def _as_payload(buf: bytes) -> array.array:
    """`uint8[]` 字段只能这么赋值。

    rclpy 对 `uint8[]` **只对 `array.array('B', ...)` 短路**；赋 `bytes` 会走
    `__debug__` 分支里的 `all(isinstance(v, int) for v in value)` 逐元素断言。
    实测一帧 9500 点（209 KB）：**29.9 ms vs 0.044 ms，678 倍**。10 Hz 下那就是
    30% 单核，还会把同节点里 200 Hz 的 IMU 回调饿死。
    """
    return array.array('B', buf)


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
    msg.data = _as_payload(pts.tobytes())
    return msg


def repack_like(src: PointCloud2, rec: np.ndarray) -> PointCloud2:
    """按 `src` 的字段布局原样打包结构化数组，保住 `ring` / `time` 等额外字段。

    激光惯性里程计（Point-LIO 等）靠逐点 `time` 做运动去畸变，`make_xyzi_cloud`
    的 16 字节瘦身布局会把它丢掉，所以喂给里程计的那一路必须走这里。
    """
    msg = PointCloud2()
    msg.header = src.header
    msg.height = 1
    msg.width = rec.shape[0]
    msg.fields = src.fields
    msg.is_bigendian = src.is_bigendian
    msg.point_step = src.point_step
    msg.row_step = src.point_step * rec.shape[0]
    msg.is_dense = True
    msg.data = _as_payload(np.ascontiguousarray(rec).tobytes())
    return msg
