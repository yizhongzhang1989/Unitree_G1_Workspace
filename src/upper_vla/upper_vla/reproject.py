"""把本机相机的图重采样到 VLA 训练相机的内参上，不依赖 ROS。

模型对图像几何的分布偏移很敏感，而实测它不读 ``head_camera_in_world``（把外参整个换成
单位阵，输出只变 0.030 m，与重发同一份输入的 0.019 m 同量级），所以相机位姿的差异只能
从像素上对齐。训练相机 1280x720 下 FOV 89.65x58.47 度，本机 D435i 彩色只有
69.74x43.03 度；换算到模型输入的 240 高之后焦距差 1.42 倍，同一个杯子在我们的图里大
42%。这一层按目标内参逐像素重采样把角度尺度对齐，视场不够的部分留黑边。

**修正只做在输入侧。** 焦距失配是投影平面上的角度误差，不是三维相似变换，单目下它
还随深度变化，标量乘不回来；输入对齐之后输出天然就在训练的系里，两边都补是双重修正。
"""

from __future__ import annotations

import cv2
import numpy as np

ZERO_DISTORTION = (0.0,) * 5


def _matrix(intrinsics) -> np.ndarray:
    fx, fy, cx, cy = (float(v) for v in np.asarray(intrinsics, dtype=np.float64).reshape(4))
    if not (fx > 0.0 and fy > 0.0):
        raise ValueError(f'焦距必须为正，收到 fx={fx} fy={fy}')
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])


def scale_intrinsics(intrinsics, size, height: int):
    """按目标高度等比缩放，复现训练侧「resize 到高 240」那一步。"""
    fx, fy, cx, cy = (float(v) for v in np.asarray(intrinsics, dtype=np.float64).reshape(4))
    width, source_height = (int(v) for v in np.asarray(size).reshape(2))
    scale = height / float(source_height)
    return ((fx * scale, fy * scale, cx * scale, cy * scale),
            (int(round(width * scale)), int(height)))


def field_of_view(intrinsics, size) -> tuple[float, float]:
    """水平/垂直 FOV，度。用来在日志里一眼看出两台相机差多少。"""
    fx, fy = (float(v) for v in np.asarray(intrinsics, dtype=np.float64).reshape(4)[:2])
    width, height = (float(v) for v in np.asarray(size).reshape(2))
    return (float(np.degrees(2.0 * np.arctan(width / (2.0 * fx)))),
            float(np.degrees(2.0 * np.arctan(height / (2.0 * fy)))))


class Reprojector:
    """映射表在构造时算一次，之后每帧只是一次 ``cv2.remap``。

    畸变按 OpenCV/ROS 的 plumb_bob 顺序 ``[k1, k2, p1, p2, k3]``——注意厂商 JSON 常写成
    ``k1 k2 k3 p1 p2``，顺序抄错不会报错，只会悄悄画歪。
    """

    def __init__(self, source_intrinsics, source_size, target_intrinsics, target_size,
                 source_distortion=ZERO_DISTORTION, target_distortion=ZERO_DISTORTION) -> None:
        width, height = (int(v) for v in np.asarray(target_size).reshape(2))
        source_width, source_height = (int(v) for v in np.asarray(source_size).reshape(2))
        k_source = _matrix(source_intrinsics)
        k_target = _matrix(target_intrinsics)
        d_source = np.asarray(source_distortion, dtype=np.float64).reshape(1, -1)
        d_target = np.asarray(target_distortion, dtype=np.float64).reshape(1, -1)

        grid = np.stack(np.meshgrid(np.arange(width, dtype=np.float64),
                                    np.arange(height, dtype=np.float64)), axis=-1)
        # 目标像素 -> 去畸变成射线 -> 加上本机畸变投回源图；两侧畸变都不能漏。
        rays = cv2.undistortPoints(grid.reshape(-1, 1, 2), k_target, d_target).reshape(-1, 2)
        points = np.concatenate([rays, np.ones((rays.shape[0], 1))], axis=1)
        source, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), k_source, d_source)
        source = source.reshape(height, width, 2)

        inside = ((source[..., 0] >= 0.0) & (source[..., 0] <= source_width - 1.0)
                  & (source[..., 1] >= 0.0) & (source[..., 1] <= source_height - 1.0))
        self.coverage = float(inside.mean())
        self.size = (width, height)
        self.source_size = (source_width, source_height)
        self._map_x = source[..., 0].astype(np.float32)
        self._map_y = source[..., 1].astype(np.float32)

    def __call__(self, image: np.ndarray) -> np.ndarray:
        if image.shape[1::-1] != self.source_size:
            raise ValueError(f'源图 {image.shape[1::-1]} 与标定的 {self.source_size} 不符')
        # 视场不够的那圈拿最近的边缘像素拉出去，不留黑边。外推出来的部分是造的，
        # 占 1-coverage，看 coverage 字段就知道多少是真的。
        return cv2.remap(image, self._map_x, self._map_y, cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_REPLICATE)
