"""重力对齐点云中的局部地面估计。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class GroundPlane:
    normal: np.ndarray
    offset: float
    rmse: float

    def height_at(self, x: float, y: float) -> float:
        return float(-(self.normal[0] * x + self.normal[1] * y + self.offset)
                     / self.normal[2])


def fit_ground_plane(points: np.ndarray, *, iterations: int = 100,
                     distance_threshold: float = 0.03,
                     max_tilt_rad: float = np.deg2rad(12.0),
                     min_inliers: int = 80,
                     min_inlier_ratio: float = 0.25,
                     seed: int = 0) -> GroundPlane | None:
    """RANSAC 拟合近水平面，并用内点 SVD 精修。"""
    xyz = np.asarray(points, dtype=np.float64)
    xyz = xyz[np.all(np.isfinite(xyz), axis=1)]
    if len(xyz) < max(min_inliers, 3):
        return None
    rng = np.random.default_rng(seed)
    min_normal_z = float(np.cos(max_tilt_rad))
    best = None
    for _ in range(int(iterations)):
        sample = xyz[rng.choice(len(xyz), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = float(np.linalg.norm(normal))
        if norm < 1e-9:
            continue
        normal /= norm
        if normal[2] < 0.0:
            normal = -normal
        if normal[2] < min_normal_z:
            continue
        offset = -float(normal @ sample[0])
        inliers = np.abs(xyz @ normal + offset) <= distance_threshold
        count = int(np.count_nonzero(inliers))
        if best is None or count > best[0]:
            best = (count, inliers)
    if best is None or best[0] < min_inliers or best[0] / len(xyz) < min_inlier_ratio:
        return None

    selected = xyz[best[1]]
    center = np.mean(selected, axis=0)
    _, _, vh = np.linalg.svd(selected - center, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0.0:
        normal = -normal
    if normal[2] < min_normal_z:
        return None
    offset = -float(normal @ center)
    residual = selected @ normal + offset
    return GroundPlane(
        normal=normal,
        offset=offset,
        rmse=float(np.sqrt(np.mean(residual * residual))),
    )