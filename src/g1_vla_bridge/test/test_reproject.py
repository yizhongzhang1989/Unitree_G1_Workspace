"""重投影层：几何正确性与覆盖率。"""

import numpy as np
import pytest

from g1_vla_bridge.reproject import (
    Reprojector,
    field_of_view,
    scale_intrinsics,
)

# VLA 训练相机（1280x720）与本机 D435i 彩色（424x240）。
TRAIN_K = (643.9313354492188, 643.236083984375, 646.4886474609375, 359.7933654785156)
TRAIN_SIZE = (1280, 720)
TRAIN_D = (-0.05372680723667145, 0.061037734150886536,
           0.0002683195343706757, 0.0005757578765042126, -0.018827488645911217)
HEAD_K = (304.226, 304.385, 215.043, 123.774)
HEAD_SIZE = (424, 240)


def test_scale_intrinsics_matches_training_resize():
    k, size = scale_intrinsics(TRAIN_K, TRAIN_SIZE, 240)
    assert size == (427, 240)
    assert np.allclose(k, (214.6438, 214.4120, 215.4962, 119.9311), atol=1e-3)


def test_field_of_view():
    assert np.allclose(field_of_view(TRAIN_K, TRAIN_SIZE), (89.65, 58.47), atol=0.02)
    assert np.allclose(field_of_view(HEAD_K, HEAD_SIZE), (69.74, 43.03), atol=0.02)


def test_identity_when_same_camera():
    """源和目标是同一台相机时应当是恒等映射。"""
    r = Reprojector(HEAD_K, HEAD_SIZE, HEAD_K, HEAD_SIZE)
    # 不是 1.0：undistortPoints 的迭代解会让最后一列落在 423.0000x，刚好出界。
    assert r.coverage > 0.997
    image = np.random.default_rng(0).integers(0, 256, (240, 424, 3), dtype=np.uint8)
    assert np.array_equal(r(image), image)


def test_head_to_training_coverage():
    """我们视场更窄，只能填满训练画布的一半左右，其余是黑边。"""
    k, size = scale_intrinsics(TRAIN_K, TRAIN_SIZE, 240)
    r = Reprojector(HEAD_K, HEAD_SIZE, k, size, target_distortion=TRAIN_D)
    assert r.size == (427, 240)
    assert 0.45 < r.coverage < 0.55


def test_pinhole_relation_is_exact():
    """无畸变时映射必须逐像素符合小孔公式，否则整幅图会平移或缩放错。"""
    k, size = scale_intrinsics(TRAIN_K, TRAIN_SIZE, 240)
    r = Reprojector(HEAD_K, HEAD_SIZE, k, size)
    for u, v in ((215, 120), (0, 0), (426, 239), (100, 200)):
        expect_x = HEAD_K[2] + (u - k[2]) / k[0] * HEAD_K[0]
        expect_y = HEAD_K[3] + (v - k[3]) / k[1] * HEAD_K[1]
        assert np.isclose(r._map_x[v, u], expect_x, atol=1e-3)
        assert np.isclose(r._map_y[v, u], expect_y, atol=1e-3)


def test_border_extrapolates_instead_of_going_black():
    """视场不够的那圈拿边缘像素拉出去。黑边是纯 OOD 输入，涂抹至少还是图像统计量。"""
    k, size = scale_intrinsics(TRAIN_K, TRAIN_SIZE, 240)
    r = Reprojector(HEAD_K, HEAD_SIZE, k, size, target_distortion=TRAIN_D)
    source = np.full((240, 424, 3), 255, np.uint8)
    source[0, :] = source[-1, :] = source[:, 0] = source[:, -1] = (10, 20, 30)
    out = r(source)
    assert out.shape == (240, 427, 3)
    # 四角落在源图之外，应等于最近的源图边缘像素，而不是 0。
    assert tuple(out[0, 0]) == (10, 20, 30)
    assert tuple(out[-1, -1]) == (10, 20, 30)
    # coverage 仍然只数「真数据」，不因为填满了就变成 1。
    assert 0.3 < r.coverage < 0.7


def test_scale_is_actually_applied():
    """核心断言：重投影后同一物体的像素尺寸应缩到约 1/1.417。"""
    k, size = scale_intrinsics(TRAIN_K, TRAIN_SIZE, 240)
    r = Reprojector(HEAD_K, HEAD_SIZE, k, size)
    image = np.zeros((240, 424, 3), np.uint8)
    # 以源主点为中心画一个方块，比较重投影前后的宽度。
    cx, cy = int(HEAD_K[2]), int(HEAD_K[3])
    image[cy - 30:cy + 30, cx - 30:cx + 30] = 255
    out = r(image)
    width_in = 60
    width_out = int((out[int(k[3]), :, 0] > 127).sum())
    assert np.isclose(width_out / width_in, 214.6438 / 304.226, rtol=0.05)


def test_rejects_wrong_source_size():
    r = Reprojector(HEAD_K, HEAD_SIZE, HEAD_K, HEAD_SIZE)
    with pytest.raises(ValueError):
        r(np.zeros((480, 848, 3), np.uint8))
