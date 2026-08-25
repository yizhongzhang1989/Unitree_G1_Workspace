import os

import numpy as np
import pytest

from camera_calibration import storage


@pytest.fixture
def store(tmp_path):
    return storage.Store(tmp_path / 'data', tmp_path / 'calibration.yaml')


def test_new_shot_does_not_reuse_a_deleted_number(store, board, flat):
    """删掉中间一张后再拍，编号必须往后走，不能盖掉已有的图"""
    detection = board.detect(flat)
    image = np.zeros((detection.size[1], detection.size[0], 3), np.uint8)
    names = [store.save_intrinsic_shot('cam', image, detection) for _ in range(3)]
    assert names == ['0001', '0002', '0003']

    store.delete_intrinsic_shot('cam', *detection.size, '0002')
    assert store.save_intrinsic_shot('cam', image, detection) == '0004'
    assert [s['name'] for s in store.list_intrinsic_shots('cam', *detection.size)] == \
        ['0001', '0003', '0004']


def test_new_pose_does_not_reuse_a_deleted_number(store):
    images = {'head': np.zeros((4, 4, 3), np.uint8)}
    for _ in range(3):
        store.save_extrinsic_pose(images, {})
    store.delete_extrinsic_pose('pose_002')
    assert store.save_extrinsic_pose(images, {}) == 'pose_004'


def test_image_is_saved_lossless(store, board, flat):
    """标定的精度就在角点那零点几个像素上，JPEG 的块效应会把它推偏"""
    detection = board.detect(flat)
    rng = np.random.default_rng(0)
    image = rng.integers(0, 255, (detection.size[1], detection.size[0], 3),
                         dtype=np.uint8)
    name = store.save_intrinsic_shot('cam', image, detection)
    loaded = store.read_intrinsic_image('cam', *detection.size, name)
    assert np.array_equal(loaded, image)
    assert os.path.splitext(str(
        store.intrinsic_dir('cam', *detection.size) / f'{name}.png'))[1] == '.png'
