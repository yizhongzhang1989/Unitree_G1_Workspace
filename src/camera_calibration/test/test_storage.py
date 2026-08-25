import numpy as np
import pytest

from camera_calibration import storage, transforms

ENTRY = {
    'width': 1920, 'height': 1080,
    'camera_matrix': [1500.0, 0.0, 960.0, 0.0, 1502.0, 540.0, 0.0, 0.0, 1.0],
    'distortion_model': 'plumb_bob',
    'distortion_coefficients': [-0.31, 0.09, 0.001, -0.002, 0.0],
    'rms': 0.31, 'images': 22, 'coverage': 0.72, 'fov_deg': [65.5, 39.2],
}


@pytest.fixture
def store(tmp_path):
    return storage.Store(tmp_path / 'data', tmp_path / 'cfg' / 'calibration.yaml')


def test_empty_calibration_has_the_expected_shape(store):
    data = store.read_calibration()
    assert data == {'version': 1, 'intrinsics': {}, 'extrinsics': {},
                    'profile_relations': {}, 'urdf_overrides': {}}


def test_intrinsic_roundtrip(store):
    store.put_intrinsic('camera_left', ENTRY)
    found = storage.find_intrinsic(store.read_calibration(), 'camera_left', 1920, 1080)
    assert found['camera_matrix'] == ENTRY['camera_matrix']
    assert 'stamp' in found


def test_put_intrinsic_replaces_same_profile(store):
    store.put_intrinsic('camera_left', ENTRY)
    store.put_intrinsic('camera_left', dict(ENTRY, rms=0.19))
    entries = store.read_calibration()['intrinsics']['camera_left']
    assert len(entries) == 1
    assert entries[0]['rms'] == 0.19


def test_missing_profile_returns_none(store):
    store.put_intrinsic('camera_left', ENTRY)
    assert storage.find_intrinsic(
        store.read_calibration(), 'camera_left', 640, 360) is None


def test_scale_relation_enables_conversion(store):
    store.put_intrinsic('camera_left', ENTRY)
    store.put_profile_relation('camera_left', {
        'from': [1920, 1080], 'to': [640, 360], 'kind': 'scale', 'ratio': 1 / 3})

    found = storage.find_intrinsic(store.read_calibration(), 'camera_left', 640, 360)
    matrix = np.asarray(found['camera_matrix']).reshape(3, 3)
    assert matrix[0, 0] == pytest.approx(500.0)
    assert matrix[0, 2] == pytest.approx(320.0)
    assert matrix[2, 2] == pytest.approx(1.0)
    # 畸变系数在归一化坐标里定义，缩放不改它们
    assert found['distortion_coefficients'] == ENTRY['distortion_coefficients']
    assert found['scaled_from'] == [1920, 1080]


def test_crop_relation_refuses_to_convert(store):
    store.put_intrinsic('camera_left', ENTRY)
    store.put_profile_relation('camera_left', {
        'from': [1920, 1080], 'to': [1440, 1080], 'kind': 'crop'})
    assert storage.find_intrinsic(
        store.read_calibration(), 'camera_left', 1440, 1080) is None


def test_allow_scale_false_skips_relations(store):
    store.put_intrinsic('camera_left', ENTRY)
    store.put_profile_relation('camera_left', {
        'from': [1920, 1080], 'to': [640, 360], 'kind': 'scale'})
    assert storage.find_intrinsic(store.read_calibration(), 'camera_left',
                                  640, 360, allow_scale=False) is None


def test_anisotropic_scale_uses_separate_ratios(store):
    """ffmpeg 缩放常常不是严格等比：640x360 -> 426x240 时 426/640 和 240/360 不相等，
    fit_size 把宽取整到了偶数。两个方向必须各用各的比例。"""
    entry = dict(ENTRY, width=640, height=360,
                 camera_matrix=[500.0, 0.0, 320.0, 0.0, 502.0, 180.0, 0.0, 0.0, 1.0])
    store.put_intrinsic('camera_left', entry)
    store.put_profile_relation('camera_left', {
        'from': [640, 360], 'to': [426, 240], 'kind': 'scale', 'source': 'ffmpeg'})

    found = storage.find_intrinsic(store.read_calibration(), 'camera_left', 426, 240)
    matrix = np.asarray(found['camera_matrix']).reshape(3, 3)
    assert matrix[0, 0] == pytest.approx(500.0 * 426 / 640)
    assert matrix[1, 1] == pytest.approx(502.0 * 240 / 360)
    assert matrix[0, 2] == pytest.approx(320.0 * 426 / 640)
    assert matrix[1, 2] == pytest.approx(180.0 * 240 / 360)
    assert matrix[0, 0] != pytest.approx(matrix[1, 1] * 500.0 / 502.0)


def test_extrinsic_roundtrip(store):
    matrix = transforms.rt_to_matrix(np.eye(3), [0.03, -0.01, 0.05])
    store.put_extrinsic('camera_left', {
        'parent': 'left_gripper_base', 'child': 'camera_left',
        **transforms.matrix_to_dict(matrix)})
    entry = store.read_calibration()['extrinsics']['camera_left']
    assert entry['parent'] == 'left_gripper_base'
    assert np.allclose(transforms.matrix_from_dict(entry), matrix)


def test_urdf_override_roundtrip(store):
    store.put_urdf_override('d435_joint', {
        'parent': 'torso_link', 'child': 'd435_link',
        'xyz': [0.0576235, 0.01753, 0.42987], 'rpy': [0.0, 0.8307767, 0.0]})
    entry = store.read_calibration()['urdf_overrides']['d435_joint']
    assert entry['parent'] == 'torso_link'
    assert entry['rpy'][1] == pytest.approx(0.8307767)
    assert entry['stamp']


def test_urdf_override_stays_out_of_extrinsics(store):
    """混进 extrinsics 就会被 calib_tf_node 发成 static TF，和 URDF 抢同一个 child"""
    store.put_urdf_override('d435_joint', {
        'parent': 'torso_link', 'child': 'd435_link',
        'xyz': [0.0, 0.0, 0.0], 'rpy': [0.0, 0.0, 0.0]})
    assert store.read_calibration()['extrinsics'] == {}


def test_intrinsic_shots_roundtrip(store, board, flat):
    detection = board.detect(flat)
    image = np.zeros((detection.size[1], detection.size[0], 3), np.uint8)
    name = store.save_intrinsic_shot('camera_left', image, detection)

    shots = store.list_intrinsic_shots('camera_left', *detection.size)
    assert [s['name'] for s in shots] == [name]
    assert shots[0]['corners'] == board.corner_count

    views = store.load_intrinsic_views('camera_left', *detection.size)
    assert np.allclose(views[0]['detection'].corners, detection.corners)

    store.delete_intrinsic_shot('camera_left', *detection.size, name)
    assert store.list_intrinsic_shots('camera_left', *detection.size) == []


def test_extrinsic_pose_roundtrip(store):
    images = {'head': np.zeros((8, 8, 3), np.uint8),
              'camera_left': np.zeros((8, 8, 3), np.uint8)}
    name = store.save_extrinsic_pose(images, {'T_base_link': np.eye(4)})

    poses = store.list_extrinsic_poses()
    assert len(poses) == 1 and poses[0]['name'] == name
    assert np.allclose(poses[0]['T_base_link'], np.eye(4))
    assert store.read_extrinsic_image(name, 'head') is not None

    store.delete_extrinsic_pose(name)
    assert store.list_extrinsic_poses() == []


@pytest.mark.parametrize('name', ['..', '.', '', '../../etc', 'a/b', '/abs'])
def test_path_traversal_is_rejected(store, name):
    with pytest.raises(ValueError):
        store.delete_extrinsic_pose(name)


def test_calib_path_follows_symlinks(tmp_path):
    """symlink-install 下 share/config 是指向 src 的链接，必须写到链接指向的真实位置"""
    real = tmp_path / 'src' / 'config'
    real.mkdir(parents=True)
    link = tmp_path / 'install' / 'config'
    link.parent.mkdir(parents=True)
    link.symlink_to(real)

    store = storage.Store(tmp_path / 'data', link / 'calibration.yaml')
    store.put_intrinsic('camera_left', ENTRY)
    assert (real / 'calibration.yaml').is_file()
    assert not (real / 'calibration.yaml.tmp').exists()
