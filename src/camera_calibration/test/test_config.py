"""config/*.yaml 的自洽性。配置写错了跑起来才发现太晚，而且错得不明显。"""

from pathlib import Path

import pytest
import yaml

from camera_calibration.board import Board

CONFIG = Path(__file__).resolve().parents[1] / 'config'


@pytest.fixture(scope='module')
def cameras():
    return yaml.safe_load((CONFIG / 'cameras.yaml').read_text(encoding='utf-8'))


@pytest.fixture(scope='module')
def board_config():
    return yaml.safe_load((CONFIG / 'board.yaml').read_text(encoding='utf-8'))


def test_board_config_builds(board_config):
    board = Board.from_config(board_config)
    assert board.corner_count == 88
    # 12*9 = 108 格要 54 个 marker，字典装不下会在这里就炸
    assert board.squares_x * board.squares_y // 2 == 54


def test_exactly_one_reference_camera(cameras):
    roles = [c.get('role') for c in cameras['cameras'].values()]
    assert roles.count('reference') == 1
    assert roles.count('target') == len(roles) - 1


def test_targets_have_a_parent_link(cameras):
    for name, config in cameras['cameras'].items():
        if config.get('role') == 'target':
            assert config.get('parent_frame'), f'{name} 缺 parent_frame，外参没法落地'
            assert config.get('frame'), f'{name} 缺 frame'


def test_every_camera_declares_where_its_frame_comes_from(cameras):
    """三个相机的 TF 都得由 URDF 出，否则又要靠 calib_tf_node 单独跑一份"""
    for name, config in cameras['cameras'].items():
        assert config.get('mount_joint'), f'{name} 缺 mount_joint'
        assert config.get('mount_parent'), f'{name} 缺 mount_parent'


def test_wrist_cameras_create_their_joint_and_the_head_does_not(cameras):
    """URDF 里只有支架的可视化 link，没有腕相机光心，所以那条边得新插"""
    for name, config in cameras['cameras'].items():
        created = bool(config.get('mount_create'))
        assert created == (config.get('role') == 'target'), \
            f'{name} 的 mount_create 和它的角色对不上'


def test_reference_declares_the_urdf_joint_to_correct(cameras):
    """头部外参不能发 static TF（那个 frame 已有人在发），只能落到 URDF 关节上"""
    reference = [c for c in cameras['cameras'].values()
                 if c.get('role') == 'reference'][0]
    assert reference.get('mount_frame'), '缺 mount_frame'
    assert reference.get('mount_joint'), '缺 mount_joint'
    assert reference['mount_frame'] != reference['frame'], \
        'mount_frame 应该是 URDF 里的安装 link，不是光学系'


def test_profiles_are_unique(cameras):
    for name, config in cameras['cameras'].items():
        keys = [(p['width'], p['height']) for p in config['profiles']]
        assert len(keys) == len(set(keys)), f'{name} 有重复档位'


def test_camera_node_profiles_all_carry_a_url(cameras):
    for name, config in cameras['cameras'].items():
        if config.get('switch', {}).get('kind') != 'camera_node':
            continue
        for profile in config['profiles']:
            assert profile.get('url'), \
                f"{name} 的 {profile['width']}x{profile['height']} 没写 url，切过去会拉不到流"
        # 两台腕相机硬件就只有 stream0/stream1，每路一个档位，不标 ffmpeg 缩出来的
        urls = [p['url'] for p in config['profiles']]
        assert len(urls) == len(set(urls)), f'{name} 有两个档位指向同一路流'


def test_realsense_profile_values_match_the_declared_size(cameras):
    for name, config in cameras['cameras'].items():
        if config.get('switch', {}).get('kind') != 'realsense':
            continue
        for profile in config['profiles']:
            # value 是 realsense 的 "宽x高x帧率"，和 width/height 对不上就会切错档
            width, height, _ = profile['value'].split('x')
            assert (int(width), int(height)) == (profile['width'], profile['height']), \
                f"{name} 的 {profile['value']} 和 {profile['width']}x{profile['height']} 不符"
