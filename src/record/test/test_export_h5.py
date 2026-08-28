"""`tools/format/YB/export.py` 与 `tools/urdf_fk.py`。

两个都要能在**没有 ROS 的导出机**上跑，所以这里连 rclpy 都不 import；
`test_no_ros_dependency` 用正则把这条钉住，和 `test_align_video.py` 一个路子。
"""

from __future__ import annotations

import re
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(TOOLS / 'format' / 'YB'))

import export as ex                                       # noqa: E402
import resample_video                                      # noqa: E402
import urdf_fk                                             # noqa: E402

URDF = """<robot name="t">
  <joint name="fixed_a" type="fixed">
    <parent link="root"/><child link="a"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
  </joint>
  <joint name="hinge" type="revolute">
    <parent link="a"/><child link="b"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="slide" type="prismatic">
    <parent link="b"/><child link="c"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
  </joint>
  <joint name="follower" type="revolute">
    <parent link="b"/><child link="d"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <mimic joint="hinge" multiplier="2" offset="0.1"/>
  </joint>
</robot>"""


@pytest.fixture
def model():
    return urdf_fk.RobotModel.from_urdf(URDF)


# ------------------------------------------------------------------------- FK


def test_no_ros_dependency():
    """递归扫整个 tools/ 而不是列文件名 —— 新加的格式不用记得回来改这里。"""
    banned = re.compile(r'^\s*(?:import|from)\s+(rclpy|sensor_msgs|pinocchio)\b', re.M)
    files = [p for p in TOOLS.rglob('*.py') if '__pycache__' not in p.parts]
    assert files, '一个文件都没扫到，路径错了'
    for path in files:
        assert not banned.search(path.read_text(encoding='utf-8')), path


def test_chain_and_moving_joints(model):
    assert [j.name for j in model.chain('root', 'c')] == ['fixed_a', 'hinge', 'slide']
    # mimic 的关节不该出现在「要给角度」的清单里，它跟着源关节走
    assert model.moving_joints('root', 'd') == ['hinge']


def test_chain_rejects_non_ancestor(model):
    with pytest.raises(ValueError):
        model.chain('c', 'a')


def test_missing_joint_value_raises(model):
    """补 0 会给出一个看着完全正常、但整条链都错的位姿，必须炸出来。"""
    with pytest.raises(KeyError):
        model.poses('root', 'b', {})


def test_revolute_and_prismatic(model):
    pose = model.poses('root', 'c', {'hinge': np.pi / 2, 'slide': 0.25})[0]
    # a 抬高 1，hinge 前移 0.5 后转 90°，slide 沿转过的 x 轴再走 0.25
    assert np.allclose(pose[:3, 3], [0.5, 0.25, 1.0])


def test_mimic_follows_source(model):
    angle = 0.3
    direct = model.poses('root', 'd', {'hinge': angle})[0]
    # d 在 hinge 之下，所以总转角是 hinge 自己的 angle 再叠 mimic 的 2*angle+0.1
    expected = urdf_fk.rotate([0, 0, 1], angle + 2 * angle + 0.1)[0]
    assert np.allclose(direct[:3, :3], expected[:3, :3])


def test_poses_broadcast_over_time(model):
    angles = np.linspace(0, 1, 7)
    batch = model.poses('root', 'b', {'hinge': angles})
    assert batch.shape == (7, 4, 4)
    for i, a in enumerate(angles):
        assert np.allclose(batch[i], model.poses('root', 'b', {'hinge': a})[0])


def test_quaternion_survives_180_degrees():
    """单主元公式在 180° 附近要除以一个接近零的数，直接崩。"""
    for axis in ([1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0]):
        matrix = urdf_fk.rotate(axis, np.pi)
        quat = urdf_fk.matrix_to_quat(matrix)[0]
        assert np.isfinite(quat).all()
        assert abs(np.linalg.norm(quat) - 1) < 1e-12
        assert quat[3] >= 0


def test_invert_round_trips(model):
    pose = model.poses('root', 'c', {'hinge': 0.4, 'slide': -0.2})
    assert np.allclose(urdf_fk.invert(pose) @ pose, np.eye(4))


def test_overrides_create_and_replace(model):
    applied = model.apply_overrides({
        'hinge': {'parent': 'a', 'child': 'b', 'xyz': [1, 0, 0], 'rpy': [0, 0, 0]},
        'new_optical': {'parent': 'b', 'child': 'lens', 'create': True,
                        'xyz': [0, 0, 0.2], 'rpy': [0, 0, 0]},
    })
    assert set(applied) == {'hinge', 'new_optical'}
    assert model.joints['hinge'].kind == 'revolute'         # 只换 origin，不改类型
    pose = model.poses('root', 'lens', {'hinge': 0.0})[0]
    assert np.allclose(pose[:3, 3], [1.0, 0.0, 1.2])


def test_overrides_skip_when_mount_moved(model):
    """存的是 T_parent<-child。挂点被改过就不是这条边了，宁可不叠。"""
    before = model.joints['hinge'].origin.copy()
    assert model.apply_overrides(
        {'hinge': {'parent': 'somewhere_else', 'child': 'b',
                   'xyz': [9, 9, 9], 'rpy': [0, 0, 0]}}) == []
    assert np.allclose(model.joints['hinge'].origin, before)


# ------------------------------------------------------------------- 时间栅格


def test_grid_covers_episode_without_overrun():
    grid = ex.build_grid(10.0, 11.0, 30.0)
    assert grid[0] == 10.0 and grid[-1] <= 11.0 and grid.size == 31
    assert np.allclose(np.diff(grid), 1 / 30.0)


def test_hold_is_zero_order_not_interpolated():
    t = np.array([0.0, 1.0])
    data = np.array([[0.0], [10.0]])
    values, valid = ex.hold(t, data, np.array([0.5]), max_age=2.0)
    assert valid[0] and values[0, 0] == 0.0          # 插值会给 5.0


def test_hold_marks_stale_and_pre_start_invalid():
    t = np.array([1.0, 2.0])
    data = np.array([[1.0], [2.0]])
    grid = np.array([0.5, 2.05, 2.5])
    values, valid = ex.hold(t, data, grid, max_age=0.1)
    assert list(valid) == [False, True, False]
    # 无效处填 NaN 而不是 0 —— 0 是一个合法的关节角，下游看不出是缺的
    assert np.isnan(values[0, 0]) and np.isnan(values[2, 0])


def test_hold_on_empty_table():
    values, valid = ex.hold(np.empty(0), np.empty((0, 3)), np.arange(4.0), 0.1)
    assert values.shape == (4, 3) and np.isnan(values).all() and not valid.any()


def test_arrived_marks_only_the_tick_that_got_a_message():
    grid = np.array([0.0, 1.0, 2.0, 3.0])
    assert list(ex.arrived(np.array([1.5]), grid)) == [False, False, True, False]


def test_frame_index_handles_leading_backward_stamp():
    """实测头部第一帧的 RealSense 戳比第二帧晚一个帧周期。"""
    pts = np.array([0.033, 0.0, 0.066, 0.099])
    index = ex.frame_index(pts, np.array([0.07, 0.10]), max_age=0.1)
    assert list(index) == [2, 3]                     # 仍然是 mkv 里的帧号


def test_frame_index_marks_gap_as_missing():
    pts = np.array([0.0, 1.0])
    assert list(ex.frame_index(pts, np.array([0.5]), max_age=0.1)) == [-1]
    assert list(ex.frame_index(np.empty(0), np.array([0.5]), 0.1)) == [-1]


def test_roles_cover_every_joint_once():
    """真实的 31 轴顺序底下，每个关节都得落进恰好一个 role。"""
    from record import signals
    names = [n for n in signals.CANONICAL_JOINTS if n not in ex.GRIPPER_JOINTS]
    picked = sorted(i for index in ex._roles(names).values() for i in index)
    assert picked == list(range(len(names)))


def test_gripper_is_an_actuator_not_a_joint():
    """规范把夹爪划在 actuator_space，例 2/例 4 都是这么分的。"""
    from record import signals
    assert set(ex.GRIPPER_JOINTS) <= set(signals.CANONICAL_JOINTS)


def test_camera_layout_matches_spec():
    """导出用对面样例的词（headcam/leftcam/rightcam），session 里的流名另存。"""
    assert [c.name for c in ex.CAMERAS] == ['headcam', 'leftcam', 'rightcam']
    assert [c.source for c in ex.CAMERAS] == ['head', 'wrist_left', 'wrist_right']
    # 头部固连 torso_link，腕相机随手臂动 —— static_extrinsic 就是这个意思
    assert [int(c.static) for c in ex.CAMERAS] == [1, 0, 0]


def test_extrinsic_formula_is_stored_not_just_a_label():
    """「world2camera」CV 圈和机器人圈指向相反，已经因此弄反过一回，所以存公式。"""
    assert ex.EXTRINSIC_FORMULA['base_T_cam'] == 'world_xyz = extrinsic @ camera_xyz'
    assert ex.EXTRINSIC_FORMULA['cam_T_base'] == 'camera_xyz = extrinsic @ world_xyz'


# ------------------------------------------------------------------- 末端约定


def _rotation(quat) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def test_quat_multiply_matches_matrix_composition():
    rng = np.random.default_rng(0)
    for _ in range(20):
        a, b = (q / np.linalg.norm(q) for q in rng.normal(size=(2, 4)))
        product = urdf_fk.quat_multiply(a, b)
        assert np.allclose(_rotation(product), _rotation(a) @ _rotation(b))


def test_unify_pose_keeps_translation_and_swaps_xy():
    pose = np.array([[1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]])
    unified = ex.unify_pose(pose)
    assert np.allclose(unified[..., :3], pose[..., :3])
    raw, new = _rotation(pose[0, 3:]), _rotation(unified[0, 3:])
    assert np.allclose(new[:, 2], raw[:, 2])         # 接近轴不动
    assert np.allclose(new[:, 0], raw[:, 1])         # 新 X = 旧 Y
    assert np.allclose(new[:, 1], -raw[:, 0])        # 新 Y = −旧 X


G1_URDF = Path('/workspace/src/unitree_g1_description/model/final.urdf')
G1_CALIBRATION = Path('/workspace/src/camera_calibration/config/calibration.yaml')


@pytest.mark.skipif(not (G1_URDF.is_file() and G1_CALIBRATION.is_file()),
                    reason='工作区里没有 G1 的 URDF 或标定')
def test_head_extrinsic_is_constant():
    """头相机固连 `torso_link`，参考系就是 `torso_link` —— 外参逐帧不变。

    `static_extrinsic` 下游照着决定只读第 0 行还是逐帧读，标错了会拿一帧的外参去投影整段。
    """
    import yaml

    class _Stub:
        def video_path(self, name):
            return Path('/nonexistent') / f'{name}.mkv'

    model = urdf_fk.RobotModel.from_urdf(G1_URDF)
    model.apply_overrides(
        yaml.safe_load(G1_CALIBRATION.read_text(encoding='utf-8'))['urdf_overrides'])
    ex.add_head_optical(model, ex.HEAD_OPTICAL)

    grid = np.arange(4) / 30.0
    joints = {n: np.zeros(grid.size) for camera in ex.CAMERAS
              for n in model.moving_joints(ex.ORIGIN, camera.frame)}
    space = ex.camera_space(model, joints, _Stub(), grid, 0.1, {}, 'base_T_cam')

    assert space['static_extrinsic'] == [1, 0, 0]
    head = space['state']['extrinsic'][:, 0]
    assert np.allclose(head, head[0])


@pytest.mark.skipif(not (G1_URDF.is_file() and G1_CALIBRATION.is_file()),
                    reason='工作区里没有 G1 的 URDF 或标定')
def test_wrist_camera_links_only_exist_after_calibration():
    """`--calibration` 对本机是**必填**，不是“不给就退成名义值”。

    腕相机的两个 link 是标定文件用 `create` 现建的，裸 URDF 里根本没有 ——
    不给就报「torso_link 不是 camera_left 的祖先」，一点看不出要补什么。
    文档一度写着“不给也能跑”，这条把它钉住。
    """
    import yaml
    bare = urdf_fk.RobotModel.from_urdf(G1_URDF)
    wrist = [c.frame for c in ex.CAMERAS if c.calib]
    assert wrist, '没有靠标定建 link 的相机，这条测试该重写了'
    for frame in wrist:
        with pytest.raises(ValueError):
            bare.chain(ex.ORIGIN, frame)

    bare.apply_overrides(
        yaml.safe_load(G1_CALIBRATION.read_text(encoding='utf-8'))['urdf_overrides'])
    for frame in wrist:
        assert bare.chain(ex.ORIGIN, frame)


@pytest.mark.skipif(not G1_URDF.is_file(), reason='工作区里没有 G1 的 URDF')
def test_unified_frame_matches_the_real_gripper():
    """统一末端系的三条约束，全部拿 URDF 实测的几何去核。

    这是把「导出的姿态到底指哪儿」钉死的那一条：URDF 换一版、夹爪换个装法，
    或者腕相机重标到另一侧，这里就会红，而不是安静地导出一批转错 90° 的姿态。
    """
    import yaml
    model = urdf_fk.RobotModel.from_urdf(G1_URDF)
    if G1_CALIBRATION.is_file():
        model.apply_overrides(
            yaml.safe_load(G1_CALIBRATION.read_text(encoding='utf-8'))['urdf_overrides'])
    axes = _rotation(ex.EE_FIX_QUAT_XYZW)            # 统一系三轴在 gripper_base 下
    forward, closing, approach = axes[:, 0], axes[:, 1], axes[:, 2]
    assert np.allclose(np.cross(forward, closing), approach)      # 右手系

    def angle(a, b):
        return np.degrees(np.arccos(np.clip(
            np.dot(a, b) / np.linalg.norm(a) / np.linalg.norm(b), -1, 1)))

    for side in ('left', 'right'):
        base = f'{side}_gripper_base'
        joints = model.moving_joints(base, f'{side}_left_connecting_rod')
        opened = {n: ex.GRIPPER_TRAVEL_RAD for n in joints}
        shut = {n: 0.0 for n in joints}
        tip = model.poses(base, f'{side}_left_connecting_rod', opened)[0, :3, 3]
        other = model.poses(base, f'{side}_right_connecting_rod', opened)[0, :3, 3]
        travel = model.poses(base, f'{side}_left_connecting_rod', shut)[0, :3, 3] - tip

        assert angle(approach, (tip + other) / 2) < 5     # +Z 指向指尖
        assert angle(closing, travel) < 1                 # +Y 是开合轴
        if f'{side}_camera_optical_joint' in model.joints:
            camera = model.joints[f'{side}_camera_optical_joint'].origin[:3, 3]
            # 「朝相机那侧」说的是横向：相机同时偏 +Y 和 +Z，不去掉沿接近轴的
            # 分量就会量出五十多度，看着像错的
            lateral = camera - np.dot(camera, approach) * approach
            assert angle(forward, lateral) < 10


# ------------------------------------------------------------------------- 夹爪


def test_gripper_normalises_to_open_zero_closed_one():
    """对面 meta.json 要 0=open 1=closed；eccentric 是 0 闭合、2.76 张开，正好反着。"""
    values = ex.normalize_gripper(np.array([[0.0, ex.GRIPPER_TRAVEL_RAD]]))
    assert np.allclose(values, [[1.0, 0.0]])


def test_gripper_clips_beyond_travel():
    """实测量到过 2.889，超过名义行程；不裁就会出现负的开合度。"""
    values = ex.normalize_gripper(np.array([[2.889, -0.1]]))
    assert np.allclose(values, [[0.0, 1.0]])


def test_gripper_normalisation_keeps_nan():
    assert np.isnan(ex.normalize_gripper(np.array([np.nan]))).all()


# --------------------------------------------------------------------- 视频重采样


def test_output_size_keeps_width_even():
    """h264 的 yuv420p 色度平面要能整除，宽是奇数直接编不出来。"""
    assert resample_video.output_size((1920, 1080), 361) == (642, 361)
    assert resample_video.output_size((1920, 1080), 0) == (1920, 1080)
    assert resample_video.output_size((640, 360), 720) == (640, 360)   # 不放大


class _FakeStream:
    def __init__(self, frames: list) -> None:
        self._data = b''.join(frames)
        self._at = 0

    def read(self, count: int) -> bytes:
        out = self._data[self._at:self._at + count]
        self._at += len(out)
        return out


def test_reader_walks_forward_and_repeats():
    reader = resample_video._Reader(_FakeStream([b'aa', b'bb', b'cc']), 2)
    assert reader.at(0) == b'aa'
    assert reader.at(0) == b'aa'                 # 停在原地就是重复上一帧
    assert reader.at(2) == b'cc'
    assert reader.at(3) is None                  # 源解完了


def test_reader_refuses_to_go_backwards():
    """解码流退不回去。默默返回当前帧的话，导出的视频会和表错位。"""
    reader = resample_video._Reader(_FakeStream([b'aa', b'bb']), 2)
    reader.at(1)
    with pytest.raises(ValueError):
        reader.at(0)


def test_episode_name_is_shared_by_h5_and_video():
    name = ex.episode_name(3, '20260821_102522', {'round': 1, 'episode': 2})
    assert name == '00000003-20260821_102522__g1__round1_episode2'


def test_episode_id_carries_the_file_serial():
    """id 前半段就是文件名那个序号 —— 对面示例里两者逐字相同，对不上就找不回文件。

    踩过：文件序号从 1 起、id 从 0 起，同一条 episode 一个叫 00000001 一个叫 00000000。
    """
    name = ex.episode_name(1, '20260826_050938', {'round': 0, 'episode': 0})
    assert ex.episode_id(name) == '00000001-0000'
    assert ex.episode_id(name).split('-')[0] == name.split('-')[0]


def test_episode_id_suffix_is_the_segment_not_the_round():
    """后半段是「该文件内第几段」。一个 h5 一条 episode，所以恒 0000，与 round 无关。"""
    for rnd in (0, 3, 12):
        name = ex.episode_name(7, 's', {'round': rnd, 'episode': 1})
        assert ex.episode_id(name) == '00000007-0000'


def test_dataset_meta_has_every_template_field():
    """对面的读取代码按 `datasetmeta_template.json` 写，缺字段就是 KeyError。"""
    meta = ex.dataset_meta([SimpleNamespace(manifest={'session_id': 's'})],
                           [{'duration': 30.0}],
                           SimpleNamespace(hz=30.0, extrinsic='base_T_cam'),
                           {})
    for key in ('dataset_name', 'year', 'environment', 'robot', 'tasks', 'scale',
                'data_links', 'other_links', 'papers', 'license'):
        assert key in meta, f'meta.json 缺模板字段 {key}'
    assert {'eef_type', 'arm_type', 'view_num', 'embodiment_count'} <= set(meta['robot'])
    assert {'episodes', 'hours'} <= set(meta['scale'])


def test_dataset_meta_combines_multiple_sessions():
    sessions = [SimpleNamespace(manifest={'session_id': 's1'}),
                SimpleNamespace(manifest={'session_id': 's2'})]
    meta = ex.dataset_meta(sessions, [{'duration': 30.0}, {'duration': 60.0}],
                           SimpleNamespace(hz=30.0, extrinsic='base_T_cam'), {})
    assert 's1--s2 (2 sessions)' in meta['dataset_name']
    assert meta['scale'] == {'episodes': 2, 'hours': 0.025}


def test_report_takes_space_names_from_the_data():
    """`--dry-run` 的形状表曾经硬编码过一份 space 名单，`base_space` 删掉之后
    没跟着改，于是 `--dry-run` 一直是 KeyError 而没人发现。"""
    spaces = {
        'camera': {'names': ['headcam'],
                   'state': {'frame_index': np.zeros((3, 1), np.int32)}},
        'joint': {'state': {'position': np.zeros((3, 29)),
                            'valid_mask': np.ones(3, bool)}},
    }
    lines = ex._report(spaces, verbose=True)
    assert any('state/joint_space  position(3, 29)' in line for line in lines)
    assert any('state/joint 100%' in line for line in lines)
