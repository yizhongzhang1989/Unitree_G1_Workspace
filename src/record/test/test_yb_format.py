"""`format/YB/README.md` 有没有和代码说的是同一件事。

写这个测试是因为：**文档写错比没写更糟**。规范里那些常量（外参公式、末端约定、
栅格频率、失效阈值）一旦和代码分叉，拿到数据的人会照着文档把数据用错，
而且很久都发现不了。所以这里把文档里出现的每个关键字面量都拿去和 `export.py`
里的定义对一遍。

不校验散文，只校验**数字和标识符** —— 那才是会被人照抄进代码的部分。
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
TOOLS = Path(__file__).resolve().parents[1] / 'tools'
HERE = TOOLS / 'format' / 'YB'
DOC = HERE / 'README.md'
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(HERE))

import export as ex                                        # noqa: E402

TEXT = DOC.read_text(encoding='utf-8')


def test_meta_points_at_a_doc_that_exists():
    """meta.json 里写着去哪儿找文档，指错了等于没写 —— 文件一搬就会错。"""
    assert DOC.is_file()
    assert (REPO / 'src' / ex.FORMAT_DOC).resolve() == DOC.resolve()


def test_format_name_and_version():
    assert ex.FORMAT == 'YB'
    assert f'# {ex.FORMAT} 数据集格式 v{ex.VERSION}' in TEXT


def test_grid_hz_matches():
    assert f'{ex.DEFAULT_HZ:g} Hz' in TEXT
    assert f'1/{ex.DEFAULT_HZ:g}' in TEXT


def test_the_summary_table_states_the_sample_rate():
    """「统一 30 Hz」必须在开头那张表里明说 ——
    拿到数据的人不会把整篇读完，采样率埋在正文里等于没写。"""
    summary = TEXT.split('## 1.')[0]
    assert f'统一 {ex.DEFAULT_HZ:g} Hz' in summary
    for must in ('采样率', '世界坐标系', '四元数顺序', '夹爪', 'xyzw'):
        assert must in summary, f'规格表里没提 {must}'


def test_says_what_the_conversion_does():
    """文档要回答「脚本把原始数据整理成了什么样」，不只是一本字段字典。"""
    assert '## 1. 转换脚本把原始数据变成了什么' in TEXT
    for must in ('零阶保持', '不做插值', 'joint_states'):
        assert must in TEXT, f'没说清楚 {must}'


def test_max_age_matches():
    """『超过 100 ms 判无效』是文档里最容易被抄进预处理代码的一个数。"""
    assert f'{ex.DEFAULT_MAX_AGE_S * 1000:g} ms' in TEXT


def test_world_frame_matches():
    """末端位姿就是 `torso_link` 下的 FK，参考系埋在正文里等于没写。"""
    assert ex.ORIGIN in TEXT
    assert ex.ORIGIN in TEXT.split('## 1.')[0]


def test_extrinsic_formula_is_quoted_verbatim():
    """方向靠公式定义，不靠名字 —— 文档必须把那一行原样抄上。"""
    assert ex.EXTRINSIC_FORMULA['base_T_cam'] in TEXT
    assert 'base_T_cam' in TEXT


def test_camera_axes_and_names():
    for camera in ex.CAMERAS:
        assert camera.name in TEXT
    assert 'OpenCV' in TEXT


def test_ee_convention_and_fix_quat():
    assert ex.EE_CONVENTION in TEXT
    # 四元数按 √2/2 写的，别把 xyzw 顺序也说反了
    assert abs(ex.EE_FIX_QUAT_XYZW[2] - 2 ** -0.5) < 1e-12
    assert ex.EE_FIX_QUAT_XYZW[:2] == (0.0, 0.0) or list(ex.EE_FIX_QUAT_XYZW[:2]) == [0.0, 0.0]
    assert 'xyzw' in TEXT and '√2/2' in TEXT


def test_gripper_travel_matches():
    assert f'{ex.GRIPPER_TRAVEL_RAD:.4f}' in TEXT
    assert '0 = 完全张开' in TEXT and '1 = 完全夹紧' in TEXT


def test_joint_count_matches_urdf_roles():
    """29 这个数写死在文档的表里，DOF 变了要跟着改。"""
    assert '(N, 29)' in TEXT
    for role, _ in ex._ROLE_RULES:
        assert role in TEXT


def test_every_written_space_is_documented():
    for space in ('joint_space', 'end_space', 'actuator_space', 'camera_space'):
        assert space in TEXT


def test_known_limits_section_mentions_the_nan_action():
    """`action/joint_space` 全 NaN 是本批数据最大的坑，必须在文档里显式写着。"""
    assert 'action/joint_space' in TEXT
    assert 'NaN' in TEXT


def test_trim_thresholds_match():
    """裁空转改的是 episode 边界 —— 拿数据的人复算时长对不上会以为丢了数据。

    所以四个阈值和 `source.trim` 的字段名都必须在文档里，且和代码一致。
    """
    for name in ('IDLE_POS_M_S', 'IDLE_GRIP_RAD', 'IDLE_WINDOW_S',
                 'DEFAULT_KEEP_IDLE_S'):
        assert name in TEXT, f'{name} 没写进文档'
        assert f'{getattr(ex, name):g}' in TEXT, f'{name} 的值和文档对不上'
    for field in ('raw_t0', 'raw_t1', 'head_s', 'tail_s', 'keep_idle_s'):
        assert field in TEXT, f'source.trim.{field} 没写进文档'
