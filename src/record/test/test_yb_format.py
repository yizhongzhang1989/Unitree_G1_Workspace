"""`format/YB/README.md` 有没有和代码说的是同一件事。

写这个测试是因为：**文档写错比没写更糟**。规范里那些常量（外参公式、末端约定、
栅格频率、失效阈值）一旦和代码分叉，拿到数据的人会照着文档把数据用错，
而且很久都发现不了。所以这里把文档里出现的每个关键字面量都拿去和 `export.py`
里的定义对一遍。

不校验散文，只校验**数字和标识符** —— 那才是会被人照抄进代码的部分。
"""

from __future__ import annotations

import re
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

#: `episode/*.json` 里的全部键，就是模板与对面样例的交集。
#: `episodes_all.json` 只多一个 `episode_name`（单条那份的文件名就是它）。
INDEX_KEYS = {'episode_id', 'start_frame', 'end_frame', 'instruction'}


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

    所以四个阈值必须在文档里，且和代码一致。
    """
    for name in ('IDLE_POS_M_S', 'IDLE_GRIP_RAD', 'IDLE_WINDOW_S',
                 'DEFAULT_KEEP_IDLE_S'):
        assert name in TEXT, f'{name} 没写进文档'
        assert f'{getattr(ex, name):g}' in TEXT, f'{name} 的值和文档对不上'


def test_only_successful_episodes_are_exported():
    """fail/discard 不导是个会让条数对不上的决定，必须写在文档里。"""
    assert 'success' in TEXT and 'discard' in TEXT
    assert 'fail / discard' in TEXT or 'fail/discard' in TEXT


def test_no_dead_field_references():
    """文档里不能再提已经不导出的字段。

    这些名字都曾经真存在过，拿到旧文档的人会直接按键取值然后 KeyError。
    """
    for gone in ('source.trim', 'video_size', 'instruction_zh', 'episode_json',
                 'pose_frame', 'value_convention', 'unified_fix_quat_xyzw',
                 'intrinsic_size_note'):
        assert gone not in TEXT, f'文档还在提已删掉的 {gone}'


def test_index_keys_are_exactly_the_documented_ones():
    """`write_sidecars` 写出的键必须和 §8 列的一致，多一个就是又混进采集侧的标注了。"""
    source = (HERE / 'export.py').read_text(encoding='utf-8')
    entry = source.split('        entry = {', 1)[1].split('\n        }', 1)[0]
    assert set(re.findall(r"^\s+'(\w+)':", entry, re.M)) == INDEX_KEYS
    for key in INDEX_KEYS:
        assert key in TEXT, f'§8 没写 {key}'


def test_episode_name_only_lives_in_the_all_index():
    """单条里不放 `episode_name` —— 文件名逐字就是它，robotwin 也没放。"""
    assert 'episode_name' not in INDEX_KEYS
    source = (HERE / 'export.py').read_text(encoding='utf-8')
    assert "'episode_name': record['name']" in source


def _postcheck_source() -> str:
    """把 `sync_and_convert.ps1` 里内嵌的那段自检 python 抠出来。"""
    text = (TOOLS / 'sync_and_convert.ps1').read_text(encoding='utf-8')
    return text.split("$checkerSource = @'\n", 1)[1].split("\n'@", 1)[0]


def test_postcheck_script_still_compiles():
    compile(_postcheck_source(), 'yb_postcheck.py', 'exec')


def test_postcheck_reads_only_keys_and_cameras_that_exist():
    """自检脚本在 B 机的 Windows 上跑，这里跑不到 —— 名字写错只能靠静态核对发现。

    踩过两处，都不响：`ep['file']` 这个键**从来没存在过**（一进循环就 KeyError）；
    相机名写的是采集端的 `wrist_left`，而 h5 里是 `leftcam`，于是最要紧的那条
    「腕相机到同侧末端的距离必须是常数」被 `continue` 静默跳过，还照样报「自检通过」。
    """
    source = _postcheck_source()
    used = set(re.findall(r"ep\['(\w+)'\]", source))
    assert used <= INDEX_KEYS | {'episode_name'}, \
        f'自检脚本读了索引里没有的键 {used - INDEX_KEYS - {"episode_name"}}'
    names = [c.name for c in ex.CAMERAS]
    cameras = set(re.findall(r"\('(\w+cam)', *\d\)", source))
    assert cameras, '自检脚本里一条相机名都没抓到，正则或脚本变了'
    assert cameras <= set(names), f'自检脚本认的相机名 h5 里没有：{cameras - set(names)}'
