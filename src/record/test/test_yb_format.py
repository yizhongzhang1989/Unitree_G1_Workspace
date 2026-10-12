"""Export documentation location and Windows postcheck compatibility."""

from __future__ import annotations

import re
from pathlib import Path
from tools.format.YB import export as ex

REPO = Path(__file__).resolve().parents[3]
TOOLS = Path(__file__).resolve().parents[1] / 'tools'
HERE = TOOLS / 'format' / 'YB'
DOC = HERE / 'README.md'

INDEX_KEYS = {'episode_id', 'start_frame', 'end_frame', 'instruction'}


def test_meta_points_at_a_doc_that_exists():
    assert DOC.is_file()
    assert (REPO / 'src' / ex.FORMAT_DOC).resolve() == DOC.resolve()


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
