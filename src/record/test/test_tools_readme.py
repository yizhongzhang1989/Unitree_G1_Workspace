"""`tools/README.md` —— 给 B 的那份说明有没有和代码对得上。

B 上的人照着它一步步敲，命令写错、文件名写错、排错表里的报错原文对不上，
第一步就卡住，而且他多半没法自己判断是文档错还是环境错。

所以这里查三件事：点名的文件在不在、给的命令行开关是不是真的存在、
排错表里引用的报错原文是不是真的会被打出来。
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
DOC = (TOOLS / 'README.md').read_text(encoding='utf-8')
SOURCE = '\n'.join(p.read_text(encoding='utf-8') for p in sorted(TOOLS.rglob('*.py'))
                   if '__pycache__' not in p.parts)
sys.path.insert(0, str(TOOLS))

import converters                                          # noqa: E402


def test_the_file_tree_matches_reality():
    """README 开头画的目录树，每一项都得真的在。"""
    listed = re.findall(r'^\s{2}(\S+\.(?:py|md))\s{2,}', DOC, re.M)
    assert len(listed) >= 8, f'只从目录树里抠出 {listed}，正则失效了'
    for name in listed:
        assert (TOOLS / name).is_file(), f'README 里列了 {name}，实际没有'


def test_no_file_is_left_undocumented():
    """反过来：新加了脚本却没写进树里，B 上的人不会知道它存在。"""
    on_disk = {p.relative_to(TOOLS).as_posix() for p in TOOLS.rglob('*.py')
               if '__pycache__' not in p.parts}
    missing = sorted(name for name in on_disk if name not in DOC)
    assert not missing, f'这些文件 README 没提：{missing}'


def test_setup_py_ships_every_tools_file():
    """源码树里有、装出来没有 = B 点「下载导出工具」拿到的是残缺的一份。

    踩过：`glob('tools/*.py')` 只捞 .py，`tools/README.md` 整份说明没装进去，
    而源码树里明明有 —— 本地怎么看都正常。
    """
    setup = (TOOLS.parent / 'setup.py').read_text(encoding='utf-8')
    want = {p.relative_to(TOOLS).as_posix() for p in TOOLS.rglob('*')
            if p.is_file() and '__pycache__' not in p.parts}
    patterns = re.findall(r"glob\(os\.path\.join\(([^)]*)\)\)", setup)
    covered = set()
    for pat in patterns:
        parts = [s.strip().strip("'\"") for s in pat.split(',')]
        if parts[0] != 'tools':
            continue
        for p in TOOLS.glob('/'.join(parts[1:])):
            if p.is_file():
                covered.add(p.relative_to(TOOLS).as_posix())
    assert want <= covered, f'setup.py 没装这些：{sorted(want - covered)}'


@pytest.mark.parametrize('flag', ['--verify', '--align', '--list', '--to',
                                  '--urdf', '--video-height', '--hz'])
def test_documented_flags_exist(flag):
    """文档里写的开关，argparse 里必须真的有。"""
    assert f"'{flag}'" in SOURCE, f'README 让人用 {flag}，代码里没这个开关'


def test_troubleshooting_quotes_real_messages():
    """排错表引用的是报错原文，代码改了文案这里要跟着改。"""
    table = DOC.split('## 5.')[-1]
    for phrase in ('session 没有 DONE，未正常收尾', '找不到 ffmpeg/ffprobe，跳过视频'):
        assert phrase in table, f'排错表里没有 {phrase!r}'
        assert phrase in SOURCE, f'{phrase!r} 不是代码里的原文'


def test_missing_dependency_wording_matches():
    """`跑不了 yb：缺 python 模块 h5py` 这句是拼出来的，逐段核对。"""
    conv = converters.Converter(id='x', label='x', script='format/YB/export.py',
                                modules=('no_such_mod',), binaries=('no_such_bin',))
    assert conv.missing() == ['python 模块 no_such_mod', '命令 no_such_bin']
    table = DOC.split('## 5.')[-1]
    assert '跑不了 yb：缺 python 模块 h5py' in table
    assert '跑不了 yb：缺 命令 ffmpeg' in table
    assert '跑不了 {converter.id}：缺 ' in SOURCE


def test_setup_section_installs_every_declared_dependency():
    """注册表里要什么，§1 就得让人装上 —— 依赖表、apt 那行、pip 那行三处都要有。

    只补表、忘了补安装命令，照着敲的人一样装不全。PyYAML 漏过一整轮：
    `--list` 报可用，跑到 `load_model()` 才崩。
    """
    section = DOC.split('## 1.')[1].split('## 2.')[0].lower()
    apt = next(ln for ln in section.splitlines() if 'apt install' in ln)
    pip = next(ln for ln in section.splitlines() if ln.strip().startswith('pip install'))
    for conv in converters.CONVERTERS.values():
        for name in conv.modules:
            assert name in section, f'§1 的依赖表没提 {name}'
            assert name in apt, f'路线 A 的 apt 那行没装 {name}'
            assert name in pip, f'路线 B 的 pip 那行没装 {name}'
        for name in conv.binaries:
            assert name in section, f'§1 没提命令 {name}'


def test_says_to_copy_the_whole_folder():
    """只拷几个文件是最常见的错法 —— 相对导入会直接崩。"""
    assert '不要只拷其中几个文件' in DOC
    assert 'record-tools.zip' in DOC, '得告诉 B 这一包从哪儿下'


def test_warns_about_what_is_not_in_the_session():
    """URDF 不在 session 里，忘了拷就跑不了；相机参数反过来 —— 只认 session 自带的。"""
    assert 'final.urdf' in DOC
    assert 'camera_params.yaml' in DOC
    assert 'urdf_overrides' in DOC


def test_does_not_claim_git_for_windows_ships_rsync():
    """Git for Windows 不带 rsync，早先版本写错过，别改回去。"""
    assert 'Git for Windows 也**不带**' in DOC


def test_does_not_recommend_append():
    """--append 实测省不了网络字节，却会在文件被改写时留下修不好的旧文件。"""
    assert '别加 `--append`' in DOC
    assert '--delete' in DOC and '绝不加' in DOC
