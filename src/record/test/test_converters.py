"""`tools/converters.py` 与 `tools/convert.py`。

这两个文件是「A 的面板」和「B 的命令行」共用的那张表。分叉了就会出现
面板下拉框里列着、B 上却跑不了（或者反过来）的情况，所以这里钉住三件事：
表本身的形状、缺依赖时的报告、以及拼出来的命令行。

和 `test_export_h5.py` 一样，不 import rclpy —— B 上没有 ROS。"""

from __future__ import annotations

import ast
import re
import subprocess
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
sys.path.insert(0, str(TOOLS))

import converters                                          # noqa: E402


def test_no_ros_dependency():
    """B 上没有 ROS，这两个文件碰了就是启动即崩。"""
    banned = re.compile(r'^\s*(import|from)\s+(rclpy|sensor_msgs|record)\b', re.M)
    for name in ('converters.py', 'convert.py'):
        assert not banned.search((TOOLS / name).read_text(encoding='utf-8')), name


def test_registry_shape():
    for key, conv in converters.CONVERTERS.items():
        assert conv.id == key, '键和 id 不一致，describe() 会给出对不上的 id'
        assert (TOOLS / conv.script).is_file(), f'{key} 指向不存在的脚本'
        assert conv.label and conv.note


def test_every_format_lives_in_its_own_folder():
    """一个格式一个 `format/<名>/`，里面必须有 export.py 和 README.md。

    README 是硬要求：把数据交给别人时要连规范一起给，没有它对方只能猜字段含义。
    """
    folders = sorted(p for p in (TOOLS / 'format').iterdir() if p.is_dir()
                     and p.name != '__pycache__')
    assert folders, 'format/ 下一个格式都没有'
    for folder in folders:
        assert (folder / 'export.py').is_file(), f'{folder.name} 缺 export.py'
        assert (folder / 'README.md').is_file(), f'{folder.name} 缺格式规范 README.md'


def test_registry_covers_every_format_folder():
    """目录里有、注册表里没 = 面板下拉框里看不到，写了等于白写。"""
    on_disk = {p.name for p in (TOOLS / 'format').iterdir()
               if p.is_dir() and p.name != '__pycache__'}
    registered = {Path(c.script).parent.name for c in converters.CONVERTERS.values()}
    assert on_disk == registered, f'目录 {sorted(on_disk)} vs 注册表 {sorted(registered)}'


def test_describe_matches_registry():
    got = converters.describe()
    assert [d['id'] for d in got] == list(converters.CONVERTERS)
    for d in got:
        assert set(d) == {'id', 'label', 'note', 'inputs', 'missing'}
        assert isinstance(d['missing'], list)


def _third_party_imports(entry: Path) -> set:
    """`entry` 连同它在 `tools/` 里 import 到的模块，一共用到哪些第三方顶层包。

    **函数体里的 import 也算** —— `import yaml` 就藏在 export.py 的 `_yaml()` 里，
    这类惰性 import 正是「`--list` 说可用、跑到一半才崩」的来源。
    """
    local = {p.stem: p for p in TOOLS.rglob('*.py') if '__pycache__' not in p.parts}
    third, queue, seen = set(), [entry], {entry}
    while queue:
        for node in ast.walk(ast.parse(queue.pop().read_text(encoding='utf-8'))):
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module] if node.level == 0 and node.module else []
            else:
                continue
            for root in (n.split('.')[0] for n in names):
                if root in sys.stdlib_module_names:
                    continue
                if root not in local:
                    third.add(root)
                elif local[root] not in seen:
                    seen.add(local[root])
                    queue.append(local[root])
    return third


@pytest.mark.parametrize('conv', list(converters.CONVERTERS.values()),
                         ids=lambda c: c.id)
def test_declared_modules_cover_every_import(conv):
    """转换脚本真正 import 的第三方包，一个都不能漏在 `modules` 外面。

    漏了的后果不是「少装一个包」：预检说可用，跑到一半抛原始 traceback，
    而最省事的绕法（去掉 `--calibration`）会静默把外参降级成 URDF 名义值。
    yaml 就这么漏过一次。
    """
    used = _third_party_imports(TOOLS / conv.script)
    assert used <= set(conv.modules), \
        f'{conv.id} 用了却没声明：{sorted(used - set(conv.modules))}'


def test_yb_is_runnable_here():
    """开发机上依赖齐了，缺了说明 Dockerfile 漏东西。"""
    assert converters.get('yb').missing() == []


def test_missing_reports_both_kinds():
    """报的是给人看的描述串，不是裸名字 —— 这串会直接上面板下拉框。"""
    conv = converters.Converter(
        id='x', label='x', script='format/YB/export.py',
        modules=('numpy', 'no_such_module_xyz'),
        binaries=('ffmpeg', 'no_such_binary_xyz'))
    assert conv.missing() == ['python 模块 no_such_module_xyz',
                              '命令 no_such_binary_xyz']


def test_get_rejects_unknown():
    """ValueError 而不是 KeyError：面板把它的文案直接回给前端，得看得懂。"""
    with pytest.raises(ValueError, match='yb'):
        converters.get('nope')


def test_command_carries_inputs_and_options():
    conv = converters.get('yb')
    cmd = conv.command('/usr/bin/python3', Path('/s'), Path('/o'),
                       {'urdf': '/u.urdf', 'calibration': '/c.yaml',
                        'video_height': 360})
    assert cmd[0] == '/usr/bin/python3'
    assert cmd[1].endswith('export.py')
    assert '/s' in cmd and '/o' in cmd
    for flag, value in (('--urdf', '/u.urdf'), ('--calibration', '/c.yaml'),
                        ('--video-height', '360')):
        assert cmd[cmd.index(flag) + 1] == value


def test_command_skips_empty_options():
    """没标定就用 URDF 名义值，不能传个空串下去让 export 去开空文件。"""
    conv = converters.get('yb')
    cmd = conv.command('py', Path('/s'), Path('/o'),
                       {'urdf': '/u.urdf', 'calibration': ''})
    assert '--calibration' not in cmd


def test_command_requires_declared_inputs():
    conv = converters.get('yb')
    with pytest.raises(ValueError, match='urdf'):
        conv.command('py', Path('/s'), Path('/o'), {'urdf': ''})


def test_progress_flag_is_opt_in():
    """面板要进度条，命令行不要 —— 那一秒好几条会把报告刷没。"""
    conv = converters.get('yb')
    values = {'urdf': '/u.urdf'}
    assert '--progress' not in conv.command('py', Path('/s'), Path('/o'), values)
    assert '--progress' in conv.command('py', Path('/s'), Path('/o'), values,
                                        progress=True)


def test_convert_list_runs():
    """B 上第一条命令就是 `--list`，它崩了等于整条路走不通。"""
    out = subprocess.run([sys.executable, str(TOOLS / 'convert.py'), '--list'],
                         capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    assert 'yb' in out.stdout
