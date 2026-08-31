#!/usr/bin/env python3
"""能把一次采集转成什么格式 —— **A 的面板和 B 的命令行读的是同一张表**。

分叉是这里唯一要防的事：面板下拉框里列的、`convert.py --list` 打出来的、真正跑的
参数，必须是同一份定义。所以注册表放在 `tools/` 里（这个目录整个拷到 B 就能用），
面板只是去 import 它。

## 加一种新格式

**一个格式一个文件夹，放在 `tools/format/<格式名>/`**，里面至少两样东西：

* `export.py` —— 转换脚本，命令行接口是 `<session> -o <out> [选项]`
* `README.md` —— 格式规范，**交数据给别人时连它一起给**

然后往 `CONVERTERS` 里加一条。面板下拉框和 `convert.py --list` 会自动出现。

通用的东西（读 session、FK、视频重采样）留在 `tools/` 根下给各格式共用，
别拷进格式目录 —— 两份 FK 漂开了，两种格式导出的末端位姿就对不上了。
"""

from __future__ import annotations

import importlib.util
import shutil
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent


@dataclass(frozen=True)
class Converter:
    id: str
    label: str
    script: str
    #: 需要哪些外部依赖。**在点之前就查**，不然跑到一半才失败，日志还在后台。
    modules: tuple = ()
    binaries: tuple = ()
    #: 需要调用方额外给的路径（面板从 ROS 参数取，命令行从 argv 取）
    inputs: tuple = ()
    note: str = ''
    options: dict = field(default_factory=dict)
    #: 跟着跑时能吐进度的开关。面板要（要画进度条），命令行不要（刷屏）。
    progress_flag: str = ''

    def missing(self) -> list[str]:
        absent = [f'python 模块 {m}' for m in self.modules
                  if importlib.util.find_spec(m) is None]
        return absent + [f'命令 {b}' for b in self.binaries if not shutil.which(b)]

    def command(self, python: str, sessions, out, values: dict,
                progress: bool = False) -> list:
        """拼出真正要跑的那条命令。缺必填输入就抛，不给半条命令。

        `sessions` 是一份清单：多个会合并导成同一份 dataset。
        """
        missing = [name for name in self.inputs if not values.get(name)]
        if missing:
            raise ValueError(f'{self.id} 还缺参数: {", ".join(missing)}')
        command = [python, str(HERE / self.script),
                   *(str(item) for item in sessions), '-o', str(out)]
        for name in self.inputs:
            command += [f'--{name.replace("_", "-")}', str(values[name])]
        for name, default in self.options.items():
            value = values.get(name, default)
            if value not in (None, ''):
                command += [f'--{name.replace("_", "-")}', str(value)]
        if progress and self.progress_flag:
            command.append(self.progress_flag)
        return command


CONVERTERS = {
    c.id: c for c in (
        Converter(
            id='yb',
            label='YB 训练数据集（h5 + mp4）',
            script='format/YB/export.py',
            # yaml 只有给了 calibration 才真的 import（export.py 的 `_yaml()`），仍然无条件
            # 要求：腕相机的 link 就是那份标定文件现建的，缺它的人去掉 --calibration 也跑不通
            modules=('numpy', 'h5py', 'yaml'),
            binaries=('ffmpeg', 'ffprobe'),
            inputs=('urdf',),
            options={'calibration': '', 'video_height': 360, 'hz': 30.0},
            progress_flag='--progress',
            note='一条 episode 一个 h5 + 每路相机一个 mp4，30 Hz 统一栅格；'
                 '只导标注成功的那些；腕相机的 link 在 calibration.yaml 里，不给就报错',
        ),
    )
}


def describe() -> list[dict]:
    """给面板下拉框用。缺依赖的那项要能置灰并说清楚缺什么。"""
    return [{'id': c.id, 'label': c.label, 'note': c.note,
             'inputs': list(c.inputs), 'missing': c.missing()}
            for c in CONVERTERS.values()]


def get(converter_id: str) -> Converter:
    if converter_id not in CONVERTERS:
        raise ValueError(f'没有这种格式: {converter_id!r}，'
                         f'可选 {", ".join(CONVERTERS)}')
    return CONVERTERS[converter_id]
