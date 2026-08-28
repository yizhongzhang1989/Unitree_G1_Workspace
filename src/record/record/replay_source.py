"""把导出机那套只读工具（`tools/session_reader.py`）接进 ROS 侧。

`tools/` 刻意不属于 `record` 包 —— 它要能整个拷到没有 ROS 的 Windows 上用。所以
这里按路径加载而不是 import。装包后在 `share/record/tools/`，在仓库里直接跑时在
源码目录。

**加载前必须先注册进 `sys.modules`**，否则模块内部的相对引用与 dataclass 解析会
找不到自己。
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


def _tools_dir() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('record')) / 'tools'
        if (share / 'session_reader.py').is_file():
            return share
    except Exception:                              # noqa: BLE001
        pass
    src = Path(__file__).resolve().parents[1] / 'tools'
    if (src / 'session_reader.py').is_file():
        return src
    raise RuntimeError('找不到 tools/session_reader.py')


def _load(name: str):
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, _tools_dir() / f'{name}.py')
    if spec is None or spec.loader is None:
        raise RuntimeError(f'加载不了 {name}')
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod                        # 必须先注册再 exec
    spec.loader.exec_module(mod)
    return mod


def open_session(path):
    return _load('session_reader').Session(path)


def episode_label(round_index: int, episode_index: int) -> str:
    """episode 的标签由读侧定义 —— 校验视频文件名和删除记录都拿它当键，分叉了就对不上。"""
    return _load('session_reader').episode_label(round_index, episode_index)


def edits_path(root) -> Path:
    """封口后的人工修订文件。写在这边，读在 ``session_reader``，名字只定义一次。"""
    return Path(root) / _load('session_reader').EDITS_FILE


def describe(path) -> dict:
    """给面板用的 session 概要：整段范围 + 每条 episode 的时间窗。"""
    sess = open_session(path)
    try:
        t, _ = sess.table('motion_control_command')
    except (KeyError, OSError, ValueError) as exc:
        # 勾选是可以不勾这一路的，所以“表不存在”是正常情况，不能报到堆栈上去
        return {'id': Path(path).name,
                'error': f'读不出 motion_control_command（{exc}）—— 没录这一路就无法回放'}
    if t.size == 0:
        return {'id': Path(path).name, 'error': '这次采集的 motion_control_command 是空的，放不了'}
    eps = []
    for e in sess.episodes(include_discarded=True):
        t0, t1 = float(e.get('t0', 0.0)), float(e.get('t1', 0.0))
        if t1 <= t0:
            continue
        eps.append({
            'label': e['label'],
            'outcome': e.get('outcome', ''),
            'instruction': e.get('instruction_en', ''),
            'duration': round(t1 - t0, 1),
            't0': t0, 't1': t1,
            'commands': int(((t >= t0) & (t <= t1)).sum()),
        })
    return {
        'id': Path(path).name,
        'note': sess.meta.get('note', ''),
        'whole': {'t0': float(t[0]), 't1': float(t[-1]),
                  'duration': round(float(t[-1] - t[0]), 1), 'commands': int(t.size)},
        'episodes': eps,
    }
