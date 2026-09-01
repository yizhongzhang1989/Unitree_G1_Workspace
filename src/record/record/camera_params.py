"""每次采集自带的相机参数快照（``camera_params.yaml``）。

标定会变（相机被碰、被重装），而一次采集是不可变的历史记录。把「这次采集用的
相机参数」钉进 session，导出时就不必猜该配哪一版标定 —— 2026-08-31 头部相机偏了
13.7°，正是没钉住的代价：同一份 `calibration.yaml` 没法同时解释 8/28 和 8/31 两批。

只留这次真正用到的那一档分辨率。`export.py::collect_intrinsics` 对腕相机拿不到
录制分辨率时会**取标定表第一档**，档位顺序一变就静默拿 640x360 的 K 去导 1080p 的
视频；表里只剩一档就不会取错。

内容是 `calibration.yaml` 的裁剪版，键完全一致（`intrinsics` / `urdf_overrides`），
导出侧直接当标定字典用。不含 `extrinsics`（求解中间量，FK 读的是 `urdf_overrides`）。
"""

from __future__ import annotations

import json
from pathlib import Path

import yaml

#: 文件名。`tools/session_reader.py` 里有一份同名常量，测试核对两者一致。
FILENAME = 'camera_params.yaml'
SCHEMA_VERSION = 1

#: (快照里的名字, `calibration.yaml` 的 intrinsics 键, 录制分辨率的来源)。
#: 头部没有 intrinsics 键 —— 它在 `cameras.yaml` 里是 role: reference，只出外参
#: 修正不出内参，内参是 D435i 出厂值，已经在 session 自己的 `meta.json` 里。
CAMERAS = (
    ('head', '', 'meta'),
    ('wrist_left', 'camera_left', 'nominal'),
    ('wrist_right', 'camera_right', 'nominal'),
)


def recorded_sizes(root: str | Path) -> dict:
    """每路录制时的实际分辨率 ``{名字: (宽, 高)}``，取不到的那路不出现。

    腕部读 `video/nominal.json`（开录时 ffprobe 探的，两台相机出厂档位本来就不同，
    不能假定），头部读 `meta.json` 的 `head_stream`。
    """
    root = Path(root)
    nominal = _json(root / 'video' / 'nominal.json')
    meta = _json(root / 'meta.json')
    out = {}
    for name, _, where in CAMERAS:
        probe = meta.get(f'{name}_stream') if where == 'meta' else nominal.get(name)
        if probe and probe.get('width') and probe.get('height'):
            out[name] = (int(probe['width']), int(probe['height']))
    return out


def build(root: str | Path, calibration: dict) -> dict:
    """从整份标定裁出这次采集用到的那部分。

    `calibration` 是 `calibration.yaml` 已解析的字典。分辨率对不上标定表的那一路
    直接不出现在 `intrinsics` 里 —— 内参不做缩放换算，导出侧看到缺就写 NaN，
    这比塞一份等比缩放出来的假 K 安全。
    """
    sizes = recorded_sizes(root)
    table = calibration.get('intrinsics') or {}
    intrinsics = {}
    for name, key, _ in CAMERAS:
        entry = _pick(table.get(key) or [], sizes.get(name)) if key else None
        if entry is not None:
            intrinsics[key] = [entry]
    return {
        'version': SCHEMA_VERSION,
        'intrinsics': intrinsics,
        # FK 要的就是这三条（头部 d435_joint + 两条腕相机光心）。原样保留，
        # `apply_overrides` 只读 parent/child/xyz/rpy/create，多余的键无害。
        'urdf_overrides': calibration.get('urdf_overrides') or {},
    }


def write(root: str | Path, calibration: dict) -> Path:
    """生成快照并落盘，返回文件路径。开录后、封口前写 —— 分辨率要等 ffprobe 探完。"""
    path = Path(root) / FILENAME
    path.write_text(yaml.safe_dump(build(root, calibration), sort_keys=False,
                                   allow_unicode=True), encoding='utf-8')
    return path


def read_calibration(path: str | Path) -> dict | None:
    """读整份 `calibration.yaml`，读不出来返回 None。"""
    source = Path(path) if path else None
    if source is None or not source.is_file():
        return None
    try:
        return yaml.safe_load(source.read_text(encoding='utf-8')) or {}
    except (OSError, ValueError):
        return None


def _pick(entries: list, size: tuple | None) -> dict | None:
    """标定表里匹配这个分辨率的那一条。**只做精确匹配** —— 腕相机两路 RTSP 是独立
    的流不是同一路的缩放，按比例换算出来的 K 看着正常但是错的。"""
    if not size:
        return None
    for entry in entries:
        if (entry.get('width'), entry.get('height')) == size:
            return entry
    return None


def _json(path: Path) -> dict:
    if not path.is_file():
        return {}
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return {}
