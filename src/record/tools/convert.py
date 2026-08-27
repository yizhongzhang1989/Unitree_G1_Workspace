#!/usr/bin/env python3
"""B 侧的转换入口。**和 A 面板下拉框读的是同一张注册表**，选项逐字一致。

    python convert.py --list
    python convert.py <session 目录> --to yb -o <输出目录> \\
        --urdf final.urdf --calibration calibration.yaml

只做三件事：查依赖、拼命令、转发输出。真正的活在 `format/<格式名>/export.py` 里，
这里不复制任何一行转换逻辑 —— 复制了就会和面板跑的那条分叉。
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import converters                                          # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session', nargs='?')
    ap.add_argument('--list', action='store_true', help='列出支持的格式与依赖状态')
    ap.add_argument('--to', default='dataset_format')
    ap.add_argument('-o', '--out')
    ap.add_argument('--urdf')
    ap.add_argument('--calibration')
    ap.add_argument('--video-height', type=int)
    ap.add_argument('--hz', type=float)
    args = ap.parse_args()

    if args.list:
        for item in converters.describe():
            state = '缺 ' + '、'.join(item['missing']) if item['missing'] else '可用'
            print(f'{item["id"]:<16} {item["label"]}  [{state}]')
            print(f'{"":16} {item["note"]}')
            print(f'{"":16} 必填: {", ".join(item["inputs"]) or "无"}')
        return 0

    if not args.session or not args.out:
        ap.error('要给 session 目录和 -o 输出目录（或者用 --list）')
    converter = converters.get(args.to)
    missing = converter.missing()
    if missing:
        print(f'跑不了 {converter.id}：缺 {"、".join(missing)}', file=sys.stderr)
        return 2
    values = {k: v for k, v in vars(args).items() if v is not None}
    command = converter.command(sys.executable, Path(args.session).expanduser(),
                                Path(args.out).expanduser(), values)
    print('$', ' '.join(command))
    return subprocess.call(command)


if __name__ == '__main__':
    raise SystemExit(main())
