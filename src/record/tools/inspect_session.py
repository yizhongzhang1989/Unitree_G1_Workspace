#!/usr/bin/env python3
"""检视一个 session：完整性、时间覆盖、每路数据的健康度、episode 清单。

在导出机上跑（只要 Python + numpy，视频对齐额外需要 ffprobe）::

    python inspect_session.py D:/data/20260821_101500
    python inspect_session.py <dir> --verify     # 逐文件核对 sha256，慢
    python inspect_session.py <dir> --align      # 估视频相对机器人状态的时间偏移
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from session_reader import Session, pts_residual          # noqa: E402


def _hz(t: np.ndarray) -> float:
    if t.size < 2:
        return 0.0
    span = float(t[-1] - t[0])
    return (t.size - 1) / span if span > 0 else 0.0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session')
    ap.add_argument('--verify', action='store_true', help='逐文件核对 sha256')
    ap.add_argument('--align', action='store_true',
                    help='用运动互相关估视频时间偏移（需要 ffprobe）')
    ap.add_argument('--align-frames', action='store_true',
                    help='对齐时解码算帧差而不是读包大小，准但慢')
    args = ap.parse_args()

    s = Session(args.session)
    m = s.manifest
    print(f'session   {m["session_id"]}   {m.get("created_iso", "")}')
    print(f'封口      {"是" if s.sealed else "否 —— 未正常收尾，数据可能不完整"}')
    on = [k for k, v in m['streams'].items() if v]
    print(f'录制流    {len(on)} 路: {", ".join(sorted(on))}')
    if s.meta.get('note'):
        print(f'备注      {s.meta["note"]}')

    print('\n--- 信号表 ---')
    print(f'{"表":<26}{"行":>9}{"列":>5}{"Hz":>8}{"跨度s":>9}  时间列')
    t_lo, t_hi = [], []
    for key in sorted(s.tables()):
        t, data = s.table(key)
        if t.size:
            t_lo.append(float(t[0]))
            t_hi.append(float(t[-1]))
        head_ok = np.isfinite(s.table(key)[0]).all()
        span = float(t[-1] - t[0]) if t.size > 1 else 0.0
        print(f'{key:<26}{data.shape[0]:>9}{data.shape[1]:>5}{_hz(t):>8.1f}'
              f'{span:>9.1f}  {"header" if head_ok else "含回退 recv"}')

    print('\n--- 视频 ---')
    print(f'{"路":<16}{"帧":>8}{"fps":>8}{"标称":>7}{"跨度s":>9}'
          f'{"残差 p50":>9}{"p95":>8}{"max":>8}  文件')
    dropped = []
    for name in ('wrist_left', 'wrist_right', 'head'):
        p = s.video_path(name)
        if not p.is_file():
            continue
        raw = s.video_pts(name, fit=False)
        span = float(raw[-1] - raw[0]) if raw.size > 1 else 0.0
        r = pts_residual(raw)
        if raw.size:
            t_lo.append(float(raw[0]))
            t_hi.append(float(raw[-1]))
        nom = s.nominal_fps(name)
        # 两台腕相机出厂帧率就不一样，不拿标称值比会把 25 fps 的那台误判成丢帧
        if nom and r['fps'] < nom * 0.95:
            dropped.append(f'{name}({r["fps"]:.1f}/{nom:.0f})')
        print(f'{name:<16}{r["frames"]:>8}{r["fps"]:>8.1f}'
              f'{(f"{nom:.0f}" if nom else "?"):>7}{span:>9.1f}'
              f'{r["p50_ms"]:>9.1f}{r["p95_ms"]:>8.1f}{r["max_ms"]:>8.1f}'
              f'  {p.stat().st_size / 1e6:.0f} MB')
    print('  残差 = 原始到达时刻与等间隔重建的偏差(ms)。腕部走 RTSP 成簇到达，'
          '几十毫秒是常态。')
    print(f'  丢帧: {", ".join(dropped)}' if dropped else '  丢帧: 无（实测 fps 均达标称值 95%）')

    if t_lo:
        print(f'\n各路起点相差 {max(t_lo) - min(t_lo):.2f} s，'
              f'终点相差 {max(t_hi) - min(t_hi):.2f} s')

    eps = s.episodes(include_discarded=True)
    kept = [e for e in eps if e['outcome'] != 'discard']
    n_ok = sum(1 for e in kept if e['outcome'] == 'success')
    print(f'\n--- episode: 共 {len(eps)} 条，留存 {len(kept)}'
          f'（成功 {n_ok} / 失败 {len(kept) - n_ok}）---')
    for e in eps[:20]:
        flag = {'success': '✓', 'fail': '✗', 'discard': '−'}[e['outcome']]
        print(f'  {flag} r{e["round"]}e{e["episode"]:<3} {e["duration"]:>5.1f}s  '
              f'{e.get("instruction_en", "")}')
    if len(eps) > 20:
        print(f'  … 还有 {len(eps) - 20} 条')

    warns = s.warnings()
    if warns:
        print(f'\n--- 警告 {len(warns)} 条（不阻塞落盘，供事后修正）---')
        for w in warns[:10]:
            print(f'  {w["code"]}: {w["detail"]}')

    if args.align:
        from align_video import align_stream
        print('\n--- 视频时间偏移（图像运动 x 关节角速度互相关）---')
        for name in ('wrist_left', 'wrist_right'):
            if not s.video_path(name).is_file():
                continue
            r = align_stream(s, name, use_frames=args.align_frames)
            if 'error' in r:
                print(f'  {name}: {r["error"]}')
                continue
            o = r['overall']
            print(f'  {name} ({r["source"]}): 整段 {o["delay_ms"]:+.1f} ms  '
                  f'置信 {o["score"]:.2f}/峰差 {o["margin"]:.2f}  '
                  f'{"可用" if o["trustworthy"] else "不可用（运动太少或不相关）"}')
            if r['trustworthy_windows'] > 1:
                print(f'    分窗 {r["trustworthy_windows"]} 个可用，'
                      f'偏移极差 {r["spread_ms"]:.1f} ms'
                      f'{"  ← 偏移在漂，导出时要逐窗修正" if r["spread_ms"] > 20 else ""}')
            for w in r['windows']:
                if w['trustworthy']:
                    print(f'      t0={w["t0"]:.1f}  {w["delay_ms"]:+.1f} ms  '
                          f'({w["score"]:.2f})')

    if args.verify:
        print('\n--- sha256 核对 ---')
        bad = s.verify()
        print('  全部一致' if not bad else '\n'.join('  ' + b for b in bad))
        return 1 if bad else 0
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
