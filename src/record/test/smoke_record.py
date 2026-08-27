#!/usr/bin/env python3
"""采集烟测：起节点 -> 打面板 API -> 录一轮 -> 封口 -> 用导出工具读回来。

默认只勾信号流，这样任何时候都能跑，守住「HTTP -> 状态机 -> 落盘 -> 读回」这条链。
加 ``--video`` 则探一下三路相机，能通的都开，验证真实的 ffmpeg 链路。
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

PORT = 8231
ROOT = Path('/tmp/record_smoke/sessions')


def call(path, payload=None):
    """payload 为 None 时是 GET；要发 POST 就传字典（空字典也行）。"""
    url = f'http://127.0.0.1:{PORT}{path}'
    data = None if payload is None else json.dumps(payload).encode()
    req = urllib.request.Request(url, data=data, method=('GET' if data is None else 'POST'),
                                 headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as exc:
        # 后端把原因放在 body 里，不看就只剩一个 400
        raise SystemExit(f'{path} 失败: {exc.read().decode("utf-8", "replace")}') from exc


def _rejected(path, payload, why: str) -> str:
    """这一条必须被后端拒掉，返回它给的理由。call() 把 400 的 body 包成 SystemExit。"""
    try:
        call(path, payload)
    except SystemExit as exc:
        return str(exc).split('失败: ')[-1]
    raise AssertionError(why)


def _stop_group(proc: subprocess.Popen) -> None:
    """按进程组收尾。SIGTERM 让节点跑完 finally 把 ffmpeg 收掉，收不干净再补 SIGKILL。"""
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    os.killpg(pgid, signal.SIGTERM)
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def _rtsp_ok(url: str) -> bool:
    """探一路 RTSP 通不通。ffprobe 没有 -nostdin，别加。"""
    try:
        out = subprocess.run(
            ['ffprobe', '-v', 'error', '-rtsp_transport', 'tcp', '-select_streams', 'v:0',
             '-show_entries', 'stream=codec_name', '-of', 'csv=p=0', url],
            capture_output=True, timeout=25, stdin=subprocess.DEVNULL, text=True)
        return out.returncode == 0 and bool(out.stdout.strip())
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--video', action='store_true', help='一并开启能探到的相机')
    ap.add_argument('--hold', type=float, default=0.4, help='每条 episode 停留秒数')
    args = ap.parse_args()

    # 上一轮失败时残留的节点会占着端口，新节点绑不上，请求就打到旧节点身上，
    # 表现为莫名其妙的「已经在录了」
    subprocess.run(['pkill', '-f', f'dashboard_port:={PORT}'], check=False)
    time.sleep(1.5)
    subprocess.run(['rm', '-rf', str(ROOT.parent)], check=False)
    node = subprocess.Popen(
        ['ros2', 'run', 'record', 'recorder', '--ros-args',
         '-p', f'output_root:={ROOT}', '-p', f'dashboard_port:={PORT}',
         '-p', 'round_items:=4', '-p', 'round_moves:=4'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        start_new_session=True)   # ros2 run 只是包一层，得按进程组杀才能要到真正的节点
    try:
        for _ in range(60):
            time.sleep(1)
            try:
                state = call('/api/state')
                break
            except Exception:                      # noqa: BLE001
                if node.poll() is not None:
                    print('节点退出了:\n', node.stdout.read())
                    return 1
        else:
            print('面板 60 s 没起来')
            return 1

        print(f'数据流 {len(state["streams"])} 路，'
              f'在线 {sum(1 for s in state["streams"] if s["online"])} 路')
        if state['status']['library_error']:
            print('物品库异常:', state['status']['library_error'])
            return 1

        streams = {s['key']: s['default_on'] and s['kind'] == 'signal'
                   for s in state['streams']}
        if args.video:
            for s in state['streams']:
                if s['kind'] != 'video' or s['key'] == 'head_depth':
                    continue
                ok = s['online'] if s['key'] == 'head' else _rtsp_ok(s['topic'])
                streams[s['key']] = ok
                print(f'  视频 {s["key"]:<12} {"开" if ok else "跳过（探不到）"}')
        # 先 roll 再开录：没 roll 就开录的话，生任务和摆桌子那几十秒全录进视频，
        # 所以 start_session 会直接拒掉
        print('设桌面尺寸…')
        full = call('/api/state')['status']['table']
        small = call('/api/table', {'width_mm': 600, 'depth_mm': 200,
                                    'near_mm': 100})['result']['table']
        print(f'  {full["width_mm"]}x{full["depth_mm"]} -> '
              f'{small["width_mm"]}x{small["depth_mm"]}，'
              f'可放格心 {full["cells"]} -> {small["cells"]}')
        assert small['cells'] < full['cells'], small
        # 浅桌子先饿死容器，这时必须当场报错而不是等「生成任务」才炸
        print('  拒掉过浅的桌面：' + _rejected(
            '/api/table', {'width_mm': 500, 'depth_mm': 200, 'near_mm': 120},
            '容器一件都放不下的桌面居然被接受了'))
        assert call('/api/state')['status']['table'] == small, '失败的设定不该改状态'
        call('/api/table', full)                 # 后面照完整可达域走

        print('生成一轮摆放…')
        call('/api/round/preview', {'seed': 20260821})
        # 改桌面会丢掉按旧桌面摆的那一轮：照它摆真桌子会摆到桌沿外面
        assert call('/api/table', full)['result']['pending_round'] is None, '预览没被清掉'
        call('/api/round/preview', {'seed': 20260821})
        print('开 session…')
        call('/api/session/start', {'streams': streams, 'note': 'smoke'})
        call('/api/round/start', {})
        detail = call('/api/state')['status']['round_detail']
        # 固化之后桌子已经照它摆好了，尺寸必须锁死
        assert call('/api/state')['status']['table_locked'], '固化后桌面尺寸没锁上'
        _rejected('/api/table', full, '固化后还能改桌面尺寸')
        print(f'  物品 {[i["zh"] for i in detail["items"]]}')
        print(f'  episode {len(detail["episodes"])} 条，lint 警告 '
              f'{len(detail["lint_warnings"])} 条')
        for ep in detail['episodes'][:4]:
            print(f'    {ep["instruction_en"]}')

        svg = urllib.request.urlopen(
            f'http://127.0.0.1:{PORT}/api/round/svg', timeout=10).read()
        print(f'  摆放样例 SVG {len(svg)} 字节')

        for k in range(min(3, len(detail['episodes']))):
            call('/api/episode/start', {'index': k})
            time.sleep(args.hold)
            call('/api/episode/end', {'outcome': 'success' if k else 'fail'})

        # 重录第 0 条。走这一步 episode_index（第几次录）就和指令表下标分家了，
        # 面板高亮哪一行只能看 episode_slot —— 曾经看错，重录后整排按钮全消失
        st = call('/api/episode/start', {'index': 0})['result']
        assert st['episode_slot'] == 0, st
        assert st['episode'] != st['episode_slot'], f'重录后这两个还相等，测试没测到点上: {st}'
        time.sleep(args.hold)
        call('/api/episode/end', {'outcome': 'discard'})
        st = call('/api/state')['status']
        assert st['slot_takes'].get('0') == ['fail', 'discard'], st['slot_takes']
        call('/api/round/end', {})

        # 「结束本轮」把这一轮退回预览：桌子还照它摆着，再开一轮不该逼人重摆一次
        st = call('/api/state')['status']
        assert st['pending_round'], f'结束本轮后配置被清空了: {st}'
        assert st['pending_round']['seed'] == detail['seed'], st['pending_round']
        st = call('/api/round/start', {})['result']
        assert st['round'] == 1, st
        assert st['round_detail']['seed'] == detail['seed'], st['round_detail']['seed']
        assert st['slot_takes'] == {}, f'新一轮该从零开始: {st["slot_takes"]}'
        call('/api/round/end', {})

        # 只换摆放和指令，物品不动 —— 换物品要起身去桌上找东西，代价差一个数量级。
        # 判据必须看 layout（桌上真摆着的），items 相同但 layout 丢件也算换了内容
        before = sorted(p['item_id'] for p in detail['layout'])
        st = call('/api/round/preview', {'keep_items': True})['result']
        got = sorted(p['item_id'] for p in st['pending_round']['layout'])
        assert got == before, f'keep_items 换了桌面内容: {got} != {before}'
        assert st['pending_round']['seed'] != detail['seed'], '没重 roll'

        print('封口…')
        result = call('/api/session/stop', {})['result']
        print(f'  {result["session"]}: {result["files"]} 个文件 '
              f'{result["bytes"] / 1e6:.2f} MB')
    finally:
        _stop_group(node)

    session_dir = sorted(ROOT.iterdir())[-1]
    tools = Path(__file__).resolve().parents[1] / 'tools'
    print(f'\n--- 用导出工具读 {session_dir.name} ---')
    out = subprocess.run([sys.executable, str(tools / 'inspect_session.py'),
                          str(session_dir), '--verify'],
                         capture_output=True, text=True)
    print(out.stdout or out.stderr)
    return out.returncode


if __name__ == '__main__':
    raise SystemExit(main())
