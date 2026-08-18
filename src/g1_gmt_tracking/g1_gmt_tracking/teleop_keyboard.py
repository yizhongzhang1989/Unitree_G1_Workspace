#!/usr/bin/env python3
"""g1_gmt_tracking 的键盘操作台：状态切换和动作切换都在这里按键完成。

它**不产生任何关节目标**——只调 ``~/engage`` / ``~/start`` / ``~/estop`` 三个服务，
再往 ``~/select_motion`` 发一个动作名。动作本身由跟踪节点自己放。

动作名单不在这里写死，而是问跟踪节点要它的 ``motion_dir`` 参数再列目录：名单和
它实际加载的那一份必然一致，往 ``config/motions/`` 里丢新 NPZ 不用改这个文件。
"""

from __future__ import annotations

import os
import select
import shutil
import sys
import termios
import time
import tty
import unicodedata
from pathlib import Path

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from g1_gmt_tracking.gmt_runtime import resolve_policy_path

HELP = """\
  G         engage：激活 forward_position_controller，机器人保持当前位形不动
  Enter     start：从实测位形插值到参考动作第 0 帧，插完自动开始放
  空格      estop：反激活控制器，机器人进入阻尼

  ← / →     上一段 / 下一段参考动作
  1 ~ 9     直接选第 N 段
  Q         退出（退出前自动急停）

第一次上机务必吊装，并且先用 proc_stand 这类准静态动作试。
"""


class GmtTeleop(Node):

    def __init__(self) -> None:
        super().__init__('gmt_teleop')
        target = self.declare_parameter('tracking_node', '/gmt_tracking') \
            .get_parameter_value().string_value.rstrip('/')

        self._select = self.create_publisher(String, f'{target}/select_motion', 10)
        self.create_subscription(String, f'{target}/status', self._on_status, 10)
        self._engage = self.create_client(Trigger, f'{target}/engage')
        self._start = self.create_client(Trigger, f'{target}/start')
        self.estop_client = self.create_client(Trigger, f'{target}/estop')
        self._parameters = self.create_client(GetParameters, f'{target}/get_parameters')

        self._motions: list[str] = []
        self._next_query = 0.0
        self._fields: dict[str, str] = {}
        self._reason = ''
        self._stamp = 0.0
        self._lines: list[str] = []
        self.pending: set[str] = set()

    # -- 输入 ------------------------------------------------------------------

    def key(self, name: str) -> bool:
        """处理一个按键，返回 False 表示要退出。"""
        if name == 'g':
            self.request(self._engage, 'engage')
        elif name in ('\r', '\n'):
            self.request(self._start, 'start')
        elif name == ' ':
            self.request(self.estop_client, 'estop')
        elif name in ('left', 'right'):
            self._step(-1 if name == 'left' else 1)
        elif name in '123456789':
            self._pick(int(name) - 1)
        elif name == 'q':
            return False
        return True

    def request(self, client, label: str) -> None:
        if not client.service_is_ready():
            self._log(f'{label}: 跟踪节点的服务不在')
            return
        self.pending.add(label)
        client.call_async(Trigger.Request()).add_done_callback(
            lambda future: self._done(label, future))

    def _done(self, label: str, future) -> None:
        self.pending.discard(label)
        result = future.result()
        if result is None:
            self._log(f'{label}: 无响应')
        else:
            self._log(f'{label}: {"ok" if result.success else "拒绝"} — {result.message}')

    def _step(self, delta: int) -> None:
        if not self._motions:
            self._log('还没拿到动作名单')
            return
        current = self._fields.get('motion', '')
        index = self._motions.index(current) if current in self._motions else 0
        self._pick((index + delta) % len(self._motions))

    def _pick(self, index: int) -> None:
        if not self._motions:
            self._log('还没拿到动作名单')
            return
        if not 0 <= index < len(self._motions):
            self._log(f'只有 {len(self._motions)} 段动作')
            return
        # 跟踪节点自己也会拒绝，但 select_motion 是话题没有回执，这里先给个即时反馈。
        if self._fields.get('state') == 'running':
            self._log('RUNNING 中不能换动作，先按空格急停')
            return
        message = String()
        message.data = self._motions[index]
        self._select.publish(message)

    # -- 订阅与查询 ------------------------------------------------------------

    def _on_status(self, message: String) -> None:
        head, _, self._reason = message.data.partition(' reason=')
        self._fields = dict(item.split('=', 1) for item in head.split() if '=' in item)
        self._stamp = time.monotonic()

    def poll(self) -> None:
        """跟自己列一遍 config/motions 相比，问节点要 motion_dir 才不会两边不一致。"""
        if self._motions or time.monotonic() < self._next_query:
            return
        self._next_query = time.monotonic() + 1.0
        if not self._parameters.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = ['motion_dir']
        self._parameters.call_async(request).add_done_callback(self._on_motion_dir)

    def _on_motion_dir(self, future) -> None:
        result = future.result()
        if result is None or not result.values:
            return
        directory = resolve_policy_path(result.values[0].string_value)
        self._motions = sorted(path.stem for path in Path(directory).glob('*.npz'))
        if self._motions:
            listing = '  '.join(f'{i + 1} {n}' for i, n in enumerate(self._motions))
            self._log(f'参考动作: {listing}')

    # -- 输出 ------------------------------------------------------------------

    def _log(self, line: str) -> None:
        self._lines.append(line)

    def drain(self) -> list[str]:
        lines, self._lines = self._lines, []
        return lines

    def render(self) -> str:
        if time.monotonic() - self._stamp > 1.0:
            line = '(收不到 ~/status，跟踪节点起来了吗？)'
        else:
            motion = self._fields.get('motion', '?')
            if motion in self._motions:
                motion = f'{self._motions.index(motion) + 1}/{len(self._motions)} {motion}'
            line = (f'state={self._fields.get("state", "?")}'
                    f' | 动作 {motion} | frame={self._fields.get("frame", "?")}')
            if self._reason:
                line += f' | ⚠ {self._reason}'
        if self.pending:
            line += ' | ' + ' '.join(sorted(self.pending)) + ' 等回执…'
        return f'\r\033[K{_clip(line, shutil.get_terminal_size().columns - 1)}'


def _clip(text: str, columns: int) -> str:
    """按显示宽度截断。状态行一旦溢出终端宽度就会折行，``\\r`` 只能回到最后一行
    的行首，改不掉上一行——于是每刷新一帧屏幕就往下滚一行。
    """
    used = 0
    for index, char in enumerate(text):
        used += 2 if unicodedata.east_asian_width(char) in 'WFA' else 1
        if used > columns:
            return text[:index]
    return text


_ARROWS = {'A': 'up', 'B': 'down', 'C': 'right', 'D': 'left'}
_pending = b''


def read_key(timeout: float) -> str | None:
    """读一个按键，方向键归一成 ``left``/``right``/``up``/``down``。

    走 ``os.read`` 而不是 ``sys.stdin.read(1)``：后者会把内核里的一整批字节吸进
    Python 自己的缓冲区，而随后的 ``select`` 只看 fd，方向键的三字节序列就会断在
    ESC 上。方向键有 CSI(``ESC [ A``) 和 SS3(``ESC O A``) 两种形式，都要认。
    """
    global _pending
    if not _pending and select.select([sys.stdin], [], [], timeout)[0]:
        _pending = os.read(sys.stdin.fileno(), 64)
    if not _pending:
        return None
    if _pending[:1] == b'\x1b' and _pending[1:2] in (b'[', b'O') and len(_pending) >= 3:
        name = _ARROWS.get(chr(_pending[2]), '')
        _pending = _pending[3:]
        return name
    char, _pending = _pending[:1], _pending[1:]
    return char.decode('utf-8', 'replace').lower()


def main(args=None) -> None:
    if not sys.stdin.isatty():
        raise SystemExit('操作台要在终端里跑：ros2 run g1_gmt_tracking teleop_keyboard')
    rclpy.init(args=args)
    node = GmtTeleop()
    print(HELP)
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key(0.05)
            if key is not None and not node.key(key):
                break
            node.poll()
            rclpy.spin_once(node, timeout_sec=0.0)
            for line in node.drain():
                sys.stdout.write(f'\r\033[K{line}\n')
            sys.stdout.write(node.render())
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print()
        # 退出即急停：这个终端是操作员唯一的停机入口，不能关掉了还让动作继续放。
        node.request(node.estop_client, '退出急停')
        deadline = time.monotonic() + 20.0
        while rclpy.ok() and node.pending and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        for line in node.drain():
            print(line)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
