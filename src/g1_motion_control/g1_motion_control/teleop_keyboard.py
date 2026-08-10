#!/usr/bin/env python3
"""下肢策略层的键盘遥控台。

速度是"按住才走"：终端读不到抬键事件，所以超过 ``hold_timeout_s`` 没有按键就
衰减到零。按住 W 时终端的自动重复会持续刷新，手感和游戏一致。高度不衰减——它是
绝对量，调到哪停在哪。

指令发到 ``<policy_node>/command``（``[vx, vy, wz, h]``）。这里只做终端侧的粗
限幅；真正对得上训练分布的限幅、限速在策略节点里。
"""

from __future__ import annotations

import json
import os
import select
import shutil
import sys
import termios
import time
import tty
import unicodedata

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

HELP = """\
  G         站立：激活 forward_position_controller，插值到默认位姿
  Enter     启动策略：站立走完后按，策略正式接管
  空格      急停：卸力到阻尼模式再零力矩，之后要从 G 重来

  W / S     前进 / 后退      (vx)
  A / D     左移 / 右移      (vy)
  J / L     左转 / 右转      (wz)
  I / K     升高 / 蹲低      (h)
  X         速度指令清零（高度保持）
  Q         退出（退出前自动急停）
"""


class Teleop(Node):

    def __init__(self) -> None:
        super().__init__('lower_body_teleop')
        p = self.declare_parameter
        target = p('policy_node', '/motion_control') \
            .get_parameter_value().string_value.rstrip('/')
        self.rate = float(p('publish_rate_hz', 50.0).get_parameter_value().double_value)
        self._step = (
            float(p('vx_step', 0.1).get_parameter_value().double_value),
            float(p('vy_step', 0.1).get_parameter_value().double_value),
            float(p('wz_step', 0.1).get_parameter_value().double_value),
        )
        self._limit = (
            float(p('vx_max', 0.8).get_parameter_value().double_value),
            float(p('vy_max', 0.4).get_parameter_value().double_value),
            float(p('wz_max', 1.5).get_parameter_value().double_value),
        )
        self._h_step = float(p('height_step', 0.01).get_parameter_value().double_value)
        self._h_range = (
            float(p('height_min', 0.50).get_parameter_value().double_value),
            float(p('height_max', 0.80).get_parameter_value().double_value),
        )
        self._height = min(max(
            float(p('initial_height', 0.76).get_parameter_value().double_value),
            self._h_range[0]), self._h_range[1])
        self._hold_timeout = float(
            p('hold_timeout_s', 0.4).get_parameter_value().double_value)
        self._decay_s = float(p('decay_s', 0.5).get_parameter_value().double_value)

        stream = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.BEST_EFFORT)
        self._publisher = self.create_publisher(
            Float64MultiArray, f'{target}/command', stream)
        self.create_subscription(String, f'{target}/status', self._on_status, 10)
        self._engage_client = self.create_client(Trigger, f'{target}/engage')
        self._start_client = self.create_client(Trigger, f'{target}/start')
        self._estop_client = self.create_client(Trigger, f'{target}/estop')

        self._vel = [0.0, 0.0, 0.0]
        self._idle = 0.0
        self._status = '(等待策略节点状态)'
        self.notice = ''
        self._message = Float64MultiArray()

    # -- 输入 ------------------------------------------------------------------

    def key(self, name: str) -> bool:
        """处理一个按键，返回 False 表示要退出。"""
        self._idle = 0.0
        axis = {'w': (0, 1), 's': (0, -1), 'a': (1, 1), 'd': (1, -1),
                'j': (2, 1), 'l': (2, -1)}.get(name)
        if axis is not None:
            index, sign = axis
            value = self._vel[index] + sign * self._step[index]
            self._vel[index] = min(max(value, -self._limit[index]), self._limit[index])
        elif name == 'i':
            self._height = min(self._height + self._h_step, self._h_range[1])
        elif name == 'k':
            self._height = max(self._height - self._h_step, self._h_range[0])
        elif name == ' ':
            self._vel = [0.0, 0.0, 0.0]
            self.request(self._estop_client, '急停')
        elif name == 'g':
            self.request(self._engage_client, '站立')
        elif name in ('\r', '\n'):
            self._vel = [0.0, 0.0, 0.0]
            self.request(self._start_client, '启动策略')
        elif name == 'x':
            self._vel = [0.0, 0.0, 0.0]
        elif name == 'q':
            return False
        else:
            self._idle = self._hold_timeout  # 无关按键不算"还按着"。
        return True

    def request(self, client, label: str) -> None:
        if not client.service_is_ready():
            self.notice = f'{label}失败：策略节点服务不在'
            return
        self.notice = f'{label}请求已发出…'
        client.call_async(Trigger.Request()).add_done_callback(
            lambda future: self._on_result(label, future))

    def _on_result(self, label: str, future) -> None:
        result = future.result()
        self.notice = (f'{label}: {result.message}' if result is not None
                       else f'{label}失败：无响应')

    def _on_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except ValueError:
            self._status = message.data
            return
        state = status.get('state', '?')
        if state == 'stand':
            state += ' (可以 Enter)' if status.get('ready_to_start') else ' (插值中)'
        detail = status.get('reason') or status.get('stale') or ''
        self._status = f'{state}{" ⚠ " + detail if detail else ""}'

    # -- 输出 ------------------------------------------------------------------

    def tick(self, dt: float) -> None:
        """衰减 + 发指令。检测不到抬键，所以用超时代替。"""
        self._idle += dt
        if self._idle > self._hold_timeout:
            keep = max(0.0, 1.0 - dt / max(self._decay_s, 1e-3))
            self._vel = [0.0 if abs(v) < 1e-3 else v * keep for v in self._vel]
        self._message.data = [*self._vel, self._height]
        self._publisher.publish(self._message)

    def stop(self) -> None:
        self._vel = [0.0, 0.0, 0.0]

    def estop(self, label: str) -> None:
        self.request(self._estop_client, label)

    def render(self) -> str:
        vx, vy, wz = self._vel
        line = (f'vx{vx:+.2f} vy{vy:+.2f} w{wz:+.2f} h{self._height:.3f}'
                f' | {self._status} | {self.notice}')
        return f'\r\033[K{_clip(line, shutil.get_terminal_size().columns - 1)}'


def _clip(text: str, columns: int) -> str:
    """按显示宽度截断。状态行一旦溢出终端宽度就会折行，``\\r`` 只能回到最后
    一行的行首，改不掉上一行——于是每刷新一帧屏幕就往下滚一行。不确定宽度的
    （east_asian_width 为 A）一律当 2 列算，宁可多截也不能折行。
    """
    used = 0
    for index, char in enumerate(text):
        used += 2 if unicodedata.east_asian_width(char) in 'WFA' else 1
        if used > columns:
            return text[:index]
    return text


_pending = b''


def read_key(timeout: float) -> str | None:
    """读一个按键。

    走 ``os.read`` 而不是 ``sys.stdin.read(1)``：后者会把内核里的一整批字节吸进
    Python 自己的缓冲区，而随后的 ``select`` 只看 fd，没取走的字节就卡在那儿了。
    """
    global _pending
    if not _pending and select.select([sys.stdin], [], [], timeout)[0]:
        _pending = os.read(sys.stdin.fileno(), 64)
    if not _pending:
        return None
    char, _pending = _pending[:1], _pending[1:]
    return char.decode('utf-8', 'replace').lower()


def main(args=None) -> None:
    if not sys.stdin.isatty():
        raise SystemExit(
            '遥控台要在终端里跑：ros2 run g1_motion_control teleop_keyboard')
    rclpy.init(args=args)
    node = Teleop()
    dt = 1.0 / node.rate
    print(HELP)
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        last = time.monotonic()
        while rclpy.ok():
            key = read_key(dt)
            if key is not None and not node.key(key):
                break
            # 缓冲里还有字节时 read_key 会立刻返回，节拍不再等于 dt，按实测走。
            now = time.monotonic()
            node.tick(now - last)
            last = now
            rclpy.spin_once(node, timeout_sec=0.0)
            sys.stdout.write(node.render())
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print()
        # 退出即急停：终端一关就没人再发指令，机器人不能停在最后一帧目标上。
        node.stop()
        node.tick(dt)
        node.estop('退出急停')
        deadline = time.monotonic() + 20.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.notice.startswith('退出急停: '):
                break
        print(node.notice)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
