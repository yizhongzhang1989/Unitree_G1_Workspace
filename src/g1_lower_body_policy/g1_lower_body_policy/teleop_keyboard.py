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
import select
import sys
import termios
import time
import tty

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
  <- / ->   左转 / 右转      (wz)
  Up / Dn   升高 / 蹲低      (h)
  X         速度指令清零（高度保持）
  Q         退出（退出前自动急停）
"""

# 方向键是 ESC [ A/B/C/D 三字节序列。
ARROWS = {'A': 'up', 'B': 'down', 'C': 'right', 'D': 'left'}


class Teleop(Node):

    def __init__(self) -> None:
        super().__init__('lower_body_teleop')
        p = self.declare_parameter
        target = p('policy_node', '/lower_body_policy') \
            .get_parameter_value().string_value.rstrip('/')
        self.rate = float(p('publish_rate_hz', 50.0).get_parameter_value().double_value)
        self._step = (
            float(p('vx_step', 0.05).get_parameter_value().double_value),
            float(p('vy_step', 0.05).get_parameter_value().double_value),
            float(p('wz_step', 0.10).get_parameter_value().double_value),
        )
        self._limit = (
            float(p('vx_max', 0.5).get_parameter_value().double_value),
            float(p('vy_max', 0.3).get_parameter_value().double_value),
            float(p('wz_max', 0.5).get_parameter_value().double_value),
        )
        self._h_step = float(p('height_step', 0.01).get_parameter_value().double_value)
        self._h_range = (
            float(p('height_min', 0.62).get_parameter_value().double_value),
            float(p('height_max', 0.76).get_parameter_value().double_value),
        )
        self._height = min(max(
            float(p('initial_height', 0.74).get_parameter_value().double_value),
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
                'left': (2, 1), 'right': (2, -1)}.get(name)
        if axis is not None:
            index, sign = axis
            value = self._vel[index] + sign * self._step[index]
            self._vel[index] = min(max(value, -self._limit[index]), self._limit[index])
        elif name == 'up':
            self._height = min(self._height + self._h_step, self._h_range[1])
        elif name == 'down':
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

    def render(self) -> str:
        vx, vy, wz = self._vel
        return (f'\r\033[Kvx {vx:+.2f}  vy {vy:+.2f}  w {wz:+.2f}  h {self._height:.3f}'
                f'  | {self._status}  | {self.notice}')

    def estop_client(self):
        return self._estop_client


def read_key(timeout: float) -> str | None:
    """读一个按键。方向键返回 'up'/'down'/'left'/'right'。"""
    if not select.select([sys.stdin], [], [], timeout)[0]:
        return None
    char = sys.stdin.read(1)
    if char != '\x1b':
        return char.lower()
    # ESC 序列的后续字节已经在缓冲里，不会阻塞。
    if not select.select([sys.stdin], [], [], 0.01)[0]:
        return '\x1b'
    if sys.stdin.read(1) != '[':
        return '\x1b'
    return ARROWS.get(sys.stdin.read(1), '\x1b')


def main(args=None) -> None:
    if not sys.stdin.isatty():
        raise SystemExit(
            '遥控台要在终端里跑：ros2 run g1_lower_body_policy teleop_keyboard')
    rclpy.init(args=args)
    node = Teleop()
    dt = 1.0 / node.rate
    print(HELP)
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key(dt)
            if key is not None and not node.key(key):
                break
            node.tick(dt)
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
        node.request(node.estop_client(), '退出急停')
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
