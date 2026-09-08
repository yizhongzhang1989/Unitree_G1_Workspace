#!/usr/bin/env python3
"""VLA 手动单块终端：回车执行一段，输入文字更新任务。"""

from __future__ import annotations

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


def input_action(line: str) -> tuple[str, str]:
    """将一行输入分成执行、内置命令或任务文字。"""
    value = line.strip()
    if not value:
        return 'next', ''
    if value.startswith('/'):
        return 'command', value
    return 'task', value


class VlaCli(Node):

    def __init__(self) -> None:
        super().__init__('vla_cli')
        self._task_publisher = self.create_publisher(String, '/vla_bridge/task', 10)
        self._start = self.create_client(Trigger, '/vla_bridge/start')
        self._next = self.create_client(Trigger, '/vla_bridge/next')
        self._stop = self.create_client(Trigger, '/vla_bridge/stop')
        self._engage = self.create_client(Trigger, '/motion_control/engage')
        self._estop = self.create_client(Trigger, '/motion_control/estop')
        self._status = {}
        self.create_subscription(String, '/vla_bridge/status', self._on_status, 1)

    def _on_status(self, message: String) -> None:
        try:
            self._status = json.loads(message.data)
        except ValueError:
            pass

    def wait_ready(self, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if all((self._start.service_is_ready(), self._next.service_is_ready(),
                    bool(self._status))):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def call(self, client, label: str) -> bool:
        if not client.wait_for_service(timeout_sec=1.0):
            print(f'{label}失败：服务不可用')
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=35.0)
        response = future.result()
        if response is None:
            print(f'{label}失败：调用超时')
            return False
        print(f'{label}{"成功" if response.success else "拒绝"}：{response.message}')
        return bool(response.success)

    def set_task(self, task: str) -> None:
        self._task_publisher.publish(String(data=task))
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._status.get('task') == task:
                print(f'目标已更新：{task}')
                return
        print(f'目标已发送，尚未收到确认：{task}')

    def execute_one(self) -> None:
        self._status = {}
        deadline = time.monotonic() + 2.0
        while not self._status and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not self._status:
            print('执行拒绝：收不到 bridge 状态')
            return
        if self._status.get('execution_mode') != 'manual':
            print('执行拒绝：CLI 单段执行需要 execution_mode=manual')
            return
        if not self._status.get('running') and not self.call(self._start, '待命'):
            return
        self.call(self._next, '执行')


def main(args=None) -> None:
    rclpy.init(args=args)
    cli = VlaCli()
    try:
        if not cli.wait_ready():
            raise RuntimeError('找不到 /vla_bridge/start 或 /vla_bridge/next，请先启动 vla_bridge')
        print('直接 Enter：请求并完整执行一个 chunk')
        print('输入文字：更新任务    /engage：使能    /estop：急停卸力')
        print('/start：进入待命    /stop：停止 VLA    /quit：退出 CLI')
        while rclpy.ok():
            try:
                line = input('vla> ')
            except EOFError:
                break
            action, value = input_action(line)
            if action == 'next':
                cli.execute_one()
            elif value == '/engage':
                cli.call(cli._engage, '使能')
            elif value == '/estop':
                cli.call(cli._estop, '急停')
            elif value == '/start':
                cli.call(cli._start, '启动')
            elif value == '/stop':
                cli.call(cli._stop, '停止')
            elif value == '/quit':
                break
            elif action == 'command':
                print(f'未知命令：{value}')
            else:
                cli.set_task(value)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        print(error, file=sys.stderr)
    finally:
        cli.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
