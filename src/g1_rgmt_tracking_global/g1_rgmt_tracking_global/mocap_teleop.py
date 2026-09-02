"""戴着头显用手柄推 RGMT 的状态机。

``teleop_keyboard`` 要人守在键盘前，而跟踪的人正戴着头显、看不见屏幕。这个节点订
``g1_mocap`` 的 ``/mocap/controllers``，把「双手同时按 B/Y」翻译成 ``~/engage`` /
``~/start`` / ``~/estop`` 三个服务调用。

按键规则和 ``g1_motion_control`` 的 ``vr_teleop`` 一模一样，是那边踩出来的：

* ``engage`` / ``start`` **松手才走**，``running``（这一步本来就是急停）**按下即走**。
  都按下即走的话，从站立长按会先把策略拉起来、再急停，中间那一秒机器人已经在跑了。
* 按满 ``estop_hold_s`` **不看当前状态直接急停**。
* 一上来当作「按着」——必须真的松手再按才算一次，避免刚连上就误触。
* 按键流断掉时按「按住且已消费」处理：长按计时作废，恢复后必须真松手再按。

    ros2 run g1_rgmt_tracking_global mocap_teleop

数据源是 ``/mocap/controllers`` 而不是 ``/mocap/frame``：后者要校准完成、骨架可用才发，
而最需要急停的时刻恰恰是那些条件不成立的时刻。
"""

from __future__ import annotations

import rclpy
from g1_mocap_msgs.msg import MocapControllers
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

# RGMT 的 engage 只激活控制器，状态仍是 idle；是否已激活要由服务回执补足。
ADVANCE = {'estop': ('engage', '激活控制器'),
           'stand': ('estop', '急停'), 'running': ('estop', '急停')}


class MocapTeleop(Node):

    def __init__(self) -> None:
        super().__init__('mocap_teleop')
        p = self.declare_parameter
        namespace = p('tracking_ns', '/rgmt_tracking').get_parameter_value().string_value
        controllers_topic = p('controllers_topic', '/mocap/controllers') \
            .get_parameter_value().string_value
        self._cooldown = float(p('button_cooldown_s', 1.0)
                               .get_parameter_value().double_value)
        # 双手 B/Y 按住这么久 -> 不看当前状态直接急停。非正值会让按下即急停、短按全废。
        self._estop_hold = float(p('estop_hold_s', 1.0).get_parameter_value().double_value)
        if self._estop_hold <= 0.0:
            raise ValueError('estop_hold_s 必须为正（秒）')
        # 多久没收到按键就当断流。手柄流是 72/90 Hz，0.3 s 已经是几十帧。
        self._timeout = float(p('input_timeout_s', 0.3).get_parameter_value().double_value)

        self._trigger = {name: self.create_client(Trigger, f'{namespace}/{name}')
                         for name in ('engage', 'start', 'estop')}
        self._state = ''
        self._engaged = False
        self._pressed = False
        # 初始当作「按着」：连上之后必须真的松手再按才算一次，避免启动瞬间误触。
        self._held, self._down_at, self._used = True, None, True
        self._stamp = 0.0
        self._last_input = 0.0

        self.create_subscription(String, f'{namespace}/status', self._on_status, 10)
        self.create_subscription(MocapControllers, controllers_topic, self._on_input, 10)
        self.create_timer(0.02, self._tick)
        self.get_logger().info(
            f'手柄遥控就绪: 双手同时按 B/Y 推进状态机（松手生效，running 那步按下即走）；'
            f'按住 {self._estop_hold:.1f} s 不看状态直接急停。'
            f'数据源 {controllers_topic}，目标 {namespace}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_status(self, message: String) -> None:
        for chunk in message.data.split():
            if chunk.startswith('state='):
                self._state = chunk[len('state='):]
                if self._state == 'estop':
                    self._engaged = False
                elif self._state in ('stand', 'running'):
                    self._engaged = True
                return

    def _on_input(self, message: MocapControllers) -> None:
        self._last_input = self._now()
        self._pressed = bool(message.left.connected and message.right.connected
                             and message.left.b_y and message.right.b_y)

    def _tick(self) -> None:
        now = self._now()
        if self._last_input <= 0.0 or now - self._last_input > self._timeout:
            # 断流时长按计时作废，且恢复后必须真的松手再按，否则一次掉帧就可能凭空推一步。
            self._held, self._down_at, self._used = True, None, True
            return

        pressed = self._pressed
        down, up = pressed and not self._held, self._held and not pressed
        if down:
            self._down_at, self._used = now, False
        held_for = now - self._down_at if self._down_at is not None else 0.0
        cooled = now - self._stamp > self._cooldown

        if pressed and not self._used and held_for >= self._estop_hold:
            self._used, self._stamp = True, now
            self.get_logger().warning(f'双手长按 B/Y {held_for:.1f} s -> 直接急停')
            self._call('estop', '急停')
        elif cooled and (up and not self._used
                         # 这一步本来就是急停，没有"先把策略拉起来"的风险，按下即走
                         or down and self._next_step()[0] == 'estop'):
            self._used, self._stamp = True, now
            self._advance()
        self._held = pressed

    def _advance(self) -> None:
        name, label = self._next_step()
        if not name:
            self.get_logger().warning(
                f'收到 B/Y，但跟踪层状态是「{self._state or "未知"}」，先确认它起来了')
            return
        self.get_logger().info(f'B/Y -> {label}')
        self._call(name, label)

    def _next_step(self) -> tuple[str, str]:
        if self._state == 'idle':
            return ('start', '启动策略') if self._engaged \
                else ('engage', '激活控制器')
        return ADVANCE.get(self._state, ('', ''))

    def _call(self, name: str, label: str) -> None:
        client = self._trigger[name]
        if not client.service_is_ready():
            self.get_logger().warning(f'{label}：服务 {client.srv_name} 还没就绪')
            return
        client.call_async(Trigger.Request()).add_done_callback(
            lambda future: self._report(name, label, future))

    def _report(self, name: str, label: str, future) -> None:
        result = future.result()
        if result is None:
            self.get_logger().error(f'{label}：服务没有返回结果')
        elif not result.success:
            self.get_logger().warning(f'{label}被拒绝：{result.message}')
        else:
            if name == 'engage':
                self._engaged = True
            elif name == 'estop':
                self._engaged = False
            self.get_logger().info(f'{label}：{result.message}')


def main() -> None:
    rclpy.init()
    node = MocapTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
