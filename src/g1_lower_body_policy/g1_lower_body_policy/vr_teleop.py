#!/usr/bin/env python3
"""VR 头显遥操作桥：WebXR 帧 -> lower_body_policy 的 ``~/command``。

    头显 / 双手柄 --WebXR--> VR/server.py --WS--> [ 本节点 50 Hz ] --20 值--> lower_body_policy

先按 ``VR/README.md`` 把三步跑通（adb reverse + server.py + 头显里打开采集页并
Enter VR），再起本节点。

映射
----
* **状态机**：**两只手同时**按 B/Y（右手 B / 左手 Y）推进一步，代替键盘的
  ``G`` / ``Enter`` / 空格。戴着头显摸不到终端，所以这三个服务得能从手柄调：

      idle / estop --按一次--> 站立(stand) --再按--> 策略接管(running) --再按--> 急停

  要两只手一起按是为了防误触。派发看的是策略层**当前上报的状态**而不是本地计数，
  所以别人用 ``ros2 service call`` 插手之后不会错位。
* **下肢**：**左摇杆**管水平速度 ``(vx, vy)``，**右摇杆**管转向 ``wz`` 与盆骨高度。
  限幅和语义都跟 ``teleop_keyboard.py`` 一致：速度松手归零，**高度是绝对量**（推着才变，
  松手停在当前值、不回弹）。
  xr-standard 的摇杆是 ``axes[2]=X``（右为正）、``axes[3]=Y``（**下**为正），而机器人是
  X 前 / Y 左 / 逆时针为正 / 上为正，所以四个轴**全要取负号**。
* **上肢**：``squeeze`` 是离合。按下的瞬间同时锁住「手柄位置」「手柄姿态」「机器人
  当前末端位姿」三个原点，之后下发的是 ``末端原点 + 手柄位移`` 与 ``手柄转角 × 末端原姿态``
  —— 全程都是**相对量**，从来不是绝对位姿。松开就冻结在最后一帧。
* **标定**：任一手柄按 **A/X** 把它**此刻所指的水平方向定为机器人正前方**。
  不标定时默认用 WebXR 参考空间的 -Z 当正前方，你一转身手往“前”推就不是机器人的前了；
  标定只取**航向（绕重力轴）**，不改上下，所以手往上抬永远是末端往上。
* **夹爪**：``trigger`` 直接映射，**0 = 完全打开、1 = 夹紧**。注意 ``eccentric_joint``
  是 **0 rad 闭合、2.76377 rad 打开**（见 unitree_g1_description/model/Gloria-M/README.md），
  所以这里是**反向**映射；写反了就是该松手的时候夹紧。

坐标系：WebXR 右手系、Y 向上、-Z 朝前；机器人 X 前、Y 左、Z 上。

安全
----
* 只在策略层进入 ``running`` 之后才发指令；进入那一刻用实测位形正解播种双臂位姿，
  和策略层自己的播种对齐。
* VR 帧超时（``frame_timeout_s``）或退出 VR 会话：速度立刻归零、双臂冻结、高度保持。
* 离合按下瞬间位移与转角都恒为 0，所以接管不会跳——上肢不限速，这是唯一的防跳保护。
"""

from __future__ import annotations

import asyncio
import json
import math
import threading

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from g1_lower_body_policy.arm_ik import ArmIK

SIDES = ('left', 'right')
# 策略层当前状态 -> 双手 B/Y 按下时该调哪个服务。看实际状态而不是本地计数，
# 别人用 ros2 service call 插一手之后不会错位。
_ADVANCE = {'idle': ('engage', '站立'), 'estop': ('engage', '站立'),
            'stand': ('start', '启动策略'), 'running': ('estop', '急停')}
# WebXR 参考空间 (x 右, y 上, -z 前) -> 机器人 (x 前, y 左, z 上)。正交且 det=+1，
# 所以拿它做共轭可以把世界系里的旋转原样搬到机器人轴系，转角不变。
_BASE_MAP = np.array([[0.0, 0.0, -1.0],
                      [-1.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0]])


def _matrix(quat_xyzw) -> np.ndarray:
    return pin.Quaternion(np.asarray(quat_xyzw, dtype=np.float64)).matrix()


def _quat(matrix: np.ndarray) -> np.ndarray:
    coeffs = np.asarray(pin.Quaternion(matrix).coeffs(), dtype=np.float64)
    return coeffs / np.linalg.norm(coeffs)


class VRTeleop(Node):

    def __init__(self) -> None:
        super().__init__('vr_teleop')
        p = self.declare_parameter
        self._url = p('vr_url', 'ws://localhost:8000/ws/subscribe') \
            .get_parameter_value().string_value
        self._rate = float(p('rate_hz', 50.0).get_parameter_value().double_value)
        # 限幅与 teleop_keyboard.py 同一组。高度上界取 0.78 而不是键盘的 0.80：
        # 策略层 command_limits 就是裁到 0.78，写 0.80 只会让摇杆顶端摸起来像卡住了。
        self._vx_max = float(p('vx_max', 0.5).get_parameter_value().double_value)
        self._vy_max = float(p('vy_max', 0.4).get_parameter_value().double_value)
        self._wz_max = float(p('wz_max', 1.5).get_parameter_value().double_value)
        self._height0 = float(p('height', 0.78).get_parameter_value().double_value)
        self._h_lo = float(p('height_min', 0.50).get_parameter_value().double_value)
        self._h_hi = float(p('height_max', 0.78).get_parameter_value().double_value)
        # 摇杆推到底时的高度变化率。策略层自己也按 0.15 m/s 限速（训练里高度指令
        # 就是这个速率缓变），所以这里取同值，再大也过不去。
        self._height_rate = float(
            p('height_rate', 0.15).get_parameter_value().double_value)
        # 死区：摇杆回中会残留零点漂移，不处理机器人会一直慢慢走。
        self._deadzone = float(
            p('stick_deadzone', 0.08).get_parameter_value().double_value)
        # squeeze 是模拟量，按到底才是 1.0；用 0.5 判定"按下"，怕误触就往上调。
        self._squeeze_on = float(
            p('squeeze_threshold', 0.5).get_parameter_value().double_value)
        self._arm_scale = float(
            p('arm_scale', 1.0).get_parameter_value().double_value)
        self._timeout = float(
            p('frame_timeout_s', 0.3).get_parameter_value().double_value)
        self._grip_open = float(
            p('gripper_open', 2.76377472169236).get_parameter_value().double_value)
        self._grip_closed = float(
            p('gripper_closed', 0.0).get_parameter_value().double_value)
        # 双手 B/Y 两次推进之间的最小间隔，防按键抖动连发。
        self._button_cooldown = float(
            p('button_cooldown_s', 1.0).get_parameter_value().double_value)

        self._arm_names = list(
            p('arm_joints', ['']).get_parameter_value().string_array_value)
        self._grip_names = list(
            p('gripper_joints', ['']).get_parameter_value().string_array_value)
        if len(self._arm_names) != 14 or len(self._grip_names) != 2:
            raise ValueError('arm_joints 需要 14 个、gripper_joints 需要 2 个关节名')
        self._ik_kwargs = dict(
            tip_frames={side: p(f'{side}_tip_frame', f'{side}_gripper_base')
                        .get_parameter_value().string_value for side in SIDES},
            base_frame=p('base_frame', 'torso_link')
            .get_parameter_value().string_value,
        )

        self._lock = threading.Lock()
        self._frame: dict | None = None
        self._frame_stamp = 0.0
        self._ik: ArmIK | None = None
        self._arm_q: np.ndarray | None = None
        self._grip_meas = np.zeros(2)
        self._state = ''
        self._js_names: list[str] = []
        self._js_index: list[int] = []
        self._grip_index: list[int] = []
        # 下面这几个只在控制线程里读写，不需要加锁。
        self._seeded = False
        self._pose: dict[str, np.ndarray] = {}
        self._grip = np.zeros(2)
        self._clutch: dict[str, tuple | None] = {side: None for side in SIDES}
        self._twist = np.zeros(3)
        self._height = self._height0
        # 初始当作"按着"：连上之后必须真的松手再按才算一次，避免启动瞬间误触。
        self._button_held = True
        self._button_stamp = 0.0
        # 世界系 -> 机器人轴系的映射，按 A/X 重新标定。标定不随重新接管重置。
        self._map = _BASE_MAP.copy()
        self._calib_held = {side: True for side in SIDES}

        stream = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        control = MutuallyExclusiveCallbackGroup()
        slow = ReentrantCallbackGroup()
        self._message = Float64MultiArray()
        self._publisher = self.create_publisher(
            Float64MultiArray,
            p('command_topic', '/lower_body_policy/command')
            .get_parameter_value().string_value, stream)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states,
                                 stream, callback_group=control)
        self.create_subscription(
            String, p('status_topic', '/lower_body_policy/status')
            .get_parameter_value().string_value, self._on_status, 10,
            callback_group=control)
        # 建模要几百毫秒，避开控制环所在的互斥组。
        self.create_subscription(
            String, p('robot_description_topic', '/robot_description')
            .get_parameter_value().string_value,
            self._on_description, latched, callback_group=slow)
        self.create_timer(1.0 / self._rate, self._tick, callback_group=control)
        # 服务得用 Reentrant 组 + call_async：engage/estop 里的 switch_controller 会阻塞
        # 好几秒（硬件卸力斜坡），同步等会把 50 Hz 定时器一起冻住。
        policy = p('policy_node', '/lower_body_policy') \
            .get_parameter_value().string_value.rstrip('/')
        self._trigger = {
            name: self.create_client(Trigger, f'{policy}/{name}',
                                     callback_group=slow)
            for name in ('engage', 'start', 'estop')}

        self._alive = True
        self._thread = threading.Thread(target=self._stream, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f'等待 VR 帧（{self._url}）。双手同时按 B/Y 推进：站立 -> 启动策略 -> 急停；'
            f'A/X 把手柄所指方向标定为机器人正前方；'
            f'squeeze 按住才跟随手，trigger 控夹爪（0 开 / 1 夹）。')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -- 数据源 ----------------------------------------------------------------

    def _stream(self) -> None:
        """后台线程收 WS，只往 _frame 里放最新一帧；断了就重连。"""
        import aiohttp

        async def run() -> None:
            while self._alive:
                try:
                    async with aiohttp.ClientSession() as session:
                        async with session.ws_connect(self._url, heartbeat=5.0) as ws:
                            self.get_logger().info(f'已连上 VR 桥 {self._url}')
                            async for message in ws:
                                if message.type is not aiohttp.WSMsgType.TEXT:
                                    continue
                                with self._lock:
                                    self._frame = json.loads(message.data)
                                    self._frame_stamp = self._now()
                except Exception as error:  # 网络/解析全按断线处理，重连即可。
                    self.get_logger().warning(
                        f'VR 桥断开，1 s 后重连: {error}', throttle_duration_sec=5.0)
                if self._alive:
                    await asyncio.sleep(1.0)

        asyncio.run(run())

    def _on_description(self, message: String) -> None:
        if self._ik is not None:
            return
        try:
            ik = ArmIK(message.data, self._arm_names, **self._ik_kwargs)
        except Exception as error:
            self.get_logger().error(f'正解模型建立失败，上肢不可用: {error}')
            return
        with self._lock:
            self._ik = ik
        self.get_logger().info('正解模型就绪')

    def _on_joint_states(self, message: JointState) -> None:
        ik = self._ik
        if ik is None:
            return
        names = list(message.name)
        if names != self._js_names:
            wanted = list(ik.joint_names) + self._grip_names
            if any(name not in names for name in wanted):
                return  # 广播还没收齐，等下一帧。
            self._js_names = names
            self._js_index = [names.index(name) for name in ik.joint_names]
            self._grip_index = [names.index(name) for name in self._grip_names]
        if len(message.position) != len(names):
            return
        position = np.asarray(message.position)
        with self._lock:
            self._arm_q = position[self._js_index]
            self._grip_meas = position[self._grip_index]

    def _on_status(self, message: String) -> None:
        try:
            state = json.loads(message.data).get('state', '')
        except ValueError:
            return
        with self._lock:
            self._state = state

    # -- 控制环 ----------------------------------------------------------------

    def _tick(self) -> None:
        with self._lock:
            frame, stamp = self._frame, self._frame_stamp
            ik, arm_q, grip_meas = self._ik, self._arm_q, self._grip_meas.copy()
            state = self._state
        fresh = (frame is not None and frame.get('session_active')
                 and self._now() - stamp <= self._timeout)
        # 状态机按键和标定任何状态下都要响应——idle / stand 里还没开始发指令就得能按。
        self._check_button(frame if fresh else None)
        self._check_calibration(frame if fresh else None)
        if ik is None or arm_q is None or state != 'running':
            self._seeded = False        # 退出 running 后重新播种，不留旧目标。
            return
        if not self._seeded:
            # 和策略层 ~/start 用的是同一份正解、同一份实测位形，两边目标天然对齐。
            self._pose = ik.fk(arm_q)
            self._grip = grip_meas
            self._clutch = {side: None for side in SIDES}
            self._twist = np.zeros(3)
            self._height = self._height0
            self._seeded = True
            self.get_logger().info('策略层已 running，双臂目标已按当前位形播种')

        if not fresh:
            # 掉帧/退出 VR：立刻停车，手臂、夹爪和高度都冻结在最后一帧。
            self._twist = np.zeros(3)
            self.get_logger().warning('VR 帧不新鲜，已停车并冻结上肢',
                                      throttle_duration_sec=2.0)
        else:
            self._update_command(frame)
            self._update_arms(frame)

        self._message.data = [
            float(self._twist[0]), float(self._twist[1]), float(self._twist[2]),
            self._height,
            *(float(v) for v in self._pose['left']),
            *(float(v) for v in self._pose['right']),
            float(self._grip[0]), float(self._grip[1]),
        ]
        self._publisher.publish(self._message)

    def _check_button(self, frame) -> None:
        """双手同时按 B/Y 的上升沿：推进状态机一步。

        帧不新鲜时按"按住"处理——恢复之后必须真的松手再按才算一次，否则一次掉帧
        就可能凭空触发一步。
        """
        if frame is None:
            self._button_held = True
            return
        pressed = all(((frame.get(side) or {}).get('buttons') or {}).get('b_y')
                      for side in SIDES)
        if (pressed and not self._button_held
                and self._now() - self._button_stamp > self._button_cooldown):
            self._button_stamp = self._now()
            self._advance()
        self._button_held = pressed

    def _advance(self) -> None:
        with self._lock:
            state = self._state
        name, label = _ADVANCE.get(state, (None, ''))
        if name is None:
            self.get_logger().warning(
                f'收到 B/Y，但策略层状态是「{state or "未知"}」，先确认它起来了')
            return
        client = self._trigger[name]
        if not client.service_is_ready():
            self.get_logger().warning(f'{label}：服务 {name} 还没就绪')
            return
        self.get_logger().info(f'B/Y -> {label}')
        client.call_async(Trigger.Request()).add_done_callback(
            lambda future: self._report(label, future))

    def _report(self, label: str, future) -> None:
        result = future.result()
        if result is None:
            self.get_logger().error(f'{label}：调用失败')
        elif result.success:
            self.get_logger().info(f'{label}：{result.message}')
        else:
            # 最常见的是"站立插值还没走完"——等满 stand_s 再按一次即可。
            self.get_logger().warning(f'{label}被拒绝：{result.message}')

    def _check_calibration(self, frame) -> None:
        """任一手柄按 A/X 的上升沿：用它重新标定世界系 -> 机器人轴系的映射。"""
        if frame is None:
            self._calib_held = {side: True for side in SIDES}
            return
        for side in SIDES:
            hand = frame.get(side) or {}
            pressed = bool((hand.get('buttons') or {}).get('a_x'))
            grip = hand.get('grip')
            if pressed and not self._calib_held[side] and grip:
                self._calibrate(side, grip['orientation'])
            self._calib_held[side] = pressed

    def _calibrate(self, side: str, orientation) -> None:
        """把手柄此刻所指的**水平**方向定为机器人正前方。

        只取航向（绕 WebXR 的 +Y，也就是重力轴），不把手柄的俯仰/横滚带进来：
        带进来的话标定时手柄稍微一歪，“往上抬手”就不是“末端往上”了。
        """
        forward = _matrix(orientation) @ np.array([0.0, 0.0, -1.0])
        if abs(forward[0]) < 1e-6 and abs(forward[2]) < 1e-6:
            self.get_logger().warning('手柄几乎竖直，定不出水平朝向，标定已忽略')
            return
        yaw = math.atan2(-forward[0], -forward[2])
        cos, sin = math.cos(yaw), math.sin(yaw)
        # R_y(yaw) 的转置：把“手柄指的方向”转回参考空间的 -Z，再走固定映射。
        self._map = _BASE_MAP @ np.array([[cos, 0.0, -sin],
                                          [0.0, 1.0, 0.0],
                                          [sin, 0.0, cos]])
        # 已接合的离合必须作废：新映射下旧原点算出来的位移会瞬间变向，不重新接合就是一下跳。
        self._clutch = {name: None for name in SIDES}
        self.get_logger().info(
            f'{side} 手柄标定：航向 {math.degrees(yaw):+.0f}° 定为机器人正前方，'
            f'离合已全部断开，重新按 squeeze 接管')

    def _stick(self, frame: dict, side: str) -> np.ndarray:
        """取一个摇杆的 ``[x, y]``，已去死区。"""
        axes = np.asarray((frame.get(side) or {}).get('thumbstick') or (0.0, 0.0),
                          dtype=np.float64)
        if axes.shape != (2,) or not np.all(np.isfinite(axes)):
            return np.zeros(2)
        magnitude = float(np.linalg.norm(axes))
        if magnitude <= self._deadzone:
            return np.zeros(2)
        # 径向死区 + 出区后重新归一化，否则刚越过死区那一下速度是阶跃的。
        return axes * ((magnitude - self._deadzone)
                       / (1.0 - self._deadzone) / magnitude)

    def _update_command(self, frame: dict) -> None:
        """左摇杆 -> 水平速度，右摇杆 -> 转向与高度。

        xr-standard 的 Y 轴是**下为正**，而机器人 X 前 / Y 左 / 逆时针为正，
        所以四个轴全取负号。
        """
        left, right = self._stick(frame, 'left'), self._stick(frame, 'right')
        self._twist = np.array([-left[1] * self._vx_max,
                                -left[0] * self._vy_max,
                                -right[0] * self._wz_max])
        # 高度和键盘的 I/K 一样是绝对量：推着才变，松手停在当前值，不回弹。
        self._height = min(max(
            self._height - right[1] * self._height_rate / self._rate,
            self._h_lo), self._h_hi)

    def _update_arms(self, frame: dict) -> None:
        for index, side in enumerate(SIDES):
            hand = frame.get(side) or {}
            buttons = hand.get('buttons') or {}
            grip = hand.get('grip')
            squeeze = float(buttons.get('squeeze', 0.0))
            if grip and squeeze >= self._squeeze_on:
                position = np.asarray(grip['position'], dtype=np.float64)
                rotation = _matrix(grip['orientation'])
                if self._clutch[side] is None:
                    # 三个原点一起锁：接管瞬间位移和转角都恒为 0，所以不会跳。
                    self._clutch[side] = (position, rotation, self._pose[side].copy())
                    self.get_logger().info(f'{side} 离合接合')
                origin, origin_rot, anchor = self._clutch[side]
                pose = self._pose[side]
                pose[:3] = anchor[:3] + self._arm_scale * (
                    self._map @ (position - origin))
                # 手转了多少，末端就转多少：世界系的相对旋转共轭到机器人轴系再左乘。
                # _map 正交且 det=+1，所以共轭出来仍是旋转、转角不变。
                turn = self._map @ rotation @ origin_rot.T @ self._map.T
                pose[3:] = _quat(turn @ _matrix(anchor[3:]))
            elif self._clutch[side] is not None:
                self._clutch[side] = None       # 松开就冻结在最后一帧。
                self.get_logger().info(f'{side} 离合断开，手臂冻结')
            # trigger: 0 -> 完全打开、1 -> 夹紧。eccentric 是 0 闭合、2.76 打开，反向。
            trigger = min(max(float(buttons.get('trigger', 0.0)), 0.0), 1.0)
            self._grip[index] = self._grip_open + \
                (self._grip_closed - self._grip_open) * trigger

    def shutdown(self) -> None:
        self._alive = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VRTeleop()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
