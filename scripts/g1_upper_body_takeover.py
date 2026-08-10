#!/usr/bin/env python3
"""G1 上半身接管试验脚本：腿交给内置运控，腰 + 双臂走 /arm_sdk 自己控。

启动把机器人带到运控模式，接管上肢并保持 UPPER_BODY 里的固定姿态，
Ctrl+C 退出时先交还上肢再卸力。

上肢参数全部写死在 UPPER_BODY 里，运行期间不变。`motor_cmd[29].q` 是 arm_sdk 的
全局接管权重：**WEIGHT = 0 时上肢完全归内置运控程序，这里填的 q/kp/kd 会被固件
直接忽略**；WEIGHT = 1 才是本脚本接管。

FSM 过渡踩过的四个坑：
1. `SetFsmId` 返回 status=0 只代表请求被收到，不代表会执行。必须回读 `GetFsmId` 确认。
2. 命令值和回读值可能不同。 `SetFsmId(801)` 之后 `GetFsmId` 回读的是 **802**。
   官网那句「29dof 设备 ai_sport 8.6.x.x 版本后更新为 802」指的是回读编号变了，
   命令仍然收 801。所以判定成功必须用「回读 ∈ 集合」，不能用「回读 == 命令值」。
3. `fsm_id` 是立即翻转的寄存器，**不代表动作做完了**。实测预备动作期间
   `fsm_mode` = 1（动态）持续约 5 s，此时大多数模式切换会被禁止。
4. **退出必须先经阻尼。** 从承重站姿直接发 FSM 0 会被静默拒绝，机器人根本没卸力。
   官方明确：「阻尼模式作为最终的保底模式，始终能够被切换」。

用法：
    source scripts/env.sh
    python3 scripts/g1_upper_body_takeover.py

进入运控后可用键盘遥控下半身（上肢始终由本脚本保持 UPPER_BODY 姿态）：
    W / S    前进 / 后退
    A / D    左转 / 右转
    空格     急停（速度归零）

警告：预备会让机器人从当前姿态站起来，接管会让上肢走向 UPPER_BODY 目标姿态，
退出的零力矩会让它直接瘫倒。跑之前确保机器人被吊起或有人扶住。
"""

import json
import os
import select
import signal
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from unitree_api.msg import Request, Response
from unitree_hg.msg import LowCmd, LowState

SPORT_REQUEST_TOPIC = "/api/sport/request"
SPORT_RESPONSE_TOPIC = "/api/sport/response"
ARM_SDK_TOPIC = "/arm_sdk"
LOWSTATE_TOPIC = "/lowstate"
GET_FSM_ID = 7001
GET_FSM_MODE = 7002
SET_FSM_ID = 7101
SET_VELOCITY = 7105

FSM_ZERO_TORQUE = 0
FSM_DAMP = 1
FSM_LOCK_STAND = 4

# 目标运控模式：走跑运控
# ❗ **命令值必须是 801，不能写 802**。实测：
#     SetFsmId(801) -> fsm_id 瞬间经过 801，稳定在 802 = 走跑运控 ✅
#     SetFsmId(802) -> fsm_id 变成 **812 = 越障模式** ❌，而且越障模式不吃速度指令
#   官网那句「29dof 设备 ai_sport 8.6.x.x 版本后更新为 802」改的只是**回读编号**，
#   命令值仍然是 801。所以 WALK_ACCEPT 要同时收 801 和 802，判定只能用
#   「回读 ∈ 集合」，不能用「回读 == 命令值」。812 绝不能列进来。
# 其他运控模式本机不可用：500/501 属于 `normal` 服务，而 `SelectMode("normal")`
# 返回 7004，固件根本没装那个服务。
WALK_COMMAND = 801
WALK_ACCEPT = {801, 802}

# 进入路径：(命令值, 可接受的回读值, 说明, 等它做完的超时)
# 已经在路径上某一步时，前面的步骤会被裁掉：已经在预备(4)就不再白跑一遍阻尼
# （那会让机器人先塌下去再重新站起来），已经在运控就整段跳过
# 实测预备动作（站起来）要 ~5 s，fsm_mode 全程为 1，所以超时给得宽
ENTER_PATH = [
    (FSM_DAMP, {FSM_DAMP}, "阻尼", 5.0),
    (FSM_LOCK_STAND, {FSM_LOCK_STAND}, "预备 / 锁定站立", 20.0),
    (WALK_COMMAND, WALK_ACCEPT, "走跑运控", 15.0),
]

# 退出路径。**必须先经阻尼**：从承重站姿直接发 FSM 0，固件返回 code=0 却不执行，
# 机器人根本没卸力。而官方明确「阻尼模式作为最终的保底模式，始终能够被切换」。
EXIT_PATH = [
    (FSM_DAMP, {FSM_DAMP}, "阻尼", 5.0),
    (FSM_ZERO_TORQUE, {FSM_ZERO_TORQUE}, "零力矩", 5.0),
]

# SetFsmId 之后先硬等这么久再开始轮询。固件把 fsm_mode 置 1 需要一点时间，立刻轮询会读到过渡前的静态值，误以为已经做完了。
TRANSITION_LEAD_IN_S = 0.5
# fsm_mode 回 0 之后再多给一点余量，避免贴着临界点发下一条。
SETTLE_MARGIN_S = 1.0

RATE_HZ = 50.0

# arm_sdk 的全局接管权重，写在 motor_cmd[29].q（29 是官方枚举里的 kNotUsedJoint 空槽，不是关节）
#   0 = 上肢完全由内置运控程序控制，**下面 UPPER_BODY 的 q/kp/kd 会被固件直接忽略**
#   1 = 完全由本脚本接管
# 官方只把 0 和 1 文档化为端点语义，中间值只用于进出过渡，不要当稳态工作点
#
# ❗ **接管上肢与行走互斥，只能二选一。只要这个权重 > 0，`SetVelocity` 就完全不生效（响应码照样是 0，但机器人不进入移动状态）。
#   **唯一能走的权重是 0**——是开关不是渐变。详见 G1.md「Arm SDK 接管与内置行走互斥」。
#   所以：想遥控走路就保持 0；想接管上肢就设 1，但机器人只能原地站着。
WEIGHT = 1.0

# 接管时把 12-28 从实测位置插值到 UPPER_BODY 目标的时长
# 权重从第一帧就是 1.0（官方 example 如此），不跳变靠的是位置插值，不是权重斜坡
TAKEOVER_RAMP_S = 3.0

# 交还时把权重从 1.0 线性降到 0 的时长，防止摔手臂
RELEASE_RAMP_S = 5.0

# ---- 键盘遥控下半身 ----
# 速度指令刷新率。官方建议 5–10 Hz 刷新短 duration 指令，而不是开启长时模式：
# 一旦脚本挂了，指令过期机器人就自己停下来。
VELOCITY_RATE_HZ = 10.0
VELOCITY_DURATION_S = 1.0
# 最后一次按键后多久把速度归零。按住时靠终端的按键重复维持，松手就停。
# ⚠️ 别设太短：实测发出速度指令后要约 0.4 s 固件才把 fsm_mode 翻到「移动状态」，
#   再到迈开腿还要更久。设成 0.6 s 的话点按一下等于"刚要走就撤指令"，看起来像没反应。
#   1.5 s 让点按也能走出一步，按住则连续。松手最多多走这么久，别设得更大。
KEY_HOLD_S = 1.5
SPEED_VX = 0.5      # m/s，speed_mode=0 时固件上限 1.0
SPEED_VYAW = 0.6    # rad/s

# ❗ 站高控制（E/Q）已移除——**`SetStandHeight`(7104) 在走跑运控 (802) 下无效**。
# 2026-07-29 配对实测：在 LowStand(0.0) 与 HighStand(UINT32_MAX)——参数空间的两个
# 极端——之间交替 4 轮，膝关节角（索引 3/9）稳态均值只差 **0.0002 rad**，且每轮
# 方向不一致。与 `GetStandHeight`(7005) 在 802 返回 7301 自洽：站高属于 LocoState，
# 而 LocoState 在 802 下整个不可用，读不了也写不了。

# 按键 -> (轴, 符号)。全部用普通字母，不用方向键：方向键在不同终端下会发
# `\x1b[A`（普通模式）或 `\x1bOA`（application cursor key 模式），不稳定。
# tick() 里会先把读到的字节转小写，所以开不开 Caps Lock 都能用。
KEY_BINDINGS = {
    "w": ("vx", 1),
    "s": ("vx", -1),
    "a": ("vyaw", 1),
    "d": ("vyaw", -1),
}

# 电机索引 -> (q, kp, kd)。索引是 Unitree 官方电机编号，12-28 是腰 + 双臂。
# 只有 WEIGHT > 0 时这些值才会生效。
UPPER_BODY = {
    12: (0.0, 10.0, 1.5),   # waist_yaw
    13: (0.0, 10.0, 1.5),   # waist_roll
    14: (0.0, 10.0, 1.5),   # waist_pitch
    15: (0.0, 10.0, 1.5),   # left_shoulder_pitch
    16: (0.0, 10.0, 1.5),   # left_shoulder_roll
    17: (0.0, 10.0, 1.5),   # left_shoulder_yaw
    18: (0.0, 10.0, 1.5),   # left_elbow
    19: (0.0, 10.0, 1.5),   # left_wrist_roll
    20: (0.0, 10.0, 1.5),   # left_wrist_pitch
    21: (0.0, 10.0, 1.5),   # left_wrist_yaw
    22: (0.0, 10.0, 1.5),   # right_shoulder_pitch
    23: (0.0, 10.0, 1.5),   # right_shoulder_roll
    24: (0.0, 10.0, 1.5),   # right_shoulder_yaw
    25: (0.0, 10.0, 1.5),   # right_elbow
    26: (0.0, 10.0, 1.5),   # right_wrist_roll
    27: (0.0, 10.0, 1.5),   # right_wrist_pitch
    28: (0.0, 10.0, 1.5),   # right_wrist_yaw
}

WEIGHT_SLOT = 29

# 从 UPPER_BODY 拆出目标位置，接管和交还都要用。
TARGET = {index: value[0] for index, value in UPPER_BODY.items()}


def build_arm_sdk_command(positions: dict, weight: float) -> LowCmd:
    """按官方 g1_arm_sdk_dds_example 的写法组帧。

    只填 12-28 和权重槽。不设 crc / mode_machine / mode_pr / motor_cmd.mode ——
    官方明确“除 12-28 和 29 外其余元素均无效”，/arm_sdk 与 /lowcmd 不是同一条处理路径。
    """
    command = LowCmd()
    for index, (_, kp, kd) in UPPER_BODY.items():
        motor = command.motor_cmd[index]
        motor.q = positions[index]
        motor.dq = 0.0
        motor.tau = 0.0
        motor.kp = kp
        motor.kd = kd
    command.motor_cmd[WEIGHT_SLOT].q = weight
    return command


def read_upper_body_positions(node: Node, latest: dict, log, timeout_s: float = 5.0):
    """等一帧 /lowstate，取 12-28 的实测位置作为接管起点。

    不读实测位置就把权重拉到 1，手臂会从当前姿态瞬间阶跃到 UPPER_BODY 目标。
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and "msg" not in latest:
        rclpy.spin_once(node, timeout_sec=0.05)
    message = latest.get("msg")
    if message is None:
        log.error("没收到 %s，无法确定上肢起始位置" % LOWSTATE_TOPIC)
        return None
    return {index: float(message.motor_state[index].q) for index in UPPER_BODY}


def publish_ramp(publisher, duration_s: float, frame, should_continue=None) -> bool:
    """以 RATE_HZ 发布 duration_s 秒，`frame(phase)` 给出该时刻的命令。

    phase 从 1/steps 走到 1.0，所以末帧一定是终点值。
    """
    period = 1.0 / RATE_HZ
    steps = max(1, int(duration_s * RATE_HZ))
    for step in range(1, steps + 1):
        if should_continue is not None and not should_continue():
            return False
        publisher.publish(frame(step / steps))
        time.sleep(period)
    return True


def run_arm_sdk(publisher, log, start: dict, should_continue, tick=None) -> None:
    """接管上肢：先从实测位置插值到目标，再保持。

    权重从第一帧就是 WEIGHT（官方 example 如此）：第一帧命令就等于实测位置，
    本来就没有跳变，所以不需要对权重做斜坡。

    `tick` 在保持阶段每拍调一次，用来搭键盘遥控。
    """
    log.info("接管上肢：weight=%.2f，%.1f s 内从实测位置插值到目标"
             % (WEIGHT, TAKEOVER_RAMP_S))
    if not publish_ramp(
            publisher, TAKEOVER_RAMP_S,
            lambda phase: build_arm_sdk_command(
                {index: start[index] * (1.0 - phase) + TARGET[index] * phase
                 for index in UPPER_BODY}, WEIGHT),
            should_continue):
        return

    log.info("保持目标姿态，Ctrl+C 退出")
    hold = build_arm_sdk_command(TARGET, WEIGHT)
    while should_continue():
        publisher.publish(hold)
        if tick is not None:
            tick()
        time.sleep(1.0 / RATE_HZ)


def release_arm_sdk(publisher, log) -> None:
    """交还上肢：位置不动，权重线性降到 0。

    必须在卸力（阻尼/零力矩）**之前**做，否则会在还握着手臂的情况下卸腿。
    """
    if WEIGHT <= 0.0:
        return
    log.info("交还上肢：%.1f s 内把权重从 %.2f 降到 0" % (RELEASE_RAMP_S, WEIGHT))
    publish_ramp(publisher, RELEASE_RAMP_S,
                 lambda phase: build_arm_sdk_command(TARGET, WEIGHT * (1.0 - phase)))
    # 斜坡末帧已经是 0，这一帧是保险：交还权宁可多发一次。
    publisher.publish(build_arm_sdk_command(TARGET, 0.0))


class SportApi:
    """/api/sport 的请求-应答封装

    请求靠 header.identity.id 与应答配对。原来这里是 fire-and-forget，机器人回的拒绝错误码全被丢掉了 —— 表现就是"什么都没发生"
    """

    def __init__(self, node: Node) -> None:
        self._node = node
        self._publisher = node.create_publisher(
            Request, SPORT_REQUEST_TOPIC, QoSProfile(depth=1))
        self._replies = {}
        node.create_subscription(
            Response, SPORT_RESPONSE_TOPIC,
            lambda m: self._replies.setdefault(m.header.identity.id, m),
            QoSProfile(depth=10))

    def call(self, api_id: int, parameter: str = "", timeout_s: float = 3.0):
        """返回 (status_code, data)，超时返回 None"""
        request = Request()
        request.header.identity.id = time.monotonic_ns()
        request.header.identity.api_id = api_id
        request.parameter = parameter
        self._publisher.publish(request)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.02)
            reply = self._replies.pop(request.header.identity.id, None)
            if reply is not None:
                try:
                    data = json.loads(reply.data) if reply.data else {}
                except ValueError:
                    data = {}
                return reply.header.status.code, data
        return None

    def _read(self, api_id: int):
        result = self.call(api_id)
        return None if result is None or result[0] != 0 else result[1].get("data")

    def fsm_id(self):
        return self._read(GET_FSM_ID)

    def fsm_mode(self):
        """0 = 静态（允许切换），1 = 动态（切换会被静默拒绝）"""
        return self._read(GET_FSM_MODE)

    def set_fsm_id(self, fsm_id: int):
        return self.call(SET_FSM_ID, '{"data": %d}' % fsm_id)

    def send(self, api_id: int, parameter: str = "") -> None:
        """只发不等应答。

        速度流要以 10 Hz 发，而 `call()` 会 spin 等到应答才返回——卡在那里会把
        arm_sdk 的 50 Hz 发布一起阻住。错误码不会丢，由 drain() 统一捞。
        """
        request = Request()
        request.header.identity.id = time.monotonic_ns()
        request.header.identity.api_id = api_id
        request.parameter = parameter
        self._publisher.publish(request)

    def drain(self):
        """收一轮应答并清空缓存，返回其中非 0 的状态码。

        不清的话 `_replies` 会随 10 Hz 的速度流一直涨。
        """
        rclpy.spin_once(self._node, timeout_sec=0.0)
        codes = [reply.header.status.code for reply in self._replies.values()
                 if reply.header.status.code != 0]
        self._replies.clear()
        return codes


class Keyboard:
    """把终端切到 cbreak 做非阻塞读。stdin 不是 tty 时自动降级成空实现。

    必须用 with：不恢复终端属性的话，脚本退出后 shell 会不回显、不换行。
    """

    def __init__(self) -> None:
        self._fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        self._saved = None

    def __enter__(self):
        if self._fd is not None:
            self._saved = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        return self

    def __exit__(self, *_exc) -> None:
        if self._saved is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)

    @property
    def available(self) -> bool:
        return self._fd is not None

    def read(self) -> str:
        """把这一拍累积的按键字节全读出来。没有就返回空串。"""
        if self._fd is None:
            return ""
        chunks = []
        while select.select([self._fd], [], [], 0)[0]:
            chunks.append(os.read(self._fd, 32).decode(errors="ignore"))
        return "".join(chunks)


class Teleop:
    """键盘遥控下半身。每拍被 arm_sdk 的 50 Hz 循环回调一次。

    速度采用「按键刷新 + 超时归零」：终端只上报按下、不上报抬起，所以靠按键
    重复维持，断流 KEY_HOLD_S 就归零。
    """

    def __init__(self, api: SportApi, log, keyboard: Keyboard) -> None:
        self._api = api
        self._log = log
        self._keyboard = keyboard
        self._vx = 0.0
        self._vyaw = 0.0
        self._last_key = 0.0
        self._next_send = 0.0
        self._logged = None

    def tick(self) -> None:
        now = time.monotonic()
        keys = self._keyboard.read().lower()
        if " " in keys:
            self._vx = self._vyaw = self._last_key = 0.0
            self._log.warn("急停：速度归零")
        for sequence, (axis, sign) in KEY_BINDINGS.items():
            if sequence not in keys:
                continue
            if axis == "vx":
                self._vx, self._last_key = sign * SPEED_VX, now
            else:
                self._vyaw, self._last_key = sign * SPEED_VYAW, now

        if self._last_key and now - self._last_key > KEY_HOLD_S:
            self._vx = self._vyaw = self._last_key = 0.0

        if now >= self._next_send:
            self._next_send = now + 1.0 / VELOCITY_RATE_HZ
            self._send_velocity(self._vx, self._vyaw)

    def stop(self) -> None:
        """退出前显式发一次零速，不等 duration 自然过期。"""
        self._send_velocity(0.0, 0.0)

    def _send_velocity(self, vx: float, vyaw: float) -> None:
        # 只在变化时打，否则 10 Hz 会刷屏。这条日志是区分「按键没读到」和
        # 「读到了但机器人不动」的唯一依据——响应码恒为 0，指望不上。
        if (vx, vyaw) != self._logged:
            self._logged = (vx, vyaw)
            self._log.info("速度指令 vx=%+.2f vyaw=%+.2f" % (vx, vyaw))
        self._api.send(SET_VELOCITY, '{"velocity": [%f, 0.0, %f], "duration": %f}'
                                     % (vx, vyaw, VELOCITY_DURATION_S))
        self._api.drain()


def go_to_fsm(api: SportApi, log, command: int, accept: set, label: str,
              timeout_s: float, should_continue=None) -> bool:
    """切一个 FSM 并等它做完

    成功判据是「回读 ∈ accept 且 fsm_mode == 0」。**不能用「回读 == command」**——
    实测 SetFsmId(801) 之后 GetFsmId 回读的是 802，用等号判定会把成功当成失败。
    """
    result = api.set_fsm_id(command)
    if result is None:
        log.error("SetFsmId(%d) %s：无应答，运控服务没在跑？" % (command, label))
        return False
    if result[0] != 0:
        log.error("SetFsmId(%d) %s：被拒绝，status=%d" % (command, label, result[0]))
        return False

    time.sleep(TRANSITION_LEAD_IN_S)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if should_continue is not None and not should_continue():
            log.warn("SetFsmId(%d) %s：等待被中断" % (command, label))
            return False
        current_id, current_mode = api.fsm_id(), api.fsm_mode()
        if current_id in accept and current_mode == 0:
            log.info("FSM %s (%s) 已就位" % (current_id, label))
            time.sleep(SETTLE_MARGIN_S)
            return True
        time.sleep(0.1)
    log.error("SetFsmId(%d) %s：%.1f s 内没就位（期望回读 %s），当前 fsm_id=%s fsm_mode=%s"
              % (command, label, timeout_s, sorted(accept),
                 api.fsm_id(), api.fsm_mode()))
    return False


def follow_path(api: SportApi, log, path, should_continue=None) -> bool:
    """依次走过一串 FSM，每步都等它真的做完，任一步失败就停。"""
    for command, accept, label, timeout_s in path:
        if should_continue is not None and not should_continue():
            return False
        if not go_to_fsm(api, log, command, accept, label, timeout_s, should_continue):
            return False
    return True


def enter_walk_mode(api: SportApi, log, should_continue) -> bool:
    """把机器人从当前状态带到走跑运控。

    按当前 fsm_id 裁掉 ENTER_PATH 里已经走过的步骤；已经在运控时整段被裁空，
    follow_path 直接返回 True。
    """
    current = api.fsm_id()
    if current is None:
        log.error("读不到 fsm_id，运控服务没在跑？")
        return False
    log.info("起始 fsm_id=%s fsm_mode=%s" % (current, api.fsm_mode()))

    start = 0
    for index, (_, accept, _, _) in enumerate(ENTER_PATH):
        if current in accept:
            start = index + 1
    return follow_path(api, log, ENTER_PATH[start:], should_continue)


def enter_zero_torque(api: SportApi, log) -> None:
    """退出卸力：先阻尼，再零力矩。

    上一版直接发 SetFsmId(0)，从承重站姿会被固件静默拒绝（返回 code=0 但 fsm_id
    不变），机器人根本没卸力。官方明确「阻尼模式作为最终的保底模式，始终能够被
    切换」，所以必须先落到阻尼，再从阻尼进零力矩。

    这里不传 should_continue：清理必须做完。每步都有超时，最坏十几秒结束。
    """
    log.warn("退出：先阻尼再零力矩，机器人会瘫倒")
    if follow_path(api, log, EXIT_PATH):
        log.info("已进入零力矩")
    else:
        log.error("卸力没走完，当前 fsm_id=%s —— 请用遥控器 L2+B 手动卸力" % api.fsm_id())


def main() -> int:
    # rclpy 自己的 SIGINT 处理会直接关掉 context，那样退出时就发不出卸力指令了。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("g1_upper_body_takeover")
    log = node.get_logger()
    arm_sdk = node.create_publisher(LowCmd, ARM_SDK_TOPIC, QoSProfile(depth=10))
    api = SportApi(node)

    lowstate = {}
    node.create_subscription(
        LowState, LOWSTATE_TOPIC, lambda m: lowstate.update(msg=m),
        QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                   reliability=ReliabilityPolicy.BEST_EFFORT))

    running = True

    def on_signal(_signum, _frame) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    teleop = None
    try:
        # 让发布者先完成发现，否则第一帧会丢在没人订阅的话题上。
        time.sleep(0.5)

        if not enter_walk_mode(api, log, lambda: running):
            return 1

        start = read_upper_body_positions(node, lowstate, log)
        if start is None:
            return 1

        with Keyboard() as keyboard:
            if keyboard.available:
                teleop = Teleop(api, log, keyboard)
                log.info("键盘遥控：W/S 前后，A/D 转向，空格急停")
            else:
                log.warn("stdin 不是终端，键盘遥控关闭，机器人只会原地站立")
            run_arm_sdk(arm_sdk, log, start, lambda: running,
                        teleop.tick if teleop else None)
    finally:
        # 顺序不能反：先停走，再交还上肢，最后卸腿。
        if teleop is not None:
            teleop.stop()
        release_arm_sdk(arm_sdk, log)
        enter_zero_torque(api, log)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
