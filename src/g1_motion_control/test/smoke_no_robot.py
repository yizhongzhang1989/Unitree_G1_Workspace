#!/usr/bin/env python3
"""无真机联调：假状态源 + 假 controller_manager，把状态机从头到尾走一遍。

真机不在手边的时候，这是唯一能验证"改完配置还能不能安全接管"的手段。它拉起真正的
``policy_node``（跑真正的 ONNX），只把机器人那一侧换成假的：

* 假 ``/joint_states`` + ``/pelvis_imu_broadcaster/imu``，100 Hz，姿态笔直、关节全 0
* 假 ``/controller_manager/switch_controller``，记录每次激活/反激活请求
* 录 ``/forward_position_controller/commands``，逐帧检查

跑法（先 ``source install/setup.bash``）：

    python3 src/g1_motion_control/test/smoke_no_robot.py
    python3 src/g1_motion_control/test/smoke_no_robot.py --passthrough

后者把上肢换成 ``arm_mode:=passthrough``，验证臂块被当成关节目标而不是末端位姿——
键盘遥控台的手臂浮动就跑在这条路径上。

假状态源放在独立进程里不是讲究：放在同一个进程里，主线程的忙等会把发布定时器压到
100 ms 以上，然后策略层的状态超时看门狗就会被自己人误触发。实机上这两个广播也确实
在 ros2_control_node 里，不在策略层进程内。
"""

import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import SwitchController
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Empty, Float64MultiArray, String
from std_srvs.srv import Trigger

STREAM = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                    reliability=ReliabilityPolicy.BEST_EFFORT)
LATCHED = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
RESULTS: list[bool] = []


def check(label, condition) -> bool:
    RESULTS.append(bool(condition))
    print(f'{"PASS" if condition else "FAIL"}  {label}', flush=True)
    return bool(condition)


URDF_PATH = (Path(get_package_share_directory('unitree_g1_description'))
             / 'model' / 'final.urdf')

# 零空间软偏好（``ik_null_gain`` / ``ik_null_target``）会一直把胘往参考角挪，所以
# “手臂没被碰过”不能再写成关节值逐位相等：实测 4 s 自由漂移下 wrist_roll 走了
# 0.23 rad、shoulder_roll 从 0.6 奔着 ik_null_target 的 0.34 去。真正守恒的是末端——
# 同一段里它只动了 0.115 mm。阈值取 IK 自己的收敛容差 ``ik_tol_pos``。
TIP_TOL = 1e-3


def tip_shift(ik, arm_slots, before, after, sides=('left', 'right')) -> float:
    """两帧 31 轴目标之间末端位置的最大变化（m）。"""
    start = ik.fk(np.asarray(before)[arm_slots])
    end = ik.fk(np.asarray(after)[arm_slots])
    return max(float(np.linalg.norm(start[side][:3] - end[side][:3]))
               for side in sides)


# --------------------------------------------------------------------------- 假状态源

class FakeState(Node):
    def __init__(self, joints):
        super().__init__('fake_state')
        self._js = self.create_publisher(JointState, '/joint_states', STREAM)
        self._imu = self.create_publisher(Imu, '/pelvis_imu_broadcaster/imu', STREAM)
        # 策略节点靠它建手臂 IK 模型；真机上这份由 ros2_control_node 发。
        self._description = self.create_publisher(String, '/robot_description', LATCHED)
        self._description.publish(String(data=URDF_PATH.read_text(encoding='utf-8')))
        self._message = JointState()
        self._message.name = list(joints)
        self._message.position = [0.0] * len(joints)
        self._message.velocity = [0.0] * len(joints)
        self._imu_message = Imu()
        self._imu_message.orientation.w = 1.0
        self._paused = False
        self.create_subscription(Empty, '/fake_state/pause', self._pause, 10)
        self.create_timer(0.01, self._publish)

    def _pause(self, _message):
        self._paused = True

    def _publish(self):
        if self._paused:
            return
        stamp = self.get_clock().now().to_msg()
        self._message.header.stamp = stamp
        self._imu_message.header.stamp = stamp
        self._js.publish(self._message)
        self._imu.publish(self._imu_message)


def run_fake_state(params_path):
    joints = yaml.safe_load(Path(params_path).read_text(encoding='utf-8'))
    joints = joints['/motion_control']['ros__parameters']['joints']
    rclpy.init()
    try:
        rclpy.spin(FakeState(joints))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


# --------------------------------------------------------------------------- 测试台

class Harness(Node):
    def __init__(self):
        super().__init__('smoke_harness')
        self.command = self.create_publisher(
            Float64MultiArray, '/motion_control/command', STREAM)
        self.pause = self.create_publisher(Empty, '/fake_state/pause', 10)
        self.create_subscription(
            Float64MultiArray, '/forward_position_controller/commands',
            self._on_target, STREAM)
        self.create_subscription(String, '/motion_control/status', self._on_status, 10)
        self.create_service(
            SwitchController, '/controller_manager/switch_controller', self._on_switch)
        self.engage = self.create_client(Trigger, '/motion_control/engage')
        self.start = self.create_client(Trigger, '/motion_control/start')
        self.estop = self.create_client(Trigger, '/motion_control/estop')
        self.targets, self.stamps, self.switches = [], [], []
        self.status, self.status_log = {}, []

    def _on_target(self, message):
        self.targets.append(np.asarray(message.data))
        self.stamps.append(time.monotonic())

    def _on_status(self, message):
        self.status = json.loads(message.data)
        self.status_log.append((time.monotonic(), self.status))

    def _on_switch(self, request, response):
        self.switches.append((list(request.activate_controllers),
                              list(request.deactivate_controllers)))
        response.ok = True
        return response

    def call(self, client, label):
        assert client.wait_for_service(timeout_sec=15.0), f'{label} 服务没出现'
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 25.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert future.done(), f'{label} 超时'
        print(f'  {label}: success={future.result().success} '
              f'"{future.result().message}"', flush=True)
        return future.result()


def scenario(node, config):
    joints = config['joints']
    policy_slots = [joints.index(name) for name in config['policy_joints']]
    passive_slots = [i for i in range(len(joints)) if i not in set(policy_slots)]
    grip_slots = [joints.index(name) for name in config['gripper_joints']]
    # 测试自己建一份模型，不从节点取：否则就是拿被测者的结果去验被测者。
    from g1_motion_control.arm_ik import ArmIK
    ik = ArmIK(URDF_PATH.read_text(encoding='utf-8'), config['arm_joints'],
               {'left': config['left_tip_frame'], 'right': config['right_tip_frame']},
               base_frame=config['base_frame'])
    arm_slots = [joints.index(name) for name in ik.joint_names]
    left_slots = [joints.index(n) for n in ik.joint_names if n.startswith('left')]
    right_slots = [joints.index(n) for n in ik.joint_names if n.startswith('right')]

    deadline = time.monotonic() + 30.0
    while not node.status and time.monotonic() < deadline:
        time.sleep(0.1)
    check('节点起来了且处于 idle', node.status.get('state') == 'idle')
    check('idle 下不发目标', not node.targets)

    print('\n-- engage --', flush=True)
    node.call(node.engage, 'engage')
    check('请求激活 FPC', node.switches[-1][0] == ['forward_position_controller'])
    time.sleep(0.5)
    check('站立第一帧贴近实测位姿', np.abs(node.targets[0]).max() < 0.05)
    check('站立未完成时拒绝 start', not node.call(node.start, 'start(早)').success)
    check('插值未走完时手臂还没接管', not node.status.get('arms_live'))

    time.sleep(config['stand_s'] + 0.3)
    expected = np.zeros(len(joints))
    expected[policy_slots] = config['stand_pose']
    expected[passive_slots] = config['passive_targets']
    settled = np.asarray(node.targets[-1])
    check('站立终点：下肢与夹爪逐位 = 策略默认位姿',
          np.abs(settled[policy_slots] - expected[policy_slots]).max() < 1e-6
          and np.abs(settled[grip_slots] - expected[grip_slots]).max() < 1e-6)
    # 手臂在 stand_s 那一刻就交给 IK 了，零空间偏好随即开始挪胘，所以只能对末端
    # 下断言——那才是 passive_targets 真正交出去的东西。
    shift = tip_shift(ik, arm_slots, expected, settled)
    check(f'站立终点：手臂末端 = passive_targets 的正解（差 {shift * 1e3:.2f} mm）',
          shift < TIP_TOL)
    # "不是阶跃"要看**逐帧增量**，而它的上界由配置本身决定：侧开那一段（stand_clear_roll / stand_clear_s）是整个 STAND 里最快的运动。写死阈值的话
    # 一调参就误报；按配置推导才既跟得上调参、又卡得住真正的阶跃——阶跃是一帧走完全程，和这个上界差几十倍。
    increments = np.abs(np.diff(np.asarray(node.targets), axis=0))
    clear_s = config['stand_clear_s']
    bound = (config['stand_clear_roll'] / clear_s / config['control_rate_hz']
             if clear_s > 0 else 0.02)
    check(f'站立是渐变不是阶跃（逐帧 ≤ {bound * 1.2:.3f} rad）',
          increments.max() <= bound * 1.2)
    check('status 报告可以 start 了', node.status.get('ready_to_start'))

    print('\n-- STAND 阶段手臂就该能动（不必等策略） --', flush=True)
    check('插值走完后 status 报告手臂已接管', node.status.get('arms_live'))
    stand_right = (node.status.get('limited_pose') or {}).get('right')
    check('STAND 阶段就发布 limited_pose', stand_right is not None)
    goal = np.asarray(stand_right, dtype=float)
    goal[2] += 0.05                                    # 右手抬 5 cm
    arm_message = Float64MultiArray()
    arm_message.data = [float(v) for v in goal]
    node.targets.clear()
    for _ in range(40):
        node.command.publish(arm_message)
        time.sleep(0.02)
    moved = np.asarray(node.targets)
    check('STAND 阶段右臂跟着动了',
          np.abs(moved[-1][passive_slots] - expected[passive_slots]).max() > 0.01)
    # 这才是这条路径的全部意义：手臂在动，腿和腰逐位不动。
    check('STAND 阶段下肢/腰逐位不动',
          np.abs(moved[:, policy_slots] - expected[policy_slots]).max() < 1e-9)
    stand_reached = (node.status.get('limited_pose') or {}).get('right')
    stand_err = float(np.linalg.norm(np.asarray(stand_reached)[:3] - goal[:3]))
    check(f'STAND 阶段右手末端到位（残差 {stand_err * 1e3:.2f} mm）', stand_err < 3e-3)

    print('\n-- start --', flush=True)
    # 删掉 _on_start 里的手臂重播种之后，策略接管不能再把手臂拽回实测位。
    before_start = np.asarray(node.targets[-1])
    node.call(node.start, 'start')
    time.sleep(0.2)
    step = config['arm_rate_limit'] / config['control_rate_hz']
    check(f'策略接管时手臂/夹爪目标不跳（≤ 一步限速 {step:.3f} rad）',
          np.abs(np.asarray(node.targets[-1])[passive_slots]
                 - before_start[passive_slots]).max() <= step + 1e-9)
    node.targets.clear()
    node.stamps.clear()
    node.status_log.clear()
    message = Float64MultiArray()
    target_height = config['command_limits'][6]        # 高度下界，行程最长
    message.data = [0.4, 0.0, 0.0, target_height]
    for _ in range(100):
        node.command.publish(message)
        time.sleep(0.02)

    run = np.asarray(node.targets)
    gaps = np.diff(node.stamps)
    check('running 在发目标', len(run) > 50)
    check(f'目标流没断（最大间隔 {gaps.max() * 1e3:.0f} ms）', gaps.max() < 0.1)
    check('下肢在动（策略真的在跑）', np.ptp(run[:, policy_slots], axis=0).max() > 0.02)
    # 没人发上肢指令时，下发给 FPC 的**末端**必须不漂移。注意不能断言“目标等于
    # 实测值”：IK 用上一帧目标热启动，接管时用实测位形播种，发布值和实测值之间
    # 还隔着 PD 的跟随误差；也不能断言关节值不动，零空间偏好本来就要挪胘。
    check('没有上肢指令时夹爪目标逐位不动',
          np.ptp(run[:, grip_slots], axis=0).max() < 1e-9)
    drift = max(tip_shift(ik, arm_slots, run[0], row) for row in run[::5])
    check(f'没有上肢指令时手臂只在零空间里动（末端漂 {drift * 1e3:.2f} mm）',
          drift < TIP_TOL)
    check('输出全部有限', np.all(np.isfinite(run)))
    lo = np.asarray(config['target_lower_limits'])
    hi = np.asarray(config['target_upper_limits'])
    check('下肢目标在 ctrlrange 内',
          np.all(run[:, policy_slots] >= lo - 1e-9)
          and np.all(run[:, policy_slots] <= hi + 1e-9))

    # status 只有 10 Hz，首尾样本的相位未知，所以只取过渡"内部"的样本量斜率。
    rate = config['height_rate_limit']
    inside = [(t, s['command'][3]) for t, s in node.status_log
              if target_height + 0.001 < s['command'][3] < config['initial_height'] - 0.001]
    slope = abs(inside[-1][1] - inside[0][1]) / (inside[-1][0] - inside[0][0])
    check(f'高度指令限速生效（实测 {slope:.3f} m/s，限 {rate}）',
          0.6 * rate < slope < 1.05 * rate)
    check('高度最终走到位', abs(node.status['command'][3] - target_height) < 1e-3)

    print('\n-- ~/command 分块：各发布者只更新自己那一段 --', flush=True)
    home = ik.fk(np.zeros(len(ik.joint_names)))

    def publish(values, repeat=20):
        message = Float64MultiArray()
        message.data = [float(v) for v in values]
        node.targets.clear()
        for _ in range(repeat):
            node.command.publish(message)
            time.sleep(0.02)
        return np.asarray(node.targets)[-1]

    check('status 报告 IK 就绪', node.status.get('ik_ready'))

    before = np.asarray(node.targets)[-1]
    last = publish([1.0, 1.0])
    shift = tip_shift(ik, arm_slots, before, last)
    check(f'长度 2 只动夹爪（双臂末端动了 {shift * 1e3:.2f} mm）',
          np.abs(last[grip_slots] - 1.0).max() < 1e-9 and shift < TIP_TOL)

    right_up = home['right'].copy()
    right_up[2] += 0.05
    before = np.asarray(node.targets)[-1]
    last = publish(right_up)
    reached = ik.fk(last[arm_slots])['right']
    error = float(np.linalg.norm(reached[:3] - right_up[:3]))
    check(f'长度 7 右手末端到位（残差 {error * 1e3:.2f} mm）', error < 3e-3)
    shift = tip_shift(ik, arm_slots, before, last, sides=('left',))
    check(f'长度 7 不碰左臂（左手末端动了 {shift * 1e3:.2f} mm），也不碰夹爪',
          shift < TIP_TOL and np.abs(last[grip_slots] - 1.0).max() < 1e-9)

    left_up = home['left'].copy()
    left_up[2] += 0.05
    last = publish(np.concatenate([left_up, right_up]))
    check('长度 14 双臂同时跟随',
          np.abs(last[left_slots]).max() > 0.01
          and np.abs(last[right_slots]).max() > 0.01)

    # limited_pose 是 IK + arm_rate_limit 后的关节指令正解，不是假装成编码器实测。
    # 它跟控制环发、目标流另发，两者相位无关，所以只能要求它等于**最近某一帧**
    # 已发布目标的正解，不能要求它恰好等于最后一帧。
    limited = (node.status.get('limited_pose') or {}).get('right')
    recent = [ik.fk(np.asarray(row)[arm_slots])['right'] for row in node.targets[-10:]]
    gap = (min(float(np.abs(np.asarray(limited) - pose).max()) for pose in recent)
           if limited is not None else float('inf'))
    check(f'status 的 limited_pose 就是已发布关节指令的末端位姿（差 {gap:.2e}）',
          gap < 1e-4)

    before = np.asarray(node.targets)[-1]
    last = publish([0.0, 0.0, 0.0, config['initial_height']])
    shift = tip_shift(ik, arm_slots, before, last)
    check(f'长度 4 完全不动上肢（末端动了 {shift * 1e3:.2f} mm，teleop_keyboard.py 的回归）',
          shift < TIP_TOL and np.abs(last[grip_slots] - 1.0).max() < 1e-9)

    before = last
    last = publish([0.0] * 5, repeat=10)
    shift = tip_shift(ik, arm_slots, before, last)
    check(f'非法长度整帧丢弃（末端动了 {shift * 1e3:.2f} mm）',
          shift < TIP_TOL and np.abs(last[grip_slots] - 1.0).max() < 1e-9)

    last = publish(np.concatenate([[0.0, 0.0, 0.0, config['initial_height']],
                                   home['left'], home['right'], [0.0, 0.0]]))
    # 契约是末端位姿，不是关节值：7 自由度冗余 + 零空间姿态偏置，同一个末端位姿会
    # 落在不同的关节位形上，这是设计如此。
    back = ik.fk(last[arm_slots])
    worst = max(float(np.linalg.norm(back[side][:3] - home[side][:3]))
                for side in ('left', 'right'))
    check(f'长度 20 全量：双臂末端回到原位（残差 {worst * 1e3:.2f} mm）、夹爪归零',
          worst < 3e-3 and np.abs(last[grip_slots]).max() < 1e-9)

    print('\n-- estop --', flush=True)
    node.call(node.estop, 'estop')
    check('请求反激活 FPC', node.switches[-1][1] == ['forward_position_controller'])
    node.targets.clear()
    time.sleep(0.5)
    check('急停后不再发目标', not node.targets)
    check('状态是 estop', node.status.get('state') == 'estop')

    print('\n-- 急停后重新接管必须重走 STAND --', flush=True)
    node.call(node.engage, 'engage(急停后)')
    time.sleep(0.3)
    check('重新 engage 回到 stand', node.status.get('state') == 'stand')
    check('不能跳过 STAND 直接 start', not node.call(node.start, 'start(早)').success)

    print('\n-- 看门狗：断掉状态源 --', flush=True)
    node.pause.publish(Empty())
    time.sleep(1.0)
    check('状态断流触发急停', node.status.get('state') == 'estop')
    check('急停原因是超时', '超时' in node.status.get('reason', ''))
    check('看门狗也走了反激活', node.switches[-1][1] == ['forward_position_controller'])


def scenario_passthrough(node, config):
    """``arm_mode:=passthrough``：臂块就是关节目标，不解 IK。

    这是键盘遥控台手臂浮动走的那条路径——浮动本身在遥控台里，节点这边只要保证
    「发进来的 14 个数原样落到那 14 个槽位、别的槽位一个都不碰」。
    """
    joints = config['joints']
    policy_slots = [joints.index(name) for name in config['policy_joints']]
    grip_slots = [joints.index(name) for name in config['gripper_joints']]

    deadline = time.monotonic() + 30.0
    while not node.status and time.monotonic() < deadline:
        time.sleep(0.1)
    check('节点起来了且处于 idle', node.status.get('state') == 'idle')
    check('status 报告 arm_mode 是 passthrough',
          node.status.get('arm_mode') == 'passthrough')

    print('\n-- engage --', flush=True)
    node.call(node.engage, 'engage')
    time.sleep(config['stand_s'] + 0.5)
    check('插值走完后手臂已接管', node.status.get('arms_live'))
    # 上层（遥控台）就是靠这份清单知道 14 个数该怎么排的，不报出来等于契约缺一半。
    arm_names = list(node.status.get('arm_joints') or [])
    check('status 报出的 arm_joints 就是配置里那 14 个手臂关节',
          sorted(arm_names) == sorted(config['arm_joints']))
    arm_slots = [joints.index(name) for name in arm_names]

    print('\n-- 臂块 = 关节目标 --', flush=True)
    goal = np.zeros(len(arm_names))
    goal[arm_names.index('left_elbow_joint')] = 0.6
    goal[arm_names.index('right_shoulder_yaw_joint')] = -0.4
    message = Float64MultiArray()
    message.data = [float(v) for v in goal]
    node.targets.clear()
    for _ in range(60):
        node.command.publish(message)
        time.sleep(0.02)
    stream = np.asarray(node.targets)
    last = stream[-1]
    check('长度 14 的臂块被原样当成关节目标', np.abs(last[arm_slots] - goal).max() < 1e-9)
    check('透传不碰下肢、不碰夹爪',
          np.ptp(stream[:, policy_slots], axis=0).max() < 1e-9
          and np.ptp(stream[:, grip_slots], axis=0).max() < 1e-9)
    # 限速是发布者阶跃的唯一一道堤；透传模式砍掉 IK 也不能把它一起砍掉。
    step = config['arm_rate_limit'] / config['control_rate_hz']
    check(f'出口限速仍生效（逐帧 ≤ {step:.3f} rad）',
          np.abs(np.diff(stream[:, arm_slots], axis=0)).max() <= step + 1e-9)

    print('\n-- 停发臂块 = 停在原地 --', flush=True)
    node.targets.clear()
    time.sleep(0.6)
    # 遥控台关掉浮动就只是不再发臂块，靠的正是“上肢指令不设超时”。
    check('停发后手臂目标不动',
          np.ptp(np.asarray(node.targets)[:, arm_slots], axis=0).max() < 1e-9)

    node.call(node.estop, 'estop')
    check('请求反激活 FPC', node.switches[-1][1] == ['forward_position_controller'])


# --------------------------------------------------------------------------- 入口

def build_params(path: Path, arm_mode: str) -> dict:
    """把包配置和 FPC 的 31 轴顺序拼成一个 params 文件，和 launch 做的事一样。"""
    share = Path(get_package_share_directory('g1_motion_control'))
    document = yaml.safe_load(
        (share / 'config' / 'motion_control.yaml').read_text(encoding='utf-8'))
    config = document['/motion_control']['ros__parameters']
    common = yaml.safe_load(
        (Path(get_package_share_directory('unitree_g1_ros2_control')) /
         'config' / 'default_31dof_param.yaml').read_text(encoding='utf-8'))
    config['joints'] = common['/**']['ros__parameters']['joints']
    config['arm_mode'] = arm_mode
    path.write_text(yaml.safe_dump(document, allow_unicode=True), encoding='utf-8')
    return config


def main() -> int:
    if len(sys.argv) > 2 and sys.argv[1] == '--fake-state':
        run_fake_state(sys.argv[2])
        return 0

    params = Path('/tmp/motion_control_smoke.yaml')
    arm_mode = 'passthrough' if '--passthrough' in sys.argv else 'ik'
    config = build_params(params, arm_mode)
    # 独立的 domain，免得撞上真的控制栈——这个测试会发目标，绝不能漏到真机上。
    environment = dict(os.environ, ROS_DOMAIN_ID=os.environ.get('SMOKE_DOMAIN_ID', '77'))
    children = [
        subprocess.Popen([sys.executable, __file__, '--fake-state', str(params)],
                         env=environment, stdout=subprocess.DEVNULL),
        subprocess.Popen(['ros2', 'run', 'g1_motion_control', 'policy_node',
                          '--ros-args', '--params-file', str(params)],
                         env=environment, stdout=subprocess.DEVNULL),
    ]
    os.environ['ROS_DOMAIN_ID'] = environment['ROS_DOMAIN_ID']
    time.sleep(6.0)

    # 站立位姿的期望值来自 ONNX metadata，测试自己再读一遍，和节点独立取数。
    from g1_motion_control.policy_runtime import load_policy
    _, spec = load_policy(str(Path(
        get_package_share_directory('g1_motion_control')) / 'config' / 'policy.onnx'))
    config['stand_pose'] = spec.default_pos

    rclpy.init()
    node = Harness()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        (scenario_passthrough if arm_mode == 'passthrough' else scenario)(node, config)
    finally:
        rclpy.shutdown()
        for child in children:
            child.terminate()
        for child in children:
            child.wait(timeout=5)
    failed = RESULTS.count(False)
    print(f'\n{len(RESULTS)} 项检查，失败 {failed} 项', flush=True)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
