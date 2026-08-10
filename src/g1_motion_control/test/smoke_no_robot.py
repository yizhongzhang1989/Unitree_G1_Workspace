#!/usr/bin/env python3
"""无真机联调：假状态源 + 假 controller_manager，把状态机从头到尾走一遍。

真机不在手边的时候，这是唯一能验证"改完配置还能不能安全接管"的手段。它拉起真正的
``policy_node``（跑真正的 ONNX），只把机器人那一侧换成假的：

* 假 ``/joint_states`` + ``/pelvis_imu_broadcaster/imu``，100 Hz，姿态笔直、关节全 0
* 假 ``/controller_manager/switch_controller``，记录每次激活/反激活请求
* 录 ``/forward_position_controller/commands``，逐帧检查

跑法（先 ``source install/setup.bash``）：

    python3 src/g1_motion_control/test/smoke_no_robot.py

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

    time.sleep(config['stand_s'] + 0.3)
    expected = np.zeros(len(joints))
    expected[policy_slots] = config['stand_pose']
    expected[passive_slots] = config['passive_targets']
    check('站立终点 = 策略默认位姿', np.abs(node.targets[-1] - expected).max() < 1e-6)
    # "不是阶跃"要看**逐帧增量**，而它的上界由配置本身决定：侧开那一段（stand_clear_roll / stand_clear_s）是整个 STAND 里最快的运动。写死阈值的话
    # 一调参就误报；按配置推导才既跟得上调参、又卡得住真正的阶跃——阶跃是一帧走完全程，和这个上界差几十倍。
    increments = np.abs(np.diff(np.asarray(node.targets), axis=0))
    clear_s = config['stand_clear_s']
    bound = (config['stand_clear_roll'] / clear_s / config['control_rate_hz']
             if clear_s > 0 else 0.02)
    check(f'站立是渐变不是阶跃（逐帧 ≤ {bound * 1.2:.3f} rad）',
          increments.max() <= bound * 1.2)
    check('status 报告可以 start 了', node.status.get('ready_to_start'))

    print('\n-- start --', flush=True)
    node.call(node.start, 'start')
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
    # 没人发上肢指令时，手臂目标必须**不漂移**。注意不能断言"目标等于实测值"：
    # IK 用上一帧目标热启动，接管时用实测位形播种，之后发布值和实测值之间还隔着
    # PD 的跟随误差，两者不必相等，但必须帧帧一致。
    check('没有上肢指令时手臂/夹爪目标不漂移',
          np.ptp(run[:, passive_slots], axis=0).max() < 1e-6)
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
    from g1_motion_control.arm_ik import ArmIK
    ik = ArmIK(URDF_PATH.read_text(encoding='utf-8'), config['arm_joints'],
               {'left': config['left_tip_frame'], 'right': config['right_tip_frame']},
               base_frame=config['base_frame'])
    arm_slots = [joints.index(name) for name in ik.joint_names]
    left_slots = [joints.index(n) for n in ik.joint_names if n.startswith('left')]
    right_slots = [joints.index(n) for n in ik.joint_names if n.startswith('right')]
    grip_slots = [joints.index(name) for name in config['gripper_joints']]
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
    check('长度 2 只动夹爪',
          np.abs(last[grip_slots] - 1.0).max() < 1e-9
          and np.abs(last[arm_slots] - before[arm_slots]).max() < 1e-6)

    right_up = home['right'].copy()
    right_up[2] += 0.05
    before = np.asarray(node.targets)[-1]
    last = publish(right_up)
    reached = ik.fk(last[arm_slots])['right']
    error = float(np.linalg.norm(reached[:3] - right_up[:3]))
    check(f'长度 7 右手末端到位（残差 {error * 1e3:.2f} mm）', error < 3e-3)
    check('长度 7 不碰左臂，也不碰夹爪',
          np.abs(last[left_slots] - before[left_slots]).max() < 1e-6
          and np.abs(last[grip_slots] - 1.0).max() < 1e-9)

    left_up = home['left'].copy()
    left_up[2] += 0.05
    last = publish(np.concatenate([left_up, right_up]))
    check('长度 14 双臂同时跟随',
          np.abs(last[left_slots]).max() > 0.01
          and np.abs(last[right_slots]).max() > 0.01)

    # limited_pose 是 IK + arm_rate_limit 后的关节指令正解，不是假装成编码器实测。
    limited = (node.status.get('limited_pose') or {}).get('right')
    reached = ik.fk(last[arm_slots])['right']
    check('status 的 limited_pose 就是已发布关节指令的末端位姿',
          limited is not None
          and np.abs(np.asarray(limited) - reached).max() < 1e-4)

    arm_before = last[arm_slots].copy()
    last = publish([0.0, 0.0, 0.0, config['initial_height']])
    check('长度 4 完全不动上肢（teleop_keyboard.py 的回归）',
          np.abs(last[arm_slots] - arm_before).max() < 1e-9
          and np.abs(last[grip_slots] - 1.0).max() < 1e-9)

    last = publish([0.0] * 5, repeat=10)
    check('非法长度整帧丢弃',
          np.abs(last[arm_slots] - arm_before).max() < 1e-9
          and np.abs(last[grip_slots] - 1.0).max() < 1e-9)

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


# --------------------------------------------------------------------------- 入口

def build_params(path: Path) -> dict:
    """把包配置和 FPC 的 31 轴顺序拼成一个 params 文件，和 launch 做的事一样。"""
    share = Path(get_package_share_directory('g1_motion_control'))
    document = yaml.safe_load(
        (share / 'config' / 'motion_control.yaml').read_text(encoding='utf-8'))
    config = document['/motion_control']['ros__parameters']
    controller = yaml.safe_load(
        (Path(get_package_share_directory('unitree_g1_ros2_control')) /
         'config' / 'forward_position_controller.yaml').read_text(encoding='utf-8'))
    config['joints'] = controller['/forward_position_controller']['ros__parameters']['joints']
    path.write_text(yaml.safe_dump(document, allow_unicode=True), encoding='utf-8')
    return config


def main() -> int:
    if len(sys.argv) > 2 and sys.argv[1] == '--fake-state':
        run_fake_state(sys.argv[2])
        return 0

    params = Path('/tmp/motion_control_smoke.yaml')
    config = build_params(params)
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
        scenario(node, config)
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
