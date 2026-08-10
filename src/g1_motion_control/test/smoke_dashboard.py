#!/usr/bin/env python3
"""dashboard_node 的联调自检：假 URDF / joint_states / status，逐个打 HTTP 口。

    python3 src/g1_motion_control/test/smoke_dashboard.py

不需要真机也不需要控制栈。跑完会自己收摊。
"""
import json
import os
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from unitree_hg.msg import LowState

PORT = 8199
URL = f'http://127.0.0.1:{PORT}'
DOMAIN_ID = os.environ.get('DASHBOARD_SMOKE_DOMAIN_ID', '78')
URDF = (Path(get_package_share_directory('unitree_g1_description'))
        / 'model' / 'final.urdf')
RESULTS = []


def description() -> str:
    """把裸相对 mesh 路径改写成 ``package://``，和 ``control.launch.py`` 一样。

    真机上 ``/robot_description`` 是被那一步处理过才发出来的；直接发文件原文
    会得到一份 mesh 全都解析不了的 URDF，测了等于没测。
    """
    return URDF.read_text(encoding='utf-8').replace(
        'filename="', 'filename="package://unitree_g1_description/model/')


def check(label, condition) -> bool:
    RESULTS.append(bool(condition))
    print(f'{"PASS" if condition else "FAIL"}  {label}', flush=True)
    return bool(condition)


def get(path, timeout=5.0):
    deadline = time.monotonic() + timeout
    while True:
        try:
            with urllib.request.urlopen(URL + path, timeout=2.0) as response:
                return response.status, response.headers, response.read()
        except urllib.error.HTTPError as error:
            return error.code, error.headers, error.read()
        except OSError:
            if time.monotonic() > deadline:
                raise
            time.sleep(0.2)


class Source(Node):
    """假数据源：latched URDF + 100 Hz joint_states + 10 Hz status。"""

    def __init__(self):
        super().__init__('dashboard_smoke_source')
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        stream = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_publisher(String, '/robot_description', latched).publish(
            String(data=description()))
        self._joints = self.create_publisher(JointState, '/joint_states', stream)
        self._lowstate = self.create_publisher(LowState, '/lowstate', stream)
        self._command = self.create_publisher(
            Float64MultiArray, '/motion_control/command', 10)
        self._status = self.create_publisher(String, '/motion_control/status', 10)
        self.create_timer(0.01, self._tick_joints)
        self.create_timer(0.1, self._tick_status)

    def _tick_joints(self):
        message = JointState()
        message.name = ['left_elbow_joint', 'right_elbow_joint', 'left_eccentric_joint']
        message.position = [0.5, 0.7, 1.2]
        self._joints.publish(message)
        lowstate = LowState()
        for index, motor in enumerate(lowstate.motor_state):
            motor.q = index / 10.0
            motor.temperature = [40 + index, 50 + index]
        lowstate.motor_state[13].motorstate = 0x200
        self._lowstate.publish(lowstate)

    def _tick_status(self):
        command = Float64MultiArray()
        command.data = (
            [0.0, 0.0, 0.0, 0.74]
            + [0.9, 0.2, 0.1, 0.0, 0.0, 0.0, 1.0]
            + [0.9, -0.2, 0.1, 0.0, 0.0, 0.0, 1.0]
            + [0.0, 2.76]
        )
        self._command.publish(command)
        self._status.publish(String(data=json.dumps({
            'state': 'running', 'stale': '', 'ik_pos_err': 0.0724, 'ik_ms': 0.38,
            'limited_pose': {'left': [0.3, 0.2, 0.1, 0, 0, 0, 1],
                             'right': [0.3, -0.2, 0.1, 0, 0, 0, 1]}})))


def main() -> int:
    # 与正在运行的真机控制栈隔离，否则同名 status/joint_states/lowstate 会混流，
    # smoke 读到哪一个发布者取决于 DDS 调度，断言会随机失败。
    os.environ['ROS_DOMAIN_ID'] = DOMAIN_ID
    rclpy.init()
    source = Source()
    spin = threading.Thread(target=rclpy.spin, args=(source,), daemon=True)
    spin.start()
    dashboard = os.path.join(
        get_package_share_directory('g1_motion_control'), '..', '..',
        'lib', 'g1_motion_control', 'dashboard_node')
    child = subprocess.Popen(
        [dashboard, '--ros-args',
         '-p', f'bind_port:={PORT}', '-p', 'bind_host:=127.0.0.1'],
        stdout=subprocess.DEVNULL)
    try:
        status, _, body = get('/')
        check('/ 返回采集页', status == 200 and b'<canvas id="viewer">' in body)

        for path in ('/dashboard.js', '/joint_chart.js', '/viewer.js', '/dashboard.css',
                     '/vendor/three.module.js',
                     '/vendor/addons/controls/OrbitControls.js',
                     '/vendor/addons/loaders/STLLoader.js'):
            status, headers, body = get(path)
            check(f'{path} 可取（{len(body) // 1024} KB）',
                  status == 200 and len(body) > 100
                  and (not path.endswith('.js')
                       or headers['Content-Type'] == 'text/javascript'))

        check('穿越路径被挡', get('/../setup.py')[0] == 404)

        deadline = time.monotonic() + 10.0
        model = None
        while time.monotonic() < deadline:
            status, _, body = get('/api/model')
            if status == 200:
                model = json.loads(body)
                break
            time.sleep(0.3)
        if not check('/api/model 拿到模型', model is not None):
            return 1

        links = {link['name'] for link in model['links']}
        check(f'只剩两条手臂（{len(links)} link / {len(model["joints"])} 关节）',
              'head_link' not in links and 'left_gripper_base' in links
              and 'right_gripper_base' in links)
        check('mesh 走 /mesh 代理',
              all(v['url'].startswith('/mesh?')
                  for link in model['links'] for v in link['visuals']))

        url = model['links'][0]['visuals'][0]['url']
        status, headers, body = get(url)
        check(f'mesh 取得到且带长缓存（{len(body) // 1024} KB）',
              status == 200 and len(body) > 100
              and 'max-age' in (headers['Cache-Control'] or ''))
        check('/mesh 不认识的包报 404', get('/mesh?pkg=no_such_pkg&path=a.stl')[0] == 404)

        status, _, body = get('/api/state')
        state = json.loads(body)
        check(f'/api/state 很小（{len(body)} 字节）', status == 200 and len(body) < 4096)
        # /joint_states 是按需订阅的，第一次轮询之后才会订上，等它到齐。
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if (state['q'] and state['motors'] and state['command_pose']
                    and state['limited_pose'] and state['ik_pos_err'] is not None):
                break
            time.sleep(0.2)
            state = json.loads(get('/api/state')[2])
        clock = time.monotonic()
        for _ in range(50):
            get('/api/state')
        per = (time.monotonic() - clock) / 50 * 1e3
        waist_roll = state['motors'].get('waist_roll_joint', [])
        check(f'29 轴温度齐全，/api/state 单次 {per:.2f} ms',
              len(state['motor_names']) == len(state['motors']) == 29
              and len(waist_roll) == 4
              and abs(waist_roll[0] - 1.3) < 1e-6
              and waist_roll[1:] == [0x200, 53, 63]
              and per < 20.0)

        # 没人看页面就退订 100 Hz 的 /joint_states——这一路在 Jetson 上占 12% 单核。
        time.sleep(4.5)
        idle = json.loads(get('/api/state')[2])
        check('闲置后两路高频状态都被退订', not idle['q'] and not idle['motors'])
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if json.loads(get('/api/state')[2])['q']:
                break
            time.sleep(0.2)
        check('再看又自动订回来', bool(json.loads(get('/api/state')[2])['q']))
    finally:
        child.terminate()
        child.wait(timeout=5)
        # 先 shutdown 让 spin 退出再收节点，反过来会在 rclpy 里段错误。
        rclpy.shutdown()
        spin.join(timeout=2.0)

    failed = RESULTS.count(False)
    print(f'\n{len(RESULTS)} 项检查，失败 {failed} 项', flush=True)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
