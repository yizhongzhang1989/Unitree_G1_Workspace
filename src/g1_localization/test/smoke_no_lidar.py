#!/usr/bin/env python3
"""不用雷达、不用机器人，验证 localization_node 的整条链路。

`test_transforms.py` 只覆盖纯数学；TF 外参查找、``set_origin``、``origin_set``
标志位、``world -> pelvis`` 广播这几段都在节点里，坏了不会报错，只会让世界坐标
悄悄歪掉。这里造一个已知位姿的合成里程计喂进去，逐条核对发出来的消息。

    python3 src/g1_localization/test/smoke_no_lidar.py
"""

import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from g1_localization.localization_node import LocalizationNode
from g1_localization.transforms import make_tf, mat_to_quat, quat_to_mat, yaw_of

QOS = QoSProfile(depth=20, history=HistoryPolicy.KEEP_LAST,
                 reliability=ReliabilityPolicy.BEST_EFFORT)

#: 设原点那一刻的躯干位姿。yaw / 倾角 / 平移都给上，三者的处理方式各不相同：
#: 平移和 yaw 要被吃掉，倾角必须留着（否则世界系 z 轴不铅垂）。
YAW0, TILT0 = 1.1, 0.08
POS0 = np.array([3.0, -2.0, 0.55])

#: 与 config/point_lio_g1.yaml 的 mapping.extrinsic_T 一致
LIDAR_IN_IMU = np.array([-0.011, -0.02329, 0.04412])
BASE_TO_LIDAR = np.array([0.0603, 0.0, 0.4283])
#: IMU 原点在躯干系的位置，也就是杠杆臂
P_BASE_IMU = BASE_TO_LIDAR - LIDAR_IN_IMU


def _stf(parent, child, xyz):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    t.transform.rotation.w = 1.0
    return t


class Fixture(Node):
    """假的 robot_state_publisher + 假的 Point-LIO。"""

    def __init__(self):
        super().__init__('lio_fixture')
        self.static = StaticTransformBroadcaster(self)
        self.static.sendTransform([
            _stf('torso_link', 'livox_frame', BASE_TO_LIDAR),
            _stf('torso_link', 'pelvis', (0.0, 0.0, -0.32)),
        ])
        self.pub = self.create_publisher(Odometry, '/aft_mapped_to_init', QOS)
        self.got = []
        self.create_subscription(Odometry, '/g1_localization/torso_pose',
                                 self.got.append, QOS)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.cli = self.create_client(Trigger, '/g1_localization/set_origin')
        self.moving = False
        self.stop = False

    def r_odom_base(self):
        rot = quat_to_mat([0.0, 0.0, math.sin(YAW0 / 2), math.cos(YAW0 / 2)])
        tilt = quat_to_mat([math.sin(TILT0 / 2), 0.0, 0.0, math.cos(TILT0 / 2)])
        return rot @ tilt

    def tick(self):
        """发一帧里程计。``moving`` 之后躯干沿自己的 x 轴前进 1 m。"""
        r = self.r_odom_base()
        t_odom_base = np.eye(4)
        t_odom_base[:3, :3] = r
        t_odom_base[:3, 3] = POS0 + (r[:, 0] if self.moving else 0.0)
        # 节点要的是 IMU 体位姿，反推回去
        t_odom_imu = t_odom_base @ make_tf(P_BASE_IMU, [0.0, 0.0, 0.0, 1.0])

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lio_odom'
        msg.child_frame_id = 'imu_link'
        pos, quat = msg.pose.pose.position, msg.pose.pose.orientation
        pos.x, pos.y, pos.z = (float(v) for v in t_odom_imu[:3, 3])
        quat.x, quat.y, quat.z, quat.w = (
            float(v) for v in mat_to_quat(t_odom_imu[:3, :3]))
        msg.twist.twist.linear.x = 0.25          # odom 系，Point-LIO 的约定
        msg.twist.twist.angular.z = 0.4          # IMU 体系，同上
        self.pub.publish(msg)

    def run(self):
        while not self.stop:
            self.tick()
            time.sleep(0.1)


def main():
    fails = []

    def check(name, cond, detail=''):
        print(f"  {'OK  ' if cond else 'FAIL'}  {name}{('  ' + detail) if detail else ''}")
        if not cond:
            fails.append(name)

    rclpy.init()
    node = LocalizationNode()
    fx = Fixture()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    ex.add_node(fx)

    def spin_for(seconds):
        end = time.time() + seconds
        while time.time() < end:
            ex.spin_once(timeout_sec=0.02)

    ticker = threading.Thread(target=fx.run, daemon=True)
    ticker.start()
    spin_for(2.0)

    print('--- 没设原点时 ---')
    check('有 torso_pose 输出', len(fx.got) > 5, f'{len(fx.got)} 条')
    first = fx.got[-1]
    check('cov[0] == -1 标着原点未设', first.pose.covariance[0] == -1.0,
          f'实际 {first.pose.covariance[0]}')
    p = first.pose.pose.position
    check('位姿是单位阵', abs(p.x) + abs(p.y) + abs(p.z) < 1e-9)
    check('frame 是 world -> torso_link',
          first.header.frame_id == 'world' and first.child_frame_id == 'torso_link',
          f'{first.header.frame_id} -> {first.child_frame_id}')
    check('原点未设时不广播 world TF',
          not fx.buffer.can_transform('world', 'pelvis', rclpy.time.Time()))

    print('\n--- 调 set_origin ---')
    fx.cli.wait_for_service(timeout_sec=5.0)
    future = fx.cli.call_async(Trigger.Request())
    end = time.time() + 5.0
    while not future.done() and time.time() < end:
        ex.spin_once(timeout_sec=0.02)
    res = future.result()
    check('服务成功', res is not None and res.success, res.message if res else '超时')

    fx.got.clear()
    spin_for(1.5)
    print('\n--- 设完原点、机器人没动 ---')
    at0 = fx.got[-1]
    check('cov[0] 归零', at0.pose.covariance[0] == 0.0)
    p = at0.pose.pose.position
    check('位置回到原点', abs(p.x) + abs(p.y) + abs(p.z) < 1e-6,
          f'({p.x:.2e}, {p.y:.2e}, {p.z:.2e})')
    q = at0.pose.pose.orientation
    m = quat_to_mat([q.x, q.y, q.z, q.w])
    check('yaw 归零', abs(yaw_of(m)) < 1e-6, f'{math.degrees(yaw_of(m)):.4f}°')
    tilt = math.degrees(math.acos(min(max(m[2, 2], -1.0), 1.0)))
    check('倾角保留（世界 z 轴仍铅垂）', abs(tilt - math.degrees(TILT0)) < 1e-3,
          f'{tilt:.3f}° vs 期望 {math.degrees(TILT0):.3f}°')
    check('广播了 world -> pelvis',
          fx.buffer.can_transform('world', 'pelvis', rclpy.time.Time()))

    print('\n--- twist 换算（转头 0.4 rad/s + 前进 0.25 m/s）---')
    tw = at0.twist.twist
    check('角速度搬进躯干系', abs(tw.angular.z - 0.4) < 1e-6, f'wz={tw.angular.z:.6f}')
    lever_term = np.cross([0.0, 0.0, 0.4], -P_BASE_IMU)
    want = fx.r_odom_base().T @ np.array([0.25, 0.0, 0.0]) + lever_term
    got = np.array([tw.linear.x, tw.linear.y, tw.linear.z])
    check('线速度含杠杆项', np.allclose(got, want, atol=1e-6),
          f'实测 {np.round(got, 5)} 期望 {np.round(want, 5)}')
    check('杠杆项量级不可忽略', np.linalg.norm(lever_term) > 0.02,
          f'{np.linalg.norm(lever_term):.4f} m/s')

    print('\n--- 机器人前进 1 m ---')
    fx.moving = True
    fx.got.clear()
    spin_for(1.5)
    p = fx.got[-1].pose.pose.position
    dist = math.sqrt(p.x ** 2 + p.y ** 2 + p.z ** 2)
    check('世界系里正好前进 1 m', abs(dist - 1.0) < 1e-6, f'{dist:.6f} m')
    check('沿世界 x 方向（设原点时的朝向）',
          abs(p.x - 1.0) < 1e-6 and abs(p.y) < 1e-6,
          f'({p.x:.6f}, {p.y:.6f}, {p.z:.6f})')

    fx.stop = True
    ticker.join(timeout=1.0)
    node.destroy_node()
    fx.destroy_node()
    print('\n' + ('全部通过' if not fails else f'失败 {len(fails)} 项: {fails}'))
    rclpy.shutdown()
    raise SystemExit(0 if not fails else 1)


if __name__ == '__main__':
    main()
