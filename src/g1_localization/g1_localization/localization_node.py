#!/usr/bin/env python3
"""把 Point-LIO 的里程计换算成「躯干在世界系的位姿」，并对外只暴露两个接口。

    /head/lidar/points_full ─┐
                             ├─> point_lio ──> /aft_mapped_to_init ──> 本节点
    /utlidar/imu_livox_mid360 ┘              （IMU 体在 lio_odom 系的位姿）

对外契约就两条，其余都是实现细节，将来整体换掉 Point-LIO 也不影响下游：

* ``~/set_origin``（``std_srvs/Trigger``）——把此刻的躯干位姿钉成世界原点。
  响应的 ``message`` 带原点时间戳，采集侧记进 meta.json 用来追溯。
* ``~/torso_pose``（``nav_msgs/Odometry``）——``world`` -> ``torso_link``。
  **没设原点时 ``pose.covariance[0]`` 恒为 -1**（REP-145 的惯例，同 ``head_lidar_node``
  标 Livox 姿态无效的做法），下游据此区分「还没设原点」和「里程计没数据」。

另外可选广播 ``world -> pelvis`` 的 TF（``publish_tf`` 参数）。**只能挂在 pelvis 上**：
URDF 的根是 pelvis，整棵树里只有它没有父；``torso_link`` 的父已被 ``waist_pitch_joint``
占着，再发一份就是两个 publisher 抢同一个 child，而 tf2 不保证取哪一个。
挂 pelvis 不损失精度——查 ``world -> torso_link`` 时 tf2 会用同一份腰角走回来，
与这里算 pelvis 时用的那份精确抵消，前提是**两边用同一个时间戳**，所以下面查腰角
一律用里程计消息自己的 stamp，而不是「当前时刻」。

末端在世界系的位姿不在这里发：``/joint_states`` 已经在录，离线拿本话题乘一遍
正运动学就有了，在线再发一路只是重复。

输入频率就是点云的 10 Hz（每帧一条收敛值），所以不限流。要是有人开了
``publish_odometry_without_downsample``，输入会变成约 1450 Hz 的逐点中间态——
那些量本来就不该用，见 ``config/point_lio_g1.yaml`` 里的说明。
"""

from __future__ import annotations

import math
import threading
from typing import TypeVar

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from g1_localization.ground_plane import GroundPlane, fit_ground_plane

from g1_localization.transforms import (
    body_twist,
    ground_level_frame,
    invert,
    make_tf,
    mat_to_quat,
)

#: `pose.covariance[0]` 借来当标志位：世界原点还没设。协方差本身一律置零，理由见
#: `_publish_pose`。
ORIGIN_UNSET_COV = -1.0

_T = TypeVar('_T')


def _to_tf(translation, rotation) -> np.ndarray:
    """ROS 的 position/orientation 或 translation/rotation -> 4x4 齐次矩阵。"""
    return make_tf([translation.x, translation.y, translation.z],
                   [rotation.x, rotation.y, rotation.z, rotation.w])


def _fill(t: np.ndarray, translation, rotation) -> None:
    """4x4 齐次矩阵 -> ROS 字段。`Point` 与 `Vector3` 同构，两种消息都能填。"""
    translation.x = float(t[0, 3])
    translation.y = float(t[1, 3])
    translation.z = float(t[2, 3])
    q = mat_to_quat(t[:3, :3])
    rotation.x, rotation.y, rotation.z, rotation.w = (float(v) for v in q)


def _cloud_xyz(msg: PointCloud2) -> np.ndarray:
    """按标准 PointCloud2 字段偏移读取 xyz，不依赖具体点类型。"""
    fields = {field.name: field for field in msg.fields}
    if not all(name in fields for name in ('x', 'y', 'z')):
        raise ValueError('点云缺少 x/y/z 字段')
    endian = '>' if msg.is_bigendian else '<'
    dtype = np.dtype({
        'names': ('x', 'y', 'z'),
        'formats': tuple(endian + 'f4' for _ in range(3)),
        'offsets': tuple(fields[name].offset for name in ('x', 'y', 'z')),
        'itemsize': msg.point_step,
    })
    records = np.frombuffer(msg.data, dtype=dtype)
    return np.column_stack((records['x'], records['y'], records['z']))


class LocalizationNode(Node):

    def __init__(self) -> None:
        super().__init__('g1_localization')

        # `declare_parameter(...).value` 的静态类型是 `Any | None`，这里钉回默认值的类型。
        def p(name: str, default: _T) -> _T:
            value = self.declare_parameter(name, default).value
            return default if value is None else value

        odom_topic = p('odom_topic', '/aft_mapped_to_init')
        self._base = p('base_frame', 'torso_link')
        self._lidar = p('lidar_frame', 'livox_frame')
        self._root = p('root_frame', 'pelvis')
        self._world = p('world_frame', 'world')
        # 必须与 config/point_lio_g1.yaml 的 mapping.extrinsic_T 一致：那是「雷达在 IMU
        # 体系下的位置」，而 Point-LIO 输出的是 IMU 体的位姿，这里要用它退回雷达。
        self._lidar_in_imu = np.asarray(
            p('lidar_in_imu_translation', [-0.011, -0.02329, 0.04412]),
            dtype=np.float64)
        # 腰角 TF 是 100 Hz、里程计是 10 Hz，两条流各自异步，里程计 stamp 常常比缓冲区里
        # 最新的腰角样本新几毫秒。tf2 不外插，不等就会有约一半的帧发不出 TF。
        self._tf_wait = Duration(
            nanoseconds=int(1e9 * p('tf_lookup_timeout_s', 0.05)))
        self._warn_period = p('warn_period_s', 5.0)
        self._ground_cloud_topic = p('ground_cloud_topic', '/cloud_registered')
        self._ground_stale_s = p('ground_stale_s', 1.0)
        self._ground_sync_s = p('ground_sync_s', 0.2)
        self._ground_radius = p('ground_radius_m', 3.0)
        self._ground_min_radius = p('ground_min_radius_m', 0.8)
        if not 0.0 < self._ground_min_radius < self._ground_radius:
            raise RuntimeError('ground 半径必须满足 0 < min < max')

        self._lock = threading.Lock()
        self._t_imu_base: np.ndarray | None = None   # 常量，等 TF 到齐才算得出
        self._t_world_odom: np.ndarray | None = None
        self._last: tuple[Odometry, np.ndarray] | None = None
        self._odom_count = 0
        self._ground: tuple[Time, GroundPlane] | None = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self) if p('publish_tf', True) else None

        qos = QoSProfile(depth=20, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pose_pub = self.create_publisher(Odometry, '~/torso_pose', qos)
        self.create_subscription(Odometry, odom_topic, self._on_odom, qos,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(PointCloud2, self._ground_cloud_topic,
                                 self._on_ground_cloud, qos,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_service(Trigger, '~/set_origin', self._on_set_origin,
                            callback_group=ReentrantCallbackGroup())
        self.create_timer(self._warn_period, self._on_watchdog)

        tf_note = ('，并广播 %s -> %s' % (self._world, self._root)
                   if self._tf_broadcaster else '（不发 TF）')
        self.get_logger().info(
            '世界定位就绪：%s -> ~/torso_pose（%s -> %s）；~/set_origin 钉原点%s'
            % (odom_topic, self._world, self._base, tf_note))

    # -- 常量外参 --------------------------------------------------------------

    def _base_from_imu(self) -> np.ndarray | None:
        """`T_imu←base`，全常量，查到一次就缓存。

        链路是 base <- lidar <- imu：前一段取自 URDF 的 `mid360_joint`（fixed），
        后一段是 Point-LIO 的 extrinsic 之逆。
        """
        with self._lock:
            if self._t_imu_base is not None:
                return self._t_imu_base
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base, self._lidar, Time()).transform
        except Exception as exc:
            self.get_logger().warn(
                '还查不到 %s -> %s，等 robot_state_publisher：%s'
                % (self._base, self._lidar, exc),
                throttle_duration_sec=self._warn_period)
            return None
        t_lidar_imu = np.eye(4, dtype=np.float64)
        t_lidar_imu[:3, 3] = -self._lidar_in_imu
        result = invert(_to_tf(tf.translation, tf.rotation) @ t_lidar_imu)
        with self._lock:
            self._t_imu_base = result
        self.get_logger().info(
            '外参就绪：%s -> %s 平移 [%.4f %.4f %.4f] m'
            % (self._base, self._lidar, tf.translation.x, tf.translation.y,
               tf.translation.z))
        return result

    # -- 主链路 ----------------------------------------------------------------

    def _on_ground_cloud(self, msg: PointCloud2) -> None:
        try:
            xyz = _cloud_xyz(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=self._warn_period)
            return
        with self._lock:
            last = self._last
        if last is None:
            return
        torso = last[1][:3, 3]
        relative_xy = xyz[:, :2] - torso[:2]
        radius = np.linalg.norm(relative_xy, axis=1)
        keep = ((radius >= self._ground_min_radius)
                & (radius <= self._ground_radius)
                & (xyz[:, 2] >= torso[2] - 1.3)
                & (xyz[:, 2] <= torso[2] - 0.35))
        roi = xyz[keep]
        plane = fit_ground_plane(roi)
        if plane is None:
            return
        stamp = Time.from_msg(msg.header.stamp)
        with self._lock:
            self._ground = (stamp, plane)

    def _on_odom(self, msg: Odometry) -> None:
        # 计数放在最前面：它回答的是「里程计有没有在来」。放到外参查成功之后的话，
        # robot_state_publisher 没起时看门狗会报「没有里程计输入，确认 point_lio 在跑」，
        # 而真因是查不到 TF —— 把排查引向完全错误的方向（已踩过）。
        with self._lock:
            self._odom_count += 1
        t_imu_base = self._base_from_imu()
        if t_imu_base is None:
            return
        try:
            t_odom_imu = _to_tf(msg.pose.pose.position, msg.pose.pose.orientation)
        except ValueError:
            self.get_logger().warn('里程计四元数非法，跳过这一帧',
                                   throttle_duration_sec=self._warn_period)
            return
        t_odom_base = t_odom_imu @ t_imu_base

        # 上游的 twist 是混着的：linear 在世界系、angular 在 IMU 体系（Point-LIO 的状态
        # 就是这么定义的）。Odometry 的约定是两者都在 child_frame_id 系，这里补上换算，
        # 顺带把 IMU 原点的速度搬到躯干原点——两者差 0.4 m 以上，转头的杠杆速度不可忽略。
        twist = msg.twist.twist
        twist_base = body_twist(
            [twist.linear.x, twist.linear.y, twist.linear.z],
            [twist.angular.x, twist.angular.y, twist.angular.z],
            t_odom_imu, t_imu_base)

        with self._lock:
            self._last = (msg, t_odom_base)
            t_world_odom = self._t_world_odom

        valid = t_world_odom is not None
        t_world_base = (t_world_odom @ t_odom_base) if valid else np.eye(4)
        self._publish_pose(msg, t_world_base, twist_base, valid)
        if valid:
            self._publish_root_tf(msg, t_world_base)

    def _publish_pose(self, msg: Odometry, t_world_base: np.ndarray,
                      twist_base: tuple[np.ndarray, np.ndarray], valid: bool) -> None:
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._world
        out.child_frame_id = self._base
        _fill(t_world_base, out.pose.pose.position, out.pose.pose.orientation)
        for vec, values in ((out.twist.twist.linear, twist_base[0]),
                            (out.twist.twist.angular, twist_base[1])):
            vec.x, vec.y, vec.z = (float(v) for v in values)

        # 协方差一律置零。Point-LIO 默认路径根本不填它（实测整个 36 项全是 0），
        # 换算一个全零矩阵只会制造「有不确定度估计」的假象。唯一保留的是第 0 项，
        # 它不是方差而是标志位：-1 = 世界原点还没设（REP-145 的惯例）。
        out.pose.covariance = [0.0] * 36
        out.twist.covariance = [0.0] * 36
        if not valid:
            out.pose.covariance[0] = ORIGIN_UNSET_COV
        self._pose_pub.publish(out)

    def _publish_root_tf(self, msg: Odometry, t_world_base: np.ndarray) -> None:
        """广播 `world -> pelvis`。腰角必须取里程计这一帧的 stamp，理由见模块头。"""
        if self._tf_broadcaster is None:
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base, self._root, Time.from_msg(msg.header.stamp),
                timeout=self._tf_wait).transform
        except Exception as exc:
            self.get_logger().warn(
                '查不到 %s -> %s @ 该时刻，本帧不发 TF：%s'
                % (self._base, self._root, exc),
                throttle_duration_sec=self._warn_period)
            return
        out = TransformStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._world
        out.child_frame_id = self._root
        _fill(t_world_base @ _to_tf(tf.translation, tf.rotation),
              out.transform.translation, out.transform.rotation)
        self._tf_broadcaster.sendTransform(out)

    # -- 服务 ------------------------------------------------------------------

    def _on_set_origin(self, _request, response):
        """把此刻的躯干位姿钉成世界原点，直接复用 `_on_odom` 算好的那一帧。"""
        def fail(reason):
            response.success, response.message = False, reason
            return response

        if self._base_from_imu() is None:
            return fail('外参未就绪：查不到 %s -> %s' % (self._base, self._lidar))
        with self._lock:
            last = self._last
            ground = self._ground
        if last is None:
            return fail('还没收到里程计，确认 point_lio 在跑且已收敛')
        msg, t_odom_base = last
        stamp = Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        if age > 1.0:
            return fail('里程计已陈旧 %.1f s，拒绝设原点' % age)

        if ground is None:
            return fail('还没有可靠地面：确认 /cloud_registered 已发布')
        ground_stamp, plane = ground
        ground_age = (self.get_clock().now() - ground_stamp).nanoseconds * 1e-9
        if ground_age > self._ground_stale_s:
            return fail('地面估计已陈旧 %.1f s，拒绝设原点' % ground_age)
        ground_delta = abs((ground_stamp - stamp).nanoseconds * 1e-9)
        if ground_delta > self._ground_sync_s:
            return fail('地面与里程计相差 %.2f s，拒绝设原点' % ground_delta)

        origin = ground_level_frame(
            t_odom_base,
            plane.height_at(t_odom_base[0, 3], t_odom_base[1, 3]))
        t_world_odom = invert(origin)
        with self._lock:
            self._t_world_odom = t_world_odom
        stamp_s = stamp.nanoseconds * 1e-9
        tilt = math.degrees(math.acos(
            min(max((t_world_odom @ t_odom_base)[2, 2], -1.0), 1.0)))
        self.get_logger().info(
            '世界原点已钉在地面 t=%.6f，此刻躯干相对铅垂倾斜 %.2f°，地面 rmse %.3f m'
            % (stamp_s, tilt, plane.rmse))
        response.success = True
        response.message = 'origin_stamp=%.6f tilt_deg=%.2f' % (stamp_s, tilt)
        return response

    # -- 看门狗 ----------------------------------------------------------------

    def _on_watchdog(self) -> None:
        with self._lock:
            count, has_origin = self._odom_count, self._t_world_odom is not None
            self._odom_count = 0
        if count == 0:
            self.get_logger().warn(
                '没有里程计输入，确认 point_lio 在跑'
                '（ros2 topic hz /aft_mapped_to_init）')
        elif not has_origin:
            self.get_logger().info(
                '里程计 %.1f Hz，等 ~/set_origin 钉世界原点'
                % (count / self._warn_period),
                throttle_duration_sec=self._warn_period * 4)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    # launch 发 SIGINT 时 rclpy 先关掉 context，spin 抛的是 ExternalShutdownException
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
