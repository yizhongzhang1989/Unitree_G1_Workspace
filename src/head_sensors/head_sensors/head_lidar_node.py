"""G1 头部 Livox MID-360 雷达接入节点。

雷达由机器人内部的 `lidar_driver` 服务驱动（IP 192.168.123.120），
直接以 DDS 发布两个标准话题，本机不需要装 Livox SDK：

* `/utlidar/cloud_livox_mid360` — `sensor_msgs/PointCloud2`，10 Hz，19968 点/帧
* `/utlidar/imu_livox_mid360`  — `sensor_msgs/Imu`，200 Hz

原始话题有三个坑，本节点负责补上：

1. `frame_id` 是 `livox_frame`，而 URDF 里的链接叫 `mid360_link`，两者没有 TF
   相连，点云挂不到机器人模型上 —— 这里补一条恒等静态变换。
2. 每帧约 55% 的点是无回波的 `(0, 0, 0)` 占位（实测 10960/19968），
   但 `is_dense=true`，下游按稠密点云用会被这堆原点污染。
3. Livox IMU 的线加速度单位是 **g** 而不是 m/s²（实测静止时模长 ≈ 0.99），
   且 `orientation` 全零却没有按 REP-145 标记为无效。
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from tf2_ros import StaticTransformBroadcaster

from head_sensors.pointcloud import cloud_to_xyzi, filter_range, make_xyzi_cloud

STANDARD_GRAVITY = 9.80665


class HeadLidarNode(Node):

    def __init__(self) -> None:
        super().__init__('head_lidar')
        p = self.declare_parameter

        self._in_cloud = p('input_cloud_topic', '/utlidar/cloud_livox_mid360').value
        self._in_imu = p('input_imu_topic', '/utlidar/imu_livox_mid360').value
        out_cloud = p('output_cloud_topic', '/head/lidar/points').value
        out_imu = p('output_imu_topic', '/head/lidar/imu').value

        self._lidar_frame = p('lidar_frame', 'livox_frame').value
        self._mount_frame = p('mount_frame', 'mid360_link').value
        publish_static_tf = p('publish_static_tf', True).value

        # MID-360 盲区 0.1 m，标称最远 70 m（80% 反射率）。
        self._min_range = float(p('min_range', 0.1).value)
        self._max_range = float(p('max_range', 70.0).value)
        self._imu_accel_in_g = bool(p('imu_acceleration_in_g', True).value)
        self._stats_period = float(p('stats_period', 5.0).value)
        self._data_timeout = float(p('data_timeout', 2.0).value)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        self._cloud_pub = self.create_publisher(PointCloud2, out_cloud, qos)
        self._imu_pub = self.create_publisher(Imu, out_imu, qos)
        self.create_subscription(PointCloud2, self._in_cloud, self._on_cloud, qos)
        self.create_subscription(Imu, self._in_imu, self._on_imu, qos)

        if publish_static_tf:
            self._static_tf = StaticTransformBroadcaster(self)
            self._static_tf.sendTransform(self._identity_tf())

        self._clouds = 0
        self._kept = 0
        self._imus = 0
        self._last_cloud_wall = None
        self._timeout_warned = False
        self.create_timer(self._stats_period, self._on_stats)
        self.create_timer(0.5, self._on_watchdog)

        self.get_logger().info(
            '头部雷达接入：%s -> %s（%s，保留 %.2f~%.2f m）'
            % (self._in_cloud, out_cloud, self._lidar_frame,
               self._min_range, self._max_range))

    def _identity_tf(self) -> TransformStamped:
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._mount_frame
        tf.child_frame_id = self._lidar_frame
        tf.transform.rotation.w = 1.0
        return tf

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._last_cloud_wall = self.get_clock().now()
        self._timeout_warned = False
        try:
            xyzi = cloud_to_xyzi(msg)
        except ValueError as exc:
            self.get_logger().error('点云布局无法解析：%s' % exc)
            return
        kept, _ = filter_range(xyzi, self._min_range, self._max_range)
        self._clouds += 1
        self._kept += kept.shape[0]
        if self._cloud_pub.get_subscription_count() > 0:
            self._cloud_pub.publish(make_xyzi_cloud(msg.header, kept))

    def _on_imu(self, msg: Imu) -> None:
        self._imus += 1
        if self._imu_pub.get_subscription_count() == 0:
            return
        out = Imu()
        out.header = msg.header
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        if self._imu_accel_in_g:
            out.linear_acceleration.x *= STANDARD_GRAVITY
            out.linear_acceleration.y *= STANDARD_GRAVITY
            out.linear_acceleration.z *= STANDARD_GRAVITY
        q = msg.orientation
        if math.isclose(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 0.0, abs_tol=1e-9):
            out.orientation.w = 1.0
            out.orientation_covariance[0] = -1.0  # REP-145：姿态无效
        else:
            out.orientation = q
            out.orientation_covariance = msg.orientation_covariance
        self._imu_pub.publish(out)

    def _on_watchdog(self) -> None:
        if self._timeout_warned:
            return
        now = self.get_clock().now()
        if self._last_cloud_wall is None:
            stale = self._clouds == 0
        else:
            stale = (now - self._last_cloud_wall).nanoseconds * 1e-9 > self._data_timeout
        if not stale:
            return
        self._timeout_warned = True
        self.get_logger().warn(
            '%s 上没有点云：确认 lidar_driver 服务在运行'
            '（ros2 topic hz %s）' % (self._in_cloud, self._in_cloud))

    def _on_stats(self) -> None:
        if self._clouds == 0:
            return
        hz = self._clouds / self._stats_period
        self.get_logger().info(
            '点云 %.1f Hz，平均有效点 %d；IMU %.0f Hz'
            % (hz, self._kept // self._clouds, self._imus / self._stats_period))
        self._clouds = 0
        self._kept = 0
        self._imus = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeadLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
