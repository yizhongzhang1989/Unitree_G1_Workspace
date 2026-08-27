"""不跑控制栈时把标定出来的腕相机外参发成 static TF。

盯着 calibration.yaml 的 mtime：dashboard 里一点保存就自动重发，不用重启节点。
StaticTransformBroadcaster 是 latched 的，同名变换重发即覆盖。

默认跳过已经进了 urdf_overrides 的那些 frame：控制栈启动时会把它们插进
URDF，robot_state_publisher 已经在发了，再发一份就是两个 publisher 抢同一个
child，而 tf2 不保证取哪一个。
"""

from __future__ import annotations

from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class CalibTfNode(Node):

    def __init__(self) -> None:
        super().__init__('camera_calib_tf')
        share = Path(get_package_share_directory('camera_calibration')) / 'config'
        path = self.declare_parameter('calib_file', '').value
        self.path = Path(path).expanduser() if path else share / 'calibration.yaml'
        self.period = float(self.declare_parameter('check_period_s', 2.0).value)
        self.skip_urdf = bool(
            self.declare_parameter('skip_urdf_overrides', True).value)

        self._broadcaster = StaticTransformBroadcaster(self)
        self._mtime = None
        self._publish()
        self.create_timer(self.period, self._publish)

    def _publish(self) -> None:
        if not self.path.is_file():
            self.get_logger().warn(f'还没有 {self.path}，等标定完再说', once=True)
            return
        mtime = self.path.stat().st_mtime
        if mtime == self._mtime:
            return
        try:
            data = yaml.safe_load(self.path.read_text(encoding='utf-8')) or {}
        except (OSError, yaml.YAMLError) as error:
            self.get_logger().warn(f'读不了 {self.path}：{error}')
            return
        self._mtime = mtime

        transforms = []
        owned = {entry.get('child')
                 for entry in (data.get('urdf_overrides') or {}).values()
                 } if self.skip_urdf else set()
        for camera, entry in (data.get('extrinsics') or {}).items():
            child = entry.get('child', camera)
            if child in owned:
                self.get_logger().info(
                    f'{child} 由 URDF 发，这里不重复发（要单独发就设 '
                    f'skip_urdf_overrides:=false）', once=True)
                continue
            message = TransformStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = entry['parent']
            message.child_frame_id = child
            message.transform.translation.x = float(entry['translation'][0])
            message.transform.translation.y = float(entry['translation'][1])
            message.transform.translation.z = float(entry['translation'][2])
            quat = entry['rotation']
            message.transform.rotation.x = float(quat[0])
            message.transform.rotation.y = float(quat[1])
            message.transform.rotation.z = float(quat[2])
            message.transform.rotation.w = float(quat[3])
            transforms.append(message)
        if not transforms:
            self.get_logger().warn(
                f'{self.path} 里没有要发的外参', once=True)
            return
        self._broadcaster.sendTransform(transforms)
        self.get_logger().info('发布外参 TF：' + '、'.join(
            f'{t.header.frame_id} → {t.child_frame_id}' for t in transforms))


def main() -> None:
    rclpy.init()
    node = CalibTfNode()
    try:
        rclpy.spin(node)
    # launch 发 SIGINT 时 rclpy 先关掉 context，spin 抛的是 ExternalShutdownException，
    # 不是 KeyboardInterrupt。只接后者的话退出时会吐一屏 traceback 且退出码非零
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
