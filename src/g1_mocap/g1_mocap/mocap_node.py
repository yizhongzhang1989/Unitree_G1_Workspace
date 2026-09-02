"""动捕重定向节点：连头显，把重定向结果发成 ROS topic。**不控制任何硬件**。

发布三个 topic：

``~/frame`` (``g1_mocap_msgs/MocapFrame``)
    一帧的**全部**产物——关节角、根/锚位姿、key body 位置、人的原始骨架，
    加一个对齐好的时间戳。**跟随头显的 72/90 Hz，不做降采样**。
``~/joint_states`` (``sensor_msgs/JointState``)
    只有关节角。为生态兼容而留：``robot_state_publisher`` / rqt / plotjuggler
    直接就能用。要完整数据请订 ``~/frame``。
``~/status`` (``g1_mocap_msgs/MocapStatus``)
    1 Hz 的链路状态，结构化，可直接用作看门狗判据。

.. warning::
   **不要先降采样再插值。** 下游（比如全身动作跟踪的参考窗口）要靠帧间差分求速度，
   先降到 50 Hz 会把原始时间分辨率丢掉，速度噪声直接放大。要 50 Hz 就订原始帧率、
   自己按 ``header.stamp`` 插值。

链路全程 WiFi：头显上的 PicoBridge APK 直连本节点的 ``/ws/device``，中间不过
PicoBridge 的 ``server.py``。头显配置面板里填「本机IP:18000」。

> 本节点、``dashboard_node``、``g1_rgmt_tracking_global`` 的跟踪层**三选一**：
> 头显同一时刻只连一个上行地址。
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from g1_mocap_msgs.msg import MocapFrame, MocapStatus
from geometry_msgs.msg import Point, Pose
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .kinematics import G1Kinematics
from .retarget import Retargeter, RetargetResult
from .skeleton import STATUS_MESSAGES, BodyFrame
from .stream import MocapStream
from .urdf import DEFAULT_URDF, resolve_package_path


def fill_pose(pose: Pose, position: np.ndarray, quat_wxyz: np.ndarray) -> None:
    """内部一律 wxyz（与 MuJoCo / 训练侧一致），ROS 是 xyzw，只在这里转一次。"""
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
    pose.orientation.w = float(quat_wxyz[0])
    pose.orientation.x = float(quat_wxyz[1])
    pose.orientation.y = float(quat_wxyz[2])
    pose.orientation.z = float(quat_wxyz[3])


def to_points(array: np.ndarray) -> list:
    return [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in array]


class MocapNode(Node):

    def __init__(self) -> None:
        super().__init__('mocap')
        p = self.declare_parameter

        joints = list(p('joints', Parameter.Type.STRING_ARRAY)
                      .get_parameter_value().string_array_value)
        if not joints:
            raise RuntimeError('参数 joints 为空：应由 launch 注入 29 轴动作关节顺序')
        key_bodies = list(p('key_bodies', Parameter.Type.STRING_ARRAY)
                          .get_parameter_value().string_array_value)
        default_pos = np.asarray(
            p('default_joint_pos', Parameter.Type.DOUBLE_ARRAY)
            .get_parameter_value().double_array_value, dtype=np.float64)
        if len(default_pos) != len(joints):
            raise RuntimeError(
                f'default_joint_pos 有 {len(default_pos)} 项，与 {len(joints)} 轴对不上')

        urdf = resolve_package_path(p('urdf_path', DEFAULT_URDF)
                                    .get_parameter_value().string_value)
        self._retarget = Retargeter(
            G1Kinematics(urdf, joints), key_bodies=key_bodies,
            anchor_body=p('anchor_body', 'torso_link').get_parameter_value().string_value,
            default_joint_pos=default_pos,
            foot_ground_clearance_m=float(
                p('foot_ground_clearance_m', 0.03).get_parameter_value().double_value))

        self._stream = MocapStream(
            self._retarget,
            host=p('host', '0.0.0.0').get_parameter_value().string_value,
            port=int(p('port', 18000).get_parameter_value().integer_value),
            token=p('token', '').get_parameter_value().string_value,
            log=self.get_logger().info)

        self._joints = joints
        self._key_bodies = key_bodies
        # 深度给足：72/90 Hz 下 depth=1 会让偶发的调度抖动直接丢帧。
        stream_qos = QoSProfile(depth=20, history=HistoryPolicy.KEEP_LAST,
                                reliability=ReliabilityPolicy.BEST_EFFORT)
        self._frame_publisher = self.create_publisher(MocapFrame, '~/frame', stream_qos)
        self._joint_publisher = self.create_publisher(JointState, '~/joint_states',
                                                      stream_qos)
        self._status_publisher = self.create_publisher(MocapStatus, '~/status', 10)
        self._joint_message = JointState()
        self._joint_message.name = joints

        self.create_service(Trigger, '~/calibrate', self._on_calibrate)
        self._stream.on_frame = self._publish_frame
        self.create_timer(1.0, self._publish_status)
        self._stream.start()
        self.get_logger().info(
            f'动捕节点就绪: {len(joints)} 轴, key body {key_bodies}; '
            f'站立高度基准 {self._retarget.stand_height:.3f} m。'
            f'人站直后按双摇杆、或调 ~/calibrate 校准。')

    def destroy_node(self) -> bool:
        self._stream.on_frame = None
        self._stream.stop()
        return super().destroy_node()

    def _on_calibrate(self, _request, response):
        try:
            calibration = self._stream.calibrate()
        except (RuntimeError, ValueError) as exc:
            response.success, response.message = False, str(exc)
            return response
        response.success = True
        response.message = (f'缩放 {calibration.scale:.3f}, '
                            f'站立高度 {calibration.stand_height:.3f} m')
        if self._stream.last_calibration_warning:
            response.message += f'  ⚠ {self._stream.last_calibration_warning}'
        return response

    def _stamp(self, monotonic_t: float):
        """把收帧线程的单调时钟换算到 ROS 时钟。

        帧的时刻来自头显（抖动小），而 ``header.stamp`` 得落在 ROS 时钟域里。直接取
        「现在」会把帧间隔弄脏，所以**只平移不改间隔**；两个时钟的相对漂移在毫秒/小时
        量级，每帧现算偏移足够。
        """
        offset = self.get_clock().now().nanoseconds * 1e-9 - time.monotonic()
        return rclpy.time.Time(seconds=monotonic_t + offset).to_msg()

    def _publish_frame(self, stamped: float, raw: BodyFrame,
                       result: RetargetResult) -> None:
        """跑在**收帧线程**里，跟随头显帧率。不能阻塞。"""
        stamp = self._stamp(stamped)
        message = MocapFrame()
        message.header.stamp = stamp
        message.seq = raw.seq & 0xFFFFFFFF
        message.body_status = raw.status
        message.body_message = raw.message
        message.joint_names = self._joints
        message.joint_positions = result.joint_pos.tolist()
        fill_pose(message.root, result.root_pos, result.root_quat)
        fill_pose(message.anchor, result.anchor_pos, result.anchor_quat)
        message.key_body_names = self._key_bodies
        message.key_body_positions = to_points(result.key_pos)
        message.human_joints = to_points(raw.positions)
        self._frame_publisher.publish(message)

        self._joint_message.header.stamp = stamp
        self._joint_message.position = message.joint_positions
        self._joint_publisher.publish(self._joint_message)

    def _publish_status(self) -> None:
        stats = self._stream.stats()
        calibration = self._stream.calibration
        message = MocapStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.connected = stats.connected
        message.calibrated = calibration is not None
        message.frames = stats.frames
        message.dropped = stats.dropped
        message.body_status = stats.status
        message.body_message = stats.message
        message.body_message_text = STATUS_MESSAGES.get(stats.message, str(stats.message))
        message.last_error = stats.last_error
        if calibration is not None:
            message.scale = calibration.scale
            message.stand_height = calibration.stand_height
        self._status_publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = MocapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
