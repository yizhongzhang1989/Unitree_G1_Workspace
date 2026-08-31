"""独立跑动捕：连上头显，把重定向后的位形发出来。

**这个节点不控制机器人**，只发数据。用途有三个：

* 调试：先确认头显连得上、``body.status`` 是 VALID、重定向出来的角度像样，再去接策略；
* 可视化：``/mocap/joint_states`` 直接能喂 RViz 的 robot_state_publisher；
* 给别的下游用：VLA、录制、别的策略都可以订。

跟踪层（``g1_rgmt_tracking_global``）**不通过话题**取动捕数据——参考窗口每拍要 42 个
时刻的插值，走话题会先被 50 Hz 采样一遍再插值，等于把 72/90 Hz 的原始时间分辨率
丢掉。它直接在进程内建 :class:`~g1_mocap.stream.MocapStream`。所以这两个节点是
**并列**关系，同时跑就是两条独立的连接，头显只能连一个。
"""

from __future__ import annotations

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .kinematics import G1Kinematics
from .retarget import Retargeter
from .stream import MocapStream

DEFAULT_URDF = ('package://unitree_g1_description/model/g1_description/'
                'g1_29dof_mode_15.urdf')


def resolve_package_path(path: str) -> str:
    prefix = 'package://'
    if not path.startswith(prefix):
        return path
    package, _, rest = path[len(prefix):].partition('/')
    return f'{get_package_share_directory(package)}/{rest}'


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
        self._kin = G1Kinematics(urdf, joints)
        self._retarget = Retargeter(
            self._kin, key_bodies=key_bodies,
            anchor_body=p('anchor_body', 'torso_link').get_parameter_value().string_value,
            default_joint_pos=default_pos,
            foot_ground_clearance_m=float(
                p('foot_ground_clearance_m', 0.03).get_parameter_value().double_value))

        self._stream = MocapStream(
            self._retarget,
            host=p('host', '0.0.0.0').get_parameter_value().string_value,
            port=int(p('port', 18000).get_parameter_value().integer_value),
            token=p('token', '').get_parameter_value().string_value,
            buffer_s=float(p('buffer_s', 2.0).get_parameter_value().double_value),
            log=self.get_logger().info)

        self._joints = joints
        self._auto = bool(p('auto_calibrate', True).get_parameter_value().bool_value)
        self._message = JointState()
        self._message.name = joints
        self._publisher = self.create_publisher(JointState, '~/joint_states', 10)
        self._status = self.create_publisher(String, '~/status', 10)
        self.create_service(Trigger, '~/calibrate', self._on_calibrate)

        rate = float(p('publish_rate_hz', 50.0).get_parameter_value().double_value)
        self.create_timer(1.0 / rate, self._publish)
        self.create_timer(1.0, self._report)
        self._stream.start()
        self.get_logger().info(
            f'动捕节点就绪: {len(joints)} 轴, key body {key_bodies}; '
            f'站立高度基准 {self._retarget.stand_height:.3f} m')

    def destroy_node(self) -> bool:
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
        return response

    def _publish(self) -> None:
        if not self._stream.calibrated:
            # 人站直的时候自动标一次。站得不对就 ~/calibrate 重来。
            if self._auto:
                try:
                    self._stream.calibrate()
                except (RuntimeError, ValueError):
                    pass
            return
        span = self._stream.span()
        if span is None:
            return
        batch = self._stream.sample(np.array([span[1]]))
        if batch is None:
            return
        self._message.header.stamp = self.get_clock().now().to_msg()
        self._message.position = batch.joint_pos[0].tolist()
        self._publisher.publish(self._message)

    def _report(self) -> None:
        message = String()
        message.data = (self._stream.describe_status()
                        + f' calibrated={self._stream.calibrated}')
        self._status.publish(message)


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
