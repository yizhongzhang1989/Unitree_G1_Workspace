"""实物对照：拍一张实拍图 + 读当前关节角 → URDF 拍照 → 把渲染轮廓叠上去。

只出一张 `_overlay.png`：左臂绿、右臂红的 URDF 轮廓压在实拍图上，对不齐就是挂载不对。

**不负责摆姿势。** 姿势用现成的手段摆好再跑这个：

    ros2 launch robot_bringup gravity_float_demo.launch.py   # 手推手臂到任意姿态
    ros2 launch head_sensors head_camera.launch.py
    ros2 run head_sensors verify_head_view --out /tmp/verify

关节角在 `--window` 秒内采样取均值。**不是为了降噪** —— 静止时 `/joint_states` 的
标准差实测只有 0.00002 rad，平均没有意义；窗口是用来发现「拍照那一刻手臂还在动」。

相机外参从 TF 读 `d435_link -> camera_color_optical_frame`，不能当成纯旋转：彩色镜头
不在 `d435_link` 原点上，实测偏 15 mm，忽略它渲染会整体横移十几个像素。

URDF 拍照那一半（`urdf_view.py` + `render_head_view.py`）不依赖 ROS，可以单独交付。
两侧通过一个临时姿态 JSON 交接（内含该外参），用完即删。
"""

import argparse
import os
import sys
import tempfile
import time
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from head_sensors.urdf_view import PinholeCamera, load_pose, save_pose, shoot

# /robot_description 是 latched 的，订阅得用 TRANSIENT_LOCAL 才能拿到已经发过的那一帧。
LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE)


class Capture(Node):
    """只订阅，不发布：URDF + 关节角 + 彩色 + 内参 + 外参。"""

    def __init__(self) -> None:
        super().__init__('verify_head_view')
        self.samples: List[Dict[str, float]] = []
        self.urdf: Optional[str] = None
        self.color: Optional[Image] = None
        self.info: Optional[CameraInfo] = None
        self.tf = Buffer()
        self._tf_listener = TransformListener(self.tf, self)
        self.create_subscription(String, '/robot_description',
                                 lambda m: setattr(self, 'urdf', m.data), LATCHED)
        self.create_subscription(JointState, '/joint_states',
                                 lambda m: self.samples.append(dict(zip(m.name, m.position))), 50)
        self.create_subscription(Image, '/head/camera/color/image_raw',
                                 lambda m: setattr(self, 'color', m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/head/camera/color/camera_info',
                                 lambda m: setattr(self, 'info', m), qos_profile_sensor_data)

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.005)

    def wait_until(self, predicate, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok() and not predicate():
            rclpy.spin_once(self, timeout_sec=0.02)
        return predicate()

    def extrinsic(self, parent: str, child: str, timeout: float = 5.0) -> Optional[np.ndarray]:
        """`parent` 到 `child` 的 4x4，查不到返回 None。"""
        zero = rclpy.time.Time()
        if not self.wait_until(lambda: self.tf.can_transform(parent, child, zero), timeout):
            return None
        tf = self.tf.lookup_transform(parent, child, zero).transform
        x, y, z, w = tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w
        out = np.eye(4)
        out[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                       [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                       [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
        out[:3, 3] = (tf.translation.x, tf.translation.y, tf.translation.z)
        return out


def _overlay(color: Image, label: np.ndarray, names: List[str]) -> np.ndarray:
    """渲染轮廓叠到实拍图上，左臂绿、右臂红。"""
    buf = np.frombuffer(color.data, dtype=np.uint8).reshape(color.height, color.width, -1)
    code = {'rgb8': cv2.COLOR_RGB2BGR,
            'yuv422_yuy2': cv2.COLOR_YUV2BGR_YUY2}.get(color.encoding)
    out = cv2.cvtColor(buf, code) if code is not None else buf.copy()
    for prefix, bgr in (('left_', (0, 255, 0)), ('right_', (0, 0, 255))):
        ids = [i for i, name in enumerate(names) if name.startswith(prefix)]
        contours, _ = cv2.findContours(np.isin(label, ids).astype(np.uint8),
                                       cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(out, contours, -1, bgr, 2)
    return out


def _package_dirs() -> str:
    """广播的 URDF 写的是 `package://unitree_g1_description/...`，要给包所在的父目录。"""
    return os.path.dirname(get_package_share_directory('unitree_g1_description'))


def _mktemp(suffix: str) -> str:
    handle, path = tempfile.mkstemp(prefix='head_view_', suffix=suffix)
    os.close(handle)
    return path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--window', type=float, default=0.5, help='关节角采样窗口（秒）')
    parser.add_argument('--still', type=float, default=0.01,
                        help='窗口内峰峰值超过这个值就警告手臂在动（rad）')
    parser.add_argument('--frame', default='d435_link')
    parser.add_argument('--optical-frame', default='camera_color_optical_frame',
                        help='彩色成像帧；从 TF 取它相对 --frame 的实测外参')
    parser.add_argument('--out', default='/tmp/verify')
    args = parser.parse_args(argv)

    rclpy.init()
    cap = Capture()

    # 渲染必须用机器人实际在跑的那份模型，不能拿磁盘上的替代。
    if not cap.wait_until(lambda: cap.urdf is not None, 10.0):
        print('收不到 /robot_description：没人广播 URDF。'
              '\n先起 all_data.launch.py scope:=whole_body topology:=dual', file=sys.stderr)
        return 1
    if not cap.wait_until(lambda: cap.color is not None and cap.info is not None, 10.0):
        print('收不到头部相机：先起 head_camera.launch.py', file=sys.stderr)
        return 1

    # --- 拍照，然后就地采一小段关节角 ---
    color = cap.color
    k = cap.info.k
    camera = PinholeCamera(cap.info.width, cap.info.height, k[0], k[4], k[2], k[5])
    cap.samples.clear()
    if not cap.wait_until(lambda: cap.samples, 10.0):
        print('收不到 /joint_states', file=sys.stderr)
        return 1
    cap.spin_for(args.window)

    names = sorted(cap.samples[0])
    table = np.array([[s[j] for j in names] for s in cap.samples])
    joints = dict(zip(names, table.mean(axis=0)))
    spread = table.ptp(axis=0)
    print('姿态 : %d 帧均值，%d 个关节，最大峰峰值 %.5f rad%s'
          % (len(table), len(names), spread.max(),
             '' if spread.max() <= args.still else '  ← 手臂在动，渲染必然对不上'))

    extrinsic = cap.extrinsic(args.frame, args.optical_frame)
    if extrinsic is None:
        print('外参 : 查不到 %s -> %s，只能当相机正好坐在原点上，渲染会整体横移十几个像素'
              % (args.frame, args.optical_frame))
    else:
        print('外参 : %s -> %s，平移 [%+.1f %+.1f %+.1f] mm'
              % (args.frame, args.optical_frame, *(1000.0 * extrinsic[:3, 3])))

    # --- 交给 URDF 侧渲染。两个临时文件都只是接口，用完即删 ---
    pose_path = _mktemp('.json')
    urdf_path = _mktemp('.urdf')
    try:
        with open(urdf_path, 'w') as fp:
            fp.write(cap.urdf)
        save_pose(pose_path, joints, camera, args.frame, extrinsic)
        pose_joints, pose_camera, pose_frame, pose_extrinsic = load_pose(pose_path)
        shot = shoot(pose_joints, urdf=urdf_path, mesh_dir=_package_dirs(),
                     camera=pose_camera, frame=pose_frame, extrinsic=pose_extrinsic)
    finally:
        os.unlink(pose_path)
        os.unlink(urdf_path)

    cv2.imwrite(args.out + '_overlay.png', _overlay(color, shot.label, shot.names))
    print('渲染 : 命中 %.1f%% 像素 -> %s_overlay.png'
          % (100.0 * np.isfinite(shot.depth).mean(), args.out))

    cap.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
