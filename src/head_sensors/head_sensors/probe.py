"""一次性体检：头部雷达/相机当前到底能不能用。

    ros2 run head_sensors head_sensors_probe

依次检查：
1. `/utlidar/*` 雷达话题是否有数据、频率和有效点数；
2. `robot_state` 服务的 ServiceList(api_id=1003)，看 `lidar_driver` 等服务状态；
3. RealSense D435i 是否枚举到 USB，以及 `/head/camera/*` 是否在出图。
"""

import glob
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu, PointCloud2
from unitree_api.msg import Request, Response

from head_sensors.pointcloud import cloud_to_xyzi, filter_range

ROBOT_STATE_API_ID_SERVICE_LIST = 1003
INTEL_VENDOR_ID = '8086'


class Probe(Node):

    def __init__(self) -> None:
        super().__init__('head_sensors_probe')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        # realsense2_camera 的图像话题是 best-effort 的 sensor QoS。
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            PointCloud2, '/utlidar/cloud_livox_mid360', self._on_cloud, qos)
        self.create_subscription(
            Imu, '/utlidar/imu_livox_mid360', self._on_imu, qos)
        self.create_subscription(
            Image, '/head/camera/color/image_raw', self._on_color, sensor_qos)
        self.create_subscription(
            Image, '/head/camera/depth/image_rect_raw', self._on_depth, sensor_qos)
        self.clouds = []
        self.imu_count = 0
        self.color = None
        self.depth = None

        self.api_pub = self.create_publisher(
            Request, '/api/robot_state/request', qos)
        self.create_subscription(
            Response, '/api/robot_state/response', self._on_api, qos)
        self._api_resp = None

    def _on_cloud(self, msg: PointCloud2) -> None:
        self.clouds.append((time.monotonic(), msg))

    def _on_imu(self, _msg: Imu) -> None:
        self.imu_count += 1

    def _on_color(self, msg: Image) -> None:
        self.color = msg

    def _on_depth(self, msg: Image) -> None:
        self.depth = msg

    def _on_api(self, msg: Response) -> None:
        self._api_resp = msg

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def call_robot_state(self, api_id: int, timeout: float = 5.0):
        self._api_resp = None
        if self.api_pub.get_subscription_count() == 0:
            return None
        req = Request()
        req.header.identity.id = int(time.time() * 1e6) % (2 ** 31)
        req.header.identity.api_id = api_id
        self.api_pub.publish(req)
        end = time.monotonic() + timeout
        while time.monotonic() < end and self._api_resp is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        return self._api_resp


def _usb_realsense() -> list:
    """从 sysfs 找 Intel(8086) USB 设备，不依赖 lsusb。"""
    found = []
    for vendor_path in glob.glob('/sys/bus/usb/devices/*/idVendor'):
        base = vendor_path.rsplit('/', 1)[0]
        try:
            with open(vendor_path) as fp:
                if fp.read().strip().lower() != INTEL_VENDOR_ID:
                    continue
            with open(base + '/idProduct') as fp:
                product_id = fp.read().strip()
        except OSError:
            continue
        try:
            with open(base + '/product') as fp:
                name = fp.read().strip()
        except OSError:
            name = '?'
        found.append('%s:%s %s' % (INTEL_VENDOR_ID, product_id, name))
    return found


def _report_lidar(probe: Probe, window: float) -> bool:
    print('== 头部雷达 Livox MID-360 ==')
    if not probe.clouds:
        print('  点云: 无数据（检查 lidar_driver 服务与 192.168.123.120）')
        return False
    stamps = [t for t, _ in probe.clouds]
    span = stamps[-1] - stamps[0]
    hz = (len(stamps) - 1) / span if span > 0 else float('nan')
    _, last = probe.clouds[-1]
    xyzi = cloud_to_xyzi(last)
    kept, dist = filter_range(xyzi, 0.1, 70.0)
    print('  点云: %.1f Hz, frame_id=%s, %d 点/帧, 有效 %d (%.0f%%)'
          % (hz, last.header.frame_id, xyzi.shape[0], kept.shape[0],
             100.0 * kept.shape[0] / max(xyzi.shape[0], 1)))
    if kept.shape[0]:
        print('  距离: %.2f ~ %.2f m' % (dist.min(), dist.max()))
    print('  IMU : %.0f Hz' % (probe.imu_count / window))
    return True


def _report_services(probe: Probe) -> None:
    print('== 机器人内部服务（robot_state ServiceList）==')
    resp = probe.call_robot_state(ROBOT_STATE_API_ID_SERVICE_LIST)
    if resp is None:
        print('  robot_state 无应答')
        return
    try:
        services = json.loads(resp.data)
    except ValueError:
        print('  应答不是 JSON: %s' % resp.data[:200])
        return
    for s in services:
        if any(k in s['name'] for k in ('lidar', 'slam', 'video', 'camera')):
            print('  %-28s status=%d protect=%d' % (s['name'], s['status'], s['protect']))
    print('  （status=0 表示服务在运行）共 %d 个服务' % len(services))


def _report_camera(probe: Probe) -> bool:
    print('== 头部相机 RealSense D435i ==')
    devices = _usb_realsense()
    if devices:
        print('  USB : ' + ', '.join(devices))
    else:
        print('  USB : 没有枚举到 Intel(8086) 设备 —— 相机没接在本机 NX 上')
    if probe.color is None and probe.depth is None:
        print('  话题: /head/camera/* 无数据'
              '（需先 ros2 launch head_sensors head_camera.launch.py）')
        return False
    for name, msg in (('color', probe.color), ('depth', probe.depth)):
        if msg is None:
            print('  %-5s: 无数据' % name)
        else:
            print('  %-5s: %dx%d %s frame_id=%s'
                  % (name, msg.width, msg.height, msg.encoding, msg.header.frame_id))
    return probe.color is not None


def main(args=None) -> int:
    rclpy.init(args=args)
    probe = Probe()
    window = 3.0
    probe.spin_for(window)
    lidar_ok = _report_lidar(probe, window)
    _report_services(probe)
    camera_ok = _report_camera(probe)
    probe.destroy_node()
    rclpy.shutdown()
    return 0 if (lidar_ok and camera_ok) else 1


if __name__ == '__main__':
    sys.exit(main())
