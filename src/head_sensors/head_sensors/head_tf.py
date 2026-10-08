from collections import deque
import math
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from unitree_hg.msg import IMUState

from .head_angle import joint_angle, zero_rotation


class HeadTF(Node):
    def __init__(self):
        super().__init__('head_tf')
        path = self.declare_parameter('calibration_file', str(Path(get_package_share_directory('camera_calibration')) / 'config/calibration.yaml')).value
        reference = yaml.safe_load(Path(path).read_text())['head_imu_reference']
        values = {name: np.asarray(reference[name], dtype=float) for name in ('head_zero', 'torso_zero')}
        if any(value.shape != (3,) or not np.isfinite(value).all() for value in values.values()):
            raise ValueError('Head calibration requires finite three-element vectors')
        head = self.declare_parameter('head_zero', values['head_zero'].tolist()).value
        torso = self.declare_parameter('torso_zero', values['torso_zero'].tolist()).value
        self.rotation = zero_rotation(head, torso)
        self.axis = None
        self.samples = {name: deque() for name in ('head', 'torso')}
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(String, 'robot_description', self.model,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Imu, '/utlidar/imu_livox_mid360', self.head, qos_profile_sensor_data)
        self.create_subscription(IMUState, '/secondary_imu', self.torso, qos_profile_sensor_data)
        self.create_timer(.02, self.publish)

    def model(self, msg):
        self.axis = None
        try:
            joint = ET.fromstring(msg.data).find("joint[@name='head_pitch_joint']")
            if joint is None or joint.get('type') != 'revolute':
                raise ValueError('Missing revolute head_pitch_joint')
            origin = joint.find('origin')
            position = np.asarray(origin.get('xyz', '0 0 0').split(), dtype=float)
            rpy = np.asarray(origin.get('rpy', '0 0 0').split(), dtype=float)
            axis = np.asarray(joint.find('axis').get('xyz').split(), dtype=float)
            if any(value.shape != (3,) or not np.isfinite(value).all() for value in (position, rpy, axis)):
                raise ValueError('Invalid neck geometry')
            if np.any(rpy != 0) or np.linalg.norm(axis) < 1e-12:
                raise ValueError('Neck requires zero origin rotation and nonzero axis')
            limit = joint.find('limit')
            lower, upper = float(limit.get('lower')), float(limit.get('upper'))
            if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
                raise ValueError('Invalid neck limits')
            self.parent = joint.find('parent').attrib['link']
            self.child = joint.find('child').attrib['link']
            self.origin = position.tolist()
            self.limits = lower, upper
            self.axis = axis / np.linalg.norm(axis)
        except (ET.ParseError, ValueError, AttributeError, KeyError, TypeError) as error:
            self.get_logger().error(f'Invalid neck model: {error}')

    def add(self, name, accel):
        now = time.monotonic()
        queue = self.samples[name]
        queue.append((now, accel))
        while queue and now - queue[0][0] > .15:
            queue.popleft()

    def head(self, msg):
        self.add('head', [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    def torso(self, msg):
        self.add('torso', list(msg.accelerometer))

    def publish(self):
        if self.axis is None:
            return
        now = time.monotonic()
        if any(not queue or now - queue[-1][0] > .2 for queue in self.samples.values()):
            return
        means = {name: np.mean([item[1] for item in queue], axis=0) for name, queue in self.samples.items()}
        if not .85 < np.linalg.norm(means['head']) < 1.15 or not 8.3 < np.linalg.norm(means['torso']) < 11.3:
            return
        try:
            angle = joint_angle(means['head'], means['torso'], self.rotation, self.axis)
        except ValueError:
            return
        if not self.limits[0] <= angle <= self.limits[1]:
            self.get_logger().warning('Head angle outside modeled range; no TF published', throttle_duration_sec=5)
            return
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.parent
        message.child_frame_id = self.child
        translation = message.transform.translation
        translation.x, translation.y, translation.z = self.origin
        rotation = message.transform.rotation
        vector = np.asarray(self.axis) / np.linalg.norm(self.axis) * math.sin(angle / 2)
        rotation.x, rotation.y, rotation.z = map(float, vector)
        rotation.w = math.cos(angle / 2)
        self.broadcaster.sendTransform(message)


def main():
    rclpy.init()
    node = HeadTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()