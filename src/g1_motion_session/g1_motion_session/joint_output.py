"""Fixed-order masked joint publication with reusable message storage."""

import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unitree_g1_ros2_control.msg import JointTargets

from . import policy_support


two_stage = policy_support.two_stage


class JointOutput:
    def __init__(self, node, topic, slots):
        self.message = JointTargets(mask=sum(1 << index for index in slots))
        self.positions = np.frombuffer(self.message.positions, dtype=np.float64)
        self.positions31 = self.positions
        self.publisher = node.create_publisher(
            JointTargets, topic, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def publish(self):
        self.publisher.publish(self.message)
