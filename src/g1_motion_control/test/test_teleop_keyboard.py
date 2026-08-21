"""键盘遥控台里那份手臂浮动逻辑。

浮动整个实现都在遥控台这一侧（策略层只提供 ``arm_mode:=passthrough`` 这个入口），
所以拦住误发的责任也在这儿：IK 模式下把关节角当位姿发出去，会被策略层当成四元数
归一化后写进 14 个槽位。

和 test_dashboard.py 一样用 ``__new__`` 绕开 rclpy：这些分支一行 ROS 都不碰。
"""

import json
import time

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from g1_motion_control.teleop_keyboard import Teleop


ARMS = ['left_elbow_joint', 'right_elbow_joint']


def _teleop():
    node = Teleop.__new__(Teleop)
    node.__dict__.update(
        _float=False, _arm_mode='', _arm_names=[], _arm_q=None, _arm_stamp=0.0,
        _status='', notice='', _vel=[0.0, 0.0, 0.0], _height=0.74, _idle=0.0,
        _hold_timeout=0.4, _decay_s=0.5, sent=[],
        _message=Float64MultiArray(), _arm_message=Float64MultiArray())
    node._publisher = type('P', (), {
        'publish': lambda _s, m: node.sent.append(list(m.data))})()
    return node


def _status(node, **fields):
    node._on_status(String(data=json.dumps(
        {'state': 'running', 'arm_mode': 'passthrough', 'arm_joints': ARMS, **fields})))


def test_float_needs_passthrough_mode():
    node = _teleop()
    node._arm_q = [0.0, 0.0]
    _status(node, arm_mode='ik')
    node._toggle_float()
    assert not node._float and 'passthrough' in node.notice

    _status(node)
    node._toggle_float()
    assert node._float


def test_float_reads_only_the_reported_arm_joints():
    node = _teleop()
    _status(node)
    message = JointState()
    message.name = ['left_elbow_joint', 'waist_yaw_joint', 'right_elbow_joint']
    message.position = [0.3, 9.9, -0.4]
    node._on_joint_states(message)
    # 顺序跟 status 报的走，不跟 /joint_states 的排列走。
    assert node._arm_q == [0.3, -0.4]


def test_float_sends_a_separate_arm_frame_and_pauses_on_stale_states():
    node = _teleop()
    _status(node)
    node._float = True
    node._arm_q, node._arm_stamp = [0.3, -0.4], time.monotonic() - 1.0

    node.tick(0.02)
    # 陈帧只停发臂块，下肢那一帧照发——浮动不该把走路一起停了。
    assert node.sent == [[0.0, 0.0, 0.0, 0.74]] and '不新鲜' in node.notice

    node.sent.clear()
    node._arm_stamp = time.monotonic()
    node.tick(0.02)
    assert node.sent == [[0.0, 0.0, 0.0, 0.74], [0.3, -0.4]]


def test_float_turns_itself_off_when_the_policy_layer_leaves_stand_or_running():
    node = _teleop()
    _status(node)
    node._float = True
    _status(node, state='estop')
    assert not node._float
