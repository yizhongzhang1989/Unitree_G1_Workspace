import ast
import runpy
from unittest.mock import patch

import rclpy
from pathlib import Path
from std_msgs.msg import String
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from head_sensors.head_tf import HeadTF


def test_console_entry():
    package = Path(__file__).resolve().parents[1]
    with patch('setuptools.setup') as setup:
        runpy.run_path(str(package / 'setup.py'))
    entries = setup.call_args.kwargs['entry_points']['console_scripts']
    assert 'head_tf = head_sensors.head_tf:main' in entries


@pytest.mark.parametrize('launch_path', [
    'unitree_g1_description/launch/description.launch.py',
    'unitree_g1_ros2_control/launch/control.launch.py',
])
def test_launch_node_name(launch_path):
    source = Path(__file__).resolve().parents[2] / launch_path
    tree = ast.parse(source.read_text())
    head_nodes = []
    for call in ast.walk(tree):
        if not (isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
                and call.func.id == 'Node'):
            continue
        keywords = {keyword.arg: keyword.value for keyword in call.keywords}
        package = keywords.get('package')
        if not isinstance(package, ast.Constant) or package.value != 'head_sensors':
            continue
        head_nodes.append(tuple(ast.literal_eval(keywords[name]) for name in ('executable', 'name')))
    assert head_nodes == [('head_tf', 'head_tf')]


def test_stale_and_zero_transform():
    rclpy.init()
    node = HeadTF()
    messages = []

    class Publisher:
        def sendTransform(self, message):
            messages.append(message)

    node.broadcaster = Publisher()
    try:
        assert node.get_name() == 'head_tf'
        node.publish()
        assert not messages
        node.add('head', node.get_parameter('head_zero').value)
        node.add('torso', node.get_parameter('torso_zero').value)
        node.publish()
        assert not messages
        model = Path(__file__).resolve().parents[2] / 'unitree_g1_description/model/final.urdf'
        node.model(String(data=model.read_text()))
        node.publish()
        message = messages[-1]
        assert message.header.frame_id == 'torso_link'
        assert message.child_frame_id == 'head_mount_link'
        assert abs(message.transform.rotation.w - 1) < 1e-10
        assert abs(message.transform.rotation.y) < 1e-10
        assert message.transform.translation.z == node.origin[2]
        angle = .25
        expected = Rotation.from_rotvec(np.asarray(node.axis) * angle)
        head = np.asarray(node.get_parameter('head_zero').value)
        node.samples['head'].clear()
        node.add('head', node.rotation.T @ expected.inv().apply(node.rotation @ head))
        node.publish()
        rotation = messages[-1].transform.rotation
        actual = Rotation.from_quat([rotation.x, rotation.y, rotation.z, rotation.w])
        assert (actual.inv() * expected).magnitude() < 1e-10
        assert all('joint_states' not in subscription.topic_name for subscription in node.subscriptions)
        count = len(messages)
        node.samples['head'].clear()
        node.publish()
        assert len(messages) == count
    finally:
        node.destroy_node()
        rclpy.shutdown()