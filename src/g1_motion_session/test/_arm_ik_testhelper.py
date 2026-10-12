"""Private offline fixtures: source-tree URDF/config and a non-Node host."""

import math
from pathlib import Path
from threading import RLock
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import yaml
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.parameter import Parameter
from std_msgs.msg import String

from g1_motion_session.arm_ik import ArmIK
from g1_motion_session.arm_ik_mixin import ArmIKMixin
from g1_motion_session.joint_output import JointOutput

ROOT = Path(__file__).resolve().parents[2]
ARM_JOINTS = [f'{side}_{joint}_joint'
              for side in ('left', 'right')
              for joint in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw',
                            'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')]
TIP_FRAMES = {'left': 'left_gripper_base', 'right': 'right_gripper_base'}


def urdf_xml():
    return (ROOT / 'unitree_g1_description/model/final.urdf').read_text(encoding='utf-8')


def config():
    return yaml.safe_load((ROOT / 'g1_motion_session/config/arm_ik.yaml').read_text())['/**']['ros__parameters']


def passive_targets():
    return [0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, -0.6, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0]


def stand_posture(model):
    by_name = dict(zip(ARM_JOINTS, passive_targets()[:14]))
    return np.array([by_name[name] for name in model.joint_names])


def configured_ik(null_gain=None, null_target=None):
    values = config()
    arms = values['arm_joints']
    if null_gain is None:
        null_gain = {name: gain for name, gain in zip(arms, values['ik_null_gain']) if gain}
    if null_target is None:
        null_target = {name: target for name, target in zip(arms, values['ik_null_target']) if target}
    return ArmIK(
        urdf_xml(), arms, TIP_FRAMES,
        base_frame=values['base_frame'], max_iters=values['ik_max_iters'],
        damping=values['ik_damping'], tol_pos=values['ik_tol_pos'],
        tol_ori=values['ik_tol_ori'], max_step_pos=values['ik_max_step_pos'],
        max_step_ori=values['ik_max_step_ori'], rotation_weight=values['ik_rotation_weight'],
        joint_limits={name: (-math.inf, high) for name, high in zip(arms, values['ik_limit_upper'])},
        null_gain=null_gain, null_target=null_target, null_gate=tuple(values['ik_null_gate']),
        reference=passive_targets()[:14], rate_limit=values['arm_rate_limit'],
        rescue_err=values['ik_rescue_err'], rescue_giveup=values['ik_rescue_giveup'])


def arm_pipeline(model, seed, targets):
    misses = model._rescue_miss
    solved = model.step(seed, targets, 0.02)[0]
    return solved, model._rescue_miss > misses


class Host(ArmIKMixin):
    def __init__(self, overrides=None):
        self.joints31 = [f'lower_{index}' for index in range(15)] + ARM_JOINTS + ['left_eccentric_joint', 'right_eccentric_joint']
        self.dt = 0.02
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.lock = RLock()
        self.output = Mock(spec=JointOutput, positions31=np.array([0.0] * 15 + passive_targets()))
        self.parameters = config()
        self.parameters.update(overrides or {})
        self.parameters['passive_targets'] = passive_targets()
        self.parameters['robot_description_topic'] = '/robot_description'
        self.declared = []
        self.subscriptions = []
        self.parameter_callbacks = []
        self.errors = []
        self.init_arm_ik()

    def declare(self, name, default):
        assert name not in self.declared
        self.declared.append(name)
        return self.parameters.setdefault(name, default)

    def get_parameter(self, name):
        return Parameter(name, value=self.parameters[name])

    def create_subscription(self, message_type, topic, callback, qos, **kwargs):
        subscription = SimpleNamespace(message_type=message_type, topic=topic,
                                       callback=callback, qos=qos, **kwargs)
        self.subscriptions.append(subscription)
        return subscription

    def add_on_set_parameters_callback(self, callback):
        self.parameter_callbacks.append(callback)

    def get_logger(self):
        return SimpleNamespace(error=self.errors.append)

    def bind(self):
        self.subscriptions[0].callback(String(data=urdf_xml()))
        assert not self.errors
