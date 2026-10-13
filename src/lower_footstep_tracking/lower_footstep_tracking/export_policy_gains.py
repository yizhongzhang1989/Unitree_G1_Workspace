"""Prepare hardware gains from a lower-body model, without starting ROS."""

import argparse
import copy
import json
from pathlib import Path

import onnxruntime
import yaml
from ament_index_python.packages import get_package_share_directory


def gain_profile(base, contract):
    result = copy.deepcopy(base)
    joints = result['/**']['ros__parameters']['joints']
    gains = result['/g1_gain_table']['ros__parameters']
    actions = contract['actions']
    for name, stiffness, damping in zip(
            actions['joint_names'], actions['stiffness'], actions['damping']):
        slot = joints.index(name)
        gains['stiffness'][slot] = stiffness
        gains['damping'][slot] = damping
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    options = onnxruntime.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    session = onnxruntime.InferenceSession(args.model, options, providers=['CPUExecutionProvider'])
    contract = json.loads(session.get_modelmeta().custom_metadata_map['footstep_contract'])
    common = (Path(get_package_share_directory('unitree_g1_ros2_control'))
              / 'config/default_31dof_param.yaml')
    base = yaml.safe_load(common.read_text())
    output = Path(args.output).expanduser()
    output.write_text(yaml.safe_dump(gain_profile(base, contract), sort_keys=False))
    print(output)


if __name__ == '__main__':
    main()
