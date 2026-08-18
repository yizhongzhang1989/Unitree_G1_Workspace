#!/usr/bin/env python3
"""由两侧的相机位姿自动解出坐标系参数，打印可直接粘贴的 YAML。

    # 对面给了训练 episode 里真实的 head_camera_in_world（4x4）
    ros2 run g1_vla_bridge calibrate_frame --camera-in-world train_cam.json

    # 对面只给了训练时的关节值，用 A2D 的 URDF + 外参自己算 head_camera_in_world
    ros2 run g1_vla_bridge calibrate_frame --lift 0.1 --body-pitch 0.0 --head-pitch 0.5

我方的 ``base_frame -> 相机光心`` 默认从实时 TF 读；没有控制栈时用 ``--camera-in-base``
手工给一个 4x4。

相机是锚点：模型的空间感建立在它自己相机的位姿上，与两台机器人的手臂长度无关。工具会
打印两套参数——**[A] 只挪原点**让两台相机的**位置**重合、模型系保持水平（推荐，这个 VLA
对相机位置最敏感）；**[B] 完整六自由度**连朝向一起对上，代价是模型系被掰斜、重力方向错。

A2D 专有的常量（头部外参、训练原点高度）在 ``backends/a2d_omnipicker.py`` 里，
换训练机改那边。
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import numpy as np

from g1_vla_bridge.backends.a2d_omnipicker import HEAD_TO_CAM, TRAIN_ORIGIN_Z
from g1_vla_bridge.transforms import (
    mat_to_rpy,
    pose_matrix,
    solve_base_frame,
    solve_origin_position,
)
from g1_vla_bridge.vla_backend import FrameSpec


def _load_matrix(text: str) -> np.ndarray:
    """接受 4x4 的 JSON 字面量或文件路径。"""
    try:
        data = json.loads(text)
    except ValueError:
        with open(text, 'r', encoding='utf-8') as handle:
            data = json.load(handle)
    if isinstance(data, dict):
        for key in ('head_camera_in_world', 'matrix', 'T'):
            if key in data:
                data = data[key]
                break
    return np.asarray(data, dtype=np.float64).reshape(4, 4)


def camera_in_world_from_joints(urdf: str, lift: float, body_pitch: float,
                                head_yaw: float, head_pitch: float) -> np.ndarray:
    """用 A2D 的 URDF + 外参算 T_训练系←相机。只在没拿到真实 4x4 时用。"""
    import pinocchio as pin

    model = pin.buildModelFromUrdf(urdf)
    data = model.createData()
    q = pin.neutral(model)
    for name, value in (('joint_lift_body', lift), ('joint_body_pitch', body_pitch),
                        ('joint_head_yaw', head_yaw), ('joint_head_pitch', head_pitch)):
        q[model.idx_qs[model.getJointId(name)]] = value
    pin.forwardKinematics(model, data, q)
    head = data.oMi[model.getJointId('joint_head_pitch')]
    cam = np.eye(4)
    cam[:3, :3] = head.rotation
    cam[:3, 3] = head.translation
    cam = cam @ HEAD_TO_CAM
    cam[2, 3] -= TRAIN_ORIGIN_Z
    return cam


def camera_in_base_from_tf(base_frame: str, camera_frame: str, timeout: float) -> np.ndarray:
    import rclpy
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = Node('calibrate_frame')
    buffer = Buffer()
    TransformListener(buffer, node)
    deadline = time.monotonic() + timeout
    while (time.monotonic() < deadline
           and not buffer.can_transform(base_frame, camera_frame, rclpy.time.Time())):
        rclpy.spin_once(node, timeout_sec=0.1)
    try:
        if not buffer.can_transform(base_frame, camera_frame, rclpy.time.Time()):
            raise SystemExit(f'{timeout:.0f} s 内等不到 TF {base_frame} -> {camera_frame}')
        tf = buffer.lookup_transform(base_frame, camera_frame, rclpy.time.Time()).transform
        return pose_matrix([tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w],
                           [tf.translation.x, tf.translation.y, tf.translation.z])
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _pitch_deg(matrix) -> float:
    """相机俯角：光学 +Z 与水平面的夹角，正为低头。"""
    return float(np.degrees(np.arcsin(-matrix[2, 2])))


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--camera-in-world', help='训练侧 head_camera_in_world，4x4 JSON 或文件')
    parser.add_argument('--camera-in-base', help='我方 base_frame -> 相机光心，4x4 JSON 或文件')
    parser.add_argument('--a2d-urdf',
                        default='/workspace/src/g1_vla_bridge/A2D_Omnipicker/A2D.urdf')
    parser.add_argument('--lift', type=float, help='训练时 joint_lift_body')
    parser.add_argument('--body-pitch', type=float, default=0.0)
    parser.add_argument('--head-yaw', type=float, default=0.0)
    parser.add_argument('--head-pitch', type=float, default=0.0)
    parser.add_argument('--base-frame', default='torso_link')
    parser.add_argument('--camera-frame', default='camera_color_optical_frame')
    parser.add_argument('--tf-timeout', type=float, default=15.0)
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    if args.camera_in_world:
        model_cam = _load_matrix(args.camera_in_world)
        source = '对面给的 head_camera_in_world'
    elif args.lift is not None:
        model_cam = camera_in_world_from_joints(
            args.a2d_urdf, args.lift, args.body_pitch, args.head_yaw, args.head_pitch)
        source = ('A2D URDF 正解 lift=%.3f bpitch=%.3f hyaw=%.3f hpitch=%.3f'
                  % (args.lift, args.body_pitch, args.head_yaw, args.head_pitch))
    else:
        raise SystemExit('必须给 --camera-in-world，或给 --lift 等关节值由 URDF 反算')

    if args.camera_in_base:
        base_cam = _load_matrix(args.camera_in_base)
    else:
        base_cam = camera_in_base_from_tf(args.base_frame, args.camera_frame, args.tf_timeout)

    tilt = float(np.degrees(np.linalg.norm(mat_to_rpy(
        model_cam[:3, :3] @ base_cam[:3, :3].T))))
    full = FrameSpec.from_solution(*solve_base_frame(model_cam, base_cam))
    origin = solve_origin_position(model_cam, base_cam)

    print('训练侧相机 (%s)' % source)
    print('  位置 %s   俯角 %+.1f度' % (np.round(model_cam[:3, 3], 4), _pitch_deg(model_cam)))
    print('我方相机 (%s -> %s)' % (args.base_frame, args.camera_frame))
    print('  位置 %s   俯角 %+.1f度' % (np.round(base_cam[:3, 3], 4), _pitch_deg(base_cam)))
    print('  两台相机朝向差 %.1f 度' % tilt)

    print('\n[A] 只对位置（推荐）：模型系保持水平朝前，原点挪到两台相机重合\n')
    print('    model_origin_in_base: [%.4f, %.4f, %.4f]' % tuple(origin))
    print('    model_rotation_rpy: [0.0, 0.0, 0.0]')
    print('    # 相机朝向仍差 %.1f 度，靠 head_reproject 和模型自身鲁棒性吸收' % tilt)

    print('\n[B] 完整六自由度：位置和朝向都重合，但模型系被掰斜 %.1f 度\n' % tilt)
    print('    model_origin_in_base: [%.4f, %.4f, %.4f]' % full.origin_in_base)
    print('    model_rotation_rpy: [%.6f, %.6f, %.6f]' % full.rotation_rpy)
    print('    # 掰斜之后重力方向在模型系里是错的，末端 state 会跟着歪')

    print('\n（head_pitch 每 0.1 rad 会让原点变约 0.047 m，务必用真实采样而不是猜的关节值）')


if __name__ == '__main__':
    main()
