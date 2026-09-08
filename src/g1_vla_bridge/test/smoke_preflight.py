#!/usr/bin/env python3
"""只读预检：三路观测是否齐、推理耗时、双臂首点偏差与整段位移。

    source install/setup.bash
    python3 src/g1_vla_bridge/test/smoke_preflight.py --rounds 6 --task "Pick up the cup"

**不发任何指令**。使用同一个 backend 和 Observation 格式，不验证执行器或闭环跟踪。
"""

from __future__ import annotations

import argparse
import json
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from g1_vla_bridge.transforms import pose_matrix, quat_angle
from g1_vla_bridge.vla_backend import SIDES, Observation, backend_parameters, load_backend
from g1_vla_bridge.vla_node import (CAMERA_FRAMES, CAMERA_INFO_TOPICS, IMAGE_TOPICS,
                                    camera_calibration, image_to_bgr)


def load_config() -> dict:
    """通用那份 + 选中 backend 的那份，与 launch 的加载顺序一致。"""
    config_dir = get_package_share_directory('g1_vla_bridge') + '/config'
    with open(config_dir + '/vla_bridge.yaml', 'r', encoding='utf-8') as handle:
        cfg = yaml.safe_load(handle)['/vla_bridge']['ros__parameters']
    path = '%s/backends/%s.yaml' % (config_dir, cfg['vla_backend'])
    with open(path, 'r', encoding='utf-8') as handle:
        cfg.update(yaml.safe_load(handle)['/vla_bridge']['ros__parameters'])
    return cfg


def gather(cfg, slots, timeout: float, warmup: float):
    """等齐图像、内参、TF 和 motion_control 状态。

    相机没订阅者就断流，本脚本一订阅相当于重新拉流；起流后要等第一个 H.264 关键帧
    才解得出画面，实测前 8 帧梯度只有 0.04~4.5。**不预热就会拿废帧去打推理。**
    持久运行的 `vla_node` 没这个问题，它从头到尾只订阅一次。
    """
    node = Node('vla_preflight')
    tf = Buffer()
    TransformListener(tf, node)
    frames, info, status, counts = {}, {}, {}, {}
    image_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                           reliability=ReliabilityPolicy.RELIABLE)
    small = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.BEST_EFFORT)

    def image_callback(key):
        def callback(msg):
            frames[key] = msg
            counts[key] = counts.get(key, 0) + 1
        return callback

    for slot, param, default in IMAGE_TOPICS:
        if slot in slots:
            node.create_subscription(Image, cfg.get(param, default),
                                     image_callback(slot), image_qos)
    for slot, param, default in CAMERA_INFO_TOPICS:
        if slot in slots:
            node.create_subscription(
                CameraInfo, cfg.get(param, default),
                lambda message, key=slot: info.__setitem__(key, message), small)
    node.create_subscription(String, cfg['status_topic'],
                             lambda m: status.update(json.loads(m.data)), 10)
    needed = (cfg['head_camera_frame'], cfg['left_camera_frame'],
              cfg['right_camera_frame'], cfg['left_tip_frame'], cfg['right_tip_frame'])
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and (
            len(frames) < len(slots) or len(info) < len(slots) or not status or not all(
                tf.can_transform(cfg['base_frame'], f, rclpy.time.Time()) for f in needed)):
        rclpy.spin_once(node, timeout_sec=0.1)
    missing_tf = [f for f in needed
                  if not tf.can_transform(cfg['base_frame'], f, rclpy.time.Time())]
    missing_images = [k for k in slots if k not in frames]
    missing_info = [k for k in slots if k not in info]
    print('图像   %s' % ('齐' if not missing_images else '缺 %s' % missing_images))
    print('内参   %s' % ('齐' if not missing_info else '缺 %s' % missing_info))
    print('TF     %s' % ('齐' if not missing_tf else '缺 %s' % missing_tf))
    print('底层   %s' % ({k: status.get(k) for k in ('state', 'arms_live')} or '收不到'))
    if missing_images or missing_info or missing_tf:
        raise SystemExit('前置条件不齐，先把相机/控制栈起起来')

    print('\n预热 %.0f s 等关键帧...' % warmup)
    warm_end = time.monotonic() + warmup
    while time.monotonic() < warm_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    for key in slots:
        bgr = image_to_bgr(frames[key])
        # 解码器在等到关键帧之前吐的是中性灰（均值 129、标准差 < 2）。
        print('  %-12s %4d 帧  %dx%d  亮度 %5.1f  对比度 %5.1f'
              % (key, counts.get(key, 0), bgr.shape[1], bgr.shape[0],
                 float(bgr.mean()), float(bgr.std())))

    if not status.get('arms_live'):
        print('  提醒：arms_live 不为真，~/start 会被拒。先 ros2 service call '
              '/motion_control/engage std_srvs/srv/Trigger')
    return node, tf, frames, info, status


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--rounds', type=int, default=6)
    parser.add_argument('--task', default='Pick up the cup using the left arm.')
    parser.add_argument('--timeout', type=float, default=40.0)
    parser.add_argument('--warmup', type=float, default=4.0)
    parser.add_argument('--proxy', default='')
    args = parser.parse_args()
    if args.rounds < 1:
        parser.error('--rounds must be positive')

    cfg = load_config()
    name = cfg.get('vla_backend', 'a2d_omnipicker')
    params = dict(backend_parameters(name))
    params.update({k: v for k, v in cfg.items() if k in params})
    if args.proxy:
        params['proxy'] = args.proxy
    backend = load_backend(name, params)
    slots = backend.spec.images.slots

    rclpy.init()
    node, tf, frames, info, status = gather(cfg, slots, args.timeout, args.warmup)

    def pose7(child):
        t = tf.lookup_transform(cfg['base_frame'], child, rclpy.time.Time()).transform
        return np.array([t.translation.x, t.translation.y, t.translation.z,
                         t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])

    camera_frames = {
        slot: cfg.get(param, default)
        for slot, param, default in CAMERA_FRAMES if slot in slots
    }
    calibrations = {slot: camera_calibration(info[slot]) for slot in slots}
    camera_poses = {
        slot: pose_matrix(pose[3:], pose[:3])
        for slot, pose in ((slot, pose7(camera_frames[slot])) for slot in slots)
    }
    grip_value = status.get('grip') or []
    grip = (grip_value if isinstance(grip_value, dict)
            else dict(zip(SIDES, grip_value)))
    measured = {s: pose7(cfg['%s_tip_frame' % s]) for s in SIDES}
    observation = Observation(
        task=args.task,
        images={slot: image_to_bgr(frames[slot]) for slot in slots},
        poses=measured,
        grippers={s: float(grip.get(s, 0.0)) for s in SIDES},
        enabled={s: bool(cfg['has_%s' % s]) for s in SIDES},
        calibrations=calibrations,
        camera_poses=camera_poses)

    print('\n观测标定（相机位姿表示在 base_frame 下，线上格式由 backend 决定）')
    for slot in slots:
        fx, fy, cx, cy = calibrations[slot].intrinsics
        width, height = calibrations[slot].size
        print('  %-12s K_norm=[%.6f %.6f %.6f %.6f]  t=%s'
              % (slot, fx / width, fy / height, cx / width, cy / height,
                 np.round(camera_poses[slot][:3, 3], 4)))
    backend.debug_dir = '/tmp/vla_preflight'
    print('实际发出的 JPEG 见 /tmp/vla_preflight/')

    print('\n实测末端 左 %s' % np.round(measured['left'][:3], 3))
    try:
        rows = {side: [] for side in SIDES}
        infer_times = []
        for _ in range(args.rounds):
            started = time.monotonic()
            chunk = backend.infer(observation)
            elapsed = time.monotonic() - started
            infer_times.append(elapsed)
            for side in SIDES:
                poses = chunk.poses[side]
                rows[side].append((
                    poses[0, :3],
                    float(np.linalg.norm(poses[0, :3] - measured[side][:3])),
                    float(np.degrees(quat_angle(poses[0, 3:], measured[side][3:]))),
                    float(np.linalg.norm(poses[-1, :3] - poses[0, :3]))))
        print('%-8s 推理 %.0f±%.0f ms' % (
            name, 1e3 * float(np.mean(infer_times)),
            1e3 * float(np.std(infer_times))))
        for side in SIDES:
            pts = np.stack([row[0] for row in rows[side]])
            print(('  %-5s 首点 %s  散布 %.4f  距实测 %.3f±%.3f  姿态差 %.0f度  '
                   '整段 %.3f')
                  % (side, np.round(pts.mean(axis=0), 3),
                     float(np.linalg.norm(pts - pts.mean(axis=0), axis=1).max()),
                     float(np.mean([row[1] for row in rows[side]])),
                     float(np.std([row[1] for row in rows[side]])),
                     float(np.mean([row[2] for row in rows[side]])),
                     float(np.mean([row[3] for row in rows[side]]))))
        print('         %s' % backend.stats())

    finally:
        backend.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
