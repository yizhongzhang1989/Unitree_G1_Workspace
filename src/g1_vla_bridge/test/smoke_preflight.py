#!/usr/bin/env python3
"""起飞前检查：链路是否齐、模型首点离实测多远、重投影开关值不值得开。

    source install/setup.bash
    python3 src/g1_vla_bridge/test/smoke_preflight.py --rounds 6 --task "Pick up the cup"

**不发任何指令**，纯读。走的是和 ``vla_node`` 完全同一条路径（同一个 backend、同一份
``Observation``），所以这里跑通就等于节点能跑通。

判据：模型自身噪声约 0.037 m（同一请求重复测得），所以两个变体的均值差要明显大于它
才说明重投影真的有用。
"""

from __future__ import annotations

import argparse
import json
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from g1_vla_bridge.backends.a2d_omnipicker import describe_reprojection
from g1_vla_bridge.transforms import pose_matrix, quat_angle
from g1_vla_bridge.vla_backend import SIDES, Observation, backend_parameters, load_backend
from g1_vla_bridge.vla_node import IMAGE_TOPICS, camera_calibration, image_to_bgr

MODEL_NOISE = 0.037


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
    node.create_subscription(CameraInfo, cfg['head_camera_info_topic'],
                             lambda m: info.setdefault('m', m), small)
    node.create_subscription(String, cfg['status_topic'],
                             lambda m: status.update(json.loads(m.data)), 10)
    needed = (cfg['camera_optical_frame'], cfg['left_tip_frame'], cfg['right_tip_frame'])
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and (
            len(frames) < len(slots) or not info or not status or not all(
                tf.can_transform(cfg['base_frame'], f, rclpy.time.Time()) for f in needed)):
        rclpy.spin_once(node, timeout_sec=0.1)
    missing_tf = [f for f in needed
                  if not tf.can_transform(cfg['base_frame'], f, rclpy.time.Time())]
    missing_images = [k for k in slots if k not in frames]
    print('图像   %s' % ('齐' if not missing_images else '缺 %s' % missing_images))
    print('内参   %s' % ('有' if info else '缺 %s' % cfg['head_camera_info_topic']))
    print('TF     %s' % ('齐' if not missing_tf else '缺 %s' % missing_tf))
    print('底层   %s' % ({k: status.get(k) for k in ('state', 'arms_live')} or '收不到'))
    if missing_images or not info or missing_tf:
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
    return node, tf, frames, info['m'], status


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--rounds', type=int, default=6)
    parser.add_argument('--task', default='Pick up the cup using the left arm.')
    parser.add_argument('--timeout', type=float, default=40.0)
    parser.add_argument('--warmup', type=float, default=4.0)
    args = parser.parse_args()

    cfg = load_config()
    name = cfg.get('vla_backend', 'a2d_omnipicker')
    params = dict(backend_parameters(name))
    params.update({k: v for k, v in cfg.items() if k in params})
    variants = {'不重投影': dict(params, head_reproject=False),
                '重投影': dict(params, head_reproject=True)}
    backends = {label: load_backend(name, value) for label, value in variants.items()}
    slots = backends['重投影'].spec.images.slots

    rclpy.init()
    node, tf, frames, info, status = gather(cfg, slots, args.timeout, args.warmup)

    def pose7(child):
        t = tf.lookup_transform(cfg['base_frame'], child, rclpy.time.Time()).transform
        return np.array([t.translation.x, t.translation.y, t.translation.z,
                         t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])

    camera = camera_calibration(info)
    cam_pose = pose7(cfg['camera_optical_frame'])
    grip = status.get('grip') or {}
    measured = {s: pose7(cfg['%s_tip_frame' % s]) for s in SIDES}
    observation = Observation(
        task=args.task,
        images={slot: image_to_bgr(frames[slot]) for slot in slots},
        poses=measured,
        grippers={s: float(grip.get(s, 0.0)) for s in SIDES},
        enabled={s: bool(cfg['has_%s' % s]) for s in SIDES},
        camera_in_base=pose_matrix(cam_pose[3:], cam_pose[:3]),
        camera=camera)

    print('\n%s' % describe_reprojection(camera))
    for label, backend in backends.items():
        backend.debug_dir = '/tmp/preflight_%s' % ('reproj' if '重' in label else 'raw')
    cv2.imwrite('/tmp/preflight_raw.png', observation.images['head'])
    print('头部原图已存 /tmp/preflight_raw.png；实际发出的 JPEG 见 /tmp/preflight_*/')

    print('\n实测末端 左 %s' % np.round(measured['left'][:3], 3))
    first_points = {}
    for label, backend in backends.items():
        rows = []
        for _ in range(args.rounds):
            chunk = backend.infer(observation)
            poses = chunk.poses['left']
            rows.append((poses[0, :3],
                         float(np.linalg.norm(poses[0, :3] - measured['left'][:3])),
                         float(np.degrees(quat_angle(poses[0, 3:], measured['left'][3:]))),
                         float(np.linalg.norm(poses[-1, :3] - poses[0, :3]))))
        pts = np.stack([r[0] for r in rows])
        first_points[label] = pts.mean(axis=0)
        print('%-8s 首点均值 %s  自身散布 %.4f  距实测 %.3f±%.3f  姿态差 %.0f度  整段位移 %.3f'
              % (label, np.round(pts.mean(axis=0), 3),
                 float(np.linalg.norm(pts - pts.mean(axis=0), axis=1).max()),
                 float(np.mean([r[1] for r in rows])), float(np.std([r[1] for r in rows])),
                 float(np.mean([r[2] for r in rows])), float(np.mean([r[3] for r in rows]))))
        print('         %s' % backend.stats())

    shift = float(np.linalg.norm(first_points['重投影'] - first_points['不重投影']))
    print('\n两个变体的首点相差 %.4f m（模型自身噪声 %.3f m）-> %s'
          % (shift, MODEL_NOISE,
             '重投影确实改变了模型判断，值得按抓取成功率进一步验证'
             if shift > 2 * MODEL_NOISE else '差异淹没在噪声里，重投影暂时看不出收益'))

    for backend in backends.values():
        backend.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
