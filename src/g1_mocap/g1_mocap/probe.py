"""连上动捕，把真实骨架和重定向结果打出来。**不控制任何硬件**。

存在的理由：单测里的骨架是拿 G1 自己的 FK 造的，那只能证明解算数学自洽，
证明不了对真实 PICO 骨架成立。真实数据有三样东西是造不出来的——段长比例、
关节点定义、噪声水平，任一出问题都会让重定向静默失真。

用法::

    ros2 run g1_mocap mocap_probe                  # 头显直连本机的 18000
    ros2 run g1_mocap mocap_probe --port 18001     # 跟踪层占着默认端口时换一个

四张表，从下往上排查：

``段长``
    人的骨架尺寸和 G1 的对比。比值应该整体一致（都是 scale 上下）；某一段明显跑偏，
    说明那个关节点的语义和 SMPL 的约定不一样。
``短向量``
    定朝向用的那几个向量的长度。**这几个是最脆的**：骨盆和躯干的朝向靠它们定，
    短到几厘米时噪声会被放大成剧烈的姿态抖动，再经差分变成巨大的角速度。
``关节角``
    当前位形。人站直时应该整体接近 0（G1 零位就是直立），偏差大就是重定向不对。
``抖动``
    人静止时每个量的帧间变化幅度。这一栏才是"机器人会不会自己抖起来"的直接证据。
"""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import numpy as np
import yaml

from .kinematics import G1Kinematics
from .retarget import ARMS, LEGS, Retargeter
from .skeleton import JOINT_INDEX
from .stream import MocapStream
from .urdf import DEFAULT_URDF, resolve_package_path

# 定朝向用的短向量。骨盆和躯干的姿态全靠它们，短了就撑不住噪声。
ORIENTATION_VECTORS = (
    ('骨盆 竖直 PELVIS->SPINE1', 'PELVIS', 'SPINE1'),
    ('骨盆 侧向 RIGHT->LEFT_HIP', 'RIGHT_HIP', 'LEFT_HIP'),
    ('躯干 竖直 SPINE3->NECK', 'SPINE3', 'NECK'),
    ('躯干 侧向 RIGHT->LEFT_SHOULDER', 'RIGHT_SHOULDER', 'LEFT_SHOULDER'),
)


def load_config() -> dict:
    """从 ``config/mocap.yaml`` 读关节顺序等参数，别在这儿再抄一份。"""
    share = resolve_package_path('package://g1_mocap/config/mocap.yaml')
    document = yaml.safe_load(Path(share).read_text(encoding='utf-8'))
    return next(iter(document.values()))['ros__parameters']


def segment_table(frames, kin: G1Kinematics, joints) -> str:
    """人的骨骼段长 vs G1 的。比值整体应该一致，某一段跑偏就是关节点语义对不上。"""
    positions = np.mean([f.positions for f in frames], axis=0)
    kin.key_body_pos(np.zeros(len(joints)), ('pelvis',))
    rows = ['  段                        人(m)   G1(m)   比值']
    for group in (LEGS, ARMS):
        for side, spec in group.items():
            for i, label in ((0, '近端'), (1, '远端')):
                human = float(np.linalg.norm(
                    positions[JOINT_INDEX[spec.smpl[i + 1]]]
                    - positions[JOINT_INDEX[spec.smpl[i]]]))
                robot = float(np.linalg.norm(kin.frame_pos(spec.kin_links[i + 1])
                                             - kin.frame_pos(spec.kin_links[i])))
                name = f'{side}-{label}'
                ratio = robot / human if human > 1e-6 else float('nan')
                rows.append(f'  {name:24s} {human:6.3f}  {robot:6.3f}  {ratio:6.3f}')
    return '\n'.join(rows)


def orientation_table(frames) -> str:
    """定朝向那几个向量的长度与抖动。短向量 + 噪声 = 姿态剧烈抖动。"""
    stack = np.stack([f.positions for f in frames])
    rows = ['  向量                              长度(m)  长度抖动(mm)  方向抖动(deg)']
    for label, tail, head in ORIENTATION_VECTORS:
        vectors = stack[:, JOINT_INDEX[head]] - stack[:, JOINT_INDEX[tail]]
        lengths = np.linalg.norm(vectors, axis=-1)
        units = vectors / np.maximum(lengths[:, None], 1e-9)
        # 方向抖动取相邻帧夹角的 p95，比标准差更能反映"最坏的一跳"。
        cosines = np.clip(np.sum(units[1:] * units[:-1], axis=-1), -1.0, 1.0)
        swing = np.degrees(np.arccos(cosines))
        flag = '   <<< 太短' if float(np.mean(lengths)) < 0.08 else ''
        rows.append(f'  {label:32s} {np.mean(lengths):6.3f}  '
                    f'{np.std(lengths) * 1000:9.2f}  '
                    f'{np.percentile(swing, 95):11.3f}{flag}')
    return '\n'.join(rows)


def joint_table(joints, angles: np.ndarray, jitter: np.ndarray) -> str:
    """当前关节角 + 帧间抖动。人静止站直时，角度该接近 0，抖动该接近 0。"""
    rows = ['  关节                            角度(deg)  抖动p95(deg/帧)']
    for i, name in enumerate(joints):
        flag = '   <<<' if jitter[i] > 1.0 else ''
        rows.append(f'  {name:30s} {math.degrees(angles[i]):8.2f}  '
                    f'{math.degrees(jitter[i]):12.3f}{flag}')
    return '\n'.join(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=18000)
    parser.add_argument('--token', default='')
    parser.add_argument('--urdf', default=DEFAULT_URDF)
    parser.add_argument('--seconds', type=float, default=3.0,
                        help='采样多久后出报告')
    parser.add_argument('--wait', type=float, default=0.0,
                        help='等头显的秒数，0 表示一直等（Ctrl-C 退出）')
    args = parser.parse_args()

    config = load_config()
    joints = list(config['joints'])
    kin = G1Kinematics(resolve_package_path(args.urdf), joints)
    retargeter = Retargeter(kin, key_bodies=list(config['key_bodies']),
                            anchor_body=config['anchor_body'],
                            default_joint_pos=np.asarray(config['default_joint_pos']))
    stream = MocapStream(retargeter, host=args.host, port=args.port,
                         token=args.token, log=print)
    stream.start()
    print(f'等头显连上…（端口 {args.port}）')
    print('头显那边在 PicoBridge 的配置面板里点一下「连接」。Ctrl-C 退出。')

    try:
        deadline = time.monotonic() + args.wait if args.wait > 0 else float('inf')
        reported = 0.0
        while stream.frame_count() < 40:
            if time.monotonic() > deadline:
                print('等超时了。检查头显是否连上、body.status 是否 VALID')
                return
            now = time.monotonic()
            if now - reported > 5.0:
                reported = now
                stats = stream.stats()
                # 分清两件事：链路没通，还是通了但 body 数据不可用（没戴好 / 没校准）。
                print(f'  [{time.strftime("%H:%M:%S")}] {stream.describe_status()}'
                      f' 可用帧={stream.frame_count()}'
                      + ('' if stats.connected else '   <<< 头显还没连进来'))
            time.sleep(0.2)

        print('\n人**站直不动**，采样 %.1f 秒…' % args.seconds)
        time.sleep(args.seconds)
        frames = stream.recent_frames()
        if len(frames) < 40:
            print('帧数不够，可能中途断流了')
            return

        print('\n=== 段长：人 vs G1 ===')
        print(segment_table(frames, kin, joints))
        print('\n=== 定朝向的短向量（最脆的一环）===')
        print(orientation_table(frames))

        calibration = retargeter.calibrate(frames)
        print(f'\n=== 校准 ===\n  缩放 {calibration.scale:.4f}   '
              f'站立高度 {calibration.stand_height:.4f} m   '
              f'站姿零位最大偏置 '
              f'{math.degrees(np.abs(calibration.joint_bias).max()):.2f} deg')

        solved = np.stack([retargeter.solve(f, calibration).joint_pos for f in frames])
        jitter = np.percentile(np.abs(np.diff(solved, axis=0)), 95, axis=0)
        print('\n=== 关节角（人站直时应整体接近 0，G1 零位就是直立）===')
        print(joint_table(joints, solved[-1], jitter))

        roots = np.stack([retargeter.solve(f, calibration).root_pos for f in frames])
        speed = np.linalg.norm(np.diff(roots, axis=0), axis=-1) / 0.02
        print(f'\n=== 根位置抖动 ===\n  折算速度 p50={np.percentile(speed, 50):.3f} '
              f'p95={np.percentile(speed, 95):.3f} max={speed.max():.3f} m/s')
        print('  参考窗口里的线速度就是这么差分出来的，静止时应该远小于 0.1 m/s')
    except KeyboardInterrupt:
        print('\n已退出')
    finally:
        stream.stop()


if __name__ == '__main__':
    main()
