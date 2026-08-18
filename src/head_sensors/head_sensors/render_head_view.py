"""URDF 拍照的命令行入口 —— **不依赖 ROS**。

连同 `urdf_view.py` 一起拷走就能独立运行，只要 `pinocchio`、`numpy`、`opencv`：

    python3 render_head_view.py --urdf final.urdf --out /tmp/hv
    python3 render_head_view.py --urdf final.urdf --pose shot.json --out /tmp/hv
    python3 render_head_view.py --urdf final.urdf --frame d435_link \\
        --hfov 120 --vfov 90 --width 640 --height 480 --out /tmp/wide

`--pose` 是一个 JSON，描述「以什么姿态、用什么相机拍」：

    {
      "frame": "d435_link",
      "camera": {"width": 848, "height": 480,
                 "fx": 608.451, "fy": 608.771, "cx": 430.086, "cy": 247.547},
      "joints": {"left_elbow_joint": 0.4762, "right_elbow_joint": 0.4650},
      "extrinsic": [[...4x4...]]
    }

`joints` 里没列到的关节保持 URDF 中立位。`extrinsic` 是 `frame` 到真实成像中心的
4x4，可以不给，不给就当相机就坐在 `frame` 原点上。不给 `--pose` 就全用中立位加默认内参。
"""

import argparse
import os
import sys

import numpy as np

try:
    from head_sensors.urdf_view import DEFAULT_CAMERA, PinholeCamera, load_pose, shoot
except ImportError:  # 直接 `python3 render_head_view.py` 跑，不经过包
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from urdf_view import DEFAULT_CAMERA, PinholeCamera, load_pose, shoot


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--urdf', required=True, help='整机 URDF；mesh 路径相对它所在目录')
    parser.add_argument('--pose', default=None, help='姿态 JSON，见模块文档')
    parser.add_argument('--frame', default=None,
                        help='相机所在的 URDF frame，默认 d435_link')
    parser.add_argument('--link-frame', action='store_true',
                        help='按 link 坐标系(x前)而不是 ROS 光学坐标系(z前)解释朝向')
    parser.add_argument('--hfov', type=float, default=None, help='按 FOV 造内参，覆盖 --pose')
    parser.add_argument('--vfov', type=float, default=None)
    parser.add_argument('--width', type=int, default=None)
    parser.add_argument('--height', type=int, default=None)
    package_dir = os.path.dirname(os.path.abspath(__file__))
    background_candidates = [
        os.path.join(package_dir, 'resource', 'bg.jpg'),
        os.path.join(package_dir, os.pardir, 'resource', 'bg.jpg'),
        os.path.join(package_dir, os.pardir, os.pardir, os.pardir, os.pardir,
                     'share', 'head_sensors', 'resource', 'bg.jpg'),
    ]
    default_background = next((os.path.abspath(path) for path in background_candidates
                               if os.path.isfile(path)), None)
    parser.add_argument('--background',
                        default=default_background,
                        help='彩色图背景图片；默认使用 resource/bg.jpg')
    parser.add_argument('--out', default='/tmp/head_view')
    args = parser.parse_args(argv)

    joints, camera, frame, extrinsic = None, DEFAULT_CAMERA, 'd435_link', None
    if args.pose:
        joints, camera, frame, extrinsic = load_pose(args.pose)
        print('姿态          : %s（%d 个关节%s）'
              % (args.pose, len(joints), '' if extrinsic is None else '，带实测外参'))
    if args.frame:
        frame = args.frame
    if args.hfov and args.vfov:
        camera = PinholeCamera.from_fov(args.width or camera.width,
                                        args.height or camera.height,
                                        args.hfov, args.vfov)

    print('URDF          : %s' % args.urdf)
    # camera 是 NamedTuple（tuple 子类），% 会把它当参数列表展开，必须再包一层。
    print('相机          : %s' % (camera,))
    shot = shoot(joints, urdf=args.urdf, camera=camera, frame=frame,
                 optical=not args.link_frame, extrinsic=extrinsic,
                 background=args.background, out=args.out)

    hit = np.isfinite(shot.depth)
    print('%-13s : 世界系位置 [%.3f %.3f %.3f]' % (frame, *shot.cam_pos))
    print('命中像素      : %d / %d (%.1f%%)' % (hit.sum(), hit.size, 100.0 * hit.mean()))
    if hit.any():
        print('深度范围      : %.3f ~ %.3f m' % (shot.depth[hit].min(), shot.depth[hit].max()))
        seen = [shot.names[i] for i in np.unique(shot.label) if i >= 0]
        print('可见部件(%d)   : %s' % (len(seen), ', '.join(seen)))
    print('输出          : %s_{color,depth16,depth,parts}.png' % args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
