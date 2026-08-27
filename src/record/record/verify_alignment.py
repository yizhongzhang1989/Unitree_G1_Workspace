"""离线校验：把 URDF 渲染的机器人轮廓 + IK 目标点叠回录下来的头部视频上。

回答的是「这次采集的几路数据彼此对得齐吗」，**不是**「渲染得像不像真的」。
判据全在图上：

* **URDF 轮廓（左绿 / 右红）压不上真实手臂** —— 关节角与视频的时间对不齐，
  或者头部相机的挂载外参不对（`calibration.yaml` 的 `d435_joint`）。
* **目标点（深色空心圆）离实际末端（亮色实心点）很远且方向乱飘** —— 指令表与关节表
  不在同一时间基准上，或者位姿根本不是 `torso_link` 系的。
  正常情况是目标稳定地领先实际一小段（控制跟随滞后），两者的接近方向射线大致同向。

控制台还会给出一份数值报告，其中「最佳时移」是最硬的那条：把指令整体平移多少秒
才让它离实际末端最近。它应该等于控制跟随的滞后（百毫秒量级）。要是算出几秒钟、
或者是个负得离谱的数，那就不是跟随滞后，是两张表的时间戳串了。

    ros2 run record verify_alignment 20260827_022837                 # 第一条 episode
    ros2 run record verify_alignment 20260827_022837 --episode 0:3
    ros2 run record verify_alignment 20260827_022837 --whole --fps 5

渲染走 `head_sensors.urdf_view` 的 `silhouette()`（只出轮廓，比逐像素 z-buffer 快
约 40 倍），实测 640x360 每帧 0.32 s —— **离实时差两个数量级**，所以这是个离线
一次性渲染的工具，不要指望它跟着回放同步出画。
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path
from xml.etree import ElementTree

import cv2
import numpy as np
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from head_sensors.urdf_view import PinholeCamera, UrdfSceneRenderer
from record.replay import GRIP, LEFT, RIGHT

sys.path.insert(0, str(Path(get_package_share_directory('record')) / 'tools'))
import urdf_fk                                             # noqa: E402
from session_reader import Session                         # noqa: E402

#: 指令位姿的参考系与末端 link，取自 `g1_motion_control/config/motion_control.yaml`。
BASE_FRAME = 'torso_link'
TIP = {'left': 'left_gripper_base', 'right': 'right_gripper_base'}
#: BGR。和 `verify_head_view` 一致：左绿右红。轮廓与实测末端用亮色，
#: **IK 指令的目标用同色系的深色** —— 对得准时目标和实测就叠在一起，
#: 同色同亮度的话两个标记互相遮掉，深浅是最容易分辨的那一维。
COLOR = {'left': (80, 255, 80), 'right': (80, 80, 255)}
DARK = {'left': (0, 110, 0), 'right': (0, 0, 150)}
#: 夹爪的接近方向（`motion_control.yaml` 的 `tool_axis: gripper_base +Z`）画多长，米。
APPROACH_M = 0.06


# --------------------------------------------------------------------- 模型


def build_renderer(urdf: Path, calibration: Path | None) -> tuple:
    """加载 URDF 并叠上标定的关节 origin，返回 `(渲染器, 模型, 生效的覆盖)`。

    必须叠 `calibration.yaml` —— 控制栈是在 launch 里把它打进内存 URDF 的
    （见 `unitree_g1_ros2_control/launch/control.launch.py`），磁盘上那份 submodule
    里的 `d435_joint` 还是名义值。不叠就是拿另一个相机位姿去对，轮廓必然偏。
    """
    root = ElementTree.fromstring(urdf.read_text(encoding='utf-8'))
    overrides = {}
    if calibration is not None and calibration.is_file():
        import yaml
        overrides = (yaml.safe_load(calibration.read_text(encoding='utf-8'))
                     or {}).get('urdf_overrides') or {}
    applied = _apply_origins(root, overrides)

    # pinocchio 只吃文件。写临时 URDF 时 mesh 仍按原目录解析，所以 mesh_dir 给源目录。
    with tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False) as fp:
        fp.write(ElementTree.tostring(root, encoding='unicode'))
        patched = Path(fp.name)
    try:
        renderer = UrdfSceneRenderer(str(patched), mesh_dir=str(urdf.parent))
    finally:
        patched.unlink()
    return renderer, urdf_fk.RobotModel.from_urdf(urdf), applied


def _apply_origins(root, overrides: dict) -> list[str]:
    """只改 URDF 里已有的那些 joint 的 origin，判据与 control.launch.py 相同。

    `create: true` 的那几条（腕相机光心）跳过：它们只是新增叶子 frame，
    不动任何已有 link 的位姿，对头部视角的渲染没有影响。
    """
    applied = []
    for joint in root.iter('joint'):
        entry = overrides.get(joint.get('name'))
        if not entry or 'xyz' not in entry or 'rpy' not in entry:
            continue
        # 存的是 T_parent<-child，挂点对不上就整条跳过，不硬套。
        if any(entry.get(tag) and joint.find(tag) is not None
               and joint.find(tag).get('link') != entry[tag]
               for tag in ('parent', 'child')):
            continue
        origin = joint.find('origin')
        if origin is None:
            origin = ElementTree.SubElement(joint, 'origin')
        origin.set('xyz', ' '.join(str(v) for v in entry['xyz']))
        origin.set('rpy', ' '.join(str(v) for v in entry['rpy']))
        applied.append(joint.get('name'))
    return applied


def head_camera(meta: dict, width: int) -> PinholeCamera:
    """meta.json 里录着的头部彩色内参，等比缩到 `width`。

    内参必须跟着缩放走。头部这一路是 1280x720（16:9），等比缩是安全的 —— 不像
    424x240 与 640x480 那种裁剪关系（见 head_sensors 的 README）。
    """
    info = meta.get('head_camera_info') or {}
    if not info.get('k'):
        raise RuntimeError('meta.json 里没有 head_camera_info，这次采集没录头部相机内参')
    scale = width / info['width']
    height = int(round(info['height'] * scale))
    k = info['k']
    return PinholeCamera(width, height - height % 2,
                         k[0] * scale, k[4] * scale, k[2] * scale, k[5] * scale)


def optical_extrinsic(meta: dict) -> np.ndarray:
    """`d435_link -> camera_color_optical_frame` 的 4x4。

    彩色镜头不在挂载 link 原点上（偏 15.3 mm），当成纯旋转会让整幅渲染横移十几像素。
    """
    spec = meta.get('head_optical') or urdf_fk.HEAD_OPTICAL
    x, y, z, w = spec['quat']
    out = np.eye(4)
    out[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                   [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                   [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    out[:3, 3] = spec['xyz']
    return out


def expand_mimic(model, positions: dict) -> dict:
    """把 mimic 关节按源关节算出来，**并夹到各段自己的 `<limit>`**。

    G1 每只夹爪有 32 条 mimic 到 `*_eccentric_joint` 的指节。pinocchio 不认 mimic，
    漏算的话夹爪永远渲染成中立位 —— 而抓取任务要看的正是夹爪。
    夹限位那一步同样不能省，理由见 `urdf_fk.mimic_clamp`：那 8 段串联是靠逐段饱和
    拼出分段线性样条的，不夹就退化成直线，开口只剩一半。
    """
    out = dict(positions)
    for name, joint in model.joints.items():
        if joint.mimic is None:
            continue
        source, multiplier, offset = joint.mimic
        if source in positions:
            out[name] = float(urdf_fk.mimic_clamp(
                positions[source] * multiplier + offset, joint.limit))
    return out


# ----------------------------------------------------------------- 时间取样


def sample_at(t_src: np.ndarray, data: np.ndarray, when: np.ndarray,
              interpolate: bool):
    """把 `data` 取到 `when` 上，返回 `(值, 该时刻离最近样本多久)`。

    关节角插值（信号连续，插值比就近取更准）；指令**零阶保持**（指令是收到那一刻
    生效的，两条指令之间机器人执行的就是前一条，插出来的中间值从没被下发过）。
    """
    index = np.clip(np.searchsorted(t_src, when, side='right') - 1, 0, t_src.size - 1)
    age = when - t_src[index]
    if not interpolate:
        return data[index], age
    upper = np.minimum(index + 1, t_src.size - 1)
    span = np.where(t_src[upper] > t_src[index], t_src[upper] - t_src[index], 1.0)
    u = np.clip((when - t_src[index]) / span, 0.0, 1.0)[:, None]
    return data[index] * (1.0 - u) + data[upper] * u, age


def best_shift(t_cmd: np.ndarray, target: np.ndarray,
               when: np.ndarray, actual: np.ndarray) -> tuple[float, float]:
    """指令整体平移多少秒时离实际末端最近，返回 `(时移, 那时的中位距离)`。

    正数 = 指令领先实际，也就是控制跟随的滞后。**这是最能说明两张表同不同基准的
    一个数** —— 跟随滞后是百毫秒量级的，算出几秒就说明时间戳串了。
    """
    grid = np.arange(-1.0, 1.0001, 0.02)
    scores = []
    for shift in grid:
        shifted, _ = sample_at(t_cmd, target, when - shift, interpolate=False)
        scores.append(np.median(np.linalg.norm(shifted - actual, axis=1)))
    pick = int(np.argmin(scores))
    return float(grid[pick]), float(scores[pick])


# --------------------------------------------------------------------- 视频


def decode(mkv: Path, first: int, count: int, width: int, height: int):
    """从第 `first` 帧起连着解 `count` 帧，逐帧产出 BGR。

    容器时间戳是从 0 起的 30 fps 均匀栅格（头部这一路是逐帧喂给 ffmpeg 软编的），
    所以帧号乘帧周期就能 `-ss` 过去。退半帧是为了让「第一个不早于 T 的帧」正好
    落在 `first` 上 —— 实测与按帧号 `select` 取出来的逐字节一致。
    """
    proc = subprocess.Popen(
        ['ffmpeg', '-nostdin', '-v', 'error', '-ss', f'{max(first - 0.5, 0.0) / 30.0:.4f}',
         '-i', str(mkv), '-frames:v', str(count), '-vf', f'scale={width}:{height}',
         '-vsync', '0', '-f', 'rawvideo', '-pix_fmt', 'bgr24', 'pipe:1'],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.DEVNULL)
    size = width * height * 3
    try:
        while True:
            blob = proc.stdout.read(size)
            if len(blob) < size:
                break
            yield np.frombuffer(blob, np.uint8).reshape(height, width, 3)
        # 只在正常读完时判退出码。消费者提前不要了（多半是写端先崩），ffmpeg 会
        # 因为管道断掉非零退出 —— 那时候再抛一个「解码失败」只会把真正的错因盖住。
        if proc.wait() != 0:
            raise RuntimeError('解码失败: '
                               + proc.stderr.read().decode('utf-8', 'replace')[:300])
    finally:
        proc.stdout.close()
        proc.stderr.close()
        if proc.poll() is None:
            proc.kill()
            proc.wait()


class Writer:
    """rawvideo 喂给 ffmpeg 编 H.264。"""

    def __init__(self, path: Path, width: int, height: int, fps: float) -> None:
        self.proc = subprocess.Popen(
            ['ffmpeg', '-nostdin', '-v', 'error', '-y', '-f', 'rawvideo',
             '-pix_fmt', 'bgr24', '-s', f'{width}x{height}', '-r', f'{fps:g}',
             '-i', 'pipe:0', '-c:v', 'libx264', '-preset', 'veryfast',
             '-crf', '23', '-pix_fmt', 'yuv420p', str(path)],
            stdin=subprocess.PIPE, stderr=subprocess.PIPE)

    def write(self, image: np.ndarray) -> None:
        self.proc.stdin.write(image.tobytes())

    def close(self) -> None:
        self.proc.stdin.close()
        if self.proc.wait() != 0:
            raise RuntimeError('编码失败: '
                               + self.proc.stderr.read().decode('utf-8', 'replace')[:300])


# --------------------------------------------------------------------- 画面


def project(points: np.ndarray, cam_rot: np.ndarray, cam_pos: np.ndarray,
            camera: PinholeCamera) -> np.ndarray | None:
    """世界系点 -> 像素。任何一个点在相机后面就返回 None（整组不画）。"""
    cam = (np.atleast_2d(points) - cam_pos) @ cam_rot
    if (cam[:, 2] <= 0.02).any():
        return None
    return np.stack([camera.fx * cam[:, 0] / cam[:, 2] + camera.cx,
                     camera.fy * cam[:, 1] / cam[:, 2] + camera.cy], axis=1)


def draw_contours(image: np.ndarray, label: np.ndarray, names: list) -> None:
    for side, bgr in COLOR.items():
        ids = [i for i, name in enumerate(names) if name.startswith(side + '_')]
        contours, _ = cv2.findContours(np.isin(label, ids).astype(np.uint8),
                                       cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(image, contours, -1, bgr, 1)


def draw_pose(image: np.ndarray, matrix: np.ndarray, bgr, hollow: bool,
              cam_rot: np.ndarray, cam_pos: np.ndarray, camera: PinholeCamera):
    """画一个末端位姿：一个点 + 一条 +Z 接近方向的射线。返回它的像素位置。

    只画接近方向这一根轴，不画三轴 gizmo —— 三轴的红绿蓝会和「左绿右红」撞车，
    而抓取要判的就是「伸向哪儿」。目标画成大一圈的深色空心圆，对齐得好时
    实测那个亮色实心点正好在它圆心里。
    """
    tip = matrix[:3, 3] + matrix[:3, 2] * APPROACH_M
    pixels = project(np.stack([matrix[:3, 3], tip]), cam_rot, cam_pos, camera)
    if pixels is None:
        return None
    origin, head = np.round(pixels).astype(int)
    cv2.line(image, tuple(origin), tuple(head), bgr, 2 if hollow else 1, cv2.LINE_AA)
    if hollow:
        cv2.circle(image, tuple(origin), 9, bgr, 2, cv2.LINE_AA)
    else:
        cv2.circle(image, tuple(origin), 3, bgr, -1, cv2.LINE_AA)
    return origin


def draw_text(image: np.ndarray, lines: list, x: int, y: int) -> None:
    """cv2 没有 CJK 字型，图上一律 ASCII；中文留给控制台报告。"""
    for i, (text, bgr) in enumerate(lines):
        at = (x, y + i * 15)
        cv2.putText(image, text, at, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image, text, at, cv2.FONT_HERSHEY_SIMPLEX, 0.4, bgr, 1, cv2.LINE_AA)


def _ascii(text: str) -> str:
    return text.encode('ascii', 'replace').decode('ascii')


def pose_matrix(pose: np.ndarray) -> np.ndarray:
    """[x,y,z,qx,qy,qz,qw] -> 4x4。"""
    out = np.eye(4)
    out[:3, :3] = _quat(pose[3:7])
    out[:3, 3] = pose[:3]
    return out


def _quat(quat) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat)
    norm = np.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# --------------------------------------------------------------------- 主流程


def pick_window(session: Session, args) -> tuple:
    """选出要渲染的时间窗，返回 `(t0, t1, 标题)`。"""
    if args.t0 and args.t1:
        return args.t0, args.t1, args.label or 'custom window'
    if args.whole:
        pts = session.video_pts('head')
        return float(pts[0]), float(pts[-1]), 'whole session'
    episodes = session.episodes()
    if not episodes:
        raise RuntimeError('这次采集没有标注完成的 episode，用 --whole 看整段')
    if args.episode:
        want = tuple(int(v) for v in args.episode.split(':'))
        found = [e for e in episodes if (e['round'], e['episode']) == want]
        if not found:
            raise RuntimeError(f'没有 episode {args.episode}，有的是 '
                               + ', '.join(f'{e["round"]}:{e["episode"]}' for e in episodes))
        episode = found[0]
    else:
        episode = episodes[0]
    return (episode['t0'], episode['t1'],
            f'ep {episode["round"]}:{episode["episode"]} [{episode["outcome"]}] '
            + episode.get('instruction_en', ''))


def _share(package: str, *parts: str) -> Path | None:
    try:
        path = Path(get_package_share_directory(package)).joinpath(*parts)
    except PackageNotFoundError:
        return None
    return path if path.is_file() else None


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('session', help='session id（在 --root 下找）或目录路径')
    parser.add_argument('--root', default=str(Path.home() / '.ros' / 'record' / 'sessions'))
    parser.add_argument('--episode', default='', help='轮次:序号，如 0:3。默认第一条')
    parser.add_argument('--whole', action='store_true', help='整段采集，不按 episode 切')
    parser.add_argument('--t0', type=float, default=0.0,
                        help='直接给窗口起点（unix 纪元秒），要和 --t1 一起给；优先于 --episode')
    parser.add_argument('--t1', type=float, default=0.0)
    parser.add_argument('--label', default='', help='图上打的标题，配 --t0/--t1 用')
    parser.add_argument('--fps', type=float, default=10.0, help='输出帧率，也就是渲染多少帧')
    parser.add_argument('--width', type=int, default=640, help='输出宽度，内参跟着等比缩')
    parser.add_argument('--out', default='', help='输出 mp4，默认写到 session 同级')
    parser.add_argument('--urdf', default='')
    parser.add_argument('--calibration', default='')
    parser.add_argument('--progress', action='store_true',
                        help='往 stdout 打 `@progress 0.42`，给面板画进度条用')
    args = parser.parse_args(argv)
    try:
        return run(args)
    except (FileNotFoundError, KeyError, RuntimeError, ValueError) as error:
        print(f'{type(error).__name__}: {error}', file=sys.stderr)
        return 1


def run(args) -> int:
    root = Path(args.session)
    session = Session(root if root.is_dir() else Path(args.root) / args.session)
    urdf = Path(args.urdf) if args.urdf else _share(
        'unitree_g1_description', 'model', 'final.urdf')
    calibration = Path(args.calibration) if args.calibration else _share(
        'camera_calibration', 'config', 'calibration.yaml')
    if urdf is None or not urdf.is_file():
        print('找不到 URDF，用 --urdf 指一个', file=sys.stderr)
        return 1

    t0, t1, title = pick_window(session, args)
    pts = session.video_pts('head')
    frames = session.slice_frames('head', t0, t1)
    if frames.size == 0:
        print(f'{t0:.3f}~{t1:.3f} 这段没有头部视频帧', file=sys.stderr)
        return 1
    step = max(int(round(session.nominal_fps('head') or 30.0) / args.fps), 1)
    pick = frames[::step]
    when = pts[pick]

    camera = head_camera(session.meta, args.width)
    renderer, model, applied = build_renderer(urdf, calibration)
    extrinsic = optical_extrinsic(session.meta)

    columns = session.columns('joint_states')
    pos_at = [i for i, name in enumerate(columns) if name.startswith('pos.')]
    pos_names = [columns[i][4:] for i in pos_at]
    t_joint, joint_data = session.table('joint_states')
    joints, joint_age = sample_at(t_joint, joint_data[:, pos_at], when, interpolate=True)
    t_cmd, cmd_data = session.table('motion_control_command')
    command, cmd_age = sample_at(t_cmd, cmd_data, when, interpolate=False)
    target = {'left': command[:, LEFT], 'right': command[:, RIGHT]}
    grip = command[:, GRIP]

    out = Path(args.out) if args.out else (
        session.root.parent / f'{session.root.name}_verify.mp4')
    calibrated = ('、'.join(applied) + ' 已叠加' if applied else
                  f'没叠上（{calibration}）—— 头部外参退回 URDF 名义值，'
                  '轮廓会整体偏十来个像素，别据此判对齐')
    report = [f'采集 : {session.root.name}  {title}',
              f'模型 : {urdf}',
              f'标定 : {calibrated}',
              f'相机 : {camera}（出厂内参，取自 meta.json）',
              f'窗口 : {t1 - t0:.2f} s，头部 {frames.size} 帧 -> 渲染 {pick.size} 帧'
              f'（每 {step} 帧取一），约 {pick.size * 0.33:.0f} s']
    for line in report:
        print(line)

    writer = Writer(out, camera.width, camera.height, args.fps)
    # 实测末端也换算到 torso 系再统计：指令就是 torso 系的，而躯干本身在动，
    # 两边混着世界系比会把腰的运动算进误差里。
    actual = {side: np.zeros((pick.size, 3)) for side in TIP}
    wanted = set(int(v) for v in pick)
    done = 0
    try:
        for n, image in enumerate(decode(session.video_path('head'), int(pick[0]),
                                         int(pick[-1] - pick[0]) + 1,
                                         camera.width, camera.height)):
            if int(pick[0]) + n not in wanted:
                continue
            row = done
            done += 1
            image = image.copy()
            q = renderer.q_from_joint_map(
                expand_mimic(model, dict(zip(pos_names, joints[row]))))
            cam_rot, cam_pos = renderer.camera_pose(q, 'd435_link', extrinsic=extrinsic)
            draw_contours(image, renderer.silhouette(q, camera, cam_rot, cam_pos),
                          renderer.names)

            torso = np.eye(4)
            torso[:3, :3], torso[:3, 3] = renderer.camera_pose(
                q, BASE_FRAME, optical=False)
            legend = []
            for side, bgr in COLOR.items():
                want = torso @ pose_matrix(target[side][row])
                have = np.eye(4)
                have[:3, :3], have[:3, 3] = renderer.camera_pose(
                    q, TIP[side], optical=False)
                actual[side][row] = torso[:3, :3].T @ (have[:3, 3] - torso[:3, 3])
                a = draw_pose(image, want, DARK[side], True, cam_rot, cam_pos, camera)
                b = draw_pose(image, have, bgr, False, cam_rot, cam_pos, camera)
                if a is not None and b is not None:
                    cv2.line(image, tuple(a), tuple(b), (255, 255, 255), 1, cv2.LINE_AA)
                legend.append((
                    '%-5s cmd-fk %5.1f mm  grip %.2f'
                    % (side, 1000.0 * np.linalg.norm(want[:3, 3] - have[:3, 3]),
                       grip[row][0 if side == 'left' else 1]), bgr))

            draw_text(image, [(_ascii(f'{session.root.name}  {title}')[:78], (255, 255, 255)),
                              ('t %+.2f s   joints %+.0f ms   command %+.0f ms'
                               % (when[row] - t0, 1000 * joint_age[row],
                                  1000 * cmd_age[row]), (255, 255, 255))], 8, 18)
            draw_text(image, legend, 8, camera.height - 26)
            writer.write(image)
            if args.progress:
                print(f'@progress {done / pick.size:.3f}', flush=True)
            if done == pick.size:
                break
    finally:
        writer.close()

    def say(line: str) -> None:
        report.append(line)
        print(line)

    say(f'关节 : {t_joint.size / (t_joint[-1] - t_joint[0]):.0f} Hz，'
        f'取样最大滞后 {1000 * joint_age.max():.0f} ms')
    say(f'指令 : {t_cmd.size / (t_cmd[-1] - t_cmd[0]):.0f} Hz，'
        f'取样最大滞后 {1000 * cmd_age.max():.0f} ms')
    for side, columns_at in (('left', LEFT), ('right', RIGHT)):
        distance = np.linalg.norm(target[side][:, :3] - actual[side], axis=1)
        travel = np.ptp(target[side][:, :3], axis=0).max()
        line = (f'{side:5s}: 目标-实际 中位 {1000 * np.median(distance):.0f} mm，'
                f'p95 {1000 * np.percentile(distance, 95):.0f} mm')
        # 目标不动时任何时移给出的距离都一样，argmin 只会落在网格边界上，是个假数。
        if travel < 0.05:
            say(line + f'；这条臂整段只动了 {1000 * travel:.0f} mm，时移测不出来')
            continue
        shift, tight = best_shift(t_cmd, cmd_data[:, columns_at][:, :3],
                                  when, actual[side])
        say(line + f'；最佳时移 {shift:+.2f} s 时降到 {1000 * tight:.0f} mm')
    # 报告和视频摆在一起：光看图判不出「差多少毫米」，而这份数只在 stdout 里的话，
    # 换个终端、隔一天回来看那段视频就没依据了。
    out.with_suffix('.txt').write_text('\n'.join(report) + '\n', encoding='utf-8')
    print(f'输出 : {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
