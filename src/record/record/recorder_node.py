"""采集节点：把视频、信号、指令、session 串起来。

**架构铁律：录制逻辑全在这里，dashboard 只是观察者 + 命令入口。** 没人开页面、HTTP
线程崩了，录制照常。所以本文件不 import 任何 HTTP 相关的东西。

线程模型：ROS 回调写各自的表（每张表只由它自己的回调写，不跨线程共享）；HTTP 线程
只调 ``start_session`` / ``start_round`` 这类状态迁移，它们全在 ``_lock`` 里。
"""

from __future__ import annotations

import json
import signal
import threading
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from record import signals as sig
from record.session import Session, State
from record.table_writer import TableWriter
from record.video import HeadRecorder, WristRecorder, probe_stream

DEFAULT_ROOT = str(Path.home() / '.ros' / 'record' / 'sessions')


def _import_msg(type_name: str):
    pkg, _, cls = type_name.replace('/msg/', '/').partition('/')
    cls = cls.split('/')[-1]
    return getattr(__import__(f'{pkg}.msg', fromlist=[cls]), cls)


def _qos(best_effort: bool, depth: int = 50) -> QoSProfile:
    return QoSProfile(
        depth=depth, history=HistoryPolicy.KEEP_LAST,
        reliability=(ReliabilityPolicy.BEST_EFFORT if best_effort
                     else ReliabilityPolicy.RELIABLE))


class Recorder(Node):
    """采集主体。对外只暴露几个状态迁移方法，dashboard 调它们。"""

    def __init__(self) -> None:
        super().__init__('recorder')
        p = self.declare_parameter
        self.root = Path(p('output_root', DEFAULT_ROOT).value)
        self.head_topic = p('head_image_topic', '/head/camera/color/image_raw').value
        self.head_info_topic = p('head_camera_info_topic',
                                 '/head/camera/color/camera_info').value
        self.head_fps = int(p('head_fps', 30).value)
        self.wrist_urls = {
            'wrist_left': p('wrist_left_url',
                            'rtsp://admin:123456@192.168.123.97/stream0').value,
            'wrist_right': p('wrist_right_url',
                             'rtsp://admin:123456@192.168.123.98/stream0').value,
        }
        self.n_items = int(p('round_items', 4).value)
        self.n_moves = int(p('round_moves', 6).value)

        self._lock = threading.RLock()
        self._group = ReentrantCallbackGroup()
        self.session: Session | None = None
        self.specs = {s.key: s for s in sig.default_specs()}
        self.writers: dict[str, TableWriter] = {}
        self.stats: dict[str, dict] = {k: {'received': 0, 'written': 0, 'last': 0.0}
                                       for k in self.specs}
        self._subs: list = []
        self._last_write: dict[str, float] = {}
        self.wrists: dict[str, WristRecorder] = {}
        self.head: HeadRecorder | None = None
        self._head_seen = {'width': 0, 'height': 0, 'encoding': '', 'count': 0,
                           'last': 0.0}
        self._head_info: dict | None = None
        self._preview: dict[str, bytes] = {}

        self.library, self.geometry, self.library_error = self._load_library()
        self.current_round: dict | None = None
        self._round_obj = None

        # 常驻订阅：不录也要在面板上显示各路的实时频率，操作者才能在开录前发现问题
        self._attach_monitors()
        self.create_subscription(Image, self.head_topic, self._on_head,
                                 _qos(False, 4), callback_group=self._group)
        self.create_subscription(CameraInfo, self.head_info_topic, self._on_head_info,
                                 _qos(False, 4), callback_group=self._group)
        self.get_logger().info(
            f'采集节点就绪，落盘根目录 {self.root}；'
            f'物品库 {"OK" if self.library else self.library_error}')

    # ------------------------------------------------------------------ 物品库

    def _load_library(self):
        try:
            from record.instruction import LIBRARY_SUBDIR, ItemLibrary
            from record.table_geometry import load_geometry
            share = Path(get_package_share_directory('record'))
            lib = ItemLibrary(share / 'items' / LIBRARY_SUBDIR)
            return lib, load_geometry(), ''
        except Exception as exc:                       # noqa: BLE001
            return None, None, f'{type(exc).__name__}: {exc}'

    # -------------------------------------------------------------- 订阅与落盘

    def _attach_monitors(self) -> None:
        for spec in self.specs.values():
            if not spec.columns:
                continue
            try:
                msg_type = _import_msg(spec.type_name)
            except (ImportError, AttributeError):
                continue
            self._subs.append(self.create_subscription(
                msg_type, spec.topic,
                (lambda m, k=spec.key: self._on_signal(k, m)),
                _qos(spec.best_effort), callback_group=self._group))

    def _on_signal(self, key: str, msg) -> None:
        st = self.stats[key]
        now = time.time()
        st['received'] += 1
        st['last'] = now
        writer = self.writers.get(key)
        if writer is None:
            return
        spec = self.specs[key]
        if spec.max_hz > 0.0:
            prev = self._last_write.get(key, 0.0)
            if now - prev < 1.0 / spec.max_hz:
                return
            self._last_write[key] = now
        try:
            writer.append([now, sig.header_stamp(msg), *spec.row(msg)])
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warn(f'{key} 落盘失败: {exc}')
            return
        st['written'] += 1

    def _on_head(self, msg: Image) -> None:
        seen = self._head_seen
        seen.update(width=msg.width, height=msg.height, encoding=msg.encoding,
                    last=time.time())
        seen['count'] += 1
        head = self.head
        if head is None:
            return
        if not head.health.alive and head.proc is None:
            try:
                head.start_from(msg.width, msg.height, msg.encoding, self.head_fps)
            except ValueError as exc:
                self.get_logger().error(f'头部相机编码不支持: {exc}')
                self.head = None
                return
        # msg.data 是 array.array('B')，实现了 buffer protocol，直接入队没有拷贝
        head.push(memoryview(msg.data), sig.header_stamp(msg))

    def _on_head_info(self, msg: CameraInfo) -> None:
        self._head_info = {
            'width': msg.width, 'height': msg.height,
            'k': list(msg.k), 'd': list(msg.d), 'p': list(msg.p),
            'distortion_model': msg.distortion_model,
            'frame_id': msg.header.frame_id,
        }

    # -------------------------------------------------------------- 状态迁移

    def start_session(self, streams: dict, note: str = '') -> dict:
        with self._lock:
            if self.session is not None:
                raise RuntimeError('已经在录了')
            meta = {
                'note': note,
                'head_camera_info': self._head_info,
                'head_stream': dict(self._head_seen),
                'table_geometry': self.geometry.meta if self.geometry else None,
                'joint_order': list(sig.CANONICAL_JOINTS),
            }
            session = Session.create(self.root, streams, meta=meta)
            paths = session.paths
            for key, spec in self.specs.items():
                if not streams.get(key) or not spec.columns:
                    continue
                self.writers[key] = TableWriter(
                    paths.signals / f'{key}.bin',
                    ['t_recv', 't_header', *spec.columns],
                    description=spec.note)
            for name, url in self.wrist_urls.items():
                if streams.get(name):
                    rec = WristRecorder(name, url, paths.video)
                    rec.start()
                    self.wrists[name] = rec
            if streams.get('head'):
                self.head = HeadRecorder('head', paths.video)
            self.session = session
            probing = {n: u for n, u in self.wrist_urls.items() if streams.get(n)}
            if probing:
                # 探一路最长 20 s，串行探完会把开录拖慢半分钟，所以扔后台；录制不依赖它
                threading.Thread(target=self._probe_video, daemon=True,
                                 args=(paths.video, probing, self.head_fps if streams.get('head') else 0)).start()
            return self.status()

    def _probe_video(self, video_dir: Path, urls: dict[str, str], head_fps: int) -> None:
        """记下各路的标称帧率，导出端才能判断 fps 偏低是丢帧还是相机本来就慢。

        实测两台腕相机出厂帧率不同（.97 是 30、.98 是 25），没有这份基准就只能靠猜。
        """
        nominal = {}
        for name, url in urls.items():
            nominal[name] = probe_stream(url)
        if head_fps:
            nominal['head'] = {'ok': True, 'fps': float(head_fps), 'source': 'param'}
        try:
            (video_dir / 'nominal.json').write_text(
                json.dumps(nominal, ensure_ascii=False, indent=1), encoding='utf-8')
        except OSError as exc:
            self.get_logger().warning(f'标称帧率写入失败: {exc}')

    def start_round(self, seed: int | None = None) -> dict:
        with self._lock:
            self._require(State.SESSION)
            if self.library is None:
                raise RuntimeError(f'物品库不可用: {self.library_error}')
            from record.instruction.builder import build_round
            rnd = build_round(self.library, self.geometry,
                              index=self.session.round_index + 1, seed=seed,
                              n_items=self.n_items, n_moves=self.n_moves)
            self._round_obj = rnd
            self.current_round = rnd.as_dict()
            self.session.start_round(self.current_round, svg=rnd.svg)
            for hit in self.current_round.get('lint_warnings', []):
                self.session.warn('lint', **hit)
            return self.status()

    def end_round(self) -> dict:
        with self._lock:
            self._require(State.ROUND)
            self.session.end_round()
            self.current_round = None
            self._round_obj = None
            return self.status()

    def start_episode(self, index: int) -> dict:
        with self._lock:
            self._require(State.ROUND)
            eps = (self.current_round or {}).get('episodes', [])
            if not 0 <= index < len(eps):
                raise RuntimeError(f'episode 序号 {index} 越界')
            self.session.start_episode(eps[index])
            return self.status()

    def end_episode(self, outcome: str, note: str = '') -> dict:
        with self._lock:
            self._require(State.EPISODE)
            self.session.end_episode(outcome, note)
            if outcome == 'success' and self.library and self._round_obj:
                self._bump_usage()
            return self.status()

    def _bump_usage(self) -> None:
        idx = self.session.episode_index
        eps = (self.current_round or {}).get('episodes', [])
        if not 0 <= idx < len(eps):
            return
        ep = eps[idx]
        if ep.get('obj', {}).get('id'):
            self.library.bump_usage(ep['obj']['id'], 'as_x')
        if ep.get('target', {}).get('id'):
            self.library.bump_usage(ep['target']['id'], 'as_y')

    def stop_session(self) -> dict:
        with self._lock:
            if self.session is None:
                raise RuntimeError('没有正在录的 session')
            # 先把所有流一起叫停，再逐个 finalize。finalize 要等 ffmpeg 冲完缓冲落盘，
            # 边停边收会让排在后面的头部白录这段时间（实测尾部相差 5.2 s）
            for rec in self.wrists.values():
                rec.stop()
            if self.head is not None:
                self.head.stop()
            for name, rec in self.wrists.items():
                stamps = rec.finalize()
                if rec.killed:
                    self.session.warn('video_hard_killed', stream=name, timestamps=stamps)
                elif not stamps:
                    self.session.warn('video_no_timestamps', stream=name)
            if self.head is not None:
                self.head.finalize()
                if self.head.dropped:
                    # 队列满才会计数，说明编码器或写线程跟不上；为 0 则丢在订阅上游
                    self.session.warn('head_queue_drop', frames=self.head.dropped)
            schema = {}
            for key, w in self.writers.items():
                w.close()
                schema[key] = w.schema()
            for name, rec in self.wrists.items():
                schema[f'{name}.pts'] = {
                    'file': rec.pts_path.name, 'dtype': 'float64', 'ncol': 1,
                    'columns': ['t_wallclock'],
                    'description': 'ffmpeg 收包墙钟；帧率硬件恒定，导出侧可鲁棒拟合去抖'}
            if self.head is not None:
                schema['head.pts'] = {
                    'file': self.head.pts_path.name, 'dtype': 'float64', 'ncol': 1,
                    'columns': ['t_header'],
                    'description': 'RealSense 硬件时间戳，无需拟合'}
            digest = self.session.finish(schema)
            if self.library:
                self.library.save_usage()
            root = self.session.paths.root
            self.session, self.writers, self.wrists = None, {}, {}
            self.head, self.current_round, self._round_obj = None, None, None
            self._last_write.clear()
            return {'session': root.name, 'files': digest['file_count'],
                    'bytes': digest['total_bytes'], 'path': str(root)}

    def _require(self, state: State) -> None:
        if self.session is None:
            raise RuntimeError('还没开始 session')
        if self.session.state is not state:
            raise RuntimeError(f'当前是 {self.session.state.value}，'
                               f'这一步要求 {state.value}')

    # ------------------------------------------------------------------ 观测

    def stream_overview(self) -> list[dict]:
        """所有数据流一览。开录前用来确认没有哑掉的话题。"""
        now = time.time()
        out = []
        for key, spec in self.specs.items():
            st = self.stats[key]
            age = now - st['last'] if st['last'] else float('inf')
            out.append({
                'key': key, 'kind': 'signal', 'topic': spec.topic,
                'type': spec.type_name, 'columns': len(spec.columns),
                'received': st['received'], 'written': st['written'],
                'online': age < 3.0, 'age': round(min(age, 999.0), 1),
                'max_hz': spec.max_hz, 'note': spec.note,
                'default_on': spec.default_on,
                'recording': key in self.writers,
            })
        for name, url in self.wrist_urls.items():
            rec = self.wrists.get(name)
            out.append({
                'key': name, 'kind': 'video', 'topic': url, 'type': 'RTSP (-c copy)',
                'columns': 0, 'received': rec.health.frames if rec else 0,
                'written': rec.health.frames if rec else 0,
                'online': bool(rec and rec.health.alive), 'age': 0.0,
                'max_hz': 0.0, 'default_on': True, 'recording': name in self.wrists,
                'note': rec.health.error if rec else '未开录',
            })
        seen = self._head_seen
        age = now - seen['last'] if seen['last'] else float('inf')
        out.append({
            'key': 'head', 'kind': 'video', 'topic': self.head_topic,
            'type': f'{seen["encoding"] or "?"} {seen["width"]}x{seen["height"]}',
            'columns': 0, 'received': seen['count'],
            'written': self.head.health.frames if self.head else 0,
            'online': age < 3.0, 'age': round(min(age, 999.0), 1),
            'max_hz': 0.0, 'default_on': True, 'recording': self.head is not None,
            'note': f'队列丢帧 {self.head.dropped}' if self.head else '',
        })
        out.append({
            'key': 'head_depth', 'kind': 'video', 'topic': '（深度图）',
            'type': '16UC1', 'columns': 0, 'received': 0, 'written': 0,
            'online': False, 'age': 999.0, 'max_hz': 0.0,
            'default_on': False, 'recording': False,
            'note': '压不了，424x240@30 就是 21 GB/h，比三路彩色加起来还大',
        })
        return out

    def status(self) -> dict:
        s = self.session
        disk = self._disk()
        return {
            'state': s.state.value if s else 'idle',
            'session': s.session_id if s else '',
            'round': s.round_index if s else -1,
            'episode': s.episode_index if s else -1,
            'counts': dict(s.counts) if s else {},
            'bytes': sum(w.bytes_written for w in self.writers.values()),
            'disk_free_gb': round(disk / 1e9, 1),
            'library_error': self.library_error,
            'round_detail': self.current_round,
        }

    def _disk(self) -> int:
        import shutil as _sh
        try:
            return _sh.disk_usage(self.root if self.root.exists()
                                  else self.root.parent).free
        except OSError:
            return 0

    def preview(self, key: str) -> bytes:
        return self._preview.get(key, b'')


def _on_sigterm(signum, frame):   # noqa: ARG001
    raise KeyboardInterrupt


def main() -> None:
    rclpy.init()
    node = Recorder()
    from record.dashboard import Dashboard
    dash = Dashboard(node, port=int(node.declare_parameter('dashboard_port', 8220).value))
    dash.start()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    # Python 对 SIGTERM 没有处理器，默认动作直接终止进程，下面的 finally 不会跑，
    # ffmpeg 子进程就成了孤儿，继续空烧 CPU 和写盘。docker stop / systemd / ros2 launch
    # 关闭走的都是 SIGTERM。
    signal.signal(signal.SIGTERM, _on_sigterm)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.session is not None:
            try:
                node.stop_session()
            except Exception as exc:                   # noqa: BLE001
                node.get_logger().error(f'退出时收尾失败: {exc}')
        dash.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
