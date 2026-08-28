"""采集节点：把视频、信号、指令、session 串起来。

**架构铁律：录制逻辑全在这里，dashboard 只是观察者 + 命令入口。** 没人开页面、HTTP
线程崩了，录制照常。所以本文件不 import 任何 HTTP 相关的东西。

线程模型：**每路订阅各占一个 ``MutuallyExclusiveCallbackGroup``**，各路之间并行、同一路
内部串行。不能图省事共用一个 ``ReentrantCallbackGroup``：那样同一个回调会在 4 个线程上
同时跑，``TableWriter`` 会把同一段写两遍、``_last_write`` 节流失效、``_on_head`` 还可能
把 ffmpeg 启两遍。HTTP 线程只调 ``start_session`` / ``start_round`` 这类状态迁移，
它们全在 ``_lock`` 里。
"""

from __future__ import annotations

import json
import shutil
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import cast

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger

from record import signals as sig
from record.session import Session, State
from record.table_writer import TableWriter
from record.video import (HeadPreview, HeadRecorder, PreviewPump, WristRecorder,
                          preview_url, probe_stream)

DEFAULT_ROOT = str(Path.home() / '.ros' / 'record' / 'sessions')


def _import_msg(type_name: str):
    """``'sensor_msgs/msg/JointState'`` -> 消息类。"""
    parts = type_name.split('/')
    return getattr(__import__(f'{parts[0]}.msg', fromlist=[parts[-1]]), parts[-1])


def _udp_rcvbuf_errors() -> int:
    """UDP 收包缓冲溢出计数。大图像丢帧时它涨得快就是丢在 DDS 传输层。"""
    try:
        with open('/proc/net/snmp', encoding='ascii') as fh:
            head, val = [ln for ln in fh if ln.startswith('Udp:')][:2]
        cols, vals = head.split(), val.split()
        return int(vals[cols.index('RcvbufErrors')])
    except (OSError, ValueError, IndexError):
        return -1


def _qos(best_effort: bool, depth: int = 50) -> QoSProfile:
    return QoSProfile(
        depth=depth, history=HistoryPolicy.KEEP_LAST,
        reliability=(ReliabilityPolicy.BEST_EFFORT if best_effort
                     else ReliabilityPolicy.RELIABLE))


def _liveness(last: float, now: float) -> dict:
    """距上一条消息多久，以及据此判的在线与否。面板上那颗绿灯就是它。"""
    age = now - last if last else float('inf')
    return {'online': age < 3.0, 'age': round(min(age, 999.0), 1)}


class Recorder(Node):
    """采集主体。对外只暴露几个状态迁移方法，dashboard 调它们。"""

    def __init__(self) -> None:
        super().__init__('recorder')
        p = self.declare_parameter
        self.root = Path(cast(str, p('output_root', DEFAULT_ROOT).value))
        self.head_topic = cast(
            str, p('head_image_topic', '/head/camera/color/image_raw').value)
        self.head_info_topic = cast(
            str, p('head_camera_info_topic',
                   '/head/camera/color/camera_info').value)
        self.head_fps = cast(int, p('head_fps', 30).value)
        # KEEP_LAST 深度就是能容忍的卡顿长度。头部 pts 取自 header.stamp，晚到不影响时间
        # 精度，丢了却会在时间轴上留空洞，所以加深只赚不亏。实测 4/30/120 在可复现
        # 的负载下没差别（那种负载压根不丢帧），30 是给复现不了的真实工况留的保险。
        self.head_qos_depth = cast(int, p('head_qos_depth', 30).value)
        self.wrist_urls = {
            'wrist_left': cast(
                str, p('wrist_left_url',
                       'rtsp://admin:123456@192.168.123.97/stream0').value),
            'wrist_right': cast(
                str, p('wrist_right_url',
                       'rtsp://admin:123456@192.168.123.98/stream0').value),
        }
        self.n_items = cast(int, p('round_items', 4).value)
        self.n_moves = cast(int, p('round_moves', 6).value)
        self.peer_port = cast(int, p('peer_port', 8221).value)
        # 置空则不代钉世界原点。只有想让多个 session 共用同一个世界系时才需要关。
        self._origin_service = cast(
            str, p('origin_service', '/g1_localization/set_origin').value)
        self._peer = (0.0, False)

        self._lock = threading.RLock()
        self.session: Session | None = None
        self.specs = {s.key: s for s in sig.default_specs()}
        self.writers: dict[str, TableWriter] = {}
        self.stats: dict[str, dict] = {k: {'received': 0, 'written': 0, 'last': 0.0,
                                           'health': ''}
                                       for k in self.specs}
        self._subs: list = []
        self._last_write: dict[str, float] = {}
        self.wrists: dict[str, WristRecorder] = {}
        self.head: HeadRecorder | None = None
        self._head_seen = {'width': 0, 'height': 0, 'encoding': '', 'count': 0,
                           'last': 0.0}
        self._head_info: dict | None = None
        self.pending_round: dict | None = None      # 已生成、还没固化进 session 的那一轮
        self._pending_obj = None
        # 开录时刷新，停录时拿来判头部收没收齐帧（见 _warn_head_gap）
        self._udp_err0, self._head_seen0, self._head_t0 = 0, 0, 0.0
        # 开录前抓快照要用。只留最近一帧，不是缓冲区
        self._head_last: tuple[bytes, int, int, str] | None = None
        # 面板上的低帧率预览。按需起、没人看就收（见 _reap_previews）
        self._preview_lock = threading.Lock()
        self._previews: dict[str, PreviewPump] = {}
        self._head_pump = HeadPreview()
        # 最近一次为头部留帧的时刻（monotonic），见 _head_due
        self._head_kept = 0.0

        self.library, self.geometry, self.library_error = self._load_library()
        # 真实桌面那块矩形。恒等于「现有这一轮实际用的桌面」——改尺寸会丢掉旧预览
        self._table: dict | None = None
        self._table_geo = None            # self.geometry 裁到桌面那块之后的副本
        if self.geometry is not None:
            self._use_table(*self.geometry.extent)
        self.current_round: dict | None = None
        self._round_obj = None
        # 正在录的是指令表里的第几条。和 session.episode_index（本轮第几次录）
        # 只在「一条一次、按顺序走」时碰巧相等，重录一次就永久分家
        self._episode_slot = -1
        # slot -> 已录各次 ``{'episode': 本轮第几次录, 'outcome': 结论}``。同一条指令可以录
        # 很多遍，每遍都是独立的一条 episode，旧的不会被覆盖；面板拿它把每次尝试列出来。
        # 存 episode 序号是为了事后能改其中某一次的标注（当场判的成功常常回头看是失败）
        self._slot_takes: dict[int, list[dict]] = {}

        # 常驻订阅：不录也要在面板上显示各路的实时频率，操作者才能在开录前发现问题
        self._attach_monitors()
        self.create_subscription(Image, self.head_topic, self._on_head,
                                 _qos(False, self.head_qos_depth),
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(CameraInfo, self.head_info_topic, self._on_head_info,
                                 _qos(False, 4),
                                 callback_group=MutuallyExclusiveCallbackGroup())
        # 自己一组：开录时要同步等它回包，不能排在信号回调后面
        self._origin_cli = self.create_client(
            Trigger, self._origin_service,
            callback_group=MutuallyExclusiveCallbackGroup()) \
            if self._origin_service else None
        # 没人看预览时把解码进程收掉。只能靠定时器：页面一关就再也不来请求了，
        # 挂在请求路径上的回收永远触发不到
        self.create_timer(2.0, self._reap_previews,
                          callback_group=MutuallyExclusiveCallbackGroup())
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

    def set_table(self, depth: float, width: float, near: float) -> dict:
        """限定物品只摆在真实桌面那块矩形里（米）。

        会丢掉还没固化的预览：那一轮是照旧桌面摆的，跟着它摆真桌子会摆到桌沿外面。
        丢掉之后 ``self._table`` 就恒等于「现有这一轮实际用的桌面」，面板直接显示它。
        """
        with self._lock:
            if self._round_is_live():
                raise RuntimeError('本轮已固化，桌子也照它摆好了，改尺寸要先「结束本轮」')
            self._use_table(depth, width, near)
            self.pending_round, self._pending_obj = None, None
            return self.status()

    def _use_table(self, depth: float, width: float, near: float) -> None:
        """可达掩码是离线跑 IK 得到的「手够得着哪」，与桌子多大无关 —— 桌子比它小时
        照原样摆会把物品摆到桌沿外面。``library.usable`` 要 20 ms，所以只在这里算一次。

        验收条件照抄 ``choose_group`` 要的那两条。只查「有物品放得下」不够：可达域纵深
        只有 250 mm，浅桌子先饿死的是容器（实测 200x500 还剩 75 件普通物品、容器 0），
        放过去的话点「生成任务」才报错，操作者看不出是桌子设小了。
        """
        if self.library is None:
            raise RuntimeError(f'物品库不可用: {self.library_error}')
        if not (depth > 0 and width > 0):
            raise RuntimeError('桌面尺寸必须为正')
        geo = self.geometry.clip(depth, width, near)
        pool = self.library.usable(geo)
        containers = sum(1 for i in pool if i.is_container)
        ordinary = sum(1 for i in pool if i.role == 'ordinary')
        cells = int(geo.reachable.sum())
        if not containers or ordinary < self.n_items - 1:
            raise RuntimeError(
                f'这块桌面摆不出一轮任务：可放格心 {cells}，容器 {containers} 件、'
                f'普通物品 {ordinary} 件（至少要 1 + {self.n_items - 1}）')
        with self._lock:
            self._table_geo = geo
            self._table = {'depth_mm': round(depth * 1000), 'width_mm': round(width * 1000),
                           'near_mm': round(near * 1000), 'cells': cells,
                           'containers': containers}

    def _round_is_live(self) -> bool:
        """本轮已固化：桌子已经照它摆好了，摆放、指令、桌面尺寸都不能再动。"""
        return (self.session is not None
                and self.session.state in (State.ROUND, State.EPISODE))

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
                _qos(spec.best_effort, spec.depth),
                callback_group=MutuallyExclusiveCallbackGroup()))

    def _on_signal(self, key: str, msg) -> None:
        st = self.stats[key]
        now = time.time()
        st['received'] += 1
        st['last'] = now
        spec = self.specs[key]
        # 体检在落盘之前做：没开录时最需要看见它，开录之后就晚了
        if spec.health is not None:
            st['health'] = spec.health(msg)
        writer = self.writers.get(key)
        if writer is None:
            return
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
        # 每帧都拷 = 30 x 1.8 MB/s 的白拷贝，所以按预览周期节流；
        # 在录、又没人看预览时一帧都不留
        watching = not self._head_pump.idle()
        if (head is None or watching) and self._head_due():
            payload = bytes(msg.data)
            self._head_last = (payload, msg.width, msg.height, msg.encoding)
            if watching:
                self._head_pump.push(payload, msg.width, msg.height, msg.encoding)
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

    def _pin_world_origin(self) -> dict:
        """把世界原点钉在开录那一刻的躯干位姿。

        忘了钉的后果是静默的：``torso_pose`` 整段是单位阵、``origin_set`` 恒 0，
        面板上只看频率一切正常，事后才发现这一路是废的。每个 session 各钉一次，
        于是「世界原点 = 开录瞬间的躯干位姿（调平）」，数据自含。

        失败只记进 meta，绝不挡录制——其余十几路不该因为雷达没起就录不成。
        """
        if self._origin_cli is None:
            return {'ok': False, 'reason': '未配置 origin_service'}
        if not self._origin_cli.wait_for_service(timeout_sec=1.0):
            return {'ok': False, 'reason': f'{self._origin_service} 不在线'}
        future = self._origin_cli.call_async(Trigger.Request())
        # 回包由执行器线程填，这里是 HTTP 线程，所以只能轮询不能自己 spin
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        res = future.result()
        if res is None:
            return {'ok': False, 'reason': '服务超时'}
        return {'ok': bool(res.success), 'message': res.message}

    def start_session(self, streams: dict, note: str = '') -> dict:
        with self._lock:
            if self.session is not None:
                raise RuntimeError('已经在录了')
            # 没 roll 就开录，生任务和摆桌子的那几十秒全录进视频里
            if self.pending_round is None:
                raise RuntimeError('先生成一轮任务并摆好桌子再开录')
            origin = self._pin_world_origin() if streams.get('torso_pose') else None
            if origin is not None:
                self.get_logger().info(
                    f'世界原点：{origin.get("message") or origin.get("reason")}'
                    if origin['ok'] else
                    f'世界原点没钉上（{origin.get("reason") or origin.get("message")}），'
                    f'torso_pose 这一路录下来也是废的')
            meta = {
                'note': note,
                'world_origin': origin,
                'head_camera_info': self._head_info,
                'head_stream': dict(self._head_seen),
                'table_geometry': self.geometry.meta if self.geometry else None,
                'table': self._table,
                'joint_order': list(sig.CANONICAL_JOINTS),
            }
            session = Session.create(self.root, streams, meta=meta)
            paths = session.paths
            for key, spec in self.specs.items():
                if not streams.get(key) or not spec.columns:
                    continue
                self.writers[key] = TableWriter(
                    paths.signals / f'{key}.bin',
                    ['t_recv', 't_header', *cast(list, spec.columns)],
                    description=spec.note)
            for name, url in self.wrist_urls.items():
                if streams.get(name):
                    rec = WristRecorder(name, url, paths.video)
                    rec.start()
                    self.wrists[name] = rec
            if streams.get('head'):
                self.head = HeadRecorder('head', paths.video)
            self.session = session
            self._udp_err0 = _udp_rcvbuf_errors()
            self._head_seen0 = self._head_seen['count']
            self._head_t0 = time.time()
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

    def _build_round(self, index: int, seed: int | None, items: list | None = None):
        if self.library is None:
            raise RuntimeError(f'物品库不可用: {self.library_error}')
        from record.instruction.builder import build_round
        return build_round(self.library, self._table_geo, index=index, seed=seed,
                           n_items=self.n_items, n_moves=self.n_moves, items=items)

    def _retitle(self, rnd, index: int):
        """复用上一轮时只改样例图里的轮次号，摆放和指令保持逐位一致。"""
        from dataclasses import replace
        from record.instruction.scene_svg import render_scene
        return replace(rnd, index=index,
                       svg=render_scene(rnd.placements, self._table_geo,
                                        title=f'Round {index}'))

    def preview_round(self, seed: int | None = None, keep_items: bool = False) -> dict:
        """生成一轮摆放但**不写进 session**，可以反复重 roll。

        摆真桌子要几十秒，以前只能开录之后才看得到任务，那段时间全录进了视频里。
        现在 IDLE 下就能预览，摆完再开录。

        ``keep_items`` 沿用当前这组物品，只换摆放和指令 —— 换物品要起身去桌上换东西，
        只挪位置在手边就能做完，两件事的代价差一个数量级。
        """
        with self._lock:
            if self._round_is_live():
                raise RuntimeError('本轮还没结束，不能重 roll')
            items = None
            if keep_items:
                if self._pending_obj is None:
                    raise RuntimeError('还没有可沿用的物品，先生成一次任务')
                items = self._pending_obj.items
            index = self.session.round_index + 1 if self.session else 0
            self._pending_obj = self._build_round(index, seed, items=items)
            self.pending_round = self._pending_obj.as_dict()
            return self.status()

    def start_round(self, seed: int | None = None) -> dict:
        """把预览的那一轮固化进 session。没预览过就现生一个。"""
        with self._lock:
            session = self._require(State.SESSION)
            rnd = self._pending_obj
            if rnd is None or seed is not None:
                rnd = self._build_round(session.round_index + 1, seed)
            elif rnd.index != session.round_index + 1:
                rnd = self._retitle(rnd, session.round_index + 1)
            self._round_obj = rnd
            self.current_round = rnd.as_dict()
            self.pending_round, self._pending_obj = None, None
            self._slot_takes = {}
            session.start_round(self.current_round, svg=rnd.svg)
            for hit in self.current_round.get('lint_warnings', []):
                session.warn('lint', **hit)
            return self.status()

    def end_round(self) -> dict:
        """结束本轮，但**把这一轮退回预览**：桌子已经照它摆好了，再点「开始本轮」
        就是同样的物品、摆放和指令，不用重新摆一次桌子。要换任务再去重 roll。
        """
        with self._lock:
            session = self._require(State.ROUND)
            session.end_round()
            self._pending_obj, self._round_obj = self._round_obj, None
            self.pending_round, self.current_round = self.current_round, None
            self._slot_takes = {}
            return self.status()

    def start_episode(self, index: int) -> dict:
        with self._lock:
            session = self._require(State.ROUND)
            eps = (self.current_round or {}).get('episodes', [])
            if not 0 <= index < len(eps):
                raise RuntimeError(f'episode 序号 {index} 越界')
            session.start_episode(eps[index])
            self._episode_slot = index
            return self.status()

    def end_episode(self, outcome: str, note: str = '') -> dict:
        with self._lock:
            session = self._require(State.EPISODE)
            session.end_episode(outcome, note)
            if self._episode_slot >= 0:
                self._slot_takes.setdefault(self._episode_slot, []).append(
                    {'episode': session.episode_index, 'outcome': outcome})
            if outcome == 'success' and self.library and self._round_obj:
                self._bump_usage()
            return self.status()

    def relabel_take(self, slot: int, take: int, outcome: str) -> dict:
        """改本轮里某一条指令第 ``take`` 次录制的结论。

        录完下一条才发现上一条其实没成是常事，所以正在录（EPISODE）时也允许改。
        ``_slot_takes`` 随轮清空，所以能改的就只有当前这一轮 ——
        封口后的 session 去数据管理面板改。
        """
        with self._lock:
            session = self.session
            if session is None or session.state not in (State.ROUND, State.EPISODE):
                raise RuntimeError('本轮已结束，改不了标注')
            takes = self._slot_takes.get(slot) or []
            if not 0 <= take < len(takes):
                raise RuntimeError(f'第 {slot} 条指令没有第 {take + 1} 次录制')
            session.relabel_episode(takes[take]['episode'], outcome)
            takes[take]['outcome'] = outcome
            return self.status()

    def _bump_usage(self) -> None:
        session = self.session
        library = self.library
        if session is None or library is None:
            return
        idx = self._episode_slot
        eps = (self.current_round or {}).get('episodes', [])
        if not 0 <= idx < len(eps):
            return
        ep = eps[idx]
        if ep.get('obj', {}).get('id'):
            library.bump_usage(ep['obj']['id'], 'as_x')
        if ep.get('target', {}).get('id'):
            library.bump_usage(ep['target']['id'], 'as_y')

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
                self._warn_head_gap()
            schema = {}
            # 先摘掉再关：回调拿不到 writer 就直接返回，不会往已关的表里塞行
            writers, self.writers = self.writers, {}
            for key, w in writers.items():
                w.close()
                entry = w.schema()
                # 文件字节数才是真相，rows 只是个计数器。下游拿 rows 决定读多少行，
                # 对不上就静默截断尾部，而 DONE 里的 sha256 只校字节，两者不交叉。
                real = w.path.stat().st_size // (entry['ncol'] * 8)
                if real != entry['rows']:
                    self.session.warn('table_rows_mismatch', table=key,
                                      declared=entry['rows'], actual=real)
                    entry['rows'] = real
                schema[key] = entry
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
            self._slot_takes = {}
            self._last_write.clear()
            return {'session': root.name, 'files': digest['file_count'],
                    'bytes': digest['total_bytes'], 'path': str(root)}

    def _require(self, state: State) -> Session:
        session = self.session
        if session is None:
            raise RuntimeError('还没开始 session')
        if session.state is not state:
            raise RuntimeError(f'当前是 {session.state.value}，'
                               f'这一步要求 {state.value}')
        return session

    # ------------------------------------------------------------------ 观测

    def stream_overview(self) -> list[dict]:
        """所有数据流一览。开录前用来确认没有哑掉的话题。"""
        now = time.time()
        out = []
        for key, spec in self.specs.items():
            st = self.stats[key]
            out.append({
                'key': key, 'kind': 'signal', 'topic': spec.topic,
                'type': spec.type_name, 'columns': len(cast(list, spec.columns)),
                'received': st['received'], 'written': st['written'],
                **_liveness(st['last'], now),
                'max_hz': spec.max_hz,
                'note': f'！{st["health"]}' if st['health'] else spec.note,
                'default_on': spec.default_on,
                'recording': key in self.writers,
            })
        for name, url in self.wrist_urls.items():
            rec = self.wrists.get(name)
            frames = rec.health.frames if rec else 0
            out.append({
                'key': name, 'kind': 'video', 'topic': url, 'type': 'RTSP (-c copy)',
                'columns': 0, 'received': frames, 'written': frames,
                'online': bool(rec and rec.health.alive), 'age': 0.0,
                'max_hz': 0.0, 'default_on': True, 'recording': name in self.wrists,
                'note': rec.health.error if rec else '未开录',
            })
        seen = self._head_seen
        out.append({
            'key': 'head', 'kind': 'video', 'topic': self.head_topic,
            'type': f'{seen["encoding"] or "?"} {seen["width"]}x{seen["height"]}',
            'columns': 0, 'received': seen['count'],
            'written': self.head.health.frames if self.head else 0,
            **_liveness(seen['last'], now),
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
            # 面板要高亮的是指令表里的那一行，不是上面那个次数
            'episode_slot': (self._episode_slot
                             if s is not None and s.state is State.EPISODE else -1),
            'counts': dict(s.counts) if s else {},
            'bytes': sum(w.bytes_written for w in self.writers.values()),
            'disk_free_gb': round(disk / 1e9, 1),
            'library_error': self.library_error,
            'table': self._table,
            'table_locked': self._round_is_live(),
            'round_detail': self.current_round,
            'pending_round': self.pending_round,
            # JSON 的键只能是字符串，前端按 takes[i] 取（JS 会自动转成 "i"）
            'slot_takes': {str(k): v for k, v in self._slot_takes.items()},
            'peer_port': self.peer_port,
            'peer_alive': self._peer_alive('data_manager'),
        }

    def _peer_alive(self, name: str, ttl: float = 5.0) -> bool:
        """对方面板在不在。问 ROS 图而不是去探那个端口 —— 跨端口探测会往控制台灌报错。"""
        now = time.monotonic()
        if now - self._peer[0] > ttl:
            self._peer = (now, name in self.get_node_names())
        return self._peer[1]

    def _disk(self) -> int:
        try:
            return shutil.disk_usage(self.root if self.root.exists()
                                     else self.root.parent).free
        except OSError:
            return 0

    def _warn_head_gap(self) -> None:
        """头部没收齐帧就把同期的 UDP 溢出计数一并记下。

        实机出过 24.9/30，但在能复现的负载（真控制栈 + 三路录制 + 合成信号）下一次都
        没重现，差的是头显真连着推流那份负载。下次再发生时这条告警能直接分开
        「丢在 UDP 分片」和「丢在别处」，不用再猬。
        """
        got = self._head_seen['count'] - self._head_seen0
        want = self.head_fps * (time.time() - self._head_t0)
        if want <= 0 or got >= want * 0.97:
            return
        err = _udp_rcvbuf_errors() - self._udp_err0
        if self.session is not None:
            self.session.warn('head_frames_short', got=got, want=int(want),
                              lost=int(want - got), rcvbuf_errors=err,
                              queue_drop=self.head.dropped if self.head else 0)

    def pending_svg(self) -> str:
        return self._pending_obj.svg if self._pending_obj is not None else ''

    def snapshot(self, key: str) -> bytes:
        """开录前抓一帧确认画面。**只在未录制时可用，且只抓一帧。**

        帧计数和在线点能发现「流哑了」，但发现不了「相机对着墙」或者被挡住了，
        那得看一眼画面。连续预览不行：实测解码是 65% 单核每路，三路就是两个核，
        录制期间绝不能加这个负载。单帧是一次性的，腕部约 1.3 s（RTSP 握手 + 等 IDR）。
        """
        with self._lock:
            if self.session is not None:
                raise RuntimeError('正在录制，不抓快照 —— 解码会跟录制抢 CPU')
        if key == 'head':
            return self._head_snapshot()
        url = self.wrist_urls.get(key)
        if url is None:
            raise ValueError(f'未知的视频流 {key}')
        out = subprocess.run(
            ['ffmpeg', '-nostdin', '-v', 'error', '-rtsp_transport', 'tcp',
             '-stimeout', '5000000', '-i', url, '-frames:v', '1',
             '-vf', 'scale=640:-2', '-f', 'mjpeg', 'pipe:1'],
            capture_output=True, timeout=30, stdin=subprocess.DEVNULL)
        if out.returncode != 0 or len(out.stdout) < 512:
            raise RuntimeError(f'抓不到画面: {out.stderr.decode("utf-8", "replace")[:200]}')
        return out.stdout

    def _head_snapshot(self) -> bytes:
        """头部走 ROS，最近一帧已经在内存里，编码成 JPEG 即可。"""
        frame = self._head_last
        if frame is None:
            raise RuntimeError('还没收到头部图像')
        payload, width, height, encoding = frame
        pix = HeadRecorder.PIX.get(encoding)
        if pix is None:
            raise RuntimeError(f'头部相机编码 {encoding} 不支持')
        out = subprocess.run(
            ['ffmpeg', '-nostdin', '-v', 'error', '-f', 'rawvideo',
             '-pix_fmt', pix, '-s', f'{width}x{height}', '-i', 'pipe:0',
             '-frames:v', '1', '-vf', 'scale=640:-2', '-f', 'mjpeg', 'pipe:1'],
            input=payload, capture_output=True, timeout=30)
        if out.returncode != 0 or len(out.stdout) < 512:
            raise RuntimeError(f'头部编码失败: {out.stderr.decode("utf-8", "replace")[:200]}')
        return out.stdout

    # ------------------------------------------------------------- 低帧率预览

    #: 预览帧的最短间隔，和 ``PreviewPump.FPS`` 对齐
    PREVIEW_PERIOD = 0.45

    def preview(self, key: str) -> bytes:
        """连续预览的最近一帧 JPEG。**录制中照样可用**，代价和快照差一个量级。

        腕部走 640x360 子码流常驻解码（实测 7% 单核/路），既不碰 ``-c copy`` 搬字节
        的主码流那一路，也不用每次重新握手；头部的帧本来就在内存里，只需编码，
        同样交给一个常驻的 ffmpeg。三路加起来不到四分之一个核。
        """
        if key == 'head':
            return self._head_pump.frame()
        url = self.wrist_urls.get(key)
        if url is None:
            raise ValueError(f'未知的视频流 {key}')
        with self._preview_lock:
            pump = self._previews.get(key)
            if pump is None:
                pump = self._previews[key] = PreviewPump(key, preview_url(url))
        return pump.frame()

    def _head_due(self) -> bool:
        """距上次给头部留帧够不够一个预览周期。

        每帧都留 = 30 x 1.8 MB/s 的白拷贝；一帧不留则预览会冻在开录前那一帧，
        画面看着像流断了（实际 ROS 流和落盘都正常）。
        """
        now = time.monotonic()
        if now - self._head_kept < self.PREVIEW_PERIOD:
            return False
        self._head_kept = now
        return True

    def _reap_previews(self) -> None:
        with self._preview_lock:
            stale = [k for k, p in self._previews.items() if p.idle()]
            pumps = [self._previews.pop(k) for k in stale]
        if self._head_pump.idle():
            pumps.append(self._head_pump)
        for pump in pumps:
            pump.stop()

    def stop_previews(self) -> None:
        with self._preview_lock:
            pumps, self._previews = list(self._previews.values()), {}
        for pump in pumps:
            pump.stop()
        self._head_pump.stop()


def _on_sigterm(signum, frame):   # noqa: ARG001
    raise KeyboardInterrupt


def main() -> None:
    rclpy.init()
    node = Recorder()
    from record.dashboard import panel
    dash = panel(node, port=cast(
        int, node.declare_parameter('dashboard_port', 8220).value))
    dash.start()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    # Python 对 SIGTERM 没有处理器，默认动作直接终止进程，下面的 finally 不会跑，
    # ffmpeg 子进程就成了孤儿，继续空烧 CPU 和写盘。docker stop / systemd 关闭走的
    # 都是 SIGTERM。SIGINT 交给 rclpy 自带的处理器，实测 1 s 内退出并把 session 封口。
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
        node.stop_previews()
        dash.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
