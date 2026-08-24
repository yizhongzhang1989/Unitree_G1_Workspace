"""数据管理节点：浏览、预览、删除、回放已录制的 session。

**回放只是其中一个功能**，所以叫数据管理而不是回放器。

与采集节点分成两个进程：采集必须最简最稳（录制逻辑不能被别的东西拖累），而数据管理
要读全盘、解码取帧、删目录，是完全不同的负载画像。两者可以在同一个 launch 里一起起，
互斥靠 `DONE` 文件天然完成 —— 没封口的 session 删不掉也放不了。

预览一律**按需取单帧**，不做连续解码。实测从已录文件取一帧含 seek 只要 0.21~0.33 s，
而连续解码是 **65% 单核每路**，三路就是两个核 —— 那种东西不能和录制并存。
"""

from __future__ import annotations

import json
import shutil
import signal
import subprocess
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from record.replay import Playback, load_commands, pose_from_status, ramp

DEFAULT_ROOT = str(Path.home() / '.ros' / 'record' / 'sessions')
_QOS = QoSProfile(depth=4, history=HistoryPolicy.KEEP_LAST,
                  reliability=ReliabilityPolicy.RELIABLE)
_STREAMS = ('wrist_left', 'wrist_right', 'head')


def _dir_size(path: Path) -> int:
    return sum(f.stat().st_size for f in path.rglob('*') if f.is_file())


class DataManager(Node):
    def __init__(self) -> None:
        super().__init__('data_manager')
        p = self.declare_parameter
        self.root = Path(p('sessions_root', DEFAULT_ROOT).value)
        self.rate_hz = float(p('rate_hz', 50.0).value)
        self.ramp_s = float(p('ramp_s', 3.0).value)
        self.cmd_topic = p('command_topic', '/motion_control/command').value
        self.status_topic = p('status_topic', '/motion_control/status').value
        self.peer_port = int(p('peer_port', 8220).value)
        self._peer = (0.0, False)

        self._group = ReentrantCallbackGroup()
        self.pub = self.create_publisher(Float64MultiArray, self.cmd_topic, _QOS)
        self.create_subscription(String, self.status_topic, self._on_status,
                                 _QOS, callback_group=self._group)
        # 回放前要 engage，不应该逼着操作者先跑去控制台敲命令再回来点网页
        ns = self.status_topic.rsplit('/', 1)[0] or '/motion_control'
        self._srv = {
            name: self.create_client(Trigger, f'{ns}/{name}',
                                     callback_group=self._group)
            for name in ('engage', 'estop')
        }

        self._lock = threading.Lock()
        self._status: dict = {}
        self._status_t = 0.0
        self._worker: threading.Thread | None = None
        self._stop = threading.Event()
        self.state = {'playing': False, 'session': '', 'label': '',
                      'phase': 'idle', 'progress': 0.0, 'elapsed': 0.0,
                      'duration': 0.0, 'error': ''}
        self.get_logger().info(f'数据管理节点就绪，session 根目录 {self.root}')

    # ------------------------------------------------------------------ 浏览

    def _dir(self, session_id: str) -> Path:
        """把外来的 id 收敛成根目录下的直接子目录，挡掉路径穿越。"""
        if not session_id or '/' in session_id or '\\' in session_id or '..' in session_id:
            raise ValueError(f'session id 非法: {session_id!r}')
        d = (self.root / session_id).resolve()
        if d.parent != self.root.resolve() or not d.is_dir():
            raise ValueError(f'找不到 session {session_id}')
        return d

    def sessions(self) -> list[dict]:
        """列出全部 session。没封口的也列出来并标注 —— 操作者需要知道它存在。"""
        out = []
        for d in sorted(self.root.glob('*/'), reverse=True):
            if not (d / 'manifest.json').is_file():
                continue
            item = {'id': d.name, 'sealed': (d / 'DONE').is_file(),
                    'bytes': _dir_size(d), 'episodes': 0, 'success': 0,
                    'warnings': 0, 'commands': 0}
            try:
                schema = json.loads((d / 'schema.json').read_text(encoding='utf-8'))
                item['commands'] = schema.get('tables', {}).get(
                    'motion_control_command', {}).get('rows', 0)
            except (OSError, ValueError):
                pass
            try:
                for line in (d / 'events.jsonl').read_text(encoding='utf-8').splitlines():
                    e = json.loads(line)
                    if e['type'] == 'episode_end':
                        item['episodes'] += 1
                        item['success'] += e.get('outcome') == 'success'
                    elif e['type'] == 'warning':
                        item['warnings'] += 1
            except (OSError, ValueError):
                pass
            out.append(item)
        return out

    def detail(self, session_id: str) -> dict:
        from record.replay_source import describe
        d = self._dir(session_id)
        info = describe(d)
        info['sealed'] = (d / 'DONE').is_file()
        info['bytes'] = _dir_size(d)
        info['streams'] = [s for s in _STREAMS if (d / 'video' / f'{s}.mkv').is_file()]
        return info

    def frame(self, session_id: str, stream: str, at: float) -> bytes:
        """取某一路某个时刻的单帧 JPEG。按需解码，不做连续预览。"""
        if stream not in _STREAMS:
            raise ValueError(f'未知的流 {stream}')
        mkv = self._dir(session_id) / 'video' / f'{stream}.mkv'
        if not mkv.is_file():
            raise ValueError(f'{session_id} 没有 {stream} 的视频')
        out = subprocess.run(
            ['ffmpeg', '-nostdin', '-v', 'error', '-ss', f'{max(at, 0.0):.3f}',
             '-i', str(mkv), '-frames:v', '1', '-vf', 'scale=640:-2',
             '-f', 'mjpeg', 'pipe:1'],
            capture_output=True, timeout=30, stdin=subprocess.DEVNULL)
        if out.returncode != 0 or len(out.stdout) < 512:
            raise RuntimeError(f'取帧失败: {out.stderr.decode("utf-8", "replace")[:200]}')
        return out.stdout

    def delete(self, session_id: str, confirm: str) -> dict:
        """删一次采集。confirm 必须等于 id：面板用弹窗拦误点，这一条拦的是手拼的请求。"""
        d = self._dir(session_id)
        if confirm != session_id:
            raise ValueError('确认串与 session id 不一致，没有删除')
        if not (d / 'DONE').is_file():
            raise RuntimeError(f'{session_id} 没有 DONE，可能正在录或异常中断，不删')
        with self._lock:
            if self.state['playing'] and self.state['session'] == session_id:
                raise RuntimeError('这次采集正在回放，先停止')
        size = _dir_size(d)
        shutil.rmtree(d)
        self.get_logger().warning(f'已删除 {session_id}（{size / 1e6:.1f} MB）')
        return {'id': session_id, 'bytes': size}

    # ------------------------------------------------------------------ 控制层

    def trigger(self, name: str) -> dict:
        """调控制层的 Trigger 服务。

        急停不能等回放线程：先把发布停了再发服务，否则卡住的一拍还会接着发目标。
        """
        client = self._srv.get(name)
        if client is None:
            raise ValueError(f'未知的服务 {name}')
        if name == 'estop':
            self._stop.set()
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f'{client.srv_name} 没人应答，控制栈起了吗')
        future = client.call_async(Trigger.Request())
        deadline = time.time() + 15.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError(f'{name} 超时未返回')
        res = future.result()
        if not res.success:
            raise RuntimeError(res.message or f'{name} 被拒绝')
        return {'service': name, 'message': res.message}

    # ------------------------------------------------------------------ 回放

    def _on_status(self, msg: String) -> None:
        try:
            self._status = json.loads(msg.data)
            self._status_t = time.time()
        except ValueError:
            pass

    def _require_ready(self) -> np.ndarray:
        age = time.time() - self._status_t
        if not self._status or age > 1.0:
            raise RuntimeError(f'{self.status_topic} 没有数据（{age:.1f}s 前），控制栈起了吗')
        mode = self._status.get('arm_mode')
        if mode != 'ik':
            raise RuntimeError(f'arm_mode 是 {mode}，回放的是末端位姿，必须在 ik 模式下')
        if not self._status.get('arms_live'):
            raise RuntimeError('上肢还没被接管（arms_live 为假）—— 先按右下角「接管上肢」并等站立插值走完')
        try:
            return pose_from_status(self._status)
        except ValueError as exc:
            raise RuntimeError(f'读不出当前位姿：{exc}') from exc

    def start(self, session_id: str, t0: float, t1: float,
              speed: float = 1.0, label: str = '') -> dict:
        with self._lock:
            if self.state['playing']:
                raise RuntimeError('正在回放，先停止')
            d = self._dir(session_id)
            if not (d / 'DONE').is_file():
                raise RuntimeError(f'{session_id} 没封口，不回放')
            from record.replay_source import open_session
            t, arm, grip = load_commands(open_session(d))
            play = Playback(t, arm, grip, t0, t1, speed)
            start_pose = self._require_ready()

            self._stop.clear()
            self.state.update(playing=True, session=session_id, label=label,
                              phase='ramp', progress=0.0, elapsed=0.0,
                              duration=round(play.duration, 2), error='')
            self._worker = threading.Thread(target=self._run, daemon=True,
                                            args=(play, start_pose))
            self._worker.start()
            return dict(self.state)

    def stop(self) -> dict:
        self._stop.set()
        w = self._worker
        if w is not None:
            w.join(timeout=5.0)
        with self._lock:
            self.state.update(playing=False, phase='idle', progress=0.0, elapsed=0.0)
            return dict(self.state)

    def _send(self, arm: np.ndarray, grip: np.ndarray) -> None:
        self.pub.publish(Float64MultiArray(data=[float(x) for x in arm]))
        self.pub.publish(Float64MultiArray(data=[float(x) for x in grip]))

    def _run(self, play: Playback, start_pose: np.ndarray) -> None:
        dt = 1.0 / self.rate_hz
        try:
            first_arm, first_grip, _ = play.sample(0.0)
            targets = ramp(start_pose, first_arm, self.ramp_s, self.rate_hz)
            for i, target in enumerate(targets):
                if self._stop.is_set():
                    return
                self._send(target, first_grip)     # 缓入期间夹爪先到位，避免撞上再开
                with self._lock:
                    self.state['progress'] = (i + 1) / len(targets)
                time.sleep(dt)

            with self._lock:
                self.state.update(phase='play', progress=0.0)
            t0 = time.time()
            while not self._stop.is_set():
                elapsed = time.time() - t0
                arm, grip, done = play.sample(elapsed)
                self._send(arm, grip)
                with self._lock:
                    self.state['elapsed'] = elapsed
                    self.state['progress'] = min(elapsed / max(play.duration, 1e-6), 1.0)
                if done:
                    break
                time.sleep(dt)
        except Exception as exc:                   # noqa: BLE001
            self.get_logger().error(f'回放失败: {exc}')
            with self._lock:
                self.state['error'] = str(exc)
        finally:
            with self._lock:
                self.state.update(playing=False, phase='idle')

    def status(self) -> dict:
        with self._lock:
            st = dict(self.state)
        st['arm_mode'] = self._status.get('arm_mode', '')
        st['arms_live'] = bool(self._status.get('arms_live'))
        st['status_age'] = round(time.time() - self._status_t, 1) if self._status_t else -1.0
        # 就绪与否走和真正开播同一套校验，否则面板说就绪、一点却报错
        try:
            self._require_ready()
            st['ready'], st['blocked'] = True, ''
        except RuntimeError as exc:
            st['ready'], st['blocked'] = False, str(exc)
        try:
            st['disk_free_gb'] = round(shutil.disk_usage(self.root).free / 1e9, 1)
        except OSError:
            st['disk_free_gb'] = -1.0
        st['peer_port'] = self.peer_port if self._peer_alive('recorder') else 0
        return st

    def _peer_alive(self, name: str, ttl: float = 5.0) -> bool:
        """对方面板在不在。问 ROS 图而不是去探那个端口 —— 跨端口探测会往控制台灌报错。"""
        now = time.monotonic()
        if now - self._peer[0] > ttl:
            self._peer = (now, name in self.get_node_names())
        return self._peer[1]


def _on_sigterm(signum, frame):   # noqa: ARG001
    raise KeyboardInterrupt


def main() -> None:
    rclpy.init()
    node = DataManager()
    from record.data_dashboard import panel
    dash = panel(node, port=int(node.declare_parameter('dashboard_port', 8221).value))
    dash.start()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    signal.signal(signal.SIGTERM, _on_sigterm)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        dash.stop()
        node.destroy_node()
        rclpy.try_shutdown()
