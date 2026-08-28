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
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from record.replay import Playback, load_commands, pose_from_status, ramp
from record.replay_source import describe, edits_path, episode_label, open_session
from record.webui import tree_entries

# 转换器注册表在 tools/ 里 —— 那个目录整个拷到导出机就能用。面板不另写一份，
# 否则下拉框里列的和 B 上真跑的会分叉。
TOOLS_DIR = Path(get_package_share_directory('record')) / 'tools'
sys.path.insert(0, str(TOOLS_DIR))
import converters                                          # noqa: E402

DEFAULT_ROOT = str(Path.home() / '.ros' / 'record' / 'sessions')
DEFAULT_BUNDLES = str(Path.home() / '.ros' / 'record' / 'bundles')
#: `_pump` 在子进程被 `terminate()` 掉时返回它。取消不是故障，调用方据此区别对待。
CANCELLED = '已取消'
_QOS = QoSProfile(depth=4, history=HistoryPolicy.KEEP_LAST,
                  reliability=ReliabilityPolicy.RELIABLE)
_STREAMS = ('wrist_left', 'wrist_right', 'head')


def _dir_size(path: Path) -> int:
    return sum(f.stat().st_size for f in path.rglob('*') if f.is_file())


def _deleted_labels(session: Path) -> set:
    """这次采集里被删掉的 episode。读不出就当没删过 —— 多显示一条比整页列不出来好。"""
    path = edits_path(session)
    if not path.is_file():
        return set()
    try:
        return set(json.loads(path.read_text(encoding='utf-8')).get('deleted') or [])
    except (OSError, ValueError):
        return set()


def _child(parent: Path, name: str, what: str) -> Path:
    """把外来的名字收敛成 ``parent`` 下的**直接**子项，挡掉路径穿越。

    session 目录和临时产物目录共用这一套 —— 后者的调用方是个 rmtree，两边的判据
    一旦分叉，漏掉的那一边删的就是采集数据。
    """
    if not name or '/' in name or '\\' in name or '..' in name:
        raise ValueError(f'{what} 非法: {name!r}')
    path = (parent / name).resolve()
    if path.parent != parent.resolve():
        raise ValueError(f'{what} 非法: {name!r}')
    return path


def _share(package: str, *parts: str) -> str:
    """包里的文件路径，没装就空串。

    URDF 和标定都不在 session 里（看 README 里 FK 那节），但 A 上它们就跟包装着，
    自己找得到就不该让人在面板上填路径。B 上没 ROS，那边走 ``convert.py`` 手动传。
    """
    try:
        path = Path(get_package_share_directory(package)).joinpath(*parts)
    except PackageNotFoundError:
        return ''
    return str(path) if path.is_file() else ''


class DataManager(Node):
    def __init__(self) -> None:
        super().__init__('data_manager')
        p = self.declare_parameter
        self.root = Path(p('sessions_root', DEFAULT_ROOT).value)
        # 临时产物目录。**必须在 session 目录之外**：不然下一次 DONE 校验会把它
        # 当成多余，而且删 session 会连它一起删。下完就删，不长期占盘。
        self._bundles = Path(p('bundles_root', DEFAULT_BUNDLES).value)
        self._bundles.mkdir(parents=True, exist_ok=True)
        self.bundle_ttl_s = float(p('bundle_ttl_s', 1800.0).value)
        self.urdf = str(p('urdf', '').value) or _share(
            'unitree_g1_description', 'model', 'final.urdf')
        self.calibration = str(p('calibration', '').value) or _share(
            'camera_calibration', 'config', 'calibration.yaml')
        self.video_height = int(p('convert_video_height', 360).value)
        self.render_fps = float(p('render_fps', 10.0).value)
        self.render_width = int(p('render_width', 640).value)
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
        self._convert = {'running': False, 'session': '', 'format': '', 'token': '',
                         'log': [], 'error': '', 'done': False, 'bytes': 0,
                         'progress': 0.0}
        self._render = {'running': False, 'session': '', 'label': '',
                        'log': [], 'error': '', 'done': False, 'bytes': 0,
                        'progress': 0.0}
        #: 当前那个吐 `@progress` 的子进程。转换与渲染互斥，所以只有一个槽。
        self._proc: subprocess.Popen | None = None
        #: 已封口 session 的概要。面板 1 Hz 轮询 sessions()，而一次概要要递归 stat 整个
        #: 目录、逐行解 events.jsonl（能到几 MB）；封口后这些数字不会再变，算一次就够。
        self._summary: dict[str, dict] = {}
        #: 转换格式与依赖齐不齐，同样被 1 Hz 轮询，见 `formats()`
        self._formats: list[dict] | None = None
        # 上一轮没取走的临时产物在盘上，而 token 随进程没了，再也取不走
        self._sweep_bundles(ttl=0.0)
        self.get_logger().info(f'数据管理节点就绪，session 根目录 {self.root}')

    # ------------------------------------------------------------------ 浏览

    def _dir(self, session_id: str) -> Path:
        d = _child(self.root, session_id, 'session id')
        if not d.is_dir():
            raise ValueError(f'找不到 session {session_id}')
        return d

    def sessions(self) -> list[dict]:
        """列出全部 session。没封口的也列出来并标注 —— 操作者需要知道它存在。"""
        out = []
        for d in sorted(self.root.glob('*/'), reverse=True):
            if not (d / 'manifest.json').is_file():
                continue
            item = self._summary.get(d.name) or self._scan(d)
            if item['sealed']:
                self._summary[d.name] = item
            out.append(item)
        return out

    @staticmethod
    def _scan(d: Path) -> dict:
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
            # 标注可以事后改、整条可以事后删，所以不能边扫边计数：先收敛成最终结论
            outcomes: dict[tuple, str] = {}
            for line in (d / 'events.jsonl').read_text(encoding='utf-8').splitlines():
                e = json.loads(line)
                if e['type'] in ('episode_end', 'episode_relabel'):
                    outcomes[(e['round'], e['episode'])] = e.get('outcome', '')
                elif e['type'] == 'warning':
                    item['warnings'] += 1
        except (OSError, ValueError):
            return item
        dropped = _deleted_labels(d)
        for (rnd, ep), outcome in outcomes.items():
            if episode_label(rnd, ep) in dropped:
                continue
            item['episodes'] += 1
            item['success'] += outcome == 'success'
        return item

    def detail(self, session_id: str) -> dict:
        d = self._dir(session_id)
        info = describe(d)
        info['sealed'] = (d / 'DONE').is_file()
        info['bytes'] = _dir_size(d)
        info['streams'] = [s for s in _STREAMS if (d / 'video' / f'{s}.mkv').is_file()]
        # 哪几段已经渲过校验视频。存在盘上而不是内存里，所以重启节点也还在
        info['verified'] = {p.stem: p.stat().st_size
                            for p in sorted((d / 'verify').glob('*.mp4'))}
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
            if self._convert['running'] and self._convert['session'] == session_id:
                raise RuntimeError('这次采集正在转换，先等它跑完')
            if self._render['running'] and self._render['session'] == session_id:
                raise RuntimeError('这次采集正在渲染校验视频，先取消')
        size = _dir_size(d)
        shutil.rmtree(d)
        self._summary.pop(session_id, None)
        self.get_logger().warning(f'已删除 {session_id}（{size / 1e6:.1f} MB）')
        return {'id': session_id, 'bytes': size}

    def delete_episode(self, session_id: str, label: str) -> dict:
        """删掉一条 episode。

        视频和信号表全程连续写，一条 episode 只是事件线上的一对时间戳 —— 真去剪那段
        素材得把整份视频重编码一遍，而 ``DONE`` 的 sha256 当场就对不上了。所以删除记在
        ``edits.json`` 这个旁挂文件里（``DONE`` 只核对它列出的那些文件），
        ``session_reader.episodes()`` 会跳过它：回放、校验、导出都再也看不到这一条。
        """
        d = self._dir(session_id)
        if not (d / 'DONE').is_file():
            raise RuntimeError(f'{session_id} 没有 DONE，可能还在录，不改')
        with self._lock:
            playing = self.state['playing'] and self.state['session'] == session_id
            if playing and self.state['label'] == label:
                raise RuntimeError('这一段正在回放，先停止')
            if (self._render['running'] and self._render['session'] == session_id
                    and self._render['label'] == label):
                raise RuntimeError('这一段正在渲染校验视频，先取消')
        known = {e['label'] for e in open_session(d).episodes(
            include_discarded=True, include_deleted=True)}
        if label not in known:
            raise ValueError(f'{session_id} 里没有 {label}')
        path = edits_path(d)
        edits = {}
        if path.is_file():
            try:
                edits = json.loads(path.read_text(encoding='utf-8'))
            except ValueError:
                edits = {}
        edits['deleted'] = list(dict.fromkeys([*(edits.get('deleted') or []), label]))
        tmp = path.with_suffix('.tmp')
        tmp.write_text(json.dumps(edits, ensure_ascii=False, indent=1), encoding='utf-8')
        os.replace(tmp, path)
        self._summary.pop(session_id, None)
        # 校验视频是这一条的产物，留着就成了无主文件
        self.drop_render(session_id, label)
        self.get_logger().warning(f'{session_id} 删除 episode {label}')
        return {'id': session_id, 'label': label, 'deleted': edits['deleted']}

    # ------------------------------------------------------------------ 转换

    def formats(self) -> list[dict]:
        """面板下拉框的选项。缺依赖的那项带着 missing 一起报出去，好置灰。

        算一次就存着：``describe()`` 会对每个依赖跑 ``find_spec`` + ``which``，
        实测 1.88 ms，而它是被 1 Hz 轮询的 —— 占了这个端点全部 CPU 的四成。
        装没装依赖不会在节点跑着的时候变，真装了重启一下节点。
        """
        if self._formats is None:
            self._formats = converters.describe()
        return self._formats

    def tools_bundle(self) -> list:
        """B 上要的全套：`tools/` 整个 + 两个不在 session 里的配置文件。

        URDF 和标定必须一起给：末端位姿和腕相机外参都要靠 FK 现算，而它俩既不在
        session 里也不在 tools/ 里。**标定尤其不能漏** —— 腕相机的 `camera_left` /
        `camera_right` 两个 link 是它用 `create` 现建的，裸 URDF 里根本没有，
        少了它导出直接报错。缺文件时当场警告，别让人拿着残包到 B 上才发现。
        """
        entries = tree_entries(TOOLS_DIR, 'tools/')
        for path in (self.urdf, self.calibration):
            if path and Path(path).is_file():
                entries.append((Path(path).name, Path(path)))
            else:
                self.get_logger().warn(f'导出工具包里没有 {path or "（未配置）"}，'
                                       'B 上跑转换会缺文件')
        return entries

    def raw_files(self, session_id: str) -> dict:
        """原始数据的文件清单。**逐个下**，不打包 —— 一小时的采集 5.4 GB，
        流式 zip 断了要重来，而单文件带 Range 能续传。"""
        d = self._dir(session_id)
        files = [{'path': f.relative_to(d).as_posix(), 'bytes': f.stat().st_size}
                 for f in sorted(d.rglob('*')) if f.is_file()]
        return {'id': session_id, 'files': files,
                'bytes': sum(f['bytes'] for f in files)}

    def raw_path(self, session_id: str, relative: str) -> Path:
        d = self._dir(session_id)
        path = (d / relative).resolve()
        if d.resolve() not in path.parents or not path.is_file():
            raise ValueError(f'{session_id} 里没有 {relative}')
        return path

    def raw_dir(self, session_id: str, relative: str) -> Path:
        """整个目录打包下载的根。``relative`` 为空就是整次采集。

        打包只是为了「一次点完」，不是为了省流量 —— 里头 mkv 已经压过，ZIP_STORED
        再压也压不动。代价是断了要重来，所以大件还是走 rsync。
        """
        d = self._dir(session_id)
        if not relative:
            return d
        path = (d / relative).resolve()
        if d.resolve() not in path.parents or not path.is_dir():
            raise ValueError(f'{session_id} 里没有目录 {relative}')
        return path

    #: 面板里能直接看的后缀。没后缀的（如 ``DONE``）靠嗅探，不写死名字。
    TEXT_SUFFIX = ('.json', '.jsonl', '.txt', '.yaml', '.yml', '.md', '.csv', '.svg')
    #: 预览截断长度。events.jsonl 能长到几 MB，整份塞给浏览器没意义。
    PREVIEW_LIMIT = 256 << 10

    def preview(self, session_id: str, relative: str) -> dict:
        """文本文件的前若干 KB，给右边的预览用。"""
        path = self.raw_path(session_id, relative)
        size = path.stat().st_size
        blob = path.read_bytes()[:self.PREVIEW_LIMIT]
        text = ''
        if path.suffix.lower() in self.TEXT_SUFFIX or not path.suffix:
            try:
                # 严格解一遍：截断处可能切在多字节中间，掉最后几字节再试
                text = blob.decode('utf-8')
            except UnicodeDecodeError as exc:
                text = blob[:exc.start].decode('utf-8') if exc.start else ''
        binary = not text and bool(blob)
        return {'id': session_id, 'path': relative, 'bytes': size,
                'text': text, 'truncated': size > len(blob), 'binary': binary}

    def start_convert(self, session_id: str, fmt: str) -> dict:
        """现转现下：转到临时目录，下完就删，盘上不留产物。

        为什么不做成「一个请求从头等到尾」：转换是 0.13x 实时，一小时素材要 8 分钟，
        浏览器等首字节必然超时。所以拆成「触发 + 轮询进度 + 取件」三步。

        走子进程而不是 import：转换脚本在 ``tools/`` 里，那边禁止 import rclpy
        （导出机没有 ROS），而且它崩了不该把节点一起带走。
        """
        session = self._dir(session_id)
        converter = converters.get(fmt)
        if not (session / 'DONE').is_file():
            raise RuntimeError(f'{session_id} 没有 DONE，还在录或异常中断，不能转换')
        missing = converter.missing()
        if missing:
            raise RuntimeError(f'跑不了 {fmt}：缺 {"、".join(missing)}')
        with self._lock:
            if self._convert['running']:
                raise RuntimeError(f'{self._convert["session"]} 正在转换，一次只能跑一个')
            if self._render['running']:
                raise RuntimeError('正在渲染校验视频，先取消或等它跑完')
            if self.state['playing']:
                raise RuntimeError('正在回放，先停止')
            token = f'{session_id}.{fmt}.{int(time.time())}'
            self._convert = {'running': True, 'session': session_id, 'format': fmt,
                             'token': token, 'log': [], 'error': '',
                             'done': False, 'bytes': 0, 'progress': 0.0}
        self._sweep_bundles()
        thread = threading.Thread(target=self._run_convert,
                                  args=(session, converter, token), daemon=True)
        thread.start()
        return {'session': session_id, 'format': fmt, 'token': token}

    def convert_state(self) -> dict:
        with self._lock:
            return dict(self._convert, log=list(self._convert['log'])[-40:])

    def _run_convert(self, session: Path, converter, token: str) -> None:
        out = self._bundles / token
        values = {'urdf': self.urdf, 'calibration': self.calibration,
                  'video_height': self.video_height}
        try:
            command = converter.command(sys.executable, session, out, values,
                                        progress=True)
        except ValueError as exc:
            error = str(exc)
        else:
            self.get_logger().info(f'转换 {token}: {" ".join(command)}')
            error = self._pump(command, self._convert)
        with self._lock:
            self._convert.update(running=False, done=not error, error=error,
                                 progress=0.0 if error else 1.0,
                                 bytes=_dir_size(out) if out.is_dir() else 0)
        if error:
            shutil.rmtree(out, ignore_errors=True)
            self.get_logger().error(f'转换 {token} 失败: {error}')
        else:
            self.get_logger().info(f'转换 {token} 完成，{_dir_size(out) / 1e6:.1f} MB')

    def _pump(self, command: list, slot: dict) -> str:
        """跑一个会吐 `@progress` 的子进程，把进度和日志喂进 `slot`。返回错误串，空 = 成功。

        句柄挂在 `self._proc` 上给「取消」用 —— 转换与渲染互斥，同一时刻只会有一个。
        """
        try:
            # PYTHONUNBUFFERED：子进程的 stdout 接的是管道，默认块缓冲，
            # 不加这个就要等它整个跑完才吐字，面板上的进度条等于没有
            # nice：转换/渲染都是 CPU-bound 的后台长任务（渲染实测 65% 单核/路），
            # 而它们经常与采集同时跑。晚几秒出结果无所谓，拖慢控制环不行。
            proc = subprocess.Popen(['nice', '-n', '10', *command],
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True,
                                    stdin=subprocess.DEVNULL,
                                    env={**os.environ, 'PYTHONUNBUFFERED': '1'})
            self._proc = proc
            for line in proc.stdout:
                line = line.rstrip()
                # 进度行不进日志：一秒好几条，会把真正要看的报告冲没
                if line.startswith('@progress '):
                    with self._lock:
                        slot['progress'] = float(line.split()[1])
                    continue
                with self._lock:
                    slot['log'].append(line)
                self.get_logger().info(f'  {line}')
            code = proc.wait()
            if code == -signal.SIGTERM:
                return CANCELLED
            return f'子进程退出码 {code}' if code else ''
        except Exception as exc:                       # noqa: BLE001
            return f'{type(exc).__name__}: {exc}'
        finally:
            self._proc = None

    def bundle(self, token: str) -> Path:
        """取件目录。token 只允许是 bundles 下的直接子目录。"""
        return self._bundle_path(token, must_exist=True)

    def _bundle_path(self, token: str, must_exist: bool) -> Path:
        path = _child(self._bundles, token, 'token')
        if must_exist and not path.is_dir():
            raise ValueError(f'{token} 已经取过或已过期')
        return path

    def drop_bundle(self, token: str) -> None:
        """自己也校验一遍。这是个 rmtree，不能靠「调用方总会先调 bundle()」保平安。"""
        try:
            path = self._bundle_path(token, must_exist=False)
        except ValueError:
            self.get_logger().error(f'拒绝删除非法 token {token!r}')
            return
        shutil.rmtree(path, ignore_errors=True)
        # 取件链接是一次性的。不撤掉的话面板会一直挂着它，再点就是「已经取过」。
        with self._lock:
            if self._convert.get('token') == token:
                self._convert.update(done=False, token='')

    def _sweep_bundles(self, ttl: float | None = None) -> None:
        """清掉过期的临时产物。下载中断没触发删除时，靠这一条兢底。"""
        cutoff = time.time() - (self.bundle_ttl_s if ttl is None else ttl)
        for path in self._bundles.glob('*'):
            if path.is_dir() and path.stat().st_mtime < cutoff:
                shutil.rmtree(path, ignore_errors=True)
                self.get_logger().info(f'清掉过期的临时产物 {path.name}')

    # ------------------------------------------------------------------ 对齐校验

    @staticmethod
    def _render_path(session: Path, label: str) -> Path:
        """校验视频就放在 session 里的 `verify/`。

        不像转换产物那样另辟临时目录 —— 它小（一条 episode 约 0.6 MB）、和这次采集
        绑死、还会反复回看。放进去就白得三件事：删采集时一并带走、同一段重渲原地
        覆盖不会越攒越多、在文件树里能直接看见和下载。
        `DONE` 只核对它自己列出的那些文件，多出来的目录不影响校验。
        """
        return _child(session / 'verify', f'{label or "whole"}.mp4', 'label')

    def start_render(self, session_id: str, t0: float, t1: float, label: str) -> dict:
        """渲染一段对齐校验视频：URDF 轮廓 + IK 目标点叠回头部实拍。

        **离线一次性，不做成跟着回放实时合成。** 渲染实测 0.32 s/帧（640x360），
        比 30 fps 慢两个数量级；而且回放时机器人是真在动的，那会儿不该有东西抢核。
        所以它和回放、转换三者互斥，谁也不许并着跑。
        """
        session = self._dir(session_id)
        if not (session / 'DONE').is_file():
            raise RuntimeError(f'{session_id} 没有 DONE，还在录或异常中断，不能渲染')
        video = self._render_path(session, label)
        with self._lock:
            if self._render['running']:
                raise RuntimeError(f'{self._render["session"]} 正在渲染，一次只能跑一个')
            if self._convert['running']:
                raise RuntimeError('正在转换，先等它跑完')
            if self.state['playing']:
                raise RuntimeError('正在回放，先停止')
            self._render = {'running': True, 'session': session_id, 'label': label,
                            'log': [], 'error': '', 'done': False,
                            'bytes': 0, 'progress': 0.0}
        threading.Thread(target=self._run_render, daemon=True,
                         args=(session, t0, t1, label, video)).start()
        return {'session': session_id, 'label': label}

    def stop_render(self) -> dict:
        """取消渲染。整段采集能渲十几分钟，没有出口就等于把回放也锁死了。"""
        with self._lock:
            proc = self._proc if self._render['running'] else None
        if proc is not None:
            proc.terminate()
        return self.render_state()

    def render_state(self) -> dict:
        with self._lock:
            return dict(self._render, log=list(self._render['log'])[-40:])

    def _run_render(self, session: Path, t0: float, t1: float,
                    label: str, video: Path) -> None:
        # 先渲进暂存目录再搬过去：取消或失败时上一份好的还在，不会被半截视频顶掉。
        # 用目录而不是 `.mp4.part`：**ffmpeg 按扩展名认容器**，`.part` 结尾会报
        # 「Unable to find a suitable output format」，而且报告那个 .txt 也要跟着搬。
        stage = video.parent / '.tmp'
        shutil.rmtree(stage, ignore_errors=True)
        stage.mkdir(parents=True, exist_ok=True)
        staged = stage / video.name
        # 走子进程：渲染是 GIL 下的重活，塞进节点会把 executor 卡住，
        # 而且 pinocchio 加载 URDF 就要 3.6 s，不该常驻在数据管理进程里。
        command = [sys.executable, '-m', 'record.verify_alignment', str(session),
                   '--t0', f'{t0:.6f}', '--t1', f'{t1:.6f}', '--label', label,
                   '--fps', str(self.render_fps), '--width', str(self.render_width),
                   '--out', str(staged), '--progress']
        for flag, value in (('--urdf', self.urdf), ('--calibration', self.calibration)):
            if value:
                command += [flag, value]
        self.get_logger().info(f'渲染 {session.name}/{label}: {" ".join(command)}')
        error = self._pump(command, self._render)
        if not error and not staged.is_file():
            error = '渲染进程没留下视频'
        if error:
            self.get_logger().error(f'渲染 {session.name}/{label} 失败: {error}')
        else:
            for name in (staged, staged.with_suffix('.txt')):
                if name.is_file():
                    name.replace(video.parent / name.name)
            # 子进程报的是暂存路径，照着去找是找不到的，补一行真实位置
            with self._lock:
                self._render['log'].append(
                    f'存放 : {session.name}/verify/{video.name}（连同同名 .txt 报告）')
            self.get_logger().info(
                f'渲染 {session.name}/{label} 完成，{video.stat().st_size / 1e6:.1f} MB')
        shutil.rmtree(stage, ignore_errors=True)
        with self._lock:
            if error == CANCELLED:
                # 取消是正常操作不是故障，清空整个槽 —— 否则卡片上会永久挂着
                # 一条撤不掉的红字和一个空日志框。点取消时前端已经弹过提示了。
                self._render.update(running=False, done=False, session='',
                                    label='', error='', log=[], bytes=0,
                                    progress=0.0)
            else:
                self._render.update(running=False, done=not error, error=error,
                                    progress=0.0 if error else 1.0,
                                    bytes=video.stat().st_size if video.is_file() else 0)

    def render_video(self, session_id: str, label: str) -> Path:
        path = self._render_path(self._dir(session_id), label)
        if not path.is_file():
            raise ValueError(f'{session_id} 还没渲染过 {label or "整段"}')
        return path

    def drop_render(self, session_id: str, label: str) -> dict:
        """删掉一段的校验视频。数据一改它就过时了，留着比没有更容易看错。"""
        with self._lock:
            if (self._render['running'] and self._render['session'] == session_id
                    and self._render['label'] == label):
                raise RuntimeError('这一段正在渲染，先取消')
        path = self._render_path(self._dir(session_id), label)
        size = path.stat().st_size if path.is_file() else 0
        path.unlink(missing_ok=True)
        path.with_suffix('.txt').unlink(missing_ok=True)
        with self._lock:
            if (self._render['session'] == session_id
                    and self._render['label'] == label):
                self._render.update(done=False, bytes=0, error='')
        return {'session': session_id, 'label': label, 'bytes': size}

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
            if self._render['running']:
                raise RuntimeError('正在渲染校验视频，先按「取消」')
            d = self._dir(session_id)
            if not (d / 'DONE').is_file():
                raise RuntimeError(f'{session_id} 没封口，不回放')
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
        st['peer_port'] = self.peer_port
        st['peer_alive'] = self._peer_alive('recorder')
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
