"""session 目录布局、事件线与封口。

三层时间粒度：**session（一次连续录制）> round（一次摆桌）> episode（一个原子动作）**。
视频和信号表全程连续写，episode 只是 ``events.jsonl`` 里的一对时间戳 —— 每条 episode
重开 ffmpeg 会让每段开头 1.2 s 全废（RTSP 冷启动实测值）。

``manifest.json`` 在开录瞬间冻结：勾了哪些数据流、软硬件版本、时间基准。开录后不允许
再改，否则 ``schema.json`` 与实际落盘内容会对不上，而这种错是静默的。
"""

from __future__ import annotations

import hashlib
import json
import os
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

from record import camera_params

_CAMERA_PARAMS_FILE = camera_params.FILENAME

SCHEMA_VERSION = 1
#: 一条 episode 的三种结论。改标注和收尾都按它校验。
OUTCOMES = ('success', 'fail', 'discard')


class State(str, Enum):
    IDLE = 'idle'
    SESSION = 'session'      # 已开录，未开 round
    ROUND = 'round'          # 桌面已摆好，可以逐条执行
    EPISODE = 'episode'      # 正在录一条


class SessionError(RuntimeError):
    pass


@dataclass(frozen=True)
class SessionPaths:
    """一次采集的目录布局。全部由 ``root`` 派生，别的地方不要再拼路径。"""

    root: Path

    video = property(lambda s: s.root / 'video')
    signals = property(lambda s: s.root / 'signals')
    rounds = property(lambda s: s.root / 'rounds')
    events = property(lambda s: s.root / 'events.jsonl')
    manifest = property(lambda s: s.root / 'manifest.json')
    schema = property(lambda s: s.root / 'schema.json')
    meta = property(lambda s: s.root / 'meta.json')
    camera_params = property(lambda s: s.root / _CAMERA_PARAMS_FILE)
    done = property(lambda s: s.root / 'DONE')

    def make(self) -> None:
        for d in (self.root, self.video, self.signals, self.rounds):
            d.mkdir(parents=True, exist_ok=True)


class EventLog:
    """只追加的时间线。视频、信号、指令三者靠它缝在一起。"""

    def __init__(self, path: str | os.PathLike) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.path, 'a', encoding='utf-8')

    def emit(self, kind: str, **fields) -> dict:
        event = {'t': time.time(), 'type': kind, **fields}
        self._fh.write(json.dumps(event, ensure_ascii=False) + '\n')
        self._fh.flush()
        return event

    def close(self) -> None:
        if not self._fh.closed:
            self._fh.flush()
            os.fsync(self._fh.fileno())
            self._fh.close()


@dataclass
class Session:
    """一次录制会话的状态机与目录。

    只管「什么时候该发生什么」和「往哪写」，不碰 ffmpeg 也不碰 ROS —— 那些由
    ``recorder_node`` 组装，这样这一层能脱离机器人单测。
    """

    paths: SessionPaths
    manifest: dict
    log: EventLog
    state: State = State.IDLE
    session_id: str = ''
    round_index: int = -1
    episode_index: int = -1
    episode_started: float = 0.0
    counts: dict = field(default_factory=lambda: {'rounds': 0, 'episodes': 0,
                                                  'success': 0, 'fail': 0,
                                                  'discard': 0})
    #: (round, episode) -> 当前结论。改标注要按它把旧的那一笔计数退回来
    outcomes: dict = field(default_factory=dict)
    #: 开录那一刻生效的标定（已解析的 ``calibration.yaml``），封口时裁成
    #: ``camera_params.yaml``。为 None 就不写 —— 导出侧会明确报缺，不会静默降级。
    calibration: dict | None = None

    @classmethod
    def create(cls, root: str | os.PathLike, streams: dict, meta: dict | None = None,
               session_id: str | None = None,
               calibration: dict | None = None) -> 'Session':
        session_id = session_id or time.strftime('%Y%m%d_%H%M%S')
        paths = SessionPaths(Path(root) / session_id)
        if paths.root.exists() and any(paths.root.iterdir()):
            raise SessionError(f'{paths.root} 已存在且非空')
        paths.make()
        manifest = {
            'schema_version': SCHEMA_VERSION,
            'session_id': session_id,
            'created': time.time(),
            'created_iso': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            # 开录瞬间冻结。之后再改会让 schema.json 与实际内容静默错位。
            'streams': {k: bool(v) for k, v in streams.items()},
            'time_base': 'CLOCK_REALTIME (time.time)',
        }
        paths.manifest.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=1), encoding='utf-8')
        if meta:
            paths.meta.write_text(
                json.dumps(meta, ensure_ascii=False, indent=1), encoding='utf-8')
        self = cls(paths=paths, manifest=manifest, log=EventLog(paths.events),
                   session_id=session_id, calibration=calibration)
        self.state = State.SESSION
        self.log.emit('session_start', session_id=session_id,
                      streams=manifest['streams'])
        return self

    # ------------------------------------------------------------------ round

    def start_round(self, round_dict: dict, svg: str = '') -> int:
        if self.state is not State.SESSION:
            raise SessionError(f'{self.state.value} 状态下不能开 round')
        self.round_index += 1
        self.episode_index = -1
        payload = dict(round_dict, round=self.round_index)
        (self.paths.rounds / f'round_{self.round_index:03d}.json').write_text(
            json.dumps(payload, ensure_ascii=False, indent=1), encoding='utf-8')
        if svg:
            (self.paths.rounds / f'round_{self.round_index:03d}.svg').write_text(
                svg, encoding='utf-8')
        self.state = State.ROUND
        self.counts['rounds'] += 1
        self.log.emit('round_start', round=self.round_index,
                      seed=payload.get('seed'),
                      items=[i['item_id'] for i in payload.get('items', [])],
                      episodes=len(payload.get('episodes', [])))
        return self.round_index

    def end_round(self) -> None:
        if self.state is not State.ROUND:
            raise SessionError(f'{self.state.value} 状态下不能结束 round')
        self.log.emit('round_end', round=self.round_index)
        self.state = State.SESSION

    # ---------------------------------------------------------------- episode

    def start_episode(self, instruction: dict) -> int:
        if self.state is not State.ROUND:
            raise SessionError('要先开 round 才能录 episode')
        self.episode_index += 1
        self.episode_started = time.time()
        self.state = State.EPISODE
        self.log.emit('episode_start', round=self.round_index,
                      episode=self.episode_index, instruction=instruction)
        return self.episode_index

    def end_episode(self, outcome: str, note: str = '') -> None:
        if self.state is not State.EPISODE:
            raise SessionError('没有正在录的 episode')
        if outcome not in OUTCOMES:
            raise SessionError(f'未知标注 {outcome}')
        self.log.emit('episode_end', round=self.round_index,
                      episode=self.episode_index, outcome=outcome, note=note,
                      duration=time.time() - self.episode_started)
        self.outcomes[(self.round_index, self.episode_index)] = outcome
        self.counts[outcome] += 1
        if outcome != 'discard':
            self.counts['episodes'] += 1
        self.state = State.ROUND

    def relabel_episode(self, episode: int, outcome: str,
                        round_index: int | None = None) -> str:
        """改一条已录完 episode 的结论，返回改之前是什么。

        当场标的「成功」事后回看常常是失败的，所以这条路必须有。**事件线只追加**，
        原来那行 ``episode_end`` 原样留着 —— 读侧按最后一条 ``episode_relabel`` 为准。
        """
        if outcome not in OUTCOMES:
            raise SessionError(f'未知标注 {outcome}')
        rnd = self.round_index if round_index is None else round_index
        was = self.outcomes.get((rnd, episode))
        if was is None:
            raise SessionError(f'r{rnd}e{episode} 没有录完的记录，改不了标注')
        if was == outcome:
            return was
        self.counts[was] -= 1
        self.counts[outcome] += 1
        # 「丢弃」不算交付的 episode，进出这一档要同时改总数
        if was == 'discard':
            self.counts['episodes'] += 1
        elif outcome == 'discard':
            self.counts['episodes'] -= 1
        self.outcomes[(rnd, episode)] = outcome
        self.log.emit('episode_relabel', round=rnd, episode=episode,
                      outcome=outcome, was=was)
        return was

    def warn(self, code: str, **detail) -> None:
        """Spec §1.5：运行时只记录警告，绝不阻塞落盘。"""
        self.log.emit('warning', code=code, detail=detail)

    # ------------------------------------------------------------------- 封口

    def _write_camera_params(self) -> None:
        """把这次采集用的相机参数裁进 session，导出时不必再猜配哪一版标定。

        放在封口前而不是开录时：腕相机的分辨率要等后台 ffprobe 探完写进
        ``video/nominal.json``。标定内容取的是开录那一刻的，中途改文件不影响。
        """
        if self.calibration is None:
            self.warn('camera_params_missing', reason='开录时没拿到标定文件')
            return
        try:
            camera_params.write(self.paths.root, self.calibration)
        except (OSError, ValueError) as exc:
            self.warn('camera_params_failed', error=str(exc))

    def finish(self, schema: dict) -> dict:
        if self.state is State.EPISODE:
            self.end_episode('discard', note='session 结束时仍在录')
        if self.state is State.ROUND:
            self.end_round()
        self.paths.schema.write_text(
            json.dumps({'schema_version': SCHEMA_VERSION, 'tables': schema},
                       ensure_ascii=False, indent=1), encoding='utf-8')
        self._write_camera_params()
        self.log.emit('session_end', counts=dict(self.counts))
        self.log.close()
        digest = write_done(self.paths)
        self.state = State.IDLE
        return digest


def write_done(paths: SessionPaths) -> dict:
    """封口：算全目录 sha256 写进 DONE。同步守护只搬带 DONE 的 session。"""
    files = {}
    for f in sorted(paths.root.rglob('*')):
        if not f.is_file() or f.name == 'DONE':
            continue
        h = hashlib.sha256()
        with open(f, 'rb') as fh:
            for block in iter(lambda: fh.read(1 << 20), b''):
                h.update(block)
        files[str(f.relative_to(paths.root))] = {'sha256': h.hexdigest(),
                                                 'bytes': f.stat().st_size}
    payload = {'session_id': paths.root.name, 'sealed': time.time(),
               'file_count': len(files),
               'total_bytes': sum(v['bytes'] for v in files.values()),
               'files': files}
    tmp = paths.done.with_suffix('.tmp')
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=1), encoding='utf-8')
    os.replace(tmp, paths.done)
    return payload


def read_events(path: str | os.PathLike) -> list[dict]:
    """读事件线。末行可能因崩溃而不完整，丢掉它而不是让整个 session 读不出来。"""
    out = []
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            break
    return out
