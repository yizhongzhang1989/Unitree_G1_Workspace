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

SCHEMA_VERSION = 1


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

    @classmethod
    def create(cls, root: str | os.PathLike, streams: dict, meta: dict | None = None,
               session_id: str | None = None) -> 'Session':
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
                   session_id=session_id)
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
        if outcome not in ('success', 'fail', 'discard'):
            raise SessionError(f'未知标注 {outcome}')
        self.log.emit('episode_end', round=self.round_index,
                      episode=self.episode_index, outcome=outcome, note=note,
                      duration=time.time() - self.episode_started)
        self.counts[outcome] += 1
        if outcome != 'discard':
            self.counts['episodes'] += 1
        self.state = State.ROUND

    def warn(self, code: str, **detail) -> None:
        """Spec §1.5：运行时只记录警告，绝不阻塞落盘。"""
        self.log.emit('warning', code=code, detail=detail)

    # ------------------------------------------------------------------- 封口

    def finish(self, schema: dict) -> dict:
        if self.state is State.EPISODE:
            self.end_episode('discard', note='session 结束时仍在录')
        if self.state is State.ROUND:
            self.end_round()
        self.paths.schema.write_text(
            json.dumps({'schema_version': SCHEMA_VERSION, 'tables': schema},
                       ensure_ascii=False, indent=1), encoding='utf-8')
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
