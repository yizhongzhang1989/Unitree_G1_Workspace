"""PicoBridge 接入：后台线程收帧、重定向、逐帧回调出去。

头显上的 APK **直连**本模块监听的 ``/ws/device``，中间不过 PicoBridge 那个
``server.py``——少一跳转发、少一个要守的进程，头显配置面板里直接填机器人的
``IP:18000`` 就行。全程 WiFi，不用 adb。

算完的帧走 ``on_frame`` 回调直接发出去，**本模块不攒重定向结果**：要按时间插值的
下游订 ``/mocap/frame``、用 :class:`~.consumer.FrameBuffer` 自己攒。这里只留一份
原始骨架（校准用）和一个单调时间戳（挡乱序帧）。

.. note::
   重定向（含一次 pinocchio FK）跑在收帧线程里，每帧一次、90 Hz 上限。控制环那边
   只做插值，不碰 pinocchio。这样 ``G1Kinematics`` 的不可重入就不会被违反。
"""

from __future__ import annotations

import asyncio
import hmac
import json
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass

import aiohttp
import numpy as np
from aiohttp import web

from .retarget import RetargetCalibration, Retargeter, RetargetResult
from .skeleton import BodyFrame, ClockAligner, both_thumbsticks_pressed, parse_body

# 一帧全身数据约 6.5 KB；给到 1 MB 已经是 150 倍余量，再大就是异常报文。
MAX_MESSAGE_BYTES = 1 << 20

# 原始骨架的留存帧数（90 Hz 下约 2.7 秒），校准和诊断用。
RAW_FRAMES = 240


@dataclass(frozen=True)
class SampleBatch:
    """缓冲区在一批时刻上的插值结果，首维是时刻。

    位置都在动捕自己的坐标系里，还没做向机器人坐标系的对齐。
    """

    t: np.ndarray
    joint_pos: np.ndarray
    root_pos: np.ndarray
    root_quat: np.ndarray
    anchor_pos: np.ndarray
    anchor_quat: np.ndarray
    key_pos: np.ndarray

    def at(self, index) -> SampleBatch:
        """按首维取子集。下游一次取两批时刻（当前 + 前一拍）再劈开做差分。"""
        return SampleBatch(*(np.asarray(v)[index] for v in
                             (self.t, self.joint_pos, self.root_pos, self.root_quat,
                              self.anchor_pos, self.anchor_quat, self.key_pos)))


@dataclass
class StreamStats:
    frames: int = 0
    dropped: int = 0
    status: int = 0
    message: int = 0
    connected: bool = False
    last_error: str = ''


def _nlerp(a: np.ndarray, b: np.ndarray, alpha: np.ndarray) -> np.ndarray:
    """四元数插值。相邻两帧最多差 1/72 秒，nlerp 与 slerp 的差别在 1e-6 量级。"""
    sign = np.where(np.sum(a * b, axis=-1, keepdims=True) >= 0.0, 1.0, -1.0)
    out = a + alpha[..., None] * (sign * b - a)
    norm = np.linalg.norm(out, axis=-1, keepdims=True)
    return np.where(norm > 1e-9, out / np.maximum(norm, 1e-12), a)


class _RingBuffer:
    """定长、按时间单调递增的样本环。读写都在同一把锁里。"""

    def __init__(self, capacity: int, n_joints: int, n_keys: int) -> None:
        self._capacity = int(capacity)
        self._t = np.zeros(self._capacity)
        self._joint = np.zeros((self._capacity, n_joints))
        self._root_pos = np.zeros((self._capacity, 3))
        self._root_quat = np.zeros((self._capacity, 4))
        self._anchor_pos = np.zeros((self._capacity, 3))
        self._anchor_quat = np.zeros((self._capacity, 4))
        self._key_pos = np.zeros((self._capacity, n_keys, 3))
        self._count = 0
        self._head = 0
        self._lock = threading.Lock()

    def push(self, t: float, result: RetargetResult) -> bool:
        with self._lock:
            if self._count and t <= self._t[(self._head - 1) % self._capacity]:
                return False  # 乱序帧直接丢：时间轴必须单调，否则插值会取到反向区间
            i = self._head
            self._t[i] = t
            self._joint[i] = result.joint_pos
            self._root_pos[i] = result.root_pos
            self._root_quat[i] = result.root_quat
            self._anchor_pos[i] = result.anchor_pos
            self._anchor_quat[i] = result.anchor_quat
            self._key_pos[i] = result.key_pos
            self._head = (i + 1) % self._capacity
            self._count = min(self._count + 1, self._capacity)
            return True

    def span(self) -> tuple[float, float] | None:
        with self._lock:
            if self._count < 2:
                return None
            oldest = (self._head - self._count) % self._capacity
            newest = (self._head - 1) % self._capacity
            return float(self._t[oldest]), float(self._t[newest])

    def sample(self, times: np.ndarray) -> SampleBatch | None:
        """在一把锁里把整批时刻插完。越界一律**钳位**到端点，不外推。

        参考窗口每拍要 21 个 token、每个 token 还要一个前一拍做差分，一次性取完才
        能避开每拍 42 次加锁；外推出来的速度会直接喂给策略，更不能要。
        """
        times = np.atleast_1d(np.asarray(times, dtype=np.float64))
        with self._lock:
            if self._count < 2:
                return None
            start = (self._head - self._count) % self._capacity
            order = (np.arange(self._count) + start) % self._capacity
            stamps = self._t[order]
            position = np.clip(np.searchsorted(stamps, times), 1, self._count - 1)
            lo, hi = order[position - 1], order[position]
            width = stamps[position] - stamps[position - 1]
            alpha = np.clip((times - stamps[position - 1])
                            / np.where(width > 1e-9, width, 1.0), 0.0, 1.0)

            def mix(array: np.ndarray) -> np.ndarray:
                extra = (1,) * (array.ndim - 1)
                weight = alpha.reshape(alpha.shape + extra)
                return array[lo] + weight * (array[hi] - array[lo])

            return SampleBatch(
                t=stamps[position - 1] + alpha * width,
                joint_pos=mix(self._joint),
                root_pos=mix(self._root_pos),
                root_quat=_nlerp(self._root_quat[lo], self._root_quat[hi], alpha),
                anchor_pos=mix(self._anchor_pos),
                anchor_quat=_nlerp(self._anchor_quat[lo], self._anchor_quat[hi], alpha),
                key_pos=mix(self._key_pos),
            )


class MocapStream:
    """收帧线程：把头显推来的骨架重定向成 G1 关节角，逐帧回调给 ``on_frame``。

    Args:
        retargeter: SMPL -> G1 的重定向器，只在收帧线程里用。
        host / port: 上行服务的监听地址。头显连 ``ws://<host>:<port>/ws/device``。
        token: 非空时所有接口要带 ``?token=``。放开放网络上时务必设。
        log: 单参数的日志回调，节点把 ``get_logger().info`` 传进来。
    """

    def __init__(self, retargeter: Retargeter, *, host: str = '0.0.0.0',
                 port: int = 18000, token: str = '',
                 log: Callable[[str], None] = print) -> None:
        self._retarget = retargeter
        self._host, self._port, self._token = host, int(port), token
        self._log = log
        self._clock = ClockAligner()
        # 只用来挡乱序帧。重定向结果不在这边攒：本类只管算完回调，要按时间
        # 插值的下游用自己的 FrameBuffer（订 /mocap/frame）。只在收帧线程里读写。
        self._last_stamp = float('-inf')
        self._calibration: RetargetCalibration | None = None
        self._raw: deque[BodyFrame] = deque(maxlen=RAW_FRAMES)
        self._raw_lock = threading.Lock()
        self._stats = StreamStats()
        self._stats_lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stop = threading.Event()
        self._device: web.WebSocketResponse | None = None
        self._sticks_were_down = False
        self.on_frame: Callable[[float, BodyFrame, RetargetResult], None] | None = None
        """每重定向完一帧就回调一次，参数是（对齐后的时刻, 原始骨架, 重定向结果）。

        跑在**收帧线程**里，所以里面不能阻塞。存在的意义是让发布跟随头显的
        72/90 Hz：换成定时器取最新样本会同时丢帧和引入拖油拖尾的拖延。"""

    ##
    # 生命周期
    ##

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name='mocap', daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None:
            # 先给协程自己收尾的机会：直接 loop.stop() 会让还在 await 的协程收到
            # GeneratorExit，打一串没意义的告警。
            thread.join(timeout=1.0)
            if thread.is_alive():
                loop = self._loop
                if loop is not None:
                    loop.call_soon_threadsafe(loop.stop)
                thread.join(timeout=2.0)
        self._thread = None

    ##
    # 给控制环用
    ##

    @property
    def calibrated(self) -> bool:
        return self._calibration is not None

    @property
    def calibration(self) -> RetargetCalibration | None:
        """当前的人机标定，没标过时为 None。"""
        return self._calibration

    def stats(self) -> StreamStats:
        with self._stats_lock:
            return StreamStats(**vars(self._stats))

    def recent_frames(self) -> list[BodyFrame]:
        """最近几秒的**原始**骨架（未重定向、未缩放）。给校准和诊断用。"""
        with self._raw_lock:
            return list(self._raw)

    def calibrate(self, *, min_frames: int = 20) -> RetargetCalibration:
        """用最近这批原始骨架标出人机比例。调用者要保证人当时**站直**。"""
        frames = self.recent_frames()
        if len(frames) < min_frames:
            raise RuntimeError(f'只攒到 {len(frames)} 帧动捕，不足 {min_frames} 帧，无法校准')
        calibration = self._retarget.calibrate(frames)
        self._calibration = calibration
        self._log(f'动捕校准完成: 缩放 {calibration.scale:.3f}, '
                  f'站立高度 {calibration.stand_height:.3f} m, '
                  f'站姿零位最大偏置 {np.abs(calibration.joint_bias).max():.3f} rad')
        return calibration

    ##
    # 收帧线程
    ##

    def _run(self) -> None:
        loop = asyncio.new_event_loop()
        self._loop = loop
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._serve_device())
        except RuntimeError:
            pass  # stop() 从别的线程把 loop 停了
        finally:
            loop.close()
            self._loop = None

    def _note(self, **fields) -> None:
        with self._stats_lock:
            for key, value in fields.items():
                setattr(self._stats, key, value)

    def _check_calibration_shortcut(self, payload: dict) -> None:
        """双摇杆同时按下 -> 原地校准。**边沿触发**，按住不会反复标。

        校准会清空缓冲、换掉坐标尺度，所以下游在跟动作的时候误按，那边会因为
        缓冲空了而断流急停——安全，但会打断操作。这里不做跨节点的状态判断：
        本包不知道下游在干什么，也不应该知道。
        """
        down = both_thumbsticks_pressed(payload)
        pressed, self._sticks_were_down = down and not self._sticks_were_down, down
        if not pressed:
            return
        try:
            self.calibrate()
        except (RuntimeError, ValueError) as exc:
            self._log(f'手柄触发的校准失败: {exc}')
            self.haptic('left', 0.3, 60)
            return
        # 戴着头显看不到日志，成了就双手各震一下当回执。
        self.haptic('left', 0.8, 120)
        self.haptic('right', 0.8, 120)

    def haptic(self, hand: str, intensity: float, duration_ms: float) -> None:
        """让手柄震一下。走的是头显上行用的同一条 ws，不需要额外通道。"""
        ws, loop = self._device, self._loop
        if ws is None or loop is None or ws.closed:
            return
        message = json.dumps({'type': 'haptic', 'hand': hand,
                              'intensity': float(intensity),
                              'duration': float(duration_ms)}, separators=(',', ':'))
        try:
            asyncio.run_coroutine_threadsafe(ws.send_str(message), loop)
        except RuntimeError:
            pass  # loop 正在关

    def _ingest(self, payload: dict) -> None:
        self._check_calibration_shortcut(payload)
        frame = parse_body(payload)
        if frame is None or not frame.usable:
            with self._stats_lock:
                self._stats.dropped += 1
                if frame is not None:
                    self._stats.status, self._stats.message = frame.status, frame.message
            return
        with self._stats_lock:
            self._stats.frames += 1
            self._stats.status, self._stats.message = frame.status, frame.message

        with self._raw_lock:
            self._raw.append(frame)

        calibration = self._calibration
        if calibration is None:
            return
        stamped = self._clock.stamp(frame.t, time.monotonic())
        try:
            result = self._retarget.solve(frame, calibration)
        except ValueError as exc:
            # 骨架里出现零长肢体（tracker 短暂丢失）时会走到这，丢掉这一帧就好。
            with self._stats_lock:
                self._stats.dropped += 1
                self._stats.last_error = str(exc)
            return
        if stamped <= self._last_stamp:
            return
        self._last_stamp = stamped
        if self.on_frame is not None:
            try:
                self.on_frame(stamped, frame, result)
            except Exception as exc:  # noqa: BLE001
                # 下游的发布出问题不能把收帧线程带死，否则整条链路一起停。
                with self._stats_lock:
                    self._stats.last_error = f'on_frame: {type(exc).__name__}: {exc}'

    async def _serve_device(self) -> None:
        app = web.Application()
        app.add_routes([web.get('/ws/device', self._on_device),
                        web.get('/health', self._on_health)])
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, self._host, self._port)
        await site.start()
        self._log(f'动捕上行服务已就绪: ws://{self._host}:{self._port}/ws/device '
                  f'（头显配置面板里填这个地址）')
        try:
            while not self._stop.is_set():
                await asyncio.sleep(0.2)
        finally:
            await runner.cleanup()

    def _authorized(self, request: web.Request) -> bool:
        if not self._token:
            return True
        got = request.query.get('token') or request.headers.get('X-Auth-Token', '')
        return hmac.compare_digest(got, self._token)

    async def _on_health(self, request: web.Request) -> web.StreamResponse:
        stats = self.stats()
        return web.json_response({'frames': stats.frames, 'dropped': stats.dropped,
                                  'status': stats.status, 'message': stats.message,
                                  'connected': stats.connected,
                                  'calibrated': self.calibrated})

    async def _on_device(self, request: web.Request) -> web.StreamResponse:
        if not self._authorized(request):
            raise web.HTTPUnauthorized(text='invalid token')
        ws = web.WebSocketResponse(max_msg_size=MAX_MESSAGE_BYTES, heartbeat=20)
        await ws.prepare(request)
        self._device = ws
        self._sticks_were_down = False
        self._note(connected=True)
        self._log(f'头显已连接: {request.remote}')
        try:
            async for message in ws:
                if message.type is aiohttp.WSMsgType.TEXT:
                    try:
                        self._ingest(json.loads(message.data))
                    except (json.JSONDecodeError, TypeError):
                        continue
                elif message.type is aiohttp.WSMsgType.ERROR:
                    break
        finally:
            self._device = None
            self._note(connected=False)
            self._log('头显断开')
        return ws
