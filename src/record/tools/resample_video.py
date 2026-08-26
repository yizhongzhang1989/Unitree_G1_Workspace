#!/usr/bin/env python3
"""把 session 里的 mkv 重采样成「一帧对一行」的 mp4。**只要 ffmpeg。**

导出的每条 episode 是 N 行 30 Hz 的表，视频必须也正好 N 帧、第 k 帧就是第 k 行
那一刻看到的画面 —— 对面给的示例就是这个契约（h5 140 行 / mp4 140 帧）。

源视频是多少帧率不重要：`frame_index[k]` 已经算好第 k 行该取源里的哪一帧
（腕相机走等间隔重建的时间戳，且已扣掉管线延迟），这里只负责把那些帧按顺序吐出来，
该重复的重复、该跳的跳。

**一路相机只解码一遍。** episode 在时间上有序且不重叠，把所有 episode 的取帧需求
接成一条按源帧号递增的清单，顺着解码流走一趟就够了。逐 episode 各解一次的话，
一小时的 session 配一百条 episode 就是一百小时的解码。
"""

from __future__ import annotations

import json
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

FPS = 30.0
CRF = 20
#: yuv420p 每像素 1.5 字节，管道带宽只有 rgb24 的一半，而且 h264 本来就编这个
PIX_FMT = 'yuv420p'


@dataclass
class Plan:
    """一条 episode 要出的视频：输出路径 + 每一行该取源里的第几帧（−1 = 没有帧）"""
    path: Path
    frames: list


def available() -> bool:
    return bool(shutil.which('ffmpeg') and shutil.which('ffprobe'))


def probe(path) -> tuple[int, int]:
    out = subprocess.run(
        ['ffprobe', '-v', 'error', '-select_streams', 'v:0',
         '-show_entries', 'stream=width,height', '-of', 'json', str(path)],
        capture_output=True, text=True, check=True).stdout
    stream = json.loads(out)['streams'][0]
    return int(stream['width']), int(stream['height'])


def output_size(source: tuple[int, int], height: int) -> tuple[int, int]:
    """按高度等比缩。宽度必须偶数，h264 的 yuv420p 要求色度平面能整除。"""
    if not height or height >= source[1]:
        return source
    width = round(source[0] * height / source[1] / 2) * 2
    return width, height


def resample(source, plans: list[Plan], height: int = 0,
             fps: float = FPS, crf: int = CRF, on_progress=None) -> list[dict]:
    """一趟解码，产出全部 episode 的视频。返回每条的统计。

    `plans` 必须按时间升序，且每条内部的帧号不递减 —— 解码流退不回去。

    ``on_progress(source_index)`` 报的是**源帧游标**，不是写出了多少帧。
    只导一小段的时候写出很快，但解码得一路啦到那一段，按写出帧数报会一下子跑满再卡住。
    """
    native = probe(source)
    size = output_size(native, height)
    scale = ['-vf', f'scale={size[0]}:{size[1]}'] if size != native else []
    decoder = subprocess.Popen(
        ['ffmpeg', '-v', 'error', '-i', str(source), *scale,
         '-f', 'rawvideo', '-pix_fmt', PIX_FMT, '-'],
        stdout=subprocess.PIPE)
    reader = _Reader(decoder.stdout, size[0] * size[1] * 3 // 2, on_progress)
    try:
        return [_write(plan, reader, size, fps, crf) for plan in plans]
    finally:
        reader.close()
        # 一般只取到最后一条 episode 就够了，源还没解完。这时候 kill 而不是关管道：
        # 关管道会让 ffmpeg 去写 trailer 再报 Broken pipe，把真错误淹在噪声里。
        if decoder.poll() is None:
            decoder.kill()
        decoder.stdout.close()
        decoder.wait()


def _write(plan: Plan, reader: '_Reader', size, fps: float, crf: int) -> dict:
    plan.path.parent.mkdir(parents=True, exist_ok=True)
    encoder = subprocess.Popen(
        ['ffmpeg', '-v', 'error', '-y',
         '-f', 'rawvideo', '-pix_fmt', PIX_FMT,
         '-s', f'{size[0]}x{size[1]}', '-r', f'{fps:g}', '-i', '-',
         '-c:v', 'libx264', '-preset', 'veryfast', '-crf', str(crf),
         '-pix_fmt', PIX_FMT, str(plan.path)],
        stdin=subprocess.PIPE)
    held = exhausted = 0
    try:
        for index in plan.frames:
            frame = reader.at(int(index)) if index >= 0 else None
            if frame is None:
                # 那一刻没有帧（相机掉线或源解完了）。重复上一帧而不是插黑帧：
                # 黑帧是训练时一眼看不出来的脏数据，重复至少还是真实画面。
                frame = reader.current
                held += 1
                exhausted += int(index >= 0)
            if frame is None:
                raise RuntimeError(f'{plan.path.name} 开头就没有可用的帧')
            encoder.stdin.write(frame)
    finally:
        encoder.stdin.close()
        encoder.wait()
    return {'file': plan.path.name, 'frames': len(plan.frames),
            'width': size[0], 'height': size[1],
            'held': held, 'past_end': exhausted}


class _Reader:
    """只能向前走的帧游标。源是一条解码流，退不回去。"""

    def __init__(self, stream, frame_bytes: int, on_progress=None) -> None:
        self._stream = stream
        self._size = frame_bytes
        self._index = -1
        self._on_progress = on_progress
        self.current = None

    @property
    def index(self) -> int:
        """已经解到源的第几帧。进度条看这个。"""
        return max(self._index, 0)

    def at(self, index: int):
        """前进到第 index 帧。源已解完时返回 None。"""
        if index < self._index:
            raise ValueError(f'帧号倒退：{index} < {self._index}，plan 没按时间排好')
        while self._index < index:
            data = self._stream.read(self._size)
            if len(data) < self._size:
                return None
            self._index += 1
            self.current = data
            # 报在这里而不是写帧那一层：只导末尾一小段时，整个解码开销都在
            # 第一次 at() 里跑完，挂在外层会先停半天再跳到头。
            if self._on_progress and self._index % 128 == 0:
                self._on_progress(self._index)
        return self.current

    def close(self) -> None:
        self.current = None
