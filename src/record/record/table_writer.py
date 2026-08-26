"""定长 float64 表的追加写。

落盘格式刻意做成「无表头的裸 float64 矩阵」，列信息全部外置到 ``schema.json``：
采集机是 Orin NX，导出机是没有 ROS 的 Windows，两边唯一都有的只有 numpy，所以
读一张表必须能退化成一行 ``np.fromfile(p, np.float64).reshape(-1, ncol)``。
带表头的格式（parquet/hdf5）都要装库，而这台机器连 pyarrow 都没有。

崩溃安全：按整行成批写，进程被 kill 只会丢掉还在缓冲里的若干行，已落盘部分永远
是整行的整数倍，读端不需要特殊处理。
"""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

#: 行数达到它就落盘一次。1024 行 x 100 列 x 8B = 800 KB，写盘开销可忽略，
#: 而最高频的话题（KWR57 实测 743 Hz）也只积压 1.4 s。
DEFAULT_CHUNK_ROWS = 1024


class TableWriter:
    """一张表 = 一个 ``.bin`` 文件 + 一份列定义。

    **自带锁。** ``recorder_node`` 跑在 ``MultiThreadedExecutor`` 上，只要订阅落在
    ``ReentrantCallbackGroup`` 里，同一个回调就会在多个线程上同时跑，“每张表只由
    它自己的回调写”保证不了串行。不加锁时实测出过两种静默损坏：两个线程
    同时进 ``_write_chunk`` 会把同一段写两遍（实测一次 session 重复 99~515 行）；
    ``self._fh.write`` 与 ``self.rows_written += self._n`` 之间插进一行，那行会被
    ``self._n = 0`` 抹掉但仍计入 ``rows``。两者都不报错。
    """

    def __init__(self, path: str | os.PathLike, columns: Sequence[str],
                 chunk_rows: int = DEFAULT_CHUNK_ROWS,
                 flush_interval_s: float = 1.0,
                 units: Sequence[str] | None = None,
                 description: str = '') -> None:
        columns = [str(c) for c in columns]
        if not columns:
            raise ValueError('列名不能为空')
        if len(set(columns)) != len(columns):
            dup = sorted({c for c in columns if columns.count(c) > 1})
            raise ValueError(f'列名重复: {dup}')
        if units is not None and len(units) != len(columns):
            raise ValueError(f'units 长度 {len(units)} 与列数 {len(columns)} 不符')
        if chunk_rows < 1:
            raise ValueError('chunk_rows 至少为 1')

        self.path = Path(path)
        self.columns = columns
        self.units = list(units) if units is not None else None
        self.description = description
        self.ncol = len(columns)
        self.rows_written = 0
        self.rows_dropped = 0

        self._chunk = np.empty((chunk_rows, self.ncol), dtype=np.float64)
        self._n = 0
        self._lock = threading.Lock()
        self._closed = False
        self._flush_interval = float(flush_interval_s)
        self._last_flush = time.monotonic()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.path, 'wb')

    def append(self, values: Iterable[float]) -> None:
        """写一行。长度不符直接抛，因为列错位是静默的、事后无法发现。"""
        with self._lock:
            # 封口后还会有几条回调在路上。收下来只会进缓冲区、永远落不了盘，
            # 却会被 schema() 计进 rows，正是“声明比文件多 1 行”的来源。
            if self._closed:
                self.rows_dropped += 1
                return
            row = self._chunk[self._n]
            try:
                row[:] = values
            except (ValueError, TypeError) as exc:
                raise ValueError(
                    f'{self.path.name}: 期望 {self.ncol} 列，收到的值装不进去') from exc
            self._n += 1
            if self._n == len(self._chunk):
                self._write_chunk()
            elif time.monotonic() - self._last_flush >= self._flush_interval:
                self._write_chunk()

    def _write_chunk(self) -> None:
        """调用方必须已持有 ``self._lock``。"""
        if self._n:
            self._fh.write(self._chunk[:self._n].tobytes())
            self.rows_written += self._n
            self._n = 0
        self._fh.flush()
        self._last_flush = time.monotonic()

    def flush(self) -> None:
        with self._lock:
            self._write_chunk()

    @property
    def bytes_written(self) -> int:
        with self._lock:
            return (self.rows_written + self._n) * self.ncol * 8

    def schema(self) -> dict:
        with self._lock:
            entry = {
                'file': self.path.name,
                'dtype': 'float64',
                'ncol': self.ncol,
                'columns': list(self.columns),
                'rows': self.rows_written + self._n,
            }
        if self.units is not None:
            entry['units'] = list(self.units)
        if self.description:
            entry['description'] = self.description
        return entry

    def close(self) -> None:
        with self._lock:
            self._closed = True
            if self._fh.closed:
                return
            self._write_chunk()
            os.fsync(self._fh.fileno())
            self._fh.close()

    def __enter__(self) -> 'TableWriter':
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def read_table(path: str | os.PathLike, ncol: int) -> np.ndarray:
    """按列数读回一张表。尾部不足一行的残片直接丢（断电才会出现）。"""
    flat = np.fromfile(str(path), dtype=np.float64)
    rows = flat.size // ncol
    return flat[:rows * ncol].reshape(rows, ncol)
