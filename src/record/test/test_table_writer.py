"""``TableWriter`` 的落盘契约。

这些用例钉的是「Windows 上只有 numpy 也能读出来」这条底线，改实现时不许放宽。
"""

import os
import subprocess
import sys
import textwrap
import time

import numpy as np
import pytest

from record.table_writer import TableWriter, read_table


def test_roundtrip_matches_written_rows(tmp_path):
    rows = np.arange(60, dtype=np.float64).reshape(12, 5)
    with TableWriter(tmp_path / 't.bin', list('abcde'), chunk_rows=4) as w:
        for r in rows:
            w.append(r)
    assert np.array_equal(read_table(tmp_path / 't.bin', 5), rows)


def test_readable_with_bare_numpy(tmp_path):
    """导出机没有本包，只能 np.fromfile + reshape —— 这条断了整个格式就废了。"""
    rows = np.random.default_rng(0).normal(size=(37, 9))
    with TableWriter(tmp_path / 't.bin', [f'c{i}' for i in range(9)]) as w:
        for r in rows:
            w.append(r)
    back = np.fromfile(tmp_path / 't.bin', dtype=np.float64).reshape(-1, 9)
    assert np.array_equal(back, rows)


@pytest.mark.parametrize('chunk', [1, 2, 7, 1024])
def test_chunk_size_does_not_change_bytes(tmp_path, chunk):
    rows = np.arange(30, dtype=np.float64).reshape(10, 3)
    with TableWriter(tmp_path / f't{chunk}.bin', list('xyz'), chunk_rows=chunk) as w:
        for r in rows:
            w.append(r)
    assert (tmp_path / f't{chunk}.bin').stat().st_size == 10 * 3 * 8


def test_flush_makes_rows_visible_before_close(tmp_path):
    path = tmp_path / 't.bin'
    w = TableWriter(path, list('ab'), chunk_rows=1000)
    for i in range(5):
        w.append([i, -i])
    assert path.stat().st_size == 0        # 还在缓冲里
    w.flush()
    assert np.array_equal(read_table(path, 2), [[0, 0], [1, -1], [2, -2], [3, -3], [4, -4]])
    w.close()


def test_wrong_column_count_raises(tmp_path):
    with TableWriter(tmp_path / 't.bin', list('abc')) as w:
        with pytest.raises(ValueError, match='3 列'):
            w.append([1.0, 2.0])
        with pytest.raises(ValueError, match='3 列'):
            w.append([1.0, 2.0, 3.0, 4.0])


def test_duplicate_columns_rejected(tmp_path):
    with pytest.raises(ValueError, match='列名重复'):
        TableWriter(tmp_path / 't.bin', ['a', 'b', 'a'])


def test_empty_columns_rejected(tmp_path):
    with pytest.raises(ValueError, match='列名不能为空'):
        TableWriter(tmp_path / 't.bin', [])


def test_units_length_checked(tmp_path):
    with pytest.raises(ValueError, match='units 长度'):
        TableWriter(tmp_path / 't.bin', list('ab'), units=['m'])


def test_schema_reports_rows_and_columns(tmp_path):
    with TableWriter(tmp_path / 'j.bin', ['t', 'q0'], units=['s', 'rad'],
                     description='关节') as w:
        w.append([1.0, 2.0])
        s = w.schema()
    assert s['file'] == 'j.bin' and s['ncol'] == 2 and s['rows'] == 1
    assert s['dtype'] == 'float64' and s['units'] == ['s', 'rad']
    assert s['description'] == '关节'


def test_schema_counts_buffered_rows(tmp_path):
    with TableWriter(tmp_path / 't.bin', list('ab'), chunk_rows=1000) as w:
        for i in range(9):
            w.append([i, i])
        assert w.schema()['rows'] == 9        # 还没落盘也要算进去


def test_bytes_written_tracks_buffer(tmp_path):
    with TableWriter(tmp_path / 't.bin', list('abcd'), chunk_rows=1000) as w:
        for i in range(6):
            w.append([i] * 4)
        assert w.bytes_written == 6 * 4 * 8


def test_close_is_idempotent(tmp_path):
    w = TableWriter(tmp_path / 't.bin', ['a'])
    w.append([1.0])
    w.close()
    w.close()
    assert read_table(tmp_path / 't.bin', 1).shape == (1, 1)


def test_survives_sigkill(tmp_path):
    """录制中被 kill -9，已 flush 的部分必须仍是整行的整数倍、能完整读出。"""
    script = textwrap.dedent(f'''
        import sys, time
        sys.path.insert(0, {os.path.dirname(os.path.dirname(os.path.abspath(__file__)))!r})
        from record.table_writer import TableWriter
        w = TableWriter({str(tmp_path / "k.bin")!r}, ["a", "b", "c"], chunk_rows=8)
        i = 0
        while True:
            w.append([i, i * 2, i * 3])
            i += 1
            time.sleep(0.001)
    ''')
    proc = subprocess.Popen([sys.executable, '-c', script])
    time.sleep(1.5)
    proc.kill()
    proc.wait(timeout=10)

    data = read_table(tmp_path / 'k.bin', 3)
    assert len(data) > 0, '1.5 秒内一行都没落盘'
    assert (tmp_path / 'k.bin').stat().st_size % (3 * 8) == 0
    expect = np.arange(len(data), dtype=np.float64)
    assert np.array_equal(data[:, 0], expect)
    assert np.array_equal(data[:, 1], expect * 2)


def test_read_table_drops_partial_tail(tmp_path):
    """断电可能留下半行；读端截掉而不是抛异常，别让一条坏尾巴废掉整个 session。"""
    path = tmp_path / 't.bin'
    with TableWriter(path, list('abc')) as w:
        w.append([1, 2, 3])
        w.append([4, 5, 6])
    with open(path, 'ab') as f:
        f.write(b'\x00' * 8)               # 追加半行
    assert np.array_equal(read_table(path, 3), [[1, 2, 3], [4, 5, 6]])
