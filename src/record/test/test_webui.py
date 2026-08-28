"""面板的文件下发：Range 续传、目录打包、路径穿越。

这几条都是「错了不会当场炸、只会安静地给错东西」的类型：Range 算错就是下下来的文件
少一截还看不出来，穿越挡不住就是把整台机器的文件都开放出去。
"""

import io
import sys
import zipfile
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from record.webui import _Chunked, make_handler, tree_entries  # noqa: E402


class _Node:
    def get_logger(self):
        class _Log:
            def error(self, *a, **k):
                pass
        return _Log()


class _Fake:
    """只实现 send_file / send_zip 用到的那几件事。"""

    def __init__(self, headers=None) -> None:
        self.headers = headers or {}
        self.wfile = io.BytesIO()
        self.status = None
        self.sent = {}

    def send_response(self, code):
        self.status = code

    def send_header(self, key, value):
        self.sent[key] = value

    def end_headers(self):
        pass


@pytest.fixture
def handler_class():
    return make_handler(_Node(), {}, {})


@pytest.fixture
def blob(tmp_path):
    path = tmp_path / 'video.mp4'
    path.write_bytes(bytes(range(256)) * 8)      # 2048 字节，内容可逐字节核对
    return path


def _run(handler_class, method, fake, *args):
    getattr(handler_class, method)(fake, *args)
    return fake


def test_whole_file(handler_class, blob):
    fake = _run(handler_class, 'send_file', _Fake(), blob, 'video.mp4')
    assert fake.status == 200
    assert fake.wfile.getvalue() == blob.read_bytes()
    assert fake.sent['Content-Length'] == '2048'
    assert fake.sent['Accept-Ranges'] == 'bytes'
    assert 'video.mp4' in fake.sent['Content-Disposition']


def test_range_resumes_from_offset(handler_class, blob):
    """断点续传：从 1000 接着下，拿到的必须正好是尾巴那一段。"""
    fake = _run(handler_class, 'send_file',
                _Fake({'Range': 'bytes=1000-'}), blob, 'video.mp4')
    assert fake.status == 206
    assert fake.sent['Content-Range'] == 'bytes 1000-2047/2048'
    assert fake.sent['Content-Length'] == '1048'
    assert fake.wfile.getvalue() == blob.read_bytes()[1000:]


def test_range_closed_interval(handler_class, blob):
    fake = _run(handler_class, 'send_file',
                _Fake({'Range': 'bytes=10-19'}), blob, '')
    assert fake.status == 206 and fake.wfile.getvalue() == blob.read_bytes()[10:20]


def test_range_suffix(handler_class, blob):
    """`bytes=-100` 是「最后 100 字节」，不是「从 0 到 100」。"""
    fake = _run(handler_class, 'send_file',
                _Fake({'Range': 'bytes=-100'}), blob, '')
    assert fake.wfile.getvalue() == blob.read_bytes()[-100:]


def test_range_past_end_is_416(handler_class, blob):
    fake = _run(handler_class, 'send_file',
                _Fake({'Range': 'bytes=9999-'}), blob, '')
    assert fake.status == 416 and fake.sent['Content-Range'] == 'bytes */2048'


def test_zip_is_stored_not_deflated(handler_class, tmp_path):
    """打包只为「一次拿走整个目录」，压缩没用（mkv 压缩率 100%）还费 CPU。"""
    root = tmp_path / 'out'
    (root / 'video_head').mkdir(parents=True)
    (root / 'meta.json').write_bytes(b'{"a": 1}')
    (root / 'video_head' / 'x.mp4').write_bytes(b'\x00' * 4096)

    fake = _run(handler_class, 'send_zip', _Fake(), 'out.zip', tree_entries(root))
    assert fake.sent['Transfer-Encoding'] == 'chunked'
    body = _dechunk(fake.wfile.getvalue())
    with zipfile.ZipFile(io.BytesIO(body)) as zf:
        assert sorted(zf.namelist()) == ['meta.json', 'video_head/x.mp4']
        assert all(i.compress_type == zipfile.ZIP_STORED for i in zf.infolist())
        assert zf.read('video_head/x.mp4') == b'\x00' * 4096


def test_zip_can_mix_a_tree_with_loose_files(handler_class, tmp_path):
    """导出工具包就是这个形状：tools/ 整棵树 + 两个不在树里的配置文件。"""
    root = tmp_path / 'tools'
    (root / 'format' / 'YB').mkdir(parents=True)
    (root / 'convert.py').write_bytes(b'# convert')
    (root / 'format' / 'YB' / 'export.py').write_bytes(b'# export')
    loose = tmp_path / 'final.urdf'
    loose.write_bytes(b'<robot/>')

    entries = tree_entries(root, 'tools/') + [(loose.name, loose)]
    fake = _run(handler_class, 'send_zip', _Fake(), 'record-tools.zip', entries)
    with zipfile.ZipFile(io.BytesIO(_dechunk(fake.wfile.getvalue()))) as zf:
        assert sorted(zf.namelist()) == [
            'final.urdf', 'tools/convert.py', 'tools/format/YB/export.py']
        assert zf.read('tools/format/YB/export.py') == b'# export'


def _dechunk(raw: bytes) -> bytes:
    out = b''
    while raw:
        head, _, rest = raw.partition(b'\r\n')
        size = int(head, 16)
        if not size:
            break
        out += rest[:size]
        raw = rest[size + 2:]
    return out


def test_chunked_wrapper_terminates():
    sink = io.BytesIO()
    stream = _Chunked(sink)
    stream.write(b'abc')
    stream.close()
    assert sink.getvalue() == b'3\r\nabc\r\n0\r\n\r\n'
