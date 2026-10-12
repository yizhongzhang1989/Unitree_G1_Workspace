"""面板的文件下发：Range 续传、目录打包、路径穿越。

这几条都是「错了不会当场炸、只会安静地给错东西」的类型：Range 算错就是下下来的文件
少一截还看不出来，穿越挡不住就是把整台机器的文件都开放出去。
"""

import io
import re
import zipfile
from pathlib import Path
from unittest.mock import Mock

import pytest

from record.webui import _Chunked, make_handler, tree_entries


@pytest.mark.parametrize('body,args', [
    ({'session': 's1', 't0': '1', 't1': '3'}, ('s1', 1.0, 3.0, 1.0, '')),
    ({'session': 's1', 't0': 1, 't1': 3, 'speed': '0.5', 'label': 'r0e0'},
     ('s1', 1.0, 3.0, 0.5, 'r0e0')),
])
def test_data_panel_dispatches_replay_actions(monkeypatch, body, args):
    from record import data_dashboard
    rec = Mock()
    rec.start.return_value = {'phase': 'loading'}
    rec.stop.return_value = {'phase': 'idle'}
    handler = Mock()
    monkeypatch.setattr(data_dashboard, 'make_handler', handler)
    monkeypatch.setattr(data_dashboard, 'Panel', Mock())
    data_dashboard.panel(rec)
    actions = handler.call_args.args[1]
    assert actions['/api/replay/start'](body) == {'phase': 'loading'}
    assert actions['/api/replay/stop']({}) == {'phase': 'idle'}
    rec.start.assert_called_once_with(*args)
    rec.stop.assert_called_once_with()


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
    return make_handler(Mock(), {}, {})


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


@pytest.mark.parametrize('header,start,end', [
    ('bytes=1000-', 1000, 2048), ('bytes=10-19', 10, 20), ('bytes=-100', 1948, 2048)])
def test_range_interval(handler_class, blob, header, start, end):
    fake = _run(handler_class, 'send_file',
                _Fake({'Range': header}), blob, '')
    assert fake.status == 206
    assert fake.wfile.getvalue() == blob.read_bytes()[start:end]
    assert fake.sent['Content-Range'] == f'bytes {start}-{end - 1}/2048'
    assert fake.sent['Content-Length'] == str(end - start)


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


STATIC = Path(__file__).resolve().parents[1] / 'record/static'
PAGES = {
    'data': ('data.html', 'data.js', 'common.js'),
    'recorder': ('index.html', 'app.js', 'common.js'),
}
_BLOCK = re.compile(r'/\*.*?\*/', re.S)
_LINE = re.compile(r'(?<![:\w])//[^\n]*')
DEFINED = set(re.findall(r'\.([a-zA-Z][\w-]*)', _BLOCK.sub('', (STATIC / 'app.css').read_text())))
NOT_STYLING = {'ep', 'episodes', 'split', 'wide', 'panel', 'note', 'acts'}


def used_classes(names) -> set:
    classes = set()
    for name in names:
        text = _LINE.sub('', _BLOCK.sub('', (STATIC / name).read_text()))
        for attribute in re.findall(r'class(?:Name)?\s*=\s*[\'"`]([^\'"`]*)', text):
            classes |= {value for value in attribute.split() if value and '$' not in value}
        classes |= set(re.findall(r'classList\.(?:add|toggle|remove)\(\s*[\'"]([\w-]+)', text))
        for line in text.splitlines():
            if re.search(r'class(Name|List)?\s*[+=]', line):
                classes |= set(re.findall(r"['\"]\s+([a-zA-Z][\w-]*)\s*['\"]", line))
    return {value for value in classes if not value.isupper()}


@pytest.mark.parametrize('page', sorted(PAGES))
def test_every_class_has_a_rule(page):
    missing = sorted(used_classes(PAGES[page]) - DEFINED - NOT_STYLING)
    assert not missing, f'{page} 用了但 app.css 里没有：{missing}'


def test_class_extraction_covers_dynamic_styles():
    found = used_classes(('data.js',))
    assert {'card', 'twist', 'dhead', 'preview', 'mute', 'spacer'} <= found


@pytest.mark.parametrize('page', sorted(PAGES))
def test_ids_referenced_by_js_exist_in_html(page):
    names = PAGES[page]
    have = set(re.findall(r'id="([\w-]+)"', (STATIC / names[0]).read_text()))
    for name in names[1:]:
        want = set(re.findall(r"\$\(\s*'([\w-]+)'\s*\)", _LINE.sub('', (STATIC / name).read_text())))
        assert want <= have, f'{page} 缺元素：{sorted(want - have)}'
