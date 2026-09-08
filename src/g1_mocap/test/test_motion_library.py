"""Playback reads every motion without modifying the dataset."""

import json
import threading
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np
import pytest

from g1_mocap.motion_library import MotionLibrary


def write_motion(path, count=101):
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = np.zeros((count, 36))
    rows[:, 2] = 0.8
    rows[:, 6] = 1.0
    rows[:, 7] = np.linspace(0, 1, count)
    np.savetxt(path, rows, delimiter=',')
    return rows


def test_empty_directory_is_not_created(tmp_path):
    root = tmp_path / 'missing'
    assert MotionLibrary(root).list()['motions'] == []
    assert not root.exists()


def test_list_and_play_without_metadata(tmp_path):
    path = tmp_path / 'motions' / 'balance_001.csv'
    expected = write_motion(path)
    before = path.read_bytes()
    library = MotionLibrary(tmp_path)
    assert library.list()['motions'][0]['file_name'] == path.name
    result = library.load(path.name)
    assert result['fps'] == 50 and result['num_frames'] == 101
    assert len(result['joint_names']) == 29
    np.testing.assert_array_equal(result['frames'], expected)
    assert path.read_bytes() == before


@pytest.mark.parametrize('name', ['../secret.csv', '/tmp/a.csv', 'sub/a.csv', 'metadata.json', ''])
def test_invalid_paths_rejected(tmp_path, name):
    with pytest.raises(ValueError):
        MotionLibrary(tmp_path).load(name)


def test_external_symlink_excluded(tmp_path):
    write_motion(tmp_path / 'secret.csv')
    motions = tmp_path / 'dataset' / 'motions'
    motions.mkdir(parents=True)
    (motions / 'external.csv').symlink_to(tmp_path / 'secret.csv')
    library = MotionLibrary(tmp_path / 'dataset')
    assert library.list()['motions'] == []
    with pytest.raises(FileNotFoundError):
        library.load('external.csv')


@pytest.mark.parametrize('defect', ['columns', 'nan', 'quat', 'oversize'])
def test_bad_file_does_not_hide_valid_recordings(tmp_path, defect):
    path = tmp_path / 'motions' / 'bad.csv'
    rows = write_motion(path)
    write_motion(path.with_name('good.csv'))
    if defect == 'columns':
        rows = rows[:, :35]
    elif defect == 'nan':
        rows[0, 0] = np.nan
    elif defect == 'quat':
        rows[0, 6] = 0
    np.savetxt(path, rows, delimiter=',')
    library = MotionLibrary(tmp_path)
    if defect == 'oversize':
        library.MAX_BYTES = 10
    assert len(library.list()['motions']) == 2
    with pytest.raises(ValueError):
        library.load(path.name)


def test_http_motion_routes(tmp_path):
    from g1_mocap.dashboard_node import _Handler, _Server
    write_motion(tmp_path / 'motions' / 'good.csv')
    (tmp_path / 'motions' / 'bad.csv').write_text('1,2,3\n')
    server = _Server(('127.0.0.1', 0), _Handler)
    server.dashboard = SimpleNamespace(motions=MotionLibrary(tmp_path))
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    base = f'http://127.0.0.1:{server.server_port}'
    try:
        with urlopen(base + '/motions', timeout=5) as response:
            assert len(json.load(response)['motions']) == 2
        with urlopen(base + '/motion?name=good.csv', timeout=5) as response:
            assert json.load(response)['num_frames'] == 101
            assert response.headers['Cache-Control'] == 'no-store'
        for name, status in [('bad.csv', 400), ('missing.csv', 404), ('..%2Fsecret.csv', 400)]:
            with pytest.raises(HTTPError) as error:
                urlopen(base + '/motion?name=' + name, timeout=5)
            assert error.value.code == status
            assert 'error' in json.load(error.value)
            error.value.close()
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=5)


def test_http_archive_route(tmp_path):
    from g1_mocap.dashboard_node import _Handler, _Server
    path = tmp_path / 'motions' / 'good.csv'
    write_motion(path)
    server = _Server(('127.0.0.1', 0), _Handler)
    server.dashboard = SimpleNamespace(motions=MotionLibrary(tmp_path))
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    base = f'http://127.0.0.1:{server.server_port}'
    try:
        request = Request(base + '/motion/archive', method='POST',
                          data=json.dumps({'name': path.name}).encode(),
                          headers={'Content-Type': 'application/json'})
        with urlopen(request, timeout=5) as response:
            assert json.load(response)['file_name'] == path.name
        assert not path.exists()
        assert (tmp_path / 'motions' / '.trash' / path.name).exists()
        for payload, status in (({'name': '../secret.csv'}, 400),
                                ({'name': 1}, 400),
                                ({'name': 'missing.csv'}, 404)):
            request = Request(base + '/motion/archive', method='POST',
                              data=json.dumps(payload).encode(),
                              headers={'Content-Type': 'application/json'})
            with pytest.raises(HTTPError) as error:
                urlopen(request, timeout=5)
            assert error.value.code == status
            assert 'error' in json.load(error.value)
            error.value.close()
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=5)
