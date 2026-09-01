"""每次采集自带的相机参数快照（`camera_params.yaml`）。

它是导出时相机内外参的**唯一**来源。2026-08-31 头部相机被碰偏 13.7°，同一份
`calibration.yaml` 没法同时解释 8/28 和 8/31 两批采集 —— 这几条钉住那次的教训。
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import yaml

from record import camera_params
from record.session import Session, read_events

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
STREAMS = {'joint_states': True, 'head': True}


def _reader():
    spec = importlib.util.spec_from_file_location('session_reader',
                                                  TOOLS / 'session_reader.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _probed(root: Path, wrist=(1920, 1080), head=(1280, 720)) -> Path:
    """补上开录后才写的那两份探测结果（腕部 ffprobe、头部实收流）。"""
    (root / 'video').mkdir(parents=True, exist_ok=True)
    (root / 'video' / 'nominal.json').write_text(json.dumps({
        name: {'ok': True, 'width': wrist[0], 'height': wrist[1], 'fps': 30.0}
        for name in ('wrist_left', 'wrist_right')}), encoding='utf-8')
    meta = json.loads((root / 'meta.json').read_text(encoding='utf-8')) \
        if (root / 'meta.json').is_file() else {}
    meta['head_stream'] = {'width': head[0], 'height': head[1]}
    (root / 'meta.json').write_text(json.dumps(meta), encoding='utf-8')
    return root


def test_filename_matches_what_the_reader_looks_for():
    """采集侧写的和导出侧读的必须是同一个名字。

    `tools/` 要能整个拷到没有 ROS 的导出机，不能 import `record.*`，于是两边
    各有一份常量 —— 改一边忘了另一边，表现是「导出说没有相机参数」，而文件就在那。
    """
    assert camera_params.FILENAME == _reader().CAMERA_PARAMS_FILE


def test_keeps_only_the_profile_this_session_actually_recorded(tmp_path, calibration):
    """腕相机标定表里有 1080p 和 640x360 两档，快照只留录制用的那一档。

    `collect_intrinsics` 拿不到录制分辨率时会**取表里第一档**。现在恰好排对了
    纯属运气，档位顺序一变就静默拿 640x360 的 K 去导 1080p 的视频。
    """
    root = _probed(tmp_path / '20260101_000000')
    snapshot = camera_params.build(root, calibration)
    left = snapshot['intrinsics']['camera_left']
    assert len(left) == 1 and (left[0]['width'], left[0]['height']) == (1920, 1080)
    # 头部没有标定表，内参走 session 自己的 meta.json（D435i 出厂值）
    assert 'head' not in snapshot['intrinsics']


def test_extrinsics_come_over_but_solver_leftovers_do_not(tmp_path, calibration):
    """FK 要的是 `urdf_overrides`；`extrinsics` 那段是求解中间量，不进快照。"""
    root = _probed(tmp_path / '20260101_000000')
    snapshot = camera_params.build(
        root, dict(calibration, extrinsics={'camera_left': {}}))
    assert set(snapshot['urdf_overrides']) == set(calibration['urdf_overrides'])
    assert 'extrinsics' not in snapshot


def test_snapshot_is_sealed_together_with_the_session(tmp_path, calibration):
    """快照在封口前写，于是它的 sha256 也进 DONE，事后被改会被 verify 抓到。"""
    session = Session.create(tmp_path, STREAMS, meta={'note': 'x'},
                             calibration=calibration)
    _probed(session.paths.root)
    session.finish({})

    reader = _reader().Session(session.paths.root)
    assert reader.verify() == []
    assert camera_params.FILENAME in json.loads(
        session.paths.done.read_text(encoding='utf-8'))['files']
    assert reader.camera_params()['urdf_overrides']['d435_joint']['rpy'] == \
        calibration['urdf_overrides']['d435_joint']['rpy']


def test_missing_calibration_is_recorded_not_swallowed(tmp_path):
    """拿不到标定就不写快照，但要在事件线上留痕 —— 导出时才发现就晚了。"""
    session = Session.create(tmp_path, STREAMS, meta={'note': 'x'})
    session.finish({})
    warned = [e for e in read_events(session.paths.events)
              if e['type'] == 'warning' and e['code'] == 'camera_params_missing']
    assert warned and not session.paths.camera_params.is_file()


def test_snapshot_round_trips_as_yaml(tmp_path, calibration):
    """导出机只保证有 PyYAML，快照必须是 safe_load 读得出来的纯数据。"""
    root = _probed(tmp_path / '20260101_000000')
    path = camera_params.write(root, calibration)
    assert yaml.safe_load(path.read_text(encoding='utf-8'))['version'] == \
        camera_params.SCHEMA_VERSION
