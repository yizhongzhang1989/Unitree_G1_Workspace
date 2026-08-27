"""采集数据的落盘布局，和 calibration.yaml 的读写。

图片一律存 PNG。JPEG 的块效应会把角点推偏零点几个像素 —— 标定的全部精度就在
这零点几个像素上，省这点磁盘不值。
"""

from __future__ import annotations

import datetime
import json
import os
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml

from camera_calibration.board import Detection

_PNG = [cv2.IMWRITE_PNG_COMPRESSION, 1]
_SECTIONS = ('intrinsics', 'extrinsics', 'profile_relations', 'urdf_overrides')
VERSION = 1


def profile_key(width: int, height: int) -> str:
    return f'{int(width)}x{int(height)}'


def _stamp() -> str:
    return datetime.datetime.now().isoformat(timespec='seconds')


class Store:
    """一次标定会话的全部产物。root 下面自解释，可以直接拷走离线重跑。"""

    def __init__(self, root, calib_file) -> None:
        self.root = Path(root).expanduser()
        # symlink-install 下 share/ 里的 config 是指向 src 的符号链接，
        # 不 resolve 的话会写进 install 目录，源码里看不到，重新 build 就没了
        self.calib_file = Path(calib_file).expanduser().resolve()
        (self.root / 'intrinsic').mkdir(parents=True, exist_ok=True)
        (self.root / 'extrinsic').mkdir(parents=True, exist_ok=True)

    # ---------- 内参 ----------

    def intrinsic_dir(self, camera: str, width: int, height: int) -> Path:
        path = self.root / 'intrinsic' / camera / profile_key(width, height)
        path.mkdir(parents=True, exist_ok=True)
        return path

    def save_intrinsic_shot(self, camera: str, image, detection: Detection) -> str:
        width, height = detection.size
        folder = self.intrinsic_dir(camera, width, height)
        name = f'{_next_index(folder.glob("*.json")):04d}'
        cv2.imwrite(str(folder / f'{name}.png'), image, _PNG)
        (folder / f'{name}.json').write_text(json.dumps({
            'name': name, 'camera': camera, 'stamp': _stamp(),
            'detection': detection.to_json(),
        }), encoding='utf-8')
        return name

    def _read_shots(self, camera: str, width: int, height: int):
        for path in sorted(self.intrinsic_dir(camera, width, height).glob('*.json')):
            data = json.loads(path.read_text(encoding='utf-8'))
            yield data, Detection.from_json(data['detection'])

    def list_intrinsic_shots(self, camera: str, width: int, height: int) -> list[dict]:
        return [{'name': data['name'], 'stamp': data.get('stamp', ''),
                 'corners': detection.count, 'markers': detection.marker_count,
                 'coverage': round(detection.coverage(), 4)}
                for data, detection in self._read_shots(camera, width, height)]

    def load_intrinsic_views(self, camera: str, width: int, height: int) -> list[dict]:
        return [{'name': data['name'], 'detection': detection}
                for data, detection in self._read_shots(camera, width, height)]

    def read_intrinsic_image(self, camera: str, width: int, height: int, name: str):
        return _imread(self.intrinsic_dir(camera, width, height) / f'{_safe(name)}.png')

    def delete_intrinsic_shot(self, camera: str, width: int, height: int,
                              name: str) -> None:
        folder = self.intrinsic_dir(camera, width, height)
        for suffix in ('.png', '.json'):
            (folder / f'{_safe(name)}{suffix}').unlink(missing_ok=True)

    def clear_intrinsic(self, camera: str, width: int, height: int) -> None:
        shutil.rmtree(self.intrinsic_dir(camera, width, height), ignore_errors=True)

    # ---------- 外参 ----------

    def save_extrinsic_pose(self, images: dict, meta: dict) -> str:
        folder = self.root / 'extrinsic'
        folder.mkdir(parents=True, exist_ok=True)
        name = f'pose_{_next_index(folder.glob("pose_*")):03d}'
        target = folder / name
        target.mkdir()
        for camera, image in images.items():
            cv2.imwrite(str(target / f'{camera}.png'), image, _PNG)
        (target / 'meta.json').write_text(
            json.dumps({**meta, 'name': name, 'stamp': _stamp()},
                       ensure_ascii=False, default=_jsonable),
            encoding='utf-8')
        return name

    def list_extrinsic_poses(self) -> list[dict]:
        return [json.loads(path.read_text(encoding='utf-8'))
                for path in sorted((self.root / 'extrinsic').glob('pose_*/meta.json'))]

    def read_extrinsic_image(self, name: str, camera: str):
        return _imread(self.root / 'extrinsic' / _safe(name) / f'{_safe(camera)}.png')

    def delete_extrinsic_pose(self, name: str) -> None:
        shutil.rmtree(self.root / 'extrinsic' / _safe(name), ignore_errors=True)

    def clear_extrinsic(self) -> None:
        shutil.rmtree(self.root / 'extrinsic', ignore_errors=True)
        (self.root / 'extrinsic').mkdir(parents=True, exist_ok=True)

    # ---------- calibration.yaml ----------

    def read_calibration(self) -> dict:
        data = {}
        if self.calib_file.is_file():
            data = yaml.safe_load(self.calib_file.read_text(encoding='utf-8')) or {}
        data.setdefault('version', VERSION)
        for section in _SECTIONS:
            data.setdefault(section, {})
        return data

    def write_calibration(self, data: dict) -> str:
        self.calib_file.parent.mkdir(parents=True, exist_ok=True)
        text = yaml.safe_dump(data, allow_unicode=True, sort_keys=False, width=120)
        # 先写临时文件再 rename：写到一半挂了不会留下半截 yaml 让相机节点读崩
        temporary = self.calib_file.with_suffix('.yaml.tmp')
        temporary.write_text(text, encoding='utf-8')
        os.replace(temporary, self.calib_file)
        return str(self.calib_file)

    def put_intrinsic(self, camera: str, result: dict, extra: dict | None = None) -> str:
        data = self.read_calibration()
        entry = {
            'width': result['width'], 'height': result['height'],
            'camera_matrix': result['camera_matrix'],
            'distortion_model': result['distortion_model'],
            'distortion_coefficients': result['distortion_coefficients'],
            'rms': result['rms'], 'images': result['images'],
            'coverage': result['coverage'], 'fov_deg': result['fov_deg'],
            'stamp': _stamp(),
        }
        entry.update(extra or {})
        entries = [e for e in data['intrinsics'].get(camera, [])
                   if (e['width'], e['height']) != (entry['width'], entry['height'])]
        entries.append(entry)
        entries.sort(key=lambda e: (-e['width'], -e['height']))
        data['intrinsics'][camera] = entries
        return self.write_calibration(data)

    def put_profile_relation(self, camera: str, relation: dict) -> str:
        data = self.read_calibration()
        relations = [r for r in data['profile_relations'].get(camera, [])
                     if (r['from'], r['to']) != (relation['from'], relation['to'])]
        relations.append(relation)
        data['profile_relations'][camera] = relations
        return self.write_calibration(data)

    def put_extrinsic(self, camera: str, entry: dict) -> str:
        data = self.read_calibration()
        data['extrinsics'][camera] = {**entry, 'stamp': _stamp()}
        return self.write_calibration(data)

    def put_urdf_override(self, joint: str, entry: dict) -> str:
        """头部外参落到这里，**不进 extrinsics**。

        extrinsics 里的东西会被 calib_tf_node 发成 static TF；头部那个 frame 已经有
        URDF + realsense-ros 在发了，再发一份就是两个 publisher 抢同一个 child。
        它该走的是另一条路：控制栈 launch 展开 URDF 时把这个 origin 叠上去。
        """
        data = self.read_calibration()
        data['urdf_overrides'][joint] = {**entry, 'stamp': _stamp()}
        return self.write_calibration(data)


def find_intrinsic(data: dict, camera: str, width: int, height: int,
                   allow_scale: bool = True) -> dict | None:
    """按 (相机, 宽, 高) 精确取内参；只在实测确认为缩放关系的档位之间换算。

    别对任意档位按比例缩放：换档位有可能是传感器裁剪（FOV 变小、fx 不变），
    那种情况下缩放出来的 K 是错的，而且错得看不出来。
    """
    entries = data.get('intrinsics', {}).get(camera, [])
    for entry in entries:
        if entry['width'] == width and entry['height'] == height:
            return dict(entry)
    if not allow_scale:
        return None
    for relation in data.get('profile_relations', {}).get(camera, []):
        if relation.get('kind') != 'scale':
            continue
        source = tuple(relation['from'])
        if tuple(relation['to']) != (width, height):
            continue
        for entry in entries:
            if (entry['width'], entry['height']) != source:
                continue
            return _scale_entry(entry, width, height, source)
    return None


def _scale_entry(entry: dict, width: int, height: int, source) -> dict:
    ratio_x = width / source[0]
    ratio_y = height / source[1]
    matrix = np.asarray(entry['camera_matrix'], float).reshape(3, 3).copy()
    matrix[0, :] *= ratio_x
    matrix[1, :] *= ratio_y
    scaled = dict(entry)
    scaled.update({
        'width': width, 'height': height,
        'camera_matrix': [float(v) for v in matrix.reshape(9)],
        # 畸变系数定义在归一化坐标上，缩放和裁剪都不改它们
        'scaled_from': list(source),
    })
    return scaled


def _imread(path: Path):
    return cv2.imread(str(path)) if path.is_file() else None


def _safe(name: str) -> str:
    """名字必须就是一截文件名。

    不能只用 basename 削一下了事 —— ``../../etc`` 会变成 ``etc``，虽然穿不出去，
    但会默默去动另一个名字的目录，调用方还以为成功了。
    """
    name = str(name)
    if not name or name in ('.', '..') or os.path.basename(name) != name:
        raise ValueError(f'非法名字：{name}')
    return name


def _next_index(paths) -> int:
    """取已有编号的最大值 +1。

    用文件数 +1 会在删过中间某一张之后撞名，新拍的图直接盖掉旧的。
    """
    largest = 0
    for path in paths:
        digits = ''.join(c for c in path.stem if c.isdigit())
        if digits:
            largest = max(largest, int(digits))
    return largest + 1


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    raise TypeError(f'不能序列化 {type(value)}')
