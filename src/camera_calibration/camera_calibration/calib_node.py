"""相机标定主节点：采集 + 求解 + dashboard。

节点只管拍照和算，机械臂由你自己遥操。页面 1 Hz 轮询，检测跑在独立线程里，
HTTP 线程只取缓存 —— 1080p 上 ChArUco 检测要上百毫秒，放进请求里会把页面卡住。
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from urllib.parse import parse_qs

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from camera_calibration import extrinsic, intrinsic, storage, transforms
from camera_calibration.board import Board, probe
from camera_calibration.sources import CameraSource, Frames, MotionGate
from camera_calibration.storage import Store, find_intrinsic
from camera_calibration.webui import Panel, make_handler

_DEFAULTS = {
    'dashboard_port': 8300,
    'data_root': '~/camera_calib_data',
    'board_config': '',
    'cameras_config': '',
    'calib_file': '',
    'base_frame': 'torso_link',
    'preview_width': 720,
    'jpeg_quality': 75,
    'detect_period_s': 0.5,
}


class CalibrationNode(Node):

    def __init__(self) -> None:
        super().__init__('camera_calibration')
        for name, default in _DEFAULTS.items():
            self.declare_parameter(name, default)
        share = Path(get_package_share_directory('camera_calibration')) / 'config'

        self.callbacks = ReentrantCallbackGroup()
        self.base_frame = self._setting('base_frame')
        self.preview_width = int(self._setting('preview_width'))
        self.jpeg_quality = int(self._setting('jpeg_quality'))

        self.board_config = _read_yaml(self._setting('board_config')
                                       or share / 'board.yaml')
        self.cameras_config = _read_yaml(self._setting('cameras_config')
                                         or share / 'cameras.yaml')
        self.board = Board.from_config(self.board_config)
        self.store = Store(self._setting('data_root'),
                           self._setting('calib_file') or share / 'calibration.yaml')

        self.sources = {name: CameraSource(self, name, config)
                        for name, config in self.cameras_config['cameras'].items()}
        self.reference = next((s for s in self.sources.values()
                               if s.role == 'reference'), None)
        if self.reference is None:
            raise ValueError('cameras.yaml 里必须有一个 role: reference 的相机')
        self.gate = MotionGate(self, self.cameras_config.get('motion_gate', {}))
        self.frames = Frames(self)

        self._live: dict[str, dict] = {}
        self._live_lock = threading.Lock()
        self._results: dict[str, dict] = {}
        self._stop = threading.Event()
        threading.Thread(target=self._detect_loop, daemon=True,
                         name='calib-detect').start()

        self.panel = Panel(self, make_handler(self, self._actions(), self._route),
                           int(self._setting('dashboard_port')), 'G1 相机标定')
        self.panel.start()
        self.get_logger().info(f'标定结果写到 {self.store.calib_file}')
        self.get_logger().info(f'采集数据放在 {self.store.root}')

    def _setting(self, name):
        return self.get_parameter(name).value

    @property
    def targets(self) -> dict:
        return {n: s for n, s in self.sources.items() if s.role == 'target'}

    def destroy_node(self) -> bool:
        self._stop.set()
        self.panel.stop()
        return super().destroy_node()

    # ---------- 后台检测 ----------

    def _detect_loop(self) -> None:
        period = float(self._setting('detect_period_s'))
        while not self._stop.wait(0.05):
            started = time.monotonic()
            for name, source in self.sources.items():
                frame = source.latest()
                if frame is None:
                    continue
                with self._live_lock:
                    cached = self._live.get(name)
                if cached is not None and cached['seq'] == frame[0]:
                    continue
                try:
                    detection = self.board.detect(frame[1])
                except Exception as error:              # 检测线程不能死
                    self.get_logger().warn(f'{name} 检测失败：{error}')
                    continue
                with self._live_lock:
                    self._live[name] = {'seq': frame[0], 'detection': detection,
                                        'image': frame[1]}
            # 1080p 上一轮检测就要几百毫秒，按实际耗时补齐周期，别把 CPU 吃满
            self._stop.wait(max(0.0, period - (time.monotonic() - started)))

    def _live_of(self, name: str):
        with self._live_lock:
            return self._live.get(name)

    def _intrinsic_for(self, source: CameraSource, size, calibration):
        """返回 (内参, 来源)。标定值优先，否则用相机自报的出厂内参。

        只有 RealSense 会发 camera_info，所以实际上只有头部能回退 —— 腕相机
        没标就是没有，不编一个出来。分辨率对不上的出厂值也不能用。
        """
        entry = find_intrinsic(calibration, source.name, *size)
        if entry is not None:
            return entry, 'calibrated'
        factory = source.factory_info
        if factory and (factory['width'], factory['height']) == tuple(size):
            return factory, 'factory'
        return None, 'none'

    # ---------- 状态 ----------

    def state(self) -> dict:
        calibration = self.store.read_calibration()
        cameras = []
        for name, source in self.sources.items():
            status = source.status()
            live = self._live_of(name)
            detection = live['detection'] if live else None
            size = (status.get('width'), status.get('height'))
            origin = self._intrinsic_for(source, size, calibration)[1] \
                if None not in size else 'none'
            status.update({
                'frame': source.frame,
                'parent_frame': source.parent_frame,
                'profiles': [{'width': p['width'], 'height': p['height']}
                             for p in source.profiles],
                'switchable': bool(source.config.get('switch', {}).get('kind')
                                   in ('camera_node', 'realsense')),
                'corners': detection.count if detection else 0,
                'markers': detection.marker_count if detection else 0,
                'coverage': round(detection.coverage(), 3) if detection else 0.0,
                'max_corners': self.board.corner_count,
                'shots': self._shot_counts(name, calibration),
                'intrinsic_source': origin,
            })
            cameras.append(status)
        return {
            'board': self.board.describe(),
            'base_frame': self.base_frame,
            'cameras': cameras,
            'motion': self.gate.state(),
            'tf': self._tf_state(),
            'poses': [_pose_summary(p) for p in self.store.list_extrinsic_poses()],
            'calib_file': str(self.store.calib_file),
            'results': self._results,
        }

    def _shot_counts(self, camera: str, calibration: dict) -> list[dict]:
        counts = []
        for profile in self.sources[camera].profiles:
            width, height = profile['width'], profile['height']
            entry = find_intrinsic(calibration, camera, width, height,
                                   allow_scale=False)
            shots = self.store.list_intrinsic_shots(camera, width, height)
            counts.append({
                'width': width, 'height': height,
                'shots': len(shots),
                # 给名字而不是只给张数：删过图之后编号有空档，前端按序号猜会 404
                'names': [s['name'] for s in shots],
                'rms': entry['rms'] if entry else None,
            })
        return counts

    def _tf_state(self) -> dict:
        needed = {}
        for source in self.sources.values():
            frame = (source.frame if source.role == 'reference'
                     else source.parent_frame)
            if frame:
                needed[source.name] = {
                    'frame': frame,
                    'ok': self.frames.available(self.base_frame, frame),
                }
        return {'base_frame': self.base_frame, 'links': needed,
                'ok': all(v['ok'] for v in needed.values())}

    # ---------- 内参 ----------

    def capture_intrinsic(self, body: dict) -> dict:
        names = body.get('cameras') or list(self.sources)
        saved = []
        for name in names:
            source = self._source(name)
            live = self._live_of(name)
            if live is None:
                raise RuntimeError(f'{source.label} 还没收到图')
            detection = live['detection']
            if detection.count < intrinsic.MIN_CORNERS:
                saved.append({'camera': name, 'ok': False,
                              'reason': f'只认出 {detection.count} 个角点，至少要 '
                                        f'{intrinsic.MIN_CORNERS} 个'})
                continue
            shot = self.store.save_intrinsic_shot(name, live['image'], detection)
            saved.append({'camera': name, 'ok': True, 'name': shot,
                          'corners': detection.count,
                          'coverage': round(detection.coverage(), 3)})
        return {'saved': saved}

    def solve_intrinsic(self, body: dict) -> dict:
        name = body['camera']
        width, height = _size(body)
        views = self.store.load_intrinsic_views(name, width, height)
        result = intrinsic.calibrate(self.board, views)
        if result.get('ok'):
            result['outliers'] = intrinsic.outliers(result)
            factory = self._source(name).factory_info
            if factory and (factory['width'], factory['height']) == (width, height):
                result['vs_factory'] = intrinsic.compare_matrices(result, factory)
        self._results[f'{name}:{storage.profile_key(width, height)}'] = result
        return result

    def _solved(self, camera: str, width: int, height: int) -> dict:
        result = self._results.get(f'{camera}:{storage.profile_key(width, height)}')
        if not result or not result.get('ok'):
            raise RuntimeError('还没有可用的结果，先点求解')
        return result

    def save_intrinsic(self, body: dict) -> dict:
        name = body['camera']
        result = self._solved(name, *_size(body))
        return {'path': self.store.put_intrinsic(name, result, {
            'board': self.board.describe(), 'source': 'camera_calibration',
        })}

    def delete_shot(self, body: dict) -> dict:
        self.store.delete_intrinsic_shot(body['camera'], *_size(body), body['name'])
        return {'deleted': body['name']}

    def drop_outliers(self, body: dict) -> dict:
        name = body['camera']
        width, height = _size(body)
        outliers = self._solved(name, width, height).get('outliers', [])
        for shot in outliers:
            self.store.delete_intrinsic_shot(name, width, height, shot)
        return {'dropped': outliers}

    def clear_shots(self, body: dict) -> dict:
        self.store.clear_intrinsic(body['camera'], *_size(body))
        return {'cleared': True}

    def probe_board(self, body: dict) -> dict:
        live = self._live_of(body['camera'])
        if live is None:
            raise RuntimeError('还没收到图')
        return {'candidates': probe(live['image'], self.board_config),
                'current': self.board.describe()}

    def relate_profiles(self, body: dict) -> dict:
        """判定两个档位是缩放还是裁剪，是缩放才允许以后互相换算"""
        name = body['camera']
        calibration = self.store.read_calibration()
        low = find_intrinsic(calibration, name, int(body['low'][0]),
                             int(body['low'][1]), allow_scale=False)
        high = find_intrinsic(calibration, name, int(body['high'][0]),
                              int(body['high'][1]), allow_scale=False)
        if not low or not high:
            raise RuntimeError('两个档位都要先标定并保存')
        relation = intrinsic.relate_profiles(low, high)
        if body.get('save') and relation['convertible']:
            relation = dict(relation, **{'from': [high['width'], high['height']],
                                         'to': [low['width'], low['height']]})
            self.store.put_profile_relation(name, relation)
        return relation

    # ---------- 外参 ----------

    def _pose_sample(self, source: CameraSource, detection, calibration):
        entry, origin = self._intrinsic_for(source, detection.size, calibration)
        if entry is None:
            raise RuntimeError(
                f'{source.label} 在 {detection.size[0]}x{detection.size[1]} 下还没有内参')
        pose = self.board.estimate_pose(detection, entry['camera_matrix'],
                                        entry['distortion_coefficients'])
        if pose is None:
            raise RuntimeError(f'{source.label} 解不出板的位姿')
        return pose, entry, origin

    def capture_pose(self, body: dict) -> dict:
        motion = self.gate.state()
        if not motion['ok'] and not body.get('force'):
            raise RuntimeError(motion['reason'])
        calibration = self.store.read_calibration()

        live = self._live_of(self.reference.name)
        if live is None:
            raise RuntimeError(f'{self.reference.label} 还没收到图')
        reference_pose, reference_entry, reference_origin = self._pose_sample(
            self.reference, live['detection'], calibration)
        images = {self.reference.name: live['image']}
        meta = {
            'base_frame': self.base_frame,
            'reference': {
                'camera': self.reference.name, 'frame': self.reference.frame,
                'corners': live['detection'].count,
                'width': reference_entry['width'],
                'height': reference_entry['height'],
                'intrinsic_source': reference_origin,
                'T_base_ref': self.frames.lookup(self.base_frame,
                                                 self.reference.frame).tolist(),
                'T_ref_board': reference_pose.tolist(),
            },
            'targets': {},
            'motion': motion,
        }

        skipped = []
        for name, source in self.targets.items():
            target_live = self._live_of(name)
            if target_live is None:
                skipped.append({'camera': name, 'reason': '没收到图'})
                continue
            try:
                pose, entry, origin = self._pose_sample(
                    source, target_live['detection'], calibration)
                link = self.frames.lookup(self.base_frame, source.parent_frame)
            except RuntimeError as error:
                skipped.append({'camera': name, 'reason': str(error)})
                continue
            images[name] = target_live['image']
            meta['targets'][name] = {
                'frame': source.frame, 'parent': source.parent_frame,
                'corners': target_live['detection'].count,
                'width': entry['width'], 'height': entry['height'],
                'intrinsic_source': origin,
                'T_base_link': link.tolist(), 'T_cam_board': pose.tolist(),
            }
        if not meta['targets']:
            detail = '；'.join(f"{s['camera']} {s['reason']}" for s in skipped)
            raise RuntimeError(f'没有一个腕相机能用：{detail or "都没收到图"}')
        name = self.store.save_extrinsic_pose(images, meta)
        return {'name': name, 'targets': list(meta['targets']), 'skipped': skipped}

    def solve_extrinsic(self, body: dict) -> dict:
        poses = self.store.list_extrinsic_poses()
        if not poses:
            raise RuntimeError('一个姿态都还没采')
        groups, results = {}, {}
        for name, source in self.targets.items():
            samples = []
            for pose in poses:
                target = pose.get('targets', {}).get(name)
                if not target:
                    continue
                samples.append({
                    'name': pose['name'],
                    'T_base_ref': np.asarray(pose['reference']['T_base_ref'], float),
                    'T_ref_board': np.asarray(pose['reference']['T_ref_board'], float),
                    'T_base_link': np.asarray(target['T_base_link'], float),
                    'T_cam_board': np.asarray(target['T_cam_board'], float),
                })
            if not samples:
                continue
            groups[name] = samples
            reference = extrinsic.solve_from_reference(samples)
            stability = extrinsic.board_stability(samples)
            if stability['fixed']:
                handeye = extrinsic.solve_handeye(samples)
            else:
                handeye = {'ok': False, 'reason': (
                    f"板在采集过程中被挪过（各组相差最大 "
                    f"{stability['trans_max_mm']:.0f} mm / {stability['angle_max_deg']:.1f}°）。"
                    f"AX=XB 的前提就是板在 base 下不动，动了它的解是错的。"
                    f"联合最小二乘和参考相机法不受影响")}
            entry = {
                'camera': name, 'label': source.label, 'samples': len(samples),
                'parent': source.parent_frame, 'child': source.frame,
                'board_stability': stability,
                'reference': reference, 'handeye': handeye,
                'joint': extrinsic.solve_joint(samples),
            }
            if reference.get('ok') and handeye.get('ok'):
                entry['cross_check'] = extrinsic.compare(reference['matrix'],
                                                         handeye['matrix'])
                entry['reference_bias'] = extrinsic.reference_bias(
                    samples, np.asarray(handeye['matrix'], float))
            results[name] = entry

        # ΔH 是共享未知量，分开解会得到两个不一样的值，一起解才只有一个答案
        combined = extrinsic.solve_all(groups)
        if combined.get('ok'):
            combined['urdf'] = self._reference_urdf(combined['reference_correction'])
            for name, item in combined['cameras'].items():
                source = self.sources[name]
                item.update({'label': source.label, 'parent': source.parent_frame,
                             'child': source.frame,
                             'urdf': self._target_urdf(source, item['transform'])})
        payload = {'cameras': results, 'combined': combined}
        self._results['extrinsic'] = payload
        return payload

    def _target_urdf(self, source, transform: dict):
        """把腕相机外参折算成一条要插进 URDF 的边。

        URDF 里只有支架的可视化 link，没有光心，所以这里是新建而不是改。
        求解用的参考系是 gripper_base，而边要挂在支架下，得换一下：
        T_mount<-cam = T_mount<-parent · X。（现在两者 origin 重合，但不能靠这个巧合。）
        """
        joint = source.config.get('mount_joint')
        parent = source.config.get('mount_parent')
        if not joint or not parent:
            return None
        try:
            mount_parent = self.frames.lookup(parent, source.parent_frame)
        except RuntimeError as error:
            return {'error': str(error)}
        fixed = mount_parent @ transforms.matrix_from_dict(transform)
        return {
            'joint': joint, 'parent': parent, 'child': source.frame,
            'create': bool(source.config.get('mount_create')),
            **_origin(fixed),
        }

    def _reference_urdf(self, correction: dict):
        """把头部修正量折算成 URDF 里那个安装关节的 origin。

        估出来的是 T_base<-optical 的修正，而 URDF 里能改的是 T_parent<-mount，
        中间那段（mount -> optical）是相机自己的几何，不动。所以要把修正量
        共轭到 mount 系：T_parent<-mount_new = T_parent<-mount · M · ΔH · M⁻¹。
        """
        mount = self.reference.config.get('mount_frame')
        joint = self.reference.config.get('mount_joint')
        parent = self.reference.config.get('mount_parent', self.base_frame)
        if not mount or not joint:
            return None
        try:
            parent_mount = self.frames.lookup(parent, mount)
            mount_optical = self.frames.lookup(mount, self.reference.frame)
        except RuntimeError as error:
            return {'error': str(error)}
        delta = transforms.matrix_from_dict(correction)
        fixed = (parent_mount @ mount_optical @ delta
                 @ transforms.invert(mount_optical))
        previous = _origin(parent_mount)
        return {
            'joint': joint, 'parent': parent, 'child': mount,
            **_origin(fixed),
            'previous_xyz': previous['xyz'], 'previous_rpy': previous['rpy'],
        }

    def save_reference(self, body: dict) -> dict:
        combined = (self._results.get('extrinsic') or {}).get('combined') or {}
        if not combined.get('ok'):
            raise RuntimeError('三相机联合解不可用，先点求解')
        if not combined.get('well_posed') and not body.get('force'):
            raise RuntimeError(
                f"姿态激励不够（条件数 {combined['condition']}），头部修正量不可信。"
                f"多采几组、把手腕转轴拉开再存")
        urdf = combined.get('urdf')
        if not urdf or urdf.get('error'):
            raise RuntimeError(f"算不出关节 origin：{(urdf or {}).get('error', '缺 mount_joint 配置')}")
        return {'path': self.store.put_urdf_override(urdf['joint'], {
            **{k: urdf[k] for k in
               ('parent', 'child', 'xyz', 'rpy', 'previous_xyz', 'previous_rpy')},
            'calibrated_frame': self.reference.frame,
            'correction': combined['reference_correction'],
            'residual_mm': combined['residual_mm'],
            'condition': combined['condition'],
            'samples': {n: c['samples'] for n, c in combined['cameras'].items()},
            'source': 'camera_calibration solve_all',
        }), 'joint': urdf['joint']}

    def save_extrinsic(self, body: dict) -> dict:
        name = body['camera']
        method = body.get('method', 'all')
        stored = self._results.get('extrinsic') or {}
        if method == 'all':
            combined = stored.get('combined') or {}
            solution = (combined.get('cameras') or {}).get(name)
            if not combined.get('ok') or solution is None:
                raise RuntimeError('三相机联合解不可用，先点求解')
            extra = {'reference_correction': combined['reference_correction'],
                     'residual_mm': combined['residual_mm'],
                     'well_posed': combined['well_posed']}
        else:
            entry = (stored.get('cameras') or {}).get(name)
            if not entry:
                raise RuntimeError('先点求解')
            solution = entry.get(method)
            if not solution or not solution.get('ok'):
                raise RuntimeError(f'{method} 这一路没有可用解')
            extra = {key: solution[key] for key in
                     ('residual_mm', 'reference_correction') if key in solution}
            if 'cross_check' in entry:
                extra['cross_check'] = entry['cross_check']

        source = self._source(name)
        payload = {
            'parent': source.parent_frame, 'child': source.frame,
            'method': method, 'samples': solution.get('samples'),
            'consistency': solution['consistency'],
            **solution['transform'], **extra,
        }
        saved = {'path': self.store.put_extrinsic(name, payload)}

        # 同步写一份 URDF 边：跟头部一样由 robot_state_publisher 发，
        # calib_tf_node 只是不跑控制栈时的备胎。
        urdf = self._target_urdf(source, solution['transform'])
        if urdf and not urdf.get('error'):
            self.store.put_urdf_override(urdf['joint'], {
                **{k: urdf[k] for k in
                   ('parent', 'child', 'create', 'xyz', 'rpy')},
                'calibrated_from': source.parent_frame,
                'method': method, 'samples': solution.get('samples'),
                'source': 'camera_calibration solve_extrinsic',
            })
            saved['joint'] = urdf['joint']
        elif urdf:
            saved['urdf_warning'] = urdf['error']
        return saved

    def delete_pose(self, body: dict) -> dict:
        self.store.delete_extrinsic_pose(body['name'])
        return {'deleted': body['name']}

    def clear_poses(self, body: dict) -> dict:
        self.store.clear_extrinsic()
        self._results.pop('extrinsic', None)
        return {'cleared': True}

    # ---------- 档位 ----------

    def apply_profile(self, body: dict) -> dict:
        return self._source(body['camera']).apply_profile(*_size(body))

    def _source(self, name: str) -> CameraSource:
        source = self.sources.get(name)
        if source is None:
            raise ValueError(f'没有叫 {name} 的相机')
        return source

    # ---------- HTTP ----------

    def _actions(self) -> dict:
        return {
            '/api/profile': self.apply_profile,
            '/api/capture': self.capture_intrinsic,
            '/api/delete_shot': self.delete_shot,
            '/api/clear_shots': self.clear_shots,
            '/api/solve_intrinsic': self.solve_intrinsic,
            '/api/save_intrinsic': self.save_intrinsic,
            '/api/drop_outliers': self.drop_outliers,
            '/api/probe_board': self.probe_board,
            '/api/relate': self.relate_profiles,
            '/api/capture_pose': self.capture_pose,
            '/api/delete_pose': self.delete_pose,
            '/api/clear_poses': self.clear_poses,
            '/api/solve_extrinsic': self.solve_extrinsic,
            '/api/save_extrinsic': self.save_extrinsic,
            '/api/save_reference': self.save_reference,
        }

    def _route(self, handler, url) -> None:
        path = url.path
        query = parse_qs(url.query)
        if path in ('/', '/index.html'):
            return handler.send_static('index.html')
        if path in ('/app.js', '/app.css', '/common.js'):
            return handler.send_static(path.lstrip('/'))
        if path == '/api/state':
            return handler.send_json(self.state())
        if path == '/api/preview':
            return self._send_preview(handler, query)
        if path == '/api/shot':
            return self._send_image(handler, self.store.read_intrinsic_image(
                _one(query, 'camera'), int(_one(query, 'width')),
                int(_one(query, 'height')), _one(query, 'name')))
        if path == '/api/pose_image':
            return self._send_image(handler, self.store.read_extrinsic_image(
                _one(query, 'name'), _one(query, 'camera')))
        return handler.send_json({'error': 'not found'}, 404)

    def _send_image(self, handler, image) -> None:
        if image is None:
            return handler.send_json({'error': 'not found'}, 404)
        return handler.send_bytes(200, self._jpeg(image), 'image/jpeg')

    def _send_preview(self, handler, query) -> None:
        live = self._live_of(_one(query, 'camera'))
        if live is None:
            return handler.send_json({'error': '还没收到图'}, 503)
        image = live['image']
        if _one(query, 'overlay', '1') != '0':
            image = self.board.overlay(image, live['detection'])
        return handler.send_bytes(200, self._jpeg(image), 'image/jpeg')

    def _jpeg(self, image) -> bytes:
        if image.shape[1] > self.preview_width:
            scale = self.preview_width / image.shape[1]
            image = cv2.resize(image, None, fx=scale, fy=scale,
                               interpolation=cv2.INTER_AREA)
        ok, buffer = cv2.imencode('.jpg', image,
                                  [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            raise RuntimeError('JPEG 编码失败')
        return buffer.tobytes()


def _pose_summary(pose: dict) -> dict:
    reference = pose.get('reference', {})
    return {
        'name': pose['name'], 'stamp': pose.get('stamp', ''),
        'reference_corners': reference.get('corners', 0),
        'reference_intrinsic': reference.get('intrinsic_source', ''),
        'targets': {name: target.get('corners', 0)
                    for name, target in pose.get('targets', {}).items()},
    }


def _read_yaml(path) -> dict:
    text = Path(path).expanduser().read_text(encoding='utf-8')
    return yaml.safe_load(text) or {}


def _size(body: dict) -> tuple[int, int]:
    return int(body['width']), int(body['height'])


def _origin(matrix) -> dict:
    """URDF <origin> 的 xyz / rpy"""
    return {'xyz': [round(float(v), 7) for v in matrix[:3, 3]],
            'rpy': [round(float(v), 7) for v in transforms.matrix_to_rpy(matrix)]}


def _one(query: dict, key: str, default=None) -> str:
    values = query.get(key)
    if not values:
        if default is None:
            raise ValueError(f'缺少查询参数 {key}')
        return default
    return values[0]


def main() -> None:
    rclpy.init()
    node = CalibrationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
