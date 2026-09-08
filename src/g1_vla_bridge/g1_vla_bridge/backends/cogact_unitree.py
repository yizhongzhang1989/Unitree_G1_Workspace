"""Unitree G1 CogACT 推理服务的 ``POST /api/inference`` 封装。"""

from __future__ import annotations

import json
import math
from typing import Any, Mapping

import cv2
import numpy as np
import requests

from g1_vla_bridge.vla_backend import (
    SIDES,
    ActionChunk,
    FrameSpec,
    GripperSpec,
    ImageSpec,
    Observation,
    VlaBackend,
    VlaSpec,
)

IMAGE_PARTS = (('image_0', 'head.jpg'), ('image_1', 'hand_left.jpg'),
               ('image_2', 'hand_right.jpg'))
IMAGE_TYPES = ('IMAGE_HEAD', 'IMAGE_LEFT', 'IMAGE_RIGHT')
_TRANS = ('ROBOT_LEFT_TRANS', 'ROBOT_RIGHT_TRANS')
_ROT = ('ROBOT_LEFT_ROT_MAT', 'ROBOT_RIGHT_ROT_MAT')
_GRIP = ('ROBOT_LEFT_GRIPPER', 'ROBOT_RIGHT_GRIPPER')

SPEC = VlaSpec(
    name='cogact_unitree',
    frame=FrameSpec(
        # 训练数据就是 torso_link；pose_unified = pose_raw * Rz(+90 deg)。
        tool_rotation_rpy=(0.0, 0.0, math.pi / 2.0)),
    # height=0 表示不在客户端缩放，原图交给 CogACT server 处理。
    images=ImageSpec(slots=('head', 'left_wrist', 'right_wrist'), height=0),
    gripper=GripperSpec(model_open=0.0, model_closed=1.0,
                        robot_open_rad=2.76377472169236, robot_closed_rad=0.0),
    horizon=30,
    action_semantics='absolute')

PARAMETERS: dict[str, Any] = {
    'server_url': 'http://10.172.100.47:5500/api/inference',
    'request_timeout_s': 30.0,
    'proxy': '',
}


def encode_jpeg(bgr: np.ndarray, quality: int = 90) -> bytes:
    """保持原分辨率编码 BGR 图像。"""
    image = np.asarray(bgr)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f'需要 HxWx3 的 BGR 图，收到 {image.shape}')
    ok, buffer = cv2.imencode(
        '.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError('cv2.imencode 失败')
    return buffer.tobytes()


def normalized_intrinsic(calibration, image: np.ndarray) -> list[list[float]]:
    """按训练侧规则将 ``fx/fy/cx/cy`` 归一化到图像宽高。"""
    height, width = image.shape[:2]
    if tuple(calibration.size) != (width, height):
        raise ValueError(
            f'CameraInfo 分辨率 {calibration.size} 与图像 {(width, height)} 不一致')
    fx, fy, cx, cy = calibration.intrinsics
    values = np.asarray((fx, fy, cx, cy), dtype=np.float64)
    if not np.all(np.isfinite(values)) or fx <= 0.0 or fy <= 0.0:
        raise ValueError(f'相机内参无效: {values.tolist()}')
    return [[fx / width, 0.0, cx / width],
            [0.0, fy / height, cy / height],
            [0.0, 0.0, 1.0]]


def build_payload(observation: Observation, frame) -> dict[str, Any]:
    """构造 CogACT RayPE 请求；外参与训练数据同为 ``base_T_cam``。"""
    slots = SPEC.images.slots
    missing_calibration = [slot for slot in slots if slot not in observation.calibrations]
    missing_pose = [slot for slot in slots if slot not in observation.camera_poses]
    if missing_calibration or missing_pose:
        raise ValueError(
            f'三视角标定不完整: 缺内参 {missing_calibration}, 缺外参 {missing_pose}')

    state = {}
    for side in SIDES:
        trans, rotation = frame.to_model(observation.poses[side])
        prefix = side.upper()
        state[f'ROBOT_{prefix}_TRANS'] = trans.tolist()
        state[f'ROBOT_{prefix}_ROT_MAT'] = rotation.tolist()

    return {
        'task_description': str(observation.task).lower(),
        'state': state,
        'image_types': list(IMAGE_TYPES),
        'intrinsics_per_view': [
            normalized_intrinsic(observation.calibrations[slot], observation.images[slot])
            for slot in slots
        ],
        'extrinsics_per_view': [
            observation.camera_poses[slot].tolist() for slot in slots
        ],
    }


def parse_action(body: Mapping[str, Any]) -> dict[str, dict[str, np.ndarray]]:
    missing = [key for key in _TRANS + _ROT + _GRIP if key not in body]
    if missing:
        raise ValueError(f'返回缺字段: {missing}')
    action = {}
    horizons = set()
    for side, trans_key, rot_key, grip_key in (
            ('left', _TRANS[0], _ROT[0], _GRIP[0]),
            ('right', _TRANS[1], _ROT[1], _GRIP[1])):
        trans = np.asarray(body[trans_key], dtype=np.float64).reshape(-1, 3)
        rot = np.asarray(body[rot_key], dtype=np.float64).reshape(-1, 3, 3)
        grip = np.asarray(body[grip_key], dtype=np.float64).reshape(-1)
        horizons.update((len(trans), len(rot), len(grip)))
        action[side] = {'trans': trans, 'rot': rot, 'grip': grip}
    if len(horizons) != 1 or horizons == {0}:
        raise ValueError(f'各字段的 horizon 无效: {sorted(horizons)}')
    for side, fields in action.items():
        for name, value in fields.items():
            if not np.all(np.isfinite(value)):
                raise ValueError(f'{side}.{name} 含非有限值')
    return action


class CogACTUnitreeBackend(VlaBackend):

    def __init__(self, url: str, timeout: float = 30.0, proxy: str = '') -> None:
        super().__init__(SPEC)
        self.url = url
        self.timeout = float(timeout)
        self._frame = SPEC.frame.transform()
        self._session = requests.Session()
        if proxy:
            self._session.proxies = {'http': proxy, 'https': proxy}

    def infer(self, observation: Observation) -> ActionChunk:
        missing = [slot for slot in self.spec.images.slots if slot not in observation.images]
        if missing:
            raise ValueError(f'缺图像 {missing}')
        images = [encode_jpeg(observation.images[slot], self.spec.images.jpeg_quality)
                  for slot in self.spec.images.slots]
        self.dump(dict(zip(self.spec.images.slots, images)))
        payload = build_payload(observation, self._frame)
        files = [(part, (filename, data, 'image/jpeg'))
                 for (part, filename), data in zip(IMAGE_PARTS, images)]
        files.append(('json', ('data.json',
                               json.dumps(payload, ensure_ascii=False).encode('utf-8'),
                               'application/json')))
        response = self._session.post(self.url, files=files, timeout=self.timeout)
        response.raise_for_status()
        return self._to_chunk(parse_action(response.json()))

    def _to_chunk(self, action: Mapping[str, Mapping[str, np.ndarray]]) -> ActionChunk:
        poses, grippers = {}, {}
        for side in SIDES:
            trans, rot = action[side]['trans'], action[side]['rot']
            poses[side] = np.stack([
                self._frame.from_model(trans[index], rot[index])
                for index in range(len(trans))
            ])
            grippers[side] = self.spec.gripper.to_robot(action[side]['grip'])
        return ActionChunk(poses=poses, grippers=grippers)

    def close(self) -> None:
        self._session.close()


def create(params: Mapping[str, Any]) -> CogACTUnitreeBackend:
    url = str(params.get('server_url') or PARAMETERS['server_url'])
    if not url.startswith(('http://', 'https://')):
        raise ValueError(f'server_url 必须是 http(s) 地址，收到 {url!r}')
    return CogACTUnitreeBackend(
        url,
        timeout=float(params.get('request_timeout_s') or 30.0),
        proxy=str(params.get('proxy') or ''))
