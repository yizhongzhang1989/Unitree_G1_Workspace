"""智元 A2D Omnipicker 推理服务的请求封装（``POST /api/inference``）。

**线协议**（换 VLA 就是换这一段）：``multipart/form-data``，

* ``image_0`` / ``image_1`` / ``image_2`` = 头部 / 左腕 / 右腕 JPEG，**高度必须 240 px**；
* ``json`` part 装任务指令、双臂 state 和头部相机外参 ``head_camera_in_world``；
* 返回 ``(N, ...)`` 的**绝对**末端位姿序列，语义 ``abs_robot_base``，实测 N=30、约 288 ms。

训练机是智元 A2D 不是 G1，坐标系/夹爪/图像的差异全部声明在下面的 ``SPEC`` 里。
身体高度和俯仰不在那个变换里，训练时是靠 ``head_camera_in_world`` 编码的。

不依赖 ROS，只用 requests / OpenCV / numpy，可以脱离节点单独跑。
"""

from __future__ import annotations

import json
import math
from dataclasses import replace
from typing import Any, Mapping, Sequence

import cv2
import numpy as np
import requests

from g1_vla_bridge.reproject import Reprojector, field_of_view, scale_intrinsics
from g1_vla_bridge.vla_backend import (
    SIDES,
    ActionChunk,
    CameraCalibration,
    FrameSpec,
    GripperSpec,
    ImageSpec,
    Observation,
    VlaBackend,
    VlaSpec,
    resolve_sequence,
)

#: 图像 part 名与文件名，顺序与 ``SPEC.images.slots`` 一一对应。
IMAGE_PARTS = (('image_0', 'head.jpg'), ('image_1', 'hand_left.jpg'),
               ('image_2', 'hand_right.jpg'))

# 服务实际用到的输出字段。ROT_6D 是 ROT_MAT 的等价表示，这里不用。
_TRANS = ('ROBOT_LEFT_TRANS', 'ROBOT_RIGHT_TRANS')
_ROT = ('ROBOT_LEFT_ROT_MAT', 'ROBOT_RIGHT_ROT_MAT')
_GRIP = ('ROBOT_LEFT_GRIPPER', 'ROBOT_RIGHT_GRIPPER')

#: 训练机头部相机，``head_reproject`` 拿它对齐镜头尺度（为什么要对齐见 reproject.py）。
#: 畔变按 OpenCV/ROS 的 plumb_bob 顺序 ``[k1,k2,p1,p2,k3]``——厂商 JSON 写的是 k1 k2 k3 p1 p2。
TRAIN_HEAD_CAMERA = CameraCalibration(
    intrinsics=(643.9313354492188, 643.236083984375,
                646.4886474609375, 359.7933654785156),
    size=(1280, 720),
    distortion=(-0.05372680723667145, 0.061037734150886536,
                0.0002683195343706757, 0.0005757578765042126,
                -0.018827488645911217))

#: A2D 头部相机相对 ``joint_head_pitch``（``link-pitch_head``）的外参。约定是
#: ``T_head<-cam``（相机位姿表示在 head 链接里），不是标定文件常见的 ``T_cam<-head``——
#: 判据是「双手 3D 对称则投影应关于主点对称」，两者不对称度 11 px vs 232 px。
#: 只给 ``calibrate_frame`` 用。
HEAD_TO_CAM = np.array([
    [-0.02361286567247678, -0.0034098340718792704, -0.999715362293856,
     -0.08779365140104009],
    [-0.005592271719712527, -0.999978087231093, 0.003542817333293842,
     0.03862264157899304],
    [-0.999705536181403, 0.0056743360181112845, 0.02359327953055912,
     -0.018019441671689576],
    [0.0, 0.0, 0.0, 1.0]])
#: 训练用的原点在 A2D ``base_link`` 上方 0.3 m。
TRAIN_ORIGIN_Z = 0.3

SPEC = VlaSpec(
    name='a2d_omnipicker',
    frame=FrameSpec(
        # 原点不是几何推的，是**拿来把两台相机的位置摹到一起**的：这个 VLA 泛化差，
        # 相机位置差一点就不行。训练相机（lift=0.28 / body_pitch=30° / head_pitch=0）在
        # 训练系 `[0.520, -0.018, 1.027]`，我们的在 torso 下 `[0.057, 0.033, 0.431]`，
        # 相减就是这个值。重算：`ros2 run g1_vla_bridge calibrate_frame`。
        origin_in_base=(-0.4633, 0.0508, -0.5964),
        # 保持水平：只挪原点不掃坐标系，重力方向才是对的、末端 state 才不会歪。
        # 代价：两台相机俯角仍差 17.8°（他们 +29.8° / 我们 +47.6°），没有补。
        rotation_rpy=(0.0, 0.0, 0.0),
        # 我方 gripper_base -> 模型 joint7：Z 轴同向（差约 4 度），差一道绕 Z 的 180 度。
        # 不补这一道姿态会整个反过来。
        tool_rotation_rpy=(0.0, 0.0, math.pi),
        tool_offset=(0.0, 0.0, -0.0281)),
    images=ImageSpec(slots=('head', 'left_wrist', 'right_wrist'), height=240),
    # 模型 0=张开、1=夹紧；我们的偏心轴 0 rad=夹紧、2.76377 rad=张开（方向是反的）。
    # 行程上界抄自 unitree_g1_description/model/Gloria-M 与 motion_control.yaml。
    gripper=GripperSpec(model_open=0.0, model_closed=1.0,
                        robot_open_rad=2.76377472169236, robot_closed_rad=0.0),
    horizon=30,
    action_semantics='absolute')

#: 由节点统一 declare 的 ROS 参数。只放**现场会改**的：网络、开关、以及
#: ``calibrate_frame`` 算出来要粘贴的坐标系标定。规格里的常量不开放，换机器人改本文件。
PARAMETERS: dict[str, Any] = {
    'server_url': 'http://10.172.100.47:5509/api/inference',
    'request_timeout_s': 30.0,
    'proxy': '',
    'head_reproject': False,
    'model_origin_in_base': list(SPEC.frame.origin_in_base),
    'model_rotation_rpy': list(SPEC.frame.rotation_rpy),
    'tool_offset': list(SPEC.frame.tool_offset),
    'tool_rotation_rpy': list(SPEC.frame.tool_rotation_rpy),
}


def encode_jpeg(bgr: np.ndarray, height: int, quality: int = 90) -> bytes:
    """等比缩放到指定高度后编码成 JPEG。模型输入要求高度 240 px。"""
    image = np.asarray(bgr)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f'需要 HxWx3 的 BGR 图，收到 {image.shape}')
    if image.shape[0] != height:
        scale = height / float(image.shape[0])
        # 640x360 -> 240 是下采样，双线性会混叠；缩小一律 INTER_AREA。
        interp = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
        width = max(1, int(round(image.shape[1] * scale)))
        image = cv2.resize(image, (width, height), interpolation=interp)
    ok, buf = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError('cv2.imencode 失败')
    return buf.tobytes()


def build_payload(task_description: str, has_left: bool, has_right: bool,
                  left: Sequence, right: Sequence,
                  head_camera_in_world) -> dict[str, Any]:
    """拼 JSON part。``left``/``right`` = ``(trans(3,), rot(3,3), gripper)``，均为模型系。"""
    state = {}
    for side, (trans, rot, grip) in (('LEFT', left), ('RIGHT', right)):
        state[f'ROBOT_{side}_TRANS'] = np.asarray(trans, dtype=float).reshape(3).tolist()
        state[f'ROBOT_{side}_ROT_MAT'] = np.asarray(rot, dtype=float).reshape(3, 3).tolist()
        state[f'ROBOT_{side}_GRIPPER'] = [float(np.asarray(grip).reshape(-1)[0])]
    return {
        'task_description': str(task_description),
        'has_left': bool(has_left),
        'has_right': bool(has_right),
        'state': state,
        'head_camera_in_world':
            np.asarray(head_camera_in_world, dtype=float).reshape(4, 4).tolist(),
    }


def parse_action(body: Mapping[str, Any]) -> dict[str, dict[str, np.ndarray]]:
    """校验并转成 numpy：``{'left'|'right': {'trans','rot','grip'}}``，仍在模型系。"""
    missing = [k for k in _TRANS + _ROT + _GRIP if k not in body]
    if missing:
        raise ValueError(f'返回缺字段: {missing}')
    out: dict[str, dict[str, np.ndarray]] = {}
    horizons = set()
    for side, key_trans, key_rot, key_grip in (
            ('left', _TRANS[0], _ROT[0], _GRIP[0]),
            ('right', _TRANS[1], _ROT[1], _GRIP[1])):
        trans = np.asarray(body[key_trans], dtype=np.float64).reshape(-1, 3)
        rot = np.asarray(body[key_rot], dtype=np.float64).reshape(-1, 3, 3)
        grip = np.asarray(body[key_grip], dtype=np.float64).reshape(-1)
        horizons.update((trans.shape[0], rot.shape[0], grip.shape[0]))
        out[side] = {'trans': trans, 'rot': rot, 'grip': grip}
    if len(horizons) != 1:
        raise ValueError(f'各字段的 horizon 不一致: {sorted(horizons)}')
    if horizons.pop() == 0:
        raise ValueError('horizon 为 0')
    for side in out:
        for name, array in out[side].items():
            if not np.all(np.isfinite(array)):
                raise ValueError(f'{side}.{name} 含非有限值')
    return out


class A2DOmnipickerBackend(VlaBackend):
    """一个长连接 ``requests.Session``，串行使用（只有推理线程碰它）。"""

    def __init__(self, spec: VlaSpec, url: str, timeout: float = 30.0,
                 proxy: str = '', reproject: bool = False) -> None:
        super().__init__(spec)
        self.url = url
        self.timeout = float(timeout)
        self._frame = spec.frame.transform()
        self._reproject = bool(reproject)
        self._reprojector: Reprojector | None = None
        self._session = requests.Session()
        # 空 proxy 时保留 trust_env，让 ALL_PROXY 之类的环境变量生效。
        if proxy:
            self._session.proxies = {'http': proxy, 'https': proxy}

    # -- VlaBackend ---------------------------------------------------------

    def infer(self, observation: Observation) -> ActionChunk:
        images = self._encode(observation)
        self.dump(dict(zip(self.spec.images.slots, images)))
        payload = self._payload(observation)
        files = [(part, (filename, data, 'image/jpeg'))
                 for (part, filename), data in zip(IMAGE_PARTS, images)]
        files.append(('json', ('data.json',
                               json.dumps(payload, ensure_ascii=False).encode('utf-8'),
                               'application/json')))
        response = self._session.post(self.url, files=files, timeout=self.timeout)
        response.raise_for_status()
        return self._to_chunk(parse_action(response.json()))

    def stats(self) -> dict[str, Any]:
        if self._reprojector is None:
            return {'reproject': self._reproject}
        return {'reproject': True, 'coverage': round(self._reprojector.coverage, 3)}

    def close(self) -> None:
        self._session.close()

    # -- 内部 ---------------------------------------------------------------

    def _encode(self, observation: Observation) -> list[bytes]:
        frames = dict(observation.images)
        missing = [s for s in self.spec.images.slots if s not in frames]
        if missing:
            raise ValueError(f'缺图像 {missing}')
        if self._reproject:
            frames['head'] = self._ensure_reprojector(observation.camera)(frames['head'])
        return [encode_jpeg(frames[slot], self.spec.images.height,
                            self.spec.images.jpeg_quality)
                for slot in self.spec.images.slots]

    def _ensure_reprojector(self, camera: CameraCalibration | None) -> Reprojector:
        """本机内参一到就建映射表；换了 profile 会自动重建。"""
        if camera is None:
            raise RuntimeError('收不到头部 camera_info，重投影没法建表')
        size = tuple(int(v) for v in camera.size)
        if self._reprojector is None or self._reprojector.source_size != size:
            target_k, target_size = scale_intrinsics(
                TRAIN_HEAD_CAMERA.intrinsics, TRAIN_HEAD_CAMERA.size, self.spec.images.height)
            self._reprojector = Reprojector(
                camera.intrinsics, size, target_k, target_size,
                source_distortion=camera.distortion,
                target_distortion=TRAIN_HEAD_CAMERA.distortion)
        return self._reprojector

    def _payload(self, observation: Observation) -> dict[str, Any]:
        state = {}
        for side in SIDES:
            trans, rot = self._frame.to_model(observation.poses[side])
            grip = float(self.spec.gripper.to_model(observation.grippers[side]))
            state[side] = (trans, rot, grip)
        # 身体高度和俯仰训练时就是靠这一项告知模型的，旋转必须一并搬进模型系。
        camera = self._frame.base_to_model(observation.camera_in_base)
        return build_payload(observation.task, observation.enabled['left'],
                             observation.enabled['right'],
                             state['left'], state['right'], camera)

    def _to_chunk(self, action: Mapping[str, Mapping[str, np.ndarray]]) -> ActionChunk:
        poses, grippers = {}, {}
        for side in SIDES:
            trans, rot = action[side]['trans'], action[side]['rot']
            poses[side] = np.stack([self._frame.from_model(trans[i], rot[i])
                                    for i in range(trans.shape[0])])
            grippers[side] = self.spec.gripper.to_robot(action[side]['grip'])
        return ActionChunk(poses=poses, grippers=grippers)


def describe_reprojection(camera: CameraCalibration) -> str:
    """一行日志：两台相机的 FOV 与画布填充率。给节点启动时打。"""
    target_k, target_size = scale_intrinsics(
        TRAIN_HEAD_CAMERA.intrinsics, TRAIN_HEAD_CAMERA.size, SPEC.images.height)
    return ('本机 %dx%d FOV %.2f°x%.2f° -> 训练 %dx%d FOV %.2f°x%.2f°'
            % (*camera.size, *field_of_view(camera.intrinsics, camera.size),
               *target_size, *field_of_view(target_k, target_size)))


def create(params: Mapping[str, Any]) -> A2DOmnipickerBackend:
    frame = FrameSpec(
        origin_in_base=resolve_sequence(params.get('model_origin_in_base'), SPEC.frame.origin_in_base, 3, 'model_origin_in_base'),
        rotation_rpy=resolve_sequence(params.get('model_rotation_rpy'), SPEC.frame.rotation_rpy, 3, 'model_rotation_rpy'),
        tool_offset=resolve_sequence(params.get('tool_offset'), SPEC.frame.tool_offset, 3, 'tool_offset'),
        tool_rotation_rpy=resolve_sequence(
            params.get('tool_rotation_rpy'),
            SPEC.frame.tool_rotation_rpy, 3,
            'tool_rotation_rpy'
        )
    )
    url = str(params.get('server_url') or PARAMETERS['server_url'])
    if not url.startswith(('http://', 'https://')):
        raise ValueError(f'server_url 必须是 http(s) 地址，收到 {url!r}')
    return A2DOmnipickerBackend(
        replace(SPEC, frame=frame), url,
        timeout=float(params.get('request_timeout_s') or 30.0),
        proxy=str(params.get('proxy') or ''),
        reproject=bool(params.get('head_reproject', False)))
