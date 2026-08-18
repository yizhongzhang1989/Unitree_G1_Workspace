"""a2d_omnipicker 的线协议：图像编码、payload 组装、返回体校验；不联网、不起 ROS。"""

import json

import cv2
import numpy as np
import pytest

from g1_vla_bridge.backends.a2d_omnipicker import (
    IMAGE_PARTS,
    PARAMETERS,
    SPEC,
    build_payload,
    create,
    encode_jpeg,
    parse_action,
)
from g1_vla_bridge.vla_backend import SIDES, ActionChunk

_EYE = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def _frame(height, width):
    rng = np.random.default_rng(0)
    return rng.integers(0, 256, size=(height, width, 3), dtype=np.uint8)


@pytest.mark.parametrize('shape', [(1080, 1920), (240, 424), (120, 160)])
def test_encode_jpeg_lands_on_240(shape):
    data = encode_jpeg(_frame(*shape), 240)
    decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape[0] == 240
    # 等比缩放：宽高比保持不变（JPEG 的 8 像素对齐允许 1 px 误差）。
    assert abs(decoded.shape[1] - round(shape[1] * 240 / shape[0])) <= 1


def test_encode_jpeg_accepts_readonly_view():
    # 订阅回调里是 np.frombuffer(msg.data)，得到的数组是只读的。
    raw = _frame(240, 424).tobytes()
    view = np.frombuffer(raw, np.uint8).reshape(240, 424, 3)
    assert not view.flags.writeable
    assert encode_jpeg(view, 240)


def test_encode_jpeg_rejects_bad_shape():
    with pytest.raises(ValueError):
        encode_jpeg(np.zeros((240, 424), np.uint8), 240)


def test_build_payload_matches_spec():
    payload = build_payload(
        'pick it up', True, False,
        ([0.61, 0.30, 0.72], _EYE, 0.0),
        ([0.56, -0.29, 0.75], _EYE, 1.5),
        np.eye(4))
    assert payload['task_description'] == 'pick it up'
    assert payload['has_left'] is True and payload['has_right'] is False
    assert np.shape(payload['head_camera_in_world']) == (4, 4)
    state = payload['state']
    assert np.shape(state['ROBOT_LEFT_ROT_MAT']) == (3, 3)
    assert state['ROBOT_RIGHT_TRANS'] == [0.56, -0.29, 0.75]
    assert state['ROBOT_RIGHT_GRIPPER'] == [1.5]
    # 必须能原样 JSON 化：numpy 标量混进去会在 POST 时才炸。
    assert json.loads(json.dumps(payload)) == payload


def _body(horizon=3, **override):
    body = {}
    for side in ('LEFT', 'RIGHT'):
        body[f'ROBOT_{side}_TRANS'] = [[0.1, 0.2, 0.3]] * horizon
        body[f'ROBOT_{side}_ROT_MAT'] = [_EYE] * horizon
        body[f'ROBOT_{side}_GRIPPER'] = [[0.0]] * horizon
    body.update(override)
    return body


def test_parse_action_shapes():
    action = parse_action(_body(30))
    for side in ('left', 'right'):
        assert action[side]['trans'].shape == (30, 3)
        assert action[side]['rot'].shape == (30, 3, 3)
        assert action[side]['grip'].shape == (30,)


def test_parse_action_ignores_extra_fields():
    parse_action(_body(2, ROBOT_LEFT_ROT_6D=[[1, 0, 0, 0, 1, 0]] * 2))


@pytest.mark.parametrize('body', [
    {k: v for k, v in _body().items() if k != 'ROBOT_LEFT_TRANS'},
    _body(0),
    _body(3, ROBOT_RIGHT_TRANS=[[0.1, 0.2, 0.3]] * 4),
    _body(3, ROBOT_LEFT_GRIPPER=[[0.0], [float('nan')], [0.0]]),
])
def test_parse_action_rejects_bad_body(body):
    with pytest.raises(ValueError):
        parse_action(body)


def test_image_part_order():
    # 顺序写死在接口里：head / left wrist / right wrist，错了模型不会报错只会变傻。
    assert [name for name, _ in IMAGE_PARTS] == ['image_0', 'image_1', 'image_2']
    assert SPEC.images.slots == ('head', 'left_wrist', 'right_wrist')
    assert len(IMAGE_PARTS) == len(SPEC.images.slots)


def test_chunk_comes_back_in_base_frame():
    """backend 必须把模型系换回 base_frame，节点永远只看 base_frame。"""
    backend = create(PARAMETERS)
    chunk = backend._to_chunk(parse_action(_body(4)))
    assert isinstance(chunk, ActionChunk) and chunk.horizon == 4
    frame = SPEC.frame.transform()
    for side in SIDES:
        assert chunk.poses[side].shape == (4, 7)
        expect = frame.from_model([0.1, 0.2, 0.3], _EYE)
        assert np.allclose(chunk.poses[side][0], expect)
        # 模型 0 = 张开，我们的偏心轴张开是行程**上**界。
        assert chunk.grippers[side][0] == pytest.approx(SPEC.gripper.robot_open_rad)
    backend.close()


def test_create_rejects_bad_url():
    with pytest.raises(ValueError):
        create(dict(PARAMETERS, server_url='ftp://nope/api'))


def test_create_rejects_wrong_length_calibration():
    with pytest.raises(ValueError):
        create(dict(PARAMETERS, model_origin_in_base=[0.0, 0.0]))
