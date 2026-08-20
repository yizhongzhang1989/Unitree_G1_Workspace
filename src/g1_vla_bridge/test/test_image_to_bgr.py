"""``image_to_bgr`` 的编码支持与内存布局。

rgb8 那条分支曾用 ``np.ascontiguousarray(frame[:, :, ::-1])``，负步长拷贝没有 SIMD，
424x240 实测 773 us/帧，换成 cv2 后 69 us。这里钉住结果不变，别再换回去。
"""

import cv2
import numpy as np
import pytest
from sensor_msgs.msg import Image

from g1_vla_bridge.vla_node import image_to_bgr

H, W = 12, 20


def _image(encoding, depth, pad=0, payload=None):
    msg = Image()
    msg.height, msg.width, msg.encoding = H, W, encoding
    msg.step = W * depth + pad
    if payload is None:
        payload = np.arange(H * msg.step, dtype=np.uint8)
    msg.data = payload.astype(np.uint8).tobytes()
    return msg


@pytest.mark.parametrize('pad', [0, 7])
def test_bgr8_is_passthrough(pad):
    msg = _image('bgr8', 3, pad)
    src = np.frombuffer(msg.data, np.uint8).reshape(H, msg.step)[:, :W * 3]
    assert np.array_equal(image_to_bgr(msg), src.reshape(H, W, 3))


@pytest.mark.parametrize('pad', [0, 7])
def test_rgb8_swaps_channels(pad):
    msg = _image('rgb8', 3, pad)
    src = np.frombuffer(msg.data, np.uint8).reshape(H, msg.step)[:, :W * 3]
    assert np.array_equal(image_to_bgr(msg), src.reshape(H, W, 3)[:, :, ::-1])


@pytest.mark.parametrize('pad', [0, 7])
def test_yuyv_matches_cv2(pad):
    msg = _image('yuv422_yuy2', 2, pad)
    src = np.frombuffer(msg.data, np.uint8).reshape(H, msg.step)[:, :W * 2]
    expected = cv2.cvtColor(np.ascontiguousarray(src.reshape(H, W, 2)),
                            cv2.COLOR_YUV2BGR_YUY2)
    assert np.array_equal(image_to_bgr(msg), expected)


@pytest.mark.parametrize('encoding,depth', [('bgr8', 3), ('rgb8', 3), ('yuv422_yuy2', 2)])
def test_output_shape_and_contiguity(encoding, depth):
    out = image_to_bgr(_image(encoding, depth))
    assert out.shape == (H, W, 3 if encoding != 'yuv422_yuy2' else 3)
    # 下游直接喂 cv2.imencode，非连续内存会静默走一次隐式拷贝
    assert out.flags['C_CONTIGUOUS']


def test_yuyv_gray_stays_gray():
    """Y=128、UV=128 是中性灰，转出来三通道应该一致 —— 防止 YUYV 的分量顺序接反。"""
    payload = np.full(H * W * 2, 128, dtype=np.uint8)
    out = image_to_bgr(_image('yuv422_yuy2', 2, payload=payload))
    assert out.std() < 2.0 and abs(float(out.mean()) - 128.0) < 4.0


def test_rejects_unknown_encoding():
    with pytest.raises(ValueError, match='不支持的编码'):
        image_to_bgr(_image('mono8', 1))


def test_rejects_short_payload():
    msg = _image('bgr8', 3)
    msg.data = msg.data[:-1]
    with pytest.raises(ValueError, match='图像数据长度'):
        image_to_bgr(msg)
