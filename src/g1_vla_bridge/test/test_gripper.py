"""夹爪方向：两边的约定是反的，写错就是「该松手时夹紧」，实机上很难当场认出来。

映射写在 backend 的 ``SPEC.gripper`` 里，所以直接对着规格测。
"""

import numpy as np
import pytest

from g1_vla_bridge.backends.a2d_omnipicker import SPEC
from g1_vla_bridge.vla_backend import GripperSpec

GRIP = SPEC.gripper


def test_model_open_maps_to_our_open():
    """模型 0 = 张开；偏心轴张开是行程**上**界 2.76377 rad，不是 0。"""
    assert GRIP.to_robot(GRIP.model_open) == pytest.approx(GRIP.robot_open_rad)
    assert GRIP.robot_open_rad == pytest.approx(max(GRIP.robot_limits))


def test_model_closed_maps_to_our_closed():
    assert GRIP.to_robot(GRIP.model_closed) == pytest.approx(GRIP.robot_closed_rad)


def test_mapping_is_inverted_and_spans_full_travel():
    """恒等映射会只走 36% 行程且方向反了，这里把两件事一起拦住。"""
    assert GRIP.robot_open_rad > GRIP.robot_closed_rad      # 与模型的方向相反
    span = abs(float(GRIP.to_robot(GRIP.model_closed) - GRIP.to_robot(GRIP.model_open)))
    assert span == pytest.approx(GRIP.robot_limits[1] - GRIP.robot_limits[0])


def test_round_trip():
    """发给模型的 state 用的是逆映射，两者必须严格互逆。"""
    values = np.array([0.0, 0.25, 0.5, 0.75, 1.0])
    assert np.allclose(GRIP.to_model(GRIP.to_robot(values)), values)


def test_clips_to_travel():
    """模型偶尔会吐出界的值，不能直接变成超程指令。"""
    assert GRIP.to_robot(-3.0) == pytest.approx(GRIP.robot_limits[1])
    assert GRIP.to_robot(9.0) == pytest.approx(GRIP.robot_limits[0])


def test_vectorised():
    assert GRIP.to_robot(np.zeros(5)).shape == (5,)


def test_degenerate_spec_is_rejected():
    with pytest.raises(ValueError):
        GripperSpec(model_open=1.0, model_closed=1.0)
    with pytest.raises(ValueError):
        GripperSpec(robot_open_rad=0.5, robot_closed_rad=0.5).to_model(0.0)
