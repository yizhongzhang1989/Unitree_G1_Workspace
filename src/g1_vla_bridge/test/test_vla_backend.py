"""VLA 接入层：规格声明的语义、数据结构校验、backend 加载。

这里钉死的是**跨 VLA 都得成立**的约定，接新 VLA 时先跑这一组。
"""

import numpy as np
import pytest

from g1_vla_bridge.backends.cogact_unitree import SPEC as COGACT
from g1_vla_bridge.transforms import invert_pose, pose_matrix, rpy_to_mat, solve_base_frame
from g1_vla_bridge.vla_backend import (
    SIDES,
    ActionChunk,
    FrameSpec,
    ImageSpec,
    VlaBackend,
    backend_parameters,
    load_backend,
)

IDENTITY_QUAT = [0.0, 0.0, 0.0, 1.0]


def _pose(xyz):
    return np.concatenate([xyz, IDENTITY_QUAT])


@pytest.mark.parametrize('origin', ([0.0, 0.0, 0.0], [0.0, 0.0, -0.493], [0.3, -0.1, 0.2]))
def test_origin_in_base_is_the_model_origin(origin):
    """``origin_in_base`` 的定义：base_frame 里的这个点，在 VLA 系里就是原点。"""
    frame = FrameSpec(origin_in_base=origin).transform()
    trans, _ = frame.to_model(_pose(origin))
    assert np.allclose(trans, 0.0, atol=1e-12)


def test_origin_in_base_holds_with_rotation():
    frame = FrameSpec(origin_in_base=[0.2, -0.1, -0.5], rotation_rpy=[0.1, -0.6, 0.3]).transform()
    trans, _ = frame.to_model(_pose([0.2, -0.1, -0.5]))
    assert np.allclose(trans, 0.0, atol=1e-12)


def test_from_solution_matches_calibration():
    """解出的变换转成声明式字段后必须还是同一个变换。"""
    model_from_base = np.eye(4)
    model_from_base[:3, :3] = rpy_to_mat([0.05, -0.62, 0.11])
    model_from_base[:3, 3] = [0.37, -0.04, 0.54]
    base_cam = pose_matrix([-0.658, 0.662, -0.253, 0.254], [0.057, 0.033, 0.430])

    spec = FrameSpec.from_solution(*solve_base_frame(model_from_base @ base_cam, base_cam))
    # 声明出来的原点 = base_from_model 的平移，也就是 VLA 原点在 base 里的位置。
    assert np.allclose(spec.origin_in_base, invert_pose(model_from_base)[:3, 3], atol=1e-12)

    point = np.array([0.30, 0.25, 0.07])
    trans, _ = spec.transform().to_model(_pose(point))
    assert np.allclose(trans, model_from_base[:3, :3] @ point + model_from_base[:3, 3],
                       atol=1e-12)


def test_image_spec_rejects_unknown_slot():
    with pytest.raises(ValueError):
        ImageSpec(slots=('head', 'belly'))


def _chunk(horizon=3):
    return ActionChunk(poses={s: np.tile(_pose([0.1, 0.2, 0.3]), (horizon, 1)) for s in SIDES},
                       grippers={s: np.zeros(horizon) for s in SIDES})


def test_action_chunk_horizon():
    assert _chunk(5).horizon == 5


@pytest.mark.parametrize('broken', [
    lambda c: c.poses.__setitem__('left', c.poses['left'][:2]),
    lambda c: c.grippers.__setitem__('right', np.zeros(7)),
    lambda c: c.poses.__setitem__('left', c.poses['left'][:, :6]),
])
def test_action_chunk_rejects_inconsistent_shapes(broken):
    chunk = _chunk()
    broken(chunk)
    with pytest.raises(ValueError):
        ActionChunk(poses=chunk.poses, grippers=chunk.grippers)


def test_load_backend_round_trip(monkeypatch):
    from g1_vla_bridge.backends.cogact_unitree import CogACTUnitreeBackend

    monkeypatch.setattr(CogACTUnitreeBackend, 'configure', lambda self: None)
    backend = load_backend('cogact_unitree', backend_parameters('cogact_unitree'))
    assert isinstance(backend, VlaBackend)
    assert backend.spec.name == 'cogact_unitree'
    assert backend.spec.action_semantics == 'absolute'
    backend.close()


@pytest.mark.parametrize('name', ['../etc/passwd', 'os.path', 'nope', ''])
def test_load_backend_rejects_junk(name):
    # 名字来自 ROS 参数，不能拿去 import 任意模块。
    with pytest.raises(ValueError):
        backend_parameters(name)


def test_spec_summary_answers_the_frame_question():
    summary = COGACT.summary()
    assert summary['origin_in_base'] == [0.0, 0.0, 0.0]
    assert summary['action_semantics'] == 'absolute'
    assert summary['image_height'] == 360
