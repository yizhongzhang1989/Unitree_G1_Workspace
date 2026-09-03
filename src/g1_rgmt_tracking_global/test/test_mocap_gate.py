from types import SimpleNamespace

import numpy as np
from g1_mocap.consumer import FrameBuffer

from g1_rgmt_tracking_global.mocap_gate import MocapFrameGate, ZeroReferenceFactory


def _pose(value: float):
    return SimpleNamespace(
        position=SimpleNamespace(x=value, y=0.0, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def _tilted_pose(value: float):
    half = np.pi / 8.0
    return SimpleNamespace(
        position=SimpleNamespace(x=value, y=0.0, z=0.0),
        orientation=SimpleNamespace(x=np.sin(half), y=0.0, z=0.0, w=np.cos(half)),
    )


def _frame(stamp: float, value: float):
    sec = int(stamp)
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=sec, nanosec=int((stamp - sec) * 1e9))),
        joint_positions=[value],
        root=_pose(value),
        anchor=_pose(value),
        key_body_positions=[SimpleNamespace(x=value, y=0.0, z=0.0)],
    )


def _factory():
    return ZeroReferenceFactory(
        joint_positions=np.array([0.25]),
        anchor_local=np.array([1.0, 0.0, 0.0]),
        anchor_rot=np.eye(3),
        key_local=np.array([[0.0, 1.0, 0.0]]),
    )


def test_initial_default_then_live_then_hold():
    buffer = FrameBuffer(n_joints=1, n_keys=1)
    gate = MocapFrameGate(buffer, _factory())

    assert gate.push(_frame(0.0, 10.0), mode='default')
    assert gate.push(_frame(1.0, 11.0), mode='default')
    assert gate.push(_frame(2.0, 2.0), mode='live')
    assert gate.push(_frame(3.0, 3.0), mode='hold')
    assert gate.push(_frame(4.0, 4.0), mode='live')
    assert gate.push(_frame(5.0, 5.0), mode='live')

    batch = buffer.sample(np.arange(6.0))
    assert batch is not None
    assert np.array_equal(batch.joint_pos[:, 0], [0.25, 0.25, 2.0, 2.0, 4.0, 5.0])
    assert np.array_equal(batch.root_pos[:, 0], [10.0, 10.0, 10.0, 10.0, 10.0, 11.0])


def test_default_freezes_the_entire_reference_pose():
    buffer = FrameBuffer(n_joints=1, n_keys=1)
    gate = MocapFrameGate(buffer, _factory())

    assert gate.push(_frame(0.0, 10.0), mode='default')
    assert gate.push(_frame(1.0, 11.0), mode='default')

    batch = buffer.sample(np.array([0.0, 1.0]))
    assert batch is not None
    assert np.array_equal(batch.joint_pos[:, 0], [0.25, 0.25])
    assert np.array_equal(batch.root_pos[:, 0], [10.0, 10.0])
    assert np.array_equal(batch.anchor_pos[:, 0], [11.0, 11.0])


def test_default_to_live_keeps_the_world_trajectory_continuous():
    buffer = FrameBuffer(n_joints=1, n_keys=1)
    gate = MocapFrameGate(buffer, _factory())

    assert gate.push(_frame(0.0, 10.0), mode='default')
    assert gate.push(_frame(1.0, 11.0), mode='default')
    assert gate.push(_frame(2.0, 12.0), mode='live')
    assert gate.push(_frame(3.0, 13.0), mode='live')

    batch = buffer.sample(np.array([0.0, 1.0, 2.0, 3.0]))
    assert batch is not None
    assert np.array_equal(batch.root_pos[:, 0], [10.0, 10.0, 10.0, 11.0])
    assert np.array_equal(np.diff(batch.root_pos[:, 0]), [0.0, 0.0, 1.0])


def test_zero_reference_geometry_matches_fk_transform():
    buffer = FrameBuffer(n_joints=1, n_keys=1)
    gate = MocapFrameGate(buffer, _factory())
    assert gate.push(_frame(10.0, 5.0), mode='default')
    assert gate.push(_frame(11.0, 5.0), mode='default')
    batch = buffer.sample(np.array([10.0]))
    assert batch is not None
    assert batch.joint_pos[0, 0] == 0.25
    assert np.array_equal(batch.root_pos[0], [5.0, 0.0, 0.0])
    assert np.array_equal(batch.anchor_pos[0], [6.0, 0.0, 0.0])
    assert np.array_equal(batch.key_pos[0, 0], [5.0, 1.0, 0.0])


def test_default_reference_discards_human_root_tilt():
    buffer = FrameBuffer(n_joints=1, n_keys=1)
    gate = MocapFrameGate(buffer, _factory())
    frame = _frame(10.0, 5.0)
    frame.root = _tilted_pose(5.0)
    assert gate.push(frame, mode='default')
    frame.header.stamp.sec = 11
    assert gate.push(frame, mode='default')
    batch = buffer.sample(np.array([10.0]))
    assert batch is not None
    assert np.allclose(batch.root_quat[0], [1.0, 0.0, 0.0, 0.0])
    assert np.allclose(batch.anchor_pos[0], [6.0, 0.0, 0.0])
    assert np.allclose(batch.key_pos[0, 0], [5.0, 1.0, 0.0])
