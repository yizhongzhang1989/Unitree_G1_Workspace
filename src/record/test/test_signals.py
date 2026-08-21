"""信号提取的契约。全部用假消息，不需要 ROS 运行。

最要紧的一条：**关节必须按名字重排**。``/joint_states`` 的顺序每次启动都可能不同，
按位置索引存会静默错列，而且不同 session 之间还不一致，事后无从发现。
"""

import json
import math
from types import SimpleNamespace as NS

import pytest

from record import signals as sig


def _js(names, pos=None, vel=None, eff=None, stamp=(3, 500000000)):
    n = len(names)
    return NS(name=list(names),
              position=list(pos if pos is not None else range(n)),
              velocity=list(vel if vel is not None else [x * 10 for x in range(n)]),
              effort=list(eff if eff is not None else [x * 100 for x in range(n)]),
              header=NS(stamp=NS(sec=stamp[0], nanosec=stamp[1])))


def test_header_stamp_combines_sec_and_nanosec():
    assert sig.header_stamp(_js(['a'])) == pytest.approx(3.5)


def test_header_stamp_nan_without_header():
    assert math.isnan(sig.header_stamp(NS()))


def test_joint_reorder_is_by_name_not_position():
    """乱序输入必须按名字对上号 —— 这是整套数据不错列的根。"""
    order = ['a', 'b', 'c']
    shuffled = _js(['c', 'a', 'b'], pos=[30, 10, 20], vel=[3, 1, 2], eff=[300, 100, 200])
    row = sig.joint_state_row(shuffled, order)
    assert row[:3] == [10, 20, 30]
    assert row[3:6] == [1, 2, 3]
    assert row[6:9] == [100, 200, 300]


def test_joint_reorder_fills_missing_with_nan():
    row = sig.joint_state_row(_js(['a'], pos=[1], vel=[2], eff=[3]), ['a', 'zzz'])
    assert row[0] == 1 and math.isnan(row[1])


def test_joint_reorder_ignores_extra_joints():
    row = sig.joint_state_row(_js(['a', 'x'], pos=[1, 9], vel=[0, 0], eff=[0, 0]),
                              ['a'])
    assert row[0] == 1 and len(row) == 3


def test_joint_columns_are_grouped_by_field():
    cols = sig.joint_state_columns(['a', 'b'])
    assert cols == ['pos.a', 'pos.b', 'vel.a', 'vel.b', 'eff.a', 'eff.b']


def test_canonical_joint_order_has_31_axes():
    assert len(sig.CANONICAL_JOINTS) == 31
    assert sig.CANONICAL_JOINTS[-2:] == ('left_eccentric_joint', 'right_eccentric_joint')
    assert len(set(sig.CANONICAL_JOINTS)) == 31


def test_float_array_pads_and_truncates():
    assert sig.float_array_row(NS(data=[1, 2]), 4)[:2] == [1.0, 2.0]
    assert all(math.isnan(v) for v in sig.float_array_row(NS(data=[1, 2]), 4)[2:])
    assert sig.float_array_row(NS(data=[1, 2, 3, 4, 5]), 3) == [1.0, 2.0, 3.0]


def test_wrench_row_order():
    w = NS(wrench=NS(force=NS(x=1, y=2, z=3), torque=NS(x=4, y=5, z=6)))
    assert sig.wrench_row(w) == [1, 2, 3, 4, 5, 6]


def test_imu_row_order():
    m = NS(orientation=NS(x=1, y=2, z=3, w=4),
           angular_velocity=NS(x=5, y=6, z=7),
           linear_acceleration=NS(x=8, y=9, z=10))
    assert sig.imu_row(m) == [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]


def test_imu_state_row_handles_missing_rpy():
    m = NS(quaternion=[1, 2, 3, 4], gyroscope=[5, 6, 7], accelerometer=[8, 9, 10])
    row = sig.imu_state_row(m)
    assert row[:10] == [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    assert all(math.isnan(v) for v in row[10:])


# ------------------------------------------------------- status 定宽

def test_status_columns_are_fixed_width():
    cols = sig.status_columns()
    assert len(cols) == len(set(cols))
    assert len(cols) == 5 + 6 + 2 + 20 + 14


def test_status_row_width_is_stable_before_and_after_engage():
    """limited_pose 只在 engage 后出现；列数变化会让 reshape 直接错位。"""
    before = sig.status_row(NS(data=json.dumps(
        {'state': 'idle', 'ready_to_start': False, 'grip': [0.1, 0.2],
         'command': [0.0] * 20})))
    after = sig.status_row(NS(data=json.dumps(
        {'state': 'run', 'ready_to_start': True, 'arms_live': True,
         'grip': [0.1, 0.2], 'command': [1.0] * 20,
         'limited_pose': {'left': [1, 2, 3, 0, 0, 0, 1],
                          'right': [4, 5, 6, 0, 0, 0, 1]}})))
    assert len(before) == len(after) == len(sig.status_columns())
    assert all(math.isnan(v) for v in before[-14:])
    assert after[-14:-7] == [1, 2, 3, 0, 0, 0, 1]


def test_status_state_is_one_hot():
    row = sig.status_row(NS(data=json.dumps({'state': 'estop'})))
    onehot = row[:len(sig.STATUS_STATES)]
    assert sum(onehot) == 1
    assert onehot[sig.STATUS_STATES.index('estop')] == 1.0


def test_status_row_survives_garbage():
    row = sig.status_row(NS(data='not json at all'))
    assert len(row) == len(sig.status_columns())
    assert all(math.isnan(v) for v in row)


def test_status_bools_become_numbers():
    row = sig.status_row(NS(data=json.dumps({'state': 'run', 'arms_live': True,
                                             'ik_ready': False})))
    cols = sig.status_columns()
    assert row[cols.index('arms_live')] == 1.0
    assert row[cols.index('ik_ready')] == 0.0


# ------------------------------------------------------- 话题清单

def test_specs_are_unique_and_column_widths_match():
    specs = sig.default_specs()
    assert len({s.key for s in specs}) == len(specs)
    assert len({s.topic for s in specs}) == len(specs)
    for s in specs:
        if s.columns:
            assert len(s.columns) == len(set(s.columns)), s.key


def test_command_topic_is_best_effort():
    """发布方是 BEST_EFFORT depth=1，用 RELIABLE 订阅会一条都收不到。"""
    spec = next(s for s in sig.default_specs() if s.key == 'motion_control_command')
    assert spec.best_effort and len(spec.columns) == 20


def test_high_rate_topics_are_throttled():
    by = {s.key: s for s in sig.default_specs()}
    assert by['arm0_wrench_raw'].max_hz == 200.0     # 实测 743 Hz
    assert by['secondary_imu'].max_hz == 100.0       # 实测 755 Hz
    assert by['joint_states'].max_hz == 0.0          # 100 Hz，不抽


def test_tf_is_off_by_default():
    """665 列/条、96 KB/s，而且能从 joint_states + URDF 离线重算。"""
    assert not next(s for s in sig.default_specs() if s.key == 'tf').default_on


def test_joint_states_spec_uses_canonical_order():
    spec = next(s for s in sig.default_specs() if s.key == 'joint_states')
    assert len(spec.columns) == 93
    assert spec.columns[0] == f'pos.{sig.CANONICAL_JOINTS[0]}'
    row = spec.row(_js(list(reversed(sig.CANONICAL_JOINTS))))
    assert len(row) == 93
