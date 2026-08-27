"""信号提取的契约。全部用假消息，不需要 ROS 运行。

最要紧的一条：**关节必须按名字重排**。``/joint_states`` 的顺序每次启动都可能不同，
按位置索引存会静默错列，而且不同 session 之间还不一致，事后无从发现。
"""

import json
import math
from pathlib import Path
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


def test_command_topics_are_best_effort():
    """两条指令话题的发布方都是 BEST_EFFORT，用 RELIABLE 订阅会一条都收不到。

    `fpc_commands` 就是这么丢的：20260821 和 20260824 两个 session 都录成 0 行，
    直到导出时 action/joint_space 全 NaN 才发现。发布方在
    `g1_motion_control/policy_node.py` 里是 depth=1 BEST_EFFORT。
    """
    by = {s.key: s for s in sig.default_specs()}
    assert by['motion_control_command'].best_effort
    assert len(by['motion_control_command'].columns) == 20
    assert by['fpc_commands'].best_effort


def test_high_rate_topics_are_throttled():
    by = {s.key: s for s in sig.default_specs()}
    assert by['arm0_wrench_raw'].max_hz == 200.0     # 实测 743 Hz
    assert by['secondary_imu'].max_hz == 100.0       # 实测 755 Hz
    assert by['dog_odom'].max_hz == 100.0            # 实测 500 Hz
    assert by['joint_states'].max_hz == 0.0          # 100 Hz，不抽


def test_dog_odom_subscribes_with_depth_one():
    """深度就是积压上限，对 500 Hz 的话题不能用默认的 50。

    实测同一条 `/dog_odom`：depth=50 时 `t_recv - t_header` 恒定 47.95 ms，
    depth=1 时 0.92 ms。滞后是常量，看不出丢帧、也不报错，只会把时间轴整体推后。
    """
    by = {s.key: s for s in sig.default_specs()}
    assert by['dog_odom'].depth == 1
    # 其余都是低频或已抽稀，默认深度是给卡顿留的余量
    assert all(s.depth == 50 for s in sig.default_specs() if s.key != 'dog_odom')


def test_dog_odom_has_no_origin_flag():
    """它和 torso_pose 都是 Odometry，但 covariance[0] 在这里不是标志位。

    固件那份实测 36 项全零，照 torso_pose 的读法会把每一帧都判成「原点已设」。
    """
    by = {s.key: s for s in sig.default_specs()}
    assert 'origin_set' not in by['dog_odom'].columns
    assert 'origin_set' in by['torso_pose'].columns
    assert len(by['dog_odom'].columns) == 13


def test_tf_is_off_by_default():
    """665 列/条、96 KB/s，而且能从 joint_states + URDF 离线重算。"""
    assert not next(s for s in sig.default_specs() if s.key == 'tf').default_on


def test_joint_states_spec_uses_canonical_order():
    spec = next(s for s in sig.default_specs() if s.key == 'joint_states')
    assert len(spec.columns) == 93
    assert spec.columns[0] == f'pos.{sig.CANONICAL_JOINTS[0]}'
    row = spec.row(_js(list(reversed(sig.CANONICAL_JOINTS))))
    assert len(row) == 93


# ------------------------------------------------------- 躯干世界位姿

def _odom(cov0=0.0, xyz=(1.0, 2.0, 3.0), quat=(0.0, 0.0, 0.0, 1.0)):
    cov = [0.0] * 36
    cov[0] = cov0
    return NS(pose=NS(pose=NS(position=NS(x=xyz[0], y=xyz[1], z=xyz[2]),
                              orientation=NS(x=quat[0], y=quat[1],
                                             z=quat[2], w=quat[3])),
                      covariance=cov),
              twist=NS(twist=NS(linear=NS(x=0.1, y=0.2, z=0.3),
                                angular=NS(x=0.4, y=0.5, z=0.6))))


def test_odometry_row_layout():
    spec = next(s for s in sig.default_specs() if s.key == 'torso_pose')
    row = spec.row(_odom())
    assert len(row) == len(spec.columns)
    cols = spec.columns
    assert [row[cols.index(k)] for k in ('x', 'y', 'z')] == [1.0, 2.0, 3.0]
    assert [row[cols.index(k)] for k in ('qx', 'qy', 'qz', 'qw')] == [0.0, 0.0, 0.0, 1.0]
    assert row[cols.index('wz')] == 0.6


def test_origin_unset_is_flagged_per_frame():
    """调过 set_origin 之后队列里的残留帧仍带 -1，只能逐帧判。"""
    cols = next(s for s in sig.default_specs() if s.key == 'torso_pose').columns
    flag = cols.index('origin_set')
    assert sig.odometry_row(_odom(cov0=sig.ORIGIN_UNSET_COV))[flag] == 0.0
    assert sig.odometry_row(_odom(cov0=0.0))[flag] == 1.0


def test_origin_unset_constant_still_matches_the_publisher():
    """这个值是 g1_localization 的对外契约，两边分叉了会静默把无效帧当成有效帧。"""
    source = (Path(__file__).resolve().parents[3] / 'src' / 'g1_localization'
              / 'g1_localization' / 'localization_node.py')
    if not source.is_file():
        pytest.skip('工作区里没有 g1_localization')
    line = next(ln for ln in source.read_text(encoding='utf-8').splitlines()
                if ln.startswith('ORIGIN_UNSET_COV'))
    assert float(line.split('=')[1]) == sig.ORIGIN_UNSET_COV


def test_torso_pose_is_on_by_default_and_not_throttled():
    """默认勾选：世界定位事后补不回来。10 Hz 本来就慢，不抽稀。"""
    spec = next(s for s in sig.default_specs() if s.key == 'torso_pose')
    assert spec.default_on
    assert spec.max_hz == 0.0
    assert spec.best_effort                    # 发布方是 BEST_EFFORT，RELIABLE 收不到


def test_origin_unset_shows_up_on_the_panel():
    """频率、行数、绿灯全都正常，只有内容是废的 —— 这种只能靠体检暴露。"""
    spec = next(s for s in sig.default_specs() if s.key == 'torso_pose')
    assert spec.health is not None
    warning = spec.health(_odom(cov0=sig.ORIGIN_UNSET_COV))
    assert 'set_origin' in warning
    assert spec.health(_odom(cov0=0.0)) == ''


def test_only_torso_pose_pays_for_the_health_check():
    """体检在落盘前逐帧跑，别的路挂上去就是白白多一次调用。"""
    assert [s.key for s in sig.default_specs() if s.health is not None] == ['torso_pose']
