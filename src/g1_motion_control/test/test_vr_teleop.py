"""vr_teleop 的离线校验。不需要真机，也不需要 VR 头显。

只测和安全直接相关的两件事：离合接合的瞬间**不能有位置跳变**（上肢不限速，跳一下
就是电机全权限的弹动），以及 trigger 到夹爪的映射方向没写反（写反 = 该松手时夹紧）。

节点用 ``__new__`` 造出来、只填这几个方法用到的字段：这里要测的是纯逻辑，不想拉起
ROS 图、也不想连 VR 桥。
"""

import numpy as np
import pytest

from g1_motion_control.vr_teleop import _BASE_MAP, VRTeleop, _matrix

OPEN = 2.76377472169236       # eccentric 全开
CLOSED = 0.0                  # eccentric 闭合
UNIT = (0.0, 0.0, 0.0, 1.0)   # 单位四元数 xyzw


class _Silent:
    def info(self, *args, **kwargs):
        pass

    def warning(self, *args, **kwargs):
        pass


def _frame(squeeze, trigger, position, orientation=UNIT):
    return {side: {'grip': {'position': list(position),
                            'orientation': list(orientation)},
                   'buttons': {'squeeze': squeeze, 'trigger': trigger}}
            for side in ('left', 'right')}


def _spin(axis, angle):
    """绕 axis 转 angle 的四元数（xyzw）。"""
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)
    return np.concatenate([axis * np.sin(angle / 2.0), [np.cos(angle / 2.0)]])


def _angle_of(rotation):
    """旋转矩阵的转角。"""
    return float(np.arccos(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)))


@pytest.fixture
def node():
    teleop = VRTeleop.__new__(VRTeleop)
    teleop._squeeze_on = 0.5
    teleop._arm_scale = 1.0
    teleop._grip_open, teleop._grip_closed = OPEN, CLOSED
    teleop._clutch = {'left': None, 'right': None}
    teleop._pose = {'left': np.array([0.30, 0.15, 0.05, 0.0, 0.0, 0.0, 1.0]),
                    'right': np.array([0.30, -0.15, 0.05, 0.0, 0.0, 0.0, 1.0])}
    teleop._grip = np.zeros(2)
    teleop._map = _BASE_MAP.copy()
    teleop._calib_held = {'left': True, 'right': True}
    teleop.get_logger = lambda: _Silent()
    return teleop


def test_below_threshold_does_not_move_the_arm(node):
    before = node._pose['right'].copy()
    node._update_arms(_frame(0.2, 0.0, [0.5, 0.8, -0.3]))
    assert np.array_equal(node._pose['right'], before)


def test_clutch_engages_without_any_jump(node):
    """接管瞬间必须严格为零位移——这是不限速的上肢唯一的防跳保护。"""
    before = node._pose['right'][:3].copy()
    node._update_arms(_frame(1.0, 0.0, [0.5, 0.8, -0.3]))
    assert np.array_equal(node._pose['right'][:3], before)
    # 松开再按一次，仍然不能跳（此时手柄已经离原点很远）。
    node._update_arms(_frame(1.0, 0.0, [0.9, 1.2, -0.9]))
    node._update_arms(_frame(0.0, 0.0, [0.9, 1.2, -0.9]))
    frozen = node._pose['right'][:3].copy()
    node._update_arms(_frame(1.0, 0.0, [-0.5, 0.2, 0.5]))
    assert np.array_equal(node._pose['right'][:3], frozen)


def test_controller_displacement_maps_to_robot_axes(node):
    """WebXR 是 Y 上、-Z 前；机器人是 X 前、Y 左。"""
    before = node._pose['right'][:3].copy()
    node._update_arms(_frame(1.0, 0.0, [0.5, 0.8, -0.3]))
    node._update_arms(_frame(1.0, 0.0, [0.55, 0.9, -0.4]))   # 右 5、上 10、前 10 cm
    delta = node._pose['right'][:3] - before
    assert np.allclose(delta, [0.10, -0.05, 0.10])


def test_release_freezes_the_arm(node):
    node._update_arms(_frame(1.0, 0.0, [0.5, 0.8, -0.3]))
    node._update_arms(_frame(1.0, 0.0, [0.5, 0.8, -0.4]))
    node._update_arms(_frame(0.0, 0.0, [0.5, 0.8, -0.4]))
    frozen = node._pose['right'].copy()
    node._update_arms(_frame(0.0, 0.0, [-0.9, 1.9, 0.9]))
    assert np.array_equal(node._pose['right'], frozen)


# --------------------------------------------------------------------- 姿态与标定

def test_orientation_follows_the_controller(node):
    """手转了多少度，末端就转多少度（轴系不同，转角必须相等）。"""
    node._update_arms(_frame(1.0, 0.0, [0.5, 0.8, -0.3], UNIT))
    before = _matrix(node._pose['right'][3:])
    for axis, angle in (([0, 1, 0], 0.5), ([1, 0, 0], 0.4), ([0, 0, 1], 0.7)):
        node._update_arms(_frame(1.0, 0.0, [0.5, 0.8, -0.3], _spin(axis, angle)))
        turn = _matrix(node._pose['right'][3:]) @ before.T
        assert _angle_of(turn) == pytest.approx(angle, abs=1e-6)


def test_clutch_engage_leaves_orientation_untouched(node):
    """接合那一帧手柄已经是歪的，但末端姿态不能动——相对量从 0 起算。"""
    before = node._pose['right'].copy()
    node._update_arms(_frame(1.0, 0.0, [0.5, 0.8, -0.3], _spin([0.3, 0.5, -0.8], 1.1)))
    assert np.allclose(node._pose['right'], before)


def test_quaternion_stays_unit(node):
    node._update_arms(_frame(1.0, 0.0, [0.5, 0.8, -0.3], UNIT))
    node._update_arms(_frame(1.0, 0.0, [0.6, 0.9, -0.5], _spin([1, 2, 3], 2.0)))
    for side in ('left', 'right'):
        assert np.linalg.norm(node._pose[side][3:]) == pytest.approx(1.0)


def test_calibration_redefines_forward(node):
    """手柄指哪儿，那儿就是机器人正前方。"""
    node._calibrate('right', UNIT)                     # 指参考空间 -Z，即默认前方
    assert np.allclose(node._map, _BASE_MAP)
    for angle in (np.pi / 2, -np.pi / 2, np.pi, 0.7):
        node._calibrate('right', _spin([0, 1, 0], angle))
        pointing = _matrix(_spin([0, 1, 0], angle)) @ np.array([0.0, 0.0, -1.0])
        assert np.allclose(node._map @ pointing, [1.0, 0.0, 0.0], atol=1e-9)
        assert np.allclose(node._map @ node._map.T, np.eye(3))


def test_calibration_keeps_up_as_up(node):
    """只取航向：不管怎么标定，手往上抬永远是末端往上。"""
    for angle in (0.0, 1.2, -2.5):
        node._calibrate('right', _spin([0, 1, 0], angle))
        assert np.allclose(node._map @ np.array([0.0, 1.0, 0.0]), [0.0, 0.0, 1.0])


def test_calibration_drops_engaged_clutches(node):
    """换了映射还用旧原点算位移，方向会瞬间变——必须强制重新接合。"""
    node._update_arms(_frame(1.0, 0.0, [0.5, 0.8, -0.3]))
    assert node._clutch['right'] is not None
    node._calibrate('right', _spin([0, 1, 0], 0.6))
    assert all(node._clutch[side] is None for side in ('left', 'right'))


def test_calibration_ignores_a_vertical_controller(node):
    node._calibrate('right', _spin([0, 1, 0], 0.6))
    expected = node._map.copy()
    node._calibrate('right', _spin([1, 0, 0], np.pi / 2))   # 手柄指向正上/正下
    assert np.allclose(node._map, expected)                  # 标定被忽略


def test_calibration_button_needs_a_rising_edge(node):
    calls = []
    node._calibrate = lambda side, orientation: calls.append(side)
    pressed = {'left': {'buttons': {'a_x': True}, 'grip': {'orientation': UNIT}},
               'right': {'buttons': {'a_x': False}, 'grip': {'orientation': UNIT}}}
    released = {'left': {'buttons': {'a_x': False}}, 'right': {'buttons': {'a_x': False}}}
    node._check_calibration(pressed)          # 初始是“按着”，不算
    assert calls == []
    node._check_calibration(released)
    node._check_calibration(pressed)          # 松手后再按才算
    assert calls == ['left']
    node._check_calibration(pressed)          # 按住不重复
    assert calls == ['left']


def test_trigger_maps_to_gripper_the_right_way_round(node):
    """trigger 0 = 完全打开、1 = 夹紧；而 eccentric 是 0 闭合、2.76 打开，所以是反的。"""
    node._update_arms(_frame(0.0, 0.0, [0, 0, 0]))
    assert node._grip[0] == pytest.approx(OPEN)
    node._update_arms(_frame(0.0, 1.0, [0, 0, 0]))
    assert node._grip[0] == pytest.approx(CLOSED)
    node._update_arms(_frame(0.0, 0.5, [0, 0, 0]))
    assert node._grip[0] == pytest.approx(OPEN / 2.0)
    # 越界的 trigger 不能把夹爪顶出行程。
    node._update_arms(_frame(0.0, 1.7, [0, 0, 0]))
    assert node._grip[0] == pytest.approx(CLOSED)


# --------------------------------------------------------------------- 摇杆

@pytest.fixture
def sticks(node):
    node._vx_max, node._vy_max, node._wz_max = 0.5, 0.4, 1.5
    node._deadzone = 0.08
    node._rate = 50.0
    node._height_rate = 0.15
    node._h_lo, node._h_hi = 0.50, 0.78
    node._height = 0.70
    node._twist = np.zeros(3)
    return node


def _stick(left, right):
    return {'left': {'thumbstick': list(left)},
            'right': {'thumbstick': list(right)}}


def test_left_stick_drives_horizontal_velocity(sticks):
    """xr-standard 的 Y 轴下为正，机器人 X 前 / Y 左，所以推前是 -1、推右也是取负。"""
    sticks._update_command(_stick([0.0, -1.0], [0.0, 0.0]))     # 推前
    assert sticks._twist[0] == pytest.approx(0.5)
    assert sticks._twist[1] == pytest.approx(0.0)
    sticks._update_command(_stick([1.0, 0.0], [0.0, 0.0]))      # 推右
    assert sticks._twist[1] == pytest.approx(-0.4)              # 机器人 +y 是左
    sticks._update_command(_stick([0.0, 1.0], [0.0, 0.0]))      # 推后
    assert sticks._twist[0] == pytest.approx(-0.5)


def test_right_stick_drives_yaw(sticks):
    sticks._update_command(_stick([0.0, 0.0], [-1.0, 0.0]))     # 推左 = 左转
    assert sticks._twist[2] == pytest.approx(1.5)
    sticks._update_command(_stick([0.0, 0.0], [1.0, 0.0]))
    assert sticks._twist[2] == pytest.approx(-1.5)


def test_height_is_absolute_and_clamped(sticks):
    """高度和键盘的 I/K 一样是绝对量：推着才变，松手停住，不回弹。"""
    start = sticks._height
    for _ in range(10):                                          # 推上 10 帧
        sticks._update_command(_stick([0.0, 0.0], [0.0, -1.0]))
    assert sticks._height == pytest.approx(start + 10 * 0.15 / 50.0)
    held = sticks._height
    for _ in range(10):                                          # 松手
        sticks._update_command(_stick([0.0, 0.0], [0.0, 0.0]))
    assert sticks._height == pytest.approx(held)                 # 不回弹
    for _ in range(1000):                                        # 一直推上
        sticks._update_command(_stick([0.0, 0.0], [0.0, -1.0]))
    assert sticks._height == pytest.approx(sticks._h_hi)         # 卡在上界
    for _ in range(1000):
        sticks._update_command(_stick([0.0, 0.0], [0.0, 1.0]))
    assert sticks._height == pytest.approx(sticks._h_lo)


def test_deadzone_rejects_drift_without_a_step(sticks):
    sticks._update_command(_stick([0.05, 0.05], [0.05, 0.05]))   # 模长 < 死区
    assert np.array_equal(sticks._twist, np.zeros(3))
    # 刚出死区时速度必须从 0 连续长起来，不能阶跃。
    sticks._update_command(_stick([0.0, -0.09], [0.0, 0.0]))
    assert 0.0 < sticks._twist[0] < 0.02


def test_missing_thumbstick_is_treated_as_centred(sticks):
    sticks._update_command({'left': {}, 'right': {}})
    assert np.array_equal(sticks._twist, np.zeros(3))
    sticks._update_command({})
    assert np.array_equal(sticks._twist, np.zeros(3))


# --------------------------------------------------------------------- B/Y 状态机

@pytest.fixture
def buttons(node):
    """把 _advance 换成计数器，只测边沿检测，不碰 ROS 服务。"""
    node._button_cooldown = 0.0
    node._button_stamp = 0.0
    node._button_held = True          # 与真实初值一致：连上就按着不算一次
    node._now = lambda: next(clock)
    node._advance = lambda: fired.append(1)
    fired: list[int] = []
    clock = iter(range(1, 10_000))
    node.fired = fired
    return node


def _by(pressed_left, pressed_right):
    return {'left': {'buttons': {'b_y': pressed_left}},
            'right': {'buttons': {'b_y': pressed_right}}}


def test_both_hands_required(buttons):
    buttons._check_button(_by(False, False))    # 先松手，解除初始的"按住"
    buttons._check_button(_by(True, False))
    buttons._check_button(_by(False, True))
    assert buttons.fired == []
    buttons._check_button(_by(True, True))
    assert len(buttons.fired) == 1


def test_holding_does_not_repeat(buttons):
    buttons._check_button(_by(False, False))
    for _ in range(20):
        buttons._check_button(_by(True, True))
    assert len(buttons.fired) == 1               # 按住只算一次
    buttons._check_button(_by(False, False))
    buttons._check_button(_by(True, True))
    assert len(buttons.fired) == 2               # 松开再按才是第二次


def test_stale_frames_cannot_fake_a_press(buttons):
    """一次掉帧不能凭空推进状态机——恢复后必须真的松手再按。"""
    buttons._check_button(_by(False, False))
    buttons._check_button(_by(True, True))
    assert len(buttons.fired) == 1
    buttons._check_button(None)                  # 帧不新鲜
    buttons._check_button(_by(True, True))       # 恢复时仍按着
    assert len(buttons.fired) == 1
    buttons._check_button(_by(False, False))
    buttons._check_button(_by(True, True))
    assert len(buttons.fired) == 2


def test_advance_table_covers_the_whole_cycle():
    """idle/estop -> engage、stand -> start、running -> estop，绕回来能重开。"""
    from g1_motion_control.vr_teleop import _ADVANCE
    assert _ADVANCE['idle'][0] == 'engage'
    assert _ADVANCE['stand'][0] == 'start'
    assert _ADVANCE['running'][0] == 'estop'
    assert _ADVANCE['estop'][0] == 'engage'
