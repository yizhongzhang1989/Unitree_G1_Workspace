"""vr_teleop 的离线校验。不需要真机，也不需要 VR 头显。

只测和安全直接相关的两件事：离合接合的瞬间**不能有位置跳变**（上肢不限速，跳一下
就是电机全权限的弹动），以及 trigger 到夹爪的映射方向没写反（写反 = 该松手时夹紧）。

节点用 ``__new__`` 造出来、只填这几个方法用到的字段：这里要测的是纯逻辑，不想拉起
ROS 图、也不想连 VR 桥。
"""

import numpy as np
import pytest

from g1_motion_control.vr_teleop import (
    VRTeleop,
    _axis_map,
    _level,
    _matrix,
)

OPEN = 2.76377472169236       # eccentric 全开
CLOSED = 0.0                  # eccentric 闭合
UNIT = (0.0, 0.0, 0.0, 1.0)   # 单位四元数 xyzw
# 参考空间（local-floor）里的“前/上/右”，分别落到躯干 +X / +Z / -Y。
GRIP_FWD = np.array([0.0, 0.0, -1.0])
GRIP_UP = np.array([0.0, 1.0, 0.0])
GRIP_RIGHT = np.array([1.0, 0.0, 0.0])
# 采集页画的那根 15 cm 朝向线（vr/index.html 里的 (0,0,-0.15)），也就是“滚手腕”的轴。
GRIP_LINE = np.array([0.0, 0.0, -1.0])
# gripper_base 的伸出/指向轴（URDF 里 +Z），线应该落到它上面。
EE_POINT = [0.0, 0.0, 1.0]


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


def _arms(node, squeeze, trigger, position, orientation=UNIT, limited=None):
    """跑一帧上肢逻辑；``limited`` 是策略层限速后的可达末端指令。"""
    node._update_arms(_frame(squeeze, trigger, position, orientation), limited or {})


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
    teleop._squeeze_off = 0.4
    teleop._arm_scale = 1.0
    teleop._grip_open, teleop._grip_closed = OPEN, CLOSED
    teleop._clutch = {'left': None, 'right': None}
    teleop._pose = {'left': np.array([0.30, 0.15, 0.05, 0.0, 0.0, 0.0, 1.0]),
                    'right': np.array([0.30, -0.15, 0.05, 0.0, 0.0, 0.0, 1.0])}
    teleop._grip = np.zeros(2)
    teleop.get_logger = lambda: _Silent()
    return teleop


@pytest.mark.parametrize('spec', ['+x +y', 'x y w', '+x +x +z', '+x -y +z', 'x y z t'])
def test_axis_map_rejects_anything_that_is_not_a_rotation(spec):
    """不是 det=+1 的轴排列就带镜像，共轭出来的转动会反向——宁可启动就报错。"""
    with pytest.raises(ValueError):
        _axis_map(spec)


def test_axis_map_reads_the_spec_column_by_column():
    """三个记号依次是输入系 X / Y / Z 的去向。"""
    matrix = _axis_map('+z +x +y')
    assert np.allclose(matrix @ np.array([1.0, 0.0, 0.0]), [0.0, 0.0, 1.0])
    assert np.allclose(matrix @ np.array([0.0, 1.0, 0.0]), [1.0, 0.0, 0.0])
    assert np.allclose(matrix @ np.array([0.0, 0.0, 1.0]), [0.0, 1.0, 0.0])


def test_below_threshold_does_not_move_the_arm(node):
    before = node._pose['right'].copy()
    _arms(node, 0.2, 0.0, [0.5, 0.8, -0.3])
    assert np.array_equal(node._pose['right'], before)


def test_clutch_engages_without_any_jump(node):
    """接管瞬间必须严格为零位移——这是不限速的上肢唯一的防跳保护。"""
    before = node._pose['right'][:3].copy()
    _arms(node, 1.0, 0.0, [0.5, 0.8, -0.3])
    assert np.array_equal(node._pose['right'][:3], before)
    # 松开再按一次，仍然不能跳（此时手柄已经离原点很远）。
    _arms(node, 1.0, 0.0, [0.9, 1.2, -0.9])
    _arms(node, 0.0, 0.0, [0.9, 1.2, -0.9])
    frozen = node._pose['right'][:3].copy()
    _arms(node, 1.0, 0.0, [-0.5, 0.2, 0.5])
    assert np.array_equal(node._pose['right'][:3], frozen)


def test_controller_displacement_maps_to_robot_axes(node):
    """grip 系的前/上/右（见文件头常量）分别映到机器人 +X / +Z / -Y。"""
    before = node._pose['right'][:3].copy()
    start = np.array([0.5, 0.8, -0.3])
    step = GRIP_FWD * 0.05 + GRIP_UP * 0.05 + GRIP_RIGHT * 0.025
    _arms(node, 1.0, 0.0, start)
    _arms(node, 1.0, 0.0, start + step)
    _arms(node, 1.0, 0.0, start + step * 2)                 # 两帧走前 10、上 10、右 5 cm
    delta = node._pose['right'][:3] - before
    assert np.allclose(delta, [0.10, -0.05, 0.10])          # 末端：前 10、右 5、上 10 cm


def test_translation_is_measured_in_the_controller_frame(node):
    """位移只脱掉手柄的**偏航**：操作员面朝哪、参考空间朝哪都不影响，不需要方向标定。

    同一个"沿手柄指向推 5 cm"的动作，手柄先后指三个不同航向，末端必须走同一个
    方向；旧的世界系实现下三次会各走各的。
    """
    want = None
    for angle in (0.0, np.pi, np.pi / 2):
        node._clutch['right'] = None
        node._pose['right'][:3] = [0.30, -0.15, 0.05]
        grip = _spin(GRIP_UP, angle)                           # 绕竖直轴换三个握法
        forward = _matrix(grip) @ GRIP_FWD                     # 手柄指向在参考空间里的方向
        start = np.array([0.5, 0.8, -0.3])
        _arms(node, 1.0, 0.0, start, grip)                     # 接合
        before = node._pose['right'][:3].copy()
        _arms(node, 1.0, 0.0, start + forward * 0.05, grip)    # 沿指向推 5 cm
        delta = node._pose['right'][:3] - before
        if want is None:
            want = delta
        assert np.allclose(delta, want)
    assert np.allclose(want, [0.05, 0.0, 0.0])                 # 末端往机器人正前方


@pytest.mark.parametrize('tilt', [
    _spin([0.0, 0.0, -1.0], 0.6),                              # 绕朝向线滚手腕
    _spin([1.0, 0.0, 0.0], 0.5),                               # 抬手柄头（俯仰）
])
def test_a_tilted_hold_does_not_tilt_the_translation(node, tilt):
    """握歪只该改朝向，不该改上下。

    ``local-floor`` 是重力对齐的，竖直方向本来就准，没有任何理由让它跟着手腕转。
    早先整个 ``origin_rot.T`` 都脱掉，滚手腕 0.6 rad 再直上抬 5 cm，末端会走成
    右 2.8 上 4.1——这就是"平移怪怪的"。
    """
    node._clutch['right'] = None
    start = np.array([0.5, 0.8, -0.3])
    _arms(node, 1.0, 0.0, start, tilt)                         # 歪着接合
    before = node._pose['right'][:3].copy()
    _arms(node, 1.0, 0.0, start + np.array([0.0, 0.05, 0.0]), tilt)
    assert np.allclose(node._pose['right'][:3] - before, [0.0, 0.0, 0.05])


def test_level_gives_up_when_the_controller_points_straight_up():
    """手柄指天时水平朝向无从谈起，只能让调用方退回上一次的基准。"""
    assert _level(_matrix(_spin([1.0, 0.0, 0.0], np.pi / 2))) is None


def test_release_freezes_the_arm(node):
    _arms(node, 1.0, 0.0, [0.5, 0.8, -0.3])
    _arms(node, 1.0, 0.0, [0.5, 0.8, -0.4])
    _arms(node, 0.0, 0.0, [0.5, 0.8, -0.4])
    frozen = node._pose['right'].copy()
    _arms(node, 0.0, 0.0, [-0.9, 1.9, 0.9])
    assert np.array_equal(node._pose['right'], frozen)


# --------------------------------------------------------------------- 离合锚点

def test_clutch_anchors_on_the_limited_pose(node):
    """锚点取 IK + 关节限速后的可达末端指令，不取飞出可达域的笛卡尔目标。

    目标够不着时两者差几十厘米，锚在目标上会让“松开-挪手-再按”的接力
    把目标一路累积出可达域（实测 6 轮 ×10 cm 前伸后目标 588 mm / 实际 171 mm）。
    """
    node._pose['right'] = np.array([0.90, -0.15, 0.05, 0.0, 0.0, 0.0, 1.0])  # 够不着
    limited = {'right': [0.30, -0.15, 0.05, 0.0, 0.0, 0.0, 1.0]}
    start = np.array([0.5, 0.8, -0.3])
    _arms(node, 1.0, 0.0, start, limited=limited)
    assert np.allclose(node._pose['right'], limited['right'])
    # 接合后的位移从限速后的位置起算，不是从那个飞出去的目标起算。
    _arms(node, 1.0, 0.0, start + GRIP_FWD * 0.05, limited=limited)
    _arms(node, 1.0, 0.0, start + GRIP_FWD * 0.10, limited=limited)   # 两帧沿指向推 10 cm
    assert node._pose['right'][0] == pytest.approx(0.40)
    # 另一侧 status 里没给，退回上一帧目标，同样往前 10 cm，且仍然零跳变。
    assert np.allclose(node._pose['left'][:3], [0.40, 0.15, 0.05])


def test_clutch_falls_back_when_limited_pose_is_missing(node):
    """status 里没给这一侧时退回上一帧目标，不能抛异常、也不能跳。"""
    before = node._pose['right'].copy()
    _arms(node, 1.0, 0.0, [0.5, 0.8, -0.3], limited={})
    assert np.array_equal(node._pose['right'], before)


def test_squeeze_hysteresis_keeps_an_engaged_clutch(node):
    """接合后 squeeze 从 0.5 降到 0.45 不能断开；低于 0.4 才释放。"""
    _arms(node, 0.6, 0.0, [0.5, 0.8, -0.3])
    _arms(node, 0.45, 0.0, [0.5, 0.8, -0.31])
    assert node._clutch['right'] is not None
    _arms(node, 0.39, 0.0, [0.5, 0.8, -0.31])
    assert node._clutch['right'] is None


def test_webxr_relocalization_does_not_jump_the_target(node):
    """通信正常但 WebXR 单帧重定位 20 cm：重置原点，末端目标必须逐位不动。"""
    _arms(node, 1.0, 0.0, [0.5, 0.8, -0.3])
    _arms(node, 1.0, 0.0, [0.5, 0.8, -0.35])  # 正常前移 5 cm
    before = {side: pose.copy() for side, pose in node._pose.items()}
    _arms(node, 1.0, 0.0, [0.7, 0.8, -0.35])  # 单帧跳 20 cm
    for side in ('left', 'right'):
        assert np.allclose(node._pose[side], before[side])


def test_emulated_webxr_position_freezes_and_preserves_clutch(node):
    _arms(node, 1.0, 0.0, [0.5, 0.8, -0.3])
    before = {side: pose.copy() for side, pose in node._pose.items()}
    frame = _frame(1.0, 0.0, [0.55, 0.8, -0.3])
    for side in ('left', 'right'):
        frame[side]['grip']['emulated_position'] = True
    node._update_arms(frame, {})
    assert all(node._clutch[side] is not None for side in ('left', 'right'))
    for side in ('left', 'right'):
        assert np.array_equal(node._pose[side], before[side])


def test_flickering_emulated_flag_does_not_swallow_hand_motion(node):
    """emulated 逐帧抖时不能按比例丢运动。

    曾经在丢 tracking 时把 clutch 的 ``previous`` 作废，于是恢复帧无条件重锚、
    把跨这一帧的位移整段丢掉：1/2 抖动时手走 900 mm 目标走 **0 mm**（完全冻住），
    1/3 抖动只走 33%。现在只看与上一帧**真实跟踪到**的位置的差，小步照样积。
    """
    start = np.array([0.5, 0.8, -0.3])
    _arms(node, 1.0, 0.0, start)
    before = node._pose['right'][:3].copy()
    for step in range(60):
        frame = _frame(1.0, 0.0, start + GRIP_FWD * (0.003 * (step + 1)))
        for side in ('left', 'right'):
            frame[side]['grip']['emulated_position'] = step % 2 == 0
        node._update_arms(frame, {})
    assert node._pose['right'][0] - before[0] == pytest.approx(0.180)


def test_long_tracking_loss_still_reanchors_without_a_jump(node):
    """丢一整段后手已挪开很远：靠单帧位移阈值重锚，目标仍逐位不动。"""
    _arms(node, 1.0, 0.0, [0.5, 0.8, -0.3])
    frozen = {side: pose.copy() for side, pose in node._pose.items()}
    for _ in range(50):
        frame = _frame(1.0, 0.0, [0.5, 0.8, -0.3])
        for side in ('left', 'right'):
            frame[side]['grip']['emulated_position'] = True
        node._update_arms(frame, {})
    _arms(node, 1.0, 0.0, [0.5, 0.8, -0.6], limited={})     # 恢复时已差 30 cm
    for side in ('left', 'right'):
        assert np.array_equal(node._pose[side], frozen[side])


@pytest.mark.parametrize('missing', ['position', 'orientation'])
def test_incomplete_grip_frame_freezes_instead_of_raising(node, missing):
    _arms(node, 1.0, 0.0, [0.5, 0.8, -0.3])
    before = {side: pose.copy() for side, pose in node._pose.items()}
    frame = _frame(1.0, 0.0, [0.6, 0.8, -0.3])
    for side in ('left', 'right'):
        del frame[side]['grip'][missing]
    node._update_arms(frame, {})
    for side in ('left', 'right'):
        assert np.array_equal(node._pose[side], before[side])


@pytest.mark.parametrize('junk', [
    {'position': None, 'orientation': UNIT},
    {'position': ['a', 'b', 'c'], 'orientation': UNIT},
    {'position': [0.0, 0.0], 'orientation': UNIT},
    {'position': [0.0, float('nan'), 0.0], 'orientation': UNIT},
    {'position': [0.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0, 0.0]},
    {'position': [0.0, 0.0, 0.0], 'orientation': 'nope'},
])
def test_malformed_grip_never_raises(node, junk):
    """帧来自默认不鉴权的 WebSocket，任何垃圾都不能把 50 Hz 定时器回调带走。"""
    _arms(node, 1.0, 0.0, [0.5, 0.8, -0.3])
    before = {side: pose.copy() for side, pose in node._pose.items()}
    frame = _frame(1.0, 0.0, [0.6, 0.8, -0.3])
    for side in ('left', 'right'):
        frame[side]['grip'] = dict(junk)
    node._update_arms(frame, {})
    for side in ('left', 'right'):
        assert np.array_equal(node._pose[side], before[side])


def test_target_mapping_stays_absolute_while_the_clutch_is_engaged(node):
    """接合期间手↔手臂的映射必须是绝对的：手回到接合位置，目标就回到接合位姿。

    曾经把可达性修正量同步减进锚点，于是够不着的那段位移被吸进锚点、把整个映射
    平移了：实测（真 URDF + 真 IK）前推 80 cm 再把手原路收回接合点，手臂停在接合点
    **后方 383 mm**；推出后只缩 5 cm（人手仍够不着）手臂就已经后退 35 mm。
    """
    # limited_pose 卡在可达域边界不动，模拟手伸到了够不着的地方。
    limited = {'right': [0.35, -0.15, 0.05, 0.0, 0.0, 0.0, 1.0],
               'left': [0.35, 0.15, 0.05, 0.0, 0.0, 0.0, 1.0]}
    start = np.array([0.5, 0.8, -0.3])
    _arms(node, 1.0, 0.0, start, limited=limited)
    engaged = {side: pose.copy() for side, pose in node._pose.items()}
    for step in range(200):                              # 手沿指向推 60 cm
        _arms(node, 1.0, 0.0, start + GRIP_FWD * (0.003 * (step + 1)), limited=limited)
    assert node._pose['right'][0] == pytest.approx(engaged['right'][0] + 0.60, abs=1e-9)
    for step in range(200):                              # 原路收回
        _arms(node, 1.0, 0.0, start + GRIP_FWD * (0.6 - 0.003 * (step + 1)), limited=limited)
    for side in ('left', 'right'):
        assert np.allclose(node._pose[side], engaged[side], atol=1e-9)


# --------------------------------------------------------------------- 姿态

def test_orientation_follows_the_controller(node):
    """手转了多少度，末端就转多少度（轴系不同，转角必须相等）。"""
    _arms(node, 1.0, 0.0, [0.5, 0.8, -0.3], UNIT)
    before = _matrix(node._pose['right'][3:])
    for axis, angle in (([0, 1, 0], 0.5), ([1, 0, 0], 0.4), ([0, 0, 1], 0.7)):
        _arms(node, 1.0, 0.0, [0.5, 0.8, -0.3], _spin(axis, angle))
        turn = _matrix(node._pose['right'][3:]) @ before.T
        assert _angle_of(turn) == pytest.approx(angle, abs=1e-6)


def test_clutch_engage_leaves_orientation_untouched(node):
    """接合那一帧手柄已经是歪的，但末端姿态不能动——相对量从 0 起算。"""
    before = node._pose['right'].copy()
    _arms(node, 1.0, 0.0, [0.5, 0.8, -0.3], _spin([0.3, 0.5, -0.8], 1.1))
    assert np.allclose(node._pose['right'], before)


def test_wrist_roll_turns_the_gripper_about_its_own_axis(node):
    """滚手腕就是滚夹爪：转动在手柄系算再右乘，与夹爪当前指向无关。

    曾经是在世界系算完左乘，那等于"绕机器人基座的轴转"——夹爪一旦偏开，同一个
    手腕动作就不再是绕夹爪自己的轴。这里用两个差别很大的起始朝向钉住这个区别：
    左乘实现下两次得到的局部转动不可能相同。
    """
    for anchor_spin in ([0, 0, 1], [0, 1, 0]):
        node._clutch['right'] = None
        node._pose['right'][3:] = _spin(anchor_spin, np.pi / 2)
        anchor = _matrix(node._pose['right'][3:])
        _arms(node, 1.0, 0.0, [0.5, 0.8, -0.3], UNIT)          # 接合
        _arms(node, 1.0, 0.0, [0.5, 0.8, -0.3], _spin(GRIP_LINE, 0.4))
        local = anchor.T @ _matrix(node._pose['right'][3:])
        # 手柄绕那根朝向线滚 0.4 -> 夹爪绕自己的伸出轴滚 0.4，两个起始朝向下逐位相同。
        assert np.allclose(local, _matrix(_spin(EE_POINT, 0.4)))


@pytest.mark.parametrize('grip_axis, ee_axis', [
    ([0.0, 0.0, -1.0], [0.0, 0.0, 1.0]),        # 朝向线 -> 伸出轴
    ([1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),        # 穿出手背 -> 张合轴反向
    ([0.0, 1.0, 0.0], [0.0, 1.0, 0.0]),         # 沿小臂 -> 侧向
])
def test_each_grip_rotation_axis_lands_on_its_gripper_axis(node, grip_axis, ee_axis):
    """三根轴的**转向**都钉住：符号错了就是"手顺时针夹爪逆时针"，实机上很难分辨是哪根。"""
    _arms(node, 1.0, 0.0, [0.5, 0.8, -0.3], UNIT)              # 接合
    _arms(node, 1.0, 0.0, [0.5, 0.8, -0.3], _spin(grip_axis, 0.4))
    assert np.allclose(_matrix(node._pose['right'][3:]), _matrix(_spin(ee_axis, 0.4)))


def test_quaternion_stays_unit(node):
    _arms(node, 1.0, 0.0, [0.5, 0.8, -0.3], UNIT)
    _arms(node, 1.0, 0.0, [0.6, 0.9, -0.5], _spin([1, 2, 3], 2.0))
    for side in ('left', 'right'):
        assert np.linalg.norm(node._pose[side][3:]) == pytest.approx(1.0)


def test_trigger_maps_to_gripper_the_right_way_round(node):
    """trigger 0 = 完全打开、1 = 夹紧；而 eccentric 是 0 闭合、2.76 打开，所以是反的。"""
    _arms(node, 0.0, 0.0, [0, 0, 0])
    assert node._grip[0] == pytest.approx(OPEN)
    _arms(node, 0.0, 1.0, [0, 0, 0])
    assert node._grip[0] == pytest.approx(CLOSED)
    _arms(node, 0.0, 0.5, [0, 0, 0])
    assert node._grip[0] == pytest.approx(OPEN / 2.0)
    # 越界的 trigger 不能把夹爪顶出行程。
    _arms(node, 0.0, 1.7, [0, 0, 0])
    assert node._grip[0] == pytest.approx(CLOSED)


def test_missing_hand_keeps_its_pose_and_gripper(node):
    node._grip[:] = [0.7, 1.1]
    pose = node._pose['left'].copy()
    frame = _frame(0.0, 0.0, [0.0, 0.0, 0.0])
    frame['left'] = None
    node._update_arms(frame, {})
    assert np.array_equal(node._pose['left'], pose)
    assert node._grip[0] == pytest.approx(0.7)


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
