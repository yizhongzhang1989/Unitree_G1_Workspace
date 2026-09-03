"""手柄推状态机的规则测试。

这些规则是 vr_teleop 踩出来的，改动时最容易悄悄破坏——尤其「松手才走」和「按下即走」
的分工：都按下即走的话，从站立长按会先把策略拉起来、再急停，中间那一秒机器人已经在跑了。
不起 ROS，直接驱动 MocapTeleop 的按键逻辑。
"""
import sys
import types

import pytest

sys.path.insert(0, '/workspace/src/g1_rgmt_tracking_global')

class Fake:
    """只保留按键状态机需要的那几个字段，不碰 rclpy。"""

    from g1_rgmt_tracking_global.mocap_teleop import MocapTeleop
    _tick = MocapTeleop._tick
    _advance = MocapTeleop._advance
    _next_step = MocapTeleop._next_step
    _report = MocapTeleop._report

    def __init__(self, state='idle', *, estop_hold=1.0, cooldown=1.0, timeout=0.3):
        self.t = 100.0
        self._state, self._pressed = state, False
        self._engaged = False
        self._held, self._down_at, self._used = True, None, True
        self._stamp, self._last_input = 0.0, 100.0
        self._estop_hold, self._cooldown, self._timeout = estop_hold, cooldown, timeout
        self.calls = []

    def _now(self):
        return self.t

    def _call(self, name, label):
        self.calls.append(name)

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *_a, **_k: None,
                                     warning=lambda *_a, **_k: None,
                                     error=lambda *_a, **_k: None)

    def step(self, pressed, dt=0.02):
        self.t += dt
        self._last_input = self.t
        self._pressed = pressed
        self._tick()

    def arm(self):
        """先松一下手。初始状态是「按着」，不先松手的话接下来的按下不算一次。"""
        self.step(False)
        return self


def test_engage_fires_on_release_not_on_press():
    """站立那步必须松手才走：按下就推进的话，长按会先拉起策略再急停。"""
    node = Fake('idle').arm()
    for _ in range(10):
        node.step(True)
    assert node.calls == [], '按住期间不能推进'
    node.step(False)
    assert node.calls == ['engage']


def test_second_press_starts_after_engage_succeeds_while_state_stays_idle():
    """RGMT 的 engage 成功后仍上报 idle；下一次短按必须 start，不能重复 engage。"""
    node = Fake('idle', cooldown=0.0).arm()
    node.step(True)
    node.step(False)
    result = types.SimpleNamespace(success=True, message='已激活')
    node._report('engage', '激活控制器', types.SimpleNamespace(result=lambda: result))
    node.step(True)
    node.step(False)
    assert node.calls == ['engage', 'start']


def test_estop_fires_on_press():
    """running 那步本来就是急停，按下即走——多等一次松手是多余的风险。"""
    node = Fake('running').arm()
    node.step(True)
    assert node.calls == ['estop']


def test_long_press_estops_regardless_of_state():
    node = Fake('idle', estop_hold=0.5).arm()
    for _ in range(40):            # 0.8 s > estop_hold
        node.step(True)
    assert node.calls == ['estop'], '长按要越过状态直接急停，而不是先站立'


def test_long_press_consumes_the_release():
    """长按急停之后松手不能再补一次短按。"""
    node = Fake('idle', estop_hold=0.5, cooldown=0.0).arm()
    for _ in range(40):
        node.step(True)
    node.step(False)
    assert node.calls == ['estop']


def test_starts_held_so_a_press_already_down_does_not_count():
    """一上来当作按着：连上瞬间手柄恰好按着，不能凭空推一步。"""
    node = Fake('idle')
    node.step(True)                # 第一次就是「按着」
    node.step(False)               # 松手——但这一次不算
    assert node.calls == []
    node.step(True)
    node.step(False)
    assert node.calls == ['engage']


def test_input_dropout_invalidates_the_hold():
    """按键流断掉时长按计时作废，恢复后必须真松手再按，否则一次掉帧就能凭空推一步。"""
    node = Fake('idle', estop_hold=0.5).arm()
    for _ in range(10):
        node.step(True)
    node.t += 1.0                  # 断流：_last_input 停在过去
    node._tick()
    assert node.calls == []
    node.step(True)                # 恢复后仍按着 -> 不算新的一次按下
    node.step(False)
    assert node.calls == []


def test_cooldown_blocks_a_second_advance():
    node = Fake('idle', cooldown=1.0).arm()
    node.step(True)
    node.step(False)
    assert node.calls == ['engage']
    node.step(True)
    node.step(False)
    assert node.calls == ['engage'], '冷却期内不能连推两步'


def test_unknown_state_does_not_call_anything():
    node = Fake('').arm()
    node.step(True)
    node.step(False)
    assert node.calls == []


@pytest.mark.parametrize('state,engaged,service', [
    ('idle', False, 'engage'), ('idle', True, 'start'),
    ('estop', False, 'engage'), ('stand', True, 'estop'), ('running', True, 'estop')])
def test_next_step_matches_rgmt_state_machine(state, engaged, service):
    node = Fake(state)
    node._engaged = engaged
    assert node._next_step()[0] == service
