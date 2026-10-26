import numpy as np
import pytest

from g1_motion_session.policy_support import two_stage


def test_stand_clearance_holds_start_then_interpolates():
    start, goal = np.zeros(15), np.ones(15)
    assert two_stage(.2, .4, 2.4, start, start, goal) == pytest.approx(start)
    assert two_stage(1.4, .4, 2.4, start, start, goal) == pytest.approx(np.full(15, .5))


def test_stand_holds_goal_after_transition():
    start, goal = np.zeros(15), np.ones(15)
    np.testing.assert_array_equal(two_stage(3., .4, 2.4, start, start, goal), goal)
