"""Python 与 C++ 两侧补偿实现的对拍。

fixture 就是运行时真正加载的 YAML 格式，两边各自读同一份文件算同一批样本，
任何一侧改了公式或坐标约定都会在这里断掉。
"""

from pathlib import Path

import numpy as np
import yaml

from arm_gravity_compensation.ft_model import (
    FtCalibration, PayloadEstimator, net_wrench, tool_wrench)


FIXTURES = (Path(__file__).parents[2] / "unitree_g1_ros2_control" / "test")


def _load(name):
    with open(FIXTURES / name, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _calibration():
    document = _load("ft_calibration_fixture.yaml")
    return FtCalibration.from_dict(
        document["ft_wrench_compensator"]["ros__parameters"]["left"])


def test_shared_fixture_still_matches_this_implementation():
    calibration = _calibration()

    for case in _load("ft_cases_fixture.yaml")["cases"]:
        np.testing.assert_allclose(
            net_wrench(case["raw"], calibration, case["gravity"]),
            case["net"], atol=1e-12)


def test_fixture_cases_carry_a_real_payload_and_a_rotated_sensor():
    calibration = _calibration()
    cases = _load("ft_cases_fixture.yaml")["cases"]

    assert calibration.polarity == -1.0
    assert not np.allclose(calibration.rotation, np.eye(3))
    # 取矩点不在 link 原点上，否则这批数据测不到那一段搬移。
    assert np.linalg.norm(calibration.origin) > 0.01
    assert len(cases) >= 4
    for case in cases:
        # 净力必须是一个纯重力负载：方向与重力平行、量级合理。
        force = np.asarray(case["net"][:3], dtype=float)
        gravity = np.asarray(case["gravity"], dtype=float)
        mass = force @ gravity / (gravity @ gravity)
        assert 0.3 < mass < 1.2
        np.testing.assert_allclose(force, mass * gravity, atol=1e-9)


def test_fixture_net_torque_places_the_payload_beyond_the_torque_point():
    """净力矩已经搬到 link 原点，所以反解出的质心必须越过取矩点。"""
    calibration = _calibration()
    estimator = PayloadEstimator()
    for case in _load("ft_cases_fixture.yaml")["cases"]:
        estimator.add(case["gravity"], case["net"])
    estimate = estimator.estimate()

    assert estimate.observability > 0.1
    assert estimate.com[2] > calibration.origin[2] + 0.05


def test_tool_wrench_is_the_first_moment_crossed_with_gravity():
    calibration = _calibration()
    gravity = np.array([0.0, 0.0, -9.81])

    wrench = tool_wrench(calibration, gravity)

    np.testing.assert_allclose(wrench[:3], calibration.mass * gravity)
    np.testing.assert_allclose(
        wrench[3:], np.cross(calibration.first_moment, gravity))
