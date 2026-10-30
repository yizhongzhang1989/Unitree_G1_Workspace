from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import yaml

from arm_gravity_compensation.auto_ft_calibration import AutoFtCalibration, check_coverage, interpolate, load_poses, stable
from arm_gravity_compensation.constants import ALL_ARM_JOINTS, ARM_JOINTS, mirror_arm_values
from arm_gravity_compensation.ft_model import FtCalibration, FtSample, expected_raw


def test_mirror_interpolation_and_coverage():
    pose = [0.1] * 7
    config = dict(transition_s=1.5, minimum_wait_s=0.5, left_joints=list(ARM_JOINTS["left"]), left_positions=[pose] * 4)
    limits = {name: {"limit": {"lower": -1, "upper": 1}} for name in ALL_ARM_JOINTS}
    targets = load_poses(config, limits)
    np.testing.assert_allclose(targets[0], pose + list(mirror_arm_values(pose)))
    np.testing.assert_allclose(interpolate(np.zeros(14), targets[0], 0.5), targets[0] / 2)
    np.testing.assert_allclose(interpolate(np.zeros(14), targets[0], 1.0), targets[0])
    check_coverage(np.array([[1, 1, 1], [1, -1, -1], [-1, 1, -1], [-1, -1, 1]]) * 5.0)
    with pytest.raises(ValueError):
        check_coverage([[0, 0, -9.81]] * 4)
    limits[ALL_ARM_JOINTS[0]]["limit"]["upper"] = 0
    with pytest.raises(ValueError):
        load_poses(config, limits)


def test_stability_requires_both_arms_and_fresh_data():
    rows = [(stamp, np.zeros(14), np.zeros(14)) for stamp in np.linspace(0, 1, 101)]
    assert stable(rows, 1, 1)
    assert not stable(rows, 2, 1)
    rows[-1][1][13] = 0.02
    assert not stable(rows, 1, 1)


def test_stale_stream_names_keep_half_second_threshold(monkeypatch):
    from arm_gravity_compensation import auto_ft_calibration as module
    monkeypatch.setattr(module.time, "monotonic", lambda: 10.0)
    node = AutoFtCalibration.__new__(AutoFtCalibration)
    node.stamps = {"joints": 9.99, "left": 9.5, "right": 9.501,
                   "left_gravity": 9.4, "right_gravity": 9.99}
    assert AutoFtCalibration.stale_streams(node) == pytest.approx({"left": 0.5, "left_gravity": 0.6})


def test_tick_keeps_spin_timeout_constant(monkeypatch):
    from arm_gravity_compensation import auto_ft_calibration as module
    clock, timeouts = [10.0], []
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])

    def spin(node, timeout_sec):
        timeouts.append(timeout_sec)
        clock[0] += 0.002

    monkeypatch.setattr(module.rclpy, "spin_once", spin)
    node = AutoFtCalibration.__new__(AutoFtCalibration)
    node.stamps = {name: 10.0 for name in ("joints", "left", "right", "left_gravity", "right_gravity")}
    node.last_publish = 10.0
    node.output = Mock()
    node.target = np.zeros(14)
    AutoFtCalibration.tick(node)
    assert len(timeouts) >= 5 and set(timeouts) == {0.001}
    node.output.publish.assert_called_once()
    node.output.reset_mock()
    node.mask = 0x1FFF8000
    node.target = np.arange(31, dtype=float)
    AutoFtCalibration.tick(node)
    published = node.output.publish.call_args.args[0]
    assert published.mask == 0x1FFF8000
    np.testing.assert_array_equal(published.positions, np.arange(31, dtype=float))


@pytest.fixture
def save_node(tmp_path):
    path, parameters = tmp_path / "ft.yaml", tmp_path / "parameters.json"
    node = AutoFtCalibration.__new__(AutoFtCalibration)
    node.calibration_path = path
    node.parameter_path = parameters
    node.records = []
    node.call = Mock(return_value=SimpleNamespace(success=True))
    return node


def _save_inputs(node, tool, residual=0.0):
    old = yaml.safe_dump({"ft_wrench_compensator": {"ros__parameters": {side: tool.to_dict() for side in ARM_JOINTS}}})
    node.calibration_path.write_text(old)
    node.parameter_path.write_text("old")
    gravity = np.array([[1, 1, 1], [1, -1, -1], [-1, 1, -1], [-1, -1, 1]]) * (9.81 / np.sqrt(3))
    samples = {side: [FtSample(vector, expected_raw(tool, vector) + residual) for vector in gravity] for side in ARM_JOINTS}
    return old, samples


@pytest.mark.parametrize("bad_side", [None, "right"])
def test_save_only_after_both_solutions_succeed(tmp_path, save_node, bad_side):
    tool = FtCalibration(mass=0.7, origin=np.asarray([0, 0, 0.053], dtype=float))
    node = save_node
    old, samples = _save_inputs(node, tool)
    path, parameters = node.calibration_path, node.parameter_path
    if bad_side:
        samples[bad_side] = samples[bad_side][:3]
        with pytest.raises(ValueError):
            AutoFtCalibration.save(node, {"ft_sensor": {}}, samples)
        assert path.read_text() == old and parameters.read_text() == "old"
        node.call.assert_not_called()
    else:
        AutoFtCalibration.save(node, {"ft_sensor": {}}, samples)
        assert node.call.call_count == 2
    assert set(tmp_path.iterdir()) == {path, parameters}


@pytest.mark.parametrize("residual", [0.0, 2.0])
def test_bias_mass_and_residual_are_not_extra_save_limits(save_node, capsys, monkeypatch, residual):
    from arm_gravity_compensation import auto_ft_calibration as module
    tool = FtCalibration(mass=0.005, force_bias=np.asarray([100.0, -200.0, 300.0], dtype=float), origin=np.asarray([0, 0, 0.053], dtype=float))
    node = save_node
    _, samples = _save_inputs(node, tool, residual)
    if residual:
        monkeypatch.setattr(module, "solve_ft_calibration", lambda *args, **kwargs: SimpleNamespace(calibration=tool, diagnostics={}))
    AutoFtCalibration.save(node, {"ft_sensor": {}}, samples)
    output = capsys.readouterr().out
    assert "left:" in output and "right:" in output and "force RMS=" in output and "torque RMS=" in output
    assert "(min" not in output and "(max" not in output
    saved = yaml.safe_load(node.calibration_path.read_text())["ft_wrench_compensator"]["ros__parameters"]["left"]
    assert saved["tool_mass"] == pytest.approx(tool.mass)
    np.testing.assert_allclose(saved["force_bias"], tool.force_bias)
    assert node.call.call_count == 2
