import numpy as np
import pytest

from arm_gravity_compensation.ft_model import (
    FtCalibration,
    FtSample,
    GRAVITY_MAGNITUDE,
    PayloadEstimator,
    expected_raw,
    gravity_aligned,
    instantaneous_mass,
    net_wrench,
    orientation_coverage,
    rezero,
    solve_ft_calibration,
    suggest_measurement_origin,
    tool_wrench,
)


TRUTH = FtCalibration(
    force_bias=[12.4, -30.1, 7.7],
    torque_bias=[0.83, -1.24, 0.31],
    mass=0.62,
    com=[0.004, -0.011, 0.068],
)


def _axis_rotation(axis, angle):
    unit = np.asarray(axis, dtype=float)
    unit = unit / np.linalg.norm(unit)
    cross = np.array([[0.0, -unit[2], unit[1]],
                      [unit[2], 0.0, -unit[0]],
                      [-unit[1], unit[0], 0.0]])
    return (np.eye(3) + np.sin(angle) * cross +
            (1.0 - np.cos(angle)) * (cross @ cross))


def _orientations(count=8, seed=3):
    """Gravity directions spanning all three axes, as the operator would pose."""
    random = np.random.RandomState(seed)
    fixed = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
             (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0)]
    vectors = [np.asarray(value, dtype=float) for value in fixed[:count]]
    while len(vectors) < count:
        sample = random.normal(size=3)
        vectors.append(sample / np.linalg.norm(sample))
    return [GRAVITY_MAGNITUDE * vector for vector in vectors]


def _samples(calibration=TRUTH, count=8, seed=3, payload=None):
    samples = []
    for gravity in _orientations(count, seed):
        reading = expected_raw(calibration, gravity)
        if payload is not None:
            extra = tool_wrench(payload, gravity).reshape(2, 3)
            reading = reading + calibration.polarity * (
                extra @ calibration.rotation.T).reshape(6)
        samples.append(FtSample(gravity=gravity, wrench=reading))
    return samples


def test_solve_recovers_bias_mass_and_centre_of_mass():
    solution = solve_ft_calibration(_samples())

    result = solution.calibration
    np.testing.assert_allclose(result.force_bias, TRUTH.force_bias, atol=1e-9)
    np.testing.assert_allclose(result.torque_bias, TRUTH.torque_bias, atol=1e-9)
    assert result.mass == pytest.approx(TRUTH.mass, abs=1e-9)
    np.testing.assert_allclose(result.com, TRUTH.com, atol=1e-9)
    assert result.polarity == 1.0
    assert solution.diagnostics["force_residual_rms"] < 1e-9
    assert solution.diagnostics["orientation_estimated"] is False


def test_solve_recovers_the_vendor_sign_convention():
    truth = FtCalibration(
        force_bias=TRUTH.force_bias, torque_bias=TRUTH.torque_bias,
        mass=TRUTH.mass, com=TRUTH.com, polarity=-1.0)

    result = solve_ft_calibration(_samples(truth)).calibration

    assert result.polarity == -1.0
    assert result.mass == pytest.approx(truth.mass, abs=1e-9)
    np.testing.assert_allclose(result.com, truth.com, atol=1e-9)


def test_mounting_rotation_is_adopted_only_when_the_data_demands_it():
    rotation = _axis_rotation([0.2, -0.4, 0.9], 0.7)
    truth = FtCalibration(
        force_bias=TRUTH.force_bias, torque_bias=TRUTH.torque_bias,
        mass=TRUTH.mass, com=TRUTH.com, rotation=rotation)

    solution = solve_ft_calibration(_samples(truth))

    assert solution.diagnostics["orientation_estimated"] is True
    np.testing.assert_allclose(solution.calibration.rotation, rotation, atol=1e-9)
    np.testing.assert_allclose(solution.calibration.com, truth.com, atol=1e-9)
    assert solution.diagnostics["shape_error"] < 1e-9


def test_pose_error_neither_moves_the_mass_nor_fakes_a_damaged_axis():
    """1° 的关节零位/IMU 误差不该被读成轴增益损伤。"""
    random = np.random.RandomState(11)
    samples = []
    for gravity in _orientations(12, seed=5):
        disturbed = _axis_rotation(random.normal(size=3),
                                   np.radians(1.0)) @ gravity
        samples.append(FtSample(
            gravity=gravity, wrench=expected_raw(TRUTH, disturbed)))

    solution = solve_ft_calibration(samples)

    assert solution.calibration.mass == pytest.approx(TRUTH.mass, rel=5e-3)
    assert solution.diagnostics["orientation_estimated"] is False
    assert solution.diagnostics["orientation_probability"] > 0.01


def test_a_touched_pose_is_rejected_as_a_whole():
    samples = _samples(count=10, seed=9)
    disturbed = FtSample(
        gravity=samples[4].gravity,
        wrench=samples[4].wrench + [9.0, -4.0, 6.0, 0.7, 0.4, -0.5])
    samples[4] = disturbed

    result = solve_ft_calibration(samples).calibration

    assert result.mass == pytest.approx(TRUTH.mass, abs=1e-3)
    np.testing.assert_allclose(result.com, TRUTH.com, atol=2e-3)


def test_three_orientations_are_the_minimum():
    with pytest.raises(ValueError):
        solve_ft_calibration(_samples(count=2))


def test_net_wrench_is_zero_without_a_payload():
    for gravity in _orientations():
        residual = net_wrench(expected_raw(TRUTH, gravity), TRUTH, gravity)
        np.testing.assert_allclose(residual, np.zeros(6), atol=1e-12)


def test_net_wrench_reports_the_payload_weight_in_the_link_frame():
    payload = FtCalibration(mass=0.8, com=[0.02, -0.01, 0.11])
    gravity = np.array([0.0, 0.0, -GRAVITY_MAGNITUDE])
    reading = expected_raw(TRUTH, gravity) + TRUTH.polarity * (
        tool_wrench(payload, gravity).reshape(2, 3) @ TRUTH.rotation.T).reshape(6)

    result = net_wrench(reading, TRUTH, gravity)

    np.testing.assert_allclose(result, tool_wrench(payload, gravity), atol=1e-12)
    assert instantaneous_mass(result, gravity) == pytest.approx(0.8)


def test_rezero_absorbs_a_drifted_offset_without_touching_the_tool():
    gravity = np.array([0.3, -1.0, -9.0])
    drift = np.array([3.5, -2.0, 1.25, 0.06, -0.02, 0.11])

    updated = rezero(expected_raw(TRUTH, gravity) + drift, TRUTH, gravity)

    assert updated.mass == TRUTH.mass
    np.testing.assert_allclose(updated.com, TRUTH.com)
    np.testing.assert_allclose(updated.bias, TRUTH.bias + drift, atol=1e-12)
    np.testing.assert_allclose(
        net_wrench(expected_raw(TRUTH, gravity) + drift, updated, gravity),
        np.zeros(6), atol=1e-12)


def test_orientation_coverage_flags_a_flat_pose_set():
    flat = [GRAVITY_MAGNITUDE * np.array([np.cos(angle), np.sin(angle), 0.0])
            for angle in np.linspace(0.0, 2.0, 6)]

    assert orientation_coverage(flat)["spread"] < 1e-9
    assert orientation_coverage(_orientations())["spread"] > 0.4


def test_orientation_coverage_survives_the_first_few_samples():
    """页面在只采了一两个点时也要能刷新，缺的方向就是零。"""
    for count in (0, 1, 2):
        coverage = orientation_coverage(_orientations(count))
        assert coverage["count"] == count
        assert len(coverage["singular_values"]) == 3
        assert coverage["spread"] == 0.0


def test_payload_estimator_needs_several_orientations_for_the_centre_of_mass():
    payload = FtCalibration(mass=1.3, com=[0.03, -0.02, 0.09])
    single = PayloadEstimator()
    single.add(*_payload_observation(payload, [0.0, 0.0, -GRAVITY_MAGNITUDE]))

    assert single.estimate().mass == pytest.approx(1.3, rel=1e-6)
    assert single.estimate().observability < 1e-6

    estimator = PayloadEstimator()
    for gravity in _orientations():
        estimator.add(*_payload_observation(payload, gravity))
    estimate = estimator.estimate()

    assert estimate.mass == pytest.approx(payload.mass, rel=1e-6)
    np.testing.assert_allclose(estimate.com, payload.com, atol=1e-6)
    assert estimate.observability > 0.3


def _payload_observation(payload, gravity):
    vector = np.asarray(gravity, dtype=float)
    return vector, tool_wrench(payload, vector)


def test_calibration_survives_a_dictionary_round_trip():
    rotation = _axis_rotation([0.0, 1.0, 0.0], 0.4)
    original = FtCalibration(
        force_bias=[1.0, 2.0, 3.0], torque_bias=[0.1, 0.2, 0.3],
        mass=0.5, com=[0.01, 0.02, 0.03], rotation=rotation, polarity=-1.0,
        origin=[0.0, 0.0, 0.053])

    restored = FtCalibration.from_dict(original.to_dict())

    np.testing.assert_allclose(restored.bias, original.bias)
    np.testing.assert_allclose(restored.rotation, original.rotation)
    np.testing.assert_allclose(restored.first_moment, original.first_moment)
    np.testing.assert_allclose(restored.origin, original.origin)
    assert restored.polarity == original.polarity


def test_the_torque_reference_point_only_moves_the_centre_of_mass():
    """取矩点在法兰面上时，同一批读数解出的质心整体外移一个传感器高度。"""
    origin = np.array([0.0, 0.0, 0.053])
    truth = FtCalibration(
        force_bias=TRUTH.force_bias, torque_bias=TRUTH.torque_bias,
        mass=TRUTH.mass, com=np.asarray(TRUTH.com) + origin, origin=origin)
    samples = _samples(truth)

    shifted = solve_ft_calibration(samples, origin=origin).calibration
    ignored = solve_ft_calibration(samples).calibration

    np.testing.assert_allclose(shifted.com, truth.com, atol=1e-9)
    np.testing.assert_allclose(ignored.com, TRUTH.com, atol=1e-9)
    # 忽略取矩点就等于把质心记错了整整一个传感器高度。
    np.testing.assert_allclose(shifted.com - ignored.com, origin, atol=1e-9)
    assert shifted.mass == pytest.approx(ignored.mass, abs=1e-9)


def test_a_flange_torque_point_is_recovered_from_the_modelled_centre_of_mass():
    origin = np.array([0.0, 0.0, 0.053])
    truth = FtCalibration(
        force_bias=TRUTH.force_bias, torque_bias=TRUTH.torque_bias,
        mass=TRUTH.mass, com=np.asarray(TRUTH.com) + origin, origin=origin)
    # 当前解还不知道取矩点，CAD 知道真实质心在哪。
    solved = solve_ft_calibration(_samples(truth)).calibration

    np.testing.assert_allclose(
        suggest_measurement_origin(solved, truth.com), origin, atol=1e-9)


def test_net_torque_is_taken_about_the_link_origin():
    origin = np.array([0.0, 0.0, 0.053])
    tool = FtCalibration(
        force_bias=TRUTH.force_bias, torque_bias=TRUTH.torque_bias,
        mass=TRUTH.mass, com=[0.004, -0.011, 0.121], origin=origin)
    payload = FtCalibration(mass=0.9, com=[0.0, 0.0, 0.2], origin=origin)
    gravity = np.array([0.0, -GRAVITY_MAGNITUDE, 0.0])
    weight = tool_wrench(payload, gravity)
    at_sensor = np.concatenate([
        weight[:3], weight[3:] + np.cross(-origin, weight[:3])])
    reading = expected_raw(tool, gravity) + tool.polarity * (
        at_sensor.reshape(2, 3) @ tool.rotation.T).reshape(6)

    np.testing.assert_allclose(
        net_wrench(reading, tool, gravity), weight, atol=1e-12)


def test_a_sideways_contact_force_is_not_taken_for_a_payload():
    gravity = np.array([0.0, 0.0, -GRAVITY_MAGNITUDE])
    payload = tool_wrench(FtCalibration(mass=0.9, com=[0.01, 0.0, 0.1]), gravity)

    assert gravity_aligned(payload, gravity) is True
    # 同样大小的横向力：投影出来的质量几乎为零，残差却是满量程。
    pushed = payload + [12.0, -4.0, 0.0, 0.0, 0.0, 0.0]
    assert gravity_aligned(pushed, gravity) is False
    # 沿重力方向多出来的一点力仍然像负载，只是质量变大。
    heavier = payload + [0.0, 0.0, -2.0, 0.0, 0.0, 0.0]
    assert gravity_aligned(heavier, gravity) is True
    assert instantaneous_mass(heavier, gravity) > 1.0
