import numpy as np

from arm_gravity_compensation.em import fit_robust_em


def test_em_recovers_parameters_with_large_outliers():
    random = np.random.RandomState(7)
    design = random.normal(size=(400, 3))
    expected = np.array([0.82, 1.17, -0.06])
    observed = design @ expected + random.normal(scale=0.015, size=400)
    outliers = random.choice(400, size=50, replace=False)
    observed[outliers] += random.normal(scale=1.5, size=outliers.size)

    result = fit_robust_em(
        design,
        observed,
        prior_mean=[1.0, 1.0, 0.0],
        prior_precision=[0.05, 0.05, 0.001],
    )

    assert result.converged
    np.testing.assert_allclose(result.parameters, expected, atol=0.015)
    assert np.mean(result.inlier_probability[outliers]) < 0.25
    assert result.noise_std < 0.03


def test_em_rejects_non_finite_input():
    design = np.eye(2)
    design[0, 0] = np.nan

    try:
        fit_robust_em(design, [1.0, 2.0])
    except ValueError as error:
        assert "finite" in str(error)
    else:
        raise AssertionError("non-finite design was accepted")


def test_underdetermined_fit_preserves_prior_in_null_space():
    design = np.array([
        [1.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    prior = np.array([0.8, 1.2, -0.3])
    observed = np.array([2.4, 0.5])

    result = fit_robust_em(
        design,
        observed,
        prior_mean=prior,
        minimum_noise_std=1e-3,
    )

    np.testing.assert_allclose(design @ result.parameters, observed, atol=1e-8)
    assert abs((result.parameters[0] - result.parameters[1]) -
               (prior[0] - prior[1])) < 1e-8


def test_bounded_fit_solves_constraints_instead_of_clipping_solution():
    design = np.array([
        [1.0, 1.0],
        [1.0, 0.0],
    ])
    observed = np.array([4.0, 0.0])

    result = fit_robust_em(
        design,
        observed,
        lower_bounds=[0.0, 0.0],
        upper_bounds=[1.0, 10.0],
        minimum_noise_std=1e-3,
    )

    np.testing.assert_allclose(result.parameters, [0.0, 4.0], atol=1e-5)
    assert np.linalg.norm(design @ result.parameters - observed) < 1e-5