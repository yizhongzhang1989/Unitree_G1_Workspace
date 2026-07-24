"""Robust linear parameter fitting with a Gaussian-mixture EM model."""

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np
from scipy.optimize import lsq_linear


@dataclass(frozen=True)
class EMResult:
    parameters: np.ndarray
    inlier_probability: np.ndarray
    inlier_fraction: float
    noise_std: float
    outlier_noise_std: float
    iterations: int
    converged: bool


def _as_parameter_vector(value, size: int, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.ndim == 0:
        return np.full(size, float(array), dtype=float)
    if array.shape != (size,):
        raise ValueError("%s must be a scalar or have shape (%d,)" % (name, size))
    return array.copy()


def _weighted_ridge(
    design: np.ndarray,
    observed: np.ndarray,
    weights: np.ndarray,
    prior_mean: np.ndarray,
    prior_precision: np.ndarray,
    prior_factor: np.ndarray,
    lower_bounds: np.ndarray,
    upper_bounds: np.ndarray,
) -> np.ndarray:
    weighted_square_root = np.sqrt(weights)
    augmented_design = design * weighted_square_root[:, None]
    augmented_observed = (
        observed - design @ prior_mean) * weighted_square_root
    regularized = prior_precision > 0.0
    if np.any(regularized):
        prior_rows = np.diag(np.sqrt(prior_precision[regularized]))
        expanded_rows = np.zeros((prior_rows.shape[0], design.shape[1]))
        expanded_rows[:, regularized] = prior_rows
        augmented_design = np.vstack([augmented_design, expanded_rows])
        augmented_observed = np.concatenate([
            augmented_observed, np.zeros(prior_rows.shape[0])])
    if prior_factor.shape[0]:
        augmented_design = np.vstack([augmented_design, prior_factor])
        augmented_observed = np.concatenate([
            augmented_observed, np.zeros(prior_factor.shape[0])])
    result = lsq_linear(
        augmented_design,
        augmented_observed,
        bounds=(lower_bounds - prior_mean, upper_bounds - prior_mean),
        method="trf",
        lsmr_tol="auto",
    )
    if not result.success:
        raise RuntimeError("bounded least-squares fit failed: %s" % result.message)
    return prior_mean + result.x


def fit_robust_em(
    design: Sequence[Sequence[float]],
    observed: Sequence[float],
    *,
    prior_mean=0.0,
    prior_precision=0.0,
    prior_factor: Optional[Sequence[Sequence[float]]] = None,
    outlier_scale: float = 10.0,
    initial_inlier_fraction: float = 0.9,
    max_iterations: int = 100,
    tolerance: float = 1e-7,
    minimum_noise_std: float = 1e-4,
    lower_bounds: Optional[Sequence[float]] = None,
    upper_bounds: Optional[Sequence[float]] = None,
) -> EMResult:
    """Fit ``observed = design @ parameters`` with a two-noise EM model.

    The narrow Gaussian models settled static samples. The second Gaussian
    absorbs disturbances and is constrained to be at least ``outlier_scale``
    times wider. A diagonal Gaussian prior anchors weakly observable inertial
    coefficients to their URDF values.
    """
    matrix = np.asarray(design, dtype=float)
    values = np.asarray(observed, dtype=float)
    if matrix.ndim != 2 or values.ndim != 1 or matrix.shape[0] != values.size:
        raise ValueError("design must be 2-D and observed must match its rows")
    if matrix.shape[0] == 0 or matrix.shape[1] == 0:
        raise ValueError("design must not be empty")
    if not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(values)):
        raise ValueError("design and observed must contain only finite values")
    if outlier_scale <= 1.0:
        raise ValueError("outlier_scale must be greater than 1")
    if not 0.0 < initial_inlier_fraction < 1.0:
        raise ValueError("initial_inlier_fraction must be between 0 and 1")

    parameter_count = matrix.shape[1]
    prior = _as_parameter_vector(prior_mean, parameter_count, "prior_mean")
    precision = _as_parameter_vector(
        prior_precision, parameter_count, "prior_precision")
    if np.any(precision < 0.0):
        raise ValueError("prior_precision must be non-negative")
    factor = (np.empty((0, parameter_count), dtype=float)
              if prior_factor is None else np.asarray(prior_factor, dtype=float))
    if (factor.ndim != 2 or factor.shape[1] != parameter_count or
            not np.all(np.isfinite(factor))):
        raise ValueError(
            "prior_factor must be finite with one column per parameter")

    lower = _as_parameter_vector(
        -np.inf if lower_bounds is None else lower_bounds,
        parameter_count,
        "lower_bounds",
    )
    upper = _as_parameter_vector(
        np.inf if upper_bounds is None else upper_bounds,
        parameter_count,
        "upper_bounds",
    )
    if np.any(lower > upper):
        raise ValueError("lower_bounds must not exceed upper_bounds")

    parameters = _weighted_ridge(
        matrix, values, np.ones(values.size), prior, precision,
        factor, lower, upper,
    )
    residual = values - matrix @ parameters
    median_residual = np.median(residual)
    robust_std = 1.4826 * np.median(np.abs(residual - median_residual))
    noise_std = max(float(robust_std), minimum_noise_std)
    outlier_noise_std = max(
        float(np.sqrt(np.mean(residual ** 2))),
        outlier_scale * noise_std,
    )
    inlier_fraction = float(initial_inlier_fraction)
    probability = np.full(values.size, inlier_fraction, dtype=float)
    converged = False

    for iteration in range(1, max_iterations + 1):
        previous = parameters
        residual = values - matrix @ parameters
        normalized = residual / noise_std
        log_inlier = (
            np.log(inlier_fraction)
            - np.log(noise_std)
            - 0.5 * normalized * normalized
        )
        log_outlier = (
            np.log1p(-inlier_fraction)
            - np.log(outlier_noise_std)
            - 0.5 * (residual / outlier_noise_std) ** 2
        )
        probability = 1.0 / (1.0 + np.exp(np.clip(
            log_outlier - log_inlier, -700.0, 700.0)))

        effective_weight = probability + (
            (1.0 - probability) * noise_std ** 2 / outlier_noise_std ** 2)
        parameters = _weighted_ridge(
            matrix, values, effective_weight, prior, precision,
            factor, lower, upper,
        )
        residual = values - matrix @ parameters
        inlier_weight = max(float(np.sum(probability)), 1.0)
        outlier_weight = max(float(np.sum(1.0 - probability)), 1.0)
        noise_std = max(
            float(np.sqrt(np.sum(probability * residual ** 2) / inlier_weight)),
            minimum_noise_std,
        )
        outlier_noise_std = max(
            float(np.sqrt(
                np.sum((1.0 - probability) * residual ** 2) / outlier_weight)),
            outlier_scale * noise_std,
        )
        inlier_fraction = float(np.clip(np.mean(probability), 0.01, 0.99))

        delta = np.linalg.norm(parameters - previous)
        scale = 1.0 + np.linalg.norm(previous)
        if delta <= tolerance * scale:
            converged = True
            break

    return EMResult(
        parameters=parameters,
        inlier_probability=probability,
        inlier_fraction=inlier_fraction,
        noise_std=noise_std,
        outlier_noise_std=outlier_noise_std,
        iterations=iteration,
        converged=converged,
    )