"""Selected-joint arm calibration built on the Pinocchio gravity regressor."""

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np

from .constants import ARM_JOINTS
from .em import EMResult, fit_robust_em


@dataclass(frozen=True)
class StaticSample:
    target_id: int
    q: np.ndarray
    gravity: np.ndarray
    applied_torque: np.ndarray
    estimated_torque: np.ndarray
    position_error: np.ndarray
    velocity_std: np.ndarray


@dataclass(frozen=True)
class CalibrationFit:
    mass_scales: np.ndarray
    torque_bias: np.ndarray
    parameter_links: tuple
    selected_joint_names: tuple
    rank: int
    nullity: int
    singular_values: np.ndarray
    scale_observability: np.ndarray
    bias_observability: np.ndarray
    rmse_before: float
    rmse_after: float
    em: EMResult


def _selected_indices(side: str, selected_joint_names: Iterable[str]) -> np.ndarray:
    names = ARM_JOINTS.get(side)
    if names is None:
        raise ValueError("side must be 'left' or 'right'")
    selected = set(selected_joint_names)
    invalid = selected.difference(names)
    if invalid:
        raise ValueError("selected joints are not in the %s arm: %s"
                         % (side, sorted(invalid)))
    indices = np.array([index for index, name in enumerate(names)
                        if name in selected], dtype=int)
    if indices.size == 0:
        raise ValueError("at least one %s arm joint must be selected" % side)
    return indices


def fit_selected_joints(
    model,
    side: str,
    selected_joint_names: Sequence[str],
    samples: Sequence[StaticSample],
    *,
    scale_prior_precision: float = 0.0,
    bias_prior_precision: float = 0.0,
    minimum_singular_value: float = 1e-6,
    minimum_singular_value_ratio: float = 1e-3,
    scale_bounds=(0.2, 3.0),
    bias_bounds=(-8.0, 8.0),
) -> CalibrationFit:
    """Fit only selected link scales and joint biases from one static sweep."""
    indices = _selected_indices(side, selected_joint_names)
    if not samples:
        raise ValueError("at least one static target sample is required")
    current_scales, current_biases = model.arm_parameters(side)
    parameter_links = model.parameter_links[side]
    parameter_count = len(parameter_links)
    current = np.concatenate([current_scales, current_biases])
    selected_names = set(selected_joint_names)
    scale_indices = np.array([
        index for index, link_name in enumerate(parameter_links)
        if model.parameter_owner[link_name] in selected_names
    ], dtype=int)
    variable_columns = np.concatenate([
        scale_indices, parameter_count + indices])
    fixed_columns = np.array(
        [index for index in range(parameter_count + 7)
         if index not in set(variable_columns)],
        dtype=int,
    )

    design_blocks = []
    observed_blocks = []
    before_blocks = []
    for sample in samples:
        q = np.asarray(sample.q, dtype=float)
        gravity = np.asarray(sample.gravity, dtype=float)
        torque = np.asarray(sample.applied_torque, dtype=float)
        if q.shape != (14,) or gravity.shape != (3,) or torque.shape != (7,):
            raise ValueError("static sample has invalid dimensions")
        full_design = model.design_matrix(side, q, gravity)
        selected_rows = full_design
        adjusted = torque
        if fixed_columns.size:
            adjusted = adjusted - selected_rows[:, fixed_columns] @ current[fixed_columns]
        design_blocks.append(selected_rows[:, variable_columns])
        observed_blocks.append(adjusted)
        before_blocks.append(torque - selected_rows @ current)

    design = np.vstack(design_blocks)
    observed = np.concatenate(observed_blocks)
    _, singular_values, right_vectors = np.linalg.svd(
        design, full_matrices=True)
    singular_threshold = max(
        minimum_singular_value,
        minimum_singular_value_ratio * singular_values[0],
    )
    rank = int(np.sum(singular_values > singular_threshold))
    nullity = design.shape[1] - rank
    variable_observability = np.sum(right_vectors[:rank] ** 2, axis=0)
    weak_direction_prior = singular_threshold * right_vectors[rank:]

    prior = current[variable_columns]
    column_energy = np.sum(design ** 2, axis=0)
    relative_prior_strength = np.concatenate([
        np.full(scale_indices.size, scale_prior_precision),
        np.full(indices.size, bias_prior_precision),
    ])
    result = fit_robust_em(
        design,
        observed,
        prior_mean=prior,
        prior_precision=relative_prior_strength * column_energy,
        prior_factor=weak_direction_prior,
        lower_bounds=np.concatenate([
            np.full(scale_indices.size, float(scale_bounds[0])),
            np.full(indices.size, float(bias_bounds[0])),
        ]),
        upper_bounds=np.concatenate([
            np.full(scale_indices.size, float(scale_bounds[1])),
            np.full(indices.size, float(bias_bounds[1])),
        ]),
    )

    updated_scales = current_scales.copy()
    updated_biases = current_biases.copy()
    scale_count = scale_indices.size
    updated_scales[scale_indices] = result.parameters[:scale_count]
    updated_biases[indices] = result.parameters[scale_count:]
    scale_observability = np.zeros(parameter_count, dtype=float)
    bias_observability = np.zeros(7, dtype=float)
    scale_observability[scale_indices] = variable_observability[:scale_count]
    bias_observability[indices] = variable_observability[scale_count:]
    residual_before = np.concatenate(before_blocks)
    residual_after = observed - design @ result.parameters
    return CalibrationFit(
        mass_scales=updated_scales,
        torque_bias=updated_biases,
        parameter_links=tuple(parameter_links),
        selected_joint_names=tuple(ARM_JOINTS[side][index] for index in indices),
        rank=rank,
        nullity=nullity,
        singular_values=singular_values,
        scale_observability=scale_observability,
        bias_observability=bias_observability,
        rmse_before=float(np.sqrt(np.mean(residual_before ** 2))),
        rmse_after=float(np.sqrt(np.mean(residual_after ** 2))),
        em=result,
    )