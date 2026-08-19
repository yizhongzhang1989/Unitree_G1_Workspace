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
    # 双向采样的**半差**，与 applied_torque 那个半和同一批数据。不进回归（回归只看
    # 重力），只是顺手把摩擦留下来。单向采样时没有这个量，为零。
    friction: np.ndarray = None  # type: ignore[assignment]

    def __post_init__(self):
        if self.friction is None:
            object.__setattr__(self, "friction", np.zeros(7))


@dataclass(frozen=True)
class CalibrationFit:
    mass_scales: np.ndarray
    torque_bias: np.ndarray
    parameter_links: tuple
    group_scales: np.ndarray
    joint_noise: np.ndarray
    rank: int
    nullity: int
    singular_values: np.ndarray
    condition_number: float
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
    torque_noise_ratio: float = 0.01,
    scale_prior_std: float = 0.3,
    bias_prior_ratio: float = 0.05,
    minimum_noise_scale: float = 1e-3,
    scale_bounds=(0.2, 3.0),
    bias_bounds=(-8.0, 8.0),
) -> CalibrationFit:
    """Fit one mass scale per rigidly welded link group plus joint biases.

    Links welded to the same moving body only enter the joint torques through
    their aggregate mass and first moment, so estimating them individually is
    singular. One scale per group keeps the regression well conditioned; the
    group scale is written back to every link the group contains.

    Rows are divided by the assumed static torque noise of their joint, which
    starts at ``torque_noise_ratio`` of the joint effort limit and is then
    refined from the residual. The result is the maximum-a-posteriori solution
    for scales known to ``scale_prior_std`` and biases known to
    ``bias_prior_ratio`` of the joint rating.
    """
    indices = _selected_indices(side, selected_joint_names)
    if not samples:
        raise ValueError("at least one static target sample is required")

    aggregation = model.group_aggregation(side)
    parameter_links = model.parameter_links[side]
    link_count = len(parameter_links)
    current_scales, current_biases = model.arm_parameters(side)
    current = np.concatenate([current_scales, current_biases])
    selected_links = aggregation[:, indices].sum(axis=1) > 0.0
    fixed_scales = np.where(selected_links, 0.0, current_scales)
    fixed_biases = current_biases.copy()
    fixed_biases[indices] = 0.0
    variable_columns = np.concatenate([indices, 7 + indices])

    design_blocks = []
    observed_blocks = []
    before_blocks = []
    for sample in samples:
        q = np.asarray(sample.q, dtype=float)
        gravity = np.asarray(sample.gravity, dtype=float)
        torque = np.asarray(sample.applied_torque, dtype=float)
        if q.shape != (14,) or gravity.shape != (3,) or torque.shape != (7,):
            raise ValueError("static sample has invalid dimensions")
        link_design = model.design_matrix(side, q, gravity)
        group_design = np.hstack([
            link_design[:, :link_count] @ aggregation,
            link_design[:, link_count:],
        ])
        design_blocks.append(group_design[:, variable_columns])
        observed_blocks.append(
            torque - link_design[:, :link_count] @ fixed_scales - fixed_biases)
        before_blocks.append(torque - link_design @ current)

    design = np.vstack(design_blocks)
    observed = np.concatenate(observed_blocks)
    efforts = model.joint_efforts[side]
    joint_weights = torque_noise_ratio * efforts
    row_weights = np.tile(joint_weights, len(samples))
    scaled_design = design / row_weights[:, None]
    scaled_observed = observed / row_weights
    unit_precision = np.concatenate([
        np.full(indices.size, 1.0 / float(scale_prior_std) ** 2),
        1.0 / (bias_prior_ratio * efforts[indices]) ** 2,
    ])

    prior = np.concatenate([
        model.group_scales(side)[indices], current_biases[indices]])
    lower_bounds = np.concatenate([
        np.full(indices.size, float(scale_bounds[0])),
        np.full(indices.size, float(bias_bounds[0])),
    ])
    upper_bounds = np.concatenate([
        np.full(indices.size, float(scale_bounds[1])),
        np.full(indices.size, float(bias_bounds[1])),
    ])
    # The prior only outweighs the data in proportion to the real noise, so the
    # assumed noise is refined from the residual. The degrees-of-freedom term
    # keeps a pose set that barely determines the parameters from pretending to
    # be noise free.
    noise_scale = 1.0
    blocks = np.repeat(np.arange(len(samples)), 7)
    for _ in range(5):
        result = fit_robust_em(
            scaled_design,
            scaled_observed,
            prior_mean=prior,
            prior_precision=unit_precision * noise_scale ** 2,
            blocks=blocks,
            lower_bounds=lower_bounds,
            upper_bounds=upper_bounds,
        )
        effective = max(float(np.sum(result.inlier_probability)), 1.0)
        correction = np.sqrt(
            effective / max(effective - variable_columns.size, 1.0))
        updated = max(result.noise_std * correction, minimum_noise_scale)
        if abs(updated - noise_scale) <= 0.01 * noise_scale:
            break
        noise_scale = updated

    # In noise units a direction is resolved by the data once its singular
    # value exceeds the fitted noise level.
    signal = np.linalg.svd(scaled_design, compute_uv=False)
    rank = int(np.sum(signal > noise_scale))
    nullity = variable_columns.size - rank
    column_norms = np.linalg.norm(scaled_design, axis=0)
    safe_norms = np.where(column_norms > 0.0, column_norms, 1.0)
    shape = np.linalg.svd(scaled_design / safe_norms, compute_uv=False)
    smallest = (float(shape[-1]) if shape.size >= variable_columns.size else 0.0)
    condition_number = (float(shape[0] / smallest) if smallest > 0.0
                        else float("inf"))
    # Posterior variance reduction: 0 keeps the prior, 1 is fully data driven.
    posterior = np.linalg.inv(
        scaled_design.T @ scaled_design / noise_scale ** 2 +
        np.diag(unit_precision))
    observability = np.clip(
        1.0 - np.diag(posterior) * unit_precision, 0.0, 1.0)

    group_scales = model.group_scales(side)
    group_scales[indices] = result.parameters[:indices.size]
    group_observability = np.zeros(7, dtype=float)
    group_observability[indices] = observability[:indices.size]
    bias_observability = np.zeros(7, dtype=float)
    bias_observability[indices] = observability[indices.size:]
    updated_scales = np.where(
        selected_links, aggregation @ group_scales, current_scales)
    updated_biases = current_biases.copy()
    updated_biases[indices] = result.parameters[indices.size:]
    return CalibrationFit(
        mass_scales=updated_scales,
        torque_bias=updated_biases,
        parameter_links=tuple(parameter_links),
        group_scales=group_scales,
        joint_noise=joint_weights * noise_scale,
        rank=rank,
        nullity=nullity,
        singular_values=signal,
        condition_number=condition_number,
        scale_observability=np.where(
            selected_links, aggregation @ group_observability, 0.0),
        bias_observability=bias_observability,
        rmse_before=float(np.sqrt(np.mean(
            np.concatenate(before_blocks) ** 2))),
        rmse_after=float(np.sqrt(np.mean(
            (observed - design @ result.parameters) ** 2))),
        em=result,
    )