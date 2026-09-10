"""Unified offline foot-contact refinement; no rigid-body dynamics validation."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.ndimage import median_filter
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .motion_capture import FPS, save_motion
from .motion_model import MotionModel
from .source_replay import retarget_source
from .urdf import DEFAULT_URDF, resolve_package_path


def geometry(model, row):
    root = Rotation.from_quat(row[3:7]).as_matrix()
    model.kin.key_body_pos(row[7:], model.feet)
    points, rotations = [], []
    for side, name in enumerate(model.feet):
        rotation = root @ model.kin.frame_rot(name)
        position = row[:3] + root @ model.kin.frame_pos(name)
        contacts = np.array([position + rotation @ center - [0, 0, radius]
                             for center, radius in model.contacts[side]])
        points.append(contacts)
        rotations.append(rotation)
    return np.array(points), np.array(rotations)


def intervals(mask):
    edges = np.diff(np.r_[False, mask, False].astype(int))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def _tracking_weights(confidence):
    return 1.0 + 999.0 * np.asarray(confidence)**2


def contact_targets(points, model=None, rotations=None):
    """Infer stationary contact points without filling short flight intervals."""
    heights = median_filter(points[..., 2], size=(3, 1, 1), mode='nearest')
    vertical_speed = np.gradient(heights, 1.0 / FPS, axis=0)
    contacts = np.zeros(heights.shape, dtype=bool)
    targets = points.copy()
    weights = np.zeros(heights.shape)
    for side in range(points.shape[1]):
        for corner in range(points.shape[2]):
            active = False
            for index in range(len(points)):
                height = heights[index, side, corner]
                speed = vertical_speed[index, side, corner]
                if active:
                    active = height < 0.04 and speed < 0.15
                else:
                    active = height < 0.02 and abs(speed) < 0.12
                contacts[index, side, corner] = active
            for start, end in intervals(contacts[:, side, corner]):
                if end - start < 3:
                    contacts[start:end, side, corner] = False
                    continue
                anchor = np.r_[np.median(points[start:end, side, corner, :2], axis=0), 0.0001]
                ramp = min(25, max(1, (end - start - 1) // 3))
                for index in range(start, end):
                    fade_in = 1.0 if start == 0 else min(1.0, (index - start) / ramp)
                    fade_out = 1.0 if end == len(points) else min(1.0, (end - index - 1) / ramp)
                    blend = min(fade_in, fade_out)
                    blend = blend * blend * (3 - 2 * blend)
                    weights[index, side, corner] = blend
                    targets[index, side, corner] = (1 - blend) * points[index, side, corner] + blend * anchor
    if model is not None and rotations is not None:
        for side in range(2):
            local = np.array([center for center, radius in model.contacts[side]])
            radii = np.array([radius for center, radius in model.contacts[side]])
            for start, end in intervals(contacts[:, side].any(axis=1)):
                flat_frames = contacts[start:end, side].all(axis=1)
                if flat_frames.any():
                    flat_rotations = rotations[start:end, side][flat_frames]
                    yaw = np.arctan2(flat_rotations[:, 1, 0], flat_rotations[:, 0, 0])
                    heading = np.arctan2(np.mean(np.sin(yaw)), np.mean(np.cos(yaw)))
                    orientation = Rotation.from_euler('z', heading).as_matrix()
                else:
                    orientation = Rotation.from_matrix(rotations[start:end, side]).mean().as_matrix()
                anchor = local @ orientation.T
                anchor[:, 2] -= radii
                anchor[:, 2] += 0.0001 - anchor[:, 2].min()
                anchor[:, :2] += np.median(points[start:end, side, :, :2].mean(axis=1), axis=0) - anchor[:, :2].mean(axis=0)
                blend = weights[start:end, side, :, None]
                targets[start:end, side] = (1 - blend) * points[start:end, side] + blend * anchor
    return contacts, targets, weights


def refine(model, rows, *, max_nfev=400):
    original = np.asarray(rows, dtype=float)
    if original.ndim != 2 or original.shape[1] != 36 or len(original) < 3 or not np.isfinite(original).all():
        raise ValueError('Expected finite Nx36 motion')
    smoothness = np.r_[[2.0] * 3, [0.5] * 12]
    deviation = np.r_[[2.0] * 3, [0.1] * 12]
    geometry_before = [geometry(model, row) for row in original]
    points = np.array([item[0] for item in geometry_before])
    rotations = np.array([item[1] for item in geometry_before])
    contacts, targets, weights = contact_targets(points, model, rotations)
    lower, upper = model.kin.limits()
    output = original.copy()
    previous_delta = np.zeros(15)
    evaluations, successes = [], []
    for index, row in enumerate(original):
        reference = np.r_[row[:3], row[7:19]]
        bounds = (np.r_[row[:3] - 0.25, lower[:12]], np.r_[row[:3] + 0.25, upper[:12]])
        if not contacts[index].any() and points[index, ..., 2].min() >= 0:
            previous_delta = np.zeros(15)
            evaluations.append(0)
            successes.append(True)
            continue
        tracking_weights = _tracking_weights(weights[index])[..., None]

        def residual(variables):
            candidate = row.copy()
            candidate[:3], candidate[7:19] = variables[:3], variables[3:]
            actual, _ = geometry(model, candidate)
            tracking = (actual - targets[index]) * tracking_weights
            penetration = np.minimum(actual[..., 2], 0.0) * 1000.0
            delta = variables - reference
            return np.r_[tracking.ravel(), penetration.ravel(),
                         delta * deviation,
                         (delta - previous_delta) * smoothness]

        initial = np.clip(reference + previous_delta, *bounds)
        results = [least_squares(
            residual, initial, bounds=bounds, max_nfev=max_nfev,
            x_scale='jac', ftol=1e-7, xtol=1e-7, gtol=1e-7)]
        if not results[0].success:
            results.append(least_squares(
                residual, initial, bounds=bounds, max_nfev=max_nfev,
                x_scale=1.0, ftol=1e-7, xtol=1e-7, gtol=1e-7))
        successful = [candidate for candidate in results if candidate.success]
        result = min(successful or results, key=lambda candidate: candidate.cost)
        output[index, :3], output[index, 7:19] = result.x[:3], result.x[3:]
        previous_delta = result.x - reference
        evaluations.append(sum(candidate.nfev for candidate in results))
        successes.append(result.success)
    return output, contacts, weights, targets, dict(
        converged_frames=int(sum(successes)), total_frames=len(original),
        evaluations_max=int(max(evaluations)),
        unconverged_frames=np.flatnonzero(~np.asarray(successes)).tolist())


def metrics(model, rows, contacts, weights, targets):
    points = np.array([geometry(model, row)[0] for row in rows])
    tracked = points.reshape(len(points), -1, 3)
    masks = contacts.reshape(len(points), -1)
    weights = weights.reshape(len(points), -1)
    speed = np.linalg.norm(np.diff(tracked[..., :2], axis=0), axis=2) * FPS
    stable = weights >= 1 - 1e-8
    supported = stable[:-1] & stable[1:]
    transition = np.any((weights > 0) & (weights < 1), axis=1)
    transition_steps = transition[:-1] | transition[1:]
    errors = np.linalg.norm(points - targets, axis=3)
    contact_error = errors.reshape(len(points), -1)[stable]
    support_speed = speed[supported]
    slips = []
    for channel in range(tracked.shape[1]):
        for start, end in intervals(stable[:, channel]):
            offset = tracked[start:end, channel, :2] - tracked[start, channel, :2]
            slips.append(float(np.linalg.norm(offset, axis=1).max()))

    def distribution(values):
        return dict(zip(('p50', 'p95', 'max'), np.percentile(values, [50, 95, 100]).tolist())) if np.size(values) else None

    return dict(
        support_metric_basis='individual_contact_points',
        stable_support_intervals=int(supported.sum()),
        penetration_max_m=float(max(0, -points[..., 2].min())),
        support_xy_speed_m_s=distribution(support_speed),
        support_anchor_error_m=distribution(contact_error),
        support_segment_slip_max_m=max(slips, default=0.0),
        joint_speed_rad_s=distribution(np.abs(np.diff(rows[:, 7:], axis=0)) * FPS),
        leg_speed_rad_s=distribution(np.abs(np.diff(rows[:, 7:19], axis=0)) * FPS),
        transition_leg_speed_rad_s=distribution(
            np.abs(np.diff(rows[:, 7:19], axis=0))[transition_steps] * FPS),
        leg_acceleration_rad_s2=distribution(np.abs(np.diff(rows[:, 7:19], n=2, axis=0)) * FPS**2),
        root_speed_m_s=distribution(np.linalg.norm(np.diff(rows[:, :3], axis=0), axis=1) * FPS),
        contact_intervals={
            f'{("left", "right")[channel // points.shape[2]]}_{channel % points.shape[2]}':
            [[start / FPS, end / FPS] for start, end in intervals(masks[:, channel])]
            for channel in range(tracked.shape[1])})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--urdf', default=DEFAULT_URDF)
    parser.add_argument('--source', type=Path,
                        help='Raw PICO .source.npz to replay before contact refinement')
    args = parser.parse_args()
    source = args.input.read_bytes()
    baseline = np.loadtxt(args.input, delimiter=',', ndmin=2)
    model = MotionModel(resolve_package_path(args.urdf))
    rows = retarget_source(model, args.source) if args.source else baseline
    if rows.shape != baseline.shape:
        raise ValueError(
            f'Replayed source shape {rows.shape} differs from CSV shape {baseline.shape}')
    output, contacts, weights, targets, solver = refine(model, rows)
    if solver['unconverged_frames']:
        raise RuntimeError(f"Unconverged frames: {solver['unconverged_frames']}")
    original_points = np.array([geometry(model, row)[0] for row in rows])
    refined_points = np.array([geometry(model, row)[0] for row in output])
    airborne = ~contacts.any(axis=(1, 2))
    swing_mask = ~contacts.any(axis=2)
    swing_error = np.linalg.norm(refined_points - original_points, axis=3)[swing_mask]
    report = dict(source=str(args.input), source_sha256=hashlib.sha256(source).hexdigest(),
                  raw_source=str(args.source) if args.source else None,
                  raw_source_sha256=(hashlib.sha256(args.source.read_bytes()).hexdigest()
                                     if args.source else None),
                  replayed_from_raw=bool(args.source),
                  replay_difference_max_rad=float(
                      np.abs(rows[:, 7:] - baseline[:, 7:]).max()),
                  algorithm='landmark_dls_then_confidence_weighted_contacts_v3'
                  if args.source else 'confidence_weighted_contacts_v2',
                  assumption='Inferred contacts with numerical no-slip penalties; no dynamics validation',
                  solver=solver, before=metrics(model, rows, contacts, weights, targets),
                  after=metrics(model, output, contacts, weights, targets),
                  root_correction_max_m=float(np.linalg.norm(output[:, :3] - rows[:, :3], axis=1).max()),
                  joint_correction_max_rad=float(np.abs(output[:, 7:] - rows[:, 7:]).max()),
                  swing_point_correction_max_m=float(swing_error.max()) if swing_error.size else 0.0,
                  airborne_frames=int(airborne.sum()),
                  airborne_unchanged=bool(np.array_equal(output[airborne], rows[airborne])),
                  preserved_root_orientation_and_upper_body=bool(
                      np.array_equal(output[:, 3:7], rows[:, 3:7])
                      and np.array_equal(output[:, 19:], rows[:, 19:])))
    lower, upper = model.kin.limits()
    if (not np.isfinite(output).all() or np.any(output[:, 7:] < lower - 1e-8)
            or np.any(output[:, 7:] > upper + 1e-8)):
        raise RuntimeError('Refinement produced invalid joint positions')
    if args.input.read_bytes() != source:
        raise RuntimeError('Source changed during experiment')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    target = save_motion(args.output_dir, output, category='refined', action=args.input.stem)
    report['output'] = str(target)
    report_path = args.output_dir / (target.stem + '_report.json')
    report_path.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
