"""Replay a captured PICO sidecar through the same live retargeter."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .motion_capture import MotionClip
from .retarget import RetargetCalibration, Retargeter
from .skeleton import SMPL_JOINTS, STATUS_VALID, BodyFrame


def _array(data, name, shape):
    if name not in data:
        raise ValueError(f'Source sidecar has no {name}')
    value = np.asarray(data[name])
    if value.shape != shape or not np.isfinite(value).all():
        raise ValueError(f'Source sidecar {name} must be finite with shape {shape}')
    return value


def retarget_source(model, path, *, landmark_iterations=None):
    """Return 50 Hz CSV rows reconstructed from raw PICO samples."""
    path = Path(path)
    with np.load(path, allow_pickle=False) as data:
        version = int(np.asarray(data['format_version']))
        if version not in (1, 2):
            raise ValueError('Unsupported source sidecar format_version')
        recorded_iterations = (int(_array(data, 'landmark_iterations', ()))
                               if version >= 2 else 2)
        if landmark_iterations is None:
            landmark_iterations = recorded_iterations
        if not isinstance(landmark_iterations, int) or not 0 <= landmark_iterations <= 4:
            raise ValueError('landmark_iterations must be an integer from 0 to 4')
        if tuple(data['joint_names'].tolist()) != SMPL_JOINTS:
            raise ValueError('Source sidecar joint_names differ from the SMPL contract')
        timestamps = np.asarray(data['timestamps'], dtype=np.float64)
        count = len(timestamps)
        if (count < 2 or timestamps.shape != (count,)
                or not np.isfinite(timestamps).all()
                or np.any(np.diff(timestamps) <= 0.0)):
            raise ValueError('Source timestamps must be finite and strictly increasing')
        sequences = _array(data, 'sequences', (count,)).astype(np.int64)
        positions = _array(data, 'positions', (count, len(SMPL_JOINTS), 3))
        rotations = _array(data, 'orientations', (count, len(SMPL_JOINTS), 3, 3))
        statuses = _array(data, 'statuses', (count,)).astype(np.uint8)
        messages = _array(data, 'messages', (count,)).astype(np.int32)
        if np.any(statuses != STATUS_VALID) or np.any(messages != 0):
            raise ValueError('Source sidecar contains invalid or limited tracking')
        orthogonality = rotations @ rotations.transpose(0, 1, 3, 2)
        if (np.max(np.abs(orthogonality - np.eye(3))) > 1e-5
                or np.max(np.abs(np.linalg.det(rotations) - 1.0)) > 1e-5):
            raise ValueError('Source orientations are not proper rotation matrices')
        calibration = RetargetCalibration(
            scale=float(_array(data, 'calibration_scale', ())),
            pelvis_ref_z=float(_array(data, 'calibration_pelvis_ref_z', ())),
            stand_height=float(_array(data, 'calibration_stand_height', ())),
            pelvis_fix=_array(data, 'calibration_pelvis_fix', (3, 3)),
            torso_fix=_array(data, 'calibration_torso_fix', (3, 3)),
            joint_bias=_array(data, 'calibration_joint_bias', (29,)),
            joint_target=_array(data, 'calibration_joint_target', (29,)),
            arm_hinge_axes=_array(data, 'calibration_arm_hinge_axes', (2, 3)))

    retargeter = Retargeter(
        model.kin, key_bodies=('torso_link',), anchor_body='torso_link',
        default_joint_pos=calibration.joint_target,
        landmark_iterations=landmark_iterations)
    clip = MotionClip(*model.kin.limits())
    previous_joint_pos = None
    for timestamp, sequence, position, rotation, status, message in zip(
            timestamps, sequences, positions, rotations, statuses, messages):
        frame = BodyFrame(float(timestamp), int(sequence), position, int(status),
                          int(message), rotation)
        result = retargeter.solve(
            frame, calibration, previous_joint_pos=previous_joint_pos)
        previous_joint_pos = result.joint_pos
        row = np.concatenate((result.root_pos, result.root_quat[[1, 2, 3, 0]],
                              result.joint_pos))
        clip.append(timestamp, row, id(calibration))
    return clip.resample()
