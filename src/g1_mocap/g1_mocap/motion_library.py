"""Read-only access to recorded motion CSVs for browser playback."""

import io
from pathlib import Path

import numpy as np

from .motion_capture import FPS, JOINT_NAMES
from .urdf import under


class MotionLibrary:
    MAX_BYTES = 8 * 1024 * 1024

    def __init__(self, directory):
        self.directory = Path(directory).expanduser().resolve()
        self.root = self.directory / 'motions'

    def _path(self, name):
        path = under(self.root, name)
        if path is None:
            return None
        path = path.resolve()
        return path if path.is_relative_to(self.root.resolve()) else None

    def list(self):
        entries = []
        for path in sorted(self.root.glob('*.csv')):
            resolved = self._path(path.name)
            if resolved is None or not resolved.is_file():
                continue
            stat = resolved.stat()
            entries.append(dict(file_name=path.name, size_bytes=stat.st_size,
                                modified_ns=stat.st_mtime_ns))
        return dict(directory=str(self.directory), motions=entries)

    def load(self, name):
        if not name or Path(name).name != name or not name.endswith('.csv'):
            raise ValueError('Invalid motion file name')
        path = self._path(name)
        if path is None or not path.is_file():
            raise FileNotFoundError('Motion not found')
        if path.stat().st_size > self.MAX_BYTES:
            raise ValueError('Motion file exceeds 8 MiB')
        with path.open('rb') as stream:
            content = stream.read(self.MAX_BYTES + 1)
        if len(content) > self.MAX_BYTES:
            raise ValueError('Motion file exceeds 8 MiB')
        rows = np.loadtxt(io.BytesIO(content), delimiter=',', ndmin=2)
        if rows.shape[1] != 36 or not 2 <= len(rows) <= 3000:
            raise ValueError('Expected 2 to 3000 frames with exactly 36 columns')
        if not np.isfinite(rows).all():
            raise ValueError('Motion contains NaN or Inf')
        norms = np.linalg.norm(rows[:, 3:7], axis=1)
        if np.any(np.abs(norms - 1.0) > 1e-3):
            raise ValueError('Root quaternion is not unit length')
        return dict(file_name=name, fps=FPS, num_frames=len(rows),
                    duration_seconds=len(rows) / FPS, joint_names=list(JOINT_NAMES),
                    frames=rows.tolist())
