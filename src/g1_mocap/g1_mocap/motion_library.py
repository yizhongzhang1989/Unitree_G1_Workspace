"""Read-only access to recorded motion CSVs for browser playback."""

import io
from pathlib import Path
from typing import TypedDict

import numpy as np

from .motion_capture import FPS, JOINT_NAMES, archive_motion
from .urdf import under


class MotionEntry(TypedDict):
    file_name: str
    size_bytes: int
    modified_ns: int


class MotionListing(TypedDict):
    directory: str
    motions: list[MotionEntry]


class MotionPlayback(TypedDict):
    file_name: str
    fps: int
    num_frames: int
    duration_seconds: float
    joint_names: list[str]
    frames: list[list[float]]


class MotionLibrary:
    MAX_BYTES: int = 8 * 1024 * 1024

    def __init__(self, directory: str | Path) -> None:
        self.directory = Path(directory).expanduser().resolve()
        self.root = self.directory / 'motions'

    def _path(self, name: str) -> Path | None:
        path = under(self.root, name)
        if path is None:
            return None
        path = path.resolve()
        return path if path.is_relative_to(self.root.resolve()) else None

    def list(self) -> MotionListing:
        entries: list[MotionEntry] = []
        for path in sorted(self.root.glob('*.csv')):
            resolved = self._path(path.name)
            if resolved is None or not resolved.is_file():
                continue
            stat = resolved.stat()
            entries.append({'file_name': path.name, 'size_bytes': stat.st_size,
                            'modified_ns': stat.st_mtime_ns})
        return {'directory': str(self.directory), 'motions': entries}

    def load(self, name: str) -> MotionPlayback:
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
        return {'file_name': name, 'fps': FPS, 'num_frames': len(rows),
                'duration_seconds': len(rows) / FPS, 'joint_names': list(JOINT_NAMES),
                'frames': rows.tolist()}

    def archive(self, name: str) -> dict[str, str]:
        path = archive_motion(self.directory, name)
        return {'file_name': name, 'archived_to': str(path)}
