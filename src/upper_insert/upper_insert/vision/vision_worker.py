"""Stateful detection and on-demand preview in a spawned, camera-free worker."""

from dataclasses import dataclass
from multiprocessing.util import Finalize
from multiprocessing.shared_memory import SharedMemory
import time

import cv2
import numpy as np

from upper_insert.vision.locate_plug_and_holes import annotate_plug_and_holes
from upper_insert.vision.vision_locator import PlugHoleTracker
from upper_insert.vision.cuda_preprocess import CudaPreprocessor


@dataclass(frozen=True)
class SharedFrameRef:
    name: str
    shape: tuple[int, int, int]


class SharedVisionFrame:
    def __init__(self) -> None:
        self._memory: SharedMemory | None = None

    def write(self, frame: np.ndarray) -> SharedFrameRef:
        if (frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3
                or frame.size == 0):
            raise ValueError('Shared vision frame requires non-empty uint8 HxWx3 BGR')
        if self._memory is None or self._memory.size < frame.nbytes:
            self.close()
            self._memory = SharedMemory(create=True, size=frame.nbytes)
        target = np.ndarray(frame.shape, dtype=np.uint8, buffer=self._memory.buf)
        np.copyto(target, frame)
        return SharedFrameRef(self._memory.name, frame.shape)

    def close(self) -> None:
        if self._memory is not None:
            memory, self._memory = self._memory, None
            memory.close()
            memory.unlink()


@dataclass
class VisionResult:
    poses: dict[int, tuple[np.ndarray, np.ndarray]]
    algo_ms: float
    jpeg: bytes = b''
    preview_error: str = ''
    cpu_ms: float = 0.0
    preprocess_backend: str = 'cpu'


_tracker: PlugHoleTracker | None = None
_frame_memory: SharedMemory | None = None
_last_id0_pose: tuple[np.ndarray, np.ndarray] | None = None


def _close_frame_memory() -> None:
    global _frame_memory
    if _frame_memory is not None:
        _frame_memory.close()
        _frame_memory = None


def initialize_vision_worker(
    pos_json: str, camera_config: str, preprocess_backend: str = 'cpu',
) -> None:
    global _tracker, _last_id0_pose
    if preprocess_backend not in ('cpu', 'cuda'):
        raise ValueError('vision_preprocess must be cpu or cuda')
    cv2.setNumThreads(1)
    Finalize(None, _close_frame_memory, exitpriority=10)
    _last_id0_pose = None
    _tracker = PlugHoleTracker(pos_json=pos_json, camera_config=camera_config, open_camera=False)
    if preprocess_backend == 'cuda':
        preprocessor = CudaPreprocessor()
        Finalize(None, preprocessor.close, exitpriority=10)
        _tracker.estimator.color_preprocessor = preprocessor


def process_vision_frame(
    frame: np.ndarray | SharedFrameRef, preview: bool, hole: str | None, depth_mm: float,
) -> VisionResult:
    global _frame_memory, _last_id0_pose
    if _tracker is None:
        raise RuntimeError('Vision worker is not initialized')
    if isinstance(frame, SharedFrameRef):
        if _frame_memory is None or _frame_memory.name != frame.name:
            _close_frame_memory()
            _frame_memory = SharedMemory(name=frame.name)
        frame = np.ndarray(frame.shape, dtype=np.uint8, buffer=_frame_memory.buf)
    started = time.perf_counter()
    cpu_started = time.process_time()
    req_ids = (1,) if _last_id0_pose is not None else (0, 1)
    _, poses = _tracker.estimator.estimate_poses(frame, draw=False, required_ids=req_ids)
    if 0 in poses:
        _last_id0_pose = (poses[0][0].copy(), poses[0][1].copy())
    elif _last_id0_pose is not None:
        poses[0] = (_last_id0_pose[0].copy(), _last_id0_pose[1].copy())
    result = VisionResult(
        poses, (time.perf_counter() - started) * 1000.0,
        cpu_ms=(time.process_time() - cpu_started) * 1000.0,
        preprocess_backend='cuda' if _tracker.estimator.color_preprocessor is not None else 'cpu',
    )
    if preview:
        try:
            image, _ = annotate_plug_and_holes(
                frame, _tracker.cam_cfg['camera_matrix'], _tracker.cam_cfg['dist_coeffs'],
                _tracker.holes_mm, _tracker.plug_mm, target_hole=hole,
                insert_depth_mm=depth_mm, poses=poses,
            )
            ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                result.jpeg = encoded.tobytes()
            else:
                result.preview_error = '网页预览 JPEG 编码失败'
        except cv2.error as error:
            result.preview_error = f'网页预览绘制失败: {error}'
    return result
