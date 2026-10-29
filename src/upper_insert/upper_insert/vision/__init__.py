"""针对特定 ArUco 插孔道具 (ID0 孔板 + ID1 插头 + pos.json) 的 6DoF 视觉定位子模块."""

from upper_insert.vision.cuda_preprocess import CudaPreprocessor
from upper_insert.vision.locate_plug_and_holes import annotate_plug_and_holes
from upper_insert.vision.vision_locator import (
    ARUCO_DICT_ID,
    DEFAULT_CAMERA_CONFIG,
    DEFAULT_POS_JSON,
    MARKER_LENGTH_M,
    PlugHoleTracker,
    R_GRIPPER_FROM_ID1,
    VisionPoseEstimator,
    _PLUG_TO_HOLE_BASIS,
    compute_plug_hole_delta,
    draw_aruco_marker,
    get_marker_object_points,
    load_camera_config,
    load_pos_config,
    rotation_matrix_to_quat,
)
from upper_insert.vision.vision_worker import (
    SharedFrameRef,
    SharedVisionFrame,
    VisionResult,
    initialize_vision_worker,
    process_vision_frame,
)

__all__ = [
    'ARUCO_DICT_ID',
    'CudaPreprocessor',
    'DEFAULT_CAMERA_CONFIG',
    'DEFAULT_POS_JSON',
    'MARKER_LENGTH_M',
    'PlugHoleTracker',
    'R_GRIPPER_FROM_ID1',
    'SharedFrameRef',
    'SharedVisionFrame',
    'VisionPoseEstimator',
    'VisionResult',
    '_PLUG_TO_HOLE_BASIS',
    'annotate_plug_and_holes',
    'compute_plug_hole_delta',
    'draw_aruco_marker',
    'get_marker_object_points',
    'initialize_vision_worker',
    'load_camera_config',
    'load_pos_config',
    'process_vision_frame',
    'rotation_matrix_to_quat',
]
