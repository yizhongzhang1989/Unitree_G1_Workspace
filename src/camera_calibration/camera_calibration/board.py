"""ChArUco 板：把 OpenCV 的 aruco 接口关在这一个文件里。

系统装的是 4.5.4（Ubuntu 22.04 自带），只有旧接口：``CharucoBoard_create`` /
``DetectorParameters_create`` / ``interpolateCornersCharuco``。4.7 起换成了
``CharucoDetector`` 那一套，两边不兼容。全包只有这里 import cv2.aruco，
以后换 OpenCV 只改这一处。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

_MIN_POSE_CORNERS = 6


def dictionary_id(name: str) -> int:
    key = name if name.startswith('DICT_') else f'DICT_{name}'
    value = getattr(cv2.aruco, key, None)
    if value is None:
        raise ValueError(f'OpenCV 不认识字典 {name}')
    return int(value)


@dataclass
class Detection:
    """一张图上的检测结果。corners/ids 是 charuco 角点，marker_* 只用于画叠加图。"""

    corners: np.ndarray                       # (N, 1, 2) float32
    ids: np.ndarray                           # (N, 1) int32
    size: tuple[int, int]                     # (width, height)
    marker_count: int = 0
    marker_corners: list = field(default_factory=list)
    marker_ids: np.ndarray | None = None

    @property
    def count(self) -> int:
        return 0 if self.ids is None else int(len(self.ids))

    def coverage(self) -> float:
        """角点凸包占画面的比例。太小说明板子只覆盖了一小块，标出来的畸变不可信。"""
        if self.count < 3:
            return 0.0
        hull = cv2.convexHull(self.corners.reshape(-1, 2).astype(np.float32))
        area = float(cv2.contourArea(hull))
        return area / float(self.size[0] * self.size[1])

    def to_json(self) -> dict:
        return {
            'corners': self.corners.reshape(-1, 2).tolist(),
            'ids': self.ids.reshape(-1).tolist(),
            'width': self.size[0],
            'height': self.size[1],
            'marker_count': self.marker_count,
        }

    @staticmethod
    def from_json(data: dict) -> 'Detection':
        corners = np.asarray(data['corners'], np.float32).reshape(-1, 1, 2)
        ids = np.asarray(data['ids'], np.int32).reshape(-1, 1)
        return Detection(corners, ids, (int(data['width']), int(data['height'])),
                         int(data.get('marker_count', 0)))


class Board:
    """一块 ChArUco 板。所有尺寸来自 config/board.yaml，这里不写死任何数。"""

    def __init__(self, squares_x: int, squares_y: int, square_size: float,
                 marker_size: float, dictionary: str) -> None:
        if marker_size >= square_size:
            raise ValueError('marker 边长必须小于方格边长')
        self.squares_x = int(squares_x)
        self.squares_y = int(squares_y)
        self.square_size = float(square_size)
        self.marker_size = float(marker_size)
        self.dictionary_name = dictionary
        self._dictionary = cv2.aruco.Dictionary_get(dictionary_id(dictionary))
        self._board = cv2.aruco.CharucoBoard_create(
            self.squares_x, self.squares_y, self.square_size, self.marker_size,
            self._dictionary)
        self._params = cv2.aruco.DetectorParameters_create()
        # 亚像素角点细化：默认 NONE 只给整数级 marker 角，标定 RMS 会差一个量级
        self._params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        needed = self.squares_x * self.squares_y // 2
        capacity = int(self._dictionary.bytesList.shape[0])
        if capacity < needed:
            raise ValueError(
                f'{dictionary} 只有 {capacity} 个 marker，{self.squares_x}x{self.squares_y} '
                f'的板需要 {needed} 个')

    @classmethod
    def from_config(cls, config: dict) -> 'Board':
        section = config['board'] if 'board' in config else config
        return cls(section['squares_x'], section['squares_y'], section['square_size'],
                   section['marker_size'], section['dictionary'])

    @property
    def raw(self):
        """底层 cv2 board 对象，只给 calibrateCameraCharuco 这类需要它的调用"""
        return self._board

    @property
    def corner_count(self) -> int:
        return (self.squares_x - 1) * (self.squares_y - 1)

    @property
    def object_points_all(self) -> np.ndarray:
        return np.asarray(self._board.chessboardCorners, np.float32)

    def object_points(self, ids) -> np.ndarray:
        index = np.asarray(ids, np.int32).reshape(-1)
        return self.object_points_all[index]

    def describe(self) -> dict:
        return {
            'squares_x': self.squares_x, 'squares_y': self.squares_y,
            'square_size': self.square_size, 'marker_size': self.marker_size,
            'dictionary': self.dictionary_name, 'corner_count': self.corner_count,
            'width_m': self.squares_x * self.square_size,
            'height_m': self.squares_y * self.square_size,
        }

    def detect(self, image) -> Detection:
        gray = _gray(image)
        height, width = gray.shape[:2]
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self._dictionary, parameters=self._params)
        empty = Detection(np.zeros((0, 1, 2), np.float32), np.zeros((0, 1), np.int32),
                          (width, height))
        if ids is None or len(ids) == 0:
            return empty
        # 用板的先验把漏检的 marker 捞回来，边角上常能多认出几个
        corners, ids, _, _ = cv2.aruco.refineDetectedMarkers(
            gray, self._board, corners, ids, rejected)
        if ids is None or len(ids) == 0:
            return empty
        count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, self._board)
        if not count or charuco_ids is None:
            empty.marker_count = int(len(ids))
            empty.marker_corners = list(corners)
            empty.marker_ids = ids
            return empty
        return Detection(
            np.asarray(charuco_corners, np.float32).reshape(-1, 1, 2),
            np.asarray(charuco_ids, np.int32).reshape(-1, 1),
            (width, height), int(len(ids)), list(corners), ids)

    def estimate_pose(self, detection: Detection, camera_matrix, distortion):
        """板在相机系下的位姿 T_cam<-board，解不出返回 None。

        板坐标系的 y 轴在板面上"朝上"、+z 指向观察者，所以正对着板拍时
        R[2][2] 约等于 -1，不是 +1。看数别以为解错了。
        """
        if detection.count < _MIN_POSE_CORNERS:
            return None
        from camera_calibration import transforms
        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            detection.corners, detection.ids, self._board,
            np.asarray(camera_matrix, float).reshape(3, 3),
            np.asarray(distortion, float).reshape(-1, 1), None, None)
        if not ok:
            return None
        return transforms.rvec_tvec_to_matrix(rvec, tvec)

    def overlay(self, image, detection: Detection):
        canvas = image.copy()
        if detection.marker_ids is not None and len(detection.marker_corners):
            cv2.aruco.drawDetectedMarkers(canvas, detection.marker_corners,
                                          detection.marker_ids)
        if detection.count:
            cv2.aruco.drawDetectedCornersCharuco(
                canvas, detection.corners, detection.ids, (0, 0, 255))
        return canvas

    def draw(self, pixels_per_meter: float = 4000.0, margin_squares: float = 0.5):
        """生成可打印的板图。默认 4000 px/m ≈ 101.6 dpi 下 1:1。"""
        width = int(round(self.squares_x * self.square_size * pixels_per_meter))
        height = int(round(self.squares_y * self.square_size * pixels_per_meter))
        margin = int(round(margin_squares * self.square_size * pixels_per_meter))
        return self._board.draw((width + 2 * margin, height + 2 * margin), None,
                                margin, 1)


def _gray(image) -> np.ndarray:
    image = np.asarray(image)
    if image.ndim == 2:
        return image
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def _planar_residual(board: Board, detection: Detection) -> float:
    """把板面坐标用单应映到图像，算残差。

    板是平面，成像是投影，所以正确的 id 映射下单应几乎是精确的（只剩镜头畸变，
    几个像素）。字典或朝向猜错时 id 对应的板面位置全乱，残差会到几十上百像素 ——
    这就是探测能区分出唯一正确组合的原因。
    """
    if detection.count < 6:
        return float('inf')
    source = board.object_points(detection.ids)[:, :2].astype(np.float32)
    target = detection.corners.reshape(-1, 2).astype(np.float32)
    matrix, _ = cv2.findHomography(source, target, 0)
    if matrix is None:
        return float('inf')
    projected = cv2.perspectiveTransform(source.reshape(-1, 1, 2), matrix).reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum((projected - target) ** 2, axis=1))))


def probe(image, config: dict) -> list[dict]:
    """遍历候选字典和朝向，按「能用」和残差排序返回。

    实物板的字典/格数朝向拿不准时用这个，别靠猜 —— 猜错了标定照样跑完，
    出来的数悄悄全错。
    """
    section = config.get('probe', {})
    dictionaries = section.get('dictionaries', ['DICT_5X5_100'])
    orientations = section.get('orientations', [[9, 12]])
    base = config['board']
    results = []
    for name in dictionaries:
        for squares_x, squares_y in orientations:
            try:
                board = Board(squares_x, squares_y, base['square_size'],
                              base['marker_size'], name)
            except ValueError as error:                # 字典装不下这么多 marker
                results.append({'dictionary': name, 'squares_x': squares_x,
                                'squares_y': squares_y, 'ok': False,
                                'reason': str(error)})
                continue
            detection = board.detect(image)
            residual = _planar_residual(board, detection)
            results.append({
                'dictionary': name, 'squares_x': squares_x, 'squares_y': squares_y,
                'ok': True, 'markers': detection.marker_count,
                'corners': detection.count, 'max_corners': board.corner_count,
                'residual_px': None if residual == float('inf') else round(residual, 3),
            })
    results.sort(key=lambda r: (not r.get('ok'), -(r.get('corners') or 0),
                                r.get('residual_px') if r.get('residual_px') is not None
                                else float('inf')))
    return results
