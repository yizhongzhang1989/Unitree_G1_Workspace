"""从 URDF 渲染「相机看到的画面」。

不依赖 ROS，只要 `pinocchio` + `numpy` + `opencv`。连同 `render_head_view.py` 一起拷走就能独立运行。

这件事也**不需要仿真器**：MuJoCo / PyBullet / Gazebo 的价值在动力学积分，
而这里一步物理都不用算 —— 只是「给定关节角，把机器人自己的网格投影到相机像平面」。

    URDF --pinocchio--> 关节树 + 可视网格 --FK--> 各网格在世界系的位姿
         --相机外参--> 相机系顶点 --针孔内参--> 像素 --光栅化--> 深度图 / 部件图

**渲染的是机器人自身**（URDF 里只有机器人）。所以输出等价于「相机能看到自己身体的
哪些部分」——用于验证挂载变换、算自遮挡掩膜、检查 FOV 覆盖。要往场景里放桌子、
物体、光照和材质，那才轮到真正的渲染器或仿真器。

深度用**逐三角形常数**（取三角形重心深度）而不是逐像素平面插值：G1 的网格三角形
边长在毫米量级，这个近似带来的深度误差同量级，对掩膜/覆盖分析足够，换来的是能直接
用 OpenCV 的多边形填充、不必在 Python 里逐像素循环。
"""

import json
import os
from typing import Dict, List, NamedTuple, Optional, Tuple

import cv2
import numpy as np
import pinocchio as pin

# ROS 光学坐标系（x 右 / y 下 / z 前）相对于 link 坐标系（x 前 / y 左 / z 上）。
# p_optical = R_OPTICAL_FROM_LINK @ p_link
R_OPTICAL_FROM_LINK = np.array([
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
    [1.0, 0.0, 0.0],
])


class PinholeCamera(NamedTuple):
    """针孔相机内参，字段与 `sensor_msgs/CameraInfo` 一致。

    NamedTuple 白送序列化：`cam._asdict()` 写 JSON，`PinholeCamera(**d)` 读回来。
    """
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_fov(cls, width: int, height: int,
                 hfov_deg: float, vfov_deg: float) -> 'PinholeCamera':
        return cls(width, height,
                   0.5 * width / np.tan(np.radians(hfov_deg) / 2.0),
                   0.5 * height / np.tan(np.radians(vfov_deg) / 2.0),
                   0.5 * width, 0.5 * height)

    def __str__(self) -> str:
        return ('%dx%d, fx=%.2f fy=%.2f cx=%.2f cy=%.2f, FOV %.1f°x%.1f°'
                % (self.width, self.height, self.fx, self.fy, self.cx, self.cy,
                   np.degrees(2.0 * np.arctan(0.5 * self.width / self.fx)),
                   np.degrees(2.0 * np.arctan(0.5 * self.height / self.fy))))


def _cylinder_mesh(radius: float, length: float, segments: int = 16):
    """URDF 的圆柱原语 pinocchio 不给 vertices()，自己三角化。"""
    ang = np.linspace(0.0, 2.0 * np.pi, segments, endpoint=False)
    ring = np.stack([radius * np.cos(ang), radius * np.sin(ang)], axis=1)
    z = 0.5 * length
    verts = np.vstack([np.c_[ring, np.full(segments, -z)],
                       np.c_[ring, np.full(segments, z)],
                       [[0.0, 0.0, -z], [0.0, 0.0, z]]]).astype(np.float32)
    i = np.arange(segments)
    j = (i + 1) % segments
    lo, hi = np.full(segments, 2 * segments), np.full(segments, 2 * segments + 1)
    tris = np.vstack([np.c_[i, j, segments + j], np.c_[i, segments + j, segments + i],
                      np.c_[lo, j, i], np.c_[hi, segments + i, segments + j]])
    return verts, tris.astype(np.int32)


class UrdfSceneRenderer:
    """加载一次 URDF，之后可以按任意关节角和相机位姿反复渲染。"""

    def __init__(self, urdf_path: str, mesh_dir: Optional[str] = None) -> None:
        urdf_path = os.path.abspath(urdf_path)
        # URDF 里的 mesh 路径形如 `g1_description/meshes/x.STL`，相对于 URDF 所在目录。
        if mesh_dir is None:
            mesh_dir = os.path.dirname(urdf_path)
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.geom_model = pin.buildGeomFromUrdf(
            self.model, urdf_path, pin.GeometryType.VISUAL, package_dirs=mesh_dir)
        self.geom_data = self.geom_model.createData()
        self._meshes = [self._extract(go) for go in self.geom_model.geometryObjects]
        self.names = [go.name for go in self.geom_model.geometryObjects]

    @staticmethod
    def _extract(geom_object) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        g = geom_object.geometry
        if hasattr(g, 'num_tris'):
            verts = np.asarray(g.vertices(), dtype=np.float32)
            tris = np.fromiter(
                (idx for i in range(g.num_tris) for idx in _tri(g.tri_indices(i))),
                dtype=np.int32, count=3 * g.num_tris).reshape(-1, 3)
            return verts, tris
        if type(g).__name__ == 'Cylinder':
            return _cylinder_mesh(g.radius, g.halfLength * 2.0)
        return None

    def triangle_count(self) -> int:
        return sum(m[1].shape[0] for m in self._meshes if m is not None)

    def neutral_q(self) -> np.ndarray:
        return pin.neutral(self.model)

    def q_from_joint_map(self, joint_positions: Dict[str, float]) -> np.ndarray:
        """按关节名填 q，URDF 里有但字典里没有的关节保持中立位。"""
        q = pin.neutral(self.model)
        for name, value in joint_positions.items():
            if not self.model.existJointName(name):
                continue
            joint = self.model.joints[self.model.getJointId(name)]
            if joint.nq == 1:
                q[joint.idx_q] = value
        return q

    def camera_pose(self, q: np.ndarray, frame: str,
                    optical: bool = True,
                    extrinsic: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
        """返回相机系在世界系下的 `(R, t)`。

        `optical=True` 时把 link 坐标系（x 前 / y 左 / z 上）转成 ROS 光学坐标系
        （x 右 / y 下 / z 前），与 `*_optical_frame` 一致。

        `extrinsic` 是 `frame` 到实际成像中心的 4x4，给了它就取代 `optical` 那个
        理想旋转。成像中心一般不在挂载 link 原点上——D435i 的彩色镜头就偏 15 mm，
        当成 0 会让渲染横向整体错位——而这个偏置只有 TF 或标定给得出。
        """
        if not self.model.existFrame(frame):
            raise ValueError('URDF 里没有 frame %r' % frame)
        pin.framesForwardKinematics(self.model, self.data, q)
        placement = self.data.oMf[self.model.getFrameId(frame)]
        rot = np.asarray(placement.rotation)
        pos = np.asarray(placement.translation)
        if extrinsic is not None:
            extrinsic = np.asarray(extrinsic, dtype=float)
            return rot @ extrinsic[:3, :3], pos + rot @ extrinsic[:3, 3]
        if optical:
            rot = rot @ R_OPTICAL_FROM_LINK.T
        return rot, pos

    def render(self, q: np.ndarray, camera: PinholeCamera,
               cam_rot: np.ndarray, cam_pos: np.ndarray,
               near: float = 0.05, far: float = 20.0):
        """渲染一帧。

        返回 `(depth, label)`：`depth` 是 float32 米（无命中为 inf），
        `label` 是命中的几何体下标（无命中为 -1）。
        """
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateGeometryPlacements(self.model, self.data,
                                     self.geom_model, self.geom_data)

        polys: List[np.ndarray] = []
        depths: List[float] = []
        labels: List[int] = []
        rot_t = cam_rot.T
        for gi, mesh in enumerate(self._meshes):
            if mesh is None:
                continue
            verts, tris = mesh
            placement = self.geom_data.oMg[gi]
            world = verts @ np.asarray(placement.rotation).T + np.asarray(placement.translation)
            cam = (world - cam_pos) @ rot_t.T

            tri_z = cam[tris, 2]
            keep = (tri_z > near).all(axis=1) & (tri_z < far).all(axis=1)
            if not keep.any():
                continue
            tris = tris[keep]

            u = camera.fx * cam[:, 0] / cam[:, 2] + camera.cx
            v = camera.fy * cam[:, 1] / cam[:, 2] + camera.cy
            pu, pv = u[tris], v[tris]

            # 视锥外整体剔除
            inside = ~((pu < 0).all(1) | (pu >= camera.width).all(1)
                       | (pv < 0).all(1) | (pv >= camera.height).all(1))
            # 背面剔除：像平面上顶点顺序为顺时针的三角形背对相机
            area = ((pu[:, 1] - pu[:, 0]) * (pv[:, 2] - pv[:, 0])
                    - (pu[:, 2] - pu[:, 0]) * (pv[:, 1] - pv[:, 0]))
            visible = inside & (area < 0)
            if not visible.any():
                continue

            tris = tris[visible]
            poly = np.stack([pu[visible], pv[visible]], axis=2)
            polys.append(np.round(poly * 16.0).astype(np.int32))  # cv2 定点 shift=4
            depths.append(cam[tris, 2].mean(axis=1))
            labels.append(np.full(tris.shape[0], gi, dtype=np.int32))

        depth = np.full((camera.height, camera.width), np.inf, dtype=np.float32)
        label = np.full((camera.height, camera.width), -1, dtype=np.int32)
        if not polys:
            return depth, label

        poly = np.concatenate(polys)
        tri_depth = np.concatenate(depths)
        tri_label = np.concatenate(labels)
        # 画家算法：由远及近覆盖，省掉逐像素 z 比较。
        for i in np.argsort(-tri_depth):
            cv2.fillConvexPoly(depth, poly[i], float(tri_depth[i]),
                               lineType=cv2.LINE_8, shift=4)
            cv2.fillConvexPoly(label, poly[i], int(tri_label[i]),
                               lineType=cv2.LINE_8, shift=4)
        return depth, label


def _tri(triangle):
    return (triangle[0], triangle[1], triangle[2])


def colorize_depth(depth: np.ndarray, near: float = 0.1,
                   far: float = 2.0) -> np.ndarray:
    """深度图转 JET 彩色，未命中处为黑。"""
    hit = np.isfinite(depth)
    norm = np.zeros(depth.shape, dtype=np.uint8)
    if hit.any():
        scaled = (depth[hit] - near) / max(far - near, 1e-6)
        norm[hit] = np.clip(scaled * 255.0, 0, 255).astype(np.uint8)
    out = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
    out[~hit] = 0
    return out


def colorize_labels(label: np.ndarray, names: List[str]) -> np.ndarray:
    """按几何体上色，同一个 link 每次运行颜色固定。"""
    out = np.zeros(label.shape + (3,), dtype=np.uint8)
    for gi in np.unique(label):
        if gi < 0:
            continue
        rng = np.random.default_rng(abs(hash(names[gi])) % (2 ** 32))
        out[label == gi] = rng.integers(60, 256, size=3)
    return out


def depth_to_png16(depth: np.ndarray) -> np.ndarray:
    """转成和 RealSense 一致的 16UC1 毫米图，未命中为 0。"""
    out = np.zeros(depth.shape, dtype=np.uint16)
    hit = np.isfinite(depth)
    out[hit] = np.clip(depth[hit] * 1000.0, 0, 65535).astype(np.uint16)
    return out


# G1 头部 D435i 彩色流的实测内参（424x240，已 rectify，畸变为零）。
DEFAULT_CAMERA = PinholeCamera(424, 240, 304.226, 304.385, 215.043, 123.774)


def save_pose(path: str, joints: Dict[str, float], camera: PinholeCamera,
              frame: str, extrinsic: Optional[np.ndarray] = None) -> None:
    """把一次拍照的参数写成 JSON，交给 URDF 拍照端用。"""
    data = {'frame': frame, 'camera': camera._asdict(),
            'joints': {k: float(v) for k, v in joints.items()}}
    if extrinsic is not None:
        data['extrinsic'] = np.asarray(extrinsic, dtype=float).tolist()
    with open(path, 'w') as fp:
        json.dump(data, fp, indent=1, sort_keys=True)


def load_pose(path: str) -> Tuple[Dict[str, float], PinholeCamera, str,
                                  Optional[np.ndarray]]:
    """读回 `save_pose` 写的 JSON，返回 `(joints, camera, frame, extrinsic)`。"""
    with open(path) as fp:
        data = json.load(fp)
    extrinsic = data.get('extrinsic')
    return (data['joints'], PinholeCamera(**data['camera']),
            data.get('frame', 'd435_link'),
            None if extrinsic is None else np.array(extrinsic, dtype=float))


class Shot(NamedTuple):
    depth: np.ndarray          # float32 米，未命中为 inf
    label: np.ndarray          # 命中的几何体下标，未命中为 -1
    names: List[str]           # 几何体名，label 的下标对应它
    camera: PinholeCamera
    cam_pos: np.ndarray        # 相机在世界系的位置


def shoot(joint_positions: Optional[Dict[str, float]] = None, *,
          urdf: str,
          mesh_dir: Optional[str] = None,
          camera: Optional[PinholeCamera] = None,
          frame: str = 'd435_link',
          optical: bool = True,
          extrinsic: Optional[np.ndarray] = None,
          out: Optional[str] = None) -> Shot:
    """URDF 拍照：给一组关节角，渲染该姿态下相机看到的画面。

    `joint_positions` 传 `None` 就用 URDF 中立位；只给部分关节时，其余保持中立位。
    `mesh_dir` 默认取 URDF 所在目录；URDF 里写 `package://pkg/...` 时要传包所在的父目录。
    `extrinsic` 见 `UrdfSceneRenderer.camera_pose`。
    给了 `out` 就同时写出 `{out}_depth16.png`、`{out}_depth.png`、`{out}_parts.png`。
    """
    renderer = UrdfSceneRenderer(urdf, mesh_dir)
    camera = camera or DEFAULT_CAMERA
    q = (renderer.q_from_joint_map(joint_positions) if joint_positions
         else renderer.neutral_q())
    rot, pos = renderer.camera_pose(q, frame, optical=optical, extrinsic=extrinsic)
    depth, label = renderer.render(q, camera, rot, pos)
    if out:
        cv2.imwrite(out + '_depth16.png', depth_to_png16(depth))
        cv2.imwrite(out + '_depth.png', colorize_depth(depth))
        cv2.imwrite(out + '_parts.png', colorize_labels(label, renderer.names))
    return Shot(depth, label, renderer.names, camera, pos)
