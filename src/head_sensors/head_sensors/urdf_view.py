"""从 URDF 渲染「相机看到的画面」。

不依赖 ROS，只要 `pinocchio` + `numpy` + `opencv`。连同 `render_head_view.py` 一起拷走就能独立运行。

这件事也**不需要仿真器**：MuJoCo / PyBullet / Gazebo 的价值在动力学积分，
而这里一步物理都不用算 —— 只是「给定关节角，把机器人自己的网格投影到相机像平面」。

    URDF --pinocchio--> 关节树 + 可视网格 --FK--> 各网格在世界系的位姿
         --相机外参--> 相机系顶点 --针孔内参--> 像素 --光栅化--> 彩色图 / 深度图 / 部件图

**渲染的是机器人自身**（URDF 里只有机器人）。所以输出等价于「相机能看到自己身体的
哪些部分」——用于验证挂载变换、算自遮挡掩膜、检查 FOV 覆盖。要往场景里放桌子、
物体、光照和材质，那才轮到真正的渲染器或仿真器。

遮挡使用逐像素 z-buffer，并透视正确地插值三角形顶点的逆深度。彩色图按 URDF 材质、
表面法线和主光源生成明暗，再合成到可选背景图；深度图和部件标签仍只包含机器人自身。
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
        self.colors = [np.asarray(go.meshColor, dtype=np.float32)
                       for go in self.geom_model.geometryObjects]

    @staticmethod
    def _extract(geom_object) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        g = geom_object.geometry
        if hasattr(g, 'num_tris'):
            verts = np.asarray(g.vertices(), dtype=np.float32)
            tris = np.fromiter(
                (idx for i in range(g.num_tris) for idx in _tri(g.tri_indices(i))),
                dtype=np.int32, count=3 * g.num_tris).reshape(-1, 3)
            if np.prod(np.asarray(geom_object.meshScale)) < 0.0:
                tris = tris[:, [0, 2, 1]]
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

    def _project(self, q: np.ndarray, camera: PinholeCamera,
                 cam_rot: np.ndarray, cam_pos: np.ndarray,
                 near: float, far: float):
        """逐几何体 FK + 投影 + 剔除，产出 `(下标, 相机系三角形, 像平面多边形)`。

        相机系三角形是 `(T, 3, 3)` 的 float32，像平面多边形是 `(T, 3, 2)`。
        """
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateGeometryPlacements(self.model, self.data,
                                     self.geom_model, self.geom_data)
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

            yield (gi, cam[tris[visible]].astype(np.float32),
                   np.stack([pu[visible], pv[visible]], axis=2).astype(np.float32))

    def silhouette(self, q: np.ndarray, camera: PinholeCamera,
                   cam_rot: np.ndarray, cam_pos: np.ndarray,
                   near: float = 0.05, far: float = 20.0) -> np.ndarray:
        """只出 `label` 的快速版，实测比 `render` 快约 40 倍（12.3 s -> 0.32 s）。

        **闭合网格的正面三角形之并就是它的剪影**，所以一个几何体内部不需要逐像素
        深度比较，整块交给一次 `cv2.fillPoly` 就行；几何体之间按最近深度从远到近
        画家算法覆盖。`render` 慢在那个逐三角形的 Python 循环上（这一帧 9.7 万个），
        而对齐校验只看轮廓，用不着它给的逐像素深度。

        代价是两个几何体在深度上互相穿插时覆盖顺序会错。刚体自视角下这种情形
        只出现在相邻 link 的接缝处，轮廓不受影响。
        """
        parts = [(float(tri_cam[:, :, 2].min()), gi, poly) for gi, tri_cam, poly
                 in self._project(q, camera, cam_rot, cam_pos, near, far)]
        label = np.full((camera.height, camera.width), -1, dtype=np.int32)
        for _, gi, poly in sorted(parts, key=lambda part: -part[0]):
            cv2.fillPoly(label, np.round(poly * 16.0).astype(np.int32),
                         int(gi), lineType=cv2.LINE_8, shift=4)
        return label

    def render(self, q: np.ndarray, camera: PinholeCamera,
               cam_rot: np.ndarray, cam_pos: np.ndarray,
               near: float = 0.05, far: float = 20.0):
        """渲染一帧。

        返回 `(depth, label, light)`：`depth` 是 float32 米（无命中为 inf），
        `label` 是命中的几何体下标（无命中为 -1），`light` 是表面光照强度。
        """
        polys: List[np.ndarray] = []
        camera_tris: List[np.ndarray] = []
        labels: List[np.ndarray] = []
        lights: List[np.ndarray] = []
        light_dir = np.array([-0.35, -0.45, -1.0], dtype=np.float32)
        light_dir /= np.linalg.norm(light_dir)
        for gi, tri_cam, poly in self._project(q, camera, cam_rot, cam_pos, near, far):
            polys.append(poly)
            camera_tris.append(tri_cam)
            labels.append(np.full(tri_cam.shape[0], gi, dtype=np.int32))
            normals = np.cross(tri_cam[:, 1] - tri_cam[:, 0],
                               tri_cam[:, 2] - tri_cam[:, 0])
            normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-9)
            diffuse = np.maximum(normals @ light_dir, 0.0)
            lights.append(0.32 + 0.78 * diffuse)

        depth = np.full((camera.height, camera.width), np.inf, dtype=np.float32)
        label = np.full((camera.height, camera.width), -1, dtype=np.int32)
        light = np.zeros((camera.height, camera.width), dtype=np.float32)
        if not polys:
            return depth, label, light

        poly = np.concatenate(polys)
        tri_vertices = np.concatenate(camera_tris)
        tri_label = np.concatenate(labels)
        tri_light = np.concatenate(lights)
        for i in range(poly.shape[0]):
            points = poly[i]
            x0 = max(0, int(np.floor(points[:, 0].min())))
            y0 = max(0, int(np.floor(points[:, 1].min())))
            x1 = min(camera.width - 1, int(np.ceil(points[:, 0].max())))
            y1 = min(camera.height - 1, int(np.ceil(points[:, 1].max())))
            if x0 > x1 or y0 > y1:
                continue

            local_poly = np.round((points - [x0, y0]) * 16.0).astype(np.int32)
            mask = np.zeros((y1 - y0 + 1, x1 - x0 + 1), dtype=np.uint8)
            cv2.fillConvexPoly(mask, local_poly, 1, lineType=cv2.LINE_8, shift=4)
            local_y, local_x = np.nonzero(mask)
            if local_x.size == 0:
                continue

            sample_x = local_x + x0
            sample_y = local_y + y0
            px, py = points[:, 0], points[:, 1]
            denominator = ((py[1] - py[2]) * (px[0] - px[2])
                           + (px[2] - px[1]) * (py[0] - py[2]))
            if abs(denominator) < 1e-9:
                continue
            weight0 = ((py[1] - py[2]) * (sample_x - px[2])
                       + (px[2] - px[1]) * (sample_y - py[2])) / denominator
            weight1 = ((py[2] - py[0]) * (sample_x - px[2])
                       + (px[0] - px[2]) * (sample_y - py[2])) / denominator
            weight2 = 1.0 - weight0 - weight1
            inside_triangle = ((weight0 >= -1e-6) & (weight1 >= -1e-6)
                               & (weight2 >= -1e-6))
            inverse_depth = (weight0 / tri_vertices[i, 0, 2]
                             + weight1 / tri_vertices[i, 1, 2]
                             + weight2 / tri_vertices[i, 2, 2])
            inside_triangle &= inverse_depth > 0.0
            pixel_depth = 1.0 / inverse_depth
            image_x = local_x + x0
            image_y = local_y + y0
            nearer = inside_triangle & (pixel_depth < depth[image_y, image_x])
            image_x = image_x[nearer]
            image_y = image_y[nearer]
            depth[image_y, image_x] = pixel_depth[nearer]
            label[image_y, image_x] = tri_label[i]
            light[image_y, image_x] = tri_light[i]
        return depth, label, light


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


def fit_background(image: np.ndarray, width: int, height: int) -> np.ndarray:
    """等比缩放并居中裁切背景图，使它铺满目标画面。"""
    source_height, source_width = image.shape[:2]
    scale = max(width / source_width, height / source_height)
    resized_width = max(width, int(np.ceil(source_width * scale)))
    resized_height = max(height, int(np.ceil(source_height * scale)))
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_AREA)
    x0 = (resized_width - width) // 2
    y0 = (resized_height - height) // 2
    return resized[y0:y0 + height, x0:x0 + width].copy()


def colorize_materials(label: np.ndarray, colors: List[np.ndarray],
                       light: np.ndarray,
                       background: Optional[np.ndarray] = None) -> np.ndarray:
    """按 URDF visual 材质色和表面光照生成 BGR 图。"""
    if background is None:
        out = np.zeros(label.shape + (3,), dtype=np.uint8)
    else:
        out = fit_background(background, label.shape[1], label.shape[0])
    for gi in np.unique(label):
        if gi < 0:
            continue
        mask = label == gi
        linear_rgb = np.power(np.clip(colors[gi][:3], 0.0, 1.0), 2.2)
        reflected = light[mask, None] * linear_rgb
        diffuse = np.clip((light[mask, None] - 0.32) / 0.78, 0.0, 1.0)
        if np.allclose(colors[gi][:3], 0.7, atol=1e-3):
            reflected *= 0.7 + 0.3 * diffuse
            reflected += np.array([0.55, 0.59, 0.65]) * np.power(diffuse, 20.0)
        elif colors[gi][:3].max() < 0.15:
            reflected += 0.008 * np.power(diffuse, 2.0)
        lit_rgb = np.power(np.clip(reflected, 0.0, 1.0), 1.0 / 2.2)
        out[mask] = np.round(lit_rgb[:, ::-1] * 255.0).astype(np.uint8)
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
          background: Optional[str] = None,
          out: Optional[str] = None) -> Shot:
    """URDF 拍照：给一组关节角，渲染该姿态下相机看到的画面。

    `joint_positions` 传 `None` 就用 URDF 中立位；只给部分关节时，其余保持中立位。
    `mesh_dir` 默认取 URDF 所在目录；URDF 里写 `package://pkg/...` 时要传包所在的父目录。
    `extrinsic` 见 `UrdfSceneRenderer.camera_pose`。
    给了 `out` 就同时写出 `{out}_color.png`、`{out}_depth16.png`、
    `{out}_depth.png`、`{out}_parts.png`。
    """
    renderer = UrdfSceneRenderer(urdf, mesh_dir)
    camera = camera or DEFAULT_CAMERA
    q = (renderer.q_from_joint_map(joint_positions) if joint_positions
         else renderer.neutral_q())
    rot, pos = renderer.camera_pose(q, frame, optical=optical, extrinsic=extrinsic)
    depth, label, light = renderer.render(q, camera, rot, pos)
    if out:
        background_image = None
        if background:
            background_image = cv2.imread(background)
            if background_image is None:
                raise ValueError('无法读取背景图片 %r' % background)
        cv2.imwrite(out + '_color.png', colorize_materials(
            label, renderer.colors, light, background_image))
        cv2.imwrite(out + '_depth16.png', depth_to_png16(depth))
        cv2.imwrite(out + '_depth.png', colorize_depth(depth))
        cv2.imwrite(out + '_parts.png', colorize_labels(label, renderer.names))
    return Shot(depth, label, renderer.names, camera, pos)
