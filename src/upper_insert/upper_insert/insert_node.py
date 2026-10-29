"""50Hz 上肢控制、独立限频视觉与力觉输入，以及 Web 控制台。"""

from __future__ import annotations

from collections import deque
from concurrent.futures import ProcessPoolExecutor, TimeoutError as FutureTimeout
from concurrent.futures.process import BrokenProcessPool
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import multiprocessing
import signal
import subprocess
import threading
import time
from typing import Any
from urllib.parse import urlsplit

import cv2
from geometry_msgs.msg import WrenchStamped
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, ExtrapolationException, TransformException, TransformListener

from g1_motion_session.arm_ik_mixin import ArmIKMixin, IKPublicStatus
from g1_motion_session.task_base import State, TaskBase
from upper_insert import package_file
from upper_insert.insert_policy import (
    DEFAULT_FILTER_SECONDS,
    DEFAULT_FORCE_LIMIT_N,
    DEFAULT_MOMENT_LIMIT_NM,
    DEFAULT_POLICY_ONNX,
    DEFAULT_ROTATION_LIMIT_RAD,
    DEFAULT_TRANSLATION_LIMIT_M,
    InsertPolicy,
    WrenchProcessor,
)
from upper_insert.tool_calibration import RobotToolCalibrator, _orthonormalize_rotation as _orthonormalize_rot
from upper_insert.vision import (
    DEFAULT_CAMERA_CONFIG,
    DEFAULT_POS_JSON,
    MARKER_LENGTH_M,
    PlugHoleTracker,
    SharedVisionFrame,
    _PLUG_TO_HOLE_BASIS,
    compute_plug_hole_delta,
    get_marker_object_points,
    initialize_vision_worker,
    process_vision_frame,
    rotation_matrix_to_quat,
)

SIDES = ('left', 'right')
MODES = ('float', 'hold', 'mech_insert', 'policy_insert')
PREVIEW_FPS = 5.0
TOOL_CALIBRATION_TF_TIMEOUT_S = 0.25
HEAD_CAMERA_TF_MAX_OFFSET_S = 1.0
HEAD_CAMERA_CAPTURE_TIMEOUT_S = 2.0


def _require_mech_calibration(calibration: dict[str, Any]) -> None:
    if calibration.get('source') != 'head_camera_tf':
        raise ValueError(
            '机械预定位拒绝旧版/名义工具标定；请在停止运动后使用主相机与发布的 TF '
            '重新标定，核验工具长度和方向。不要直接恢复默认标定后执行运动。'
        )


class InsertStatus(IKPublicStatus):
    upper_source: str
    gripper_source: str
    mode: str
    active_arm: str
    policy_loaded: bool
    vision_valid: bool
    target_hole: str
    insert_depth_mm: float
    current_depth_mm: float
    delta_pos: list[float]
    delta_quat: list[float]
    last_action: list[float]
    command_pose: dict[str, list[float]]
    grip: list[float]
    joint_positions: list[float]


def _mat_to_quat(rot_mat: np.ndarray) -> np.ndarray:
    coeffs = np.asarray(pin.Quaternion(np.asarray(rot_mat, dtype=np.float64)).coeffs(), dtype=np.float64)
    norm = float(np.linalg.norm(coeffs))
    return coeffs / norm if norm > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)


def _apply_rotvec(quat_xyzw: np.ndarray, rotvec: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-9:
        return quat_xyzw
    rot_cur = pin.Quaternion(np.asarray(quat_xyzw, dtype=np.float64)).matrix()
    rot_delta = pin.exp3(np.asarray(rotvec, dtype=np.float64))
    rot_next = rot_cur @ rot_delta
    return _mat_to_quat(rot_next)


def _input_health(
    stamp: float, now: float, timeout_s: float, *, valid: bool = True
) -> dict[str, Any]:
    received = stamp > 0.0
    age = max(0.0, now - stamp) if received else None
    fresh = age is not None and age <= timeout_s
    status = 'missing' if not received else 'invalid' if not valid else 'stale' if not fresh else 'ready'
    return {
        'received': received,
        'valid': received and valid,
        'fresh': fresh,
        'ready': status == 'ready',
        'status': status,
        'age_ms': round(age * 1000.0, 2) if age is not None else None,
        'timeout_ms': round(timeout_s * 1000.0, 2),
    }


class _DashboardHTTPServer(ThreadingHTTPServer):
    request_queue_size = 128
    allow_reuse_address = True

    def __init__(self, server_address: tuple[str, int], task_node: 'InsertTask'):
        self.task_node = task_node
        super().__init__(server_address, _DashboardHTTPHandler)


class _DashboardHTTPHandler(BaseHTTPRequestHandler):
    def log_message(self, format: str, *args: Any) -> None:  # noqa: A002
        return

    def _send_json(self, status_code: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(status_code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        server: _DashboardHTTPServer = self.server  # type: ignore[assignment]
        task = server.task_node
        path = urlsplit(self.path).path
        try:
            if path in ('/', '/index.html', '/dashboard'):
                html_path = package_file('static/dashboard.html')
                body = html_path.read_bytes()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path in ('/api/state', '/status'):
                self._send_json(200, task.dashboard_state())
                return
            if path in ('/snapshot.jpg', '/head-calibration.jpg'):
                if path == '/head-calibration.jpg':
                    with task._sensor_lock:
                        jpg = task._head_calibration_jpeg
                else:
                    jpg = task.latest_jpeg()
                    deadline = time.monotonic() + 0.5
                    while not jpg and task._vision_alive and time.monotonic() < deadline:
                        time.sleep(0.01)
                        jpg = task.latest_jpeg()
                if not jpg:
                    self.send_error(503, 'No camera frame available')
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(jpg)))
                self.end_headers()
                self.wfile.write(jpg)
                return
            if path == '/stream.mjpg':
                self.send_response(200)
                self.send_header('Cache-Control', 'no-cache, private')
                self.send_header('Pragma', 'no-cache')
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                last_jpg = None
                while task._vision_alive:
                    jpg = task.latest_jpeg()
                    if jpg and jpg is not last_jpg:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(f'Content-Length: {len(jpg)}\r\n\r\n'.encode('ascii'))
                        self.wfile.write(jpg)
                        self.wfile.write(b'\r\n')
                        last_jpg = jpg
                    time.sleep(1.0 / PREVIEW_FPS)
                return
            self._send_json(404, {'ok': False, 'error': f'未知路径: {path}'})
        except (BrokenPipeError, ConnectionResetError):
            return
        except Exception as err:
            try:
                self._send_json(500, {'ok': False, 'error': str(err)})
            except Exception:
                pass

    def do_POST(self) -> None:
        server: _DashboardHTTPServer = self.server  # type: ignore[assignment]
        task = server.task_node
        path = urlsplit(self.path).path
        try:
            if path != '/api/command':
                self._send_json(404, {'ok': False, 'error': f'未知接口: {path}'})
                return
            length = int(self.headers.get('Content-Length', '0'))
            if length <= 0 or length > 65536:
                self._send_json(400, {'ok': False, 'error': '请求体长度非法'})
                return
            raw = self.rfile.read(length)
            payload = json.loads(raw.decode('utf-8'))
            if not isinstance(payload, dict):
                raise ValueError('请求体必须为 JSON 对象')
            result = task.handle_dashboard_command(payload)
            self._send_json(200, {'ok': True, **result, 'state': task.dashboard_state()})
        except ValueError as err:
            self._send_json(400, {'ok': False, 'error': str(err)})
        except Exception as err:
            self._send_json(500, {'ok': False, 'error': str(err)})


class InsertTask(TaskBase, ArmIKMixin):
    quat_order: str = 'wxyz'
    max_policy_rot_rad: float = 0.0
    vision_fk_alpha_pos: float = 0.45
    vision_fk_alpha_rot: float = 0.35
    vision_fk_coast_s: float = 0.0
    max_tip_lead_m: float = 0.012
    max_tip_lead_rad: float = math.radians(4.0)
    _last_id0_pose: tuple[np.ndarray, np.ndarray] | None = None
    _fused_hole_torso: dict[str, Any] | None = None
    _vision_cfg_seq: int = 0
    _policy_step_count: int = 0
    _policy_tip_cmd: np.ndarray | None = None
    _last_policy_step_info: dict[str, Any] | None = None

    def __init__(self) -> None:
        super().__init__('upper_insert', slots=range(15, 31), imu_required=False)
        self.init_arm_ik()
        if set(self.arm_slots) != set(range(15, 29)):
            raise ValueError('arm_joints must cover the 14 arm slots')

        self.declare('gripper_joints', self.joints31[29:])
        grippers = list(
            self.get_parameter('gripper_joints').get_parameter_value().string_array_value
        )
        self.gripper_slots = [self.joints31.index(name) for name in grippers]
        if len(grippers) != 2 or set(self.gripper_slots) != {29, 30}:
            raise ValueError('gripper_joints must cover the two gripper slots')
        self.declare('gripper_limits', [0.0, 2.76377472169236])
        self._grip_lo, self._grip_hi = (
            self.get_parameter('gripper_limits').get_parameter_value().double_array_value
        )
        self._grip_closed = float(self._grip_lo)
        self._grip_open = float(self._grip_hi)

        self.active_arm = self.declare('active_arm', 'right')
        if self.active_arm not in SIDES:
            raise ValueError(f'active_arm must be in {SIDES}, got {self.active_arm!r}')

        self.default_mode = self.declare('default_mode', 'float')
        if self.default_mode not in MODES:
            raise ValueError(f'default_mode must be in {MODES}, got {self.default_mode!r}')
        self.mode: str = self.default_mode

        self.model_path = DEFAULT_POLICY_ONNX
        self.camera_config = self.declare('camera_config', DEFAULT_CAMERA_CONFIG)
        self.pos_json = self.declare('pos_json', DEFAULT_POS_JSON)
        raw_hole = self.declare('target_hole', 'auto').strip()
        self.target_hole = raw_hole if raw_hole else 'auto'
        self.insert_depth_mm = float(self.declare('insert_depth_mm', 15.0))
        self.align_orientation = bool(self.declare('align_orientation', True))
        self.quat_order = self.declare('quat_order', 'wxyz')

        self.wrench_order = self.declare('wrench_order', 'force_torque')
        self.left_wrench_topic = self.declare('left_wrench_topic', '/arm0/wrench_net')
        self.right_wrench_topic = self.declare('right_wrench_topic', '/arm1/wrench_net')
        self.wrench_timeout_s = float(self.declare('wrench_timeout_s', 0.2))
        self.wrench_filter_tau_s = float(
            self.declare('wrench_filter_tau_s', DEFAULT_FILTER_SECONDS)
        )
        self.force_limit_n = float(self.declare('force_limit_n', DEFAULT_FORCE_LIMIT_N))
        self.moment_limit_nm = float(self.declare('moment_limit_nm', DEFAULT_MOMENT_LIMIT_NM))
        self.vision_timeout_s = float(self.declare('vision_timeout_s', 0.5))
        self.open_camera = bool(self.declare('open_camera', True))
        self.vision_preprocess = self.declare('vision_preprocess', 'cuda')
        if self.vision_preprocess not in ('cpu', 'cuda'):
            raise ValueError('vision_preprocess must be cpu or cuda')
        self.vision_rate_hz = float(self.declare('vision_rate_hz', 30.0))
        if not math.isfinite(self.vision_rate_hz) or self.vision_rate_hz <= 0.0:
            raise ValueError('vision_rate_hz must be finite and positive')
        self.camera_index = int(self.declare('camera_index', -1))
        self.action_scale = float(self.declare('action_scale', 1.0))
        self.max_step_m = float(self.declare('max_step_m', DEFAULT_TRANSLATION_LIMIT_M))
        self.max_step_rad = float(self.declare('max_step_rad', DEFAULT_ROTATION_LIMIT_RAD))
        self.max_policy_rot_deg = float(self.declare('max_policy_rot_deg', 1.5))
        if not math.isfinite(self.max_policy_rot_deg) or self.max_policy_rot_deg < 0.0:
            raise ValueError('max_policy_rot_deg must be finite and non-negative')
        self.max_policy_rot_rad = math.radians(self.max_policy_rot_deg)
        self.vision_fk_alpha_pos = float(self.declare('vision_fk_alpha_pos', 0.35))
        self.vision_fk_alpha_rot = float(self.declare('vision_fk_alpha_rot', 0.25))
        self.vision_fk_coast_s = float(self.declare('vision_fk_coast_s', 1.0))
        self.policy_step_scale = float(self.declare('policy_step_scale', 0.50))
        self.max_tip_lead_m = float(self.declare('max_tip_lead_m', 0.012))
        self.max_tip_lead_rad = math.radians(float(self.declare('max_tip_lead_deg', 4.0)))
        self.web_host = self.declare('web_host', '0.0.0.0')
        self.web_port = int(self.declare('web_port', 8088))
        policy_node = self.declare('policy_node', '/motion_control').rstrip('/')

        self.tool_calibrator = RobotToolCalibrator(pos_json=self.pos_json)
        self.policy = InsertPolicy(
            self.model_path,
            wrench_order=self.wrench_order,
            action_scale=self.action_scale,
            max_step_m=self.max_step_m,
            max_step_rad=self.max_step_rad,
        )

        self._sensor_lock = threading.Lock()
        self._wrench: dict[str, tuple[np.ndarray, np.ndarray, float]] = {
            side: (np.zeros(3), np.zeros(3), 0.0) for side in SIDES
        }
        self._wrench_net_stamp: dict[str, float] = {side: 0.0 for side in SIDES}
        self._wrench_source: dict[str, str] = {side: 'none' for side in SIDES}
        self._wrench_inputs: dict[str, dict[str, tuple[float, bool]]] = {
            side: {'raw': (0.0, False), 'net': (0.0, False)} for side in SIDES
        }
        self._wrench_bias: dict[str, tuple[np.ndarray, np.ndarray]] = {
            side: (np.zeros(3), np.zeros(3)) for side in SIDES
        }
        self._wrench_proc: dict[str, WrenchProcessor] = {
            side: WrenchProcessor(
                tau_s=self.wrench_filter_tau_s,
                force_limit_n=self.force_limit_n,
                moment_limit_nm=self.moment_limit_nm,
            )
            for side in SIDES
        }
        self._safety_warning: str = ''
        self._ft_calib_running: bool = False
        self._ft_calib_status: str = '已就绪'
        self._ft_calib_error: str = ''
        self._ft_calib_proc: subprocess.Popen[str] | None = None
        self._head_image: Image | None = None
        self._head_camera_info: CameraInfo | None = None
        self._head_calibration_jpeg: bytes = b''
        self._head_capture_lock = threading.Lock()
        self._head_capture_requested = threading.Event()
        self._head_capture_ready = threading.Event()
        self._tf_buffer = Buffer(node=self)
        self._vision_state: dict[str, Any] = {
            'valid': False,
            'seq': 0,
            'stamp': 0.0,
            'stamp_source': 'none',
            'fps': 0.0,
            'processed_frames': 0,
            'processed_frame_age_ms': 0.0,
            'algo_ms': 0.0,
            'algo_cpu_ms': 0.0,
            'preprocess_backend': 'not_started',
            'target_hole': self.target_hole,
            'nearest_hole': '',
            'distance_mm': math.nan,
            'lateral_err_mm': math.nan,
            'depth_err_mm': math.nan,
            'insert_depth_mm': self.insert_depth_mm,
            'current_depth_mm': math.nan,
            'plug_in_id0_mm': np.zeros(3, dtype=np.float64),
            'hole_in_id0_mm': np.zeros(3, dtype=np.float64),
            'axis_angle_deg': math.nan,
            'yaw_angle_deg': math.nan,
            'orientation_ok': False,
            'delta_pos': np.zeros(3, dtype=np.float64),
            'delta_quat': np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            'delta_quat_wxyz': np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            'R_HT': np.eye(3, dtype=np.float64),
            'delta_id0_m': np.zeros(3, dtype=np.float64),
            'delta_id1_m': np.zeros(3, dtype=np.float64),
            'delta_rot_id1': np.eye(3, dtype=np.float64),
            'r1_t_r0': _PLUG_TO_HOLE_BASIS.copy(),
            'capture_gripper_pose': None,
            'missing_ids': [0, 1],
            'error': '',
        }
        self._jpeg_bytes: bytes = b''
        self._last_id0_pose: tuple[np.ndarray, np.ndarray] | None = None
        self._joint_history: deque[tuple[float, np.ndarray]] = deque(maxlen=512)
        self._fused_hole_torso: dict[str, Any] | None = None
        self._vision_cfg_seq: int = 0
        self._preview_requested_until = 0.0
        self._target_stamps: deque[float] = deque(maxlen=251)
        self._last_action = np.zeros(3, dtype=np.float64)
        self._last_action_6d = np.zeros(6, dtype=np.float64)
        self._policy_step_count = 0
        self._last_policy_step_info: dict[str, Any] | None = None
        self._pose: dict[str, np.ndarray] = {}
        self._mech_goal_pose: np.ndarray | None = None
        self._grip = np.zeros(2, dtype=np.float64)
        self._seeded = False

        wrench_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        slow_group = ReentrantCallbackGroup()
        self._subscribe_wrench_inputs(wrench_qos)
        self._head_qos = wrench_qos
        self.create_subscription(
            CameraInfo, '/head/camera/color/camera_info', self._on_head_camera_info,
            wrench_qos, callback_group=slow_group,
        )
        self.create_subscription(
            Float64MultiArray,
            '/upper_insert/localization_input',
            self._on_external_localization,
            wrench_qos,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self._loc_pub = self.create_publisher(
            Float64MultiArray, '/upper_insert/localization', wrench_qos
        )
        self._detail_pub = self.create_publisher(String, '/upper_insert/status', 10)
        self._session_clients = {
            name: self.create_client(Trigger, f'{policy_node}/{name}', callback_group=slow_group)
            for name in ('engage', 'start', 'stop', 'estop')
        }
        self._gripper_enable_clients = {
            side: self.create_client(Trigger, f'/grip_arm{i}/enable', callback_group=slow_group)
            for i, side in enumerate(SIDES)
        }

        self._vision_alive = True
        self._vision_executor: ProcessPoolExecutor | None = None
        self._vision_frames = SharedVisionFrame()
        self.tracker = PlugHoleTracker(
            pos_json=self.pos_json,
            camera_config=self.camera_config,
            target_hole=None if self.target_hole == 'auto' else self.target_hole,
            open_camera=self.open_camera,
            camera_index=None if self.camera_index < 0 else self.camera_index,
            camera_decode_rate_hz=self.vision_rate_hz,
        )
        self._vision_thread: threading.Thread | None = None
        if self.open_camera:
            self._vision_executor = ProcessPoolExecutor(
                max_workers=1, mp_context=multiprocessing.get_context('spawn'),
                initializer=initialize_vision_worker,
                initargs=(self.pos_json, self.camera_config, self.vision_preprocess),
            )
            self._vision_thread = threading.Thread(target=self._vision_loop, daemon=True)
            self._vision_thread.start()

        self._http_server: _DashboardHTTPServer | None = None
        self._http_thread: threading.Thread | None = None
        if self.web_port > 0:
            try:
                self._http_server = _DashboardHTTPServer((self.web_host, self.web_port), self)
                self._http_thread = threading.Thread(
                    target=self._http_server.serve_forever, daemon=True
                )
                self._http_thread.start()
                self.get_logger().info(
                    f'Upper Insert Dashboard 已启动: http://{self.web_host}:{self.web_port}'
                )
            except OSError as err:
                self.get_logger().error(f'Dashboard 端口 {self.web_port} 启动失败: {err}')

        self.create_timer(self.dt, self._control, callback_group=self.control_group)

    @property
    def ready(self) -> bool:
        return self.ik_ready

    def _bind_arm_ik(self, message: String) -> None:
        super()._bind_arm_ik(message)
        with self.lock:
            if self._ik is not None and hasattr(self._ik, '_null_terms'):
                for side, (ungated, ref, gains) in list(self._ik._null_terms.items()):
                    self._ik._null_terms[side] = (
                        np.zeros_like(ungated, dtype=bool),
                        ref,
                        gains,
                    )

    def prepared(self, snapshot) -> None:
        self.reset_ik(snapshot.position)
        self._grip[:] = snapshot.position[self.gripper_slots]
        self.policy.reset()
        self.mode = 'float'
        self._mech_goal_pose = None
        self._seeded = False
        self._seed_from_ik()
        self.state = State.RUNNING

    def stop(self) -> None:
        self._seeded = False
        self.mode = 'float'
        self._mech_goal_pose = None
        self.policy.reset()
        self._last_action[:] = 0.0

    def latest_jpeg(self) -> bytes:
        with self._sensor_lock:
            self._preview_requested_until = time.monotonic() + 1.0
            return self._jpeg_bytes

    def publish_target(self, positions=None) -> bool:
        published = super().publish_target(positions)
        if published:
            with self.lock:
                self._target_stamps.append(time.monotonic())
        return published

    def target_timing(self) -> dict[str, Any]:
        with self.lock:
            stamps = list(self._target_stamps)
        intervals = np.diff(stamps) * 1000.0
        return {
            'configured_hz': round(1.0 / self.dt, 2),
            'measured_hz': round(1000.0 / float(intervals.mean()), 2) if len(intervals) else None,
            'p95_gap_ms': round(float(np.percentile(intervals, 95)), 2) if len(intervals) else None,
            'max_gap_ms': round(float(intervals.max()), 2) if len(intervals) else None,
            'last_target_age_ms': round(max(0.0, time.monotonic() - stamps[-1]) * 1000.0, 2)
            if stamps else None,
            'samples': len(stamps),
        }

    def _subscribe_wrench_inputs(self, qos: QoSProfile) -> None:
        net_group = MutuallyExclusiveCallbackGroup()
        raw_group = MutuallyExclusiveCallbackGroup()
        for index, (side, topic) in enumerate(zip(SIDES, (self.left_wrench_topic, self.right_wrench_topic))):
            is_net = topic.endswith('wrench_net')
            raw_topic = f'/arm{index}/wrench_raw'
            self.create_subscription(
                WrenchStamped, topic, partial(self._on_wrench, side, is_net=is_net), qos,
                callback_group=net_group if is_net else raw_group,
            )
            if topic != raw_topic:
                self.create_subscription(
                    WrenchStamped, raw_topic,
                    partial(self._on_wrench, side, is_net=False, fallback_only=True),
                    qos, callback_group=raw_group,
                )

    def _current_arm_q7(self, side: str) -> tuple[np.ndarray, str]:
        now = time.monotonic()
        idx_start = 0 if side == 'left' else 7
        arm_indices = self.arm_slots[idx_start: idx_start + 7]
        if self._joint_stamp > 0.0 and (now - self._joint_stamp) <= 1.0:
            q_js = self._q[arm_indices]
            if np.all(np.isfinite(q_js)):
                return q_js.copy(), 'joint_states'
        return self.output.positions[arm_indices].copy(), 'nominal'

    def _compensate_raw_wrench_in_sensor(
        self, side: str, f_raw: np.ndarray, t_raw: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        ft = self.tool_calibrator.ft_calibs[side]
        q_arm7, _ = self._current_arm_q7(side)
        fk = self.tool_calibrator.compute_arm_fk(side, q_arm7)
        R_torso_S = fk['T_torso_S'][:3, :3]
        f_grav_s = R_torso_S.T @ np.array([0.0, 0.0, -ft.tool_mass * 9.81])
        return (f_raw - ft.force_bias - f_grav_s,
                t_raw - ft.torque_bias - np.cross(ft.tool_com, f_grav_s))

    def _on_wrench(
        self, side: str, message: WrenchStamped, *, is_net: bool = True,
        fallback_only: bool = False,
    ) -> None:
        f = message.wrench.force
        t = message.wrench.torque
        now = time.monotonic()
        valid = all(map(math.isfinite, (f.x, f.y, f.z, t.x, t.y, t.z)))
        with self._sensor_lock:
            self._wrench_inputs[side]['net' if is_net else 'raw'] = (now, valid)
            net_stamp = self._wrench_net_stamp[side]
        if not valid:
            self.get_logger().error(
                f'{side} wrench contains non-finite values', throttle_duration_sec=2.0
            )
            return
        if fallback_only and net_stamp > 0.0 and now - net_stamp <= self.wrench_timeout_s:
            return
        force = np.array([f.x, f.y, f.z], dtype=np.float64)
        torque = np.array([t.x, t.y, t.z], dtype=np.float64)
        if not is_net:
            force, torque = self._compensate_raw_wrench_in_sensor(side, force, torque)
        with self._sensor_lock:
            # Net callbacks may finish while raw compensation runs in its own group.
            net_stamp = self._wrench_net_stamp[side]
            if (fallback_only and net_stamp > 0.0
                    and time.monotonic() - net_stamp <= self.wrench_timeout_s):
                return
            if is_net:
                self._wrench_net_stamp[side] = now
            self._wrench_source[side] = 'wrench_net' if is_net else 'wrench_raw+urdf_grav'
            self._wrench[side] = (force, torque, now)
            f_bias, t_bias = self._wrench_bias[side]
            self._wrench_proc[side].update_sample(
                force - f_bias, torque - t_bias, stamp_s=now
            )

    def _capture_head_inputs(self, arm: str) -> dict[str, Any]:
        if not self._head_capture_lock.acquire(blocking=False):
            raise ValueError('主相机标定正在采集，请勿重复请求')
        try:
            with self._sensor_lock:
                self._head_image = None
            self._head_capture_ready.clear()
            self._head_capture_requested.set()
            capture_node = Node(
                'upper_insert_head_capture', context=self.context, namespace=self.get_namespace(),
                use_global_arguments=False, enable_rosout=False, start_parameter_services=False,
            )
            capture_executor = SingleThreadedExecutor(context=self.context)
            capture_listener = None
            try:
                capture_listener = self._create_capture_listener(capture_node)
                capture_node.create_subscription(
                    Image, self.resolve_topic_name('/head/camera/color/image_raw'),
                    self._on_head_image, self._head_qos,
                )
                capture_executor.add_node(capture_node)
                deadline = time.monotonic() + HEAD_CAMERA_CAPTURE_TIMEOUT_S
                while True:
                    if not self.context.ok():
                        raise RuntimeError('ROS 已关闭，主相机标定采集已取消')
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        if self._head_image is not None:
                            raise ValueError('等待主相机图像对应 TF 超时，请检查 TF 发布')
                        raise ValueError('等待主相机新图像超时，请检查主相机驱动和图像话题')
                    capture_executor.spin_once(timeout_sec=min(0.1, remaining))
                    if self._head_capture_ready.is_set():
                        with self._sensor_lock:
                            image = self._head_image
                        if image is not None and self._capture_transforms_ready(arm, image):
                            break
                        if time.monotonic() - self._head_capture_received_s > .1:
                            self._head_capture_ready.clear()
            finally:
                # No other thread spins this executor; all take/callback work has returned.
                capture_executor.remove_node(capture_node)
                capture_executor.shutdown()
                if capture_listener is not None:
                    capture_listener.unregister()
                capture_node.destroy_node()
            return self._head_calibration_inputs(arm)
        finally:
            self._head_capture_requested.clear()
            self._head_capture_lock.release()

    def _create_capture_listener(self, node: Node) -> TransformListener:
        return TransformListener(self._tf_buffer, node, qos=10)

    def _capture_transforms_ready(self, arm: str, image: Image) -> bool:
        stamp = Time.from_msg(image.header.stamp)
        return (
            self._tf_buffer.can_transform(self.base_frame, self.tip_frames[arm], stamp)
            and self._tf_buffer.can_transform(self.base_frame, f'{arm}_kwr57b_link', stamp)
            and self._tf_buffer.can_transform(self.base_frame, image.header.frame_id, Time())
        )

    def _on_head_image(self, message: Image) -> None:
        if not self._head_capture_requested.is_set() or self._head_capture_ready.is_set():
            return
        with self._sensor_lock:
            self._head_image = message
            self._head_capture_received_s = time.monotonic()
        self._head_capture_ready.set()

    def _on_head_camera_info(self, message: CameraInfo) -> None:
        with self._sensor_lock:
            self._head_camera_info = message

    def _head_calibration_inputs(self, arm: str) -> dict[str, Any]:
        with self._sensor_lock:
            message = self._head_image
            info = self._head_camera_info
        if message is None or info is None:
            raise ValueError('主相机图像或 CameraInfo 缺失')
        stamp, frame = Time.from_msg(message.header.stamp), message.header.frame_id
        label = f'{arm} | frame {stamp.nanoseconds * 1e-9:.3f}s'
        enc = message.encoding.lower()
        channels = 2 if enc in ('yuv422_yuy2', 'yuyv') else 3
        pixels = np.ndarray(
            (message.height, message.width, channels), dtype=np.uint8,
            buffer=message.data, strides=(message.step, channels, 1),
        )
        if enc == 'rgb8':
            image = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
        elif channels == 2:
            image = cv2.cvtColor(pixels, cv2.COLOR_YUV2BGR_YUYV)
        else:
            image = pixels.copy()
        self._save_head_calibration_preview(image, label)
        image_age = (self.get_clock().now().nanoseconds - stamp.nanoseconds) * 1e-9
        if image_age < -0.1 or image_age > 1.0:
            raise ValueError('主相机图像时间戳过期或时钟不同步')
        inputs: dict[str, Any] = {
            'frame_bgr': image,
            'head_K': np.asarray(info.k, dtype=np.float64).reshape(3, 3),
            'head_dist': np.asarray(info.d, dtype=np.float64),
            'camera_tf_time_offset_s': 0.0,
            'capture_label': label,
        }
        deadline = time.monotonic() + TOOL_CALIBRATION_TF_TIMEOUT_S
        try:
            for key, link in (
                ('T_torso_camera', frame),
                ('T_torso_gripper', self.tip_frames[arm]),
                ('T_torso_sensor', f'{arm}_kwr57b_link'),
            ):
                try:
                    stamped = self._tf_buffer.lookup_transform(
                        self.base_frame, link, stamp,
                        timeout=Duration(seconds=max(0.0, deadline - time.monotonic())),
                    )
                except ExtrapolationException:
                    if key != 'T_torso_camera':
                        raise
                    stamped = self._tf_buffer.lookup_transform(self.base_frame, link, Time())
                    offset = (Time.from_msg(stamped.header.stamp).nanoseconds - stamp.nanoseconds) * 1e-9
                    if abs(offset) > HEAD_CAMERA_TF_MAX_OFFSET_S:
                        raise ValueError(f'相机 TF 时间差 {offset:+.3f}s 超过 1 秒，拒绝标定')
                    inputs['camera_tf_time_offset_s'] = offset
                    self.get_logger().warning(f'相机 TF 时间差 {offset:+.3f}s，标定时头部须静止')
                q, p = stamped.transform.rotation, stamped.transform.translation
                quat = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
                inputs[key] = pin.SE3(
                    pin.Quaternion(quat).matrix(), np.array([p.x, p.y, p.z], dtype=np.float64),
                ).homogeneous
        except TransformException as error:
            raise ValueError(
                f'TF {self.base_frame} <- {link} 不可用；手臂必须对齐图像时刻: {error}'
            ) from error
        return inputs

    def _save_head_calibration_preview(
        self, frame: np.ndarray, label: str,
        camera_matrix: np.ndarray | None = None, dist_coeffs: np.ndarray | None = None,
        poses: dict[int, tuple[np.ndarray, np.ndarray]] | None = None,
    ) -> None:
        preview = frame.copy()
        caption = 'ID1: NOT CHECKED' if poses is None else 'ID1: NOT DETECTED'
        if poses and 1 in poses:
            rvec, tvec = poses[1]
            corners, _ = cv2.projectPoints(
                get_marker_object_points(), rvec, tvec, camera_matrix, dist_coeffs,
            )
            cv2.aruco.drawDetectedMarkers(
                preview, [corners.reshape(1, 4, 2).astype(np.float32)],
                np.array([[1]], dtype=np.int32),
            )
            cv2.drawFrameAxes(
                preview, camera_matrix, dist_coeffs, rvec, tvec, MARKER_LENGTH_M * 0.75,
            )
            caption = 'ID1: DETECTED'
            tip_offset = self.tool_calibrator.T_id1_tip[:3, 3]
            points, _ = cv2.projectPoints(
                np.array([np.zeros(3), tip_offset]), rvec, tvec,
                camera_matrix, dist_coeffs,
            )
            origin_px, tip_px = points.reshape(2, 2)
            height, width = preview.shape[:2]
            if 0 <= tip_px[0] < width and 0 <= tip_px[1] < height:
                origin = tuple(np.rint(origin_px).astype(int))
                tip = tuple(np.rint(tip_px).astype(int))
                color = (255, 0, 255)
                cv2.line(preview, origin, tip, color, 1, cv2.LINE_AA)
                cv2.circle(preview, tip, 8, color, 2, cv2.LINE_AA)
                cv2.drawMarker(preview, tip, color, cv2.MARKER_CROSS, 22, 2, cv2.LINE_AA)
                cv2.putText(
                    preview, 'TIP', (min(tip[0] + 12, max(0, width - 50)), max(18, tip[1] - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA,
                )
            else:
                caption += ' | TIP: OUTSIDE IMAGE'
        preview = cv2.copyMakeBorder(
            preview, 72, 0, 0, 0, cv2.BORDER_CONSTANT, value=(25, 25, 25),
        )
        for index, text in enumerate((label, caption)):
            cv2.putText(
                preview, text, (12, 27 + index * 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (230, 230, 230), 2, cv2.LINE_AA,
            )
        ok, encoded = cv2.imencode('.jpg', preview, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            raise RuntimeError('主相机标定截图编码失败')
        with self._sensor_lock:
            self._head_calibration_jpeg = encoded.tobytes()

    def _on_external_localization(self, message: Float64MultiArray) -> None:
        data = np.asarray(message.data, dtype=np.float64)
        if data.size < 7 or not np.all(np.isfinite(data[:7])):
            return
        d_pos = data[:3].copy()
        d_quat = data[3:7].copy()
        if self.quat_order.lower() == 'xyzw':
            d_quat_wxyz = np.array([d_quat[3], d_quat[0], d_quat[1], d_quat[2]], dtype=np.float64)
        else:
            d_quat_wxyz = d_quat.copy()
        norm_q = float(np.linalg.norm(d_quat_wxyz))
        if norm_q > 1e-9:
            d_quat_wxyz = d_quat_wxyz / norm_q
            if d_quat_wxyz[0] < 0.0:
                d_quat_wxyz = -d_quat_wxyz
        else:
            d_quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        R_err = pin.Quaternion(
            float(d_quat_wxyz[0]),
            float(d_quat_wxyz[1]),
            float(d_quat_wxyz[2]),
            float(d_quat_wxyz[3]),
        ).matrix()
        R_HT = R_err.T
        r1_t_r0 = _PLUG_TO_HOLE_BASIS @ R_HT.T
        d_id1 = r1_t_r0 @ d_pos
        d_rot_id1 = r1_t_r0 @ _PLUG_TO_HOLE_BASIS.T
        now = time.monotonic()
        with self._sensor_lock:
            seq = int(self._vision_state['seq']) + 1
            self._vision_state.update(
                valid=True,
                seq=seq,
                stamp=now,
                stamp_source='external_receive',
                delta_pos=d_pos,
                delta_quat=d_quat,
                delta_quat_wxyz=d_quat_wxyz,
                R_HT=R_HT,
                delta_id0_m=d_pos.copy(),
                delta_id1_m=d_id1,
                delta_rot_id1=d_rot_id1,
                r1_t_r0=r1_t_r0,
                capture_gripper_pose=None,
                distance_mm=(
                    float(data[7]) if data.size > 7 else float(np.linalg.norm(d_pos) * 1000.0)
                ),
                lateral_err_mm=float(np.hypot(d_pos[0], d_pos[1]) * 1000.0),
                depth_err_mm=float(-d_pos[2] * 1000.0),
                missing_ids=[],
            )

    def _vision_loop(self) -> None:
        last_seq = -1
        last_finished = 0.0
        fps_ema = 0.0
        next_preview = 0.0
        next_frame = 0.0
        period = (
            1.0 / self.vision_rate_hz
            if self.vision_rate_hz < self.tracker.cam_cfg['fps'] else 0.0
        )
        while self._vision_alive:
            delay = next_frame - time.monotonic()
            if delay > 0.0:
                time.sleep(min(delay, 0.05))
                continue
            if self.tracker.camera_reader is None or not self.tracker.camera_reader.running:
                time.sleep(0.05)
                continue
            ret, frame, seq, frame_stamp = self.tracker.camera_reader.read_latest_timed()
            if not ret or frame is None or seq == last_seq:
                time.sleep(0.002)
                continue
            last_seq = seq
            with self._sensor_lock:
                cfg_seq = self._vision_cfg_seq
                hole_arg = None if self.target_hole in ('', 'auto') else self.target_hole
                depth_arg = self.insert_depth_mm
                preview = time.monotonic() <= self._preview_requested_until and time.monotonic() >= next_preview

            t0 = time.perf_counter()
            if period:
                now = time.monotonic()
                if not next_frame:
                    next_frame = now
                next_frame += period * (int((now - next_frame) / period) + 1)
            try:
                if self._vision_executor is None:
                    raise RuntimeError('Vision worker unavailable')
                # Only one frame is in flight; the next read skips frames captured while it ran.
                future = self._vision_executor.submit(
                    process_vision_frame, self._vision_frames.write(frame),
                    preview, hole_arg, depth_arg,
                )
                while self._vision_alive:
                    try:
                        result = future.result(timeout=0.1)
                        break
                    except FutureTimeout:
                        continue
                else:
                    return
            except (BrokenProcessPool, RuntimeError, ValueError, OSError, cv2.error) as error:
                with self._sensor_lock:
                    self._vision_state.update(valid=False, error=f'视觉进程失败: {error}')
                self.get_logger().error(f'视觉进程失败: {error}')
                return
            poses = dict(result.poses)
            if 0 in poses:
                self._last_id0_pose = (poses[0][0].copy(), poses[0][1].copy())
            elif self._last_id0_pose is not None:
                poses[0] = (self._last_id0_pose[0].copy(), self._last_id0_pose[1].copy())
            capture_pose = self._gripper_pose_at_stamp(self.active_arm, frame_stamp)
            stamp_source = 'camera_read'
            delta = None
            if 0 in poses and 1 in poses:
                delta = compute_plug_hole_delta(
                    *poses[0], *poses[1],
                    holes_mm=self.tracker.holes_mm,
                    plug_mm=self.tracker.plug_mm,
                    target_hole=hole_arg,
                    insert_depth_mm=depth_arg,
                )
                if capture_pose is not None:
                    delta = self._fuse_vision_delta_with_fk(delta, capture_pose, frame_stamp)
            elif capture_pose is not None:
                delta = self._coast_vision_delta_with_fk(
                    capture_pose, frame_stamp, hole_arg, depth_arg
                )
                if delta is not None:
                    stamp_source = 'fk_coast'
            finished = time.perf_counter()
            algo_ms = result.algo_ms
            interval = finished - last_finished if last_finished else finished - t0
            last_finished = finished
            inst_fps = 1.0 / max(0.0001, interval)
            fps_ema = inst_fps if fps_ema == 0.0 else (0.85 * fps_ema + 0.15 * inst_fps)

            with self._sensor_lock:
                if self._vision_cfg_seq != cfg_seq:
                    continue
                self._vision_state['fps'] = round(fps_ema, 1)
                self._vision_state['processed_frames'] += 1
                self._vision_state['processed_frame_age_ms'] = round(
                    (time.monotonic() - frame_stamp) * 1000.0, 2
                )
                self._vision_state['algo_ms'] = round(algo_ms, 2)
                self._vision_state['algo_cpu_ms'] = round(result.cpu_ms, 2)
                self._vision_state['preprocess_backend'] = result.preprocess_backend
                self._vision_state['stamp_source'] = stamp_source
                self._vision_state['error'] = ''
                if delta is not None:
                    self._vision_state.update(
                        valid=True,
                        seq=seq,
                        stamp=frame_stamp,
                        **{key: delta[key] for key in (
                            'target_hole', 'nearest_hole', 'distance_mm', 'lateral_err_mm',
                            'depth_err_mm', 'insert_depth_mm', 'current_depth_mm',
                            'plug_in_id0_mm', 'hole_in_id0_mm', 'axis_angle_deg',
                            'yaw_angle_deg', 'orientation_ok', 'delta_pos', 'delta_quat',
                            'delta_quat_wxyz', 'R_HT', 'delta_id0_m', 'delta_id1_m', 'delta_rot_id1',
                        )},
                        r1_t_r0=_PLUG_TO_HOLE_BASIS @ delta['R_HT'].T,
                        capture_gripper_pose=capture_pose,
                        missing_ids=[],
                    )
                else:
                    self._vision_state['valid'] = False
                    self._vision_state['capture_gripper_pose'] = None
                    self._vision_state['missing_ids'] = [i for i in (0, 1) if i not in poses]
                if result.jpeg:
                    self._jpeg_bytes = result.jpeg
            if preview:
                next_preview = time.monotonic() + 1.0 / PREVIEW_FPS
            if result.preview_error:
                self.get_logger().error(result.preview_error, throttle_duration_sec=2.0)

    def _seed_from_ik(self) -> bool:
        if not self.ik_ready:
            return False
        limited = self.status_ik().get('limited_pose') or {}
        if not all(side in limited for side in SIDES):
            return False
        self._pose = {
            side: np.asarray(limited[side], dtype=np.float64).copy() for side in SIDES
        }
        return True

    def _sync_ik_to_measured(self) -> bool:
        if not self.ik_ready:
            return False
        positions = self.output.positions31.copy()
        if self._joint_stamp > 0.0 and np.all(np.isfinite(self._q[self.arm_slots])):
            positions[self.arm_slots] = self._q[self.arm_slots]
        self.reset_ik(positions)
        return self._seed_from_ik()

    def _on_joint_states(self, message: Any) -> None:
        super()._on_joint_states(message)
        with self.lock:
            if self._joint_stamp > 0.0 and hasattr(self, 'arm_slots'):
                if np.all(np.isfinite(self._q[self.arm_slots])):
                    self._joint_history.append((self._joint_stamp, self._q.copy()))

    def _gripper_pose_at_stamp(self, side: str, stamp: float) -> np.ndarray | None:
        with self.lock:
            if self._ik is None or len(self.arm_indices) == 0:
                return None
            history = list(getattr(self, '_joint_history', ()))
            if history and stamp > 0.0:
                best_q = min(history, key=lambda item: abs(item[0] - stamp))[1]
                joints = best_q[self.arm_indices].copy()
            else:
                joints = self._q[self.arm_indices].copy()
            if not np.all(np.isfinite(joints)):
                return None
            return np.asarray(self._ik.fk(joints, clamp=False)[side], dtype=np.float64).copy()

    def _measured_gripper_pose(self, side: str) -> np.ndarray:
        with self.lock:
            joints = self._q[self.arm_indices].copy()
            if self._ik is None or not np.all(np.isfinite(joints)):
                raise RuntimeError('IK model or finite measured joints unavailable')
            return np.asarray(self._ik.fk(joints, clamp=False)[side], dtype=np.float64).copy()

    def _delta_from_h_state(
        self,
        hole_name: str,
        nearest_hole: str,
        depth_mm: float,
        d_id0_m: np.ndarray,
        R_ht: np.ndarray,
        base_delta: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        d_id0_m = np.asarray(d_id0_m, dtype=np.float64).reshape(3)
        R_ht = np.asarray(R_ht, dtype=np.float64).reshape(3, 3)
        d_id0_mm = d_id0_m * 1000.0
        hole_id0_mm = self.tracker.holes_mm[hole_name].copy()
        hole_id0_mm[2] = -depth_mm
        plug_id0_mm = hole_id0_mm - d_id0_mm
        cur_depth_mm = float(-plug_id0_mm[2])

        d_id1_m = _PLUG_TO_HOLE_BASIS @ R_ht.T @ d_id0_m
        d_rot_id1 = _PLUG_TO_HOLE_BASIS @ R_ht.T @ _PLUG_TO_HOLE_BASIS.T
        q_wxyz = rotation_matrix_to_quat(R_ht.T, order='wxyz')
        q_cfg = rotation_matrix_to_quat(R_ht.T, order=self.quat_order)
        axis_deg = float(np.degrees(np.arccos(np.clip(float(R_ht[2, 2]), -1.0, 1.0))))
        yaw_deg = float(np.degrees(np.arccos(np.clip(float(R_ht[0, 0]), -1.0, 1.0))))
        dist_mm = float(np.linalg.norm(d_id0_mm))
        lat_mm = float(np.hypot(d_id0_mm[0], d_id0_mm[1]))
        depth_err_mm = float(depth_mm - cur_depth_mm)

        out = dict(base_delta) if base_delta is not None else {}
        out.update(
            target_hole=hole_name,
            nearest_hole=nearest_hole,
            insert_depth_mm=depth_mm,
            current_depth_mm=cur_depth_mm,
            plug_in_id0_mm=plug_id0_mm,
            hole_in_id0_mm=hole_id0_mm,
            distance_mm=dist_mm,
            lateral_err_mm=lat_mm,
            depth_err_mm=depth_err_mm,
            axis_angle_deg=axis_deg,
            yaw_angle_deg=yaw_deg,
            orientation_ok=yaw_deg <= 25.0,
            delta_pos=d_id0_m.copy(),
            delta_quat=q_cfg,
            delta_quat_wxyz=q_wxyz,
            R_HT=R_ht,
            delta_id0_m=d_id0_m,
            delta_id1_m=d_id1_m,
            delta_rot_id1=d_rot_id1,
        )
        return out

    def _fuse_vision_delta_with_fk(
        self,
        delta: dict[str, Any],
        capture_pose: np.ndarray,
        frame_stamp: float,
    ) -> dict[str, Any]:
        """在孔系 H 下执行「帧间差分 FK 预测 + 视觉观测更新」互补滤波：消除 PnP 抖动与运动滞后，且稳态 100% 锚定相机相对测量，不引入绝对 FK 漂移"""
        cal = self.tool_calibrator.get_calibration(self.active_arm)
        R_ht_vis = np.asarray(delta['R_HT'], dtype=np.float64).reshape(3, 3)
        d_id0_vis = np.asarray(delta['delta_id0_m'], dtype=np.float64).reshape(3)
        hole_name = str(delta['target_hole'])
        nearest_name = str(delta.get('nearest_hole', hole_name))
        depth_mm = float(delta['insert_depth_mm'])

        prev = self._fused_hole_torso
        if (
            prev is None
            or prev.get('arm') != self.active_arm
            or prev.get('target_hole') != hole_name
            or abs(float(prev.get('insert_depth_mm', 0.0)) - depth_mm) > 1e-4
            or (frame_stamp - float(prev.get('last_vis_stamp', 0.0))) > 0.5
        ):
            d_id0_filt = d_id0_vis.copy()
            R_ht_filt = R_ht_vis.copy()
        else:
            d_pred, _, R_pred, _ = self._compensate_vision_with_fk(
                np.asarray(prev['delta_id0_m'], dtype=np.float64),
                None,
                np.asarray(prev['R_HT'], dtype=np.float64),
                None,
                np.asarray(prev['capture_pose'], dtype=np.float64),
                capture_pose,
                cal,
            )
            err_p = d_id0_vis - d_pred
            err_r = pin.log3(_orthonormalize_rot(R_pred.T @ R_ht_vis))
            d_id0_filt = d_pred + float(self.vision_fk_alpha_pos) * err_p
            R_ht_filt = _orthonormalize_rot(R_pred @ pin.exp3(float(self.vision_fk_alpha_rot) * err_r))

        self._fused_hole_torso = {
            'arm': self.active_arm,
            'target_hole': hole_name,
            'nearest_hole': nearest_name,
            'insert_depth_mm': depth_mm,
            'delta_id0_m': d_id0_filt.copy(),
            'R_HT': R_ht_filt.copy(),
            'capture_pose': np.asarray(capture_pose, dtype=np.float64).copy(),
            'last_vis_stamp': frame_stamp,
        }
        return self._delta_from_h_state(
            hole_name, nearest_name, depth_mm, d_id0_filt, R_ht_filt, base_delta=delta
        )

    def _coast_vision_delta_with_fk(
        self,
        capture_pose: np.ndarray,
        frame_stamp: float,
        hole_arg: str | None,
        depth_arg: float,
    ) -> dict[str, Any] | None:
        """当 ID1 暂时被遮挡或丢码时，在允许的时间窗内使用上一有效视觉帧 + 实时差分 FK 继续推算相对位姿"""
        fused = self._fused_hole_torso
        coast_s = float(self.vision_fk_coast_s)
        if fused is None or coast_s <= 0.0:
            return None
        if fused.get('arm') != self.active_arm:
            return None
        if hole_arg is not None and fused.get('target_hole') != hole_arg:
            return None
        if abs(float(fused.get('insert_depth_mm', 0.0)) - float(depth_arg)) > 1e-4:
            return None
        if (frame_stamp - float(fused.get('last_vis_stamp', 0.0))) > coast_s:
            return None
        cal = self.tool_calibrator.get_calibration(self.active_arm)
        d_now, _, R_now, _ = self._compensate_vision_with_fk(
            np.asarray(fused['delta_id0_m'], dtype=np.float64),
            None,
            np.asarray(fused['R_HT'], dtype=np.float64),
            None,
            np.asarray(fused['capture_pose'], dtype=np.float64),
            capture_pose,
            cal,
        )
        return self._delta_from_h_state(
            str(fused['target_hole']),
            str(fused.get('nearest_hole', fused['target_hole'])),
            float(fused['insert_depth_mm']),
            d_now,
            R_now,
        )

    def policy_input_status(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            joint = _input_health(
                self._joint_stamp, now, self._state_timeout,
                valid=bool(np.all(np.isfinite(self._q[self.arm_slots]))),
            )
            side = self.active_arm
            with self._sensor_lock:
                vision = _input_health(
                    float(self._vision_state['stamp']), now, self.vision_timeout_s,
                    valid=bool(self._vision_state['valid']),
                )
                vision['detail'] = (
                    f"missing IDs: {self._vision_state['missing_ids']}"
                    if self._vision_state['missing_ids'] else self._vision_state.get('stamp_source', '')
                )
                streams = {
                    arm: {
                        kind: _input_health(stamp, now, self.wrench_timeout_s, valid=valid)
                        for kind, (stamp, valid) in self._wrench_inputs[arm].items()
                    } for arm in SIDES
                }
                force, torque, stamp = self._wrench[side]
                source = self._wrench_source[side]
                wrench = _input_health(
                    stamp, now, self.wrench_timeout_s,
                    valid=bool(np.all(np.isfinite(np.r_[force, torque]))),
                )
                wrench['source'] = source
                wrench['detail'] = source
                if source == 'wrench_net' and not streams[side]['net']['valid']:
                    wrench.update(ready=False, status=streams[side]['net']['status'])
            checks = {
                'vision': {'label': '视觉位姿', **vision},
                'wrench': {'label': f'{side} 策略力觉', **wrench},
                'joint_feedback': {'label': '实测关节 / FK', 'detail': '/joint_states', **joint},
                'model': {'label': 'ONNX 模型', 'ready': self.policy.loaded, 'detail': '13 → 6'},
                'ik': {'label': 'IK 模型', 'ready': self.ik_ready, 'detail': self.base_frame},
            }
            warnings = [
                f"{item['label']}: {item.get('status', 'not_ready')} {item.get('detail', '')}".strip()
                for item in checks.values() if not item['ready']
            ]
            if source == 'wrench_raw+urdf_grav':
                warnings.append('正在使用原始力备用补偿，参考点转换尚未验收')
            return {
                'ready': all(item['ready'] for item in checks.values()),
                'checks': checks, 'warnings': warnings, 'wrench_streams': streams,
            }

    def _try_recover_from_timeout(self) -> None:
        if (
            self.state is State.ERROR
            and getattr(self, '_registered', False)
            and not self._preparing
            and not self.stale_reason()
        ):
            self.state = State.RUNNING
            self._error = ''

    def set_mode(self, new_mode: str) -> None:
        if new_mode not in MODES:
            raise ValueError(f'不支持的控制模式 {new_mode!r}，可选: {MODES}')
        with self.execution_lock, self.lock:
            if new_mode == 'mech_insert':
                _require_mech_calibration(self.tool_calibrator.get_calibration(self.active_arm))
            if new_mode == 'float':
                self._try_recover_from_timeout()
            prev_mode = self.mode
            if new_mode != 'float' and (
                prev_mode == 'float' or not self._seeded
                or (prev_mode == 'mech_insert' and new_mode in ('hold', 'policy_insert'))
            ):
                if self._sync_ik_to_measured():
                    self._seeded = True
            self.mode = new_mode
            self._mech_goal_pose = None
            if new_mode in ('float', 'policy_insert'):
                self.policy.reset()
                self._policy_step_count = 0
                self._policy_tip_cmd = None
            self._last_action[:] = 0.0
            self._last_action_6d[:] = 0.0

    def set_active_arm(self, arm: str) -> None:
        if arm not in SIDES:
            raise ValueError(f'active_arm 必须为 {SIDES} 之一，收到 {arm!r}')
        with self.execution_lock, self.lock:
            if arm != self.active_arm:
                self.active_arm = arm
                if self.mode in ('mech_insert', 'policy_insert'):
                    self.set_mode('float')
                else:
                    self._mech_goal_pose = None

    def set_gripper(self, state: str, arm: str | None = None) -> float:
        side = self.active_arm if not arm else arm
        if side not in SIDES:
            raise ValueError(f'arm 必须为 {SIDES} 之一')
        idx = SIDES.index(side)
        if state == 'open':
            val = self._grip_open
        elif state == 'close':
            val = self._grip_closed
        else:
            raise ValueError("state 必须为 'open' 或 'close'")
        with self.execution_lock, self.lock:
            self._grip[idx] = val
            self.output.positions[self.gripper_slots[idx]] = val
        enable_clients = getattr(self, '_gripper_enable_clients', {})
        client = enable_clients.get(side)
        if client is not None and client.service_is_ready():
            client.call_async(Trigger.Request())
        return val

    def _invalidate_vision_target(self) -> None:
        self._mech_goal_pose = None
        self._fused_hole_torso = None
        self._vision_cfg_seq += 1
        self._vision_state['capture_gripper_pose'] = None

    def handle_dashboard_command(self, payload: dict[str, Any]) -> dict[str, Any]:
        action = str(payload.get('action', '')).strip()
        if action == 'set_arm':
            arm = str(payload.get('arm', '')).strip()
            self.set_active_arm(arm)
            return {'message': f'已切换执行手臂为: {"左臂 (left)" if arm == "left" else "右臂 (right)"}'}

        if action == 'gripper':
            arm = str(payload.get('arm', self.active_arm)).strip() or self.active_arm
            st = str(payload.get('state', '')).strip()
            rad = self.set_gripper(st, arm=arm)
            label = '打开' if st == 'open' else '闭合'
            return {'message': f'{"左臂" if arm == "left" else "右臂"}夹爪已设为{label} ({rad:.2f} rad)'}

        if action == 'set_mode':
            new_mode = str(payload.get('mode', '')).strip()
            self.set_mode(new_mode)
            names = {
                'float': '手臂浮动 (Float)',
                'hold': '位置保持 (Hold)',
                'mech_insert': '机械插入 (Mechanical Insert)',
                'policy_insert': '策略插入 (Policy Insert)',
            }
            return {'message': f'已切换控制模式: {names.get(new_mode, new_mode)}'}

        if action == 'step_policy':
            dry_run = bool(payload.get('dry_run', False))
            info = self.step_policy_once(dry_run=dry_run)
            prefix = '策略单步干跑预览' if dry_run else f'已执行策略第 {info["step_index"]} 步并保持'
            t_mm = info['tip_step_torso_mm']
            g_mm = info['gripper_step_torso_mm']
            return {
                'message': (
                    f'{prefix}: 尖端躯干系[{t_mm[0]:+.2f}, {t_mm[1]:+.2f}, {t_mm[2]:+.2f}]mm, '
                    f'夹爪躯干系[{g_mm[0]:+.2f}, {g_mm[1]:+.2f}, {g_mm[2]:+.2f}]mm'
                ),
                'policy_step': info,
            }

        if action == 'reset_policy':
            with self.execution_lock, self.lock:
                self.policy.reset()
                self._policy_step_count = 0
                self._policy_tip_cmd = None
                self._last_policy_step_info = None
                self._last_action[:] = 0.0
                self._last_action_6d[:] = 0.0
                if self.mode == 'hold' and self.ik_ready:
                    if self._sync_ik_to_measured():
                        self._seeded = True
            return {'message': '已重置策略 GRU 隐状态并对齐当前实测位姿'}

        if action == 'set_mech_params':
            unsupported = payload.keys() - {
                'action', 'target_hole', 'insert_depth_mm', 'align_orientation',
            }
            if unsupported:
                raise ValueError(f'不支持的机械参数: {", ".join(sorted(unsupported))}')
            with self.execution_lock, self.lock, self._sensor_lock:
                if 'target_hole' in payload:
                    th = str(payload['target_hole']).strip() or 'auto'
                    if th != 'auto' and th not in self.tracker.holes_mm:
                        raise ValueError(f'未知孔位: {th}')
                    self.target_hole = th
                    self.tracker.target_hole = None if th == 'auto' else th
                if 'insert_depth_mm' in payload:
                    d_mm = float(payload['insert_depth_mm'])
                    if not math.isfinite(d_mm) or not -80.0 <= d_mm <= 60.0:
                        raise ValueError('insert_depth_mm 需在 [-80, +60] mm 范围内')
                    self.insert_depth_mm = d_mm
                if 'align_orientation' in payload:
                    self.align_orientation = bool(payload['align_orientation'])
                self._invalidate_vision_target()
            return {
                'message': (
                    f'插入参数已更新: 目标孔={self.target_hole}, '
                    f'设定深度={self.insert_depth_mm:+.1f} mm'
                )
            }

        if action == 'step_depth':
            delta_mm = float(payload.get('delta_mm', 0.0))
            if not math.isfinite(delta_mm):
                raise ValueError('delta_mm 必须为有限数值')
            with self.execution_lock, self.lock, self._sensor_lock:
                self.insert_depth_mm = float(
                    np.clip(self.insert_depth_mm + delta_mm, -80.0, 60.0)
                )
                self._invalidate_vision_target()
            return {'message': f'目标插入深度已调整为 {self.insert_depth_mm:+.1f} mm'}

        if action == 'tare_wrench':
            arm = str(payload.get('arm', self.active_arm)).strip()
            reset = bool(payload.get('reset', False))
            sides = SIDES if arm == 'both' else ((arm,) if arm in SIDES else (self.active_arm,))
            with self._sensor_lock:
                for s in sides:
                    if reset:
                        self._wrench_bias[s] = (np.zeros(3), np.zeros(3))
                        self._wrench_proc[s].reset()
                    else:
                        f_raw, t_raw, _ = self._wrench[s]
                        self._wrench_bias[s] = (f_raw.copy(), t_raw.copy())
                        self._wrench_proc[s].reset()
                self._safety_warning = ''
            return {
                'message': (
                    '力传感器零偏与滤波器已重置'
                    if reset
                    else f'已完成力传感器软件调零并重置 15ms 低通滤波器 ({arm})'
                )
            }

        if action == 'calibrate_ft':
            with self._sensor_lock:
                if self._ft_calib_running:
                    raise ValueError('手部力传感器校准正在执行中，请勿重复启动')
                self._ft_calib_running = True
                self._ft_calib_status = '正在启动多姿态重力校准...'
                self._ft_calib_error = ''
            self.set_mode('float')
            threading.Thread(target=self._run_ft_calibration_worker, daemon=True).start()
            return {
                'message': '已启动手部力传感器多姿态重力校准（双臂将依次走 6 组姿态并自动补偿重力）'
            }

        if action == 'calibrate_tool':
            arm = str(payload.get('arm', self.active_arm)).strip() or self.active_arm
            if arm not in SIDES:
                raise ValueError(f'arm 必须为 {SIDES} 之一')
            cal_mode = str(payload.get('mode', 'head_camera')).strip().lower()
            if cal_mode not in ('nominal', 'head_camera'):
                raise ValueError(f'未知标定模式: {cal_mode!r}')
            with self.execution_lock, self.lock:
                if self.mode not in ('float', 'hold'):
                    raise ValueError('请先停止机械/策略运动，再修改工具标定')
                if cal_mode == 'nominal':
                    res = self.tool_calibrator.reset_nominal(arm)
                    return {
                        'message': f'已恢复名义标定 ({arm})，不可用于机械预定位',
                        'calibration': res,
                    }
            with self._sensor_lock:
                self._head_calibration_jpeg = b''
            inputs = self._capture_head_inputs(arm)
            capture_label = inputs.pop('capture_label')
            try:
                res = self.tool_calibrator.calibrate_from_head_camera(arm=arm, **inputs)
            finally:
                self._save_head_calibration_preview(
                    inputs['frame_bgr'], capture_label, inputs['head_K'],
                    inputs['head_dist'], self.tool_calibrator.last_head_poses,
                )
            with self.execution_lock, self.lock:
                self._mech_goal_pose = None
            return {
                'message': (
                    f'标定完成并已保存 ({arm})：夹爪至尖端 {res["flange_to_tip_norm_mm"]:.1f} mm，'
                    f'相机 TF 时间差 {res["camera_tf_time_offset_s"]:+.3f}s；'
                    f'请核验长度和方向，非同刻标定要求头部静止'
                ),
                'calibration': res,
            }

        if action == 'session':
            cmd = str(payload.get('command', '')).strip()
            if cmd not in self._session_clients:
                raise ValueError(f'不支持的会话指令: {cmd}')
            client = self._session_clients[cmd]
            if not client.service_is_ready():
                raise ValueError(f'会话服务 /motion_control/{cmd} 未就绪，请确认 g1_motion_session 已启动')
            client.call_async(Trigger.Request())
            return {'message': f'已发送会话请求: {cmd}'}

        raise ValueError(f'未知操作: {action!r}')

    def _run_ft_calibration_worker(self) -> None:
        """后台运行 arm_gravity_compensation auto_ft_calibration，期间暂停本节点向 FPC 发布指令"""
        last_err = ''
        proc: subprocess.Popen[str] | None = None
        try:
            proc = subprocess.Popen(
                ['ros2', 'run', 'arm_gravity_compensation', 'auto_ft_calibration'],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            with self._sensor_lock:
                self._ft_calib_proc = proc
            if proc.stdout is not None:
                for raw_line in proc.stdout:
                    line = raw_line.strip()
                    if not line:
                        continue
                    if 'Both arms pose' in line:
                        pose_str = line.split('Both arms pose', 1)[1].strip()
                        with self._sensor_lock:
                            self._ft_calib_status = f'校准中: 姿态 {pose_str}'
                    elif line.startswith('Stopped:'):
                        last_err = line.split('Stopped:', 1)[1].strip()
                    elif 'Calibration saved and reloaded' in line:
                        with self._sensor_lock:
                            self._ft_calib_status = '力传感器校准完成'
            ret_code = proc.wait()
            if ret_code == 0:
                self.tool_calibrator.reload_ft_calibrations()
                with self._sensor_lock:
                    for s in SIDES:
                        self._wrench_bias[s] = (np.zeros(3), np.zeros(3))
                        self._wrench_proc[s].reset()
                    self._ft_calib_status = '已校准并重载'
                    self._ft_calib_error = ''
                with self.execution_lock, self.lock:
                    self._sync_ik_to_measured()
            else:
                err_msg = last_err or f'退出码 {ret_code}'
                with self._sensor_lock:
                    self._ft_calib_status = '校准失败'
                    self._ft_calib_error = f'手部力传感器校准失败: {err_msg}'
        except Exception as err:
            with self._sensor_lock:
                self._ft_calib_status = '校准异常'
                self._ft_calib_error = f'手部力传感器校准异常: {err}'
        finally:
            with self._sensor_lock:
                self._ft_calib_running = False
                self._ft_calib_proc = None

    def _compute_mech_target(
        self,
        vis_valid: bool,
        delta_id1_m: np.ndarray,
        delta_rot_id1: np.ndarray,
    ) -> np.ndarray | None:
        cal = self.tool_calibrator.get_calibration(self.active_arm)
        _require_mech_calibration(cal)
        if not vis_valid:
            return None

        cur_pose = self._measured_gripper_pose(self.active_arm)
        R_torso_gripper = pin.Quaternion(cur_pose[3:7]).matrix()
        R_gripper_from_id1 = (
            np.asarray(cal['R_G_T'], dtype=np.float64).reshape(3, 3)
            @ _PLUG_TO_HOLE_BASIS.T
        )
        p_gripper_tip = np.asarray(cal['p_G_T_m'], dtype=np.float64).reshape(3)
        goal_tip = (
            cur_pose[:3] + R_torso_gripper @ p_gripper_tip
            + R_torso_gripper @ R_gripper_from_id1 @ delta_id1_m
        )
        if self.align_orientation:
            R_delta_gripper = (
                R_gripper_from_id1 @ delta_rot_id1 @ R_gripper_from_id1.T
            )
            R_goal = R_torso_gripper @ R_delta_gripper
        else:
            R_goal = R_torso_gripper
        goal_pos = goal_tip - R_goal @ p_gripper_tip
        return np.concatenate([goal_pos, _mat_to_quat(R_goal)])

    def _limit_policy_action(self, raw_act: np.ndarray) -> np.ndarray:
        """限制单步旋转向量幅值，防止 176mm 长力臂将单拍大旋转放大为剧烈夹爪横移"""
        act = np.asarray(raw_act, dtype=np.float64).copy()
        max_rot_rad = float(self.max_policy_rot_rad)
        if max_rot_rad > 0.0 and act.size >= 6:
            rot_norm = float(np.linalg.norm(act[3:6]))
            if rot_norm > max_rot_rad:
                act[3:6] *= max_rot_rad / rot_norm
        return act

    def _compensate_vision_with_fk(
        self,
        delta_id0_m: np.ndarray,
        delta_quat_wxyz: np.ndarray | None,
        R_HT: np.ndarray,
        r1_t_r0: np.ndarray | None,
        capture_pose: np.ndarray | None,
        cur_pose: np.ndarray,
        cal: dict[str, Any],
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """利用图像采集时刻到当前控制时刻的实测差分 FK 前推孔系位姿误差，补偿视觉传输与解算延迟"""
        d_id0 = np.asarray(delta_id0_m, dtype=np.float64).reshape(3).copy()
        r_ht = _orthonormalize_rot(np.asarray(R_HT, dtype=np.float64).reshape(3, 3))
        if (
            capture_pose is None
            or len(capture_pose) < 7
            or not (np.all(np.isfinite(capture_pose)) and np.all(np.isfinite(cur_pose)))
        ):
            q_wxyz = (
                rotation_matrix_to_quat(r_ht.T, order='wxyz')
                if delta_quat_wxyz is None
                else np.asarray(delta_quat_wxyz, dtype=np.float64).reshape(4).copy()
            )
            r1_r0 = (
                _PLUG_TO_HOLE_BASIS @ r_ht.T
                if r1_t_r0 is None
                else np.asarray(r1_t_r0, dtype=np.float64).reshape(3, 3).copy()
            )
            return d_id0, q_wxyz, r_ht, r1_r0

        R_gt = _orthonormalize_rot(np.asarray(cal['R_G_T'], dtype=np.float64).reshape(3, 3))
        p_gt = np.asarray(cal['p_G_T_m'], dtype=np.float64).reshape(3)

        R_bg_cam = pin.Quaternion(np.asarray(capture_pose[3:7], dtype=np.float64)).matrix()
        p_bt_cam = np.asarray(capture_pose[:3], dtype=np.float64).reshape(3) + R_bg_cam @ p_gt
        R_bt_cam = R_bg_cam @ R_gt

        R_bg_now = pin.Quaternion(np.asarray(cur_pose[3:7], dtype=np.float64)).matrix()
        p_bt_now = np.asarray(cur_pose[:3], dtype=np.float64).reshape(3) + R_bg_now @ p_gt
        R_bt_now = R_bg_now @ R_gt

        dR_fk = _orthonormalize_rot(R_bt_cam.T @ R_bt_now)
        dp_fk_t = R_bt_cam.T @ (p_bt_now - p_bt_cam)

        d_id0_now = d_id0 - r_ht @ dp_fk_t
        r_ht_now = _orthonormalize_rot(r_ht @ dR_fk)
        q_wxyz_now = rotation_matrix_to_quat(r_ht_now.T, order='wxyz')
        r1_r0_now = _PLUG_TO_HOLE_BASIS @ r_ht_now.T
        return d_id0_now, q_wxyz_now, r_ht_now, r1_r0_now

    def _compute_tip_centered_policy_step(
        self,
        raw_act_6d: np.ndarray,
        R_torso_gripper: np.ndarray,
        R_HT: np.ndarray,
        r1_t_r0: np.ndarray | None,
        cal: dict[str, Any],
    ) -> tuple[np.ndarray, np.ndarray]:
        """按 DEPLOYMENT_ONNX.md 第 4 节将孔系 H 下的 6D 增量 [dp_H, dr_H] 绕插头名义尖端 T 合成到 gripper_base (G) 与 torso 系"""
        dp_h = np.asarray(raw_act_6d[:3], dtype=np.float64).reshape(3)
        dr_h = (
            np.asarray(raw_act_6d[3:6], dtype=np.float64).reshape(3)
            if raw_act_6d.size >= 6
            else np.zeros(3, dtype=np.float64)
        )
        dp_t = R_HT.T @ dp_h
        dr_t = R_HT.T @ dr_h
        dR_t = pin.exp3(dr_t) if float(np.linalg.norm(dr_t)) > 1e-12 else np.eye(3, dtype=np.float64)

        R_gt = np.asarray(cal['R_G_T'], dtype=np.float64).reshape(3, 3)
        p_gt = np.asarray(cal['p_G_T_m'], dtype=np.float64).reshape(3)
        dR_g = R_gt @ dR_t @ R_gt.T
        dp_g = R_gt @ dp_t + (np.eye(3, dtype=np.float64) - dR_g) @ p_gt

        step_pos_torso = R_torso_gripper @ dp_g
        step_ori_g = pin.log3(dR_g) if float(np.linalg.norm(dr_t)) > 1e-12 else np.zeros(3, dtype=np.float64)
        return step_pos_torso, step_ori_g

    def _advance_commanded_tip_pose(
        self,
        cmd_before: np.ndarray,
        cur_pose: np.ndarray,
        raw_act: np.ndarray,
        R_HT: np.ndarray,
        cal: dict[str, Any],
        *,
        step_scale: float = 0.50,
    ) -> np.ndarray:
        """在目标尖端位姿 (p_tip_cmd, R_tip_cmd) 上累加策略 delta 以保留重力与静摩擦支撑偏置，
        并通过相对实测尖端 (p_tip_meas, R_tip_meas) 的防积分饱和限幅 (12mm / 4°) 防止受阻时无限累积。
        """
        R_gt = _orthonormalize_rot(np.asarray(cal['R_G_T'], dtype=np.float64).reshape(3, 3))
        p_gt = np.asarray(cal['p_G_T_m'], dtype=np.float64).reshape(3)
        R_cmd_prev = pin.Quaternion(np.asarray(cmd_before[3:7], dtype=np.float64)).matrix()
        R_tip_cmd = _orthonormalize_rot(R_cmd_prev @ R_gt)
        R_tg = pin.Quaternion(np.asarray(cur_pose[3:7], dtype=np.float64)).matrix()
        p_tip_meas = np.asarray(cur_pose[:3], dtype=np.float64) + R_tg @ p_gt
        R_tip_meas = _orthonormalize_rot(R_tg @ R_gt)
        R_ht = _orthonormalize_rot(np.asarray(R_HT, dtype=np.float64).reshape(3, 3))

        scale = float(step_scale)
        dp_h = np.asarray(raw_act[:3], dtype=np.float64).reshape(3) * scale
        dr_h = (
            np.asarray(raw_act[3:6], dtype=np.float64).reshape(3) * scale
            if raw_act.size >= 6
            else np.zeros(3, dtype=np.float64)
        )
        dp_t = R_ht.T @ dp_h
        dr_t = R_ht.T @ dr_h
        dr_torso = R_tip_meas @ dr_t

        p_tip_cmd = self._policy_tip_cmd
        if p_tip_cmd is None:
            p_tip_cmd = np.asarray(cmd_before[:3], dtype=np.float64) + R_cmd_prev @ p_gt
        p_tip_cmd = np.asarray(p_tip_cmd, dtype=np.float64) + R_tip_meas @ dp_t

        lead_p = p_tip_cmd - p_tip_meas
        max_lead_p = float(self.max_tip_lead_m)
        lead_xy = float(np.hypot(lead_p[0], lead_p[1]))
        if lead_xy > max_lead_p:
            lead_p[0] *= max_lead_p / lead_xy
            lead_p[1] *= max_lead_p / lead_xy
        if abs(float(lead_p[2])) > max_lead_p:
            lead_p[2] = math.copysign(max_lead_p, float(lead_p[2]))
        p_tip_cmd = p_tip_meas + lead_p
        self._policy_tip_cmd = p_tip_cmd

        R_tip_new = (
            _orthonormalize_rot(pin.exp3(dr_torso) @ R_tip_cmd)
            if float(np.linalg.norm(dr_torso)) > 1e-12
            else R_tip_cmd
        )
        lead_r = pin.log3(_orthonormalize_rot(R_tip_meas.T @ R_tip_new))
        max_lead_r = float(self.max_tip_lead_rad)
        if float(np.linalg.norm(lead_r)) > max_lead_r:
            R_tip_new = _orthonormalize_rot(
                R_tip_meas @ pin.exp3(lead_r * (max_lead_r / float(np.linalg.norm(lead_r))))
            )

        R_cmd_new = _orthonormalize_rot(R_tip_new @ R_gt.T)
        p_cmd_new = p_tip_cmd - R_cmd_new @ p_gt
        return np.concatenate([p_cmd_new, _mat_to_quat(R_cmd_new)])

    def _record_policy_step_info(
        self,
        *,
        step_index: int,
        dry_run: bool,
        target_hole: str,
        dist_mm: float,
        axis_deg: float,
        yaw_deg: float,
        delta_id0_m: np.ndarray,
        raw_act: np.ndarray,
        R_tg: np.ndarray,
        R_HT: np.ndarray,
        cal: dict[str, Any],
        step_pos: np.ndarray,
        step_ori: np.ndarray,
        cur_pose: np.ndarray,
        cmd_pose: np.ndarray,
    ) -> dict[str, Any]:
        R_gt = np.asarray(cal['R_G_T'], dtype=np.float64).reshape(3, 3)
        dp_h = np.asarray(raw_act[:3], dtype=np.float64).reshape(3)
        dr_h = np.asarray(raw_act[3:6], dtype=np.float64).reshape(3)
        tip_step_torso = R_tg @ R_gt @ (R_HT.T @ dp_h)
        sag_torso_mm = (
            np.asarray(cmd_pose[:3], dtype=np.float64) - np.asarray(cur_pose[:3], dtype=np.float64)
        ) * 1000.0
        info = {
            'step_index': int(step_index),
            'dry_run': bool(dry_run),
            'target_hole': str(target_hole),
            'distance_mm': round(float(dist_mm), 2),
            'axis_angle_deg': round(float(axis_deg), 2),
            'yaw_angle_deg': round(float(yaw_deg), 2),
            'delta_pos_h_mm': [round(float(v * 1000.0), 2) for v in delta_id0_m],
            'act_pos_h_mm': [round(float(v * 1000.0), 3) for v in dp_h],
            'act_rot_h_deg': [round(float(math.degrees(v)), 3) for v in dr_h],
            'tip_step_torso_mm': [round(float(v * 1000.0), 3) for v in tip_step_torso],
            'gripper_step_torso_mm': [round(float(v * 1000.0), 3) for v in step_pos],
            'gripper_rot_deg': [round(float(math.degrees(v)), 3) for v in step_ori],
            'sag_torso_mm': [round(float(v), 2) for v in sag_torso_mm],
        }
        self._last_policy_step_info = info
        return info

    def _evaluate_policy_step(
        self,
        *,
        f_tip: np.ndarray,
        m_tip: np.ndarray,
        delta_id0_m: np.ndarray,
        delta_quat_wxyz: np.ndarray,
        R_HT: np.ndarray,
        r1_t_r0: np.ndarray,
        capture_pose: np.ndarray | None,
        cur_pose: np.ndarray,
        cmd_pose: np.ndarray,
        cal: dict[str, Any],
        target_hole: str,
        dist_mm: float,
        axis_deg: float,
        yaw_deg: float,
        dry_run: bool = False,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
        delta_id0_m, delta_quat_wxyz, R_HT, r1_t_r0 = self._compensate_vision_with_fk(
            delta_id0_m, delta_quat_wxyz, R_HT, r1_t_r0, capture_pose, cur_pose, cal
        )
        if capture_pose is not None:
            dist_mm = float(np.linalg.norm(delta_id0_m) * 1000.0)
            axis_deg = float(np.degrees(np.arccos(np.clip(float(R_HT[2, 2]), -1.0, 1.0))))
            yaw_deg = float(np.degrees(np.arccos(np.clip(float(R_HT[0, 0]), -1.0, 1.0))))
        h_backup = self.policy.hidden_state if dry_run else None
        try:
            raw_act = self._limit_policy_action(
                self.policy.step(
                    f_tip, m_tip, delta_id0_m, delta_quat_wxyz, quat_order='wxyz',
                )
            )
        finally:
            if dry_run and h_backup is not None:
                self.policy._hidden = h_backup
        R_tg = pin.Quaternion(np.asarray(cur_pose[3:7], dtype=np.float64)).matrix()
        step_pos, step_ori = self._compute_tip_centered_policy_step(
            raw_act, R_tg, R_HT, r1_t_r0, cal
        )
        step_idx = self._policy_step_count if dry_run else self._policy_step_count + 1
        if not dry_run:
            self._policy_step_count = step_idx
            self._last_action_6d[: raw_act.size] = raw_act[:6]
        info = self._record_policy_step_info(
            step_index=step_idx,
            dry_run=dry_run,
            target_hole=target_hole,
            dist_mm=dist_mm,
            axis_deg=axis_deg,
            yaw_deg=yaw_deg,
            delta_id0_m=delta_id0_m,
            raw_act=raw_act,
            R_tg=R_tg,
            R_HT=R_HT,
            cal=cal,
            step_pos=step_pos,
            step_ori=step_ori,
            cur_pose=cur_pose,
            cmd_pose=cmd_pose,
        )
        return raw_act, R_HT, step_pos, step_ori, info

    def step_policy_once(self, *, dry_run: bool = False) -> dict[str, Any]:
        now = time.monotonic()
        cal = self.tool_calibrator.get_calibration(self.active_arm)
        r_st_t = np.asarray(cal['r_ST_T_m'], dtype=np.float64)
        r_ts = np.asarray(cal['R_TS'], dtype=np.float64)
        with self._sensor_lock:
            ft_cal_busy = self._ft_calib_running
            _, _, wrench_stamp = self._wrench[self.active_arm]
            f_tip, m_tip = self._wrench_proc[self.active_arm].tip_wrench(
                r_st_t, r_ts, use_filtered=True
            )
            safe_ok, safe_msg, _, _ = self._wrench_proc[self.active_arm].check_safety(
                r_st_t, r_ts
            )
            vis_valid = (
                bool(self._vision_state['valid'])
                and (now - float(self._vision_state['stamp'])) <= self.vision_timeout_s
            )
            delta_id0_m = np.asarray(self._vision_state['delta_id0_m'], dtype=np.float64).copy()
            delta_quat_wxyz = np.asarray(
                self._vision_state['delta_quat_wxyz'], dtype=np.float64
            ).copy()
            R_HT = np.asarray(self._vision_state['R_HT'], dtype=np.float64).copy()
            r1_t_r0 = np.asarray(self._vision_state['r1_t_r0'], dtype=np.float64).copy()
            capture_pose = (
                None
                if self._vision_state.get('capture_gripper_pose') is None
                else np.asarray(self._vision_state['capture_gripper_pose'], dtype=np.float64).copy()
            )
            target_hole = str(self._vision_state['target_hole'])
            dist_mm = float(self._vision_state['distance_mm'])
            axis_deg = float(self._vision_state['axis_angle_deg'])
            yaw_deg = float(self._vision_state['yaw_angle_deg'])

        if not self.policy.loaded:
            raise ValueError('ONNX 策略模型未加载')
        if not vis_valid:
            raise ValueError('当前视觉位姿无效或已超时，请确认 ID0 与 ID1 均可见')
        if wrench_stamp <= 0.0:
            raise ValueError('尚未收到力传感器数据')
        if not safe_ok:
            raise ValueError(safe_msg)

        with self.execution_lock, self.lock:
            if not self.ik_ready:
                raise ValueError('IK 模型尚未就绪')
            if not dry_run:
                if self.state is not State.RUNNING or self._preparing or ft_cal_busy:
                    raise ValueError('运动会话未处于运行状态，请先站立预备')
                reason = self.stale_reason()
                if reason:
                    raise ValueError(f'反馈超时: {reason}')
                if self.mode != 'hold' or not self._seeded or self.active_arm not in self._pose:
                    if self._sync_ik_to_measured():
                        self._seeded = True
                    self.policy.reset()
                    self._policy_step_count = 0
                self.mode = 'hold'
                self._mech_goal_pose = None

            cur_pose = self._measured_gripper_pose(self.active_arm)
            cmd_pose = self._pose.get(self.active_arm, cur_pose).copy()
            _, _, step_pos, step_ori, info = self._evaluate_policy_step(
                f_tip=f_tip,
                m_tip=m_tip,
                delta_id0_m=delta_id0_m,
                delta_quat_wxyz=delta_quat_wxyz,
                R_HT=R_HT,
                r1_t_r0=r1_t_r0,
                capture_pose=capture_pose,
                cur_pose=cur_pose,
                cmd_pose=cmd_pose,
                cal=cal,
                target_hole=target_hole,
                dist_mm=dist_mm,
                axis_deg=axis_deg,
                yaw_deg=yaw_deg,
                dry_run=dry_run,
            )
            if not dry_run:
                self._last_action[:] = step_pos
                pose = cmd_pose.copy()
                pose[:3] += step_pos
                if np.any(step_ori != 0.0):
                    pose[3:7] = _apply_rotvec(pose[3:7], step_ori)
                self._pose[self.active_arm] = pose
            return info

    def _control(self) -> None:
        now = time.monotonic()
        cal = self.tool_calibrator.get_calibration(self.active_arm)
        r_st_t = np.asarray(cal['r_ST_T_m'], dtype=np.float64)
        r_ts = np.asarray(cal['R_TS'], dtype=np.float64)
        with self._sensor_lock:
            ft_cal_busy = self._ft_calib_running
            f_raw, t_raw, wrench_stamp = self._wrench[self.active_arm]
            f_bias, t_bias = self._wrench_bias[self.active_arm]
            force = (f_raw - f_bias).copy()
            torque = (t_raw - t_bias).copy()
            f_tip, m_tip = self._wrench_proc[self.active_arm].tip_wrench(
                r_st_t, r_ts, use_filtered=True
            )
            safe_ok, safe_msg, _, _ = self._wrench_proc[self.active_arm].check_safety(
                r_st_t, r_ts
            )
            vis_valid = (
                bool(self._vision_state['valid'])
                and (now - float(self._vision_state['stamp'])) <= self.vision_timeout_s
            )
            vis_received = float(self._vision_state['stamp']) > 0.0
            delta_pos = np.asarray(self._vision_state['delta_pos'], dtype=np.float64).copy()
            delta_quat = np.asarray(self._vision_state['delta_quat'], dtype=np.float64).copy()
            delta_quat_wxyz = np.asarray(
                self._vision_state['delta_quat_wxyz'], dtype=np.float64
            ).copy()
            R_HT = np.asarray(self._vision_state['R_HT'], dtype=np.float64).copy()
            delta_id0_m = np.asarray(self._vision_state['delta_id0_m'], dtype=np.float64).copy()
            delta_id1_m = np.asarray(self._vision_state['delta_id1_m'], dtype=np.float64).copy()
            delta_rot_id1 = np.asarray(self._vision_state['delta_rot_id1'], dtype=np.float64).copy()
            r1_t_r0 = np.asarray(self._vision_state['r1_t_r0'], dtype=np.float64).copy()
            capture_pose = (
                None
                if self._vision_state.get('capture_gripper_pose') is None
                else np.asarray(self._vision_state['capture_gripper_pose'], dtype=np.float64).copy()
            )
            target_hole = str(self._vision_state['target_hole'])
            dist_mm = float(self._vision_state['distance_mm'])
            axis_deg = float(self._vision_state['axis_angle_deg'])
            yaw_deg = float(self._vision_state['yaw_angle_deg'])
            vision_depth_mm = float(self._vision_state['insert_depth_mm'])
            vision_from_camera = self._vision_state['stamp_source'] == 'camera_read'

        if vis_valid:
            self._loc_pub.publish(
                Float64MultiArray(
                    data=[
                        float(delta_pos[0]),
                        float(delta_pos[1]),
                        float(delta_pos[2]),
                        float(delta_quat[0]),
                        float(delta_quat[1]),
                        float(delta_quat[2]),
                        float(delta_quat[3]),
                        dist_mm,
                        axis_deg,
                        yaw_deg,
                    ]
                )
            )

        with self.execution_lock, self.lock:
            if self.mode == 'float' and self._error == '/joint_states timeout':
                self._try_recover_from_timeout()
            if self.state is not State.RUNNING or self._preparing or ft_cal_busy:
                return
            reason = self.stale_reason()
            if reason:
                self.estop(reason)
                return
            if self.mode == 'mech_insert':
                try:
                    _require_mech_calibration(self.tool_calibrator.get_calibration(self.active_arm))
                except ValueError as error:
                    self.estop(str(error))
                    return

            target = self.output.positions.copy()
            target[self.gripper_slots] = np.minimum(
                np.maximum(self._grip, self._grip_lo), self._grip_hi
            )

            if self.mode == 'float':
                if not np.all(np.isfinite(self._q[self.arm_slots])):
                    self.estop('non-finite arm feedback')
                    return
                target[self.arm_slots] = self._q[self.arm_slots]
                self._last_action[:] = 0.0
                self._last_action_6d[:] = 0.0
                if self.publish_target(target) and self.ik_ready:
                    self.commit_ik(target)
                    self._seed_from_ik()
                    self._seeded = True
            else:
                if not self.ik_ready:
                    return
                if not self._seeded and not self._sync_ik_to_measured():
                    return
                self._seeded = True

                step_pos = np.zeros(3, dtype=np.float64)
                step_ori = np.zeros(3, dtype=np.float64)
                if self.mode == 'mech_insert':
                    try:
                        if self._mech_goal_pose is None:
                            is_coasted = self._vision_state.get('stamp_source') == 'fk_coast'
                            matches = not is_coasted and (
                                not vision_from_camera
                                or (
                                    vision_depth_mm == self.insert_depth_mm
                                    and (self.target_hole == 'auto' or target_hole == self.target_hole)
                                )
                            )
                            self._mech_goal_pose = self._compute_mech_target(
                                vis_valid and matches, delta_id1_m, delta_rot_id1,
                            )
                    except (ValueError, RuntimeError, np.linalg.LinAlgError) as error:
                        self.estop(f'机械目标计算异常: {error}')
                        return
                    if self._mech_goal_pose is not None:
                        step_pos = self._mech_goal_pose[:3] - self._pose[self.active_arm][:3]
                        self._pose[self.active_arm] = self._mech_goal_pose.copy()
                elif self.mode == 'policy_insert':
                    inputs = self.policy_input_status()
                    if inputs['warnings']:
                        self.get_logger().warning(
                            '; '.join(inputs['warnings']), throttle_duration_sec=2.0
                        )
                    if not safe_ok:
                        with self._sensor_lock:
                            self._safety_warning = safe_msg
                        self._sync_ik_to_measured()
                        self.mode = 'hold'
                        self.policy.reset()
                        self._last_action_6d[:] = 0.0
                    elif not vis_valid:
                        self._policy_tip_cmd = None
                        self._last_action_6d[:] = 0.0
                    elif self.policy.loaded and vis_received and wrench_stamp > 0.0:
                        try:
                            cur_pose = self._measured_gripper_pose(self.active_arm)
                            cmd_before = self._pose.get(self.active_arm, cur_pose).copy()
                            raw_act, R_HT, step_pos, step_ori, _ = self._evaluate_policy_step(
                                f_tip=f_tip,
                                m_tip=m_tip,
                                delta_id0_m=delta_id0_m,
                                delta_quat_wxyz=delta_quat_wxyz,
                                R_HT=R_HT,
                                r1_t_r0=r1_t_r0,
                                capture_pose=capture_pose,
                                cur_pose=cur_pose,
                                cmd_pose=cmd_before,
                                cal=cal,
                                target_hole=target_hole,
                                dist_mm=dist_mm,
                                axis_deg=axis_deg,
                                yaw_deg=yaw_deg,
                            )
                        except Exception as error:
                            self.estop(f'ONNX 推理异常: {error}')
                            return
                        if hasattr(self, 'policy_step_scale'):
                            new_pose = self._advance_commanded_tip_pose(
                                cmd_before,
                                cur_pose,
                                raw_act,
                                R_HT,
                                cal,
                                step_scale=float(self.policy_step_scale),
                            )
                            self._pose[self.active_arm] = new_pose
                            step_pos = new_pose[:3] - np.asarray(cmd_before[:3], dtype=np.float64)
                            step_ori = np.zeros(3, dtype=np.float64)
                        else:
                            self._pose[self.active_arm] = cur_pose

                    if np.any(step_pos != 0.0) or np.any(step_ori != 0.0):
                        if not hasattr(self, 'policy_step_scale'):
                            pose = self._pose[self.active_arm].copy()
                            pose[:3] += step_pos
                            if np.any(step_ori != 0.0):
                                pose[3:7] = _apply_rotvec(pose[3:7], step_ori)
                            self._pose[self.active_arm] = pose

                self._last_action = step_pos.copy()
                arms, _, _, _ = self.solve_ik(targets=self._pose)
                target[self.arm_slots] = arms
                if self.publish_target(target):
                    self.commit_ik(target)

        self._detail_pub.publish(
            String(
                data=json.dumps(
                    {
                        'mode': self.mode,
                        'active_arm': self.active_arm,
                        'policy_loaded': self.policy.loaded,
                        'vision_valid': vis_valid,
                        'target_hole': target_hole,
                        'insert_depth_mm': round(self.insert_depth_mm, 2),
                        'delta_pos': [round(float(v), 5) for v in delta_pos],
                        'delta_quat_wxyz': [round(float(v), 5) for v in delta_quat_wxyz],
                        'force_sensor': [round(float(v), 3) for v in force],
                        'torque_sensor': [round(float(v), 3) for v in torque],
                        'force_tip': [round(float(v), 3) for v in f_tip],
                        'moment_tip': [round(float(v), 3) for v in m_tip],
                        'r_ST_T_mm': [round(float(v * 1000.0), 2) for v in r_st_t],
                        'last_action': [round(float(v), 5) for v in self._last_action],
                    }
                )
            )
        )

    def dashboard_state(self) -> dict[str, Any]:
        now = time.monotonic()
        policy_inputs = self.policy_input_status()
        with self._sensor_lock:
            vis_valid = (
                bool(self._vision_state['valid'])
                and (now - float(self._vision_state['stamp'])) <= self.vision_timeout_s
            )
            vis_info = {
                **_input_health(
                    float(self._vision_state['stamp']), now, self.vision_timeout_s,
                    valid=bool(self._vision_state['valid']),
                ),
                'valid': vis_valid,
                'stamp_source': self._vision_state['stamp_source'],
                'fps': float(self._vision_state['fps']),
                'processed_frames': self._vision_state['processed_frames'],
                'processed_frame_age_ms': self._vision_state['processed_frame_age_ms'],
                'capture_format': self.tracker.cam_cfg['pixel_format'],
                'rate_limit_hz': self.vision_rate_hz,
                'algo_ms': float(self._vision_state['algo_ms']),
                'algo_cpu_ms': float(self._vision_state['algo_cpu_ms']),
                'preprocess_backend': self._vision_state['preprocess_backend'],
                'error': self._vision_state['error'],
                'missing_ids': list(self._vision_state['missing_ids']),
                'resolved_hole': str(self._vision_state['target_hole']),
                'nearest_hole': str(self._vision_state['nearest_hole']),
                **{key: round(float(self._vision_state[key]), 2)
                   if math.isfinite(float(self._vision_state[key])) else None
                   for key in ('distance_mm', 'lateral_err_mm', 'depth_err_mm',
                               'current_depth_mm', 'axis_angle_deg', 'yaw_angle_deg')},
                'insert_depth_mm': round(self.insert_depth_mm, 2),
                'orientation_ok': bool(self._vision_state['orientation_ok']),
                **{key: [round(float(v), digits) for v in self._vision_state[key]]
                   for key, digits in (('plug_in_id0_mm', 2), ('hole_in_id0_mm', 2),
                                       ('delta_pos', 5), ('delta_quat', 5), ('delta_quat_wxyz', 5))},
            }
            wrench_info = {}
            for s in SIDES:
                f_raw, t_raw, w_stamp = self._wrench[s]
                f_b, t_b = self._wrench_bias[s]
                f_net = f_raw - f_b
                t_net = t_raw - t_b
                cal_s = self.tool_calibrator.get_calibration(s)
                f_tip_s, m_tip_s = self._wrench_proc[s].tip_wrench(
                    cal_s['r_ST_T_m'], cal_s['R_TS'], use_filtered=True
                )
                _, _, f_raw_norm, m_raw_norm = self._wrench_proc[s].check_safety(
                    cal_s['r_ST_T_m'], cal_s['R_TS']
                )
                wrench_info[s] = {
                    **_input_health(
                        w_stamp, now, self.wrench_timeout_s,
                        valid=self._wrench_inputs[s][
                            'net' if self._wrench_source[s] == 'wrench_net' else 'raw'
                        ][1],
                    ),
                    'streams': policy_inputs['wrench_streams'][s],
                    'source': self._wrench_source[s],
                    'force': [round(float(v), 3) for v in f_net],
                    'torque': [round(float(v), 3) for v in t_net],
                    'force_tip': [round(float(v), 3) for v in f_tip_s],
                    'moment_tip': [round(float(v), 3) for v in m_tip_s],
                    'raw_force_norm_n': round(f_raw_norm, 2),
                    'raw_moment_norm_nm': round(m_raw_norm, 3),
                }
            safety_warn = self._safety_warning
            self._safety_warning = ''
            ft_cal_running = self._ft_calib_running
            ft_cal_status = self._ft_calib_status
            ft_cal_error = self._ft_calib_error
            self._ft_calib_error = ''

        q_arm7, fk_src = self._current_arm_q7(self.active_arm)
        live_fk = self.tool_calibrator.compute_live_fk(self.active_arm, q_arm7)
        live_fk['fk_source'] = fk_src
        live_fk['ft_calib_running'] = ft_cal_running
        live_fk['ft_calib_status'] = ft_cal_status

        with self.lock:
            ik_stat = self.status_ik()
            grip_cmd = {
                side: round(float(self._grip[idx]), 4) for idx, side in enumerate(SIDES)
            }
            grip_meas = {
                side: round(float(self._q[self.gripper_slots[idx]]), 4)
                for idx, side in enumerate(SIDES)
            }
            grip_state = {
                side: ('open' if self._grip[idx] > 0.5 * self._grip_open else 'closed')
                for idx, side in enumerate(SIDES)
            }
            arm_pose = ik_stat.get('limited_pose', {})

        return {
            'ok': True,
            'session_state': self.state.value,
            'preparing': self._preparing,
            'registered': self._registered,
            'ik_ready': self.ik_ready,
            'ik_backend': ik_stat['ik_backend'],
            'ik_null_status': ik_stat['ik_null_status'],
            'error': self._error or ft_cal_error or vis_info['error'],
            'safety_warning': safety_warn,
            'mode': self.mode,
            'active_arm': self.active_arm,
            'target_hole': self.target_hole,
            'available_holes': list(self.tracker.holes_mm.keys()),
            'insert_depth_mm': round(self.insert_depth_mm, 2),
            'align_orientation': self.align_orientation,
            'policy_action_gains': {
                'translation': self.policy.action_scale,
                'rotation': self.policy.action_scale,
            },
            'target_timing': self.target_timing(),
            'policy_loaded': self.policy.loaded,
            'policy_inputs': policy_inputs,
            'gripper': {
                'open_rad': round(self._grip_open, 4),
                'closed_rad': round(self._grip_closed, 4),
                'command_rad': grip_cmd,
                'measured_rad': grip_meas,
                'state': grip_state,
            },
            'vision': vis_info,
            'wrench': wrench_info,
            'tool_calibration': live_fk,
            'last_action_mm': [round(float(v * 1000.0), 3) for v in self._last_action],
            'last_action_6d': [round(float(v), 5) for v in self._last_action_6d],
            'last_policy_step': self._last_policy_step_info,
            'arm_pose': {
                s: [round(float(x), 4) for x in arm_pose.get(s, [])] for s in SIDES
            },
        }

    def status(self) -> InsertStatus:
        now = time.monotonic()
        with self._sensor_lock:
            vis_valid = (
                bool(self._vision_state['valid'])
                and (now - float(self._vision_state['stamp'])) <= self.vision_timeout_s
            )
            delta_pos = [round(float(v), 5) for v in self._vision_state['delta_pos']]
            delta_quat = [round(float(v), 5) for v in self._vision_state['delta_quat']]
            target_hole = str(self._vision_state['target_hole'])
            cur_depth = float(self._vision_state['current_depth_mm'])
        ik_stat = self.status_ik()
        is_float = self.mode == 'float'
        result: InsertStatus = {
            **ik_stat,
            'upper_executor': 'float' if is_float else 'ik',
            'arm_mode': 'passthrough' if is_float else 'ik',
            'upper_source': 'none' if is_float else 'insert',
            'gripper_source': 'insert',
            'mode': self.mode,
            'active_arm': self.active_arm,
            'policy_loaded': self.policy.loaded,
            'vision_valid': vis_valid,
            'target_hole': target_hole,
            'insert_depth_mm': round(self.insert_depth_mm, 2),
            'current_depth_mm': round(cur_depth, 2) if math.isfinite(cur_depth) else 0.0,
            'delta_pos': delta_pos,
            'delta_quat': delta_quat,
            'last_action': [round(float(v), 5) for v in self._last_action],
            'command_pose': {side: pose.tolist() for side, pose in self._pose.items()},
            'grip': self.output.positions[self.gripper_slots].tolist(),
            'joint_positions': self.output.positions[self.arm_slots].tolist(),
        }
        return result

    def destroy_node(self) -> None:
        self._vision_alive = False
        self._head_capture_requested.clear()
        with self._sensor_lock:
            proc = self._ft_calib_proc
        if proc is not None and proc.poll() is None:
            try:
                proc.send_signal(signal.SIGINT)
                proc.wait(timeout=1.0)
            except Exception:
                pass
        if self._http_server is not None:
            try:
                self._http_server.shutdown()
                self._http_server.server_close()
            except Exception:
                pass
            self._http_server = None
        if self._vision_thread is not None and self._vision_thread.is_alive():
            self._vision_thread.join(timeout=0.5)
        if self._vision_executor is not None:
            self._vision_executor.shutdown(wait=True, cancel_futures=True)
        self._vision_frames.close()
        self.tracker.stop_camera()
        super().destroy_node()


def main(args=None) -> None:
    cv2.setNumThreads(1)
    rclpy.init(args=args)
    node = InsertTask()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
