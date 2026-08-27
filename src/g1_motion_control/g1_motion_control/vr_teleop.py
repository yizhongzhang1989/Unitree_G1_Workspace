#!/usr/bin/env python3
"""VR 头显遥操作桥：WebXR 帧 -> motion_control 的 ``~/command``。

    头显 / 双手柄 --WebXR--WS--> [ 本节点：aiohttp + 50 Hz 定时器 ] --20 值--> motion_control

**采集页由本节点自己托管**（默认 ``0.0.0.0:8000``），头显直连过来，中间没有独立的
桥接进程。所以上机只要两步：``adb reverse tcp:8000 tcp:8000``，然后在头显里打开
``http://localhost:8000`` 点 Enter VR。详见 ``vr/README.md``。

映射
----
* **状态机**：**两只手同时**按 B/Y（右手 B / 左手 Y）推进一步，代替键盘的
  ``G`` / ``Enter`` / 空格。戴着头显摸不到终端，所以这三个服务得能从手柄调：

      idle / estop --按一次--> 站立(stand) --再按--> 策略接管(running) --再按--> 急停

  要两只手一起按是为了防误触。派发看的是策略层**当前上报的状态**而不是本地计数，
  所以别人用 ``ros2 service call`` 插手之后不会错位。**触发时机看这一步要干什么**：
  ``running`` 下那一步本来就是急停，**按下即走**；``engage`` / ``start`` 那两步
  **松手才走**，因为按住 ``estop_hold_s``（默认 1 s）**不看当前状态直接急停**，
  而从站立按下就推进的话长按会先把步态策略拉起来、中间那一秒机器人已经在跑了。
* **下肢**：**左摇杆**管水平速度 ``(vx, vy)``，**右摇杆**管转向 ``wz`` 与盆骨高度。
  限幅和语义都跟 ``teleop_keyboard.py`` 一致：速度松手归零，**高度是绝对量**（推着才变，
  松手停在当前值、不回弹）。
* **上肢**：``squeeze`` 是离合。按下的瞬间同时锁住「手柄位置」「手柄姿态」「策略层
  已限速的末端指令 ``limited_pose``」三个原点，之后下发的全是**相对量**，从来不是
  绝对位姿（映射见下面「坐标系」）。松开就冻结在最后一帧。
  **一次按住期间是绝对映射，允许越界**：手伸到可达域外时末端停在边界上，要让它
  重新动起来必须把越界那一段**原样空推回去**；想省掉这段就松开再按一次，重新接合
  会拿当前 ``limited_pose`` 重做锦点，越界那段作废。
* **夹爪**：``trigger`` 直接映射，**0 = 完全打开、1 = 夹紧**。注意 ``eccentric_joint``
  是 **0 rad 闭合、2.76377 rad 打开**（见 unitree_g1_description/model/Gloria-M/README.md），
  所以这里是**反向**映射；写反了就是该松手的时候夹紧。
* **归位**：单手按 **A/X**（右手 A / 左手 X）把**那一只**手送回预备姿势。轨迹不在
  这儿算：调策略层的 ``~/home_left`` / ``~/home_right``，和 B/Y 调 engage 完全同构。
  归位期间本节点**只跟随不驱动**（目标钉在 ``limited_pose`` 上），否则 50 Hz 的
  陈旧目标会和插值对着干、走完还把手臂拽回去。握 squeeze 或再按一次 A/X 即取消。

坐标系：平移和转角**各自**可选「世界」或「局部」，随便搭配出四种手感。参数是
``arm_position_frame`` / ``arm_rotation_frame``，取值 ``world`` / ``local``；默认
``world`` + ``local``。

* **世界（``world``）**：delta 原样留在 WebXR 参考空间（``local-floor``）的轴上，按
  ``_BASE_AXIS_MAP`` 落到躯干轴，**跟你怎么握手柄、按 squeeze 那一刻指哪都没关系**。
  转身面对机器人往自己前方推手，末端就往机器人身后收——这才是世界系该有的样子。
  转角走这一支时 delta 同样留在世界轴上，**左乘**到锚点姿态：绕竖直轴拧手 = 绕躯干
  竖直轴拧夹爪。``local-floor`` 重力对齐，+Y 真的是上，所以竖直方向天然准确；
  **水平朝向要自己校准**：进 VR（或长按头显的重定位键）那一刻你面朝哪，参考空间的
  -Z 就是哪，所以进 VR 时要和机器人同向站。接合日志会打印朝向线在参考空间里的方向，
  手柄水平指向机器人正前方时读数接近 ``[0, 0, -1]`` 就说明校准对了。
* **局部（``local``）**：把手柄当成夹爪本身（认为两者朝向一致）。delta 脱掉**整个
  握姿**（``origin_rot.T``）读成手柄自身轴上的量，按 ``_TOOL_AXIS_MAP`` 换到
  ``gripper_base`` 自身轴系（+Z 伸出、+X 张合），再搬到夹爪身上：平移转到锚点姿态
  下加到锚点位置，转角**右乘**到锚点姿态。于是「沿手柄指向推」= 夹爪沿自己的伸出
  方向前伸、「滚手腕」= 滚夹爪，两者都**跟着夹爪当前朝向走**，不需要方向校准。

两条都只算相对量，所以参考空间的**原点**不进入映射：换地方站不影响手感。参考系在
**接合那一刻锁死**：中途 ``ros2 param set`` 换系不会把正在跟随的目标掰过去，松手再按
一次才生效。

安全
----
* 只在策略层报 ``arms_live`` 之后才发指令；播种与接管原点取策略层的
  ``limited_pose``（IK + 关节限速后的可达指令），本节点不自己建 IK，也不做可达性夹紧。
* VR 帧超时（``frame_timeout_s``）或退出 VR 会话：速度立刻归零、高度保持，**上肢块
  整个停发**。策略层的上肢目标只在收到时覆写、没有超时，停发即冻结，同时把上肢
  所有权让给回放之类的其它发布者（照发就是两个 50 Hz 源抢同一个目标）。恢复的第一
  帧之前按当时的 ``limited_pose`` 重新对齐一次，否则陈旧目标会一帧拽过去。
* squeeze 有接合/释放迟滞；WebXR 报推算位置时冻结但保留离合，距上一帧真实跟踪位置
  超过 ``_MAX_HAND_STEP_M`` 才无跳变重锚。策略层 IK 出口的 ``arm_rate_limit`` 是最后
  一道关节跳变保护。
* 上行帧来自一个默认不鉴权的 WebSocket，所有数值都过 ``_vector``，永不把异常抛进
  50 Hz 定时器；暴露到局域网时务必设 ``token``。
"""

from __future__ import annotations

import asyncio
import hmac
import json
import ssl
import threading
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
from aiohttp import WSMsgType, web
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from g1_motion_control.command_protocol import join_command
from g1_motion_control.make_vr_cert import DEFAULT_DIR, DEFAULT_TLS_PORT

SIDES = ('left', 'right')
# 策略层当前状态 -> 双手 B/Y 按下时该调哪个服务。看实际状态而不是本地计数，
# 别人用 ros2 service call 插一手之后不会错位。
_ADVANCE = {'idle': ('engage', '站立'), 'estop': ('engage', '站立'),
            'stand': ('start', '启动策略'), 'running': ('estop', '急停')}
# 两个轴映射都是几何定死的，不是可调项。
# 转角 grip -> gripper_base：采集页那根朝向线画在 grip -Z 上（vr/index.html），而 URDF 里
# wrist_yaw -> kwr57b 的 rpy="pi/2 0 pi/2" 让 gripper_base +Z 成为伸出方向、+X 成为张合
# 方向，所以“绕线滚手腕”= grip -Z -> gripper +Z。定死 Z 后 det=+1 还剩 X/Y 一对符号，
# 实机试出来是这个（另一个 '+x -y -z' 绕这两根轴转向相反）。
_TOOL_AXIS_MAP = '-x +y -z'
# 平移 local-floor (+X 右, +Y 上, -Z 前) -> 躯干 (+X 前, +Y 左, +Z 上)。
_BASE_AXIS_MAP = '-y +z -x'
_MAX_HAND_STEP_M = 0.1
# 归位服务是阻塞的。这是本地兜底：策略层掉了不能让那只手永远卡在“跟随”里。
_HOME_TIMEOUT_S = 10.0
# 平移与转角各自的参考系。含义见模块 docstring 的「坐标系」。
_FRAME_MODES = ('world', 'local')
# 参数名 -> 存在节点上的属性名。默认值只写在 declare_parameter 那里。
_FRAME_PARAMS = {'arm_position_frame': '_pos_frame',
                 'arm_rotation_frame': '_rot_frame'}


def _vector(value, size: int):
    """WebXR 帧里的一个数组 -> 长度 ``size`` 的有限值向量；不合格返回 ``None``。

    帧来自一个默认不鉴权的 WebSocket，所以这里必须**永不抛异常**：
    定时器回调里招一个异常就会把整个 50 Hz 控制环带走。
    """
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if array.shape != (size,) or not np.all(np.isfinite(array)):
        return None
    return array


def _matrix(quat_xyzw) -> np.ndarray:
    return pin.Quaternion(np.asarray(quat_xyzw, dtype=np.float64)).matrix()


def _axis_map(spec: str) -> np.ndarray:
    """``'+x +y +z'`` -> 3x3；三个记号依次是输入系 X / Y / Z 落到目标系的哪根轴。

    必须是 x/y/z 的一个排列且 det=+1：不是旋转的话，共轭出来的“转动”带镜像，
    手转一下末端会往反方向转。
    """
    tokens = spec.replace(',', ' ').split()
    if len(tokens) != 3:
        raise ValueError(f'轴映射需要 3 个记号（输入 X/Y/Z 的去向），收到 {spec!r}')
    matrix = np.zeros((3, 3))
    for column, token in enumerate(tokens):
        axis = token[-1:].lower()
        if axis not in 'xyz' or token[:-1] not in ('', '+', '-'):
            raise ValueError(f'轴映射记号非法：{token!r}，应形如 +x 或 -z')
        matrix['xyz'.index(axis)][column] = -1.0 if token.startswith('-') else 1.0
    if (not np.allclose(matrix @ matrix.T, np.eye(3))
            or round(float(np.linalg.det(matrix))) != 1):
        raise ValueError(f'轴映射必须是 det=+1 的轴排列，收到 {spec!r}')
    return matrix


_TOOL_MAP = _axis_map(_TOOL_AXIS_MAP)
_BASE_MAP = _axis_map(_BASE_AXIS_MAP)


def _grip_pose(grip) -> tuple:
    """WebXR ``grip`` -> ``(position, rotation)``；不可用的那一半给 ``None``。

    位置和姿态**分开判**：光学跟踪丢了（``emulatedPosition``）姿态仍然可信，
    所以只把位置置空，由调用方决定冻结哪一部分。
    """
    grip = grip or {}
    orientation = _vector(grip.get('orientation'), 4)
    if orientation is None:
        return None, None
    norm = float(np.linalg.norm(orientation))
    if not 0.5 < norm < 2.0:
        return None, None
    rotation = _matrix(orientation / norm)
    position = _vector(grip.get('position'), 3)
    if position is None or grip.get('emulated_position', False):
        return None, rotation
    return position, rotation


def _quat(matrix: np.ndarray) -> np.ndarray:
    coeffs = np.asarray(pin.Quaternion(matrix).coeffs(), dtype=np.float64)
    return coeffs / np.linalg.norm(coeffs)


def _frame_mode(value, name: str) -> str:
    """校验一个参考系参数。写错宁可报错：静默回退到默认值的话，手感不对时没人知道为什么。"""
    mode = str(value).strip().lower()
    if mode not in _FRAME_MODES:
        raise ValueError(f'{name} 只能是 {" / ".join(_FRAME_MODES)}，收到 {value!r}')
    return mode


def _move_map(mode: str, origin_rot: np.ndarray, anchor_rot: np.ndarray) -> np.ndarray:
    """手柄位移（参考空间）-> 末端位移（躯干系）的 3x3。"""
    if mode == 'world':
        return _BASE_MAP                              # 参考空间轴 -> 躯干轴，与握法无关
    return anchor_rot @ _TOOL_MAP @ origin_rot.T      # 手柄自身轴 -> 夹爪自身轴 -> 躯干


def _turn_pose(mode: str, rotation: np.ndarray, origin_rot: np.ndarray,
               anchor_rot: np.ndarray) -> np.ndarray:
    """手柄握姿 -> 末端目标姿态（躯干系 3x3）；``rotation is origin_rot`` 时恰好是锚点。"""
    if mode == 'local':
        # 脱掉整个握姿、换到夹爪自身轴系后**右乘**：滚手腕就是滚夹爪。
        return anchor_rot @ (_TOOL_MAP @ origin_rot.T @ rotation @ _TOOL_MAP.T)
    # delta 留在世界轴上、共轭到躯干轴后**左乘**：绕竖直轴拧手就是绕躯干竖直轴拧夹爪。
    return (_BASE_MAP @ rotation @ origin_rot.T @ _BASE_MAP.T) @ anchor_rot


async def _safe_send(ws, text: str) -> None:
    try:
        await ws.send_str(text)
    except (ConnectionResetError, RuntimeError, OSError):
        pass


class VRTeleop(Node):

    def __init__(self) -> None:
        super().__init__('vr_teleop')
        p = self.declare_parameter
        # adb reverse 把头显的 localhost:8000 转到本机，所以只用头显时可以把
        # bind_host 收成 127.0.0.1；默认 0.0.0.0 是为了让别的机器能开 /monitor。
        self._host = p('bind_host', '0.0.0.0').get_parameter_value().string_value
        self._port = int(p('bind_port', 8000).get_parameter_value().integer_value)
        # HTTPS 另开一个口，和明文口**同时**听。两条链路服务的是同一份采集页，
        # 互不影响：adb reverse 只能转明文，而局域网直连只能走 HTTPS（WebXR 要求安全上下文）。设成 0 就不开 TLS 口。
        self._tls_port = int(p('tls_port', DEFAULT_TLS_PORT).get_parameter_value().integer_value)
        # 非空则所有 HTTP/WS 入口都要带 ?token=；服务暴露在局域网上时务必设。
        self._token = p('token', '').get_parameter_value().string_value
        # 默认指向 make_vr_cert 的输出位置，所以签过证书之后直接 ros2 run 也是
        # 双口，不必非走 launch。
        self._cert = p('tls_cert', str(DEFAULT_DIR / 'cert.pem')).get_parameter_value().string_value
        self._key = p('tls_key', str(DEFAULT_DIR / 'key.pem')).get_parameter_value().string_value
        self._rate = float(p('rate_hz', 50.0).get_parameter_value().double_value)
        # 限幅与 teleop_keyboard.py 同一组，且与策略层 command_limits 对齐：写得比它大
        # 只会让摇杆顶端摸起来像卡住。
        self._vx_max = float(p('vx_max', 0.4).get_parameter_value().double_value)
        self._vy_max = float(p('vy_max', 0.2).get_parameter_value().double_value)
        self._wz_max = float(p('wz_max', 1.0).get_parameter_value().double_value)
        self._height0 = float(p('height', 0.80).get_parameter_value().double_value)
        self._h_lo = float(p('height_min', 0.50).get_parameter_value().double_value)
        self._h_hi = float(p('height_max', 0.80).get_parameter_value().double_value)
        # 摇杆推到底时的高度变化率。策略层自己也按 0.15 m/s 限速（训练里高度指令
        # 就是这个速率缓变），所以这里取同值，再大也过不去。
        self._height_rate = float(p('height_rate', 0.15).get_parameter_value().double_value)
        # 死区：摇杆回中会残留零点漂移，不处理机器人会一直慢慢走。
        self._deadzone = float(p('stick_deadzone', 0.05).get_parameter_value().double_value)
        # squeeze 是模拟量，按到底才是 1.0；用 0.5 判定"按下"，怕误触就往上调。
        self._squeeze_on = float(p('squeeze_threshold', 0.5).get_parameter_value().double_value)
        # 模拟按钮在阈值附近有噪声。接合后降到 80% 阈值才释放，避免一帧帧断开/重建原点。
        self._squeeze_off = 0.8 * self._squeeze_on
        self._arm_scale = float(p('arm_scale', 1.0).get_parameter_value().double_value)
        # 平移与转角各自选参考系，四种搭配（含义见模块 docstring 的「坐标系」）。
        self._pos_frame = _frame_mode(
            p('arm_position_frame', 'world').get_parameter_value().string_value,
            'arm_position_frame')
        self._rot_frame = _frame_mode(
            p('arm_rotation_frame', 'world').get_parameter_value().string_value,
            'arm_rotation_frame')
        self._timeout = float(p('frame_timeout_s', 0.3).get_parameter_value().double_value)
        self._grip_open = float(p('gripper_open', 2.76377472169236).get_parameter_value().double_value)
        self._grip_closed = float(p('gripper_closed', 0.0).get_parameter_value().double_value)
        # 双手 B/Y 两次推进之间的最小间隔，防按键抖动连发。
        self._button_cooldown = float(
            p('button_cooldown_s', 1.0).get_parameter_value().double_value)
        # 双手 B/Y 按住这么久 -> 不看当前状态直接急停。非正值会让按下即急停、短按全废。
        self._estop_hold = float(
            p('estop_hold_s', 1.0).get_parameter_value().double_value)
        if self._estop_hold <= 0.0:
            raise ValueError('estop_hold_s 必须为正（秒）')

        self._lock = threading.Lock()
        self._frame: dict | None = None
        self._frame_stamp = 0.0
        # 下面两个只在 aiohttp 那个事件循环里读写，不需要加锁。
        self._device = None
        self._monitors: set = set()
        self._status: dict = {}
        # 下面这几个只在控制线程里读写，不需要加锁。
        self._seeded = False
        # 头显连接的边沿。和 _seeded 分开：那个管 arms_live 的接管边沿，还会复位盆骨高度。
        self._offline = False
        self._pose: dict[str, np.ndarray] = {}
        self._grip = np.zeros(2)
        self._clutch: dict[str, tuple | None] = {side: None for side in SIDES}
        self._twist = np.zeros(3)
        self._height = self._height0
        # 初始当作"按着"：连上之后必须真的松手再按才算一次，避免启动瞬间误触。
        self._button_held = True
        self._button_stamp = 0.0
        # 本次按住的起始时刻，以及“已被长按急停消费掉”（消费了就不再补一次短按）。
        self._button_down: float | None = None
        self._button_used = True
        # A/X 归位：每侧一个超时时刻（None = 没在归位）加一个按键沿。
        # 这两个会被服务回调那个线程碰，但只是整个替换字典项，不需要加锁。
        self._homing: dict[str, float | None] = {side: None for side in SIDES}
        self._home_held = {side: True for side in SIDES}

        stream = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.BEST_EFFORT)
        control = MutuallyExclusiveCallbackGroup()
        slow = ReentrantCallbackGroup()
        self._message = Float64MultiArray()
        self._publisher = self.create_publisher(
            Float64MultiArray,
            p('command_topic', '/motion_control/command')
            .get_parameter_value().string_value, stream)
        self.create_subscription(
            String, p('status_topic', '/motion_control/status')
            .get_parameter_value().string_value, self._on_status, 10, callback_group=control)
        self.create_timer(1.0 / self._rate, self._tick, callback_group=control)
        # 服务得用 Reentrant 组 + call_async：engage/estop 里的 switch_controller 会阻塞
        # 好几秒（硬件卸力斜坡），同步等会把 50 Hz 定时器一起冻住。
        policy = p('policy_node', '/motion_control') \
            .get_parameter_value().string_value.rstrip('/')
        self._trigger = {
            name: self.create_client(Trigger, f'{policy}/{name}', callback_group=slow)
            for name in ('engage', 'start', 'estop',
                         'home_left', 'home_right', 'home_cancel')
        }
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._alive = True
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        self.get_logger().info(
            '头显里打开采集页并 Enter VR。'
            '双手同时按 B/Y 推进：站立 -> 启动策略 -> 急停（急停那一步按下即走），'
            f'按住 {self._estop_hold:.1f} s 不看状态直接急停；'
            'squeeze 按住才跟随手，trigger 控夹爪（0 开 / 1 夹）；'
            '单手按 A/X 把那只手送回预备姿势。'
            f'上肢参考系：平移 {self._pos_frame} / 转角 {self._rot_frame}。')

    def _on_set_parameters(self, parameters):
        """现场换上肢参考系。改完的那套要**下一次离合接合**才生效：正按着的时候换系，
        目标会从锚点按新映射重算、凭空跳一下。"""
        pending = {}
        for parameter in parameters:
            attribute = _FRAME_PARAMS.get(parameter.name)
            if attribute is None:
                continue
            try:
                pending[attribute] = _frame_mode(parameter.value, parameter.name)
            except ValueError as error:
                return SetParametersResult(successful=False, reason=str(error))
            self.get_logger().info(
                f'{parameter.name} -> {pending[attribute]}（松开 squeeze 再按一次才生效）')
        for attribute, mode in pending.items():
            setattr(self, attribute, mode)
        return SetParametersResult(successful=True)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -- WebXR 桥：头显直连本节点，中间没有独立进程 -------------------------------

    def _serve(self) -> None:
        """后台线程里跑 aiohttp：托管采集页、收头显上行帧。

        ``web.run_app`` 要装信号处理器、只能在主线程跑，所以用低层的 ``AppRunner``。
        帧只覆盖 ``_frame`` 的最新一份、不排队：头显是 72~90 Hz、控制环 50 Hz，
        多出来的帧本来就该丢，排队只会积压延迟。
        """
        static = Path(get_package_share_directory('g1_motion_control')) / 'vr'

        def page(name: str):
            async def handler(_: web.Request) -> web.StreamResponse:
                return web.FileResponse(static / name)
            return handler

        app = web.Application()
        app.add_routes([
            web.get('/', page('index.html')),
            web.get('/monitor', page('monitor.html')),
            web.get('/state', self._on_state),
            web.post('/haptic', self._on_haptic),
            web.get('/ws/device', self._on_device),
            web.get('/ws/subscribe', self._on_monitor),
        ])
        context = None
        if self._tls_port and self._cert and self._key:
            if all(Path(f).is_file() for f in (self._cert, self._key)):
                context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
                context.load_cert_chain(self._cert, self._key)
            else:
                # 不静默：以为在跑 HTTPS 而实际没有，比直接起不来更坏。
                self.get_logger().warning(
                    f'证书不在 {Path(self._cert).parent}，只开明文口。签一张：'
                    f'ros2 run g1_motion_control make_vr_cert')

        async def run() -> None:
            runner = web.AppRunner(app, handle_signals=False)
            await runner.setup()
            # 明文口：adb reverse 把头显的 localhost:8000 转到这里。它是基础路径，
            # 任何情况下都要起来。
            await web.TCPSite(runner, self._host, self._port).start()
            # 等真的绑上了再报地址：在 __init__ 里打这行的话，端口被占着也照样说
            # “已就绪”，反而把人往错处引。
            self.get_logger().info(
                f'WebXR 明文口已就绪：http://{self._host}:{self._port}'
                f'（配 adb reverse tcp:{self._port} tcp:{self._port} 用）')
            if context is not None:
                try:
                    await web.TCPSite(runner, self._host, self._tls_port,
                                      ssl_context=context).start()
                except OSError as error:
                    # TLS 口起不来不能连带把明文口拉下水，adb 那条路得继续能用。
                    self.get_logger().error(
                        f'HTTPS 口 {self._tls_port} 起不来：{error}')
                else:
                    self.get_logger().info(
                        f'WebXR HTTPS 口已就绪：https://<本机局域网IP>:{self._tls_port}'
                        f'（自签证书，头显里点「高级 → 继续前往」）')
            while self._alive:
                await asyncio.sleep(0.25)
            await runner.cleanup()

        try:
            asyncio.run(run())
        except OSError as error:
            # 最常见的是明文口被占：上一轮节点没退干净。
            self.get_logger().error(
                f'WebXR 桥起不来（{self._host}:{self._port}）：{error}。'
                f'查一下 `ss -lptn "sport = :{self._port}"`')

    def _authorize(self, request: web.Request) -> None:
        if not self._token:
            return
        got = request.query.get('token') or request.headers.get('X-Auth-Token', '')
        if not hmac.compare_digest(got, self._token):
            raise web.HTTPUnauthorized(text='invalid token')

    async def _on_device(self, request: web.Request) -> web.StreamResponse:
        """头显上行：WebXR 帧。断线立刻把 _frame 清掉，不用等 frame_timeout_s。"""
        self._authorize(request)
        ws = web.WebSocketResponse(max_msg_size=1 << 20, heartbeat=20)
        await ws.prepare(request)
        self._device = ws
        self.get_logger().info('头显已连接')
        try:
            async for message in ws:
                if message.type is not WSMsgType.TEXT:
                    continue
                try:
                    frame = json.loads(message.data)
                except ValueError:
                    continue
                with self._lock:
                    self._frame = frame
                    self._frame_stamp = self._now()
                self._fanout(message.data)
        finally:
            if self._device is ws:      # 刷新页面时新连接已经建好了，别把它清掉。
                self._device = None
                with self._lock:
                    self._frame = None
                self.get_logger().warning('头显已断开，已停车并冻结上肢')
        return ws

    async def _on_monitor(self, request: web.Request) -> web.StreamResponse:
        """``/monitor`` 页面下行。没人开监控页时这条路径完全不产生开销。"""
        self._authorize(request)
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        self._monitors.add(ws)
        try:
            async for _ in ws:
                pass
        finally:
            self._monitors.discard(ws)
        return ws

    def _fanout(self, text: str) -> None:
        for ws in list(self._monitors):
            if ws.closed:
                self._monitors.discard(ws)
            else:
                # 不 await：监控页跟不上是它自己的事，绝不能拖住头显这条链路。
                asyncio.create_task(_safe_send(ws, text))

    async def _on_state(self, request: web.Request) -> web.StreamResponse:
        """``curl localhost:8000/state`` 的自检口：seq 在涨就说明帧在流。"""
        self._authorize(request)
        with self._lock:
            frame = self._frame
        return web.json_response(frame or {'seq': 0, 'session_active': False})

    async def _on_haptic(self, request: web.Request) -> web.StreamResponse:
        """POST ``{"hand": "right", "intensity": 0.6, "duration": 80}`` -> 手柄震动。"""
        self._authorize(request)
        body = await request.json()
        if body.get('hand') not in SIDES:
            raise web.HTTPBadRequest(text="hand must be 'left' or 'right'")
        device = self._device
        if device is None or device.closed:
            return web.json_response({'delivered': False})
        await _safe_send(device, json.dumps(
            {'type': 'haptic', 'hand': body['hand'],
             'intensity': float(body.get('intensity', 0.6)),
             'duration': float(body.get('duration', 80))}, separators=(',', ':')))
        return web.json_response({'delivered': True})

    # -- ROS 输入 --------------------------------------------------------------

    def _on_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except ValueError:
            return
        with self._lock:
            self._status = status
    # -- 控制环 ----------------------------------------------------------------

    def _tick(self) -> None:
        with self._lock:
            frame, stamp = self._frame, self._frame_stamp
            status = self._status
        # 不新鲜的帧一律当成没有帧：往下就只剩"有帧 / 没帧"两种情况，不必再把
        # `frame if fresh else None` 到处传一遍。
        if frame is not None and not (frame.get('session_active')
                                      and self._now() - stamp <= self._timeout):
            frame = None
        # 状态机按键任何状态下都要响应——idle / stand 里还没开始发指令就得能按。
        self._check_button(frame)
        # 跟的是手臂接管而不是策略：站立插值走完就能动手，不必等下肢策略启动。
        if not status.get('arms_live'):
            self._seeded = False        # 失去接管后重新播种，不留旧目标。
            return
        if not self._seeded:
            if not self._reseed(status):
                return                  # 手臂刚接管，正解还没落到 status 上。
            self._grip = np.asarray(status.get('grip') or (0.0, 0.0),
                                    dtype=np.float64)
            self._homing = {side: None for side in SIDES}
            self._home_held = {side: True for side in SIDES}
            self._twist = np.zeros(3)
            self._height = self._height0
            self._seeded = True
            self.get_logger().info('手臂已接管，双臂目标已按策略层发布的位姿播种')

        if frame is None:
            # 掉帧/退出 VR：停车，上肢块**整个不发**。策略层的 `_pose` 只在收到时覆写、
            # 没有超时，不发就等于冻结，同时把上肢让给回放之类的其它发布者——
            # 照发就是两个 50 Hz 源抢同一个目标。
            if not self._offline:       # 下面几项是状态复位，断联期间重做没意义
                self._offline = True
                self._twist = np.zeros(3)
                self._cancel_home('VR 帧不新鲜')
                self._home_held = {side: True for side in SIDES}
                self.get_logger().warning('VR 帧不新鲜，已停车并交还上肢')
            self._send(base=(*self._twist, self._height))
            return

        if self._offline:
            # 断联期间别人可能把手臂开走了，拿陈旧目标恢复会一帧拽过去（实测 418 mm）。
            if not self._reseed(status):
                return
            self._offline = False
            self.get_logger().info('头显恢复，双臂目标已按策略层当前位姿重新对齐')

        self._update_command(frame)
        self._update_arms(frame, status.get('limited_pose') or {})

        base = (*self._twist, self._height)
        if status.get('arm_mode', 'ik') == 'ik':
            self._send(base=base, left=self._pose['left'],
                       right=self._pose['right'], grip=self._grip)
        else:
            # 透传模式下那两个 7 值块是关节角。VR 只会算末端位姿，填进去就是拿
            # 四元数当关节目标发，所以这时只发下肢。
            self.get_logger().warning('策略层是逐关节透传模式，VR 只发下肢指令',
                                      throttle_duration_sec=5.0)
            self._send(base=base)

    def _reseed(self, status: dict) -> bool:
        """双臂目标重设为 ``limited_pose``、清掉离合；正解还没落下来就返回假。

        用 IK + 关节限速后真正下发的末端指令：一定可达、无编码器静差，又不会把别的
        发布者已经推飞的笛卡尔目标原样复读回去。
        """
        limited = status.get('limited_pose') or {}
        if not all(side in limited for side in SIDES):
            return False
        self._pose = {side: np.asarray(limited[side], dtype=np.float64)
                      for side in SIDES}
        self._clutch = {side: None for side in SIDES}
        return True

    def _send(self, **chunks) -> None:
        self._message.data = join_command(**chunks)
        self._publisher.publish(self._message)

    def _check_button(self, frame) -> None:
        """双手同时按 B/Y：推进状态机一步，按满 ``estop_hold_s`` 则不看状态直接急停。

        推进的时机**看这一步要干什么**：``running``（策略行走 + 上肢 VR）那一步本来
        就是急停，**按下即走**；``engage`` / ``start`` 则**松手才走**——按下就推进的话，
        从站立长按会先把步态策略拉起来、再急停，中间那一秒机器人已经在跑了。

        帧不新鲜时按"按住且已消费"处理：长按计时作废，且恢复后必须真的松手再按
        才算一次，否则一次掉帧就可能凭空触发一步。
        """
        if frame is None:
            self._button_held, self._button_down, self._button_used = True, None, True
            return
        pressed = all(((frame.get(side) or {}).get('buttons') or {}).get('b_y')
                      for side in SIDES)
        now = self._now()
        down, up = pressed and not self._button_held, self._button_held and not pressed
        if down:
            self._button_down, self._button_used = now, False
        held_for = now - self._button_down if self._button_down is not None else 0.0
        cooled = now - self._button_stamp > self._button_cooldown

        if pressed and not self._button_used and held_for >= self._estop_hold:
            self._button_used, self._button_stamp = True, now
            self.get_logger().warning(f'双手长按 B/Y {held_for:.1f} s -> 直接急停')
            self._call('estop', '急停')
        elif cooled and (up and not self._button_used
                         # 这一步本来就是急停，没有“先把策略拉起来”的风险，按下即走
                         or down and self._next_step()[1] == 'estop'):
            self._button_used, self._button_stamp = True, now
            self._advance()
        self._button_held = pressed

    def _next_step(self) -> tuple[str, str, str]:
        """当前上报的状态，以及“推进一步”该调的服务与中文名（不认得就两个空串）。"""
        with self._lock:
            state = self._status.get('state', '')
        return (state, *_ADVANCE.get(state, ('', '')))

    def _advance(self) -> None:
        state, name, label = self._next_step()
        if not name:
            self.get_logger().warning(
                f'收到 B/Y，但策略层状态是「{state or "未知"}」，先确认它起来了')
            return
        self.get_logger().info(f'B/Y -> {label}')
        self._call(name, label)

    def _call(self, name: str, label: str) -> None:
        client = self._trigger[name]
        if not client.service_is_ready():
            self.get_logger().warning(f'{label}：服务 {name} 还没就绪')
            return
        client.call_async(Trigger.Request()).add_done_callback(
            lambda future: self._report(label, future))

    def _report(self, label: str, future) -> None:
        result = future.result()
        if result is None:
            self.get_logger().error(f'{label}：调用失败')
        elif result.success:
            self.get_logger().info(f'{label}：{result.message}')
        else:
            # 最常见的是"站立插值还没走完"——等满 stand_s 再按一次即可。
            self.get_logger().warning(f'{label}被拒绝：{result.message}')

    # -- A/X 归位 ---------------------------------------------------------------

    def _check_home(self, side: str, pressed: bool, clutching: bool) -> None:
        """A/X 上升沿：把这一只手交给策略层的归位服务。再按一次或握 squeeze 都取消。"""
        deadline = self._homing[side]
        if deadline is not None and self._now() > deadline:
            self._homing[side] = deadline = None
            self.get_logger().warning(f'{side} 归位服务超时没返回，已恢复手动')
        if deadline is not None:
            if clutching or (pressed and not self._home_held[side]):
                self._cancel_home('squeeze 接管' if clutching else '再按一次 A/X')
        elif pressed and not self._home_held[side] and not clutching:
            self._start_home(side)
        self._home_held[side] = pressed

    def _start_home(self, side: str) -> None:
        client = self._trigger[f'home_{side}']
        if not client.service_is_ready():
            self.get_logger().warning(f'{side} 归位：服务还没就绪')
            return
        self._homing[side] = self._now() + _HOME_TIMEOUT_S
        self._clutch[side] = None
        self.get_logger().info(f'A/X -> {side} 归位')
        client.call_async(Trigger.Request()).add_done_callback(
            lambda future: self._home_returned(side, future))

    def _home_returned(self, side: str, future) -> None:
        self._homing[side] = None
        self._report(f'{side} 归位', future)

    def _cancel_home(self, reason: str) -> None:
        """取消两侧归位。标志留给服务返回时清：取消也是走同一条返回路径。"""
        if not any(deadline is not None for deadline in self._homing.values()):
            return
        client = self._trigger['home_cancel']
        if not client.service_is_ready():
            self.get_logger().warning('取消归位：服务还没就绪')
            return
        client.call_async(Trigger.Request())
        self.get_logger().info(f'取消归位（{reason}）')

    def _stick(self, frame: dict, side: str) -> np.ndarray:
        """取一个摇杆的 ``[x, y]``，已去死区。"""
        axes = _vector((frame.get(side) or {}).get('thumbstick'), 2)
        if axes is None:
            return np.zeros(2)
        magnitude = float(np.linalg.norm(axes))
        if magnitude <= self._deadzone:
            return np.zeros(2)
        # 径向死区 + 出区后重新归一化，否则刚越过死区那一下速度是阶跃的。
        return axes * ((magnitude - self._deadzone)
                       / (1.0 - self._deadzone) / magnitude)

    def _update_command(self, frame: dict) -> None:
        """左摇杆 -> 水平速度，右摇杆 -> 转向与高度。

        xr-standard 的 Y 轴是**下为正**，而机器人 X 前 / Y 左 / 逆时针为正，
        所以四个轴全取负号。
        """
        left, right = self._stick(frame, 'left'), self._stick(frame, 'right')
        self._twist = np.array([-left[1] * self._vx_max,
                                -left[0] * self._vy_max,
                                -right[0] * self._wz_max])
        # 高度和键盘的 I/K 一样是绝对量：推着才变，松手停在当前值，不回弹。
        self._height = min(max(
            self._height - right[1] * self._height_rate / self._rate,
            self._h_lo), self._h_hi)

    def _rebase(self, side: str, position, rotation) -> tuple:
        """以当前手柄位姿为原点、当前末端目标为锚重建基准。

        两条映射各走哪套参考系也在这里**锁死**，中途换系不会抽掉正在跟的那一次。
        """
        return (position, rotation, self._pose[side].copy(), position,
                (self._pos_frame, self._rot_frame))

    def _update_arms(self, frame: dict, limited: dict) -> None:
        """手柄位姿 -> 末端目标；``limited`` 只用来选离合接合时的锦点。

        接合期间是**绝对映射**：目标每帧从锦点重算，不接受任何可达性反馈。越界是
        允许的：末端停在边界上，要让它重新动起来就得把越界那段原样空推回去；
        松开再按一次则重新以 ``limited_pose`` 为锦，越界那段作废。

        **别拿可达性反馈去修锦点来“取消空推”。** 那样够不着的那段位移会被吸进锦点、
        把整个手↔手臂映射平移：实测（真 URDF + 真 IK）前推 80 cm 再把手原路收回接合点，
        手臂停在接合点**后方 383 mm**；推出后只缩 5 cm（人手仍够不着）手臂就已经后退 35 mm。
        现在这三项实测是 0 mm / 1 mm，而重按后缩 5 cm 手臂跟着退 49 mm。

        越界时不需要额外保护：目标飘出可达域后 IK 本就只能停在边界上，而
        ``ik_limit_upper``（肘收到 1.4）与 ``ik_max_step_pos`` 已经把卡死和抖动都採了。
        实测推远-拉回 8 轮 × 4 个方向：末端单帧最大 10.3~10.8 mm、逃生种子 0 次、
        回原位残差 0.4~1.0 mm。再加一道可达性夹紧反而把单帧推到 13.5~14.6 mm。
        """
        for index, side in enumerate(SIDES):
            hand = frame.get(side)
            if not hand:
                continue                  # 该侧输入缺失：目标和夹爪都保持。
            buttons = hand.get('buttons') or {}
            position, rotation = _grip_pose(hand.get('grip'))
            squeeze = float(buttons.get('squeeze', 0.0) or 0.0)
            clutch = self._clutch[side]
            active = squeeze >= (self._squeeze_off if clutch is not None else self._squeeze_on)
            self._check_home(side, bool(buttons.get('a_x')), active)
            if self._homing[side] is not None:
                # 归位由策略层驱动，这里只把目标钉在它已经限速的指令上：
                # 自己再发一份就是两个目标对着干，走完还会把手臂拽回去。
                if side in limited:
                    self._pose[side] = np.asarray(limited[side], dtype=np.float64)
            elif position is None:
                # 只冻结，不动 clutch。**千万别在这里作废 ``previous``**：那样恢复帧会
                # 无条件重锚，把跨这一帧的位移整段丢掉。emulated 标志逐帧抖时会按比例
                # 丢运动（实测 1/2 抖动 -> 手走 900 mm 目标走 0 mm，完全冻住）。
                if clutch is not None:
                    self.get_logger().warning(
                        f'{side} 手柄失去光学位置，已冻结',
                        throttle_duration_sec=1.0)
            elif active:
                if clutch is None:
                    # 接管瞬间位移和转角都恒为 0，所以不会跳。
                    self._pose[side] = np.asarray(
                        limited.get(side, self._pose[side]), dtype=np.float64)
                    clutch = self._clutch[side] = self._rebase(side, position, rotation)
                    # world 系的水平朝向只由参考空间决定，所以这行是唯一的校准自检：
                    # 手柄水平指向机器人正前方时读数应接近 [0, 0, -1]。
                    self.get_logger().info(
                        f'{side} 离合接合｜朝向线指向参考空间 '
                        f'{np.round(-rotation[:, 2], 2)}（+Y 上, -Z 前）'
                        f'｜平移 {self._pos_frame} / 转角 {self._rot_frame}')
                origin, origin_rot, anchor, previous, modes = clutch
                # 距上一帧**真实跟踪到**的位置超过这么多，就不可能是人手的运动。
                # 冻结期间这个差会累积，所以长时丢跟踪 + 大幅挪手自然落进这一支。
                if np.linalg.norm(position - previous) > _MAX_HAND_STEP_M:
                    origin, origin_rot, anchor, previous, modes = self._rebase(
                        side, position, rotation)
                    self.get_logger().warning(
                        f'{side} 手柄单帧跳变超过 {_MAX_HAND_STEP_M:.2f} m，已无跳变重置原点',
                        throttle_duration_sec=1.0)
                pose = self._pose[side]
                pos_frame, rot_frame = modes
                anchor_rot = _matrix(anchor[3:])
                # 两条 delta 各走各的参考系，别合并。理由见模块 docstring 的「坐标系」。
                pose[:3] = anchor[:3] + self._arm_scale * (
                    _move_map(pos_frame, origin_rot, anchor_rot) @ (position - origin))
                pose[3:] = _quat(_turn_pose(rot_frame, rotation, origin_rot, anchor_rot))
                self._clutch[side] = (origin, origin_rot, anchor, position, modes)
            elif clutch is not None:
                self._clutch[side] = None       # 松开就冻结在最后一帧。
                self.get_logger().info(f'{side} 离合断开，手臂冻结')
            # trigger: 0 -> 完全打开、1 -> 夹紧。eccentric 是 0 闭合、2.76 打开，反向。
            if 'trigger' in buttons:
                trigger = min(max(float(buttons['trigger'] or 0.0), 0.0), 1.0)
                self._grip[index] = self._grip_open + \
                    (self._grip_closed - self._grip_open) * trigger

    def shutdown(self) -> None:
        self._alive = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VRTeleop()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
