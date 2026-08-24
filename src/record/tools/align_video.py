"""估计视频相对机器人状态的时间偏移。**纯 Python + numpy，不 import rclpy。**

延迟由两段构成，性质完全不同：

* **相机侧**（曝光积分 + 读出/ISP + 编码）：几十毫秒量级，但**基本固定**。
  自动曝光在暗处会拉长积分时间，所以光照变化时它会跟着变。
* **传输侧**（网络排队 + 内核缓冲 + ffmpeg 打戳的线程调度）：**逐帧抖动大但可消除**。
  ``session_reader.fitted_pts`` 的分段等间隔重建已经把它压到窗中心偏差 4.8 ms、
  单帧残差 p5..p95 跨 21 ms（实测 1080p@25 一路 90 秒）。

所以**本模块要标的是相机侧那段固定延迟**，不是传输抖动 —— 后者已经在重建里解决了。
既然它固定，正常情况下一次 session 标一次即可；只有光照明显变化时才需要重标，
分窗功能用来抽查这一点。

标法：腕相机装在手腕上，手臂一动画面就整体动，图像运动强度与该臂关节角速度强相关，
互相关峰值即延迟。采集时手臂一直在动，不需要专门的标定动作。

**注意 RTCP 这条路走不通**：RTP 标准的 Sender Report 会把媒体时钟映射到发送端 NTP
墙钟，有它就不需要本模块。实测这两台相机不实现——用最小 RTSP 客户端抓 30 秒，
RTP 包收到 4070 个而 SR 一个没有，主动发接收报告催也没用；``ffprobe`` 的
``start_time_realtime`` 同样为空。换成支持 RTCP SR + NTP 的相机后，本模块可退化为
验证工具。

运动信号有两个来源：

* ``motion_from_packets``  直接读 H.264 包大小，**零解码**。运动越剧烈 P 帧残差越大。
  实测静止场景 P 帧中位 429 B、动态范围 2.9 倍，手臂动起来会高一个量级。
* ``motion_from_frames``   解码成小灰度图算帧差，准但要解码。包大小信噪比不够时才用。

机器人侧不用正解：相机随手腕转，图像运动主要由相机角速度决定，而它约等于该臂各关节
角速度之和 —— 直接取 ``joint_states`` 里那 7 个速度的模即可，B 侧没有 pinocchio 也能算。
"""

from __future__ import annotations

import subprocess
from dataclasses import dataclass

import numpy as np

ARM_JOINTS = {
    'left': ('left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
             'left_shoulder_yaw_joint', 'left_elbow_joint',
             'left_wrist_roll_joint', 'left_wrist_pitch_joint',
             'left_wrist_yaw_joint'),
    'right': ('right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
              'right_shoulder_yaw_joint', 'right_elbow_joint',
              'right_wrist_roll_joint', 'right_wrist_pitch_joint',
              'right_wrist_yaw_joint'),
}
WRIST_SIDE = {'wrist_left': 'left', 'wrist_right': 'right'}

#: 已标定的相机管线延迟定义在 `session_reader.CAMERA_DELAY_S`，这里不再存第二份。
#: `video_pts()` 默认已经把它减掉，所以**本模块必须拿未修正的值**，
#: 否则量到的是修正后的残差而不是延迟本身。
#:
#: **两路都约 110 ms，不确定度 ±15 ms**（2026-08-24，两台相机参数统一之后）。
#: 单次互相关的峰高只有 0.17~0.24，没到 0.3 的判据线，但下面几条互相独立的证据都指向
#: 同一个数，合起来足够用：
#:
#: * 同一段数据换 4 种机器人侧信号（全 7 轴 / 腕 3 轴 / 肘+腕 / 末端），延迟都落在
#:   98~116 ms；
#: * 换窗长 4/12/16 s 重算，左路 +100.3 / +103.9 / +101.4，右路 +101.3 / +109.8 / +115.1；
#: * 参数统一前 `.97` 在高置信度下测得 +114 ms（峰高 0.60），与本次 +101 一致；
#: * **改帧率前就预测过** `.98` 的 +195 ms 会向 `.97` 靠拢（那 81 ms 差 ≈ 25 fps 的两个
#:   帧周期），改完实测 +115 ms，预测应验。
#:
#: 头部没有值：D435i 给的是硬件时间戳，且它不随手臂动，互相关这条路对它不适用。

GRID_HZ = 50.0          # 两路重采样到的公共网格
MAX_LAG_S = 0.6         # 搜索范围，覆盖曝光+ISP+编码+网络的全部量级


@dataclass
class Alignment:
    delay: float          # 视频比机器人状态晚多少秒；正值表示要把视频时间戳减去它
    score: float          # 归一化互相关峰值，0~1
    margin: float         # 峰值比次高旁瓣高多少，低了说明峰不显著
    samples: int

    @property
    def trustworthy(self) -> bool:
        return self.samples >= 200 and self.score >= 0.3 and self.margin >= 0.1

    def as_dict(self) -> dict:
        return {'delay_ms': round(self.delay * 1000, 1),
                'score': round(self.score, 3), 'margin': round(self.margin, 3),
                'samples': self.samples, 'trustworthy': self.trustworthy}


def motion_from_packets(mkv_path) -> tuple[np.ndarray, np.ndarray]:
    """从 H.264 包大小取运动强度。返回 (帧序号, 强度)，零解码。

    关键帧字节数比 P 帧大两三个数量级（实测 761 倍），必须剔除后插值，
    否则每个 GOP 边界都会造出一个假的运动尖峰。
    """
    out = subprocess.run(
        ['ffprobe', '-v', 'error', '-select_streams', 'v:0',
         '-show_entries', 'packet=size,flags', '-of', 'csv=p=0', str(mkv_path)],
        capture_output=True, text=True, check=True).stdout
    rows = [line.split(',') for line in out.strip().splitlines() if line]
    if not rows:
        return np.empty(0), np.empty(0)
    size = np.array([float(r[0]) for r in rows])
    key = np.array(['K' in r[1] for r in rows])
    idx = np.arange(size.size, dtype=np.float64)
    if key.all():
        return idx, size
    size[key] = np.interp(idx[key], idx[~key], size[~key])
    return idx, size


def motion_from_frames(mkv_path, width: int = 160) -> tuple[np.ndarray, np.ndarray]:
    """解码成小灰度图算帧间绝对差。比包大小准，代价是要解码整段。"""
    proc = subprocess.run(
        ['ffmpeg', '-nostdin', '-v', 'error', '-i', str(mkv_path),
         '-vf', f'scale={width}:-2', '-pix_fmt', 'gray',
         '-f', 'rawvideo', 'pipe:1'],
        capture_output=True, check=True)
    probe = subprocess.run(
        ['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries',
         'stream=width,height', '-of', 'csv=p=0', str(mkv_path)],
        capture_output=True, text=True, check=True).stdout.strip().split(',')
    w0, h0 = int(probe[0]), int(probe[1])
    h = int(round(h0 * width / w0 / 2)) * 2
    buf = np.frombuffer(proc.stdout, dtype=np.uint8)
    n = buf.size // (width * h)
    frames = buf[:n * width * h].reshape(n, h, width).astype(np.float32)
    motion = np.abs(np.diff(frames, axis=0)).mean(axis=(1, 2))
    return np.arange(1, n, dtype=np.float64), motion


def arm_speed(session, side: str) -> tuple[np.ndarray, np.ndarray]:
    """该臂 7 个关节角速度的模。相机随手腕转，图像运动主要由它驱动。

    **必须用 `joint_states` 的实测速度，不能用 `motion_control_status` 的 `limited_pose`。**
    后者是关节指令的正解，机器人跟随指令有滞后，拿它做互相关会把伺服跟踪滞后一起算进
    延迟里 —— 实测系统性偏大 30~45 ms（+142 vs +101、+145 vs +114、+243 vs +195）。
    """
    t, data = session.table('joint_states')
    cols = session.columns('joint_states')
    wanted = [f'vel.{j}' for j in ARM_JOINTS[side]]
    idx = [cols.index(c) for c in wanted if c in cols]
    if not idx:
        raise KeyError(f'joint_states 里没有 {side} 臂的速度列')
    return t, np.linalg.norm(np.nan_to_num(data[:, idx]), axis=1)


def _resample(t: np.ndarray, v: np.ndarray, grid: np.ndarray) -> np.ndarray:
    if t.size < 2:
        return np.zeros_like(grid)
    # np.interp 要求 x 递增，乱序会静默插出错值。实测每张表有 1~2 行倒序
    # （多线程执行器下 header 戳可能乱序），当前量级不影响结果，但不能指望它一直这么少。
    if np.any(np.diff(t) < 0):
        order = np.argsort(t, kind='stable')
        t, v = t[order], v[order]
    return np.interp(grid, t, v, left=np.nan, right=np.nan)


def _whiten(x: np.ndarray) -> np.ndarray:
    """去掉直流与慢漂，只留下运动的起伏 —— 互相关要对齐的是形状不是电平。"""
    x = np.nan_to_num(x, nan=0.0)
    if x.size >= 21:
        k = np.ones(21) / 21.0
        x = x - np.convolve(x, k, mode='same')
    x = x - x.mean()
    s = x.std()
    return x / s if s > 0 else x


def estimate_delay(video_t: np.ndarray, video_motion: np.ndarray,
                   sig_t: np.ndarray, sig_motion: np.ndarray,
                   max_lag_s: float = MAX_LAG_S,
                   grid_hz: float = GRID_HZ) -> Alignment:
    """互相关求「视频比机器人状态晚多少」。"""
    lo = max(float(video_t[0]), float(sig_t[0]))
    hi = min(float(video_t[-1]), float(sig_t[-1]))
    if hi - lo < 2.0:
        return Alignment(0.0, 0.0, 0.0, 0)
    grid = np.arange(lo, hi, 1.0 / grid_hz)
    a = _whiten(_resample(video_t, video_motion, grid))
    b = _whiten(_resample(sig_t, sig_motion, grid))
    if a.size < 20:
        return Alignment(0.0, 0.0, 0.0, int(a.size))

    max_lag = int(max_lag_s * grid_hz)
    lags = np.arange(-max_lag, max_lag + 1)
    corr = np.array([np.dot(np.roll(a, -int(k)), b) for k in lags]) / a.size
    peak = int(np.argmax(corr))
    score = float(corr[peak])

    # 抛物线插值取亚采样峰位，否则分辨率被网格锁死在 20 ms
    delay = lags[peak] / grid_hz
    if 0 < peak < corr.size - 1:
        y0, y1, y2 = corr[peak - 1], corr[peak], corr[peak + 1]
        denom = y0 - 2 * y1 + y2
        if denom != 0:
            delay += (0.5 * (y0 - y2) / denom) / grid_hz

    guard = max(3, int(0.05 * grid_hz))
    side = np.concatenate([corr[:max(peak - guard, 0)], corr[peak + guard:]])
    margin = score - float(side.max()) if side.size else 0.0
    return Alignment(delay, score, margin, int(a.size))


def align_stream(session, name: str, window_s: float = 30.0,
                 use_frames: bool = False) -> dict:
    """对一路腕部视频做分窗对齐，返回 delay(t) 曲线与整段估计。

    分窗是为了跟踪慢变的偏移 —— 实测 90 s 内 t0 就能漂 138 ms。
    """
    side = WRIST_SIDE.get(name)
    if side is None:
        raise ValueError(f'{name} 不是腕部视频')
    other = 'right' if side == 'left' else 'left'
    pts = session.video_pts(name, corrected=False)   # 标定要量的就是这个修正量本身
    idx, motion = (motion_from_frames(session.video_path(name)) if use_frames
                   else motion_from_packets(session.video_path(name)))
    if idx.size == 0:
        return {'stream': name, 'error': '视频里没有包'}
    keep = idx.astype(int) < pts.size
    video_t, video_motion = pts[idx.astype(int)[keep]], motion[keep]
    sig_t, sig_motion = arm_speed(session, side)

    overall = estimate_delay(video_t, video_motion, sig_t, sig_motion)
    # 双臂同时动时，画面里两条臂的运动混在一起，光看峰高会把串扰当成信号。
    # 实测 wrist_right 同侧 0.55 / 异侧 0.16（可信），wrist_left 同侧 0.23 / 异侧 0.22
    # （不可信，两者分不开）。所以同侧必须明显赢过异侧才算标出来了。
    cross = estimate_delay(video_t, video_motion, *arm_speed(session, other))
    windows = []
    lo, hi = float(video_t[0]), float(video_t[-1])
    n = max(1, int((hi - lo) // window_s))
    for k in range(n):
        a, b = lo + k * window_s, lo + (k + 1) * window_s
        m = (video_t >= a) & (video_t < b)
        s = (sig_t >= a) & (sig_t < b)
        if m.sum() < 30 or s.sum() < 30:
            continue
        est = estimate_delay(video_t[m], video_motion[m], sig_t[s], sig_motion[s])
        windows.append({'t0': round(a, 3), **est.as_dict()})

    good = [w['delay_ms'] for w in windows if w['trustworthy']]
    side_specific = overall.score >= cross.score * 2 and overall.trustworthy
    return {
        'stream': name, 'side': side,
        'source': 'frames' if use_frames else 'packets',
        'overall': overall.as_dict(),
        'cross': cross.as_dict(),
        'side_specific': side_specific,
        'windows': windows,
        'trustworthy_windows': len(good),
        'spread_ms': round(max(good) - min(good), 1) if len(good) > 1 else 0.0,
    }


def corrected_pts(session, name: str, delay_s: float) -> np.ndarray:
    """把估出来的延迟从时间戳里减掉，得到采集时刻。"""
    return session.video_pts(name) - float(delay_s)
