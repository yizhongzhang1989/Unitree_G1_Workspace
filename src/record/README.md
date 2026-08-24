# record — 上肢 VR 遥操作数据采集

三路视频 + 全量信号 + 随机指令生成 + 采集面板。采集端只落原始数据，格式转换与上云
全部在导出机上做。

---

## 架构

```mermaid
flowchart TB
  subgraph CAM["相机"]
    WL["腕相机 左<br/>192.168.123.97"]
    WR["腕相机 右<br/>192.168.123.98"]
    HD["D435i<br/>USB3"]
  end

  subgraph A["A · 机器人 Orin NX"]
    RS["realsense2_camera<br/>建议 color_format:=YUYV"]
    MC["motion_control<br/>50 Hz IK"]
    VR["vr_teleop<br/>头显遥操作"]
    HW["all_data.launch.py<br/>硬件栈 + broadcasters"]

    subgraph REC["record 节点"]
      direction TB
      VID["video.py<br/>腕 -c copy / 头 libx264"]
      SIG["signals.py<br/>话题 → 定宽 float64 表"]
      SES["session.py<br/>三层状态机 + 事件线 + 封口"]
      INS["instruction/<br/>物品库 → 布局 → 摆放样例 → 指令"]
      DASH["dashboard.py :8220"]
    end
    DISK[("~/.ros/record/sessions/")]
  end

  subgraph B["B · 导出机 Windows，无 ROS"]
    BROW["浏览器<br/>控制 + 监看"]
    TOOLS["tools/<br/>纯 Python + numpy"]
    CLOUD[("转格式 → 上云")]
  end

  WL & WR -->|"RTSP H.264 1080p"| VID
  HD --> RS -->|"/head/.../image_raw"| VID
  HW & MC --> SIG
  VR --> MC
  VID & SIG & INS --> SES --> DISK
  DASH -.->|"只读状态 + 发命令"| SES
  BROW <-->|"HTTP 1 Hz 轮询"| DASH
  DISK -->|"rsync / WinSCP"| TOOLS --> CLOUD
```

**架构铁律：录制逻辑全在 ROS 节点里，面板只是观察者 + 命令入口。** 没人开页面、HTTP
线程崩了，录制照常。

### 三层时间粒度

```
session   一次连续录制。视频与信号表全程连续写
 └ round  一次摆桌。一组物品 + 一张摆放样例 + 一串有状态依赖的指令
    └ episode  一个原子动作（一个 verb）。这是交付单位
```

**episode 不切文件**，只是 `events.jsonl` 里的一对时间戳。每条 episode 重开 ffmpeg
会让每段开头 1.2 s 全废（RTSP 冷启动实测值）。

交付粒度的依据：上游 `pp_g1` 的 skeleton 计数逐项相加恰好等于 episode 数
（`807+188+181+218+230+75+34+4 = 1737`，`pp_g1_v2` 同样对得上 1986），
所以对面口径是「一条 episode = 一个 verb」。

---

## 快速开始

```bash
# 1. 硬件栈与控制栈
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual
ros2 launch g1_motion_control motion_control.launch.py
ros2 launch g1_motion_control vr_teleop.launch.py

# 2. 头部相机。YUYV 直出省掉一次色彩转换，720p 编码从 98% 降到 60% 单核
ros2 launch head_sensors head_camera.launch.py \
    color_profile:=1280x720x30 color_format:=YUYV

# 3. 采集节点 + 面板 + 数据管理
ros2 launch record record.launch.py
```

采集面板 `http://<机器人 IP>:8220`，数据管理 `:8221`。只想要采集就 `data:=false`。

两个节点**分开进程但一起启动**：采集必须最简最稳，数据管理要读全盘、解码取帧、删目录，
负载画像完全不同。但操作者不该记两条命令。互斥靠 `DONE` 天然完成 —— 没封口的
 session 删不掉也放不了。

### 面板上的四件事

1. **数据一览** — 每路的话题、类型、实时频率、已收/已写。开录前先确认没有哑掉的流。
   帧计数只能发现「流哑了」，发现不了「相机对着墙」—— 那得点「抓一帧看看」。
   **只在未录制时可用、只抓一帧**：实测连续解码是 65% 单核每路，三路就是两个核，
   而单帧只要 1.5 s（腕部，RTSP 握手主导）或 0.25 s（头部，帧就在内存里）
2. **勾选要录哪些** — 默认全勾**除深度图**；点「开始 session」后整块置灰，
   勾选结果冻结进 `manifest.json`
3. **生成新一轮摆放** — 出一张摆放样例图，照着把真实桌面摆好
4. **逐条执行 episode** — 开始 → 操作 → 标注成功/失败/丢弃

结束时点「结束 session」，会收尾所有文件、算全目录 sha256、写 `DONE`。
同步脚本只搬带 `DONE` 的目录。

---

## 落盘格式

```
~/.ros/record/sessions/<时间戳>/
  manifest.json      开录瞬间冻结的流勾选 + 时间基准
  meta.json          camera_info、桌面几何、关节顺序、备注
  schema.json        每张 .bin 的列名/列数/dtype
  events.jsonl       唯一的时间线，只追加
  video/
    wrist_left.mkv       H.264 1080p 原码流，-c copy
    wrist_left.pts.bin   每帧一个 float64（收包墙钟）
    wrist_right.* / head.*  signals/
    joint_states.bin     [t_recv, t_header, 31×pos, 31×vel, 31×eff]
    motion_control_command.bin  [t_recv, t_header, 20]
    ...
  rounds/
    round_000.json     物品 + 布局 + seed + 全部 episode 的结构化指令
    round_000.svg      摆放样例
  DONE               全目录 sha256
```

**信号表是无表头的裸 float64 矩阵**，列信息全在 `schema.json`。采集机是 Orin NX、
导出机是没有 ROS 的 Windows，两边唯一都有的只有 numpy，所以读一张表必须能退化成：

```python
np.fromfile(p, np.float64).reshape(-1, ncol)
```

带表头的格式（parquet / hdf5）都要装库，而这台机器连 pyarrow 都没有。

### 指令为什么不只存一个字符串

`instruction_en` 是对面要的契约字段，但每条 episode 同时存结构化的一坨：

```
scene / round / instruction_id / step_index / step_total
verb / obj{id,en,zh} / target{id,en,zh} / prep / arm / arm_to
instruction_en / instruction_zh / lint_warnings[] / outcome
```

字符串是给模型看的，结构化字段是给自己看的 —— 后者决定了以后能不能筛数据、按动作
切分、改名、查错。方向不可逆：结构化 → 字符串是纯函数，反过来有损。

---

## 导出机侧

把 `tools/` 整个拷过去，只要 Python + numpy（视频对齐额外需要 ffprobe）：

```bash
python inspect_session.py <session 目录>            # 概览
python inspect_session.py <session 目录> --verify   # 逐文件核对 sha256
python inspect_session.py <session 目录> --align    # 估视频时间偏移
```

```python
from session_reader import Session
s = Session(r'D:\data\20260821_101500')

t, q = s.table('joint_states')                # 时间列 + 数据列
for ep in s.episodes():                       # 默认不含 discard
    print(ep['instruction_en'], ep['outcome'])
    t, q = s.slice_table('joint_states', ep['t0'], ep['t1'])
    frames = s.slice_frames('head', ep['t0'], ep['t1'])
```

`tools/` 里**禁止 import rclpy**，有测试用正则守着这条。

---

## 时间对齐

所有数据落在同一个 `CLOCK_REALTIME` 上。**每一行、每一帧都有可用时间戳**（实测两个
session 全部 12 张表 + 3 路视频，无效时间戳 0 行）。

信号表每行同时存 `t_recv` 与 `t_header`，读的时候优先用 `t_header`：

| 时间列 | 哪些表 | 语义 |
|---|---|---|
| `header`（源端打戳） | `joint_states`、`pelvis_imu`、四路 `wrench` | 数据产生的时刻，不含传输抖动 |
| `recv`（接收时刻） | `motion_control_command/status`、`secondary_imu` | 消息类型没有 header 字段。**对指令话题这本来就是正确语义** —— 指令是收到那一刻才生效的，不存在更早的「采集时刻」 |

区分这两者只是为了知道对齐精度：源端戳不含传输抖动，接收戳含。实测有 header 的表
`t_recv − t_header` 的 p50 在 4.6~50.1 ms，抖动量级就是这么大。
`inspect_session.py` 会如实标出每张表用的是哪种。

头部相机用 RealSense 的时间戳，**只有腕部需要额外处理**——它是网络设备，没有可用时钟。

腕部三步：

1. **收包打戳** — `-use_wallclock_as_timestamps 1 -copyts`，每包到达时打主机墙钟
2. **等间隔重建** — `fitted_pts()` 丢掉到达时刻，换成一条直线。依据是 `-c copy` 下
   包数严格等于帧数、帧率由硬件恒定。RTSP over TCP 成簇到达，逐帧时刻不可用，
   但「第 i 帧采于 t0 + i/fps」可用
3. **按时间取帧** — `slice_frames` 与 `slice_table` 用同一对时间戳

**`video_pts()` 默认已经把管线延迟减掉了**，所以 `slice_frames` 取到的帧与信号表
天然对齐，导出不需要再补。修正只在读的时候做，**落盘文件永远是原始值** —— 标定值
已经改过两回，烧进文件就没得救。重新标定时用 `video_pts(name, corrected=False)`。

### `pts.bin` 是什么，为什么腕部那份看着像重复

每帧一个 float64 = 8 字节，1335 帧才 10 KB，是 mkv 的 **0.064%**。

腕部那份**确实**与 mkv 内置的 pts 逐帧相同（实测差 0.000 ms）—— `-copyts` 让 mkv
自己也带上了绝对墙钟。留着它是因为两件事：读它只要 numpy（导出机不必装 ffprobe），
以及 **mkv 没封口时它仍然完好**（进程被硬杀过一次，靠的就是它）。

头部那份**不重复**：head.mkv 里是从 0 起算的相对时间，绝对采集时刻只存在 `pts.bin`
里（RealSense 的硬件时间戳）。

---

## 数据管理与回放

`:8221` 那个面板。左边列出历次采集（大小、episode 数、告警数、是否封口），点开看到
每段的**预览帧**与指令，可以回放，也可以删除。

- **预览** — 每条 episode 取中点那一帧（开头往往手还没进画面）。**按需取单帧，不做
  连续解码**：从已录文件取一帧含 seek 只要 0.21~0.33 s，而连续解码是 65% 单核每路
- **删除** — 要把 session id 原样打一遍确认。**没有 `DONE` 的删不掉**，那可能正在录
  或异常中断
- **回放** — 见下



**为什么不按关节回放**：`motion_control` 一直开着并占着 FPC，关节级指令的出口只有它
一个。想按关节放就得让它退出 `ik` 模式，那会改变整条链路的行为。重发末端位姿走的是
和当时完全相同的那条路（同一套 IK、同一个出口限速），复现度更高，也不用改控制层。

**只发上肢**：长度 14（双臂位姿）+ 长度 2（双夹爪）。**绝不发长度 4 或 20** —— 那会
带上 `vx/vy/wz/h` 让机器人走路。录的是桌面操作，回放时它该站着不动。

三道安全：

1. `arm_mode` 必须是 `ik`。透传模式下那 7 个数会被当成关节角，**把四元数当关节角发
   下去是一次事故**，不是显示问题
2. `arms_live` 必须为真，否则上肢还没被接管
3. 开播前从当前 `limited_pose` **缓入**到录制起点（余弦缓入，两端速度为零）。机器人
   现在在哪和录制从哪开始可能差很远，直接发第一帧会被出口限速摊成一段全速运动

停止就是停止发布：上肢指令不设超时，手臂停在原地。要卸力用控制层的 `~/estop`。

> 姿态插值用 slerp 不用线性插值，且点积为负时取反走短弧 —— 缓入段可能有大转角，
> 线性插值再归一化会让角速度不均匀，取长弧则会绕一大圈。

| 误差来源 | 量级 | 状态 |
|---|---|---|
| 网络排队 + 内核缓冲 + ffmpeg 调度 | 单帧 p5..p95 跨 23 ms | 第 2 步已消除，残余 ±10 ms |
| 相机管线（曝光 + ISP + 编码） | 两路都 **110 ms ± 15** | 已标定，见下 |

第二项让视频时间轴整体偏晚一个固定量。`tools/align_video.py` 用图像运动强度 ×
该臂关节角速度做互相关来标它。参数统一后两台相机的延迟趋于一致：

| 采集 | 相机配置 | `wrist_left` | `wrist_right` |
|---|---|---|---|
| 2026-08-24 早 | `.97` 30fps/GOP120，`.98` **25fps**/GOP75 | +114 ms（峰高 0.60） | +195 ms（峰高 0.65） |
| 2026-08-24 晚 | 两台统一 30fps/GOP90 | +101 ms（峰高 0.17） | +115 ms（峰高 0.24） |

**取 110 ms，不确定度 ±15 ms。** 后一次的峰高没到 0.3 的判据线，但几条互相独立的
证据都指向同一个数：

- 同一段数据换 4 种机器人侧信号（全 7 轴 / 腕 3 轴 / 肘+腕 / 末端），延迟都落在 98~116 ms
- 换窗长 4/12/16 s 重算，左路 +100.3 / +103.9 / +101.4，右路 +101.3 / +109.8 / +115.1
- 参数统一前 `.97` 在高置信度下测得 +114 ms，与本次 +101 一致
- **改帧率前就预测过** `.98` 的 +195 会向 `.97` 靠拢（那 81 ms 差 ≈ 25 fps 的两个帧
  周期），改完实测 +115，预测应验

不修正就是 110 ms 的系统性错位，修正后残余 ±15 ms，所以带着不确定度用。想收紧就再
采一次单臂数据。

**机器人侧信号必须用 `joint_states` 的实测速度**，不能用 `motion_control_status` 的
`limited_pose`：后者是关节指令的正解，机器人跟随指令有滞后，拿它做互相关会把伺服
跟踪滞后一起算进延迟里 —— 实测系统性偏大 30~45 ms。

> **两台相机必须保持一致。** 用 `scripts/set_wrist_camera_fps.py --diff` 自检，
> 不一致时退出码非零，可直接当检查项。出厂就不一样，不自检发现不了。
> **改过相机参数就要重标**：延迟直接跟编码参数相关。

> **标定采集要单臂动。** 一次只动一条臂、幅度大、方向多变，每条 10 s 就够（实测
> 单臂 10 s 的置信 0.60，而双臂齐动 104 s 只有 0.23）。判据是**同侧至少是异侧的两倍**：
> 双臂齐动时两条臂的运动混在一起，光看峰高会把串扰当成信号。
>
> 相机是**自动曝光且上限 100 ms**（`MaxExposureTime`），曝光时间本身就是延迟的一部分，
> 所以标定和采集的光照条件应当接近。

包大小那路运动信号信噪比不够（实测置信只有 0.10，帧差是 0.56~0.62），标定一律加
`--align-frames` 走解码帧差。它连「哪台相机在动」都分不出来（两台包大小差异被码率
基线淹没），帧差则是 12~21 vs 4~9，一目了然。

> **RTCP 这条路不通。** RTP 标准的 Sender Report 会把媒体时钟映射到发送端 NTP 墙钟，
> 有它就不需要互相关。实测这两台相机不实现：抓 30 秒收到 4070 个 RTP 包、SR 一个没有，
> 主动发接收报告催也没用。换成支持 RTCP SR + NTP 的相机可以让整个问题消失。

---

## 桌面几何

桌面尺寸**不能照抄对面 A2D**：G1 肩→夹爪 0.508 m vs A2D 0.914（1.80 倍），
肩间距 0.200 vs 0.426（2.13 倍），两个比值不同，不存在单一缩放系数。

用 pinocchio 逐格 IK + 邻格热启动洪水填充实测（腰锁死、工具轴垂直向下 30° 内、
含自碰撞过滤）：

```
可达区外接矩形  275 mm(纵深) x 900 mm(横向)   其中 80% 可达
桌高            0.80 m（离地）时面积最大 0.198 m²，约为对面的 1/4
近边            x = 0.100（torso_link 系）—— 经验值，实机要拿尺子核
```

掩码固化在 `record/data/reach_mask.npz`。**布局与落点一律对掩码判**——用矩形近似会
得出「5 件承载面全放不下」的假结论，对掩码判则 220 mm 的盘子还有 5 个可放位置。

物品库 91 件 active，可达域内可用 83 件。一个 round 建议 **3–5 件**（对面是 5–10 件）。

---

## 为什么这样选（实测依据）

| 决策 | 依据 |
|---|---|
| 腕部 `-c copy` 录 1080p | 1.33% 单核/路；解码转 720p 是 103.8%，**贵 78 倍且画质更差**。720p 留到导出机离线降 |
| 时间戳那路也要 `-c copy` | 漏了它 ffmpeg 走默认编码器，把 1080p 整个解码一遍：1.0% → 25.1% 单核 |
| 头部走 ROS 不走 v4l2 | 贵约 1.5 倍，换 `header.stamp`（p50=p95=33.4 ms）与 `camera_info` |
| 头部用 YUYV | 编码 98.0% → 60.3% 单核，且 RGB8 会丢帧、YUYV 不会 |
| `-crf 26` | ultrafast 默认码率虚高 14.99 Mbps，加 crf 后 6.18 且 CPU 还少 15 个点 |
| 不录 `/tf` | 665 列/条、96 KB/s，且能从 `joint_states` + URDF 离线重算 |
| 不录 `/lowstate` | 1046 Hz、6.8 GB/h，而 `joint_states` 是它的重打包 |
| 关节按 `msg.name` 重排 | `joint_state_broadcaster` 的 `joints` 是空数组，顺序每次启动都可能变 |

容量约 **6.5 GB/h**，1.8 T 可录 250 小时以上。

---

## 已知限制

- **头部相机的管线延迟未标定**（约 30 ms 是估计值）。头相机不随手臂动，互相关这条路
  对它不适用；要标得靠 D435i 的时间戳语义或外部闪光
- **腕部延迟的不确定度还有 ±15 ms**。要收紧就再采一次单臂数据
- **两台腕相机经常轮流掉线**。参数已统一（1080p / 30 fps / 3000 kbps / GOP 90 /
  H264 High），用 `scripts/set_wrist_camera_fps.py --diff` 自检
- **回放没有视频同步画面**。只重演动作，预览帧是静态的；要逐帧看得自己开播放器
- **没有导出/打包功能**。数据管理只能浏览、预览、删除、回放，搬运还是 rsync
- **A→B 自动同步没接**。`DONE` 已经写了，rsync/WinSCP 还要人工触发
- **导出脚本没写**，等对面给格式口径。中间格式设计成零依赖可读，届时只加导出脚本
- 头部深度图默认不录：16UC1 压不了，424x240@30 就是 21 GB/h，比三路彩色加起来还大

---

## 测试

```bash
python3 -m pytest src/record/test -q          # 124 条，约 55 s
python3 src/record/test/smoke_record.py       # 真机烟测，不接相机也能跑
```

其中几条是钉契约的：只有 numpy 也能读出落盘数据、录制中 `kill -9` 后已落盘部分仍是
整行的整数倍、关节必须按名字重排、以及 Instruction Spec §1.5 要求的发布前批量验收
（120 个 round、1000+ 条指令、lint 命中必须为 0）。
