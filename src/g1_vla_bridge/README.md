# g1_vla_bridge

VLA 推理服务与 `g1_motion_control` 之间的桥。**流程是固定的，VLA 是可换的**：

```
采观测 → backend.infer() → 重锚 → 可选逐帧限幅 → /motion_control/command
```

节点只认三样东西，全部定义在 [vla_backend.py](g1_vla_bridge/vla_backend.py)：

| | 是什么 | 表示在哪个系 |
|---|---|---|
| `Observation` | 图像 + 双臂末端位姿 + 夹爪 + 各视角相机内外参 + 任务指令 | `base_frame`（默认 `torso_link`） |
| `ActionChunk` | N 个 waypoint 的末端位姿 + 夹爪 | `base_frame` |
| `VlaSpec` | 这家 VLA 的**规格**：坐标系原点在哪、要几张什么图、夹爪怎么换算 | — |

**模型系的换算不在节点里做**，由 backend 拿 `spec.frame` 完成，所以 [vla_node.py](g1_vla_bridge/vla_node.py)
里没有任何一家 VLA 的协议细节，换 VLA 不会把坐标系的坑扩散到执行侧。

```mermaid
flowchart LR
    subgraph obs["观测（base_frame）"]
        head["/head/camera/color/image_raw"]
        wl["左腕 RTSP + 收包 PTS"]
        wr["右腕 RTSP + 收包 PTS"]
        tf["实测 joint_states + URDF FK<br/>头部历史 TF"]
    end

    subgraph node["vla_node（与 VLA 无关）"]
        worker["推理线程<br/>按 chunk 请求"]
        buffer[("动作缓冲")]
        timer["下发定时器<br/>execution_rate_hz，可选限幅"]
    end

    subgraph be["backends/&lt;名字&gt;.py + config/backends/&lt;名字&gt;.yaml"]
        spec["VlaSpec<br/>坐标系 / 图像 / 夹爪"]
        wire["请求封装<br/>编码 · POST · 解析"]
    end

    obs --> worker
    worker -- Observation --> be
    be -- ActionChunk --> worker
    wire <--> vla["VLA 服务"]
    worker -- 完整一段 --> buffer
    buffer --> timer
    timer --> mc["/motion_control/command<br/>14 双臂位姿 + 2 夹爪"]
```

推理和下发是两条线程（一轮推理几百毫秒且抖动大，放回调里会堵死执行器）。默认是手动模式：
每次请求得到一个完整 chunk，下发定时器按 `execution_rate_hz` 从第 0 点播到最后一点，中途绝不
替换；播完停在最后一点，等待下一次请求。`execution_mode:=continuous` 则播完后自动请求下一段。
默认 `cartesian_limit_enabled: false`，VLA 不裁剪位置和姿态，按下发频率原样下发 waypoint；
motion_control 的关节限速和 IK 保护仍保留。设置该参数为 `true` 可恢复 VLA 限速。
开启时，最后一点若被限速截断，会继续下发到指令达到末点后才结束；这不代表实机反馈已经到位。
任何一轮推理失败都只是这一轮作废，手臂保持当前目标；manual 模式等待下一次 Enter，
continuous 模式在 `retry_delay_s` 后自动重试。`async` 模式见下方。

### 异步执行

顿挫与振幅增长的检查结果见 [Async 审查](ASYNC_AUDIT.md)。时间对齐和 EMA 不保证闭环
稳定；长延迟下可能只执行预测尾部、随后断供，当前尚未验证真机闭环稳定性。

`execution_mode:=async` 让推理与动作执行并行，始终最多一个 HTTP 推理在途。
每次响应合并进未来动作队列后，立即采最新观测并请求下一段，不等队列播完。
位置按 `(1-alpha)*旧预测 + alpha*新预测` 更新，旋转使用 SLERP；夹爪直接采用新预测，
不平均开闭决策。非重叠部分直接追加。

时间约定：第 0 个动作对应选中观测的获取时刻，不是 HTTP 请求开始或返回时刻。
取下面公共观测时间格的时刻 T，按同次采样的 ROS 时钟与单调时钟
映射为 `acquired_monotonic`；第 k 个动作对应 `T + k/action_rate_hz`。
因此已计入修正后的观测年龄、本轮观测处理、编码、网络及推理耗时。
`action_rate_hz` 必须与训练动作时间间隔匹配，
不是模型自报频率；当前按 30 Hz 解释。非整数拍的请求起点通过位置插值与旋转 SLERP
映射到统一执行网格，夹爪取前一采样值。迟到的控制 tick 不补播历史动作。

例如 0 s 请求 30 个动作、0.2 s 返回，只保留 24 个未来动作并立即请求下一段；
0.4 s 再次返回时，18 个未来重叠动作融合、6 个更远期动作追加，已执行动作不修改。
以上假设图像在请求时刻获取；若图像为 0 s 获取、0.1 s 请求、0.3 s 返回，则丢弃
前 9 个动作，只保留 21 个，不能只按 0.2 s 推理时间丢 6 个。

### 与 record 的观测对齐

三个执行模式使用同一个观测入口：

- 腕图直接通过 PyAV/libavformat 读取 RTSP，使用与 record 相同的 TCP、low_delay、
  `use_wallclock_as_timestamps=1`。保留解码帧的绝对 PTS，不用读出帧时的当前时间；
  缺 PTS 的启动帧跳过。默认 stream0 原分辨率，不额外限帧或缩放。
- 复用 record 的 `fitted_pts` 和 `CAMERA_DELAY_S`：腕图拟合后减 110 ms，
  头图保留 RealSense header。在线拟合用最近最多 900 个已到达的帧，图像仅缓存 32 帧；
  解码持续取流，推理只转换选中的帧。断流清空对应缓存并重连。
- `observation_rate_hz=30`，从首个可用公共时刻建立固定时间格。
  T 不晚于任何输入的最新时刻，也不晚于头部 TF 的最新可用时刻。
  图像和 `/joint_states` 均选最后一条时间戳不大于 T 的记录，不插值。
  `observation_sample_max_age_s=0.1` 与导出默认值一致，限制每条记录相对 T 的年龄。
- 双臂末端与腕相机外参复用 record FK，由**同一条实测关节记录**计算；
  夹爪也来自这条记录的 eccentric 轴，不再使用指令值。
  模型读取实际 `/robot_description`，只解析顶层运动学关节，保留运行中的标定。
  腕内参从 `observation_calibration_file` 按真实分辨率精确匹配，默认是已安装的
  camera_calibration/config/calibration.yaml，不使用别的档位缩放代替。

`observation_max_age_s=0.5` 限制公共观测到当前的总年龄；解码处理后再次检查。
缺关节、缺图、过期或缺历史 TF 均拒绝本轮，async 仅继续已有有效预测，断供超时停止。
状态的 `observation_sample_age_s` 是选中图像/关节各自的修正年龄，
`observation_skew_s` 是它们的时间差，`observation_age_s` 是公共观测年龄；
`wrist_stream_errors` 显示直连流是否异常。不支持 ROS 仿真时钟或运行中的时钟跳变。

**一致性的边界：** 离线导出使用整段视频拟合，在线无法使用未来帧；公式和延迟补偿相同，
但拟合窗口与时间格起点不同，不能承诺逐帧编号完全相同。110 ms 是既有标定而非本次重测，
编码设置变化须重新标定。当前新增的可动头部仅发布 TF、不发布 JointState，record 导出仍
将头部当静态链；在线头部因此使用 T 时刻的历史 TF。动态头部训练/导出尚未闭合，
本改动不宣称与旧静态头部数据完全一致，也没有验证真实模型的闭环效果。

依赖安装与只读预检（不调用模型、不发运动命令）：

```bash
python3 -m pip install --user -r src/g1_vla_bridge/requirements.txt
colcon build --packages-select g1_vla_bridge --symlink-install
source install/setup.bash
python3 src/g1_vla_bridge/test/observation_preflight.py
```

腕流地址由 `left_wrist_rtsp_url`、`right_wrist_rtsp_url` 配置；无需给 VLA 额外发布腕部
ROS 裸图。原相机预览与标定节点不受修改，但同时开多路解码仍有 CPU 成本。

### 异步参数与切换

`execution_rate_hz` 单独控制目标下发定时器，当前本地配置暂设为 **10 Hz**；
`action_rate_hz` 保持 **30 Hz**，仍表示模型预测点的训练时间间隔。
async 在每个 100 ms tick 取对应的预测点，跳过期间的旧点，不把一秒预测拉长为三秒。
manual/continuous 仍逐点播放，因此在 10 Hz 下 30 点需要约三秒。
降低下发频率不是限速或平滑，单次目标变化可能更大，不能保证减轻跳动。
此参数在启动时读取，修改后需要停止并重新启动 VLA 节点；不要同时启动两个桥。
保留原有 proxy、server_url 等启动参数，另加 `execution_rate_hz:=10.0`；
恢复原频率用 `execution_rate_hz:=30.0`。状态中会显示实际 `execution_rate_hz`。

参数 `async_ema_alpha` 默认 0.5，范围 `(0, 1]`，1 表示直接采用新预测。
`async_hold_timeout_s` 默认 1 秒：队列耗尽后保持最后目标，超过此时间停止 VLA 下发，
首次启动则从启动时刻计算等待超时。全部过期的响应不会延长等待期限。
停止不等于卸力；卸力仍需 `/estop`。状态提供 `async_pending`、`async_buffer_s`、
`async_ema_alpha` 和 `async_hold_timeout_s`，超时原因保留在 `error`。

async 仅支持绝对位姿，要求 `delta_position=false`、`delta_rotation=false`、
`skip_intermediate_waypoints=false`。`action_horizon` 仍限制每次预测使用的前缀长度。
EMA 不保证轨迹可达或避障，也不能证明跳过的动作已经物理完成；限速仍遵循原有配置。

```bash
ros2 launch g1_vla_bridge vla_bridge.launch.py execution_mode:=async \
  async_ema_alpha:=0.5 async_hold_timeout_s:=1.0
ros2 run g1_vla_bridge vla_cli
```

CLI 用 `/stop`、`/mode async`、`/start` 切入；`/mode manual` 和 `/mode continuous`
恢复原两种执行方式。`/mode` 只选择模式，不自动启动。所有模式变更都要求先停止，
并清空队列、使旧请求失效。服务接口为 `~/set_async`（SetBool，true=async，false=manual）
和 `~/set_auto`（SetBool，true=continuous，false=manual）。CLI 退出不影响节点调度。

## 接一个新的 VLA

加两个同名文件，把 `vla_backend` 指过去。**`vla_node.py` 一行都不用改。**

```
g1_vla_bridge/backends/<名字>.py     # SPEC + PARAMETERS + create(params)
config/backends/<名字>.yaml          # 参数值，launch 按 vla_backend 自动挂上
```

配置分层，两边的键不许重叠（[test_config_layout.py](test/test_config_layout.py) 机械核对）：

| | 装什么 |
|---|---|
| 代码里的 `PARAMETERS` | 默认值 |
| `config/vla_bridge.yaml` | 与 VLA 无关：话题、坐标系名、下发速率、限幅 |
| `config/backends/<名字>.yaml` | 这家的：服务地址、坐标系标定、图像预处理 |
| launch 的 arg | 现场临时改的那几个 |

```python
SPEC = VlaSpec(
    name='<名字>',
    frame=FrameSpec(
        origin_in_base=(...),        # VLA 坐标系原点落在 base_frame 的哪里
        rotation_rpy=(...),          # VLA 坐标系相对 base_frame 的朝向
        tool_offset=(...),           # 我方 tip frame -> VLA 末端 frame
        tool_rotation_rpy=(...)),
    images=ImageSpec(slots=('head', 'left_wrist', 'right_wrist'), height=240),
    gripper=GripperSpec(model_open=0.0, model_closed=1.0,             # VLA 侧
                        robot_open_rad=2.76377, robot_closed_rad=0.0),  # 我方关节
    horizon=30,
    action_semantics='absolute')     # 'absolute' 才允许开 delta 重锚

PARAMETERS = {...}          # 要节点替它 declare 的 ROS 参数及默认值
def create(params): ...     # -> VlaBackend 子类，实现 infer(Observation) -> ActionChunk
```

**接入清单**——下面这些必须逐条问清楚，猜不得：

| 要问的 | 落到哪 | 猜错的后果 |
|---|---|---|
| **动作/state 在哪个系，原点在机器人的什么位置** | `frame.origin_in_base` | 绝对模式下整段偏掉 |
| 那个系相对地面是不是水平的、朝向如何 | `frame.rotation_rpy` | **delta 也救不了**，「往前」会走成别的方向 |
| 末端参考点是法兰还是夹爪、姿态轴怎么定 | `frame.tool_*` | 姿态整个反过来 |
| 要几张图、顺序、分辨率、预处理 | `ImageSpec` + backend 的编码 | 模型不报错，只是变傻 |
| 训练相机的内参/畸变/分辨率 | backend 常量（重投影用） | 同一物体尺度对不上 |
| 夹爪的取值范围与方向 | `GripperSpec` | 该松手时夹紧 |
| 输出是绝对位姿还是增量、N 是多少 | `action_semantics` / `horizon` | 重锚逻辑用错 |
| 训练 episode 里真实的相机外参 4×4 | 喂给 `calibrate_frame` | 见下面「灵敏度」 |

一致性由 [test_vla_backend.py](test/test_vla_backend.py) 兜底，接新 VLA 先跑它。

## 运行

前置：`motion_control` 已 `~/engage`（`/motion_control/status` 里 `arms_live=true`）；
头部 ROS 图像与两路腕部 RTSP 可用；能连到推理服务。本包只发目标，不做使能、不碰控制器切换。

| 槽位 | 输入 | 格式 |
|---|---|---|
| `head` | `/head/camera/color/image_raw` | yuv422_yuy2, 1280x720x30 |
| `left_wrist` | `left_wrist_rtsp_url` | RTSP, 1920x1080x30（stream0） |
| `right_wrist` | `right_wrist_rtsp_url` | RTSP, 1920x1080x30（stream0） |

默认 backend 是 `cogact_unitree`。客户端保持三路图像的原分辨率并编码成 JPEG，缩放由
CogACT server 完成。这些输入 profile 与 `record` 采集时一致；导出器再把训练视频统一为
640x360。当前模型实际看到的训练分辨率为 448x256，服务端使用：

```bash
python -m cogact.inference.serve_batch \
  --checkpoint_path <checkpoint-dir> \
  --dataset_class UnifiedV2EpisodicDataset \
  --image_size 448 256 \
  --has-left --has-right \
  --use_bf16 \
  --port 5500
```

客户端目前发送三路 `image_types`、归一化内参和 `base_T_cam`，其方向与 `record` 默认导出一致：
`world_xyz = extrinsic @ camera_xyz`。注意：训练文件的存储方向不等于 HTTP 接口的输入契约；
仍需用服务端 dataset 和 serve_batch 的消费代码确认是否取逆、是否再次归一化 K，以及响应是否为
`pose_unified`。仅推理成功或客户端测试通过不能证明这些约定正确。机器人侧必须提供：

| 槽位 | 内参来源 | 外参来源（相对 `torso_link`） |
|---|---|---|
| `head` | `/head/camera/color/camera_info` | `camera_color_optical_frame` 历史 TF |
| `left_wrist` | `observation_calibration_file` | 实测关节 FK 到 `camera_left` |
| `right_wrist` | `observation_calibration_file` | 实测关节 FK 到 `camera_right` |

内参宽高必须和对应原图一致；腕部按分辨率精确匹配标定文件。

```bash
# 先决条件 相机参数和 record 保持一致
ros2 launch robot_bringup all_data.launch.py \
  scope:=whole_body topology:=dual \
  wrist_left_url:=rtsp://admin:123456@192.168.123.97/stream0 \
  wrist_right_url:=rtsp://admin:123456@192.168.123.98/stream0 \
  wrist_image_width:=1920 wrist_image_height:=1080 wrist_fps:=30
ros2 launch g1_motion_control motion_control.launch.py
ros2 launch head_sensors head_camera.launch.py \
  color_profile:=1280x720x30 color_format:=YUYV

# 服务在电脑 B 的局域网里时，先从 B 开反向 SOCKS：ssh -N -R 1080 user@<本机>
ros2 launch g1_vla_bridge vla_bridge.launch.py proxy:=socks5h://127.0.0.1:1080

ros2 run g1_vla_bridge vla_cli
# CLI 中：/engage 明确使能；输入任务文字并 Enter 只更新目标；空行 Enter 请求并完整执行
# 一个 30 点 chunk。切换模式先 /stop；/auto on 选择连续并启动，/auto off 选择单段模式；/skip on 跳过中间点，
# /skip off 恢复逐点执行。/estop 急停卸力；/stop 停止 VLA；/quit 只退出 CLI，不急停机器人。
```

启动日志会打出这次用的规格摘要（原点、图像、语义），现场先核这一行。
`~/stop` **只是停止下发新目标**，手臂保持在最后一帧；卸力走 `/motion_control/estop`。

## 安全边界

| 机制 | 参数 | 作用 |
|---|---|---|
| 可选单帧限速 | `cartesian_limit_enabled` / `max_step_pos` / `max_step_ori` | 默认关闭；开启后裁剪笛卡尔单帧步长 |
| 跳过中间点 | `skip_intermediate_waypoints` | 默认关闭；开启后每个 chunk 直接以最后一个有效 waypoint 为目标 |
| 图像到达活性 | `image_timeout_s` | 限制选中图像到达距今时间，不代表曝光年龄 |
| 观测时间门限 | `observation_max_age_s` / `observation_sample_max_age_s` | 限制公共观测年龄和样本相对公共时刻的年龄，不保证曝光同步 |
| 接管检查 | — | `arms_live` 掉了自动 `stop` |
| 只记录不拦截 | `~/status` 的 `jump` / `lead` | manual/continuous 的首点距实测、指令领先实测；async 为 null |

整段准入门（首点离实测太远就丢整段）**已删**：标定没定死之前首点总在 0.3 m 上下，它会
把每一段都拒掉。可选单帧限速在 `motion_control` 的 IK 限幅**之上**，不冲突——那个管的是数值
稳定性，管不住「目标本身给错了」。

## 坐标系与标定

`frame.origin_in_base` 的定义只有一句：**`base_frame` 里的这个点，在 VLA 系里就是原点。**

模型的空间感全部来自图像，对「东西在哪」的判断由**它自己的相机内外参**建立，与手臂长度
无关——所以**相机才是锚点，不是手臂**。整条链路只有一个式子：

```
T_model←base = T_model←cam · (T_base←cam)⁻¹
```

- `T_model←cam` = 训练侧的 `head_camera_in_world`
- `T_base←cam` = 我方 TF 的 `torso_link -> camera_color_optical_frame`

```bash
# 首选：对面给了训练 episode 里真实的 head_camera_in_world（4x4 JSON 或文件）
ros2 run g1_vla_bridge calibrate_frame --camera-in-world train_cam.json

# 备选：对面只给关节值，用训练机的 URDF + 头部外参自己反算
ros2 run g1_vla_bridge calibrate_frame --lift 0.28 --body-pitch 0.5236 --head-pitch 0

# 控制栈没起来时，我方相机位姿也可以手工给：--camera-in-base our_cam.json
```

它打印两套参数：

```
[A] 只对位置：模型系保持水平朝前，原点挪到两台相机重合   <- 现在用这个
[B] 完整六自由度：位置和朝向都重合，但模型系被掰斜
```

**A2D 这个模型泛化很差、对相机位置极敏感**，所以原点不按几何真值取，而是取 `[A]`——让
我们的相机落到训练相机那个位置上。`rotation_rpy` 保持 0：掰了坐标系两台相机的朝向也能
对上，但重力方向就错了、末端 state 跟着歪。代价是俯角仍差 17.8°，这是物理视角差，
`head_reproject` 只改得了焦距和畸变，改不了它。推导过程写在
[config/backends/a2d_omnipicker.yaml](config/backends/a2d_omnipicker.yaml) 里。

**灵敏度：`head_pitch` 每 0.1 rad 让原点变 0.047 m；`body_pitch` 从 0 转到 55° 让原点的
x 差 38 cm。** 务必用真实采样，别拿猜的关节值凑。改了 `origin_in_base` 却没重跑标定，
`test_vla_backend.py` 会拦下来。

## delta 模式（`delta_position` / `delta_rotation`，默认关）

标定不准时可以只取模型整段的**形状**，重锚到当前指令值：

```
out[k].p = anchor.p + (poses[k].p − poses[0].p)
out[k].R = poses[k].R · poses[0].Rᵀ · anchor.R
```

| | 绝对 | delta |
|---|---|---|
| `model_origin_in_base` / `tool_offset` | 必须准 | **相减时抵消** |
| `model_rotation_rpy` | 必须准 | **仍然必须准**（`Δp_model = R · Δp_base`，方向不抵消） |
| `tool_rotation_rpy` | 必须准 | 抵消 |
| 误差累积 | 无 | 指令是累积的，跟不上时会一路往前堆 |

原点对齐好之后一般走绝对模式——delta 会把原点减掉，对齐就白做了。夹爪不走 delta，
它是开合量不是位姿。

**锚点必须是「上一段留下的指令值」，不是实测值**（2026-08-17 踩过，锚在实测上机器人只
在原地抖）：推理一轮约 250 ms，30 Hz 下只播得完 30 个 waypoint 里的前 8 个，实测在这
250 ms 里几乎没动，锚回去就把走过的一截抹掉，再叠上模型噪声就是以约 4 Hz 抖 ±4 cm。
代价是指令可能跑在实测前面，`~/status` 的 `lead` 就是这个领先量。与 `vr_teleop` 的离合
锚点同一套取舍：**绝不拿可达性反馈去修锚点**。

`test_delta_mode.py` 钉死了「偏置必须抵消」「旋转必须不抵消」「进度必须累积」。

## 已知坑

- **坐标系必须和 `motion_control` 一致。** `base_frame: torso_link` 是因为它的 IK 就是
  相对 `torso_link` 解的。改它要同步核对 `motion_control.yaml`——两边不一致不会报错，
  只会让手臂去错地方。
- **相机没订阅者时根本不拉流**，所以刚起来的头 1~3 秒会因图像过期跳过几轮推理，属正常。
- **VLA 启动必须显式使用 record 的相机 profile。** 头部是 `1280x720x30 YUYV`，腕部是
  两路 `stream0 1920x1080x30`；不要沿用普通 bringup 的低带宽 stream1 配置。图像不在
  机器人侧缩放，由 CogACT server 统一缩到 448x256。
- **`head_reproject`** 把我们的图重采样到训练相机内参上（焦距差 1.42 倍，同一物体在我们
  图里大 42%），代价是画布只填得满约 49%、其余靠边缘外推——**那本身也是分布偏移**。用
  分别配置开关后用预检比较。修正只做**输入侧**：焦距失配是角度误差不是三维相似变换，
  输出侧再乘系数是双重修正。
- **畸变顺序**：厂商 JSON 给 `k1 k2 k3 p1 p2`，OpenCV/ROS 要 `[k1,k2,p1,p2,k3]`。抄错
  不报错，只会悄悄画歪。
- `RemoteDisconnected` / curl exit=52 → 服务端不回数据。`ssh -R` 的反向 SOCKS 是**乐观
  应答**（关闭的端口也回 "request granted"），拿对照端口分不出「服务挂了」还是「路由断
  了」，只能去 B 上 curl。
- 打不通时先看 `/vla_bridge/status` 的 `error` 字段，那里是原始异常。

## 测试

```bash
python3 -m pytest src/g1_vla_bridge/test -q
python3 -m pycodestyle --ignore=E501,W503 \
    src/g1_vla_bridge/g1_vla_bridge src/g1_vla_bridge/test src/g1_vla_bridge/launch
```

`test_*.py` 不启动 ROS 节点、不访问网络；部分测试需要已安装 ROS Python 包。
`test/observation_preflight.py` 是使用真实观测入口的只读传感器预检，命令见上方
「与 record 的观测对齐」。它不调用模型、不验证闭环跟踪。
执行回归测试直接跑真实推理线程与下发回调（mock backend/publisher），逐帧核对双臂位姿、
夹爪、单次请求和 stop/start 后旧响应丢弃；关闭 delta 与 VLA 限速、未冻结且 horizon=0 时，
完整输出每个 waypoint。底层 IK、关节限速和物理跟踪误差仍然存在。
