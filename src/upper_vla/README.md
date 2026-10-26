# upper_vla
VLA 上肢任务，继承 `TaskBase` 并使用 `ArmIKMixin`。下肢由独立任务控制，本包负责双臂和夹爪的 15..30 槽关节目标。

```
采观测 → backend.infer() → 重锚 → 可选逐帧限幅 → 末端缓存 → 本地 50 Hz IK → JointTargets → FPC
```

节点只认三样东西，全部定义在 [vla_backend.py](upper_vla/vla_backend.py)：

| | 是什么 | 表示在哪个系 |
|---|---|---|
| `Observation` | 图像 + 双臂末端位姿 + 夹爪 + 各视角相机内外参 + 任务指令 | `base_frame`（默认 `torso_link`） |
| `ActionChunk` | N 个 waypoint 的末端位姿 + 夹爪 | `base_frame` |
| `VlaSpec` | 这家 VLA 的**规格**：坐标系原点在哪、要几张什么图、夹爪怎么换算 | — |

模型系的换算由 backend 使用 `spec.frame` 完成，[vla_node.py](upper_vla/vla_node.py) 负责通用观测与执行。

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
        timer["动作播放<br/>execution_rate_hz，可选限幅"]
        target[("末端目标缓存")]
        ik["ArmIKMixin<br/>50 Hz 本地追踪"]
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
    timer --> target --> ik
    ik --> gate["TaskBase<br/>RUNNING / generation 门禁"]
    gate --> mc["JointTargets · 31 槽 / 上肢 mask<br/>FPC"]
```

## 运行

组合启动统一使用 [robot_bringup](../robot_bringup/README.md)：`ros2 launch robot_bringup motion_tasks.launch.py upper_task:=vla` 启动一个 session、下肢速度任务和上肢 VLA；需要当前终端提供下肢键盘控制时，另选 `ros2 run robot_bringup teleop_keyboard --upper-task vla`。默认键盘入口的上肢是 float，直接启动 `motion_tasks.launch.py` 默认上肢仍为 VR，只有显式选择 vla 才启动本任务。硬件与相机显式另起，运动组合不自动接管，也不自动启动推理。

推理前置：session 已 engage 并完成本任务准备，本地执行器为 `RUNNING`、IK 模型就绪；头部 ROS 图像与两路腕部 RTSP 可用，并连接推理服务。TaskBase 管理准备、接管和关节发布，本地 IK 直接发布到 FPC，关节合成由 ros2_control 完成。广播 start 用于下肢策略，不启动 VLA 推理，仍需显式 `~/start`。

| 槽位 | 输入 | 格式 |
|---|---|---|
| `head` | `/head/camera/color/image_raw` | yuv422_yuy2, 1280x720x30 |
| `left_wrist` | `left_wrist_rtsp_url` | RTSP, 1920x1080x30（stream0） |
| `right_wrist` | `right_wrist_rtsp_url` | RTSP, 1920x1080x30（stream0） |

默认 backend 为 `cogact_unitree`，启动检查服务端 `/api/health` 和 `/api/config`。三图完整缩放至 640×360 后 JPEG 编码，发送 `return_dict=true`、归一化内参和 `base_T_cam` 外参（`base_xyz = extrinsic @ camera_xyz`），接收绝对 30 步动作。服务端消费约定需另行核对，仅推理成功不能证明坐标与图像契约正确。标定来源：

| 槽位 | 内参来源 | 外参来源（相对 `torso_link`） |
|---|---|---|
| `head` | `/head/camera/color/camera_info` | `camera_color_optical_frame` 历史 TF |
| `left_wrist` | `observation_calibration_file` | 实测关节 FK 到 `camera_left` |
| `right_wrist` | `observation_calibration_file` | 实测关节 FK 到 `camera_right` |

内参宽高必须与原图一致。服务地址、代理和历史数量见 [后端配置](config/backends/cogact_unitree.yaml)。

```bash
python3 -m pip install --user -r src/upper_vla/requirements.txt
colcon build --packages-select upper_vla --symlink-install
source scripts/env.sh

# 硬件终端，相机参数和 record 保持一致，已有栈时复用
ros2 launch robot_bringup all_data.launch.py \
  scope:=whole_body topology:=dual \
  wrist_left_url:=rtsp://admin:123456@192.168.123.97/stream0 \
  wrist_right_url:=rtsp://admin:123456@192.168.123.98/stream0 \
  wrist_image_width:=1920 wrist_image_height:=1080 wrist_fps:=30
ros2 launch head_sensors head_camera.launch.py \
  color_profile:=1280x720x30 color_format:=YUYV

# 任务终端，启动 session、下肢速度与上肢 VLA
source scripts/env.sh
ros2 launch robot_bringup motion_tasks.launch.py upper_task:=vla

# 控制终端，显式接管、准备完成后启动推理
source scripts/env.sh
ros2 run upper_vla vla_cli
```

只选择一套包含 session 的方案。VLA 组合 launch 启动后若需要键盘控制下肢，另开交互终端执行 `ros2 run lower_velocity_tracking teleop_keyboard`，它是 control-only，不启动任务、session 或硬件；也可直接改用正式键盘入口的 vla 组合，但不要两套同时启动。键盘入口支持 `--lower-config` 与 `--upper-config`，未知参数作为键盘 ROS 参数转交；组合 launch 的任务配置使用 `lower_config:=... upper_config:=...`。已加载 Humble 与所需 SDK 环境时，也可 `source install/setup.bash`。

手动拆分是组合入口的另一种方案：先显式启动唯一 session 和所需下肢任务，再启动上肢包。单包参数覆盖仍按原 launch 使用；例如服务在电脑 B 的局域网里时，先从 B 开反向 SOCKS `ssh -N -R 1080 user@<本机>`，再运行 `ros2 launch upper_vla vla_bridge.launch.py proxy:=socks5h://127.0.0.1:1080`。组合入口只转交任务配置文件，不将这些单包参数透传给上肢；已有组合中应使用配置文件选择相应参数，而不另起第二个 bridge。

正式组合键盘退出先请求急停，再向自身启动的任务进程组发送 SIGINT，既有硬件进程保持运行；VLA 组合 launch 以 Ctrl+C 关闭自身组合。VLA 推理通过 CLI `/start` 或 `/vla_bridge/start` 手动启动，组合启动与推理 start 是两个独立阶段。

CLI 输入任务文字只更新指令，空行 Enter 请求并完整执行一段；`/engage` 请求 session 接管，`/start` 进入 VLA 待命，`/stop` 停止推理和播放，本地 IK 继续追踪最后末端目标；session stop 则关闭关节发布。`/estop` 请求 session 急停。`/auto [on|off]` 选择连续或手动，开启时自动 start；`/skip [on|off]` 选择末点直达或逐点执行。`/quit` 只退出普通 CLI，不停止节点。不要同时启动两个控制 bridge；频率参数在启动时读取，修改后需重启。

Online RL 人工评分的用途、操作和恢复见 [独立说明](upper_vla/online_rl/README.md)。

## 执行方式
推理在线程中进行，下发由定时器完成，避免网络和模型耗时阻塞控制。`manual` 每次请求完整执行一个 chunk，播完等待下一次 Enter；`continuous` 播完后等待 `continuous_next_delay_s` 再请求下一段。推理失败时保持当前目标，manual 等待人工请求，continuous/async 按 `retry_delay_s` 重试。

默认观测和模型动作均为 10 Hz，末端目标播放为 30 Hz：位置和夹爪线性插值，姿态 SLERP，30 个模型动作约三秒，不是加速三倍。独立 50 Hz IK 定时器持续追踪末端缓存，包括等待新 chunk、业务 stop 和 Cartesian home 期间。`action_horizon` 限制每段使用的前缀，0 表示全部；`hold_*` 冻结对应侧的执行。

### 异步执行
即得到请求返回结果后立即观测并执行下一次请求。重合的action之间使用加权融合，加权方案为：第 k 个动作的新预测权重为 `k/(x+1)`，旧预测权重为 `1-k/(x+1)`，其中 x 为重合的动作点数，k=1..x。位置按此线性加权，旋转按同一权重 SLERP，夹爪不做跨 chunk 加权，重合点直接用新预测覆盖。

为减少推理间隙的目标突变，`async_min_overlap_actions` 默认 7，不足时用旧末点或当前指令补齐，权重分母相应使用实际融合长度。动作时间从选中观测起算，首点在观测后 0.1 秒；仅执行未来点，不补播过期动作，队列耗尽时保持最后目标并继续请求。async 只支持绝对位姿，要求关闭 `delta_position`、`delta_rotation` 和 `skip_intermediate_waypoints`。

```bash
ros2 launch upper_vla vla_bridge.launch.py execution_mode:=async \
  async_min_overlap_actions:=7
ros2 run upper_vla vla_cli
```

运行中切换用 `/stop` → `/mode async` → `/start`；模式变更清空队列并作废旧请求。`~/status` 中的 `async_pending`、`async_buffer_s` 用于查看预测缓存，融合本身不保证闭环稳定或避障。

### 与 record 的观测对齐
为了让模型输入接近训练数据，三个模式共用固定 10 Hz 时间格：选取不晚于公共时刻的图像和关节样本，不插值。腕图直接读取 RTSP，用原始 PTS 减既有 110 ms 延迟补偿；头图保留 ROS 时间戳。双臂 FK、腕外参和夹爪来自同一条实测关节，头部外参使用该时刻的历史 TF。腕内参按原图分辨率精确匹配 `observation_calibration_file`，默认使用 camera_calibration 的标定文件。

缺图、关节或 TF 时不请求模型；普通 bridge 不按数据年龄拦截。断流重连，时间戳回退清对应缓存，关节或发令时间回退还会停止执行。`~/status` 提供观测年龄、时间偏差和腕流错误，便于检查输入质量。在线原始 PTS 与离线拟合时间不完全一致；动态头部的训练/导出尚未闭合，不能承诺逐帧相同。

### 真实动作与反馈历史
为模型提供执行上下文，请求附带最近 `history_length` 对真实 action/state，默认 16 对。播放将位姿和夹爪原子更新到末端缓存后进入待配对队列，以首条不早于命令时间的实测反馈配对，每个 100 ms 时间格取首条记录。action 保存末端 command，state 使用实测 JointState 的 FK；关节限幅后的提交 FK 只用于 IK 状态，不替代历史测量。历史只取当前观测之前的记录，旧到新排列，不足时只发已有记录，首次为 null；等待不会补造样本。start、stop、任务变化和 reset 清空本地历史，reset 不请求服务端。缓存保留整个 episode 供延迟观测回查，无反馈时待配对队列可能增长；配对不是控制器接收或机械臂到位确认。

## 接一个新的 VLA
backend 隔离模型协议、坐标转换和图像预处理，节点只负责观测与执行。新增同名模块和配置，再通过 `vla_backend` 选择：

```
upper_vla/backends/<名字>.py     # SPEC + PARAMETERS + create(params)
config/backends/<名字>.yaml          # 参数值，launch 按 vla_backend 自动挂上
```

模块导出 `SPEC`、`PARAMETERS` 和 `create(params)`，实现 `infer(Observation) -> ActionChunk`；可参考 [cogact_unitree.py](upper_vla/backends/cogact_unitree.py)。接入前核对：

| 约定 | 定义位置 |
|---|---|
| 模型坐标系原点、朝向 | `FrameSpec.origin_in_base / rotation_rpy` |
| 末端参考点和姿态轴 | `FrameSpec.tool_offset / tool_rotation_rpy` |
| 图像槽位、顺序、尺寸和预处理 | `ImageSpec` 与 backend |
| 相机内参、畸变和外参方向 | backend 请求协议 |
| 夹爪范围与开合方向 | `GripperSpec` |
| 绝对或增量动作、序列长度 | `VlaSpec.action_semantics / horizon` |

通用配置放 [config/vla_bridge.yaml](config/vla_bridge.yaml)，模型参数放同名 backend YAML，键不重叠。生效顺序为代码默认值 → 通用配置 → backend 配置 → launch 覆盖。

### 双臂复位
`/home` 用于回到固定双臂起始姿态：先停止 VLA、清空历史和播放队列、作废在途响应，再更换本地双臂 Cartesian 缓存，由 50 Hz IK 持续追踪，夹爪保持不变。也可调用 `~/home` Trigger 服务。必须已接管手臂，要求 `torso_link` 和左右 `gripper_base`，不受冻结或禁用侧限制。固定位姿见 [HOME_POSES](upper_vla/vla_node.py)，服务成功仅表示目标已缓存，不代表到位或自动恢复推理。

### 夹爪截断实验
`gripper_gate.launch.py` 在夹爪开合跨过阈值时终止后续动作，让下一次 Enter 重新观测：从 `>=0.75` 降到 `<0.75`，或从 `<=0.25` 升到 `>0.25` 触发。启动 `ros2 launch upper_vla gripper_gate.launch.py`，仍使用普通 `vla_cli`。

## 安全边界

| 机制 | 参数 | 作用 |
|---|---|---|
| 可选单帧限速 | `cartesian_limit_enabled` / `max_step_pos` / `max_step_ori` | 默认关闭；开启后裁剪笛卡尔单帧步长 |
| 跳过中间点 | `skip_intermediate_waypoints` | 默认关闭；开启后每个 chunk 直接以最后一个有效 waypoint 为目标 |
| 接管检查 | — | Base 状态与 generation 门禁；session stop 禁止关节发布 |
| 只记录不拦截 | `~/status` 的 `jump` / `lead` | manual/continuous 的首点距实测、指令领先实测；async 为 null |

公共 IK 保护和关节限速仍生效，但不保证模型目标正确或轨迹安全。业务 `/stop` 保留末端目标并持续追踪；需要关闭关节发布或卸力时使用 session stop/estop。

## delta 模式（`delta_position` / `delta_rotation`，默认关）
用于保留模型轨迹的相对变化，将首点重锚到上一段留下的指令值；仅适用于绝对动作的 manual/continuous：

```text
out[k].p = anchor.p + (poses[k].p − poses[0].p)
out[k].R = poses[k].R · poses[0].Rᵀ · anchor.R
```

锚在指令而非实测上，是为了避免跟踪滞后把已累积的轨迹进度抹掉；代价是指令可能领先实测，可看 `~/status.lead`。位置相减能消去固定原点偏置，但方向误差不会抵消，不能代替坐标标定。夹爪不做 delta；标定对齐后通常使用绝对模式。

## 已知坑
- `base_frame` 与本地 IK 共用同一参数，改动时核对观测与标定。
- 相机 profile 必须显式使用 record 配置；普通 bringup 的低带宽 stream1 不能替代腕部 stream0。
- 畸变系数：厂商 JSON 常为 `k1,k2,k3,p1,p2`，OpenCV/ROS 为 `k1,k2,p1,p2,k3`。
- 网络错误先看 `~/status.error`；反向 SOCKS 的连接成功不代表服务端可用，必要时在服务端本机 curl。

## 测试与预检
```bash
python3 -m pytest src/upper_vla/test -q
python3 -m flake8 --extend-ignore=E501 \
    src/upper_vla/upper_vla src/upper_vla/test src/upper_vla/launch
python3 src/upper_vla/test/observation_preflight.py
```

回归覆盖坐标变换、真实执行回调、停止后的响应作废、50 Hz 本地 IK、协议和反馈恢复；使用纯 mock 与本机 HTTP，不调用远端模型或控制真机。部分测试需要已安装 ROS Python 包和共享 `g1_motion_session` 模块。`observation_preflight.py` 使用真实观测入口做只读传感器检查，不调用模型、不发运动命令。这些检查不验证真实模型效果和机械臂闭环跟踪。

## 共享接口依赖
[TaskBase](../g1_motion_session/g1_motion_session/task_base.py) 提供准备插值、生命周期事件和掩码目标发布。VLA 注册 engage/stop/status 回调；`prepared(snapshot)` 播种 IK 并开放本地执行，推理仍通过 `~/start` 手动启动。运行期间由本任务 50 Hz IK 追踪末端缓存，`publish_target(positions31)` 返回 False 时不提交 IK 状态。

[ArmIKMixin](../g1_motion_session/g1_motion_session/arm_ik_mixin.py) 在任务进程内绑定模型、求解与提交 IK，使用实际发布后的关节向量更新状态；末端 command 历史与实测反馈分别保留。