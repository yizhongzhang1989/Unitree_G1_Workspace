# Lower Velocity Tracking

独立腿腰速度任务，拥有 0–14 槽并直接交付掩码关节目标。TaskBase 提供传感器、任务注册和启停事件；本包持有站立过渡、速度平滑、GRU 隐状态与策略推理，以及 [teleop_keyboard.py](lower_velocity_tracking/teleop_keyboard.py) 的键盘控制业务。上肢由独立任务处理，默认键盘组合选择 [upper_float](../upper_float/README.md)。

## 启动

推荐在工作区根目录使用 [robot_bringup 集中入口](../robot_bringup/README.md)。硬件显式另起，两个终端各先加载环境：

```bash
# 终端 A，已有硬件栈时复用
source scripts/env.sh
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual
```

```bash
# 终端 B，默认 session + 下肢速度 + 上肢浮动 + 键盘
source scripts/env.sh
ros2 run robot_bringup teleop_keyboard
```

`--upper-task` 可选 `float/vr/vla`，默认 `float`；`--lower-config` 与 `--upper-config` 指定任务配置，未知参数转交键盘节点作为 ROS 参数。正式入口只启动一个 session、任务与控制台，不启动硬件、不自动接管；接管与策略启动由操作者显式执行。

手动拆分部署时，先启动唯一 [g1_motion_session](../g1_motion_session/README.md)，再选择以下一条下肢 launch，不与正式键盘入口或组合 launch 叠加：

```bash
ros2 launch lower_velocity_tracking velocity_tracking.launch.py
ros2 launch lower_velocity_tracking velocity_tracking.launch.py config:=lower_vr.yaml
```

`config` 支持本包配置名或绝对路径，默认配置为 [lower_body.yaml](config/lower_body.yaml)，VR 输入使用 [lower_vr.yaml](config/lower_vr.yaml)。基础参数与 profile 由 `g1_motion_session.launch_support.task_parameters` 合并，31 轴顺序由公共控制参数加载，可 override。运行环境需已安装 onnxruntime；已加载 Humble 与所需 SDK 环境时也可 `source install/setup.bash`。

## 键盘控制

已有 session 和下肢任务时，可另开交互终端启动 control-only 控制台；它不启动任务、会话或硬件：

```bash
source scripts/env.sh
ros2 run lower_velocity_tracking teleop_keyboard
```

| 按键 | 操作 |
|---|---|
| `G` | 请求 session 接管并准备站姿 |
| Enter | 准备就绪后启动下肢策略 |
| `W/S`、`A/D`、`J/L` | 前后、左右、转向速度 |
| `I/K` | 增减目标高度 |
| `X` | 速度清零，高度保持 |
| 空格 | 请求 session 急停 |
| `Q` / Ctrl+C | 退出前先速度清零并请求急停 |

键盘持续发布 `[vx, vy, wz, height]`，默认 50 Hz。速度在无按键刷新超过 `hold_timeout_s` 后衰减到零，高度保持；终端粗限幅之外，策略节点继续执行自身限幅与平滑。键盘控制生命周期和下肢速度，不直接生成手臂关节目标。

ROS 参数可通过两种入口传入，例如 `ros2 run robot_bringup teleop_keyboard --ros-args -p vx_step:=0.05` 或 `ros2 run lower_velocity_tracking teleop_keyboard --ros-args -p vx_step:=0.05`。正式组合键盘退出时，先请求 session 急停，再向自身启动的任务进程组发 SIGINT；control-only 退出同样先请求急停，但只关闭自身控制台，不停止已有任务进程。两者均不停止既有硬件进程。

## 接口

速度入口为 `/motion_control/lower/input/<lower_source>`，消息 `std_msgs/Float64MultiArray`，内容 `[vx, vy, wz, height]`。默认来源 `keyboard`，VR profile 为 `vr`。

`engage` 由 TaskBase 执行两段准备插值，终点发布后由 FPC 保持，等待显式会话 `start`；`start(snapshot)` 重置策略与 GRU 后逐拍推理。`stop` 停止本任务输出，FPC 保持最后目标。`status()` 提供 `lower_ready`、`lower_executor`、`lower_source`、`command` 字段，基类汇总生命周期状态。

## 验证

```bash
python3 -m pytest -q src/lower_velocity_tracking/test
```