# G1 动作会话

## 上下肢划分

本包协调独立运行的上肢与下肢任务，按[硬件关节表](../unitree_g1_ros2_control/config/default_31dof_param.yaml)的 31 轴顺序登记所有权：下肢为 0–14 位的腿和腰，上肢为 15–30 位的双臂和夹爪。任务自行订阅反馈、计算目标并直接发布到 FPC，会话负责登记、生命周期事件、状态汇总和控制器启停。会话自身生成位置目标的例外是未登记关节的默认准备与保持（default/unowned prepare）；该输出只覆盖当前无人拥有的掩码。

### 上肢能力

| 任务包 | 能力 | 入口 |
|---|---|---|
| [upper_vr](../upper_vr/README.md) | 手柄双臂遥操作、夹爪控制与回位 | [vr_teleop.launch.py](../upper_vr/launch/vr_teleop.launch.py) |
| [upper_vla](../upper_vla/README.md) | VLA 推理、动作执行、观测历史和在线 RL | [vla_bridge.launch.py](../upper_vla/launch/vla_bridge.launch.py) |
| [upper_float](../upper_float/README.md) | 双臂实测位置跟随、夹爪保持与回位 | [float.launch.py](../upper_float/launch/float.launch.py) |

会话内任务默认由 `TaskBase` 执行 `engage` 准备插值，准备终点发布后调用 `prepared(snapshot)`，不会调用 `start()`。`upper_vr`、`upper_vla` 和 `upper_float` 在准备完成通知中开放上肢执行，与原先准备结束即可操作的行为一致；VLA 只播种 IK，推理仍需通过 `/vla_bridge/start` 显式启动。

录制回放在 [record](../record/README.md) 内复用 IK 工具，非会话任务。

### 下肢能力

| 任务包 | 能力 | 入口 |
|---|---|---|
| [lower_velocity_tracking](../lower_velocity_tracking/README.md) | 速度模型执行与速度指令跟踪 | [velocity_tracking.launch.py](../lower_velocity_tracking/launch/velocity_tracking.launch.py) |
| [lower_footstep_tracking](../lower_footstep_tracking/README.md) | 足步模型执行与落脚计划跟踪 | [footstep_tracking.launch.py](../lower_footstep_tracking/launch/footstep_tracking.launch.py) |

下肢任务将模型站姿作为基类准备目标，报告 `ready` 后等待会话 `start`，再重置并运行策略；准备结束到 start 之间由 FPC 保持终点。组合启动与正式键盘入口集中在 [robot_bringup](../robot_bringup/README.md)，默认键盘入口启动会话、下肢速度和上肢浮动；键盘控制业务由 [lower_velocity_tracking](../lower_velocity_tracking/README.md) 实现。[whole_vr.launch.py](../robot_bringup/launch/whole_vr.launch.py) 是 `robot_bringup` 对会话、`upper_vr` 和使用 VR 指令的 `lower_velocity_tracking` 的启动组合；[motion_tasks.launch.py](../robot_bringup/launch/motion_tasks.launch.py) 提供可选的上下肢组合，默认上肢仍为 VR。

## 架构

```mermaid
flowchart TD
    Sensors[关节状态与盆骨 IMU] --> Session[g1_motion_session]
    Sensors --> Upper[上肢任务]
    Sensors --> Lower[下肢任务]
    Description[robot_description] --> IK[任务进程内 ArmIKMixin]
    IK --> Upper
    Upper -->|登记掩码与任务状态| Session
    Lower -->|登记掩码与任务状态| Session
    Session -->|可靠生命周期事件| Upper
    Session -->|可靠生命周期事件| Lower
    Session -->|未登记关节目标| FPC[ForwardPositionController]
    Upper -->|所拥有的关节目标| FPC
    Lower -->|所拥有的关节目标| FPC
    Session -->|激活与急停停用| Manager[controller_manager]
```

[TaskBase](g1_motion_session/task_base.py) 是会话内任务进程的 ROS 节点基类，提供登记客户端、事件订阅、任务状态发布、反馈订阅和掩码目标输出。控制周期数据不经会话转发。[ArmIKMixin](g1_motion_session/arm_ik_mixin.py) 在使用它的上肢进程内绑定机器人描述、求解与提交 IK，并共享[手臂 IK 配置](config/arm_ik.yaml)，不是远程 IK 服务。

## 构建与启动

在工作区根目录构建当前会话、任务与启动组合：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to g1_motion_session lower_velocity_tracking lower_footstep_tracking upper_vr upper_vla upper_float record robot_bringup --symlink-install
source install/setup.bash
```

以下命令在工作区根目录执行，每个终端先 `source scripts/env.sh`；已加载 Humble 与所需 SDK 环境时，也可 `source install/setup.bash`。设备侧由 [all_data.launch.py](../robot_bringup/launch/all_data.launch.py) 显式另起唯一控制器管理器：

```bash
source scripts/env.sh
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual
```

另一交互终端启动默认组合，一次启动一套会话、下肢速度任务、上肢浮动任务与键盘控制台：

```bash
source scripts/env.sh
ros2 run robot_bringup teleop_keyboard
```

正式键盘入口支持 `--upper-task {float,vr,vla}`、`--lower-config`、`--upper-config`，其余参数转交键盘节点作为 ROS 参数。入口不启动硬件、不自动 `engage` 或 `start`；退出先请求急停，再对自身启动的任务进程组发 SIGINT，既有硬件进程保持运行。

其他组合任选其一，均包含一套会话：

```bash
ros2 launch robot_bringup whole_vr.launch.py
ros2 launch robot_bringup motion_tasks.launch.py upper_task:=vla
ros2 launch robot_bringup motion_tasks.launch.py lower_task:=none upper_task:=float
```

已有会话与任务时，额外键盘终端使用 `ros2 run lower_velocity_tracking teleop_keyboard`，这是 control-only，只连接并控制既有任务，不重复启动会话、任务或硬件。独立部署是正式键盘入口或组合 launch 的另一种选择：[session.launch.py](launch/session.launch.py) 只启动协调节点，再用各任务包的 launch 启动所选上下肢。每套组合只包含一个会话，只选择一套方案，避免重复会话或任务登记。[dashboard.launch.py](launch/dashboard.launch.py) 可另起面板，不启动第二套会话。面板从 `status_topic`（默认 `/motion_control/status`）读取上肢任务的原始末端目标 `command_pose` 和 IK 提交关节指令正解 `limited_pose`，两者均为按 `left` / `right` 索引的 `[x, y, z, qx, qy, qz, qw]`；实测末端由页面模型计算，姿态权重写回当前任务的参数服务。

```bash
# 手动拆分部署时只启动一次会话
ros2 launch g1_motion_session session.launch.py

# 面板可连接已有会话
ros2 launch g1_motion_session dashboard.launch.py
```

反馈和任务登记就绪后接合；下肢及默认准备完成、`ready_to_start` 为真时再启动下肢策略。调用成功的 `engage` 仅表示已接受，控制器激活在后台继续完成；`start` 在尚未就绪时立即拒绝，不阻塞等待。

```bash
ros2 service call /motion_control/engage std_srvs/srv/Trigger '{}'
ros2 service call /motion_control/start std_srvs/srv/Trigger '{}'
```

使用 VLA 时，在上肢执行器就绪并满足该任务的服务端、任务和观测条件后，另行启动推理：

```bash
ros2 service call /vla_bridge/start std_srvs/srv/Trigger '{}'
```

## 生命周期

每次 `engage` 广播递增的事件 epoch 和提前 50 ms 的共同 ROS 时钟起点。未运行任务从最新实测位置重新准备，已经 `RUNNING` 的任务忽略该事件并继续执行；未登记关节也重新进行默认准备。运行期间新登记的任务从 idle 开始，登记本身不触发动作，需要再次 `engage`。准备完成时调用任务的 `prepared(snapshot)`；上肢任务在该回调中开放执行，下肢等待显式会话 `start`。会话 `start` 要求控制器已激活、默认准备结束，且所有非运行任务均无错误并报告 ready。

`stop` 广播任务停止事件并停止会话默认目标发布，FPC 保持最后接收的目标，控制器仍激活；`estop` 先执行同样的停止广播，再请求停用控制器。会话在活动状态检查关节与 IMU 新鲜度和躯干倾角。任务准备、启动或回位抛出异常时由 `TaskBase` 转为本地 `ERROR` 并报告错误；生命周期定时器异常还调用本地停止处理，这不等于会话全局 `estop`。

```bash
ros2 service call /motion_control/stop std_srvs/srv/Trigger '{}'
ros2 service call /motion_control/home_left std_srvs/srv/Trigger '{}'
ros2 service call /motion_control/home_right std_srvs/srv/Trigger '{}'
ros2 service call /motion_control/home_cancel std_srvs/srv/Trigger '{}'
ros2 service call /motion_control/estop std_srvs/srv/Trigger '{}'
```

左右回位由拥有对应七轴手臂的任务在本地执行两段轨迹，总时长默认 2 秒。服务等待参与任务报告完成，超时为 `home_s + 1` 秒；取消、停止、急停、任务注销或任务错误会中断等待。

## 登记与接口

- `/motion_control/register` 使用 [RegisterTask.srv](srv/RegisterTask.srv)：请求含 `task_id`、`mask`、`release`，响应含 `accepted`、`reason`。有效位为 0–30；任务名必须非空，掩码在有效位内必须非空，不同任务的所有权重叠会被拒绝。登记在默认发布共用锁内立即移除新拥有的位。
- `TaskBase` 启动后每 0.5 秒尝试登记；接受后停止重试，明确拒绝也停止重试并报告原因。退出销毁节点时，若已经登记、ROS context 有效且服务可用，则异步请求 `release=True`；这是尽力注销，不保证请求送达，异常退出也不会自动释放所有权。
- 显式注销会移除任务，将释放位交回默认输出，并以当前实测位置重置默认目标数组。下一次 `engage` 再从实测位置执行默认准备。
- `/motion_control/events` 使用 [MotionEvent.msg](msg/MotionEvent.msg)，QoS 为 RELIABLE、VOLATILE，字段为 `command`、`epoch`、`start_time`；命令包括 `engage`、`start`、`stop`、`home_left`、`home_right`、`home_cancel`。晚加入任务不会收到历史事件。
- `/motion_control/tasks` 使用 RELIABLE 的 JSON `std_msgs/String`，包含 `task_id`、`mask`、小写 `state`、`epoch`、`ready`、`error` 及任务附加状态；准备状态为 `preparing`，准备完成等待启动为 `ready`。回位期间对应 `home_left` 或 `home_right` 为真，完成后为假。
- `/motion_control/status` 汇总会话状态 `idle/engaging/stand/running/estop`、`ready_to_start`、`execution_epoch`、反馈新鲜度、关节和任务摘要。执行 epoch 在首次成功激活或急停后的重新激活时更新，已激活控制器上的重复 `engage` 保持该值。
- 任务和会话默认输出直接发布 [JointTargets.msg](../unitree_g1_ros2_control/msg/JointTargets.msg)，各自只写拥有的掩码；底层接口见 [unitree_g1_ros2_control](../unitree_g1_ros2_control/README.md)。

## 配置

[session.yaml](config/session.yaml) 以 `/**` 提供准备、回位和反馈安全参数。默认未登记下肢目标为站姿，上肢目标为被动姿态；默认准备先用 0.4 秒将肩 roll 向外清至至少左 +0.7、右 -0.7 rad，其余轴保持起始值，再在剩余 2.1 秒移到默认目标。只发布未登记位，全被登记时无需会话默认准备。

[task_parameters](g1_motion_session/launch_support.py) 按顺序合并会话公共配置、任务使用的共享手臂 IK 配置、任务包默认配置、指定覆盖配置，最后注入规范硬件关节顺序。配置可用包内相对路径或绝对路径。[CMakeLists.txt](CMakeLists.txt) 安装 Python 实现、生成的 ROS 接口、配置、launch 和可执行入口。

## 验证

[登记与生命周期单测](test/test_registry.py) 覆盖掩码冲突、默认输出交接、重复接合、就绪门控、任务异常、停止与急停、回位和配置覆盖。[真实 ROS 生命周期测试](test/test_ros_lifecycle.py) 验证独立节点通信；[VLA ROS 初始化测试](../upper_vla/test/test_ros_initialization.py) 验证真实 VLA 节点登记、机器人描述绑定与无推理准备。

构建并加载当前工作区后，在隔离 ROS 域执行已有测试：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=217 python3 -m pytest src/g1_motion_session/test/test_registry.py src/g1_motion_session/test/test_ros_lifecycle.py src/upper_vla/test/test_ros_initialization.py -q
```
