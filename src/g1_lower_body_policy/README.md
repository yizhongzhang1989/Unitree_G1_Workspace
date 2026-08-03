# g1_lower_body_policy —— 下肢 ONNX 策略 + 上肢 IK（真机部署交接文档）

在 `forward_position_controller`（下称 FPC）之上加的**唯一一层**。一个 50 Hz 定时器算齐 31 轴：

| 段 | 轴数 | 来源 |
|---|---|---|
| 下肢（12 腿 + 3 腰） | 15 | ONNX 策略，输入 `vx / vy / wz / h` |
| 上肢双臂 | 14 | 双臂 IK，输入两个末端位姿（`*_gripper_base` 相对 `torso_link`） |
| 夹爪偏心轴 | 2 | 直接透传，只按 URDF 行程裁剪 |

不分层、不加第二个定时器、不加第二个发布者。手臂**始终开启**，没有开关参数也没有服务。
策略的职责是在上肢被遥操 / VLA 随意摆布时保持平衡并跟随速度与盆骨高度指令。

```mermaid
flowchart TD
    OP["键盘遥控台（发长度 4）"]
    VLA["上肢遥操 / VLA（发长度 2 / 7 / 14 / 20）"]
    OP -- "/lower_body_policy/command" --> POL
    VLA -- "/lower_body_policy/command" --> POL

    subgraph POLICY["lower_body_policy 节点（本包，50 Hz 单定时器）"]
        POL["观测装配 → ONNX → 下肢 15 轴<br/>双臂 DLS IK → 上肢 14 轴<br/>夹爪 2 轴透传"]
    end

    JS["/joint_states<br/>sensor_msgs/JointState · 100 Hz"] --> POL
    IMU["/pelvis_imu_broadcaster/imu<br/>sensor_msgs/Imu · 100 Hz"] --> POL
    RD["/robot_description<br/>std_msgs/String · latched"] --> POL

    POL -- "/forward_position_controller/commands<br/>Float64MultiArray[31]" --> FPC
    POL -- "/lower_body_policy/status<br/>String(JSON) · 10 Hz" --> OP
    POL -. "engage / start / estop<br/>经 /controller_manager/switch_controller" .-> CM

    subgraph RC["ros2_control_node（500 Hz 实时环）"]
        CM["controller_manager"]
        FPC["forward_position_controller<br/>手臂重力补偿在这一层"]
        HW["G1TopicSystem<br/>硬件插件"]
        JSB["joint_state_broadcaster"]
        IMUB["pelvis_imu_broadcaster"]
        FPC -- "position command interface" --> HW
        HW -- "state interface" --> JSB
        HW -- "sensor interface" --> IMUB
    end

    CM -. "激活 / 反激活（反激活触发卸力斜坡）" .-> FPC
    JSB --> JS
    IMUB --> IMU

    HW -- "/lowcmd (unitree_hg/LowCmd) · 500 Hz<br/>29 轴 q/kp/kd" --> G1["G1 本体 29 电机"]
    HW -- "/grip_arm0 与 /grip_arm1 的 mit_command · 100 Hz" --> GRIP["Gloria-M 夹爪 ×2"]
    G1 -- "/lowstate (unitree_hg/LowState) · 500 Hz" --> HW
    GRIP -- "/grip_arm0 与 /grip_arm1 的 joint_states" --> HW
```

## 1. 指令话题契约：`~/command` 按长度分块

上下肢共用**一个**话题。全量 20 值的布局如下，其余合法长度都是它的**连续子块**：

```
下标  0   1   2   3 | 4 ....... 10 | 11 ...... 17 | 18   19
     vx  vy  wz  h  | 左臂位姿（7）  | 右臂位姿（7）  | 左夹爪 右夹爪
```

| 长度 | 内容 | 典型发布者 |
|---|---|---|
| 2 | `[左夹爪, 右夹爪]` | 抓取逻辑（手不动只开合） |
| 4 | `[vx, vy, wz, h]` | `teleop_keyboard.py`、下肢 VLA |
| 7 | 右臂位姿 `[x, y, z, qx, qy, qz, qw]` | 单臂遥操 |
| 14 | 双臂位姿（先左后右） | 双臂遥操 |
| 20 | 以上全部 | 全身 VLA |

**只覆写本次带来的字段，其余保持**，所以多个发布者可以同时存在且互不干扰。
不在 `{2, 4, 7, 14, 20}` 里的长度、含非有限值、四元数模长不在 `(0.5, 2.0)` —— **整帧丢弃**，
上一帧的目标继续生效。模长合法的四元数会先归一化再用。

> 订阅队列 depth = 4（不是 1）。**多发布者并存时 depth=1 会让同一个执行器周期内
> 后到的消息挤掉先到的**，下肢和上肢的指令互相吞。

### 输入：数据从哪来

| 话题 | 类型 | 频率 | 再上游 |
|---|---|---|---|
| `/pelvis_imu_broadcaster/imu` | `sensor_msgs/Imu` | 100 Hz | `G1TopicSystem` ← `/lowstate.imu_state`。陀螺仪原样透传（盆骨系）；四元数已从 Unitree 的 `(w,x,y,z)` 转成 ROS 的 `(x,y,z,w)` |
| `/joint_states` | `sensor_msgs/JointState` | 100 Hz | `G1TopicSystem::read()` ← `/lowstate.motor_state[i].q/dq`（500 Hz） |
| `~/command` | `std_msgs/Float64MultiArray` | 任意（建议 ≥10 Hz） | 键盘遥控台 / VR / VLA。超 `command_timeout_s`(0.5 s) 无更新则速度归零 |
| `/robot_description` | `std_msgs/String`（latched） | 一次 | 建 IK 缩减模型用 |

每一维观测怎么从这些话题装配出来，见第 4 节。注意 `/joint_states` 里是**全部 31 轴**
（29 本体 + 2 夹爪偏心轴），节点按 `policy_joints` 的名字取其中 15 个；名字对不上就等
下一帧，不会拿错位的数据去推理。

上肢的输入不进观测，直接驱动 IK：`~/command` 的臂位姿块是 `*_gripper_base` 相对
`torso_link` 的 `[x,y,z,qx,qy,qz,qw]`；夹爪块直接透传到槽位 29/30，只按 `gripper_limits` 裁剪。

上肢指令**不设超时**：没人发就保持上一个位姿目标。手臂停在原地才是安全行为，
归零或回零位都不是。

### 输出：结果送到哪

| 内容 | 话题 / 服务 | 类型 | 频率 | 消费者 |
|---|---|---|---|---|
| 31 轴位置目标 | `/forward_position_controller/commands` | `std_msgs/Float64MultiArray[31]` | 50 Hz | FPC。下肢 15 槽来自策略，上肢 14 槽来自 IK，夹爪 2 槽透传 |
| 层状态 | `/lower_body_policy/status` | `std_msgs/String`（JSON） | 10 Hz | 遥控台；要接 dashboard 也从这里取 |
| 使能 / 启动 / 急停 | `/lower_body_policy/engage`、`/start`、`/estop` | `std_srvs/Trigger` | 按需 | 遥控台或人工 `ros2 service call` |
| 控制器开关 | `/controller_manager/switch_controller` | `controller_manager_msgs/SwitchController` | 按需 | 本节点**调用**它，用来激活 / 反激活 FPC |

`~/status`（JSON）字段：`state / ready_to_start / command / request / reason / stale`，
加上上肢的 `ik_ready`、`ik_pos_err` / `ik_ori_err`（最近一帧两臂的最大残差，米 / 弧度）、
`ik_iters`、`ik_ms`、`grip`，以及 `pose`（当前生效的两个末端位姿目标）。现场排查先看这几项。

> `pose` 不只是给人看的：遥操源接管时直接拿它当原点，就**不必自己再建一份
> IK 模型算正解**。`vr_teleop` 走的就是这条路。

再往下游（不归本包管）：FPC 写 command interface → `G1TopicSystem::write()` 补上
`default_29dof_param.yaml` 的 kp/kd → `/lowcmd`（500 Hz，前 29 轴）与
`/grip_arm{0,1}/mit_command`（100 Hz，第 30/31 轴）。

---

## 2. 启动顺序
```bash
# 1.控制栈（会把 FPC 加载成 inactive，本层在 engage 时才激活它）
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual

# 2.策略层
ros2 launch g1_lower_body_policy lower_body_policy.launch.py
#   换策略试跑：  policy_path:=/abs/path/to/policy.onnx

# 3.遥控台（需要真终端，不要用 launch 包）
ros2 run g1_lower_body_policy teleop_keyboard
```

### 遥控台按键

遥控台**只发长度 4**，只管下半身；上肢保持在接管时的姿态不动。完整键位启动时会打印，
关键的三个：

| 键 | 作用 |
|---|---|
| `G` | **站立**：激活 FPC，3 s 内从当前实测位姿插值到策略默认位姿 |
| `Enter` | **启动策略**：站立插值走完后按 |
| `空格` | **急停**：反激活 FPC → kp 斜坡降到 0（阻尼）→ 最后一帧 kd 归零（零力矩） |

其余：`W`/`S`/`A`/`D` 走、`J`/`L` 转、`I`/`K` 升降、`X` 清零速度、`Q` 退出（自动急停）。

速度是"按住才走"：终端读不到抬键事件，所以超过 `hold_timeout_s`(0.4 s) 没按键就在
`decay_s`(0.5 s) 内衰减到零。高度不衰减——它是绝对量，调到哪停哪。

### VR 头显遥操（可选，代替键盘）

WebXR 采集页由 `vr_teleop` 节点**自己托管**（内嵌 aiohttp），头显直连节点。节点
**同时**听明文口和 TLS 口，两条接入路径并存：`https://<机器人IP>:8443`（自签证书，
不经过 adb）和 `http://localhost:8000`（需 `adb reverse`）。WebXR 只在安全上下文里
可用，所以只有这两种进法；`adb reverse` 的规则会**静默失效**，所以两条都开着。

```bash
ros2 run g1_lower_body_policy make_vr_cert     # 签自签证书，只需一次
ros2 launch g1_lower_body_policy vr_teleop.launch.py
```

完整上机流程、证书、排查清单见 [vr/README.md](vr/README.md)。已验证 Meta Quest 2/3、
PICO 4；手柄读的是 `xr-standard` 标准映射。

| VR 输入 | 机器人 |
|---|---|
| **左摇杆** | 水平速度 `vx` / `vy` |
| **右摇杆** | 转向 `wz`（左右）与盆骨高度 `h`（前后，绝对量，松手不回弹） |
| **squeeze** 按住 | 该侧手臂跟随手柄的位移**与转角**；松开即冻结在最后一帧 |
| **trigger** | 夹爪：0 = 完全打开，1 = 夹紧 |
| **A/X** | 把手柄此刻所指的水平方向标定为机器人正前方 |
| **双手同时 B/Y** | 推进状态机：站立 → 启动策略 → 急停（戴着头显摸不到终端） |

几个必须知道的点：

* **本节点只在策略层进入 `running` 之后才发指令**，接管的原点直接取策略层在
  `~/status` 里发布的 `pose`，两边的起点是同一个值。所以顺序是：先站立、再启动策略，
  才轮到 VR。
* **离合接合的瞬间位移与转角都恒为 0**（手柄位置、手柄姿态、末端位姿三个原点一起锁）。
  上肢不限速，这是唯一的防跳保护，已经锁进单测。
* **VR 帧超时（`frame_timeout_s` 0.3 s）或退出 VR 会话**：速度立刻归零、双臂冻结、
  夹爪保持。不会卸力——要卸力请用策略层的 `~/estop`。
* **刚进 `running` 时夹爪会弹到全开**。trigger 松开就是 0、对应完全打开，这是定义使然；
  手里有东西时先把 trigger 扣住再进 `running`。
* 坐标系：WebXR 右手系、Y 向上、-Z 朝前；机器人 X 前、Y 左、Z 上。手部位移在
  **标定后的世界系**里映射，不跟头显朝向转。

---

## 3. 状态机

照搬 `deploy/robots/g1` 官方 FSM 的 `Passive → FixStand → RLBase` 三段式，这是实机上
唯一被验证过的接管顺序。**策略绝不能从任意位姿冷启动**：训练里它只见过默认位姿附近
的开局。

```
IDLE ──~/engage──► STAND ──~/start──► RUNNING
  ▲                  │                   │
  └───────── ~/estop ┴───────────────────┘
```

| 状态 | 行为 |
|---|---|
| `IDLE` | 不发目标。FPC 处于 inactive |
| `STAND` | 激活 FPC，`stand_s`(3 s) 内 31 轴线性插值到「策略默认位姿 + `passive_targets`」，插完停在那儿等确认。对应官方 `State_FixStand` 的 `ts:[0,2]` + `qs`。**这一段 IK 还没接管** |
| `RUNNING` | 策略与手臂 IK **同时**接管。进入时清零 `last_action` 与步态相位（等价于官方 `env->reset()`），并用当前实测位形正解播种两个末端位姿目标 |
| `ESTOP` | 停止发目标 + 反激活 FPC。`G1TopicSystem::release_body()` 负责卸力斜坡：kp 在 `release_ramp_s` 内降到 0（kd 保留＝阻尼模式），最后一帧 kd 也归零（＝零力矩模式） |

分成两步（`engage` 再 `start`）而不是一步，是为了防终端按键自动重复：按住 `G` 会连发
十几次请求，一步式会直接把策略拉起来。

> `~/start` 在 IK 未就绪时会**拒绝**（返回“手臂 IK 未就绪，在等 /robot_description”）。
> 手臂始终开启，没有降级成“只跑下肢”的静默路径 —— 宁可拒绝接管，不带病启动。

### 自动急停条件

| 触发 | 阈值 | 说明 |
|---|---|---|
| `/joint_states` 或 IMU 超时 | `state_timeout_s` 0.1 s | 广播是 100 Hz |
| 姿态倾覆 | `tilt_limit_rad` 1.0 rad | 与官方 `mdp::bad_orientation` 默认阈值一致（官方那份被注释掉了，这里是打开的） |
| 推理抛异常 / 输出非有限值 | — | 观测里出 NaN 也在这里被拦 |

指令超时（`command_timeout_s` 0.5 s）只把速度归零、保持高度，**不急停**——遥控手松手
不该让机器人卸力。

**上肢的任何问题都不急停**：够不着、不收敛、求解抛异常，都只保持上一帧手臂目标并打
节流告警。正在平衡的下肢不该被上肢拖下水。

---

## 4. 下肢观测与动作契约

这一节是整个部署里最要命的部分：错一位就是错一个策略。层内的 `policy_runtime.py` 把关节顺序、默认位姿、动作缩放**全部从 ONNX 的 metadata 里读**，不在代码里抄第二份；启动时再和 `config/lower_body_policy.yaml` 的 `policy_joints` 逐个比对，对不上直接拒绝启动。

### 观测（57 维，顺序即拼接顺序）

| # | 项 | 维 | 实机来源 | 训练侧对应 |
|---|---|---|---|---|
| 0 | `base_ang_vel` | 3 | `/pelvis_imu_broadcaster/imu.angular_velocity` | `robot/imu_ang_vel`（gyro@`imu_in_pelvis`） |
| 1 | `projected_gravity` | 3 | 同上的 `orientation`，`R(q)ᵀ·[0,0,-1]` | `projected_gravity_b` |
| 2 | `command_twist` | 3 | `[vx, vy, wz]` | `generated_commands("twist")` |
| 3 | `command_height` | 1 | `[h]` | `generated_commands("height")` |
| 4 | `phase` | 2 | `[sin, cos]`，`period=0.6 s`；`‖[vx,vy,wz]‖<0.1` 时置零 | `mdp.phase` |
| 5 | `joint_pos` | 15 | `q_meas - q_default` | `joint_pos_rel` |
| 6 | `joint_vel` | 15 | `dq_meas` | `joint_vel_rel` |
| 7 | `actions` | 15 | 上一拍策略**原始**（未裁剪）输出 | `last_action` |

归一化已经在导出时折进 ONNX 图里（`Sub`/`Div`），**不要**在外面再减均值。

### 动作（15 维）

```
q_target = default_joint_pos + action_scale * action
q_target = clip(q_target, target_lower_limits, target_upper_limits)   # 见下，不是关节行程
```

**目标位置故意不按关节行程裁剪。** 底层是 PD：`tau = kp*(q_target - q) - kd*dq`，关节靠到
行程边上还想要力，就只能把目标顶到行程之外。mjlab 建 `<position>` 执行器时显式设了
`ctrllimited=False` + `inheritrange=0`，源码注释原文：

> *clamping ctrl to the joint range would produce zero force when the joint is at its limit*

用 CPU MuJoCo 闭环重跑（站立 / 0.5、1.0 m/s 前进 / 蹲行 / 原地转），目标位置越出硬
`jnt_range` 的比例是 **2%~18.6%**，越界最多的正是 `waist_roll`、`ankle_roll`、
`ankle_pitch`——撑平衡最吃力的那几个。`waist_roll` 的目标能到 −0.763 rad，而行程只有
±0.520。**按行程裁剪等于把这几个关节在最需要出力的时刻掉掉。**

实际裁的是 MuJoCo 算的 informational `ctrlrange`（`jnt_range ± effort_limit / stiffness`）：
越过它力矩已饱和，裁在这儿物理上是空操作（同样五个场景越界率 **0%**），纯粹用来拦跑飞的输出。

关节顺序正好是真机电机索引 0..14：

```
 0..5   left  hip_pitch / hip_roll / hip_yaw / knee / ankle_pitch / ankle_roll
 6..11  right 同上
12..14  waist_yaw / waist_roll / waist_pitch
```

`action_scale = 0.25 × effort_limit / stiffness`：

| 关节 | scale | 关节 | scale |
|---|---|---|---|
| hip_pitch / hip_yaw / waist_yaw | 0.548 | hip_roll / knee | 0.351 |
| ankle_pitch / ankle_roll / waist_roll / waist_pitch | 0.439 | | |

### PD 增益与频率

kp/kd 就是 `unitree_g1_ros2_control/config/default_29dof_param.yaml`（ONNX metadata 里的
`joint_stiffness/joint_damping` 与之逐项相同），硬件侧从同一个文件加载，**两边天然一致**。

训练是 sim dt 0.005 × decimation 4 = **50 Hz**，实机必须一致——这个频率同时决定步态
相位的推进速度。单拍推理实测 **0.084 ms**，占 20 ms 预算的 0.4%。

---

## 5. 上肢 IK

### 模型：锁掉腰和腿的 14 轴缩减模型

`/robot_description` 到达后建一次，用 `pin.buildReducedModel` 把 14 个手臂关节以外的**全部**
关节锁死，得到 `nq == 14` 的模型。两个直接好处：

* **求解规模就是 14**，雅可比 6×7、线性方程 6×6，没有整机 95 维的开销。
* **`torso_link` 在模型世界系里是个常量位姿**，构造时算一次 `oMb` 就够。求解时完全
  不必关心策略把腰摆到了哪儿 —— 这正是参考系选 `torso_link` 而不是 `pelvis` 的理由。

建模要几百毫秒，放在 `ReentrantCallbackGroup` 里跑，避开控制环所在的互斥组。
`ArmIK.joint_names` 由缩减模型自己报出，节点按它反查 31 轴槽位，不假设“左 7 + 右 7
正好是 15..28”。

### 求解：定阻尼 DLS + 零空间姿态偏置，无线搜索、无 SVD、迭代数硬上限

每次迭代（误差与雅可比均在 `LOCAL_WORLD_ALIGNED`）：

```
e   = [p_target - p_cur,  log3(R_target · R_curᵀ)]     # 6 维
J   = 该末端的 6×7 雅可比
dq  = Jᵀ(J Jᵀ + λ²I)⁻¹ e  +  [I - Jᵀ(J Jᵀ + ε²I)⁻¹J] · w · (q_rest - q)
      └──── 任务步 ────┘    └──── 零空间投影 ────┘   └─ 姿态偏置 ─┘
q  ← clip(q + dq, URDF 下限, URDF 上限)
```

加了 `λ²I` 之后矩阵恒正定，`np.linalg.solve` 不会奇异，所以**不需要 SVD、不需要奇异值
判据、也不需要线搜索**。

#### 为什么必须有零空间项

手臂 7 自由度、末端任务只有 6 维，多出来的那一维**不管就会漂**。实测：让右手画一个
6 cm 的圆（每帧才走 1.9 mm），纯最小范数 DLS 会让 `shoulder_yaw` 一路漂到 **-1.23 rad**，
然后在第 196 帧被迫重构、**单帧跳 0.23 rad**。上肢不限速，这一下就是电机全权限的弹动。

加上 `w=0.05` 的零空间偏置后：单帧最大跳变 0.234 → **0.073 rad**，绕回原点后的 yaw
漂移 −1.228 → **+0.049 rad**，不可达目标下的残留抖动 1.4e-2 → **1.1e-7 rad**（肉眼可见
的嗡动消失），而残差 p50 **不变**（0.45 mm）——它在零空间里，不影响末端精度。

提高迭代上限治不了这个问题（60 次迭代仍然跳 0.247 rad），降阻尼只会更糟
（阻尼 0.01 时跳 1.79 rad）——它不是精度问题，是冗余自由度没人管的问题。

> ⚠️ **投影算子的阻尼 `ε²` 必须和任务步的 `λ²` 分开取值**（代码里是 `_NULL_DAMPING`）。
> 本机手臂雅可比的最小奇异值约 0.083（σ²≈0.0069），和默认阻尼 0.05²=0.0025 同量级；
> 两者用同一个值时投影算子会严重泄漏到任务空间，姿态偏置变成和末端目标抢：残差从
> 0.45 恶化到 0.91 mm，且 **31% 的帧永远达不到收敛判据**、白白跑满迭代。也不能取得更小：
> ε²=1e-9 时投影几乎精确、近奇异方向病态，跳变反而恶化到 0.295 rad。1e-4 是实测拐点。

#### 收敛判据按侧算

已经到位的那一侧当轮**完全不碰**。否则它的任务项虽然是 0，零空间偏置却照样推着它的肘部走
—— 只发右臂指令时左臂会悄悄改位形，违反第 1 节“只覆写本次字段”的契约。

**不插值、不限速**；**够不着就尽力而为**（到迭代上限直接返回当前值，不抛异常）。

### 种子与连续性

`solve()` 的种子是**上一帧已发布的手臂目标**，不是实测值——手臂被卡住时，用实测值
做种子会产生积分饱和式的追赶。

`~/start` 时用当前实测位形做正解播种两个末端位姿目标，所以**没人发上肢指令时手臂
就停在接管那一刻的姿态**。这个正解放在**控制线程**里做（`~/start` 只置一个标志）：
`ArmIK` 内部有 pinocchio 的 `data` 缓存，只让一个线程碰它就不需要额外加锁。

### 关节限位

直接取 URDF 的 `<limit lower/upper>`，不额外收紧。

> 这和下肢的 `target_lower/upper_limits` 是**两回事**。下肢故意裁在 MuJoCo 的
> `ctrlrange` 而不是行程（原因见第 4 节）；上肢没有那个诉求。

### 实测性能（aarch64，50 Hz = 20 ms 预算）

| 场景 | 迭代 | 双臂耗时 | 占预算 |
|---|---|---|---|
| 稳态（目标不动） | 0 | 0.11 ms | 0.6% |
| 跟随（末端 0.1 m/s，每帧 2 mm） | 1 | 0.38 ms | 1.9% |
| 冷启动（从零位解到 0.3 rad 位形） | 5 | 1.36 ms | 6.8% |
| **最坏（双侧不可达，跑满上限）** | 10 | **2.52 ms** | **12.6%** |

位置残差：厘米级小步跟随 p50 = 0.45 mm、max = 1.0 mm。上限由 `ik_max_iters` 硬保证，
不存在“某一帧突然算很久”的可能。

---

## 6. 参数速查（`config/lower_body_policy.yaml`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `policy_path` | `package://g1_lower_body_policy/config/policy.onnx` | 换策略只要覆盖这个文件 |
| `policy_joints` | 15 个下肢关节 | 必须与 ONNX metadata 的前 15 项完全一致 |
| `passive_targets` | 16 × 0.0 | 14 臂 + 2 夹爪偏心轴。**只在 STAND 阶段生效**；进 RUNNING 后上肢改由 IK 与透传接管。见下方风险提示 |
| `target_lower/upper_limits` | MuJoCo `ctrlrange` | **不是关节行程**，见第 4 节；物理上是空操作，只拦跑飞 |
| `command_limits` | `vx[-0.3,0.5] vy[-0.3,0.3] wz[-0.5,0.5] h[0.62,0.76]` | **首次上机的保守子集**，训练分布见下 |
| `height_rate_limit` | 0.15 m/s | 不是"可选的平滑"，见下方 |
| `linear/angular_accel_limit` | 1.5 / 3.0 | 纯安全项，训练里速度指令是允许阶跃的 |
| `initial_height` | 0.74 | 进入 `RUNNING` 时的高度指令 |
| `stand_s` | 3.0 | 官方 `FixStand` 是 2 s，这里多 1 s 是因为还要把手臂收到 `passive_targets` |
| `control_rate_hz` | 50.0 | **不要改**，会改变步态相位速度。上肢 IK 共用这一个定时器 |
| `tilt_limit_rad` | 1.0 | 同官方 `bad_orientation` |

### 上肢参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `arm_joints` | 14 个手臂关节 | 先后顺序不重要：真正的 q 顺序由缩减模型自己定 |
| `gripper_joints` | `[left_eccentric_joint, right_eccentric_joint]` | 不进 IK 模型，直接透传 |
| `gripper_limits` | `[0.0, 2.76377]` | 取自 URDF 的 `<limit>`；透传前只做这一道裁剪 |
| `base_frame` | `torso_link` | 末端位姿的参考系。选它而不是 `pelvis`，IK 链里就没有腰 |
| `left/right_tip_frame` | `*_gripper_base` | 求解的末端帧，帧名不存在会在建模时报错并列出可用帧 |
| `ik_max_iters` | 10 | **硬上限**。它决定了最坏耗时（2.52 ms） |
| `ik_damping` | 0.05 | 任务步 DLS 的 λ。调大更稳但收敛慢；调小会放大近奇异处的跳变（0.01 时实测跳 1.79 rad） |
| `ik_posture_weight` | 0.05 | 零空间姿态偏置强度。**不能设 0**，否则冗余自由度会漂移（见上）；设到 0.2 会把残差抬到 2.2 mm |
| `ik_rest_posture` | elbow 0.5 / shoulder_roll ±0.25 | 参考位形，顺序同 `arm_joints`。同时兼顾避开大腿 |
| `ik_tol_pos` / `ik_tol_ori` | 0.001 m / 0.0035 rad | **按侧**判断，两者都满足那一侧当轮就不再动 |
| `robot_description_topic` | `/robot_description` | latched（TRANSIENT_LOCAL），正常只进来一次 |

`joints`（FPC 的 31 轴顺序）由 launch 从
`unitree_g1_ros2_control/config/forward_position_controller.yaml` 读进来后注入，本包
不抄第二份——两份不同步就是左右腿指令互换级别的事故。

### 训练分布（调稳之后可以逐档放开到这里）

| 量 | play 模式训练分布 | 当前实机默认 |
|---|---|---|
| `vx` | [-0.5, 1.0] | [-0.3, 0.5] |
| `vy` | [-0.5, 0.5] | [-0.3, 0.3] |
| `wz` | [-0.5, 0.5] | [-0.5, 0.5] |
| `h` | [0.55, 0.76] | [0.62, 0.76] |

### 为什么高度必须限速

训练里高度指令由 `BaseHeightCommandCfg.max_rate = 0.15 m/s` 缓变，**策略从没见过高度
阶跃**。遥控每按一下 `↑` 是 1 cm 的阶跃，不限速就是分布外输入。速度指令则相反：训练
里每 3~8 s 重采样一次是真阶跃，所以那两档限速纯粹是安全冗余，可以放宽。

---

## 7. 已知风险与取舍

### 手臂在 STAND 阶段写 0 与自碰撞

`passive_targets` 默认全 0，**它只决定 STAND 那 3 秒手臂走到哪里**；进 RUNNING 后上肢
改由 IK 接管，起点就是 STAND 结束时的那个姿态。但训练时手臂是被随机摆在
`elbow ∈ [0.3, 1.6]`、`shoulder_roll ∈ ±[0.25, 1.20]` 的——这个下界是量出来的，
目的就是**避开细长的 Gloria-M 夹爪撞大腿**（`elbow` 下界 0.0 时随机位姿自碰撞率 11%，
0.3 时降到 7.5%）。`elbow=0` 且 `shoulder_roll=0` 落在那个区间之外。

站立那一步务必目视确认夹爪没蹭到大腿。CPU 闭环重跑里手臂写 0 并**没有**让下肢站不住，
所以主要风险是几何干涉而不是平衡。要更保险就把 `passive_targets` 换成（顺序是左臂 7 轴
→ 右臂 7 轴 → 左右夹爪偏心轴）：

```yaml
passive_targets: [0.0, 0.25, 0.0, 0.5, 0.0, 0.0, 0.0,
                  0.0, -0.25, 0.0, 0.5, 0.0, 0.0, 0.0,
                  0.0, 0.0]
```

### 策略当前水平

训练还在跑（写这份文档时第 5500/8800 轮），每 100 轮重新导出一次，直接覆盖
`config/policy.onnx` 就能换新策略。

**包里装的不是最新的那个，是 `2026-07-30_07-09-08/`（约第 2800 轮）**，因为最新的
那个前进速度跟随已经塌掉了。两版都是正常的交替步态、8/8 不摔，差别全在**前进速度**：

| 前进指令 | iter≈2800（**包里这个**） | iter≈5500（最新） |
|---|---|---|
| 0.5 m/s | **+0.35** m/s（70%） | −0.00 m/s（0%） |
| 1.0 m/s | **+0.67** m/s（67%） | −0.05 m/s（0%） |
| 蹲行 0.3 m/s | **+0.18** m/s（60%） | +0.03 m/s（10%） |

训练日志里 `track_linear_velocity` 从第 2950 轮的 0.81 掉到第 5650 轮的 0.27：上肢扰动
课程拉到满档后，策略拿前进速度换了抗扰动。

**已知不足**（两个版本都有）：

* 横移和转向的跟随增益很低（约 30% 和 15%），实机上表现为"横着挪不动、转得慢"。
* 策略**没有航向反馈**。`wz=0` 是"不要主动转"，不是"保持航向"；实测 12 s 直行会侧偏
  2.6 m。航向环得由操作员或 VLA 在外面闭。

### 没做的事

* 没有 `/cmd_vel`(Twist) 接口。VLA 直接向 `~/command` 发对应长度的 `Float64MultiArray`
  即可，节点内部的限幅、限速、超时保护对任何指令源一视同仁。
* **上肢没有限速、插值、轨迹规划**。平滑性由发布者自己负责，目标位姿要连续变化。
* **没有自碰撞检测**。IK 只管关节限位，不知道两只手会不会撞到一起、会不会撞腿。
* **没有臂角（arm-angle）冗余优化**，不能指定肘部朝向。
* 只在平地策略上验证过。

---

## 8. 故障排查

| 现象 | 原因 |
|---|---|
| 启动即抛 `ONNX 的关节顺序和配置不一致` | 换了策略但 `policy_joints` 没跟着改。**不要绕过这个检查**，它拦的正是左右腿互换 |
| 启动即抛 `观测项不匹配` | 这个 ONNX 不是下肢任务导出的（metadata 里 `observation_names` 对不上） |
| 启动即抛 `arm_joints + gripper_joints 必须正好是策略之外的全部关节` | yaml 里这两项的并集和 `joints − policy_joints` 对不上。拦的是“漏了一轴没人写” |
| `~/start` 返回 `手臂 IK 未就绪` | 没收到 `/robot_description`。确认控制栈起了：`ros2 topic echo /robot_description --once`；或看日志里有没有 `手臂 IK 建模失败` |
| 日志报 `URDF 缺少坐标帧 *_gripper_base` | `/robot_description` 里没有夹爪链（起控制栈时漏了 Gloria-M），或把 `left/right_tip_frame` 改成了不存在的帧 |
| 上肢没反应，日志刷 `丢弃非法指令：长度 N 不在 [2, 4, 7, 14, 20] 里` | 发布者的数组长度不对。参照第 1 节的分块表 |
| 上肢没反应，日志刷 `四元数模长 … 异常` | 发了全零四元数。注意顺序是 **xyzw**，单位四元数是 `[0,0,0,1]` |
| 手臂到不了位，`ik_pos_err` 一直很大 | 目标不可达或被 URDF 限位挡住。这是设计行为（尽力而为），不会急停；看 `ik_iters` 是不是顶到了 `ik_max_iters` |
| `~/engage` 返回 `/joint_states 超时` | 控制栈没起，或 `scope:=whole_body` 忘了写 |
| `~/engage` 返回 `switch_controller 拒绝激活` | FPC 被别的东西占着（IKT Pose Commander / JTC / dashboard），先把它们停掉 |
| `~/start` 返回 `站立插值还没走完` | 等满 `stand_s` 再按 `Enter` |
| 跑着跑着莫名急停、原因是 `/joint_states 超时` | Jetson 上负载高时广播确实会抖。把 `state_timeout_s` 从 0.1 提到 0.2——仍然小于硬件自己的 `state_timeout_s`(0.25 s)，不会削弱保护 |
| 急停后机器人还硬着 | 看日志有没有 `卸力失败`。有的话立刻用手柄断电——这条路径失效说明 controller_manager 没响应 |
| 站立位姿正常但一放策略就抖 | 先确认 `/joint_states` 的 `velocity` 字段非空（观测第 6 项全 0 会让策略瞎跑）：`ros2 topic echo /joint_states --field velocity --once` |

---

## 9. 文件导航

```
g1_lower_body_policy/
├── config/
│   ├── lower_body_policy.yaml   # 节点参数，每一项都有注释
│   └── policy.onnx              # 策略权重（自带元数据：关节顺序/默认位姿/动作缩放）
├── g1_lower_body_policy/
│   ├── policy_runtime.py        # 下肢：观测装配 + 推理 + 契约校验，不依赖 ROS
│   ├── arm_ik.py                # 上肢：缩减模型 + DLS 求解，不依赖 ROS
│   ├── policy_node.py           # ROS 节点 + 状态机 + 看门狗 + 指令分块
│   ├── teleop_keyboard.py       # 键盘遥控台（只发长度 4，只管下半身）
│   ├── vr_teleop.py             # VR 桥接（内嵌 WebXR 服务，发长度 20，上下肢全管）
│   └── make_vr_cert.py          # 签局域网自签证书，兼作证书路径/端口的唯一定义
├── launch/
│   ├── lower_body_policy.launch.py
│   └── vr_teleop.launch.py
├── vr/                          # WebXR 采集页 + 监控页（装到 share/）+ adb 守护脚本
│   └── README.md                # VR 上机流程、证书、排查清单
└── test/
    ├── test_policy_runtime.py   # 13 个纯逻辑用例，pytest
    ├── test_arm_ik.py           # 8 个 IK 用例（正逆往返 / 不可达 / 单侧求解 / 零空间不漂移）
    ├── test_vr_teleop.py        # 22 个 VR 用例（离合零跳变 / 轴映射 / 标定 / 摇杆 / 夹爪方向）
    └── smoke_no_robot.py        # 无真机联调，32 项检查，直接 python3 跑
```

> 数据（`config/*`、`vr/*.html`）走 `data_files` 装到 `share/`，运行时用
> `get_package_share_directory` 解析；代码在模块目录里。**别把页面挪进模块目录**——
> 那样 `Path(__file__).parent` 在 `--symlink-install` 下能跑通、在干净安装下会 404，
> 是个只在部署机上暴露的坑。

### 上机前的自检

```bash
# 假状态源 + 假 controller_manager，把状态机从头走一遍，32 项检查
python3 src/g1_lower_body_policy/test/smoke_no_robot.py

# 纯逻辑单测 43 项
cd src/g1_lower_body_policy && PYTHONPATH=$PWD python3 -m pytest test/ -q
```

换了策略还要在训练仓库里跑 `scripts/check_deploy_policy.py`（CPU 闭环重跑整条部署
链路，不占 GPU）。跑完确认没有残留进程：上一轮的节点会在同一个 `ROS_DOMAIN_ID` 上
继续发 `status`，下一轮会读到旧状态。
