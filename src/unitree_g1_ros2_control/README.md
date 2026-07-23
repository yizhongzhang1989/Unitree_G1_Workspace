# unitree_g1_ros2_control
`unitree_g1_ros2_control` 将 Unitree G1、双 Gloria-M、双 KWR57 和 pelvis IMU 接入 ROS 2 Foxy 原生 `ros2_control`。包内提供一个统一 `SystemInterface`、一个自定义 C++ forward-position controller、一个标准 JointTrajectoryController 实例和两个节流 broadcaster；不创建额外通信节点或 controller-manager facade。

## 资源与数据流
`unitree_g1_ros2_control/G1TopicSystem` 导出：

| 资源 | 数量 | 输入/输出 |
|---|---:|---|
| 关节 position command | 31 | G1 `/lowcmd`，双 Gloria-M `~/mit_command` |
| 关节 position/velocity/effort state | 93 | `/lowstate` 与双 Gloria-M `JointState` |
| 双 FT state | 12 | 左右 KWR57 原始 `WrenchStamped` |
| pelvis IMU state | 10 | `/lowstate.imu_state` |

manager 以 500 Hz 调用 `read()`/`write()`。G1 命令直接从 `write()` 发布；Gloria-M 在同一路径内用 steady clock 固定相位 deadline 降采样到 100 Hz。若一次 `write()` 错过一个或多个时隙，deadline 直接前移到下一个未来时隙，不补发过期命令，也不按“当前时刻 + 10 ms”累积漂移。KWR57 raw 保持设备节点原有 1 kHz 话题，插件用每侧原子快照读取，不增加转发节点。FT 数值按 `9.80665` 从 kgf/kgf m 转为 SI；Unitree 四元数从 `w,x,y,z` 转为 ROS `x,y,z,w` 并归一化。

G1 增益表保持物理电机顺序不变。启动参数 `arm_stiffness_scale` 只缩放双臂 15–28 号关节的 `kp`，默认 `2.5`（肩肘约 `35.8`、腕部约 `42.0`），腿、腰和全部 `kd` 不变；该值接近 Unitree 低层双臂示例常用的 `kp=40`。需要回退原始增益时传入 `arm_stiffness_scale:=1.0`，允许范围为 `(0, 4]`。

硬件导出的 31 个 command interface 由 `forward_position_controller`（FPC）或 `joint_trajectory_controller`（JTC）互斥 claim。ros2_control 的 claim 只提供命令资源互斥，不检查反馈是否新鲜；feedback freshness 是 `G1TopicSystem` 自己实现的安全策略。G1 使用 `state_timeout_s=0.25 s`，Gloria 使用独立的 `gripper_state_timeout_s=0.75 s`。单侧夹爪 stale 时只跳过该侧 MIT 输出，G1 LowCmd 和另一侧不受影响；反馈恢复后该侧自然恢复。

启动反馈到达前，对外 joint state 使用有限零值，IMU 使用单位四元数，避免 `robot_state_publisher` 产生 NaN TF。控制安全仍由独立的 `received` 标志和 freshness 检查决定，中性启动值不能使 controller 通过 Engage。

### 进程与通信边界
`controller_manager`、`ForwardCommandController`、`JointTrajectoryController` 和 `G1TopicSystem` 都在同一个 `ros2_control_node` 进程中。每个 500 Hz 周期按“硬件 `read()` -> active controller `update()` -> 硬件 `write()`”运行，state/command interface 是指向插件存储的 C++ 接口；controller 与硬件插件之间没有 ROS topic、序列化或 DDS。

`G1TopicSystem` 内部用于 `/lowstate`、双 Gloria、双 KWR57、MotionSwitcher 和服务客户端的节点由 `SingleThreadedExecutor` 驱动。回调只更新缓存或完成事务，500 Hz manager 循环继续通过硬件接口读写；这里不增加并发 callback worker，避免在 PC2 上为高频订阅引入额外调度竞争。

设备驱动边界保持不变：

- G1：硬件插件内部生成 `LowCmd` 和 CRC，再发布 Unitree 官方 `/lowcmd`；反馈订阅 `/lowstate`。
- Gloria-M：硬件插件发布既有 `gloria_ros/msg/MitCommand`，独立 `gloria_ros` 节点继续负责模式、量程、安全检查和 CAN 编码；反馈仍用其 `JointState`。
- KWR57：CAN 三帧协议由 `canalystii_native_bridge` 在 C++ 进程内解析；硬件插件直接订阅既有 raw `WrenchStamped`，不会创建中间节点或再次发布 raw Wrench。

所以 ros2_control 的 controller-to-hardware 路径本身不增加 DDS hop。外部 Dashboard/IK 通过 FPC commands 或 JTC action 进入当前 active controller；这与旧 Python facade 的命令输入边界等价。Gloria 保留两段已有的 ROS 设备边界，KWR57 只增加 ros2_control 作为 raw Wrench 的一个订阅者。默认不启动 FT broadcaster，避免把 1 kHz 状态再发布一次。

2026-07-23 的四设备 30 秒实机验收中，左右 MIT 命令为 `100.000/100.000 Hz`，bridge 实际 CAN TX 为 `99.999/100.001 Hz`；同场景双 KWR57 source 最大 gap 为 `6.860/7.322 ms`，ROS receive 最大 gap 为 `7.027/7.433 ms`。完整配置、USB 空包根因和测试边界见 [canalystii_native_bridge/README.md](../canalystii_native_bridge/README.md)。

## 启动
推荐生产启动，一条命令同时启动末端设备与唯一控制栈：
```bash
source scripts/env.sh
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual
```

另开终端按需启动只作为客户端的 Dashboard：
```bash
source scripts/env.sh
ros2 launch robot_bringup whole_body_dashboard.launch.py
```

`all_data scope:=whole_body` 已经 include 本包的 `control.launch.py`。不要再启动第二个 manager，也不要重复启动 CAN bridge 或设备节点。

仅当外部已经启动匹配拓扑的 Gloria-M、KWR57 和 CAN bridge 时，才独立启动本包：

```bash
source scripts/env.sh
ros2 launch unitree_g1_ros2_control control.launch.py topology:=dual
```

这个独立入口只创建 manager、硬件插件、broadcaster、RSP 和 inactive controllers，不打开 CAN 或创建设备驱动。

启动结果：

- `/controller_manager`：唯一真实 manager，500 Hz；
- `joint_state_broadcaster`：active，默认 100 Hz；
- `pelvis_imu_broadcaster`：active，默认 100 Hz；
- `forward_position_controller`：已配置但 inactive；
- `joint_trajectory_controller`：已配置但 inactive；
- `left_ft_broadcaster`、`right_ft_broadcaster`：已注册但默认不启动。

`forward_position_controller` 类型为 `unitree_g1_forward_command_controller/ForwardCommandController`，命令话题为 `/forward_position_controller/commands`，消息类型为 `std_msgs/msg/Float64MultiArray`。`joint_trajectory_controller` 类型为 Foxy 标准 `joint_trajectory_controller/JointTrajectoryController`，动作接口为 `/joint_trajectory_controller/follow_joint_trajectory`。两者的 31 个 `joints` 顺序完全相同，并请求同一组 position command interface；controller_manager 的 resource claim 保证它们互斥 active。两者都只写硬件 position interface，G1 和 Gloria-M 的 MIT 帧仍统一由 `G1TopicSystem::write()` 产生。

FPC 的流式命令订阅使用 BEST_EFFORT、`KEEP_LAST(1)`，实时缓冲也只暴露最新样本；短暂调度繁忙后不会依次执行过时 setpoint。可靠发布者仍可与该订阅匹配，G1 Pose Commander 则直接使用相同的 BEST_EFFORT latest-only 配置。

`robot_test_dashboard` 中的 JTC/FPC 代码只是测试命令生成器，不应搬入本包。JTC 的插值、目标容差和 action 状态机已经由标准插件实现；重复实现会产生第二套语义。Cartesian IK 同样保持在 `ikt_core`/Pose Commander 算法层，它输出关节目标但不拥有 hardware interface。只有将来确实需要硬实时 Cartesian servo 时，才应新增独立 C++ ros2_control controller 包，并复用这里的 position interfaces，而不是把 Python IK 或网页逻辑放进硬件插件。

启动后先检查，不要直接 Engage：

```bash
ros2 control list_controllers --controller-manager /controller_manager
ros2 control list_hardware_interfaces --controller-manager /controller_manager
```

预期两个控制器都为 `inactive`，31 个 position command interface 全部 `unclaimed`；任意激活其中一个后 31 个接口都被 claim，另一个必须保持 `inactive`。

## 安全切换

controller inactive 时不 claim command interface。FPC/JTC 切换由 `controller_manager/switch_controller` 在一个请求中一停一启；二者 claim 集相同，manager 不允许同时 active。第一次从全 inactive 状态 Engage 时，硬件插件按顺序执行：

1. 检查 29 轴 G1 反馈未超过 `state_timeout_s`，两侧 Gloria 反馈未超过 `gripper_state_timeout_s`；本体还要求 `mode_pr == 0`。
2. 使用 MotionSwitcher API 1001/1003 检查并释放现有运动模式。
3. 等待外部 `/lowcmd` 连续静默，避免双 publisher 同时控制本体。
4. 调用两侧 Gloria `enable` 服务。
5. 再次检查反馈 freshness；成功后才允许硬件 `write()` 输出。

任一步失败都会关闭输出、失能相应夹爪并尝试恢复原运动模式。Disengage 先关闭输出，再调用夹爪 `disable`，最后用 MotionSwitcher API 1002 恢复 Engage 前记录的模式；若记录为空则使用 `fallback_motion_mode`。

自定义 FPC 还执行三层命令保护：

- 首条外部命令与当前反馈的最大差值不得超过 `max_initial_position_error`；
- 后续相邻目标最大跳变不得超过 `max_command_step`；
- 超过 `command_timeout_s` 未收到新目标时，一次性锁存并保持当下反馈姿态。

三类超时相互独立：

| 参数 | 默认值 | 所属层 | 动作 |
|---|---:|---|---|
| `command_timeout_s` | `0.25 s` | C++ controller | Dashboard/IK 目标流超时后一次性锁存反馈姿态，不失能硬件 |
| `state_timeout_s` | `0.25 s` | `G1TopicSystem` | G1 `/lowstate` stale 时停止本体 LowCmd |
| `gripper_state_timeout_s` | `0.75 s` | `G1TopicSystem` | 单侧 Gloria stale 时跳过该侧 MIT，反馈恢复后继续 |
| `feedback_timeout_s` | `0.5 s` | `gloria_ros` 驱动 | 驱动自身发送 disable 并阻断夹爪命令 |

## 检查

```bash
ros2 control list_controllers --controller-manager /controller_manager
ros2 control list_hardware_interfaces --controller-manager /controller_manager
ros2 topic hz /joint_states
ros2 topic hz /pelvis_imu_broadcaster/imu
```

未 Engage 时，31 个 position command interface 应全部显示 `unclaimed`；Engage 后 31 个接口应全部为 `claimed`。标准 FT 输出如有需要可按侧手动启动：

```bash
ros2 run controller_manager spawner.py left_ft_broadcaster \
  --param-file install/unitree_g1_ros2_control/share/unitree_g1_ros2_control/config/left_ft_broadcaster.yaml \
  --controller-manager /controller_manager
```

不要在相同 manager 路径启动第二套控制栈，也不要在机器人未可靠支撑、现场不可急停时 Engage。Dashboard 入口 `robot_bringup/whole_body_dashboard.launch.py` 只连接此 manager，不负责启动它。