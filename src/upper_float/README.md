# Upper Float

独立 TaskBase 上肢任务申请14臂关节与2个夹爪。`engage` 后由基类完成准备，`prepared(snapshot)` 开放上肢执行；运行时实测14臂角直接发布到 FPC，使现有重力补偿与阻尼承担浮动控制。腰腿由其它任务或 session 默认目标管理。

## 启动

在工作区根目录使用 [robot_bringup 集中入口](../robot_bringup/README.md)，硬件显式另起。每个终端先 `source scripts/env.sh`，已加载 Humble 与所需 SDK 环境时也可 `source install/setup.bash`：

```bash
# 终端 A，已有硬件栈时复用
source scripts/env.sh
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual
```

另一终端按需选择一条入口：

```bash
source scripts/env.sh

# 默认键盘组合：session + 下肢速度 + 上肢浮动 + 键盘
ros2 run robot_bringup teleop_keyboard

# 仅上肢浮动组合：session + 上肢浮动，lower_task:=none
ros2 launch robot_bringup motion_tasks.launch.py lower_task:=none upper_task:=float
```

默认键盘入口保留真实下肢速度控制；独立上肢组合将腰腿交给 session 默认目标。两套组合各只含一个 session，不启动硬件、不自动接管；操作者显式 `engage` 后，本任务在准备完成回调中进入 `RUNNING`，下肢速度任务仍需会话 `start`。默认键盘入口可用 `--upper-config` 覆盖浮动配置，独立组合可用 `upper_config:=...`。

只选择一套包含 session 的方案。手动拆分时，在硬件与唯一 g1_motion_session 已启动后，只启动上肢任务：

```bash
ros2 launch upper_float float.launch.py
```

需要控制已有任务时，另开终端执行 `ros2 run lower_velocity_tracking teleop_keyboard`，它是 control-only，不启动第二套任务、session 或硬件。正式组合键盘退出先请求急停，再向自身启动的任务进程组发送 SIGINT，既有硬件进程保持运行。

浮动效果依赖硬件栈已有的手臂重力补偿与增益配置。夹爪默认消费 `/motion_control/upper/input/keyboard` 的 `std_msgs/msg/Float64MultiArray`，`data` 为两个弧度值 `[left_rad, right_rad]`，保留最后生效请求与原行程裁剪。`gripper_source:=vr` 将默认话题切为 `/motion_control/upper/input/vr`；`gripper_command_topic:=...` 可显式覆盖话题，消息类型始终为该数组。准备期间夹爪随预备目标，运行时双臂关节持续跟随实测。

## 接口

- FPC：`/forward_position_controller/joint_targets`，掩码仅15至30槽位。
- 生命周期：`/motion_control/register`、`/motion_control/events`、`/motion_control/tasks`。
- 反馈：`/joint_states`，不依赖 IMU 或 IK。
- 夹爪任务输入：`std_msgs/msg/Float64MultiArray.data`，默认左右顺序对应 `gripper_joints`，单位为弧度。仅接受长度2且全部有限的数组，按 `gripper_limits` 裁剪；空数组、其它长度或非有限值保持当前目标。仅在 `RUNNING` 且准备完成后生效。
- 状态：`upper_executor=float`、`arm_mode=passthrough`、`upper_source=none` 表示手臂跟随实测反馈；`gripper_source` 标识夹爪输入来源。

任务运行后可单次发送夹爪目标：

```bash
ros2 topic pub --once /motion_control/upper/input/keyboard std_msgs/msg/Float64MultiArray '{data: [1.0, 1.0]}'
```

```bash
ROS_DOMAIN_ID=217 python3 -m pytest src/upper_float/test -q
```