# Upper VR

上肢任务使用 TaskBase 注册14臂关节与2个夹爪，ArmIKMixin 在进程内求解后直接发布 JointTargets 到 FPC。摇杆独立发布标准四值速度数组 `[vx, vy, wz, height]`，由下肢任务消费。`engage` 后完成准备，`prepared(snapshot)` 开放上肢执行；B/Y 的第二步 `start` 用于启动下肢。

## 启动

推荐在工作区根目录使用 [robot_bringup 集中入口](../robot_bringup/README.md)，硬件显式另起。每个终端先 `source scripts/env.sh`；已加载 Humble 与所需 SDK 环境时也可 `source install/setup.bash`：

```bash
# 终端 A，已有硬件栈时复用
source scripts/env.sh
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual
```

```bash
# 终端 B，证书首次使用或网段变化时生成
source scripts/env.sh
ros2 run upper_vr make_vr_cert
ros2 launch robot_bringup whole_vr.launch.py
```

[whole_vr.launch.py](../robot_bringup/launch/whole_vr.launch.py) 一次启动一个 session、上肢 VR 和配置为 `lower_vr.yaml` 的下肢速度任务，`lower_source` 为 `vr`。组合不启动硬件、不自动接管；上肢与下肢生命周期由操作者显式触发，硬件保持独立。

若下肢仍由键盘控制而上肢使用 VR，另选 `ros2 run robot_bringup teleop_keyboard --upper-task vr`，下肢保持默认键盘来源；`--upper-config` 与 `--lower-config` 分别覆盖所选任务配置。正式键盘入口默认上肢为 float，直接启动 `motion_tasks.launch.py` 默认上肢为 VR。组合键盘退出先请求急停，再对自身启动的任务进程组发 SIGINT，既有硬件进程保持运行。

手动拆分是上述组合入口的另一种方案，在硬件与唯一 g1_motion_session 已启动后按需运行：

```bash
ros2 launch lower_velocity_tracking velocity_tracking.launch.py config:=lower_vr.yaml
ros2 launch upper_vr vr_teleop.launch.py
```

不需要行走时只启动 upper_vr。单包 launch 的 VR 参数覆盖方式保持不变。VR 与其他上肢任务使用同一组关节，每次只选择一套 session 和任务组合；已有任务需要额外键盘控制台时使用 `ros2 run lower_velocity_tracking teleop_keyboard`，它不启动任务或 session，速度只由选择键盘来源的下肢任务消费。

WebXR、ADB、手柄分工与监控见 [VR 操作](vr/README.md)。单手归位由 TaskBase 插值；归位侧不参与 IK 目标求解，完成或取消后以实际输出 FK 重新接合。

## 接口

- 输出：`/forward_position_controller/joint_targets`，仅15至30槽位。
- 摇杆：`/motion_control/lower/input/vr`，Float64MultiArray 四值数组。
- 生命周期：`/motion_control/register`、`/motion_control/events`、`/motion_control/tasks`。
- B/Y、A/X 服务与会话状态：默认 `/motion_control` 和 `/motion_control/status`。
- WebXR：HTTP 8000、HTTPS 8443、`/monitor`、`/state`、`/haptic` 与两个 WebSocket 路径。

## 验证

```bash
ROS_DOMAIN_ID=217 python3 -m pytest src/upper_vr/test -q
```