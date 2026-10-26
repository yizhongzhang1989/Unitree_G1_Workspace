# VR 遥操作

头显通过 WebXR 获取手柄位姿，经 WebSocket 发送给 `vr_teleop`。双臂由本地 ArmIKMixin 求解，夹爪直接映射，TaskBase 注册并发布自身关节目标到 FPC；摇杆独立发布标准四值速度数组。

## 启动

以下命令在工作区根目录执行，每个终端先 `source scripts/env.sh`；已加载 Humble 与所需 SDK 环境时也可 `source install/setup.bash`。硬件显式另起，已有硬件栈时复用：

```bash
source scripts/env.sh
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual
```

另一终端生成证书并启动 [全身 VR 入口](../../robot_bringup/launch/whole_vr.launch.py)：

```bash
source scripts/env.sh
ros2 run upper_vr make_vr_cert
ros2 launch robot_bringup whole_vr.launch.py
```

该入口包含一个 session、上肢 VR 和使用 `lower_vr.yaml` 的下肢速度任务，`lower_source` 为 `vr`，摇杆负责行走输入。组合不启动硬件、不自动接管；接管与下肢 start 由 B/Y 显式触发，不另起 session 或其他组合入口。需要下肢键盘而上肢 VR 时，改选 `ros2 run robot_bringup teleop_keyboard --upper-task vr`，下肢保持默认键盘来源；键盘入口不指定上肢时默认为 float。组合键盘退出先请求急停，再向自己启动的任务进程组发 SIGINT，不停止已有硬件。更多组合、配置参数与退出顺序见 [robot_bringup 集中入口说明](../../robot_bringup/README.md)。

手动拆分部署时，在硬件与唯一 session 已启动后分别运行 `ros2 launch lower_velocity_tracking velocity_tracking.launch.py config:=lower_vr.yaml` 与 `ros2 launch upper_vr vr_teleop.launch.py`；不需要行走时只启动上肢任务。已有任务需要额外控制台时可用 `ros2 run lower_velocity_tracking teleop_keyboard`，它只连接现有 session 与任务，不改变下肢已选输入来源。

头显浏览器打开 `https://<机器人IP>:8443`，接受自签名证书并进入 VR。更换机器人网段后重新运行 `make_vr_cert`。WebXR 需要 HTTPS 或 localhost。

## ADB 连接

也可通过 ADB 反向映射使用 localhost。以下命令在 devcontainer 内执行：

```bash
adb connect <头显IP>:5555
adb reverse tcp:8000 tcp:8000
```

头显浏览器打开 `http://localhost:8000`。头显休眠或连接中断后可能需要重新设置 reverse；长期使用优先 HTTPS。

## 操作

| 输入 | 操作 |
|---|---|
| squeeze | 对应手臂离合；握住后跟随手柄 |
| trigger | 对应夹爪开合 |
| 左摇杆 | 水平速度 |
| 右摇杆 | 转向与高度 |
| 单手 A/X | 对应手臂归位；握 squeeze 或再次按键取消 |
| 双手 B/Y | 站立、启动、急停；长按急停 |

一次握住期间采用绝对映射。手柄超出可达范围后，松开再握 squeeze 可在当前手臂位置重新接合。跟踪丢失时目标冻结。

`engage` 后，任务的准备完成回调开放上肢操作。B/Y 的 `start` 步骤用于启动下肢；默认平移与转角均保留节点源码中的 world，可通过 launch 参数分别选择 local。

## 监控

```bash
ros2 launch g1_motion_session dashboard.launch.py
```

浏览器访问 `http://<机器人IP>:8181/`。任务组合和接口见 [运动任务文档](../README.md)。
