# upper_insert

基于视觉定位与六维力觉的 Unitree G1 上肢插孔控制任务包，提供 50 Hz 闭环控制、工具与力传感器标定以及 Web 调试控制台。真机调试中遇到的典型问题、根因分析与优化记录详见 [真机调试问题排查与解决方案汇总](./TROUBLESHOOTING.md)。策略训练见 [madderscientist/InsertAnything](https://github.com/madderscientist/InsertAnything)。

## 目录结构与道具视觉定位解耦

- **上层插孔控制（通用）**：由 [insert_node.py](./upper_insert/insert_node.py)（50 Hz 控制主循环与 Web 控制台）、[insert_policy.py](./upper_insert/insert_policy.py)（ONNX GRU 策略推理与 15 ms 力觉低通滤波）和 [tool_calibration.py](./upper_insert/tool_calibration.py)（主相机 + TF 末端标定与力臂变换）组成，仅依赖插头尖端 $T$ 到目标孔 $H$ 的 6DoF 相对位姿差与六维腕力。
- **特定插孔道具/模具视觉定位（[upper_insert/vision/](./upper_insert/vision)）**：为在无通用孔位检测器的条件下实时获取插头距离目标孔的相对位置与姿态，实验使用一套贴有 ArUco `ID0`（14 孔插板）与 `ID1`（插头夹具）的特定插入道具。[config/pos.json](./config/pos.json) 专门记录了这套特定插入道具在 `ID0` 孔板系下的 14 个孔位坐标与 `ID1` 插头系下的尖端偏移（单位 mm），目的是通过双码 6DoF 位姿直接解算插头尖端到目标孔的相对位置与姿态误差。所有与该模具/道具配套的视觉定位、CUDA 预处理、C++ 遮挡恢复、画面标注及贴纸打印生成代码均集中在 [upper_insert/vision/](./upper_insert/vision) 文件夹下。

## 启动

各命令在独立终端运行，已有节点不要重复启动：

```bash
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual
ros2 launch g1_motion_session session.launch.py
ros2 launch upper_insert insert.launch.py target_hole:=circle_tight insert_depth_mm:=-15.0
```

Web 控制台默认地址为 `http://localhost:8088`，默认使用右臂、`float` 模式、`auto` 孔位与 `15 mm` 插入深度（负深度表示孔口外上方预定位）。可通过启动参数覆盖 `active_arm`、`camera_index`、`vision_preprocess`（`cuda` 或 `cpu`）、`vision_rate_hz`、`web_port` 与 `config`。在网页“选择插入孔位”下拉框中可切换 `auto`（最近孔）或 [pos.json](./config/pos.json) 中的 14 个具体孔位，机械插入与策略插入共用该孔位与深度设置。

整机入口已启动净力补偿和负载估计，确认机器人支撑、运动空间、急停、输入状态和末端标定后再执行接合：

```bash
ros2 service call /motion_control/engage std_srvs/srv/Trigger '{}'
```

准备完成后自动进入 `float` 浮动模式，无需另发 `start`。普通停止可调用 `/motion_control/stop`，紧急停止调用 `/motion_control/estop`。

## 控制模式与策略契约

- `float`：以 50 Hz 发布双臂实测关节角跟随手拉摆位，夹爪保持当前设定开合度；若此前因 `/joint_states` 短暂超时进入错误态，反馈恢复后切回 `float` 可自动恢复运行。
- `hold`：冻结双臂当前 14 轴实测关节角（不运行 IK 零空间自运动），供下肢行走或等待期间锁定手臂姿态；可通过网页“冻结手臂（锁定当前位置）”按钮或调用 `/upper_insert/freeze`（兼容 `/upper_float/freeze`）触发，调用 `/upper_insert/unfreeze`（兼容 `/upper_float/unfreeze`）或点击浮动按钮即可解冻。
- `mech_insert`：采集一帧新鲜视觉与实测 FK 计算完整尖端终点，转换为夹爪目标后交由 C++ 高精度 IK 持续跟踪；不随新帧反复重定向，修改孔位、深度或切换模式时自动清除旧机械终点。
- `policy_insert`：以 50 Hz 运行 [policy.onnx](./config/policy.onnx) 双层 GRU 策略（输入 `obs[1,13] = [Δp_H, Δq_wxyz, F_T, M_T]`，输出 `actions[1,6]`，导出尺度为每单位 `1 mm / 0.5°`，详见 [DEPLOYMENT_ONNX.md](./config/DEPLOYMENT_ONNX.md) 与 [contract.json](./config/contract.json)）。每拍通过 500 Hz `/joint_states` 历史缓冲与差分 FK 补偿视觉延迟，在指令尖端位姿 `(p_tip_cmd, R_tip_cmd)` 上累加策略增量以保留抗重力/静摩擦偏置，并施加相对于实测尖端的 XY/Z 独立防积分饱和限幅（`max_tip_lead_m: 12 mm`、`max_tip_lead_deg: 4.0°`）。面板同时提供“策略单步执行 (走1步并保持)”与“仅算一步 (不移动)”按钮，便于逐步核验策略输出方向与末端执行响应。

本任务在 [upper_insert.yaml](./config/upper_insert.yaml) 中将 IK 迭代上限设为 `20`、位置容差 `0.02 mm`、姿态容差 `0.00007 rad`，并将 `wrist_roll` 零空间增益设为 `0.0`（保留 `shoulder_roll=0.05`、`shoulder_yaw=0.20`、`elbow=0.20` 及 `[0.6, 1.2] rad` 门限死区），避免 7 轴臂在 6D 末端硬约束下的 1 维零空间内因手腕回零把肘部拉向躯干或将手腕展平。当单拍末端目标超出关节限速窗口时，C++ IK 沿 SE(3) 螺旋路径等比缩步（`task_fraction`），保证尖端移动方向不被逐轴限速裁剪破坏。

## 视觉流水线与工具标定

- **外置相机定位（30 Hz）**：外置 Azure Kinect 默认按 [camera_config.json](./config/camera_config.json) 以原生 `YUYV 1280×720@30` 采集（支持 USB 闪断后自动重扫 V4L2 节点重连），通过 `SharedMemory` 单帧零拷贝送入独立 `spawn` 视觉子进程，使用 CUDA 灰度/饱和度预处理与 C++ 候选轮廓恢复解算 6DoF 位姿。因孔板 `ID0` 相对外置相机固定，当机械臂遮挡 `ID0` 时自动沿用上一有效 `ID0` 位姿并仅在局部 ROI 快速检测 `ID1`；若 `ID1` 暂时丢失，在允许时间窗内利用帧间差分 FK 互补滤波短暂保持，超时后策略立即停止推进。如需重新生成 3.5 cm（3.0 cm 黑框 + 2.5 mm 白边）标定码贴纸，可运行 `ros2 run upper_insert generate_markers`。
- **主相机 + TF 末端标定**：保持头部与手臂静止且主相机能清晰看到正面完整的 `ID1` 码，在 `float` 或 `hold` 模式下点击“主摄像机 + 发布 TF 标定末端”。节点通过临时采集节点按需抓取一帧 `/head/camera/color/image_raw` 并查询同刻 TF，解算夹爪到尖端变换 $(R_{GT}, p_{GT})$ 及力传感器力臂 $r_{ST}^T$，自动持久化至 [tool_calibration.json](./config/tool_calibration.json) 并生成带 `TIP` 投影的核验截图。`mech_insert` 强制要求使用 `head_camera_tf` 标定结果，拒绝未标定的名义默认值。
- **力觉滤波与安全保护**：优先订阅 `/arm{0,1}/wrench_net`（净力超时自动回退至 `/arm{0,1}/wrench_raw` 加 URDF 重力补偿），在传感器系执行 `15 ms` 低通滤波后转换至插头尖端 $T$；未滤波原始尖端力/力矩超过 `30 N / 2 N·m` 时立即触发安全保护并切回 `hold` 模式。

## 构建与测试

```bash
colcon build --packages-select upper_insert
python3 -m ament_flake8.main --linelength 120 src/upper_insert
python3 -m pytest -q src/upper_insert/test
```
