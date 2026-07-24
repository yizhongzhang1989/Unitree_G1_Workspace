# arm_gravity_compensation
Unitree G1 双臂相对 `torso_link` 的重力参数标定工具。网页串联参数初始化、关节选择、手拉采点、纯扭矩自动标定和同构 URDF 导出。

## 边界
- 只使用 `/lowstate`、`/lowcmd` 和 MotionSwitcher API，不使用夹爪、力传感器或 CAN 接口。
- 只控制 G1 左右各 7 个手臂电机（索引 15–21、22–28）。`LowCmd` 的 `q/dq/kp` 为零，软件位置闭环写入 `tau`；电机端 `kd=3` 提供附加速度阻尼。
- Pinocchio 以 `torso_link` 为固定根，只计算两条完整手臂子树。`final.urdf` 中固连的 KWR57、Gloria-M 和安装件都保留为独立 link 参数；夹爪主动关节固定在闭合位，mimic 链按 URDF 展开后锁定。
- 采点阶段不创建 `/lowcmd` publisher，也不调用 MotionSwitcher。只有显式允许并从页面确认后，自动标定阶段才取得低层输出权。
- 当前只修正刚体质量缩放。对每个 link 使用同一系数 $s$：`mass *= s`，六个 inertia 分量同时 `*= s`，`origin xyz/rpy` 不变。

固定末端各 link 在静态重力数据中并非全部独立可辨识。系统仍逐 link 保存和输出参数，但 EM 只更新数据可观测的组合；零空间取距离当前 URDF 最近的解。JSON 为每个 link 记录 `observability` 和来源：
- `data_identified`：该参数方向由数据完整辨识。
- `prior_distributed`：部分结果由 URDF 最近解约束分配。
- `urdf_initial`：尚未标定。

## 构建
```bash
source scripts/env.sh
colcon build --symlink-install --packages-select unitree_api unitree_hg unitree_g1_description arm_gravity_compensation
```

## 第一步：参数初始化与手拉采点
默认禁止输出扭矩：
```bash
ros2 launch arm_gravity_compensation gravity_calibration.launch.py
# http://<本机 IP>:8310
```

节点启动时检查 `~/.ros/arm_gravity_compensation/parameters.json`：
1. 文件不存在时，从安装后的 `unitree_g1_description/model/final.urdf` 提取全部 link、joint 和 inertial 参数。
2. 文件存在时直接加载，不覆盖已有标定记录。
3. 页面勾选待标定 joint。勾选腕偏航时，其下游 KWR57 和固定夹爪实体自动属于同一选择组，但仍逐 link 保存结果。
4. 点击“开始采点”后手拉机械臂。自动模式在所选关节发生明显移动并再次稳定后记录姿态；也可手动点击“记录当前姿态”。

也可独立生成参数文件；该脚本不作为 ROS executable 安装：
```bash
PYTHONPATH=src/arm_gravity_compensation \
python3 src/arm_gravity_compensation/extract_urdf_parameters.py \
  src/unitree_g1_description/model/final.urdf \
  ~/.ros/arm_gravity_compensation/parameters.json
```

## 第二步：自动标定
执行前必须满足：
- 机械臂得到可靠支撑，周围无人且运动范围无障碍物。
- ros2_control 的 FPC/JTC 都是 inactive，不存在其他 `/lowcmd` publisher。
- `/lowstate` 新鲜，`mode_pr == 0`。
- 已记录足够多且分布不同的姿态。页面会显示回归 rank/nullity；增加姿态只能改善可观测子空间，固定末端固有的零空间仍由 URDF 约束分配。

重新启动并显式开放扭矩输出：
```bash
ros2 launch arm_gravity_compensation gravity_calibration.launch.py \
  allow_torque_output:=true
```

页面要求输入固定确认词。开始后每个姿态依次执行：

1. 开始接管前等待 IMU 稳定窗口，平均 `LowState.imu_state.accelerometer`，经 `imu_in_torso` 固定旋转得到躯干坐标重力方向。
2. Pinocchio 使用源 URDF 参数计算当前位置重力前馈；软件侧 PD 只生成 `tau`，以限速、限力矩变化率轨迹把手臂引导到目标附近。当前不做力矩幅值裁剪。
3. 轨迹结束后不要求实际位置贴近手拉目标。只要实测位置窗口稳定，就在保持扭矩的同时同步平均实际位置、IMU、命令扭矩和 `tau_est`；`dq` 只用于诊断，目标误差仅写入记录。
4. 所有目标均采集完成前不更新模型参数；使用每个静态窗口的新重力方向进入下一个标记点。
5. 整侧全部目标完成后，使用所有静态样本统一执行一次带 URDF 弱先验的有界鲁棒 EM，随后原子写回 JSON，并导出 `calibrated.urdf`。每个点的实际位置、IMU 重力和两种力矩都保存在该批次记录中。

任一 LowState 超时、PR 模式错误、IMU 不稳定、目标超时或用户停止都会终止输出。节点退出时先停止 `/lowcmd`，再尝试恢复接管前的 MotionSwitcher 模式。

## 文件
默认输出目录为 `~/.ros/arm_gravity_compensation/`：
- `parameters.json`：源 URDF 参数、当前逐 link 标定值、采点和每轮迭代记录。
- `calibrated.urdf`：与源 `final.urdf` 保持相同 link/joint/mimic 结构，只替换标定后的 `mass` 和六个 inertia 分量。

可通过 launch 参数覆盖 `urdf_path`、`parameter_file`、`calibrated_urdf`、`lowstate_topic`、`lowcmd_topic` 和 `port`。