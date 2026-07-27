# arm_gravity_compensation
Unitree G1 双臂相对 `torso_link` 的重力参数标定工具。网页串联参数初始化、关节选择、手拉采点、纯扭矩自动标定和同构 URDF 导出。

## 边界
- 只使用 `/lowstate`、`/secondary_imu`、`/lowcmd` 和 MotionSwitcher API，不使用夹爪、力传感器或 CAN 接口。
- 只控制 G1 左右各 7 个手臂电机（索引 15–21、22–28）。**位置环由电机内部以自身频率闭合**：`LowCmd` 写入 `tau`（重力前馈）、`q`（参考轨迹）、`kp`（`motor_stiffness`，默认 40×5/20×2）和 `kd`（`motor_damping`，默认 3×5/1.5×2），`dq` 恒为零。本节点不做软件 PD，因此跟踪不受 ROS/DDS 延时和力矩限速影响。
- 电机实际输出 $\tau = \tau_{ff} + k_p(q_{des}-q) - k_d\dot q$，三项均已知或可测，所以它就是静态辨识的观测量；手臂停在哪里都不影响正确性。随着迭代收敛 $q_{des}-q \to 0$，观测量趋近纯前馈，也就越来越不依赖 $k_p$ 的标称精度。
- Pinocchio 以 `torso_link` 为固定根，只计算两条完整手臂子树。`final.urdf` 中固连的 KWR57、Gloria-M 和安装件都保留为独立 link 参数；夹爪主动关节固定在闭合位，mimic 链按 URDF 展开后锁定。

### 重力方向必须用躯干 IMU
G1 有**两个** IMU，这里只能用躯干那个：

| 话题 | 位置 | 能否用 |
|---|---|---|
| `/secondary_imu`（`unitree_hg/msg/IMUState`） | 躯干，对应 URDF 的 `imu_in_torso` | ✅ 本包使用 |
| `/lowstate.imu_state`（硬件插件导出为 `pelvis_imu`） | 盆骨 | ❌ 不能直接用 |

重力模型固定在 `torso_link`，而 `pelvis` 与 `torso_link` 之间隔着 `waist_yaw/roll/pitch` 三个关节。直接拿盆骨 IMU 当躯干系用，在弯腰时会把重力方向算错。实机实测：腰在 yaw 11.9° / roll 3.9° / pitch −1.6° 时，两个 IMU 的重力方向相差 **4.97°**；用腰关节链旋转后残差降到 1.15°（反向假设则放大到 8.91°），确认了上表的对应关系。

因为 `imu_in_torso_joint` 的 `rpy` 为零，`/secondary_imu` 的加速度计读数只需取反并归一化到 9.81 就是躯干系重力，不需要腾关节变换。`imu_topic` 参数可覆盖。

> **2026-07-27 之前的标定结果不可用**：当时错用了 `/lowstate.imu_state`（盆骨）。`parameters.json` 只存了 14 个手臂关节角、没存腰关节角，无法事后旋转修正，需要重跑。
- 采点阶段不创建 `/lowcmd` publisher，也不调用 MotionSwitcher。只有显式允许并从页面确认后，自动标定阶段才取得低层输出权。
- 当前只修正刚体质量缩放。对每个 link 使用同一系数 $s$：`mass *= s`，六个 inertia 分量同时 `*= s`，`origin xyz/rpy` 不变。

固连在同一个运动体上的若干 link（腕偏航下游的 KWR57、Gloria-M 和全部夹爪实体）只通过合计质量和合计一阶矩影响关节力矩，逐 link 估计是奇异的。因此**估计时按固连组归并为一个系数**（每侧 7 组，对应 7 个手臂关节），**写回 URDF 时再把该系数分配给组内每个 link**。JSON 仍逐 link 记录 `scale`、`observability` 和来源：
- `data_identified`：后验方差相对先验下降超过 90%，该系数由数据决定。
- `prior_distributed`：信噪比不足，结果主要来自 URDF 先验。
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

节点启动时检查 `config/parameters.json`：
1. 文件不存在时，从安装后的 `unitree_g1_description/model/final.urdf` 提取全部 link、joint 和 inertial 参数。
2. 文件存在时直接加载，不覆盖已有标定记录。
3. 页面勾选待标定 joint。勾选腕偏航时，其下游 KWR57 和固定夹爪实体自动属于同一选择组，但仍逐 link 保存结果。
4. 点击“开始采点”后手拉机械臂。自动模式在所选关节发生明显移动并再次稳定后记录姿态；也可手动点击“记录当前姿态”。

也可独立生成参数文件；该脚本不作为 ROS executable 安装：
```bash
PYTHONPATH=src/arm_gravity_compensation \
python3 src/arm_gravity_compensation/extract_urdf_parameters.py \
  src/unitree_g1_description/model/final.urdf \
  src/arm_gravity_compensation/config/parameters.json
```

### 一批采点两只手臂都能用
每个采点会记录它是在哪侧手拉出来的（`side` 字段，由开始采点时所选关节决定；两侧都选则记为 `both`）。标定**对侧**时，目标位置自动取镜像值：

$$q_{\text{other}} = [\,+1,\ -1,\ -1,\ +1,\ -1,\ +1,\ -1\,] \odot q_{\text{source}}$$

即 **pitch / elbow / wrist_pitch 保号，roll / yaw 取反**。`final.urdf` 的左右臂关于矢状面严格镜像（所有 origin 满足 `left_xyz = diag(1,-1,1) @ right_xyz`，轴向相同），200 组随机姿态的正运动学校验最大偏差 1e-5 m。关节限位本身也是镜像的（右 `shoulder_roll [-2.25, 1.59]` ↔ 左 `[-1.59, 2.25]`），所以源侧合法的姿态镜像后必定落在对侧限位内。由对称性，两侧的条件数完全一致。

镜像只省掉手拉采点，**对侧仍须实际跑一遍标定**才能测到它自己的力矩。`ParameterStore.mirror_link_estimate(side)` 可把一侧已标定的质量系数和（按同一符号向量镜像的）力矩零偏同步给另一侧作为起点。

## 第二步：自动标定
执行前必须满足：
- 机械臂得到可靠支撑，周围无人且运动范围无障碍物。
- ros2_control 的 FPC/JTC 都是 inactive，不存在其他 `/lowcmd` publisher。
- `/lowstate` 新鲜，`mode_pr == 0`。
- 已记录足够多且分布不同的姿态。页面会显示回归 rank/nullity 和条件数；姿态越多、分布越分散，可观测子空间越完整。

重新启动并显式开放扭矩输出：
```bash
ros2 launch arm_gravity_compensation gravity_calibration.launch.py \
  allow_torque_output:=true
```

页面要求输入固定确认词。开始后每个姿态依次执行：

1. 开始接管前等待 IMU 稳定窗口，平均 `/secondary_imu` 的加速度计，经 `imu_in_torso` 固定旋转得到躯干坐标重力方向。
2. Pinocchio 用**当前已标定参数**算重力前馈写入 `tau`，平滑参考轨迹（smoothstep）写入 `q`，电机自己完成位置跟踪。参考轨迹从接管瞬间的实测位置起步，所以 `kp` 项初始为零，不会甩手臂；`tau` 受 `torque_slew_rate` 限速，反馈项不受限。
3. 轨迹结束后不要求实际位置贴近目标。只要实测位置窗口稳定，就同步平均实际位置、IMU、电机实际输出力矩和 `tau_est`；回归始终在**实测位置**上建立，目标误差只写入记录。
4. 所有目标均采集完成前不更新模型参数；使用每个静态窗口的新重力方向进入下一个标记点。
5. 整侧全部目标完成后，用所有静态样本一次性求解静态平衡方程 $\tau_{cmd}=G(q)\,s+b$：按固连组归并列，按关节额定力矩把残差归一化到噪声单位，噪声水平由残差自适应（含自由度修正），并对系数施加 URDF 先验取最大后验解。EM 的两高斯混合按**整个姿态**判定内点，被人碰过或卡住的姿态会整块剔除。随后原子写回 JSON，并导出 `calibrated.urdf`。
6. 每次点击标定都从上一次的结果继续，是一次真正的外层迭代。页面记录每轮的 rank、nullity、条件数、每关节噪声和 RMSE。

任一 LowState 超时、PR 模式错误、IMU 不稳定、目标超时或用户停止都会终止输出。节点退出时先停止 `/lowcmd`，再尝试恢复接管前的 MotionSwitcher 模式。

## 两个已知缺陷（都没做，需要时按下面做）

这两件事都**没有实现**，2026-07-27 那次标定是用一次性脚本和手感修正绕过去的。它们是当前精度的主要限制。

### 缺陷一：零空间漂移，左右不对称

静态重力方程在 11 个姿态下 rank 10 / nullity 4，条件数约 130。近奇异方向上的参数可以互相抵消而几乎不改变残差，所以**外层迭代越多、漂得越远**，RMSE 却看不出来。实测右臂 7 轮的轨迹：

| 轮次 | `sh_yaw` 尺度 | `w_pitch` 尺度 | rmse_before → after |
|---|---|---|---|
| 1 | 1.387 | 1.086 | 0.908 → 0.595 |
| 3 | 1.734 | 0.979 | 0.565 → 0.556 |
| 5 | 1.999 | 0.851 | 0.554 → 0.547 |
| 7 | 2.210 | 0.743 | 0.547 → 0.542 |

第 2 轮之后 RMSE 只降了 6%，`sh_yaw` 却单调涨了 40%，同时 `w_pitch` 单调跌了 32%——这两个参数在残差里几乎抵消。`sh_yaw` 到 2.21 意味着 CAD 质量错了 121%，不可能是真的。左臂只迭代 3 轮，所以漂得少，操作者的直接感受就是"左臂效果好很多"。

**什么时候要做**：每次新一轮标定之后。判据是比较两侧的组尺度，几何均值偏离 1 超过 3% 就该处理；或者直接在浮动 demo 里感觉某一侧发飘。

**怎么做**。两臂在 URDF 里逐 link 质量完全相同（实测最大差 0 kg），是同一硬件的镜像，真值必须相等，所以差值全部是辨识误差，取平均可以把方差减半：

```python
store = ParameterStore("src/arm_gravity_compensation/config/parameters.json")
document = store.load()
# 偏置先读出来：它是每个电机自己的量，不镜像也不平均。
biases = {side: store.link_estimate(side)[1] for side in ("left", "right")}
scales = 0.5 * (store.link_estimate("left")[0] + store.link_estimate("right")[0])
for side in ("left", "right"):
    links = tuple(document["model_scope"]["parameter_links"][side])
    observed = [document["links"][name]["inertial"]["identification"]["observability"]
                for name in links]
    store.apply_link_estimate(
        side, links, scales, biases[side], observed, np.zeros(7),
        {"source": "symmetrized", "sample_count": 0, "rank": 0, "nullity": 0,
         "rmse_before": 0.0, "rmse_after": 0.0})
```

`observed` 要从文档里读出来原样传回，不能图省事填 `np.ones`——那会把每个 link 都标成 `data_identified`，掩盖掉哪些其实是先验兜底的。做完重新导出重力表。上次这样做把左右最大力矩差从 0.756 N·m 压到 0。

这只消除不对称，**不修正共模漂移**：两臂之后漂得一样多，`sh_yaw` 仍然是 2.0 量级。要根治得收紧 `scale_prior_std`（当前 0.3 太松，可试 0.1），用已存的样本重新拟合，不必重新采点。

### 缺陷二：静摩擦被吸进质量，需要双向逼近

RMSE 稳定在 **0.53 N·m** 附近，加姿态、加轮次都降不下去。这是静摩擦的地板，不是噪声。

问题在于每个姿态只从**一个方向**趋近。静摩擦总是反抗最后一次运动的方向，所以它在所有样本里同号，被回归当成了系统项，一部分进 `torque_bias`、一部分抬高质量。证据有两条：

- `right_shoulder_roll` 的偏置到了 **0.813 N·m**，而其余 13 个关节的绝对值中位数只有 0.122
- 手感标定出的整体倍率是 **0.95**，即静态拟合把质量系统性抬高了约 5%

在位置保持模式下这看不出来（$k_p$ 把它吃掉了），但手臂一浮起来就是单向爬行——因为浮动模式没有位置反馈，任何恒定力矩都无人对抗。

**什么时候要做**：想去掉 `compensation_scale` 那个经验倍率、或者要给 `torque_bias` 一个可信值时。只做重力补偿的话，当前的手感修正已经够用。

**怎么做**。每个姿态采两次：一次从**大于**目标的位置降下来停住，一次从**小于**目标的位置升上去停住，两个静态窗口都记录。设静摩擦幅值为 $f$，则

$$\tau_\uparrow = G(q) + f,\qquad \tau_\downarrow = G(q) - f$$

平均值 $\tfrac12(\tau_\uparrow + \tau_\downarrow)$ 精确消掉静摩擦，差值的一半 $\tfrac12(\tau_\uparrow - \tau_\downarrow)$ 直接给出每个关节的 $f$。前者喂给现有回归即可，无需改拟合代码；后者可以单独存一张库仑摩擦表。

代价是**标定时间翻倍**（每个姿态两个静态窗口）。实现上要改 `TorquePoseController` 的参考轨迹：在目标两侧各加一个超调点，`workflow_node` 的 `_sample_static_pose` 采两次并打上方向标签。

做完之后 `torque_bias` 才有资格导出到运行时——现在它被排除在重力表之外正是因为它混了静摩擦，见 [unitree_g1_ros2_control/README.md](../unitree_g1_ros2_control/README.md)。

## 文件
标定结果放在包内 `config/`，**纳入版本管理**，三个文件都在点击“导出”时原子写入：
- `config/parameters.json`：源 URDF 参数、当前逐 link 标定值、采点和每轮迭代记录。
- `config/calibrated.urdf`：与源 `final.urdf` 保持相同 link/joint/mimic 结构，只替换标定后的 `mass` 和六个 inertia 分量。
- `config/gravity_table.yaml`：给运行时用的**归并刚体链**，每侧 7 个体的 `axis / origin_xyz / origin_rotation / mass / com` 平铺数组，另带 `imu_to_torso`。它按 ROS 2 参数文件格式书写，由 `unitree_g1_controllers/ArmGravityCompensation` 通过 `package://arm_gravity_compensation/config/gravity_table.yaml` 读取。标定出的力矩偏置不在其中（只留在 `parameters.json`），原因见 [unitree_g1_ros2_control/README.md](../unitree_g1_ros2_control/README.md)。

节点拿到的是安装后的 share 路径，而 `--symlink-install` 使它经 `build/` 指回源码树，写入前的 `Path.resolve()` 会跟随这条链，所以导出直接落在 `src/arm_gravity_compensation/config/` 上，符号链接也不会被替换。三个路径均可用 `parameter_file` / `calibrated_urdf` / `gravity_table` 参数覆盖；重力表的顶层键名取自 `gravity_controller_name`（默认 `arm_gravity_compensation`）。

导出后需重新加载 controller 才会生效，见 [unitree_g1_ros2_control/README.md](../unitree_g1_ros2_control/README.md)。

可通过 launch 参数覆盖 `urdf_path`、`parameter_file`、`calibrated_urdf`、`lowstate_topic`、`imu_topic`、`lowcmd_topic` 和 `port`。