# arm_gravity_compensation
Unitree G1 双臂相对 `torso_link` 的重力参数标定工具。网页串联参数初始化、关节选择、手拉采点、纯扭矩自动标定和同构 URDF 导出，并在同一批静态姿态上顺带完成 KWR57 力传感器的零偏与末端工具标定。

数学推导（观测方程、双向消摩擦、分组可辨识性、MAP 与鲁棒 EM）见 [`CALIBRATION.md`](CALIBRATION.md)。

## 边界
- 手臂惯性辨识只使用 `/lowstate`、`/secondary_imu`、`/lowcmd` 和 MotionSwitcher API，不使用夹爪或 CAN 接口。
- 力传感器标定**只订阅** `wrench_raw`，不发任何 LowCmd：把手臂摆到一个朝向、松手让现有重力补偿悬停（或用手扶前臂，别碰工具）就能采点。它和力矩辨识共享采点动作，但求解完全独立——一次线性最小二乘，不迭代。
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

## 第三步：力传感器标定
页面第 05 段。整段**只读**：不发 LowCmd，也不需要 `allow_torque_output`。

1. 让手臂能自己停住（现有重力补偿浮动，或用手扶前臂），把末端摆到一个朝向。
2. 点"记录当前朝向"。节点在一个 IMU 平均窗口里同时统计两台传感器的均值和抖动；抖动超阈值（默认 1 N / 0.1 N·m）直接拒绝——那说明还有人碰着工具。
3. 换朝向重复。**至少 4 个**，并且要让传感器的三根轴都朝下过一次；页面上的 `spread` 是重力单位向量矩阵的最小/最大奇异值之比，越接近 1 越好，实践上做 8~12 个。
4. 点"求解"，核对"建议取矩点"的量级后点"采用建议取矩点重解"（见下），再点第 01 段的"导出全部结果"。那一个按钮同时写 `calibrated.urdf`、`gravity_table.yaml` 和 `ft_calibration.yaml`。

自动扭矩标定跑到每个姿态停稳时也会顺带记一条力样本（`source = automatic_settle`），因为那一刻工具那端最干净。

### 求解了什么，没求解什么
力通道解 $F = A u + b_F$：$A$ 取完整 3×3 时未知量 12 个，每个朝向给 3 个方程，所以 4 个朝向恰好可解。对 $A$ 做极分解 $A = \text{polarity}\cdot m\cdot R$ 一次拿到三样东西：工具质量、安装姿态偏差、以及奇异值的离散——**前者是可复现的常值偏差，后者才是轴增益/正交性损伤**。

只有当自由模型相对受约束模型（$R$ 固定为名义安装姿态）的残差下降通过 F 检验（p < 0.01）时才采纳估计出的 $R$，否则一律用名义值。这一道是必须的：关节零位和 URDF 几何误差合起来约 1°，会伪造出约 1.7% 的表观非正交性，而它对质量估计只是二阶影响（< 0.02%）。页面同时显示两个模型的残差和 p 值。

力矩通道在 $A$ 定下来之后对一阶矩线性，需要至少 3 个不共面的朝向。**纯重力标定解不出力矩通道的轴增益**：$M = B(h\times u)$ 里 $B\,\text{skew}(h)$ 的秩最多是 2，$B$ 和 $h$ 分不开，所以那一路只出零偏和一阶矩。

标定用的是**实测关节角**，不是指令位置，所以到位精度不进模型。

### 力矩参考点必须从外面给
传感器对**自己的力矩参考点**取矩，厂家把它放在工具侧法兰面上，不在 URDF link 原点。重力标定同样解不出它：一阶矩只在 $c - d$ 这个组合里出现，$h$ 和 $d$ 是同一项。所以 `measurement_origin` 是**输入**，不是输出。

页面给出建议值的办法是拿 CAD 反推 —— 质量偏差只改大小不改方向，于是

$$d = c_{\text{CAD}} - \frac{h_{\text{meas}}}{m_{\text{meas}}}$$

**必须核对量级**：它应该正好等于一个传感器高度（KWR57 是 53 mm）。对上了才说明"参考点在法兰面"这个假设成立；对不上说明 CAD 质心本身不可信，这时宁可留 0。

实机首次标定的数据（2026-08-11，11 个朝向/侧）：

| | 左 | 右 |
|---|---|---|
| 测得工具质心 z | 48.5 mm | 49.7 mm |
| CAD 远端质心 z | 101.5 mm | 101.5 mm |
| **反推出的参考点** | **[−1.1, −5.7, **53.0**] mm** | **[−1.2, −6.2, **51.8**] mm** |

KWR57 圆柱高 53 mm、夹爪法兰在 z = 53 mm。真正的证据不是 z 分量对得上（那是定义使然），而是**两台独立标定的传感器都给出同一个方向**：横向分量只有 1~6 mm，几乎落在传感器轴线上；假设若不成立，$d$ 会指向任意方向、量级任意。

设错的后果只在力矩上：负载被挂在比真实位置靠近手腕 52 mm 处，1 kg 负载在腕部少算 0.51 N·m。净力的**力**部分完全不受影响。

流程：先照常求解一次（`measurement_origin` 默认 0），核对建议值的量级与方向，然后点「采用建议取矩点重解」。该值随标定结果一起存进 `parameters.json`，之后每次求解都会沿用，不必重复设置。

### 运行时
整机入口默认已经拉起这两个节点：

```bash
ros2 launch robot_bringup all_data.launch.py scope:=whole_body
# 已有整机栈时单独补上：
ros2 launch robot_bringup end_effector_load.launch.py
```
- `ft_wrench_compensator`（C++，在 `unitree_g1_ros2_control`）：`wrench_raw` → `wrench_net`，扣掉零偏和工具自重，输出的是**负载或环境施加给工具侧的物理力旋量**，静挂 1 kg 就是 9.81 N 指向地面，frame 是 `*_kwr57b_link` 且对其原点取矩。服务 `~/rezero` 在已知空载时单姿态重估零偏（只改内存，应对温漂）。
- `payload_estimator`（Python，本包）：净力 + `<arm>/gravity` → `geometry_msgs/InertiaStamped` 的质量与质心 → 重力补偿。**不把净力直接喂回补偿**：那是一条增益 $1/k_p$ 的导纳环，而且分不清"一直拎着的负载"和"顶到桌子上的接触力"。压成缓变参数则天然稳定——只在重力方向不再动（手臂静止）且净力确实沿重力方向时才更新。质心要多个朝向才可辨识，在那之前按可观测度线性混合工具自身的质心当先验。

**运动学只算一遍**：补偿节点为了扣工具自重本来就要算传感器系的重力方向，于是把它一并发到 `<arm>/gravity`，估计器直接用——不订阅 `/joint_states`，也不加载重力表。养第二份 FK 除了多一份 CPU，还会在两次求解落到不同 `joint_states` 采样上时悄悄错开。

> **标定完成后不要再按 dashboard 的"置零"**：那是驱动内部的软件 tare，会把标定好的零偏整个错开。`ft_wrench_compensator` 启动时会主动调一次 `reset_tare` 把它清掉。

## 两个已知缺陷

缺陷二已于 2026-08-19 实现；**缺陷一被它顺带解决**。原理与公式见 [`CALIBRATION.md`](CALIBRATION.md)。

### 缺陷一：零空间漂移（**已解决**）

近奇异方向上的参数可以互相抵消而几乎不改变残差，所以**外层迭代越多、漂得越远**，RMSE 却看不出来。
单向标定时代 `sh_yaw` 尺度能漂到 **2.21**（意味着 CAD 质量错了 121%，不可能是真的）。

根因是摩擦：它作为同号系统项时，拟合只能把它往不可辨识方向上塞。改双向后一并好转：

| | 单向 | 双向 |
|---|---|---|
| rank / nullity | 10 / 4 | **11 / 3** |
| rmse_after | 0.53 ~ 0.60 | **0.11 ~ 0.14** |
| 最大尺度偏离 | 2.21 | **1.25** |

**唯一仍需注意的**：rmse 收敛后别再跑外层迭代。边际收益没了，漂移风险还在——存盘的多轮
累积值 `sh_yaw` 是 1.338，而同一批样本单轮重拟合只有 1.207。

收紧 `scale_prior_std` 试过，不划算：0.30 → 0.10 只把左右不对称从 0.049 压到 0.037，共模几乎不动；
收到 0.02 则先验压过数据，rank 掉回 10、rmse 翻倍。**保持 0.30**。

剩下的共模偏离多半是真的：两侧 `wrist_pitch` 1.16~1.18、`wrist_yaw` 1.21~1.25 高度一致，而
`wrist_yaw` 的 `scale_observability` 是全臂最高的 0.99。腕上挂着 KWR57 + Gloria-M + 线缆，
URDF 低估 20% 合理。左右不对称同理——**右臂换过硬件，两侧真值本就不必相等**，不要再做对称化平均。


### 缺陷二：静摩擦被吸进质量（**2026-08-19 已实现双向逼近**）

RMSE 曾稳定在 **0.53 N·m** 附近，加姿态、加轮次都降不下去。这是静摩擦的地板，不是噪声。

问题在于每个姿态只从**一个方向**趋近。静止时力矩平衡是 $\tau_{\text{applied}} + G + \tau_f = 0$，
而 $\tau_f$ 在 $[-\tau_s, +\tau_s]$ 内**取值不定**——静力平衡对它欠定。静摩擦总是反抗最后一次
运动的方向，所以它在所有样本里同号，被回归当成了系统项，一部分进 `torque_bias`、一部分抬高质量。
证据有三条：

- `right_shoulder_roll` 的偏置到了 **0.813 N·m**，而其余 13 个关节的绝对值中位数只有 0.122
- 手感标定出的整体倍率是 **0.95**，即静态拟合把质量系统性抬高了约 5%
- 2026-08-19 梯形波实测 $\tau_s$ 为 **0.05~0.77 N·m**，而改进前左臂的标定残差中位 0.081 N·m
  ——**恰好落在同一量级，残差主要就是摩擦而不是模型误差**

在位置保持模式下这看不出来（$k_p$ 把它吃掉了），但手臂一浮起来就是单向爬行——因为浮动模式
没有位置反馈，任何恒定力矩都无人对抗。

**实现**。`workflow_node` 每个姿态采两次：先退到 `target - offset` 再回到 `target`
（最后一段向 +），然后退到 `target + offset` 再回来（最后一段向 −），两个 `StaticSample` 取平均。
设摩擦幅值为 $f$，则

$$\tau_\uparrow = -G(q) + f,\qquad \tau_\downarrow = -G(q) - f$$

平均值 $\tfrac12(\tau_\uparrow + \tau_\downarrow)$ **恒等消掉摩擦，与 $f$ 多大无关**；
差值的一半直接给出每个关节的 $f$。拟合代码一行没动。

**必须先退开再回来**：直接走到目标点的话方向取决于手臂原先在哪，两次可能同向。

**退让幅度必须超过死区** $2\tau_s/k_p$（实测最大 0.107 rad），否则关节根本没挪到另一侧，
两次停在同一点、同一摩擦符号，平均下来**等于没做**。故 `approach_offset_rad` 默认 **0.12**；
置 0 退回旧的单向采样。

代价是**标定时间约 4 倍**（每个姿态 4 次移动 + 2 个静态窗口）。

**差值也留着**：$\tfrac12(\tau_\uparrow - \tau_\downarrow)$ 就是那一点的摩擦力矩，记在
`StaticSample.friction`，随每轮迭代写进 `parameters.json`，并在每侧标定结束时打印
（跨位姿中位数与 p90）。它**不进回归**，是白拿的副产物。

实测摩擦**正比于关节载荷**：$\tau_f \approx \mu|\tau_g| + \tau_0$，$\mu$ 在 0.13~0.22，
各关节与两侧高度一致。这是摩擦补偿能否安全实施的前提——同一关节的摩擦跨位姿能差 11 倍，
按常数补偿会在轻载位姿过补成负阻尼。详见 [`CALIBRATION.md`](CALIBRATION.md) 第 10 节。

平均量（重力）的组内相对离散度只有 **3.3%**，而差值（摩擦）是 30.3%。后者不是噪声，
正是上面那条载荷相关性——位姿一换，载荷跟着换，摩擦当然跟着变。

`torque_bias` 现在才有资格导出到运行时，但**导出尚未实现**（重力表 schema 与读取端都还没有这一项）；
在此之前它被排除在重力表之外正是因为混了静摩擦，见
[unitree_g1_ros2_control/README.md](../unitree_g1_ros2_control/README.md)。

## 文件
标定结果放在包内 `config/`，**纳入版本管理**，四个文件都在点击"导出"时原子写入：
- `config/parameters.json`：源 URDF 参数、当前逐 link 标定值、采点和每轮迭代记录，外加 `ft_sensor` 段（力传感器样本与每侧结果）。schema v3；v2 的文件在加载时自动补齐这一段。
- `config/calibrated.urdf`：与源 `final.urdf` 保持相同 link/joint/mimic 结构，只替换标定后的 `mass` 和六个 inertia 分量。
- `config/gravity_table.yaml`：给运行时用的**归并刚体链**，每侧 7 个体的 `axis / origin_xyz / origin_rotation / mass / com` 平铺数组，另带 `imu_to_torso`，以及力传感器测量系相对腕 yaw 关节系的常值位姿 `payload_origin_xyz / payload_origin_rotation`。后者既是末端负载的挂载点，也让运行时能把重力方向转进测量系。它按 ROS 2 参数文件格式书写，由 `forward_position_controller` 的 `gravity_table` 参数通过 `package://arm_gravity_compensation/config/gravity_table.yaml` 读取。标定出的力矩偏置不在其中（只留在 `parameters.json`），原因见 [unitree_g1_ros2_control/README.md](../unitree_g1_ros2_control/README.md)。
- `config/ft_calibration.yaml`：每侧的 `force_bias / torque_bias / tool_mass / tool_com / measurement_origin / rotation / polarity / frame`，由 `ft_wrench_compensator` 读取。`tool_com` 对 **link 原点**取矩，`measurement_origin` 是传感器自己的力矩参考点在同一个系里的位置。

节点拿到的是安装后的 share 路径，而 `--symlink-install` 使它经 `build/` 指回源码树，写入前的 `Path.resolve()` 会跟随这条链，所以导出直接落在 `src/arm_gravity_compensation/config/` 上，符号链接也不会被替换。三个路径均可用 `parameter_file` / `calibrated_urdf` / `gravity_table` 参数覆盖；重力表的顶层键名取自 `gravity_controller_name`（默认 `arm_gravity_compensation`）。

导出后需重新加载 controller 才会生效，见 [unitree_g1_ros2_control/README.md](../unitree_g1_ros2_control/README.md)。

可通过 launch 参数覆盖 `urdf_path`、`parameter_file`、`calibrated_urdf`、`lowstate_topic`、`imu_topic`、`lowcmd_topic` 和 `port`。