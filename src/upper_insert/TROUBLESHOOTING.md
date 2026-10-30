# `upper_insert` 真机调试问题排查与解决方案汇总
本文档记录 `upper_insert` 与 `g1_motion_session` 在 Unitree G1 真机插孔调试过程中遇到的全部典型问题、根因分析（区分**代码 Bug**、**算法缺陷/优化**与**仿真-真机机电特性差异**）及最终解决方案。

## 总览表

| # | 现场现象 | 问题分类 | 核心根因 | 解决方案与涉及文件 |
|---|---|---|---|---|
| 1 | 手臂关节/IK 控制卡顿，浮动模式手拉也卡顿 | 系统调度与 GIL 竞争 | 视觉/力觉高频回调与 50 Hz 控制环争抢 Python GIL 及执行器，阻塞 500 Hz 底层插值 | 共享内存传图、C++/CUDA 释放 GIL、净力与原始力回调组隔离 ([insert_node.py](upper_insert/insert_node.py)) |
| 2 | 执行手眼标定立即报错 `rclpy InvalidHandle` 退出 | 代码 Bug（生命周期竞态） | 标定采集临时订阅在回调尚在执行队列中时调用 `destroy_subscription` | 仅停止并销毁临时节点与独立执行器，避免重入销毁句柄 ([insert_node.py](upper_insert/insert_node.py)) |
| 3 | 刚完成手眼标定，点击“机械插入”仍总是向前偏 ~18 mm | 算法缺陷（IK 求解精度） | 旧版单步 `standard` IK 零空间偏好泄漏与腕部触界截断产生 +22.8 mm 求解前偏 | 用高精度 IK（20 次迭代 + 严格零空间投影 + 窗口内 SE(3) 缩步）完全替代旧版 IK ([arm_ik_native.cpp](../g1_motion_session/src/arm_ik_native.cpp)) |
| 4 | 插头靠近孔口遮挡 `ID0` 时视觉掉到 14 Hz；顶光反光下 `ID1` 漏检抖动 | 算法优化（视觉提速与抗反光） | 遮挡静态 `ID0` 时仍跑全图多通道恢复（50 ms/帧）；顶光反射使黑格饱和度升至 126、外白框进 `rejected` | 缓存静态 `ID0` 仅检 `ID1`、局部跟踪 ROI 快路径、`rejected` 外白框按 `30/35` 内缩解码 ([vision_locator.py](upper_insert/vision/vision_locator.py)) |
| 5 | 夹爪频繁掉线报错 `gripper is not enabled` | 代码 Bug（超时恢复缺失） | 启动使能成功后即取消定时器；CAN 总线偶发超时触发失能后不再重发使能 | 超时失能后自动重激活使能定时器，控制侧下发夹爪指令时异步补发 `enable` ([gripper_node.py](../gloria_ros/gloria_ros/gripper_node.py)) |
| 6 | 点击“机械插入”偶尔完全不动（`task_fraction = 0`） | 代码 Bug（双腕触界死锁） | `wrist_roll` 与 `wrist_yaw` 同时贴限位且 `shoulder_roll` 顶住躯干；`reference` 救援种子被局部步长窗裁剪且跨拍丢失 | C++ 高精度 IK 对窗口外救援种子不裁剪，Python 侧锁存 `_rescue_target` 直至脱离死区 ([arm_ik_native.cpp](../g1_motion_session/src/arm_ik_native.cpp), [arm_ik.py](../g1_motion_session/g1_motion_session/arm_ik.py)) |
| 7 | 策略插入时手臂以 ~3 Hz 围绕孔口画圈乱晃 | 延迟 + 力臂失配 Bug | ① 相机存在 40~80 ms 延迟；② `p_cmd = p_tip_cmd - R_tg @ p_gt` 混用实测腕角 $R_{tg}$ 与指令腕角 $R_{\text{cmd}}$，经 179 mm 力臂产生反相自激振荡 | ① 500 Hz `/joint_states` 历史缓冲 + 差分 FK 延迟补偿与互补滤波；② 严格使用刚体恒等式 `p_cmd = p_tip_cmd - R_cmd_new @ p_gt` ([insert_node.py](upper_insert/insert_node.py)) |
| 8 | 节点运行十余分钟后策略插入“姿势不变、一路前冲” | 代码 Bug（$SO(3)$ 非正交累积） | `to_dict()` 将 $R_{GT}$ 截断为 5 位小数（$R_{GT}^T R_{GT} \neq I$），帧间 FK 滤波每帧累乘未重正交化，20 分钟后 $R_{HT}$ 畸变为 $\det=2.13$ 的剪切矩阵，将 `-Z_H` 下插偏转为 `+X_torso` 前冲 | 保留 8 位小数精度，并在 `_compensate_vision_with_fk` 与 `_fuse_vision_delta_with_fk` 中每步用 SVD 投影回严格正交群 $SO(3)$ ([insert_node.py](upper_insert/insert_node.py), [tool_calibration.py](upper_insert/tool_calibration.py)) |
| 9 | 策略连续执行“越走越远后卡死” & “动一下直接把手放平” | 机电特性差异 + 零空间过优化 | ① 真机存在重力/摩擦静差 $\delta_{\text{droop}} = p_{\text{cmd}} - p_{\text{meas}}$，每拍用 `p_meas + dp` 重置目标导致 $\Delta p_{\text{cmd}} = \Delta p - \delta_{\text{droop}}$ 反向滑移并在 $\Delta p = \delta_{\text{droop}}$ 处死区卡死；② `shoulder_roll` 偏好未加门限死区，20 步零空间迭代把 `elbow` 伸直、`wrist_pitch` 展平 | ① 在目标尖端 `(p_tip_cmd, R_tip_cmd)` 上累加策略步长并加 XY/Z 独立防积分饱和限幅（`12 mm / 4°`）；② 为 `shoulder_roll` 启用相同的 `[0.6, 1.2] rad` 零空间门限死区 ([insert_node.py](upper_insert/insert_node.py)) |
| 10 | 策略首拍未靠近孔口 & 严格零空间优化下手肘撞击躯干 | 算法缺陷（限速裁剪与零空间耦合） | ① 首拍含 `9.36°` 旋转绕 `176 mm` 尖端折算为夹爪 `30.21 mm` 横移，求完再逐轴限速裁剪破坏尖端运动方向；② 7 轴臂锁死 6D 位姿后零空间仅 1 维，`wrist_roll` 回零偏好与 `shoulder_roll` 外展反向，将右肘拉向躯干约 `75 mm` | ① 将 `seed ± rate_limit * dt` 窗口施加于整个 IK 并在超窗时沿 SE(3) 路径等比缩步（`task_fraction`）；② 在 [upper_insert.yaml](config/upper_insert.yaml) 中将 `wrist_roll` 零空间增益置 `0.0` |
| 11 | 切入策略插入后一直往下走且视觉卡死 | 硬件闪断恢复缺失 | 外置 Azure Kinect 发生 USB 闪断重枚举，旧 `/dev/video6 (deleted)` 未释放，新设备变为 `/dev/video7`，视觉停留在旧帧 | `AsyncCameraReader` 连续 `grab()` 失败后自动重扫 V4L2 节点重连，且策略在视觉超时/无效时冻结目标 ([vision_locator.py](upper_insert/vision/vision_locator.py), [insert_node.py](upper_insert/insert_node.py)) |
| 12 | 进入手部力传感器 6 姿态重力标定时机身剧烈抖动 | 多节点控制竞争与前馈串扰 | ① `auto_ft_calibration` 以 100 Hz 发布全 31 轴 `commands` 与全身会话争抢腰腿 15 轴；② `GravityFeedforward` 全身共用标量 `target_hold_time_`，上下肢异步更新互相清零计时器制造速度尖峰 | ① 校准节点改向 `joint_targets` 发布仅含双臂 14 轴的掩码目标；② `target_hold_time_` 改为逐关节独立数组 ([auto_ft_calibration.py](../arm_gravity_compensation/arm_gravity_compensation/auto_ft_calibration.py), [gravity_feedforward.cpp](../unitree_g1_ros2_control/src/gravity_feedforward.cpp)) |


## 详细问题分析与解决过程

### 1. 手臂控制与浮动拖动卡顿
- **现象**：关节控制、IK 控制不平滑，甚至在 `float`（浮动）模式下用手拉动机械臂都一顿一顿。
- **根因**：
  1. `float` 模式并非零力矩模式，而是以 50 Hz 将实测关节角 `/joint_states` 回填发布给底层位置控制器（FPC），一旦 50 Hz 定时器被阻塞产生 30~50 ms 抖动，手拉时就会感到明显顿挫；
  2. 1280×720 图像跨进程 pickle 序列化、Python 遮挡恢复循环以及力传感器 raw 备用补偿回调争抢主进程 GIL。
- **解决**：
  - 使用 `SharedMemory` 单帧零拷贝传递相机图像，灰度和饱和度预处理移至 CUDA，候选轮廓搜索移至释放 GIL 的 C++ 扩展（`vision_native.cpp`）；
  - 将 `/arm{0,1}/wrench_net` 与 `/arm{0,1}/wrench_raw` 隔离到独立回调组，当 `wrench_net` 正常时跳过 `wrench_raw` 的 Pinocchio FK 重力补偿计算。

### 2. 手眼标定报错 `rclpy._rclpy_pybind11.InvalidHandle`
- **现象**：点击主相机手眼标定后立即抛出 `InvalidHandle: cannot use Destroyable because destruction was requested` 并退出进程。
- **根因**：标定采集完成后，在单线程执行器仍在处理订阅队列时对同一个节点句柄调用了 `destroy_subscription`。
- **解决**：标定采集使用独立的临时 `Node` 与 `SingleThreadedExecutor`，采集结束后直接退出执行器循环并销毁临时节点，避免在回调执行中途销毁订阅句柄。

### 3. 机械插入总是向前偏 ~18 mm
- **现象**：刚完成主相机手眼标定，点击“机械插入”移动到 `rect_tight` 上方时，插头尖端总是比目标孔位靠前（`+X_torso`）约 17~18 mm。
- **根因**：
  - 逐项核对现场 `/api/state` 与 `/joint_states` 发现：手眼标定本身准确，`mech_insert` 计算出的 `goal_tip` 也在孔口正上方；
  - 但 `upper_insert.yaml` 未显式指定 `ik_solver: high_precision`，节点实际运行在默认的单步 `standard` IK 下，零空间偏好项向主任务空间泄漏以及关节限速后的逐轴硬截断造成了 `+22.82 mm` 的 IK 求解前偏（扣除 `-5.27 mm` 重力下垂后净前偏 `+17.55 mm`）。
- **解决**：在 [upper_insert.yaml](config/upper_insert.yaml) 中默认设置 `ik_solver: high_precision`，使主任务收敛至 `0.02 mm` 容差内。

### 4. 遮挡 `ID0` 时视觉掉帧 & 顶光反射下 `ID1` 漏检抖动
- **现象**：
  1. 插头移动到孔口上方遮挡住孔板 `ID0` 码时，视觉帧率从 30 Hz 掉到 14~18 Hz（单帧耗时 49.5 ms）；
  2. 现场冷光顶灯照射下，插头上的 `ID1` 码频繁闪烁识别不出。
- **根因**：
  1. 虽然 `vision_worker.py` 已缓存静态孔板 `ID0` 的位姿 `_last_id0_pose`，但每帧仍传入 `required_ids=(0, 1)`，导致 `ID0` 被遮挡时每帧都在全图跑耗时的多尺度遮挡恢复；且局部跟踪 ROI 包含了历史 `ID0` 坐标导致 ROI 过大；
  2. 顶光反射下 `ID1` 黑色编码格的 HSV 饱和度升至 `115~126`（超过原阈值 `75/85`），且左下角反光使内黑框断裂，OpenCV 标准检测器只能检出 `35 mm` 外白框并将其放入 `rejected`。
- **解决**：
  1. 当 `_last_id0_pose` 已缓存时传入 `required_ids=(1,)`，且局部 ROI 仅围绕 `req_tuple` 构建；
  2. 将黑格饱和度门限放宽至 `145`，并在局部 ROI 和全图检测中直接对 `rejected` 候选外框按 `30/35` 比例内缩解码（`_ingest_standard_detections`），使遮挡 `ID0` 时的单帧耗时降至 **6.3~9.5 ms（稳定 30 FPS）**。

### 5. 夹爪反复报 `gripper is not enabled` 掉线
- **现象**：运行过程中操作夹爪报错 `gripper is not enabled`。
- **根因**：`gripper_node.py` 启动使能成功后即调用 `self._enable_timer.cancel()` 永久关闭了使能定时器；当 CAN 总线偶发通信超时触发 `_poll_state` 将 `self._enabled` 置为 `False` 后，再也不会重发使能指令。
- **解决**：在 [gripper_node.py](../gloria_ros/gloria_ros/gripper_node.py) 的 `_poll_state` 中，若因超时失能且 `enable_on_start=True`，自动重激活 `_auto_enable_pending` 并重置定时器；同时在 `insert_node.py` 的 `set_gripper` 中异步调用 `/grip_arm{0,1}/enable`。

### 6. 点击“机械插入”偶尔完全不动（双腕触界死锁）
- **现象**：手臂处于某些浮动姿态时点击“机械插入”，机械臂分毫不动，`ik_null_status` 显示 `task_fraction = 0.0`。
- **根因**：
  - 现场关节角显示 `right_wrist_roll = -1.963 rad`（下限 `-1.972 rad`）、`right_wrist_yaw = +1.569 rad`（上限 `+1.571 rad`）双腕同时贴死限位，且 `right_shoulder_roll = +0.583 rad` 向内顶住躯干；
  - 局部关节窗口内无法在不越界的情况下产生指向目标的步长（`task_fraction = 0.0`）；而 `ArmIK._step` 调用 `solve(self.reference, targets)` 尝试全局救援时，`highPrecision` 仍用当前死锁种子的局部窗口裁剪 `reference` 救援种子，且下一拍又丢弃了救援目标。
- **解决**：在 [arm_ik_native.cpp](../g1_motion_session/src/arm_ik_native.cpp) 中，当救援种子超出局部窗口时不施加局部窗口约束；在 [arm_ik.py](../g1_motion_session/g1_motion_session/arm_ik.py) 中跨拍锁存 `_rescue_target` 直至脱离限位死区。

### 7. 策略插入时手臂 3 Hz 画圈乱晃
- **现象**：开启 `policy_insert` 后，插头在孔口上方围绕孔心做周期约 `0.33 s`（3 Hz）、幅度 `±13 mm` 的画圈晃动。
- **根因**：
  1. **视觉延迟未做控制拍前推**：相机图像从曝光、传输到 PnP 解算完成存在 `40~80 ms` 延迟，策略拿到的是过去时刻的位置误差；
  2. **夹爪位置反算的力臂失配**：在反算夹爪基座位置时混用了实测手腕姿态 $R_{tg}$ 与指令手腕姿态 $R_{\text{cmd, new}}$（写成 `p_cmd_new = p_tip_virt - R_tg @ p_gt`），导致送入 IK 的目标对应的真实尖端位置变为：
     $$p_{\text{tip, IK}} = p_{\text{cmd, new}} + R_{\text{cmd, new}} p_{GT} = p_{\text{tip, virt}} + (R_{\text{cmd, new}} - R_{tg}) p_{GT}$$
     因力臂 $|p_{GT}| = 179.2\text{ mm}$，$3^\circ\sim 5^\circ$ 的腕角超前直接把 IK 目标尖端甩出 `10~15 mm`，并在手腕滞后响应时形成 3 Hz 反相自激振荡。
- **解决**：
  1. 维护 500 Hz `/joint_states` 历史队列 `_joint_history`，在视觉回路执行帧间差分 FK 互补滤波（`_fuse_vision_delta_with_fk`）及丢码保持（`_coast_vision_delta_with_fk`），并在 50 Hz 控制拍用实测差分 FK 将视觉误差前推至当前时刻（`_compensate_vision_with_fk`）；
  2. 严格满足刚体几何恒等式 `p_cmd_new = p_tip_cmd - R_cmd_new @ p_gt`，确保腕部旋转时尖端位置严格不变。

### 8. 节点运行十余分钟后“姿势不变、一路前冲”
- **现象**：节点连续运行 10~30 分钟后执行策略插入，手臂姿势不变却一路向机器人正前方（`+X_torso`）直冲出去。
- **根因**：
  - [tool_calibration.py](upper_insert/tool_calibration.py) 的 `to_dict()` 将标定旋转矩阵 `R_G_T` 用 `round(x, 5)` 截断到了 5 位小数，导致 $R_{GT}^T R_{GT}$ 的对角线元素为 `[1.0000046, 1.0000054, 1.0000090] > 1.0`；
  - 在 `_fuse_vision_delta_with_fk` 中，每帧（30 Hz）都会累乘 $dR_{\text{fk}} = R_{GT}^T R_{bg,\text{cam}}^T R_{bg,\text{now}} R_{GT}$，而 `pin.log3` 仅修正反对称旋转部分，无法消除对称部分的非正交膨胀与剪切；
  - 连续运行 20 分钟（约 40,000 帧）后，$R_{HT}$ 畸变为 $\det(R_{HT}) = 2.13$、非对角剪切项达 `0.632` 的非正交矩阵，将策略原本垂直向下的 `-Z_H` 步长在 `dp_t = R_HT.T @ dp_h` 中直接耦合偏转为向 `+X_torso`（机器人正前方）的推力。
- **解决**：`to_dict()` 保留 8 位小数精度，并在 `_compensate_vision_with_fk` 与 `_fuse_vision_delta_with_fk` 中每步通过 SVD（`_orthonormalize_rot`）将旋转矩阵严格投影回 $SO(3)$（实测 40,000 帧后正交误差为 `1.15e-15`，$\det = 1.0$）。

### 9. 策略连续执行“越走越远后卡死” & “动一下直接把手放平”
- **现象**：
  1. 若每拍直接按 `p_cmd(k+1) = p_meas(k) + dp_policy` 更新目标，手臂会先顺着重力/外侧方向越走越远，最后在离孔 `151.9 mm` 处彻底卡死不动；
  2. 在竖直插孔姿态下启动移动时，肘部突然伸直（`elbow -> +1.45 rad`）、手腕下弯角归零（`wrist_pitch -> -0.07 rad`），直接把手放平。
- **根因**：
  1. **真机 PD 静差导致 `p_meas + dp` 反向滑移与死区卡死**：
     真机底层关节 PD 控制器需要静态位置超前量 $\delta_{\text{droop}} = p_{\text{cmd}} - p_{\text{meas}}$（实测约 `2~6 mm`）来平衡残余重力与静摩擦。若每拍用 $p_{\text{meas}}(k) = p_{\text{cmd}}(k) - \delta_{\text{droop}}$ 重置指令：
     $$p_{\text{cmd}}(k+1) - p_{\text{cmd}}(k) = \Delta p_{\text{policy}} - \delta_{\text{droop}}$$
     当孔口附近 $|\Delta p_{\text{policy}}| < |\delta_{\text{droop}}|$ 时，指令反而每拍沿 $-\delta_{\text{droop}}$ 方向滑走；直到滑到远处使策略回拉步长增大到 $\Delta p_{\text{policy}} == \delta_{\text{droop}}$（实测 `[+0.72, +2.30, -0.66] mm` vs `[+0.56, +2.33, -0.69] mm`）时，$\Delta p_{\text{cmd}} = 0$，彻底卡死在平衡点。同理，若姿态超前限幅 `max_tip_lead_deg`（`1.0°`）小于手腕抗重力静差（`~3°`），手腕也会因抗重力支撑角被抹除而下垂放平。
  2. **`high_precision` IK 中 `shoulder_roll` 未加门限死区引发零空间翻转**：
     `arm_ik.yaml` 中仅 `shoulder_roll`（目标 `-0.34 rad`）未启用 `[0.6, 1.2] rad` 死区门限（`ungated = True`），其余关节在 `[-0.6, +0.6] rad` 内零空间代价为 `0`。`high_precision` IK 每拍执行 20 步零空间优化时，为了将 `shoulder_roll` 从 `-0.62 rad` 拉向 `-0.34 rad`，会把零代价的 `elbow` 推到 `+1.40 rad`（180° 伸直）并将 `wrist_pitch` 从 `+38.5°` 推到 `0°`（手腕展平）。
- **解决**：
  1. 在目标尖端位姿 `(p_tip_cmd, R_tip_cmd)` 上累加策略增量（保留克服重力与静摩擦所需的 $\delta_{\text{droop}}$），并对 `p_tip_cmd - p_tip_meas` 施加 **XY 与 Z 独立的防积分饱和限幅**（`max_tip_lead_m: 12 mm`，`max_tip_lead_deg: 4.0°`），既能在 2~3 拍内破开任意方向的静摩擦死区，又保证受阻时绝不无限累积；
  2. 在 `InsertTask._bind_arm_ik` 中为所有零空间偏好项统一启用 `[0.6, 1.2] rad` 门限死区（`ungated = False`），使正常插孔姿态下零空间梯度为 `0`，杜绝零空间自运动将手腕放平。

### 10. 策略首拍未靠近孔口 & 严格零空间优化下手肘撞击躯干
- **现象**：
  1. 在现场冻结姿态（`circle_loose`、深度 `15 mm`）下执行策略首拍，虽然策略原始平移仅 `2.58 mm`，但限速后的实际夹爪 FK 反而远离孔口（到孔口距离从 `36.54 mm` 恶化为 `40.59 mm`）；
  2. 启用严格零空间迭代优化后，右肘向机身内侧收拢约 `75 mm` 并撞击躯干。
- **根因**：
  1. **长力臂旋转触发单拍限速后逐轴硬裁剪破坏方向**：首拍动作除 `2.58 mm` 平移外还包含 `9.36°` 旋转；因尖端距夹爪基座约 `176 mm`，绕尖端旋转折算为夹爪基座 `30.21 mm` 横移，超出单拍 `0.2 rad`（`10 rad/s × 0.02 s`）限速窗口；求完完整解后再逐关节硬裁剪会破坏各轴比例与尖端运动方向；
  2. **1 维严格零空间内腕部与肩部偏好反向竞争**：7 轴机械臂在严格锁死 6D 末端位姿后，严格零空间 $\ker(J_{6\times 7})$ 仅剩 **1 个臂角自由度**；插孔姿态下 `wrist_roll`（`+0.87 ~ +1.49 rad`，原增益 `0.20`）的回零拉力与 `shoulder_roll`（外展目标 `-0.34 rad`，增益 `0.05`）在该 1 维零空间上方向相反，手腕回零主导了零空间移动并将右肘拉向躯干。
- **解决**：
  1. 将 `seed ± rate_limit * dt` 窗口约束直接施加于整个 IK 主任务与零空间求解，当完整末端目标超出单拍窗口时沿 SE(3) 螺旋路径等比缩步（`task_fraction`），使首拍执行 `50%` 位姿任务、最大关节变化 `0.168 rad < 0.2 rad`，到孔口距离从 `36.54 mm` 推进至 `35.76 mm`（纯毫米级平移则 `task_fraction = 1.0` 完整执行）；
  2. 在 [upper_insert.yaml](config/upper_insert.yaml) 中将 `wrist_roll` 零空间增益设为 `0.0`（保留 `shoulder_roll=0.05`、`shoulder_yaw=0.20`、`elbow=0.20`），使右第二关节从 `-0.368 rad` 收敛至 `-0.313 rad` 并保持肘部外展避开躯干。

### 11. 切入策略插入后一直往下走且视觉卡死
- **现象**：插头已到达洞口，切换到 `policy_insert` 后机械臂仍持续向下移动，且网页端视觉画面不再更新。
- **根因**：检查 `/proc/<pid>/fd` 发现外置 Azure Kinect 发生 USB 闪断重枚举，旧设备节点 `/dev/video6 (deleted)` 已失效而新设备变为 `/dev/video7`；`AsyncCameraReader` 持续在失效句柄上 `grab()` 失败，导致控制环一直使用闪断前的旧视觉观测。
- **解决**：在 [vision_locator.py](upper_insert/vision/vision_locator.py) 的 `AsyncCameraReader._loop` 中增加连续 `grab()` 失败自动释放并重扫 V4L2 节点重连机制；并在 `policy_insert` 中当视觉无效或超时时清空 `_policy_tip_cmd`、保持当前位置不动。

### 12. 进入手部力传感器 6 姿态重力标定时机身剧烈抖动
- **现象**：`float` 零力拖动时机器人平稳，但点击“手部力传感器重力校准 (6姿态)”后机器人全身剧烈抖动。
- **根因**：
  1. `auto_ft_calibration.py` 向 `/forward_position_controller/commands` 发送全 31 轴 `Float64MultiArray`，以 100 Hz 将腰腿 15 轴改写为启动瞬间的下垂实测角，与 `motion_session` 的 50 Hz 站立目标每秒争抢 150 次；
  2. `GravityFeedforward::update_target_velocity` 的 `target_hold_time_` 为全机身共用单标量，上下肢异步更新互相清零计时器，在摩擦前馈中制造 5~10 倍速度尖峰。
- **解决**：`auto_ft_calibration.py` 改为向 `/forward_position_controller/joint_targets` 发送仅含双臂 14 轴掩码的 `JointTargets`；`GravityFeedforward` 将 `target_hold_time_` 改为逐关节独立的 `std::vector<double>`。
