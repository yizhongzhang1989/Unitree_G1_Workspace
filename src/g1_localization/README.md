# g1_localization

用头部 Livox MID-360 做世界定位，对外只给**两个**东西：

| 接口 | 类型 | 说明 |
|---|---|---|
| `~/set_origin` | `std_srvs/Trigger` | 把此刻的躯干位姿钉成世界原点。响应 `message` 带原点时间戳 |
| `~/torso_pose` | `nav_msgs/Odometry` | `world` → `torso_link` 的位姿、速度 |

外加一条可关的 TF：`world -> pelvis`（参数 `publish_tf`，默认开）。

其余都是实现细节。**将来把 Point-LIO 换成别的里程计，只改本包的输入端，
契约不动，采集侧和下游一行不用改** —— 这就是把接口做这么窄的原因。

```
/utlidar/cloud_livox_mid360 ─┐
                             ├─ head_lidar_node ─┬─ /head/lidar/points_full ─┐
/utlidar/imu_livox_mid360   ─┘                   └─ /head/lidar/imu ─────────┤
                                                                             │
                                              point_lio ◄─────────────────────┘
                                                  │ /aft_mapped_to_init (10 Hz)
                                                  ▼
                                          localization_node
                                                  │
                            ~/torso_pose  +  world→pelvis TF
```

## 跑起来

```bash
ros2 launch g1_localization localization.launch.py     # 全套
ros2 service call /g1_localization/set_origin std_srvs/srv/Trigger
```

只起里程计本体（调参、看原始输出时用）：

```bash
ros2 launch g1_localization point_lio.launch.py
```

`head_lidar_node` 由 `head_sensors` 起；`torso_link -> livox_frame` 的外参从 TF 读，
所以 `robot_state_publisher` 也得在跑。

## 两个必须知道的语义

**原点未设时 `pose.covariance[0] = -1`。** 这是 REP-145 的惯例（同 `head_lidar_node`
标 Livox 姿态无效的做法），用来区分「还没设原点」和「里程计没数据」。
**其余 35 项协方差恒为 0** —— Point-LIO 默认路径根本不填它，换算一个全零矩阵只会制造
「有不确定度估计」的假象。下游要判有效性就看这一位，**按帧判，别按「调过服务没有」判**：
调用 `set_origin` 之后，队列里的残留帧仍然带 -1。

**世界系的 z 轴铅垂。** 设原点时只冻结 yaw 和平移，roll/pitch 交给 Point-LIO 的
`gravity_align`。直接拿当时的躯干位姿当原点是不对的：那一刻的躯干倾角会被一起冻进去
（实测有 2.2°），之后所有高度和水平距离都是斜的。

## 为什么 TF 挂在 pelvis 而不是 torso_link

TF 里每个 frame 只能有一个父。`torso_link` 的父已经被 URDF 的 `waist_pitch_joint` 占了
（`robot_state_publisher` 一直在发 `waist_roll_link -> torso_link`），再发一份就是两个
publisher 抢同一个 child，**而 tf2 不保证取哪一个，且完全静默**。整棵树里没有父的
只有 `pelvis`（URDF 的根），所以那是唯一能挂世界系的位置。

**挂 pelvis 不损失精度。** 我们按 `T_world←pelvis = T_world←torso · T_torso←pelvis`
发布，下游查 `world -> torso_link` 时 tf2 会用同一份腰角走回来，两者精确抵消 ——
前提是**两边取同一个时间戳**，所以查腰角一律用里程计消息自己的 stamp，不用「当前时刻」。
实测 48% 的帧逐位相等，p50 1.14 mm / p95 4.78 mm，残差与里程计自身 ±5 mm 的抖动同量级。

改 URDF 把根换成 `torso_link` 是**不行**的：三个腰关节 parent/child 对调后 `<axis>`
要换到新父系表达、转角符号会翻，而 `/joint_states` 的腰角来自电机编码器、符号不会跟着变；
更要命的是下肢 ONNX 策略拿官方 URDF 训的，`waist_*` 都在 `policy_joints` 里。
高代价换零收益（TF 查询结果本来就一样）。

## 末端的世界位姿：不在这里发

`/joint_states` 已经在录，离线拿 `~/torso_pose` 乘一遍正运动学就有了，在线再发一路是重复。
在线要用的话直接 `lookup_transform('world', 'left_gripper_base', stamp)` —— TF 会把
`world → pelvis → waist×3 → torso → shoulder×3 → elbow → wrist×3 → gripper_base`
整条链串好，还能问过去某个时刻（比如「这张图拍的时候手在哪」）。

## 精度（2026-08-25 实测，吊绳吊着 + 双脚落地）

静止 20 s：**最大漂移 8 mm，姿态 0.109°**（0.5 m 力臂上约 0.9 mm），漂移率 0.19 mm/s。

作为对照，真实遥操作 session 里躯干晃动 p95 是 **7.31°（折到 0.5 m 力臂 = 63.8 mm）**，
所以里程计噪声比信号小一个量级。其中大部分是腰关节贡献的（骨盆只晃 1.59°），
**那部分编码器就能反解**；雷达真正不可替代的是骨盆的 yaw 漂移和平移。

## 配置里几个别乱改的地方

`config/point_lio_g1.yaml` 每一项都有注释，这里只点最容易踩的：

- **`lidar_type: 2`（Velodyne）不能改成 1（AVIA）**。上游把 AVIA 分支连同
  `livox_ros_driver2` 一起注释掉了，`switch(lidar_type)` 里只剩 OUST64/VELO16/HESAI/UNILIDAR，
  填 1 一个点都收不到。上游自带的 `config/mid360.yaml` 正是填的 1，**别直接拿来用**。
- **`satu_acc: 29.42` 不是笔误**。饱和判定用的是归一化**前**的原始读数
  （`Estimator.cpp` 里先判 `fabs(acc_avr(i)) >= 0.99*satu_acc`，之后才乘 `G_m_s2/acc_norm`）。
  我们喂的 `/head/lidar/imu` 已经是 m/s²，静止就是 9.81；照抄上游的 `3.0` 会**开机即误判
  IMU 饱和**。29.42 = 3 g × 9.80665。改喂原始的 g 单位话题就要连 `acc_norm` 一起改回去。
- **`blind: 1.0`** 剔的是雷达自己所在的头部外壳（实测 0~0.3 m 有 6.14% 的点，
  平均距离 0.154 m）。这些点刚性固连、在雷达系里永远静止，会把里程计锚死。
  代价接近零：实测 0.3~1.5 m 之间总共只有 0.33% 的点。
- **`publish_odometry_without_downsample: false`** 见配置里的注释。

## 已知限制

- **正前方 az ∈ [-45°, +45°] 是盲区**（实测 75° 完全无回波，推测头部外壳遮挡），
  水平只有 285° 覆盖。桌面操作时机器人面朝的方向恰好看不见。定位靠侧后方的几何，
  室内够用；**未来要边移动边操作，前方无感知是导航侧的隐患**。
- **纯里程计，没有回环检测**，长时间会漂。每次开录重设一次原点即可，
  不要跨 session 复用世界系。
- 空旷大厅或全白墙会让里程计退化，而**协方差全 0 给不出预警**，
  只能靠静止漂移和回原点重复性事后判断。
- `mid360_joint` 的标称值比实物偏约 3°（2026-08-14 为迁就相机扳过头，见
  `head_sensors` 的 README）。小幅晃动下这个常量外参会一阶抵消，当下不必重标；
  **边移动边操作之前必须重标**，那时机器人走远、转角变大，抵消失效。

## 验收

```bash
colcon build --symlink-install --packages-select g1_localization
python3 -m pycodestyle --max-line-length=120 src/g1_localization/
```

实机检查项：

1. 起 `point_lio` 后**不出现** `Failed to find match for field 'time'`
2. 未调 `set_origin` 时 `pose.covariance[0]` 恒为 -1；调用后转为 0
3. 设原点瞬间 `~/torso_pose` 的位姿约等于单位（平移 0、yaw 0，只剩 roll/pitch）
4. 静止 10 分钟位置漂移在几厘米以内
5. `lookup_transform('world','torso_link')` 与 `~/torso_pose` 一致（p50 应为 0）
6. 手臂大幅摆动时，`~/torso_pose` 的姿态变化应与 `/secondary_imu` 高度相关 ——
   **不相关就说明里程计被手臂拖着走了**，回去查 `blind` 和自体点云
