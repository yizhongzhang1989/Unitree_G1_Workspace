# g1_gmt_tracking

G1 全身动作跟踪层。读一段参考动作，在 50 Hz 定时器里算出 31 轴关节位置目标，由
`forward_position_controller` 带重力补偿写到硬件。

和 `g1_motion_control` 是**并列的一层，不能同时启动**——两者都往
`/forward_position_controller/commands` 写，同时跑就是两个策略抢同一组电机。

| | `g1_motion_control` | `g1_gmt_tracking`（本包） |
|---|---|---|
| 下肢 15 轴 | ONNX 策略 | 同属策略的 29 轴 |
| 上肢 14 轴 | 双臂 IK | 同属策略的 29 轴 |
| 夹爪 2 轴 | 透传 | 透传 |
| 输入 | `vx/vy/w/h` + 末端位姿 | 一段参考动作 NPZ |

`config/policy.onnx` 当前是 `g1_gloria_gmt/2026-08-14_18-57-53_scratch/model_139999`：
24.19 h 语料从零训 140000 迭代，GRU(64) + MLP[512,256]，观测 866 维、动作 29 维。

## 换动作不需要换策略

策略是 GMT 式通用跟踪器（[arXiv:2506.14770](https://arxiv.org/abs/2506.14770) 的方法，
用 mjlab 复现，训练代码在 `~/g1_lower_rl`）。它吃的不是绝对轨迹，而是**偏航与平移不变**的
前瞻特征——未来 10 个时间点（最远 1.9 秒）在根坐标系下的离地高度、重力方向、线/角速度
和关节角。所以同一段动作放在场地哪个位置、朝哪个方向，喂给网络的数字完全一样。

结果就是：**往 `config/motions/` 里丢一个同格式的 NPZ，就能放一段新动作**，不用重训、
不用重新导出 ONNX。`test/test_gmt_runtime.py::test_lookahead_is_yaw_and_translation_invariant`
把这条不变性钉成了单测。

唯一的硬约束是**帧率必须是 50 fps**：播放是一个控制拍推进一帧，帧率不对就是变速播放，
而观测里的参考速度还是按原帧率算的。节点在加载时就会拒绝。

参考动作用训练侧的 `~/g1_lower_rl/scripts/build_corpus.py` 从 CSV 生成，格式是
`joint_pos (T,31)` / `body_pos_w (T,44,3)` / `body_quat_w (T,44,4)` 加线角速度。

## 启动

### 1. 先起控制栈

```bash
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual
```

这条会把 `forward_position_controller` 加载成 **inactive**。本层在 `~/engage` 时才去
激活它，急停时反激活——所以起控制栈这一步不会让机器人动。

### 2. 起跟踪层

```bash
ros2 launch g1_gmt_tracking gmt_tracking.launch.py
```

可选参数（留空则用 `config/gmt_tracking.yaml` 里的值）：

```bash
ros2 launch g1_gmt_tracking gmt_tracking.launch.py \
  motion:=walk1_slow \
  loop:=true \
  policy_path:=/home/ruigangli/g1_lower_rl/export/policy.onnx
```

### 3. 挂好机器人，然后开操作台

```bash
ros2 run g1_gmt_tracking teleop_keyboard
```

| 键 | 作用 |
|---|---|
| `G` | engage：激活控制器。此时机器人保持当前位形，不动。 |
| `Enter` | start：从实测位形插值到参考动作第 0 帧（默认 3 秒），插完自动开始放 |
| `空格` | estop：反激活控制器，机器人进入阻尼 |
| `←` / `→` | 上一段 / 下一段参考动作 |
| `1`~`9` | 直接选第 N 段 |
| `Q` | 退出（**退出前自动急停**——这个终端是操作员唯一的停机入口） |

动作名单是问跟踪节点要它的 `motion_dir` 参数再列目录得到的，所以往 `config/motions/`
里丢新 NPZ 之后这里不用改。RUNNING 中不允许换动作，会被拒绝——中途换等于让策略瞬间
面对一个不连续的参考。

急停后不需要重启节点：再次按 `G` 重新激活控制器，成功后状态回到 `IDLE`；再按
`Enter`，必须重新走一遍从实测位形到参考第 0 帧的插值，不能直接续播急停前的帧。

**第一次上机务必吊装**，并且先用 `proc_stand` 这类准静态动作试，不要一上来就放舞蹈。

不开操作台时的等价命令：

```bash
ros2 service call /gmt_tracking/engage std_srvs/srv/Trigger
ros2 service call /gmt_tracking/start std_srvs/srv/Trigger
ros2 service call /gmt_tracking/estop std_srvs/srv/Trigger
ros2 topic pub --once /gmt_tracking/select_motion std_msgs/msg/String "{data: walk1_slow}"
```

### 4. 看状态

```bash
ros2 topic echo /gmt_tracking/status
# state=running motion=proc_stand frame=137/300
```

## 自带的参考动作

除 `proc_stand` 外都是 **LAFAN1 真实动捕**（Unitree 官方重定向到 G1 29 轴的版本，
公开镜像 `lvhaidong/LAFAN1_Retargeting_Dataset`），不是合成动作。全部 50 fps。

| 名称 | 时长 | 旧权重存活 | 旧权重体位误差 | 说明 |
|---|---|---|---|---|
| `proc_stand` | 6.0 s | 跟满 | 0.009 m | 站立不动。**首次上机用这个。** |
| `walk1_slow` | 392 s | 45 s | 0.061 m | 行走，放慢 1.5×。**真实动作里最稳的。** |
| `walk3_slow` | 370 s | 42 s | 0.057 m | 行走，放慢 1.5× |
| `walk1` | 261 s | 40 s | 0.059 m | 同 `walk1_slow` 的原速版 |
| `walk2` | 238 s | 32 s | 0.067 m | 行走，原速 |
| `dance2_slow` | 339 s | 22 s | 0.067 m | 舞蹈，放慢 1.5×。手臂动作多，最先失稳。 |

**上表是旧权重的数字，当前权重没在这 6 段上重测。** 新权重的训练语料已经含 LAFAN1，
不再是零样本，固定基准 `bench_lafan1` 上 walk 类均值存活 128.9 s（上一代 92.3 s）。
实机应当好于上表，但好多少没有实测背书，别拿它当验收依据。

三件必须知道的事：

**存活远短于动作全长。** `loop` 默认 false，放到失稳为止；实机上请随时准备 estop，
不要指望放完整段。

**放完后节点回 IDLE 就不再发指令**，FPC 保持最后一帧的位形，**此时已经没有任何平衡
控制在跑**。`proc_stand` 的末帧是站姿所以没事，行走类的末帧可能是单脚支撑，会直接倒。

**放慢 1.5× 是分布外的。** 训练时的播放倍率随机化只到 0.8，`_slow` 相当于 0.667。
方向是“更慢更容易”，风险不大，但确实不在训练分布里。

要在高动态动作上更稳只能把对应片段加进训练语料重训（`~/g1_lower_rl/scripts/build_corpus.py`）。
`run`/`sprint`/`fight` 这几类**加数据也没用**，瓶颈在手臂 25 N·m 的硬件上限（见下节）。

## 参考动作的文件格式

部署包里的 NPZ 是**瘦身格式**，只存策略真正会读的字段：

```
fps  joint_pos(T,31)  root_pos(T,3)  root_quat(T,4)
root_lin_vel(T,3)   root_ang_vel(T,3)   anchor_quat(T,4)
```

训练格式为每帧存全部 44 个刚体的位姿与速度，而部署端只读根刚体和锚刚体两个——
其余 42 个是训练时算奖励用的，上机一个字节都不会被读到。单条 6 分钟的行走因此
从 50 MB 降到 3 MB（16 倍）。

`MotionClip` 两种格式都认，所以训练产物可以直接丢进 `config/motions/` 试跑，只是文件大。
正式入库前跑一下瘦身：

```bash
cd ~/g1_lower_rl && PYTHONPATH=. python scripts/slim_motion.py \

**快速臂部动作跟不住。** Gloria-M 夹爪让手臂子树从 3.52 kg 增到 5.45 kg（+55%），绕肩
转动惯量增加 70%，而手臂电机仍是原厂 5020（25 N·m）。所以自带动作里舞蹈排在最后，
而且给的是放慢 1.5× 的版本。同样的原因，`walk4`、`fight`、`jumps`、`sprint` 这些实测
只能跟住 5~8 秒，没有放进来。放慢能改善约 20%，但改不掉根因。

**参考动作的世界系在放第 0 帧那一拍锁定。** `motion_anchor_ori_b` 含绝对偏航差，要用当时的
躯干朝向锁一次。**不能在 `~/start` 那一刻锁**：STAND 会把腰三轴插值到参考第 0 帧，而
躯干偏航 = 盆骨偏航 + `waist_yaw`，在起点锁会把整个插值量当成偏航差——而训练时这一项的
噪声只有 ±0.05（约 2.9°），`waist_yaw` 差 0.05 rad 就吃满了。

中途没有重定位，动作里机器人自己转的 90° 是动作内容会照做；`loop` 回卷到第 0 帧时重锁。

**夹爪编码器必须可读。** 观测里的 31 轴含两个 `eccentric` 轴，而且它们**夹在名单中间
（第 22、30 位）不在末尾**。`/joint_states` 里缺了它们，节点会在启动时直接报错拒绝跑；
读数错了则观测整体错位，策略输出无意义但看起来正常。

## 安全设计

* **关节顺序全部从 ONNX metadata 读**，和 `config/gmt_tracking.yaml` 里声明的对不上就
  拒绝启动。换权重忘了改配置是实机上最容易出人命的一类错误，宁可起不来也不要跑错。
* **控制周期、参考动作帧率与 ONNX 契约三者必须一致**：50 Hz 同时决定参考动作的播放
  速度，任一项对不上都直接抛。
* **躯干倾角超过 0.8 rad 急停**，与训练侧站立任务的终止条件同值。
* `/joint_states` 或 IMU 超过 0.2 s 没更新即急停。
* **手臂重力补偿是控制律的一部分，不是可选项。** 训练时就把 `tau_g / kp` 的位置偏移建进了
  动作项（只加在 14 个手臂轴上），对应 FPC 里的 `GravityFeedforward`。`compensation_scale`
  要留在 1.0 附近：训练时对补偿器增益只随机化了 0.9~1.01。
* 目标位置**故意不按关节行程裁剪**，只裁到力矩已饱和的 `ctrlrange`。理由与
  `g1_motion_control/policy_runtime.py` 完全相同：底层是 PD，关节靠到限位还想要力就得
  把目标顶到行程外，按行程裁会把撑平衡最吃力的几个关节掐掉。前 15 轴的边界与
  `motion_control.yaml` 逐位相同（同一套模型算的）。

## 测试

```bash
# 离线自测，不接机器人、不起控制栈
source /opt/ros/humble/setup.bash
cd src/g1_gmt_tracking && PYTHONPATH=.:$PYTHONPATH python3 -m pytest test/test_gmt_runtime.py -q
```

`PYTHONPATH` 后面那截 `:$PYTHONPATH` 不能省：直接写 `PYTHONPATH=.` 会把 ROS 的路径顶掉，
收集测试时就在 `import rclpy` 上炸。

`test/cross_check_obs.py` 是**观测对拍**：把部署侧装配出来的 866 维观测和训练环境里
算出来的逐位比较。观测错一位在实机上看不出来（网络照样给你 29 个有限的数），所以这条
是上机前的必过项。它需要训练环境，不在部署机上跑。

> **训练仓库主线已经不能直接跑对拍了。** `G1-Gloria-MotionTracking` 现在是 RGMT 架构
> （多组 token 观测），和本包配的 GRU 版 866 维单组不兼容。拉个只读 worktree 到 GRU 版提交：
>
> ```bash
> cd ~/g1_lower_rl && git worktree add /tmp/gmt_gru 53c97e6
> cd /tmp/gmt_gru && PYTHONPATH=.:<本包路径> MUJOCO_GL=egl \
>   python <本包路径>/test/cross_check_obs.py
> # 30 拍内最大逐元素误差 = 2.139e-07
> ```

## 重新导出策略

训练侧存的是 `.pt`，需要显式导出（mjlab 的默认 runner 不会自动导 ONNX）。**同样受 RGMT
切换影响**：`export_onnx.py` 按当前任务配置装配模型再 `load_state_dict`，拿主线加载 GRU 版
检查点会直接报不匹配。所以也要在上面那个 worktree 里导：

```bash
cd /tmp/gmt_gru && PYTHONPATH=. MUJOCO_GL=egl \
  python scripts/export_onnx.py \
    --checkpoint ~/g1_lower_rl/logs/rsl_rl/g1_gloria_gmt/<run>/model_139999.pt \
    --output-dir export
cp export/model_139999.onnx <本包路径>/config/policy.onnx
cp export/model_139999_contract.json <本包路径>/config/policy_contract.json
```

`policy_contract.json` **必须和权重一起拷**，它补两件事：① 训练循环自动存的 `policy.onnx`
没有 `lookahead_steps` / `anchor_body_name` / `all_body_names` / `control_dt`；
② ONNX metadata 的数值只到 3 位小数，`action_scale` 这种直接乘进关节目标的量取 JSON 那份。
两边都有的键会逐个比对，对不上直接拒绝启动——防的就是拿 A 的契约配 B 的权重。
