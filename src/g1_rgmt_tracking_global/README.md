# g1_rgmt_tracking_global

G1 全身动作跟踪层：50 Hz 输出 31 轴关节位置目标。
29 轴（12 腿 + 3 腰 + 14 臂）由 RGMT 策略驱动，2 个夹爪偏心轴由 VR
手柄 trigger 控制。

参考动作二选一，`reference_source`：

| 值 | 参考从哪来 | 有没有终点 |
|---|---|---|
| `motion`（默认） | `config/motions` 下录好的 NPZ | 放完回 IDLE |
| `mocap` | PICO 全身动捕实时流（[`g1_mocap`](../g1_mocap/README.md)） | 没有，只能 estop |

**不要和 `g1_motion_control` 或 `g1_gmt_tracking` 同时启动**——三者都往
`/forward_position_controller/commands` 写，同时跑就是多个策略抢同一组电机。

## 为什么叫 `_global`

RGMT 的参考窗口每个 token 68 维，后 30 维是**参考 key body 相对机器人躯干**的位移和速度：

```
rel   = ref_pos_w - robot_anchor_pos_w
local = quat_apply(quat_inv(yaw_quat(robot_anchor_quat)), rel)
```

`reference_key_bodies` 的第一个是 `torso_link`，也就是 anchor 自己，所以**那 3 维就是漂移量本身**——机器人跟得完美时恒为 0，漂了多少它就是多少。这是策略唯一的位置误差输入通道，去掉就退回开环（250 s 漂移从 0.29 m 回到 5.9 m）。

代价是**必须知道躯干的世界位置**。旧的 `g1_gmt_tracking` 完全回避了这一点（它的前瞻特征对偏航与平移全不变），本包不行。

> 不需要绝对定位。`rel` 是两个世界位置之差，对原点平移不变，所以只要
> **参考和机器人处在同一坐标系**即可——这靠 `~/start` 时刻的一次对齐建立。

## 三个量分属两个刚体，别搞混

`pelvis` 和 `torso_link` 只差 4.4 cm 平移加腰三轴转角，很容易被"顺手统一"，但取错**不会报错**，只会让策略完全失效：

| 观测 | 刚体 | 真机来源 |
|---|---|---|
| `projected_gravity` | **pelvis** | IMU 四元数直接投影，**不要做腰部 FK** |
| `base_ang_vel` | **pelvis** | IMU 角速度直接用 |
| key body 局部化 / 倾角保护 | **torso_link** | IMU + 腰三轴 FK；局部化姿态再乘定位 yaw 修正 |
| 里程计位置 | **torso_link** | `/dog_odom` 给盆骨，需 FK；雷达直接给躯干 |

依据在 `mjlab/entity/data.py:586`：

```python
def projected_gravity_b(self):
    return quat_apply_inverse(self.root_link_quat_w, self.gravity_vec_w)
```

`root_link` 是自由关节所在的 body，也就是 `pelvis`。另外 `gravity_vec_w` 在
`entity.py:809` 写死为**归一化**的 `[0, 0, -1]`，不是 $-9.81$——用重力加速度实际值会差 9.81 倍。

`test_projected_gravity_uses_base_not_anchor` 把这两条都钉死了。

## 里程计

两路来源，`odometry_mode` 三选一：

| 模式 | 组成 | 适用 |
|---|---|---|
| `fused`（默认） | 雷达 10 Hz × `/dog_odom` 500 Hz | 推荐 |
| `odom_only` | 只用 `/dog_odom` | 没起定位栈时，**仅限短动作** |
| `lidar_only` | 只用雷达 | 仅供对照，10 Hz 喂 50 Hz 会有台阶 |

融合式：

```
T_world_torso(t) = T_world_odom(t_k) @ T_odom_torso(t)
```

左项是 odom 的累积漂移，属慢变量，滤得狠也不引入动态滞后；全部动态由 500 Hz 快通道承担。

三个实现要点：

- 雷达 stamp 比 odom 滞后约 34 ms，**必须按 stamp 回溯匹配**同一时刻的 odom，直接和当前值相除会把这 34 ms 的运动算成漂移
- `/dog_odom` 订阅**必须 `depth=1`**，实测 `depth=50` 时接收时刻恒定滞后约 48 ms，静默不报错
- `/dog_odom` 给的是盆骨，本包用腰三轴把位置和姿态都 FK 到 `torso_link`；雷达与快通道必须是同一个刚体，否则腰偏航会被误算成定位 yaw 漂移
- 融合输出的位置和 anchor 姿态必须乘同一个定位 yaw 修正；只旋位置会让 key body 的局部误差整体转错

## 已知风险：漂移会被策略当真

失效模式是**正反馈**：

```
里程计慢漂 → 策略以为自己漂了 → 主动往回纠 → 真的走偏 → 读数更大
```

训练时这一路只加了 ±2 cm 的逐帧白噪声（上游 BeyondMimic 对同类项用的是 **±25 cm**），所以对慢漂移的容忍度偏低。两道防线：

1. `max_anchor_offset_m`（默认 0.3 m）钳住 anchor 那 3 维，超限退化成开环而不是把机器人推倒。`~/status` 里出现 `CLAMPED` 就是触发了
2. 首次上机用**短片段**，随包的三段都截到 15 s

## 用法

本 launch **只起 tracking_node 一个节点**。默认 `odometry_mode: fused` 依赖一条四级链路，
每一级都得自己先起：

```
/utlidar/cloud_livox_mid360 -> head_lidar_node -> /head/lidar/points_full
    -> point_lio -> /aft_mapped_to_init -> localization_node -> ~/torso_pose
```

按顺序起，前四步各占一个终端：

```bash
# 1. 控制栈。把 FPC 加载成 inactive，同时提供 /joint_states 与 TF
#    （定位层要拿腰角把 lio 结果推到 torso_link，所以它得在第 3 步之前）
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual

# 1b. 关掉 FPC 的手臂自适应刚度：策略训练时是纯 PD，YAML 默认的 2.0 会把手臂
#     零误差处的 kp 抬到 3 倍。all_data.launch.py 没有对应的 launch 参数，
#     只能起完再设；FPC 此时还是 inactive，engage 之前设完即可
ros2 param set /forward_position_controller adaptive_stiffness_scale 0.0

# 2. 头部传感器。**它不在控制栈里，漏了就没有 /head/lidar/points_full**，
#    表现是 localization_node 刷「没有里程计输入」
ros2 launch head_sensors head_sensors.launch.py camera:=false

# 3. 雷达定位栈（Point-LIO + 接口层）。不用手工钉原点：本层每次 engage 都会自动调用
#    /g1_localization/set_origin；定位未就绪或调用失败时 engage 会被拒绝
ros2 launch g1_localization localization.launch.py

# 4. 本层
ros2 launch g1_rgmt_tracking_global rgmt_tracking.launch.py

# 5. 操作台（终端是唯一停机入口，退出时自动 estop）
ros2 run g1_rgmt_tracking_global teleop_keyboard
```

只想跑一下、不想起雷达的话，第 2/3 步可以整个省掉，但**必须换模式**，
否则 `lidar_timeout_s` 到点就急停：

```bash
ros2 launch g1_rgmt_tracking_global rgmt_tracking.launch.py odometry_mode:=odom_only
```

调试时也可以让定位栈代起一个雷达节点（代替第 2 步，但不出相机）：
`ros2 launch g1_localization localization.launch.py with_lidar:=true`。

链路断了就逐级往上查，哪一条没频率就是断在那一级：

| 话题 | 应有频率 | 没有则缺 |
|---|---|---|
| `/utlidar/cloud_livox_mid360` | 10 Hz | 雷达或机器人自带驱动 |
| `/head/lidar/points_full` | 10 Hz | 第 2 步 head_sensors |
| `/aft_mapped_to_init` | 10 Hz | 第 3 步 point_lio |
| `/g1_localization/torso_pose` | 10 Hz | 第 3 步 localization_node |
| `/dog_odom` | 500 Hz | 机器人自带里程计 |

状态机与 `g1_motion_control` 一致：

```
IDLE --engage--> (激活 FPC) --start--> STAND --(插值到位)--> RUNNING
                                            任何时候 estop --> ESTOP
```

录制 motion 在 `~/start` 时按偏航和水平平移对齐到机器人当前位姿；高度不对齐，因为
参考的离地高度属于动作内容。实时 mocap 的高度规则见下一段。

实时 mocap 的垂直原点不能直接把 torso z 贴到一起：人和机器人可能屈膝或弯腰，二者
torso 到地面的高度并不相同。当前做法是分别按当拍关节姿态做 FK，取左右
`ankle_roll_link` 中沿重力方向的最低点并令两者重合；XY 和 yaw 仍按 torso 对齐。
localization 的 `world` 本身以雷达 RANSAC 物理地面为 `z=0`，因此参考和实机最终落在同一
地面坐标系。录制 motion 的离地高度属于动作内容，不走这条实时对齐规则。

`~/status` 每 100 ms 一条：

```
state=running motion=walk1_subject1 frame=253/750 offset=0.081 drift=0.012
```

- `offset`：参考锚点与机器人躯干的距离，也就是策略读到的漂移量
- `drift`：里程计被雷达修正的累积量，持续增长说明 odom 在漂

## 上机前必过

```bash
cd src/g1_rgmt_tracking_global && python3 -m pytest test/ -q
```

其中 `test_reference_window_matches_training` 把参考窗口和训练侧 `reference_tokens`
的公式逐位对拍。**这项不过绝对不要上机**：观测错位不报错，只会让策略输出看起来正常但完全无意义的动作。

## 换动作

```bash
python scripts/slim_motion.py --input-dir <训练NPZ目录> \
    --output-dir config/motions \
    --contract config/policy_contract.json --max-seconds 15
```

瘦身格式比旧包多三个字段：`anchor_pos`（对齐用）、`key_pos`、`key_lin_vel`（参考窗口后 30 维的来源）。**key body 的顺序必须跟契约走**，重排了就是静默错位。

运行时切换：`ros2 topic pub --once ~/select_motion std_msgs/String "data: walk2_subject1"`。RUNNING 中拒绝切换。

## 用实时动捕当参考

把 NPZ 换成人：戴 PICO 4 Ultra + 5 个 Motion Tracker，机器人实时跟着走。收头显、跑重定向、
做校准全在 [`g1_mocap`](../g1_mocap/README.md)，本层只订它的 `/mocap/frame`，把那条流装配成
68 维的参考窗口。**本层不碰头显**，所以三个节点可以同时跑。

```bash
# 前四步同上（控制栈 / head_sensors / 定位栈 / 钉原点），然后：

# a. 动捕数据源。头显在配置面板里填「机器人IP:18000」，全程 WiFi，不用 adb
ros2 launch g1_mocap mocap.launch.py

# b. 人站直，校准人机比例。三选一：戴着头显按双摇杆（会震动回执）／面板上点／服务
ros2 service call /mocap/calibrate std_srvs/srv/Trigger

# c. 可选但建议：先在面板上看一遍重定向出来的姿态对不对
ros2 launch g1_mocap dashboard.launch.py     # http://<机器人IP>:18080

# d. 本层
ros2 launch g1_rgmt_tracking_global rgmt_tracking.launch.py reference_source:=mocap

# e. 操作台：G = engage，Enter = start，空格 = estop
ros2 run g1_rgmt_tracking_global teleop_keyboard

# e'. 或者用手柄推状态机——戴着头显看不见屏幕时用这个
ros2 run g1_rgmt_tracking_global mocap_teleop
```

运行中两只手柄分工如下：

- `trigger`：各自控制同侧夹爪，0 = 完全打开，1 = 完全闭合
- `start` 后先平滑插值到策略契约的 `default_joint_pos`；首次按住 `squeeze` 前，参考窗口
    的 21 个 token 也始终填满这套固定直立站姿，只借用动捕世界中的位置和 yaw，
    不继承人的实时关节角或骨盆 roll/pitch
- 如果调用 `start` 时已经按住 `squeeze`，则保持旧版行为：STAND 直接平滑插值到当前
    实时动捕姿态；进入 RUNNING 的边界再用机器人当拍 torso 建立坐标对齐
- 双手同时按住 `squeeze`：接合首帧先把人体 root 的位置和 yaw 接到队列中最后一个
    固定姿态，避免参考速度跳变；XY 和 yaw 按 torso 对齐，z 则分别用当前关节姿态做
    FK，令参考与机器人双踝最低点处在同一地面。每次重新按住都会重新建立这两层对齐
- 松开 `squeeze`：后续参考帧持续复制松手前最后一个有效动捕姿态，相当于人一直保持
    该姿势；重新按住后才恢复实时流

`squeeze` 使用 0.7/0.5 的接合/释放迟滞，避免模拟量卡在门限附近反复切换。手柄断连也会
立即冻结参考；动捕骨架流断开仍按 `mocap_stale_timeout_s` 急停。

`~/status` 里会多出 `mocap[...]`，`link=up` 且 `body_status=1` 才算通。
`body_status=2` 配 `message=7` 是头显没被正常佩戴/站好，站直走两步通常能回到 VALID。

### 手柄推状态机（`mocap_teleop`）

**双手同时按 B/Y** 走一步：`idle`/`estop` → 站立 → 启动策略 → 急停。规则和
`g1_motion_control` 的 `vr_teleop` 完全一致，是那边踩出来的：

- 站立 / 启动策略 **松手才走**，急停那一步 **按下即走**。都按下即走的话，从站立长按会先把
  策略拉起来、再急停，**中间那一秒机器人已经在跑了**。
- 按满 `estop_hold_s`（默认 1 s）**不看当前状态直接急停**。
- 一上来当作「按着」，必须真松手再按才算一次——避免刚连上就误触。
- 按键流断掉时长按计时作废，恢复后同样要真松手再按。

数据源是 `/mocap/controllers` 而**不是** `/mocap/frame`：后者要校准完成、骨架可用才发，
而最需要急停的时刻恰恰是那些条件不成立的时刻（tracker 全丢、骨架出现非有限值、还没标定）。

> ⚠️ 这意味着**戴头显的人能直接启动机器人**，而他自己看不见周围。旁边必须有人守着物理急停。

### 三件必须知道的事

**一、有 0.34 s 的端到端延迟，砍不掉。** 参考窗口的 `lookahead_steps` 最大 +15，也就是
要 0.3 s 之后的参考。实时动捕没有未来，唯一诚实的做法是让播放头**落后**最新帧
`15 + mocap_lead_margin_frames` 拍。把 `mocap_lead_margin_frames` 调小并不会让延迟消失，
只会让 `+15` 那个 token 被钳成当前帧——**前瞻静默失效**，策略突然没有了未来信息。

**二、`~/start` 之前必须先校准，而且要人站直。** 校准归 `mocap_node`，本层只检查
「标过没有」，没标过直接拒绝 `~/start`。标的是人机比例：按腿长比缩放位移，把站立高度
锚到 G1 自己的高度，再把整个站立位形映射到 `default_joint_pos`。

**RUNNING 中人误按双摇杆会触发重标**，校准会清空动捕缓冲，本层随即因断流急停——
安全，但会打断操作。跟人说清楚别乱按。

**三、STAND 使用固定站姿。** `start` 后机器人平滑插值到策略契约的
`default_joint_pos`，进入 RUNNING 后、首次按住 `squeeze` 前也继续以该站姿作为参考。
人可以在这段时间调整自己的位置和姿势；按住 `squeeze` 的上升沿才会建立坐标对齐并接管。

### 断流会怎样

断流后参考被钳在最后一帧，机器人保持最后的姿势继续站着，**看起来毫无异常**。所以
`mocap_stale_timeout_s`（默认 0.3 s）到点直接急停，不要调大。

## 关节名单

| 名单 | 数量 | 说明 |
|---|---|---|
| FPC 指令 | 31 | 夹爪在**末尾** |
| 策略观测 | 31 | 夹爪夹在**中间**（第 23、31 位） |
| 策略动作 | 29 | 不含夹爪 |

两套 31 轴顺序**不同**，所以全部按名字查槽位，绝不切片。`joints` 由 launch 从
`unitree_g1_ros2_control/config/default_31dof_param.yaml` 注入，不在本包抄第二份。
两个夹爪偏心轴虽然出现在 31 维观测名单里，但策略训练时恒为默认值 0；实机 trigger
可以照常控制夹爪，送入策略的位置与速度仍固定为训练默认值，避免夹爪动作污染全身输出。

这条不是可选优化，而是实机确认过的部署契约。物理夹爪完全打开时 eccentric 约为
`2.764 rad`；若把该反馈原样送进策略，真实 ONNX 离线重放中仅 `0.5 rad` 就会把肩 roll
从约 `+0.30/-0.33 rad` 推到 `+1.29/-1.25 rad`，`2.764 rad` 时多路手臂目标直接逼近限位。
表现就是**不按 squeeze、刚进入 RUNNING 也会抬手并站不稳**，而调整参考 z 几乎没有改善。

因此两条链必须分开：

- FPC 指令：trigger 正常控制实际夹爪开合
- 策略观测：左右 eccentric 位置恒填 `0`，速度恒填 `0`

排查类似问题时不要只检查 29 轴策略动作；还必须审计 31 维本体观测中那两个“策略不控制、
但模型看得见”的轴。
