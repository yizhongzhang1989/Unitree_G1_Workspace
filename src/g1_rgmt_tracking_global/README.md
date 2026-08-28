# g1_rgmt_tracking_global

G1 全身动作跟踪层：读一段参考动作 NPZ，50 Hz 输出 31 轴关节位置目标。
29 轴（12 腿 + 3 腰 + 14 臂）由 RGMT 策略驱动，2 个夹爪偏心轴透传。

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
| key body 局部化 / 倾角保护 | **torso_link** | IMU + 腰三轴 FK |
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
- `/dog_odom` 给的是盆骨，本包用腰偏航做一次 FK 推到 `torso_link`（偏移 4.4 cm，漏掉就是恒定偏置）

## 已知风险：漂移会被策略当真

失效模式是**正反馈**：

```
里程计慢漂 → 策略以为自己漂了 → 主动往回纠 → 真的走偏 → 读数更大
```

训练时这一路只加了 ±2 cm 的逐帧白噪声（上游 BeyondMimic 对同类项用的是 **±25 cm**），所以对慢漂移的容忍度偏低。两道防线：

1. `max_anchor_offset_m`（默认 0.3 m）钳住 anchor 那 3 维，超限退化成开环而不是把机器人推倒。`~/status` 里出现 `CLAMPED` 就是触发了
2. 首次上机用**短片段**，随包的三段都截到 15 s

## 用法

```bash
# 控制栈（会把 FPC 加载成 inactive）
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual

# 本层
ros2 launch g1_rgmt_tracking_global rgmt_tracking.launch.py

# 没有雷达定位栈时
ros2 launch g1_rgmt_tracking_global rgmt_tracking.launch.py odometry_mode:=odom_only

# 操作台（终端是唯一停机入口，退出时自动 estop）
ros2 run g1_rgmt_tracking_global teleop_keyboard
```

状态机与 `g1_motion_control` 一致：

```
IDLE --engage--> (激活 FPC) --start--> STAND --(插值到位)--> RUNNING
                                            任何时候 estop --> ESTOP
```

`~/start` 那一刻会把参考动作**按偏航和平移**对齐到机器人当前位姿（旧包只锁偏航）。高度不对齐——参考的离地高度是动作内容。

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

## 关节名单

| 名单 | 数量 | 说明 |
|---|---|---|
| FPC 指令 | 31 | 夹爪在**末尾** |
| 策略观测 | 31 | 夹爪夹在**中间**（第 22、30 位） |
| 策略动作 | 29 | 不含夹爪 |

两套 31 轴顺序**不同**，所以全部按名字查槽位，绝不切片。`joints` 由 launch 从
`unitree_g1_ros2_control/config/forward_position_controller.yaml` 注入，不在本包抄第二份。
