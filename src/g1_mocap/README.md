# g1_mocap

PICO 4 Ultra 全身动捕（5 个 Motion Tracker，24 关节 SMPL 骨架）到 G1 29 轴关节角的重定向。
数据经 **WiFi** 从头显上的 [PicoBridge](https://github.com/madderscientist/PicoBridge) APK 过来。

本包**不含任何策略逻辑**，输出就是关节角加各刚体位姿。谁来消费是消费者的事——
`g1_rgmt_tracking_global` 的 `MocapClip` 把它装配成 RGMT 的参考窗口，那部分属于策略契约，
不在这里。

## 数据从哪来

```
PICO 4 Ultra + 5x Motion Tracker ──ws──> 本包的 /ws/device
```

头显上的 APK **直连**本包，中间不过 PicoBridge 的 `server.py`——少一跳转发、少一个要守的
进程。头显的配置面板里填**本机局域网 IP** 加 `:18000`，点连接就行。全程 WiFi，不用 adb。

## 对外的 topic

| Topic | 类型 | 频率 | 内容 |
|---|---|---|---|
| `~/frame` | `g1_mocap_msgs/MocapFrame` | **跟随头显 72/90 Hz** | 一帧的全部产物：关节角、根/锚位姿、key body 位置、人的原始骨架 |
| `~/joint_states` | `sensor_msgs/JointState` | 同上 | 只有关节角。喂 `robot_state_publisher` / rqt / plotjuggler |
| `~/status` | `g1_mocap_msgs/MocapStatus` | 1 Hz | 结构化链路状态，可直接作看门狗判据 |
| `~/calibrate` | `std_srvs/Trigger` 服务 | — | 标人机差异 |

`MocapFrame` 里的字段全部出自**同一帧**骨架。拆成多个 topic 发会引入时间同步问题，
而配错了不报错、只是姿态悄悄不对——所以打成一个原子消息。

> ⚠️ **不要先降采样再插值。** 下游要靠帧间差分求速度（参考窗口就是这么用的），先降到
> 50 Hz 会把原始时间分辨率丢掉，速度噪声直接放大。要 50 Hz 就订原始帧率、自己按
> `header.stamp` 插值。`header.stamp` 已从头显时钟平移到 ROS 时钟，**只平移不改帧间隔**。

## 用法

```bash
ros2 launch g1_mocap mocap.launch.py       # 发 topic
ros2 launch g1_mocap dashboard.launch.py   # 带 mesh 的可视化面板，http://<本机IP>:18080
```

**人站直，然后校准**——三选一：

- 戴着头显**双手摇杆同时按下**（成功会双手各震一下当回执）
- 面板上点「校准」
- `ros2 service call /mocap/calibrate std_srvs/srv/Trigger`

没校准之前 `~/frame` 不会发任何数据。

> 只有 `mocap_node` 连头显。`dashboard_node` 和 `g1_rgmt_tracking_global` 的跟踪层
> 都只订 `~/frame`，三者可以同时跑。

## 重定向怎么做的

**只吃关节位置，不吃厂商给的关节朝向。** PICO 没公开 `XrBodyJointBD` 各关节的局部系定义，
拿它去对 G1 的关节轴等于赌一个没写进文档的约定；而三点位置定一个刚体朝向是确定的几何。

每条肢体按「球窝三轴 + 单铰链」解：

1. 铰链角由近端段与远端段的**夹角**定；
2. 球窝三轴由两个约束定死：近端段方向对上，铰链转轴方向对上。

零位常量**全部从 URDF 现算**，一个都没写死。G1 的零位不是人的立正：

| 量 | 值 | 写死会怎样 |
|---|---|---|
| 肘的零位弯角 | 79.4° | 整条手臂差 80° |
| 髋 roll 轴前倾 | 10.02° | 髋三轴解出来全是错的，且随姿态变 |
| 肩 roll 轴外倾 | 16.0° | 同上 |
| 膝 origin 自带旋转 | 10.02° | 小腿朝向差 10°，踝角跟着错 |

后三项是关节 `origin` 自带的旋转，夹在三个轴中间，所以 `R = Ry(p)Rx(r)Rz(y)` 这个分解
**直接用是错的**，得先把它们并进各轴的零点。这类错误不会报任何异常。

### 两处解不出来的自由度

绕肢体自身轴的自转从单个方向向量里解不出来，**恒为 0**：

- `*_ankle_roll_joint`（踝内外翻，G1 行程只有 ±15°）
- `*_wrist_roll_joint`（腕自转）

要它们就得另接手势通道（PicoBridge 的 `hands` 字段有 26 关节），本包不做。

### 精度

用 G1 自己的 FK 造骨架再解回来（`test_retarget.py` 的闭环），40 个随机位形：

| | p50 | p95 |
|---|---|---|
| 全部 29 轴 | 0.023 rad (1.3°) | 0.22 rad |
| 膝 / 肘 | 0.015 | 0.03 |
| 髋 / 肩 pitch·roll | 0.02 | 0.10 |
| 髋 / 肩 yaw | 0.09 | 0.22 |
| 腕 pitch / yaw | 0.12 / 0.28 | 0.23 / 0.46 |

误差集中在偏航和腕上，两个都是结构性的：G1 的髋/肩三轴不共点（yaw 轴在肢体内部 10 cm），
腕是三轴串联跨 8.9 cm 而人只有一个腕点。

闭环量的是**解算数学本身**。它量不到人机骨架定义的差异（真人的肩髋中心固定在躯干上，
G1 的等效中心会随姿态微动），那一项由 `test_fixed_joint_centers_degrade_gracefully`
单独兜住，p50 约 0.2 rad。要压下去只能上数值 IK，那是另一个量级的复杂度和另一类失效
模式（不收敛）。

## 人机差异怎么对齐

全部靠一段**站立**姿态标出来，分三层：

- **尺度**：按**腿长比**等比缩放全局位移（决定步幅和蹲起幅度），再把站立高度锚到
  G1 自己的站立高度；
- **姿态偏置**（`pelvis_fix` / `torso_fix`）：人的盆骨本就前倾，`PELVIS→SPINE1` 不是
  盆骨真正的 z 轴。不扣就是恒定的高俯仰 +19.8°（训练数据 max 才 +10.3°），而且会污染
  `projected_gravity`，光改关节角修不掉；
- **站立零位映射**（`joint_bias` → `joint_target`）：把校准帧解出来的整个位形搬到 G1 的
  `default_joint_pos` 上，之后按**增量**走。人站立自然外八而 G1 的踝没有偏航自由度，
  那个外旋无处可去会全压到髋偏航上（±35°，p95 才 ±14°）；SMPL 的 `*_FOOT` 是脚趾根部，
  站直时 `ANKLE→FOOT` 本就朝前下方，不扣就是一直勾着脚。这一层把两者一并吸掉。

代价是**绝对姿态会丢**：人站直不再对应“G1 腿伸直”，而是对应 default 姿态。这个交换是
值的——策略只在训练分布内可靠。除此之外**一概不改**，多做一步就是在篡改参考。

## 坐标系

PicoBridge 走 OpenXR 右手系（X 右、Y 上、−Z 前），本包内部是 X 前、Y 左、Z 上。
换算 `(x, y, z) -> (-z, -x, y)`，只在 `skeleton.py` 换一次。

## 上机前必过

```bash
cd src/g1_mocap && python3 -m pytest test/ -q
```

## 已知风险

- **`body.status`**：头显放桌上没戴时是 `2 (LIMITED) / message=7`。LIMITED 本包放行
  （精度降级但数值仍连续），`0 (INVALID)` 才丢帧。站直走两步通常能回到 VALID。
- **参考空间是 `LOCAL_FLOOR`**，长按 Home 重置视角会让所有坐标整体跳变。跳变会被下游
  当成真实运动。要避免就在头显上 `setprop debug.pico.bridge_space stage`。
- **`token` 默认为空**，任何人都能往 `/ws/device` 推数据。放在开放网络上时务必设。
