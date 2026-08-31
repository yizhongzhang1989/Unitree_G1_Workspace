# g1_mocap

PICO 4 Ultra 全身动捕（5 个 Motion Tracker，24 关节 SMPL 骨架）到 G1 29 轴关节角的重定向。
数据经 **WiFi** 从头显上的 [PicoBridge](https://github.com/madderscientist/PicoBridge) APK 过来。

本包**不含任何策略逻辑**，输出就是关节角加各刚体位姿。谁来消费是消费者的事——
`g1_rgmt_tracking_global` 的 `MocapClip` 把它装配成 RGMT 的参考窗口，那部分属于策略契约，
不在这里。

## 数据从哪来

```
PICO 4 Ultra + 5x Motion Tracker
    ──ws──> mocap_node 内建的 /ws/device            source=device（默认）
    ──ws──> PicoBridge 的 server.py ──ws──> 本包    source=bridge
```

`device` 少一跳转发，也不用另起 `server.py`；头显的配置面板里填**本机局域网 IP** 加 `:8000`，
点连接就行。想同时用 PicoBridge 的 `/monitor` 面板看骨架时才用 `bridge`。

## 用法

```bash
ros2 launch g1_mocap mocap.launch.py
ros2 topic echo /mocap/status
ros2 topic echo /mocap/joint_states --field position
```

`status` 长这样，`body_status=1` 才是正常：

```
frames=8421 dropped=3 link=up body_status=1 (正常) calibrated=True
```

**启动时人要站直**：节点攒够帧会自动标一次人机比例。站得不对就重标：

```bash
ros2 service call /mocap/calibrate std_srvs/srv/Trigger
```

> `mocap_node` 和 `g1_rgmt_tracking_global` 的跟踪层**不要同时起**——头显同一时刻只连一个
> 上行地址。跟踪层是在自己进程里建连接的，不走话题（走话题会把 72/90 Hz 的原始时间分辨率
> 先降到 50 Hz，参考窗口的速度差分就毁了）。

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

## 人机比例

只做两件事，**其余一概不改**——多做的每一步都是在篡改参考：

- 按**腿长比**等比缩放全局位移（决定步幅和蹲起幅度）；
- 按校准帧把站立高度锚到 G1 自己的站立高度。

外加一个踝俯仰零点：SMPL 的 `*_FOOT` 是脚趾根部而不是脚掌前缘，站直时 `ANKLE→FOOT`
本来就朝前下方，不扣掉就是机器人一直勾着脚。

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
