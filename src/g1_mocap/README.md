# g1_mocap

PICO 4 Ultra 全身动捕（5 个 Motion Tracker，24 关节 SMPL 骨架）到 G1 29 轴关节角的重定向。
数据经 **WiFi** 从头显上的动作捕捉桥接软件 [madderscientist/PicoBridge](https://github.com/madderscientist/PicoBridge) APK 过来。

本包**不含任何策略逻辑**，输出就是关节角加各刚体位姿。谁来消费是消费者的事——
`g1_rgmt_tracking_global` 的 `MocapClip` 把它装配成 RGMT 的参考窗口，那部分属于策略契约，
不在这里。

## 数据从哪来

```
PICO 4 Ultra + 5x Motion Tracker ──ws──> 本包的 /ws/device
```

头显上的 APK **直连**本包，中间不过 PicoBridge 的 `server.py`——少一跳转发、少一个要守的进程。头显的配置面板里填**本机局域网 IP** 加 `:18000`，点连接就行。全程 WiFi，不用 adb。

## 对外的 topic

| Topic | 类型 | 频率 | 内容 |
|---|---|---|---|
| `~/frame` | `g1_mocap_msgs/MocapFrame` | **跟随头显 72/90 Hz** | 一帧的全部产物：关节角、根/锚位姿、key body 位置、人的原始骨架 |
| `~/joint_states` | `sensor_msgs/JointState` | 同上 | 只有关节角。喂 `robot_state_publisher` / rqt / plotjuggler |
| `~/status` | `g1_mocap_msgs/MocapStatus` | 1 Hz | 结构化链路状态，可直接作看门狗判据 |
| `~/calibrate` | `std_srvs/Trigger` 服务 | — | 标人机差异 |

`MocapFrame` 里的字段全部出自**同一帧**骨架。拆成多个 topic 发会引入时间同步问题，而配错了不报错、只是姿态悄悄不对——所以打成一个原子消息。

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

## 重定向约定

- 位置决定肢体方向和屈伸；厂商关节朝向只补位置无法观测的腕/踝自转，并在腿伸直时提供稳定的膝轴。
- G1 的关节轴、零位弯角和 key body 位置全部从 URDF/FK 计算，不直接拿人体关节点冒充机器人刚体。
- 校准按腿长缩放位移、修正盆骨/躯干姿态，并把人的站立位形映射到策略的默认关节角；因此必须站直校准。
- 拿不到关节朝向时会退回纯位置解法，自转归零且膝轴精度下降。
- PicoBridge 使用 OpenXR 右手系（X 右、Y 上、-Z 前）；本包统一转换为 X 前、Y 左、Z 上。

离线闭环测试覆盖随机位形、固定人体关节中心退化、关节限位和 key body FK；详细公式与实测误差见
[`retarget.py`](g1_mocap/retarget.py) 和 [`test_retarget.py`](test/test_retarget.py)。

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
