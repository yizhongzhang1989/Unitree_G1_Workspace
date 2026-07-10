# `kwr57_ros`（ROS 2 封装 · bridge 架构）

坤维 **KWR57** 六轴力/力矩传感器（CAN）的 ROS 2 驱动。采用**"总线作为共享资源"**的
分层架构：一个通用 [`can_bridge`](../../../can_bridge) 节点独占物理 CAN 总线，KWR57 只是
一个**纯 ROS 设备节点**——订阅总线帧、按自己的 CAN ID 过滤、发布 `WrenchStamped`。
这样**同一条总线可挂多个同构/异构设备**，每个设备一个节点，不必各自开总线。

```
第1层 CAN Driver : python-can 后端（canalystii/socketcan/...）
第2层 can_bridge : 独占总线；发布所有帧到 /can0/rx，订阅 /can0/tx 下发   ← src/can_bridge
第3层 设备节点   : 本包 kwr57_ros，订阅 /can0/rx 过滤自己的 ID，发 WrenchStamped
```

数据流：`CAN 帧 → can_bridge(/can0/rx) → kwr57 设备节点(组包) → WrenchStamped`；
命令：`kwr57 设备节点(build_*) → /can0/tx → can_bridge → 传感器`。

消息契约用标准 `can_msgs/msg/Frame`（与 [`ros2_socketcan`] 一致）；日后换 SocketCAN 硬件可
直接用官方 `ros2_socketcan` 替换 bridge，**设备节点无需改动**。

---

## 1. 环境（重要）

- ROS 2 节点跑在 **foxy 系统 `python3`(3.8)**（conda `robot` 的 3.13 无法加载 foxy 的 rclpy；
  即使激活 conda，`ros2 run` 也由 shebang 用系统 3.8 执行）。
- 运行用 **CycloneDDS**（默认 FastRTPS 在本机会刷 `std::bad_alloc`）：

```bash
source /opt/ros/foxy/setup.bash
source ~/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

---

## 2. 安装

```bash
# 2.1 标准消息包
sudo apt-get install -y ros-foxy-can-msgs

# 2.2 pip 依赖 + 力传感器 SDK（纯Python，非 ROS 包，供节点 import）
source /opt/ros/foxy/setup.bash
python3 -m pip install --user 'python-can>=4.0' canalystii 'libusb-package>=1.0.24'
python3 -m pip install --user -e ~/end_effector_ros/src/KWR57-SDK

# 2.3 构建整个工作区（含 can_bridge / kwr57_ros / gloria_ros 等）
cd ~/end_effector_ros && colcon build --symlink-install
source install/setup.bash
```

CANalyst-II 需 udev 权限（一次性，见 `src/can_bridge/README.md`）。

---

## 3. 运行

### 3.1 一条龙 demo（推荐）

`scripts/run.sh` 会 source 好环境、起 **can_bridge + 设备节点**，Ctrl-C 退出时自动清理：

```bash
bash ~/end_effector_ros/scripts/run.sh          # 单总线
bash ~/end_effector_ros/scripts/run.sh dual     # 双总线（每臂一条总线）
```

只要看力传感器数据：`ros2 run kwr57_ros wrench_echo`。

### 3.2 手动分步（两个终端）

```bash
# 终端 A：先起通用 bridge（独占 CANalyst-II CAN1 -> /can0/rx、/can0/tx）
ros2 launch can_bridge can_bridge.launch.py config:=single_bus.yaml

# 终端 B：起 KWR57 设备节点（订阅 /can0/rx，命令发 /can0/tx）
ros2 launch kwr57_ros ft_sensor.launch.py rx_topic:=/can0/rx tx_topic:=/can0/tx
```

> 先起 bridge 再起设备节点；bridge 没起时设备节点会提示 "stream start not confirmed"。

---

## 4. 话题 / 服务 / 参数（设备节点）

| 方向 | 名称 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `<rx_topic>` (默认 `/can0/rx`) | `can_msgs/Frame` | 来自 bridge 的所有总线帧 |
| 发布 | `<tx_topic>` (默认 `/can0/tx`) | `can_msgs/Frame` | 下发给 bridge 的命令帧 |
| 发布 | `<topic>` (默认 `~/wrench_raw`) | `geometry_msgs/WrenchStamped` | 六轴力/力矩，BEST_EFFORT/KEEP_LAST(200) |
| 订阅 | `~/command` | `std_msgs/String` | `start`/`stop`/`tare`(别名`zero`)/`reset_tare` |

服务（`std_srvs/Trigger`）：`~/start` `~/stop` `~/tare`（软件调零：下一帧作零点）`~/reset_tare`。

> **QoS**：`wrench_raw` 是 BEST_EFFORT，`ros2 topic echo` 需加 `--qos-reliability best_effort`，
> 或用 `ros2 run kwr57_ros wrench_echo`。

参数（及默认）：

| 名称 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `rx_topic` | string | `/can0/rx` | bridge 发布的总线帧话题 |
| `tx_topic` | string | `/can0/tx` | bridge 订阅的命令帧话题 |
| `cmd_id` | int | `16`(0x10) | 本设备命令(接收)CAN ID |
| `data_base_id` | int | `21`(0x15) | 本设备数据起始 CAN ID（帧 base/+1/+2）|
| `topic` | string | `~/wrench_raw` | 输出 wrench 话题 |
| `frame_id` | string | `kwr57_ft_sensor_link` | 输出 `header.frame_id` |
| `period_ms` | int | `1` | 上传周期(ms)，1 ≈ 1000 Hz |
| `sample_rate_hz` | int | `1000` | 内部采样率（100/200/400/500/600/1000）|
| `publish_rate` | double | `0.0` | 0 = 每帧都发 |
| `use_si` | bool | `false` | false=原始值(与非ROS一致)；true=换算 N/N·m |
| `autostart` | bool | `true` | 启动即起流 |
| `tare_on_start` | bool | `false` | 启动后用首帧做软件调零 |

---

## 5. 多设备（同一条总线挂多个 KWR57）

新架构下"多设备"很自然：**共享一个 bridge，每个设备起一个设备节点**，各自过滤自己的 CAN ID。

```bash
# 0) 先用 examples/set_id.py 给每个传感器设不同 CAN ID（各接一个、逐个改）：
#      left : 接收 0x10 / 发送基址 0x15   right: 接收 0x11 / 发送基址 0x18
python ~/end_effector_ros/src/KWR57-SDK/examples/set_id.py --interface canalystii --channel 0 \
    --host-id 0x11 --sensor-id 0x18 --verify        # 配置 right（此时只接 right）

# 1) 一个 bridge
ros2 launch can_bridge can_bridge.launch.py config:=single_bus.yaml

# 2) 每个设备一个节点（不同 cmd_id/data_base_id/node_name/topic）
ros2 launch kwr57_ros ft_sensor.launch.py node_name:=kwr57_left \
    cmd_id:=16 data_base_id:=21 topic:=/left/wrench_raw frame_id:=left_ft_link
ros2 launch kwr57_ros ft_sensor.launch.py node_name:=kwr57_right \
    cmd_id:=17 data_base_id:=24 topic:=/right/wrench_raw frame_id:=right_ft_link
```

> ⚠️ 两个未改 ID 的设备会发出**相同的 CAN ID**，无法区分且冲突——必须先设不同 ID。
> 设 ID 见 `examples/set_id.py`（非 ROS，直接开总线；配置时请**只接一个设备**）。

---

## 6. Demo / 可视化

- **`wrench_echo`**：BEST_EFFORT 订阅并打印六轴数值与频率（`ros2 run kwr57_ros wrench_echo`）。
- **`web_wrench`**：浏览器可视化（六轴条形 + 力/力矩矢量），订阅 wrench 话题：
  `ros2 run kwr57_ros web_wrench`，浏览器开 `http://127.0.0.1:8765`（SSH 用 `ssh -L 8765:127.0.0.1:8765 user@server`）。
- **`read_kwr57`**：非 ROS 控制台读取（**直接开总线**，不经 bridge，用于台架调试）：
  `ros2 run kwr57_ros read_kwr57 --interface canalystii --channel 0`。

---

## 7. 常见问题

| 现象 | 处理 |
|---|---|
| 设备节点 `stream start not confirmed` | bridge 没起或 rx/tx 话题名不对；先起 `can_bridge`，确认 `/can0/rx` 有帧 |
| `/can0/rx` 没有帧 | bridge 未连上适配器 / 传感器没上电；或未下发起流命令 |
| `ros2 topic echo` 一直没输出 | 话题是 BEST_EFFORT，加 `--qos-reliability best_effort` 或用 `wrench_echo` |
| 满屏 `std::bad_alloc` | 用了默认 FastRTPS；改 CycloneDDS（第 1 节）|
| `[Errno 16] Resource busy`（bridge 打不开）| 上个 bridge 没关干净：`pkill -INT -f bridge_node`（不行再 `-KILL`）|
| `No module named 'can'` / `kwr57_sensor` | can 依赖未装进 foxy python：`pip install --user python-can canalystii libusb-package`；kwr57_sensor 未构建：`cd ~/end_effector_ros && colcon build` |
| 多设备只出一个 | 两设备 CAN ID 相同，先用 set_id.py 改成不同 ID |

---

## 8. 目录结构

```
~/end_effector_ros/                  ← 一个 colcon 工作区（见顶层 README）
└── src/
    ├── can_bridge/                  通用 CAN bridge（第2层，多通道）
    ├── KWR57-SDK/                   力传感器 SDK（纯Python，非ROS包；协议层被设备节点复用）
    │   └── examples/set_id.py       设置/复位设备 CAN ID（多设备前置）
    └── kwr57_ros/                   本包（力传感器 ROS 设备节点）
        └── kwr57_ros/
            ├── ft_sensor_node.py    订阅 /rx 过滤、发 /tx、发 WrenchStamped
            ├── wrench_echo.py       demo 订阅（控制台）
            └── web_wrench_node.py   demo 订阅（浏览器可视化）
```

[`ros2_socketcan`]: https://github.com/autowarefoundation/ros2_socketcan
