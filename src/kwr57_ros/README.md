# `kwr57_ros`（ROS 2 封装 · bridge 架构）

坤维 **KWR57** 六轴力/力矩传感器（CAN）的 ROS 2 驱动。采用**"总线作为共享资源"**的
分层架构：一个通用 [`can_bridge_ros`](../can_bridge_ros) 节点独占物理 CAN 总线，KWR57 只是
一个**纯 ROS 设备节点**——订阅 bridge 按 CAN ID 分配的专属 RX 话题、组包并发布
`WrenchStamped`；未配置路由时也可订阅默认总线 RX 并自行过滤。
这样**同一条总线可挂多个同构/异构设备**，每个设备一个节点，不必各自开总线。

```
第1层 can_sdk        : python-can 后端与基础 I/O（无 ROS、无设备协议）
第2层 can_bridge_ros : 独占总线；按 CAN ID 路由高频 RX，订阅 /can0/tx 下发
第3层 设备节点   : 本包 kwr57_ros，订阅设备专属 RX，发 WrenchStamped
```

高频数据流：`CAN 帧 → can_bridge_ros(/can0/ft_left/rx) → kwr57 节点(组包) → WrenchStamped`；
命令：`kwr57 设备节点(build_*) → /can0/tx → can_bridge_ros → 传感器`。

消息契约用标准 `can_msgs/msg/Frame`（与 [`ros2_socketcan`] 一致）；日后换 SocketCAN 硬件可
直接用官方 `ros2_socketcan` 替换 bridge，**设备节点无需改动**。

这里的 `can_msgs` 是上游 ROS 2 消息包，由
[`ros-industrial/ros_canopen`](https://github.com/ros-industrial/ros_canopen/tree/dashing-devel/can_msgs)
提供，不属于 `python-can`、`can_sdk` 或 KWR57-SDK。Foxy 使用
`sudo apt-get install ros-foxy-can-msgs` 安装；本包也在 `package.xml` 中声明了
`<exec_depend>can_msgs</exec_depend>`。

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

# 2.2 只安装第三方运行依赖；本地 SDK 不需要安装
source /opt/ros/foxy/setup.bash
python3 -m pip install --user 'python-can>=4.0' canalystii 'libusb-package>=1.0.30'

# 2.3 SDK 位于根目录 sdk/，不参与 colcon 对 src/ 的扫描
cd ~/end_effector_ros && colcon build --symlink-install
source scripts/env.sh
```

`scripts/env.sh` 通过 `PYTHONPATH` 暴露 `CAN-SDK`、`KWR57-SDK` 和 Gloria submodule
的源码。它们不是 ROS 包，也不会被 colcon 安装；若要在仓库外使用，再选择 `pip install -e`。

CANalyst-II 需 udev 权限（一次性，见 `src/can_bridge_ros/README.md`）。

---

## 3. 运行

### 3.1 一条龙 demo（推荐）

`scripts/run.sh` 会 source 好环境、起 **can_bridge_ros + 设备节点**，Ctrl-C 退出时自动清理：

```bash
bash ~/end_effector_ros/scripts/run.sh          # 单总线
bash ~/end_effector_ros/scripts/run.sh dual     # 双总线（每臂一条总线）
```

只要看力传感器数据：`ros2 run kwr57_ros wrench_echo`。

### 3.2 手动分步（两个终端）

```bash
# 终端 A：单独起通用 bridge；物理 YAML 不包含设备路由
source ~/end_effector_ros/scripts/env.sh
ros2 launch can_bridge_ros can_bridge_ros.launch.py config:=single_bus.yaml

# 终端 B：调试模式直接订阅默认总线 RX
source ~/end_effector_ros/scripts/env.sh
ros2 launch kwr57_ros ft_sensor.launch.py rx_topic:=/can0/rx tx_topic:=/can0/tx
```

> 这种手动方式用于单设备调试，没有高频 ID 分流。多设备 1 kHz 部署应直接启动
> `robot_bringup`，由它在启动时生成专属路由。

---

## 4. 话题 / 服务 / 参数（设备节点）

| 方向 | 名称 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `<rx_topic>` (默认 `/can0/rx`) | `can_msgs/Frame` | bridge 默认 RX 或 bringup 专属路由帧 |
| 发布 | `<tx_topic>` (默认 `/can0/tx`) | `can_msgs/Frame` | 下发给 bridge 的命令帧 |
| 发布 | `<topic>` (默认 `~/wrench_raw`) | `geometry_msgs/WrenchStamped` | 六轴力/力矩，BEST_EFFORT/KEEP_LAST(32) |
| 订阅 | `~/command` | `std_msgs/String` | `start`/`stop`/`tare`(别名`zero`)/`reset_tare` |

服务（`std_srvs/Trigger`）：`~/start` `~/stop` `~/tare`（软件调零：下一帧作零点）`~/reset_tare`。

> **QoS**：RX 是 BEST_EFFORT/KEEP_LAST(128)，`wrench_raw` 是
> BEST_EFFORT/KEEP_LAST(32)。`ros2 topic echo` 需加 `--qos-reliability best_effort`，
> 或用 `ros2 run kwr57_ros wrench_echo`。

参数（及默认）：

| 名称 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `rx_topic` | string | `/can0/rx` | bridge RX；完整 bringup 会自动改为设备专属话题 |
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

多设备共享一个 bridge，每个设备仍是独立节点。生产启动由
[`robot_bringup/launch`](../robot_bringup/launch) 中的一份 `Kwr57Device` 清单同时生成：

- bridge 的三条数据 CAN ID 路由；
- 设备节点的 `rx_topic`、`cmd_id`、`data_base_id`、输出话题和 `frame_id`。

因此 CAN ID 和专属话题只声明一次，启动时还会检查同一通道的命令 ID、数据 ID、节点名
和专属话题是否冲突。

```bash
# 0) 先用 examples/set_id.py 给每个传感器设不同 CAN ID（各接一个、逐个改）：
#      left : 接收 0x10 / 发送基址 0x15   right: 接收 0x11 / 发送基址 0x18
python ~/end_effector_ros/sdk/KWR57-SDK/examples/set_id.py --interface canalystii --channel 0 \
    --host-id 0x11 --sensor-id 0x18 --verify        # 配置 right（此时只接 right）

# 1) 确认 single_bus.launch.py 中两台 Kwr57Device 的 ID 与硬件一致
# 2) 一次启动 bridge、两台 KWR57 和两台 Gloria-M
ros2 launch robot_bringup single_bus.launch.py
```

> ⚠️ 两个未改 ID 的设备会发出**相同的 CAN ID**，无法区分且冲突——必须先设不同 ID。
> 设 ID 见 `examples/set_id.py`（非 ROS，直接开总线；配置时请**只接一个设备**）。

### 5.1 1 kHz 数据路径

- `can_bridge_ros` 对每个物理接收帧只发布一次：KWR57 数据 ID 进入设备专属话题，
  其他 ID 进入默认 `/canX/rx`，避免两个 KWR57 和两个 Gloria-M 节点重复处理全部高频帧。
- 每个 KWR57 节点使用 `SingleThreadedExecutor`。这里只有一个高频订阅，线程池会增加
  CPython GIL 竞争和任务调度开销。
- 一个 KWR57 每个样本占三帧；两台设备在 1 kHz 时合计 6000 frame/s。1 Mbps 标准 CAN
  无位填充时约占 666 kbit/s，按最坏位填充估算约 810 kbit/s，仍可容纳低频夹爪通信，
  但布线、终端电阻、USB 稳定性和夹爪发送频率都会影响实际余量。
- 修改传感器 ID 后，只同步修改对应 bringup launch 中该设备的 `cmd_id` 和
  `data_base_id`；专属路由会由同一个 `Kwr57Device` 自动重新生成。

目标机上逐个检查两个输出（工具每秒打印一次，避免控制台 I/O 干扰高频回调）：

```bash
ros2 run kwr57_ros wrench_echo --ros-args -p topic:=/ft_left/wrench_raw
ros2 run kwr57_ros wrench_echo --ros-args -p topic:=/ft_right/wrench_raw
```

测频订阅者本身也占用 CPU；最终控制节点应保持 BEST_EFFORT，并避免同时运行多个
`topic echo/hz` 或可视化订阅者干扰高频测试。

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
| 设备节点 `stream start not confirmed` | bridge 没起、bringup 清单与传感器实际 ID 不一致，或命令未送达 |
| 专属 RX 没有帧 | 检查 `Kwr57Device` 的总线/ID、传感器实际 ID、供电、接线和起流命令 |
| `/can0/rx` 看不到 KWR57 帧 | 配置路由后的正常行为；KWR57 帧只进入设备专属 RX |
| `ros2 topic echo` 一直没输出 | 话题是 BEST_EFFORT，加 `--qos-reliability best_effort` 或用 `wrench_echo` |
| 发布频率低于 1 kHz | 确认使用 CycloneDDS、专属 RX 和 `period_ms=1`；停止额外 echo/hz 后检查总线错误与 CPU |
| 满屏 `std::bad_alloc` | 用了默认 FastRTPS；改 CycloneDDS（第 1 节）|
| `[Errno 16] Resource busy`（bridge 打不开）| 上个 bridge 没关干净：`pkill -INT -f bridge_node`（不行再 `-KILL`）|
| `No module named 'can'` | 将 python-can/CANalyst-II 依赖安装到 ROS 使用的系统 Python |
| `No module named 'can_sdk'` / `kwr57_sensor` | 当前终端未加载 SDK 源码路径；执行 `source ~/end_effector_ros/scripts/env.sh` |
| 多设备只出一个 | 两设备 CAN ID 相同，先用 set_id.py 改成不同 ID |

---

## 8. 目录结构

```
~/end_effector_ros/                  ← 一个 colcon 工作区（见顶层 README）
├── sdk/
    ├── CAN-SDK/                     无 ROS 的 CAN 后端与基础 I/O
    ├── KWR57-SDK/                   力传感器 SDK（协议层被设备节点复用）
  │   └── examples/set_id.py       设置/复位设备 CAN ID（多设备前置）
  └── Gloria-M-SDK/                夹爪 SDK submodule
└── src/
  ├── can_bridge_ros/              通用 ROS 2 CAN bridge（第2层，多通道）
  └── kwr57_ros/                   本包（力传感器 ROS 设备节点）
        └── kwr57_ros/
            ├── ft_sensor_node.py    订阅 /rx 过滤、发 /tx、发 WrenchStamped
            ├── wrench_echo.py       demo 订阅（控制台）
            └── web_wrench_node.py   demo 订阅（浏览器可视化）
```

[`ros2_socketcan`]: https://github.com/autowarefoundation/ros2_socketcan
