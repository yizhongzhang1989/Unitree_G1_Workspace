# Unitree_G1_Workspace

Unitree G1 项目的 ROS 2 工作区。

## 末端执行器

末端执行器（力传感器 + 夹爪）的 ROS 2 集成工作区，采用 **"CAN 总线作为共享资源"** 的分层架构：
一个通用 `can_bridge_ros` 独占物理 CAN 总线，各设备是独立 ROS 节点。bridge 可按 CAN ID 把高频帧分流到设备专属 RX 话题，未路由帧走默认 `/canX/rx`。**一设备一节点**，同一条总线可挂多个同构/异构设备；路由由 `robot_bringup` 在启动时根据完整设备清单生成，换接线只换启动配置。

设备：2 个力传感器（KWR57）+ 2 个夹爪（Gloria-M）。支持两种接线：
- **单总线**：所有设备都在 CANalyst-II 的统一 CAN 上（`/can0` 或者 `/can1`）。
- **双总线**：一个力传感器 + 一个夹爪为一组（一个手臂），分别接两条总线（`/can0`、`/can1`）。

```
第1层 can_sdk        : 无 ROS 的 python-can 后端、CANalyst-II 准备和单消费者基础 I/O
第2层 can_bridge_ros : 独占一个 USB-CAN 设备、桥接多通道并按可选 CAN ID 路由 RX
第3层 设备节点        : kwr57_ros / gloria_ros 各自订阅 bringup 分配的专属 RX
第4层 bringup        : robot_bringup 用一份设备清单生成 bridge 路由和设备节点参数
```

消息契约使用上游 ROS 2 [`can_msgs`](https://index.ros.org/p/can_msgs/) 包提供的 `can_msgs/Frame`（与 [ros2_socketcan](https://index.ros.org/p/ros2_socketcan/) 一致）。
它是 ROS 消息定义，不属于 `python-can` 或本项目的 `can_sdk`；Foxy 对应系统包为 `ros-foxy-can-msgs`。日后换 SocketCAN 硬件可直接换官方桥。


## 目录

```
Unitree_G1_Workspace/             一个 colcon workspace
├── README.md
├── .gitignore
├── .gitmodules
├── scripts/                      env.sh（环境）/ run.sh（一键单/双总线，含清理）
├── sdk/                          纯 Python SDK（不参与 colcon 构建）
|   ├── CAN-SDK/                  通用 CAN 基础库（无 ROS、无设备协议）
|   ├── KWR57-SDK/                力传感器 SDK（纯Python，pip 安装；非ROS可用）
|   └── Gloria-M-SDK/             git submodule（云犀夹爪 SDK）
└── src/                          colcon 扫描的 ROS 2 包
    ├── can_bridge_ros/           通用 ROS 2 CAN bridge（多通道）
    ├── kwr57_ros/                力传感器 ROS 设备节点（import kwr57_sensor）
    ├── gloria_ros/               夹爪 ROS 设备节点 + MIT/PV 消息（复用 Gloria SDK 协议）
    └── robot_bringup/            单/双总线 launch + 声明式设备拓扑
```

- SDK 保留：`CAN-SDK`（模块 `can_sdk`）、`KWR57-SDK`（模块 `kwr57_sensor`）和 `gloria_m_sdk` 均可脱离 ROS 使用；ROS 封装只复用基础 I/O 和设备协议，不重复实现。
- 三个 SDK 均不作为 ROS 包，且不由 colcon 构建；`scripts/env.sh` 统一把它们的源码目录加入 `PYTHONPATH`。ROS 节点不需要先安装本地 SDK，也不在节点代码中修改 `sys.path`。
- `can_sdk` 刻意不提供多订阅：直连 SDK 的 `recv()` 是单消费者语义；ROS 多设备系统由 `can_bridge_ros` 成为物理总线的唯一接收者并通过话题分发。
- 夹爪 SDK 整体声明 Python≥3.11；本项目只使用其 `protocol_mit`/`types` 逻辑并已做 Python 3.8 静态语法检查，不打开 SDK 的串口转 CAN 传输层。由于 Python 导入包子模块时仍会执行上游 `gloria_m_sdk/__init__.py`，运行环境当前仍需提供上游依赖 `pyserial`。


## 环境与安装

ROS 2 节点跑在 **foxy 系统 `python3`(3.8)**；运行用 **CycloneDDS**（默认 FastRTPS 会刷 `std::bad_alloc`）。

```bash
# 一次性依赖；ros-foxy-can-msgs 提供 Python 导入 can_msgs.msg.Frame
sudo apt-get install -y ros-foxy-can-msgs
source /opt/ros/foxy/setup.bash
python3 -m pip install --user 'python-can>=4.0' canalystii 'libusb-package>=1.0.30' pyserial

# 拉取含 submodule 的仓库
git clone --recurse-submodules https://github.com/yizhongzhang1989/Unitree_G1_Workspace.git ~/Unitree_G1_Workspace
# 已克隆则： git submodule update --init --recursive

# SDK 位于根目录 sdk/，不在 colcon 默认扫描的 src/ 下；只构建 ROS 包。
cd ~/Unitree_G1_Workspace
colcon build --symlink-install
source scripts/env.sh
```

CANalyst-II 需 udev 权限（VID:PID 04d8:0053），见 `src/can_bridge_ros/README.md`。

`scripts/env.sh` 会将 `CAN-SDK`、`KWR57-SDK` 与 Gloria submodule 的源码目录加入
`PYTHONPATH`，随后加载 ROS 和工作区环境。若要在仓库外独立使用 SDK，可选择安装：

```bash
python3 -m pip install -e './sdk/CAN-SDK[canalystii]'
python3 -m pip install -e ./sdk/KWR57-SDK
```


## 运行

每个手动运行 ROS 命令的终端先 source `scripts/env.sh`；一键脚本会自动处理。

```bash
# 一键（推荐）：脚本 source 好环境、起整套、Ctrl-C 自动清理
bash scripts/run.sh single      # 单总线
bash scripts/run.sh dual        # 双总线

# 或手动（先 source scripts/env.sh 配置环境）
source scripts/env.sh
ros2 launch robot_bringup single_bus.launch.py
ros2 launch robot_bringup dual_bus.launch.py
```

- 单总线下各设备的非共享活动 CAN ID 必须互不冲突；Gloria-M 状态兼容 ID `0x000` 可按协议共享，但各夹爪 `command_id` 的低 4 位设备号必须不同。
- 双总线下两臂在不同总线，**CAN ID 可相同**，无需改。
- 换接线**只改用哪个 launch**，设备节点代码不动。
- 自定义设备 ID、总线或专属 RX 话题时，只修改对应 bringup launch 中的 `CanBus`/`Kwr57Device`/`GloriaDevice` 清单；bridge 路由和设备节点参数会从同一份数据生成。
- 两个 1 kHz KWR57 会产生 6000 个 8-byte 标准 CAN 数据帧/秒。在 1 Mbps 下，无位填充时约占 666 kbit/s，按最坏位填充估算约 810 kbit/s；两个低频 Gloria-M 可以共存，但余量有限，必须保证终端匹配并在实机监控丢帧和实际发布频率。

话题：`/ft_left/wrench_raw`、`/grip_left/joint_states` 等。BEST_EFFORT，`ros2 topic echo` 加
`--qos-reliability best_effort` 或用 `ros2 run kwr57_ros wrench_echo`。

Gloria 节点默认不自动使能。其 `~/enable` 服务会先设置并确认 MIT/PV 控制模式，再使能并等待状态反馈；未使能、模式未确认或反馈过期时默认拒绝运动命令。完整接口与安全参数见 `src/gloria_ros/README.md`。

各包细节见各自 README：`sdk/CAN-SDK/README.md`、`src/can_bridge_ros/README.md`、
`src/kwr57_ros/README.md`、`src/robot_bringup/README.md`。
