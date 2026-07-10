# end_effector_ros

末端执行器（力传感器 + 夹爪）的 ROS 2 集成工作区，采用**"CAN 总线作为共享资源"**的分层架构：
一个通用 `can_bridge` 独占物理 CAN 总线，各设备只是**订阅总线帧、按 CAN ID 过滤**的独立
设备节点。**一设备一节点**，同一条总线可挂多个同构/异构设备；换接线只换启动配置，驱动不改。

设备：2 个力传感器（KWR57）+ 2 个夹爪（Gloria-M）。支持两种接线：
- **单总线**：所有设备都在 CANalyst-II 的 CAN1（`/can0`）。
- **双总线**：一个力传感器 + 一个夹爪为一组（一个手臂），分别接两条总线（`/can0`、`/can1`）。

```
第1层 CAN Driver : python-can 后端（canalystii/socketcan/...）
第2层 can_bridge : 独占一个 USB-CAN 设备、可同时桥接多通道；发布 /canX/rx，订阅 /canX/tx
第3层 设备节点   : kwr57_ros / gloria_ros，各订阅 /canX/rx 过滤自己的 ID
第4层 bringup    : robot_bringup 用 launch/config 描述单/双总线接线
```

消息契约用标准 `can_msgs/Frame`（与 [ros2_socketcan] 一致）；日后换 SocketCAN 硬件可直接换官方桥。

---

## 目录

```
end_effector_ros/                 ← 一个 colcon workspace + 你自己的 git 仓库
├── .gitmodules
├── scripts/                      env.sh（环境）/ run.sh（一键单/双总线，含清理）
├── src/
│   ├── can_bridge/               通用 CAN bridge（多通道）
│   ├── KWR57-SDK/                力传感器 SDK（纯Python，非 ROS 包，pip 安装；非ROS也可用）
│   ├── kwr57_ros/                力传感器 ROS 设备节点（import kwr57_sensor）
│   ├── gloria_ros/               夹爪 ROS 设备节点（复用 gloria_m_sdk 的 MIT 协议）
│   ├── Gloria-M-SDK/   ← git submodule（云犀夹爪 SDK，非本仓库）
│   └── robot_bringup/            单/双总线 launch + config
```

- 你自己的代码放在本仓库里（普通目录）；**别人的仓库**（Gloria-M-SDK）用 **git submodule** 链接。
- **SDK 保留**：`KWR57-SDK`(模块 `kwr57_sensor`) 和 `gloria_m_sdk` 都作为纯 Python SDK 保留供非 ROS 使用；
  ROS 封装**复用 SDK 的方法**（协议打包/解包），不重复实现。
- 夹爪 SDK 整体要求 Python≥3.11，但其 `protocol_mit`/`types` 兼容 3.8，故 `gloria_ros`
  只 import 这部分（运行时按 submodule 路径加载），绕开版本限制、无需 SDK 的串口传输层。

---

## 环境与安装

ROS 2 节点跑在 **foxy 系统 `python3`(3.8)**；运行用 **CycloneDDS**（默认 FastRTPS 会刷 `std::bad_alloc`）。

```bash
# 一次性依赖
sudo apt-get install -y ros-foxy-can-msgs
source /opt/ros/foxy/setup.bash
python3 -m pip install --user 'python-can>=4.0' canalystii 'libusb-package>=1.0.24' pyserial

# 拉取含 submodule 的仓库
git clone --recurse-submodules <本仓库URL> ~/end_effector_ros
# 已克隆则： git submodule update --init --recursive

# 力传感器 SDK（纯Python，非 ROS 包）装进 foxy python，供 ROS 节点 import
python3 -m pip install --user -e ~/end_effector_ros/src/KWR57-SDK

# 构建（KWR57-SDK / Gloria-M-SDK 带 COLCON_IGNORE，不被 colcon 编译）
cd ~/end_effector_ros
colcon build --symlink-install
source install/setup.bash
```

CANalyst-II 需 udev 权限（VID:PID 04d8:0053），见 `src/can_bridge/README.md`。

---

## 运行

先 source CycloneDDS 环境（见上）与 `install/setup.bash`。

```bash
# 一键（推荐）：脚本 source 好环境、起整套、Ctrl-C 自动清理
bash scripts/run.sh single      # 单总线
bash scripts/run.sh dual        # 双总线

# 或手动（先 source scripts/env.sh 配置环境）
source scripts/env.sh
ros2 launch robot_bringup single_bus.launch.py
ros2 launch robot_bringup dual_bus.launch.py
```

- 单总线下同一条总线各设备 **CAN ID 必须不同**（力传感器用 `src/KWR57-SDK/examples/set_id.py` 改）。
- 双总线下两臂在不同总线，**CAN ID 可相同**，无需改。
- 换接线**只改用哪个 launch**，设备节点代码不动。

话题：`/ft_left/wrench_raw`、`/grip_left/joint_states` 等。BEST_EFFORT，`ros2 topic echo` 加
`--qos-reliability best_effort` 或用 `ros2 run kwr57_ros wrench_echo`。

各包细节见各自 README：`src/can_bridge/README.md`、`src/kwr57_ros/README.md`。

[ros2_socketcan]: https://github.com/autowarefoundation/ros2_socketcan
