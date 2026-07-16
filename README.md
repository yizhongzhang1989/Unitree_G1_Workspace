# Unitree_G1_Workspace

Unitree G1 项目的 ROS 2 工作区。

## 末端执行器与相机

末端执行器（力传感器 + 夹爪）的 ROS 2 集成采用“CAN 总线作为共享资源”的分层架构。`can_bridge_ros` 独占物理 CAN；KWR57 作为 bridge 进程中的独立 ROS node 处理高频原始帧，Gloria-M 作为独立进程订阅专属 ROS Frame 话题。左右 IP 相机各由一个 `camera_node` 进程读取 RTSP 并发布图像。末端子系统由 `robot_bringup` 的 `end_effectors_*` 入口统一启动，与全身控制入口分开。

设备：2 个力传感器（KWR57）+ 2 个夹爪（Gloria-M）+ 2 个 IP 相机（左手 `192.168.123.97`、右手 `192.168.123.98`）。CAN 设备支持两种接线：
- **单总线**：所有设备都在 CANalyst-II 的统一 CAN 上（`/can0` 或者 `/can1`）。
- **双总线**：一个力传感器 + 一个夹爪为一组（一个手臂），分别接两条总线（`/can0`、`/can1`）。

```
第1层 can_sdk        : 无 ROS 的 python-can 后端、CANalyst-II 准备和单消费者基础 I/O
第2层 can_bridge_ros : 独占 USB-CAN，桥接多通道，提供路由和通用 handler 注册点
第3层 设备节点        : kwr57_ros 进程内处理；gloria_ros 订阅专属 RX；camera_node 读取 RTSP
第4层 end effectors   : 生成 CAN handler、路由、设备节点及左右相机节点
```

消息契约使用上游 ROS 2 [`can_msgs`](https://index.ros.org/p/can_msgs/) 包提供的 `can_msgs/Frame`（与 [ros2_socketcan](https://index.ros.org/p/ros2_socketcan/) 一致）。它是 ROS 消息定义，不属于 `python-can` 或本项目的 `can_sdk`；Foxy 对应系统包为 `ros-foxy-can-msgs`。


## 宇树 G1

机器人本体使用官方 [`unitree_ros2`](https://github.com/unitreerobotics/unitree_ros2) 消息定义。G1 只需要以下两个包：

- `unitree_api`：机器人服务请求/响应消息。
- `unitree_hg`：G1/H1 系列的状态与控制消息。

本项目不编译 `unitree_go` 和 `unitree_ros2_example`。

`robot_test_dashboard` 为已经运行的 `ros2_control` 全身控制栈提供控制器发现、切换和安全点动界面。`robot_bringup/whole_body_dashboard.launch.py` 只启动该面板；机器人侧仍须先提供 `/robot_description`、`/joint_states`、TF 和 `/controller_manager`。该入口不会运行官方 `g1_dual_arm_example`，也不会主动接管 `/lowcmd`。


## 目录

```
Unitree_G1_Workspace/             一个 colcon workspace
├── README.md
├── .gitignore
├── .gitmodules
├── scripts/                      env.sh（环境）/ run_end_effectors.sh（末端设备一键启动）
├── sdk/                          纯 Python SDK（不参与 colcon 构建）
|   ├── CAN-SDK/                  通用 CAN 基础库（无 ROS、无设备协议）
|   ├── KWR57-SDK/                力传感器 SDK（纯Python，pip 安装；非ROS可用）
|   └── Gloria-M-SDK/             git submodule（云犀夹爪 SDK）
└── src/                          colcon 扫描的 ROS 2 包
    ├── can_bridge_ros/           通用 ROS 2 CAN bridge（多通道）
    ├── camera_node/              左右 IP 相机 RTSP、ROS 图像与 Web 预览
    ├── kwr57_ros/                力传感器 ROS 设备节点（import kwr57_sensor）
    ├── gloria_ros/               夹爪 ROS 设备节点 + MIT/PV 消息（复用 Gloria SDK 协议）
    ├── robot_bringup/            全身控制与末端设备的分层 launch 编排
    ├── robot_test_dashboard/     git submodule（机器人测试 Dashboard）
    └── unitree_ros2/             git submodule（仅构建 unitree_api、unitree_hg）
```

- SDK 保留：`CAN-SDK`（模块 `can_sdk`）、`KWR57-SDK`（模块 `kwr57_sensor`）和 `gloria_m_sdk` 均可脱离 ROS 使用；ROS 封装只复用基础 I/O 和设备协议，不重复实现。
- 三个 SDK 均不作为 ROS 包，且不由 colcon 构建；`scripts/env.sh` 统一把它们的源码目录加入 `PYTHONPATH`。ROS 节点不需要先安装本地 SDK，也不在节点代码中修改 `sys.path`。
- `can_sdk` 刻意不提供多订阅：直连 SDK 的 `recv()` 是单消费者语义；ROS 多设备系统由 `can_bridge_ros` 成为物理总线的唯一接收者并通过话题分发。
- 夹爪 SDK 整体声明 Python≥3.11；本项目只使用其 `protocol_mit`/`types` 逻辑并已做 Python 3.8 静态语法检查，不打开 SDK 的串口转 CAN 传输层。由于 Python 导入包子模块时仍会执行上游 `gloria_m_sdk/__init__.py`，运行环境当前仍需提供上游依赖 `pyserial`。


## 环境与安装
ROS 2 节点跑在 **Foxy 系统 `python3`（3.8）**；运行用 **CycloneDDS**（默认 FastRTPS 会刷 `std::bad_alloc`）。机器人出厂环境已在 `~/cyclonedds_ws` 安装兼容版本，项目继续复用该环境；仓库内的 `src/unitree_ros2/cyclonedds_ws` 只用于构建 Unitree 消息包。

```bash
# ROS/Python 图像栈统一使用 Ubuntu 软件包，避免与 Foxy cv_bridge 产生 ABI 冲突；
# ffprobe 和 ffplay 均由 ffmpeg 软件包提供。
sudo apt-get install -y ros-foxy-can-msgs \
    ros-foxy-cv-bridge ros-foxy-rmw-cyclonedds-cpp \
    ros-foxy-rosidl-generator-dds-idl \
    ffmpeg libyaml-cpp-dev python3-flask python3-opencv python3-numpy
source /opt/ros/foxy/setup.bash
# pip 安装 CAN SDK 运行依赖
python3 -m pip install --user 'python-can>=4.0' canalystii 'libusb-package>=1.0.30' pyserial

# 拉取含 submodule 的仓库
git clone --recurse-submodules https://github.com/yizhongzhang1989/Unitree_G1_Workspace.git ~/Unitree_G1_Workspace
# 已克隆则： git submodule update --init --recursive
```

### CycloneDDS
先确认出厂自带的 CycloneDDS 环境存在：
```bash
test -f ~/cyclonedds_ws/install/setup.bash && echo "CycloneDDS 已安装"
```

若文件存在，无需重复安装。检查 `~/cyclonedds_ws/cyclonedds.xml`，将其中的 `NetworkInterface name` 设置为连接 G1 的有线网卡名称（可用 `ip link` 查看）。

若 `~/cyclonedds_ws` 尚未安装，则按照 `unitree_ros2` 的 Foxy 安装方式，在一个**未 source ROS 2** 的新终端中编译官方要求的 CycloneDDS 0.10.x：
```bash
mkdir -p ~/cyclonedds_ws/src
cd ~/cyclonedds_ws/src
git clone -b foxy https://github.com/ros2/rmw_cyclonedds.git
git clone -b releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds.git
cd ~/cyclonedds_ws
export LD_LIBRARY_PATH="/opt/ros/foxy/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
colcon build --packages-select cyclonedds

# env.sh 从该文件读取 G1 网卡配置；复制后修改 NetworkInterface name
cp ~/Unitree_G1_Workspace/src/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml \
    ~/cyclonedds_ws/cyclonedds.xml
```

完整上游说明见 [`src/unitree_ros2/README.md`](src/unitree_ros2/README.md)。若 CycloneDDS 安装在其他位置，可在 source 环境前设置 `UNITREE_CYCLONEDDS_WS=/实际路径`。

### 构建项目
SDK 位于根目录 `sdk/`，不参与 colcon 构建。必须保留 `--packages-select`，否则 colcon 还会发现不需要的 `unitree_go` 和官方示例。

```bash
cd ~/Unitree_G1_Workspace
source /opt/ros/foxy/setup.bash
source ~/cyclonedds_ws/install/setup.bash
colcon build --symlink-install --packages-select \
    unitree_api unitree_hg \
    camera_node can_bridge_ros gloria_ros kwr57_ros robot_bringup \
    robot_test_dashboard
source scripts/env.sh
```

CANalyst-II 需 udev 权限（VID:PID 04d8:0053），见 `src/can_bridge_ros/README.md`。

`scripts/env.sh` 会依次加载 ROS 2 Foxy、`~/cyclonedds_ws` 和项目安装环境（其中包含 Unitree G1 消息），同时将 `CAN-SDK`、`KWR57-SDK` 与 Gloria submodule 的源码目录加入 `PYTHONPATH`。若要在仓库外独立使用 SDK，可选择安装：
```bash
python3 -m pip install -e './sdk/CAN-SDK[canalystii]'
python3 -m pip install -e ./sdk/KWR57-SDK
```


## 运行

### 末端设备
每个手动运行 ROS 命令的终端先 source `scripts/env.sh`；一键脚本会自动处理。
```bash
# 一键（推荐）：脚本 source 好环境、起整套、Ctrl-C 自动清理
bash scripts/run_end_effectors.sh single      # 单总线
bash scripts/run_end_effectors.sh dual        # 双总线

# 或手动（先 source scripts/env.sh 配置环境）
source scripts/env.sh
ros2 launch robot_bringup end_effectors_single_bus.launch.py
ros2 launch robot_bringup end_effectors_dual_bus.launch.py
```

以上入口都会启动左右两个相机。Web 地址分别为 `http://<机器人 IP>:8010` 和 `http://<机器人 IP>:8011`，ROS 图像话题为 `/camera_left/image_raw` 和 `/camera_right/image_raw`。

- 单总线下各设备的非共享活动 CAN ID 必须互不冲突；Gloria-M 状态兼容 ID `0x000` 可按协议共享，但各夹爪 `command_id` 的低 4 位设备号必须不同。
- 双总线下两臂在不同总线，**CAN ID 可相同**，无需改。
- 换接线**只改用哪个 launch**，设备节点代码不动。
- 自定义设备 ID、总线、Wrench 输出或夹爪专属 RX 话题时，只修改对应 bringup launch 中的 `CanBus`、`Kwr57Device`、`GloriaDevice` 清单；handler、路由和节点参数会从同一份数据生成。
- 每个 1 kHz KWR57 会在所属物理 CAN 上产生 3000 个 8-byte 标准帧/s；加一台 100 Hz Gloria-M 往返后，每条 1 Mbps 总线预计占用约 35.742% 至 43.470%。CAN0/CAN1 是独立物理总线，优化后的 bridge 已在完整四设备负载下验证双路 1 kHz。

话题：`/ft_left/wrench_raw`、`/grip_left/joint_states` 等。BEST_EFFORT 话题使用 `ros2 topic echo --qos-reliability best_effort`，KWR57 也可使用 `ros2 run kwr57_ros wrench_echo`。Dashboard 使用 raw `WrenchStamped` 和 `KEEP_LAST(64)` 展示 3 秒平均接收频率；1 kHz 控制订阅建议采用 `rclcpp`、BEST_EFFORT 和 `KEEP_LAST(64)`。

Gloria 节点默认不自动使能。其 `~/enable` 服务会先设置并确认 MIT/PV 控制模式，再使能并等待状态反馈；未使能、模式未确认或反馈过期时默认拒绝运动命令。完整接口与安全参数见 `src/gloria_ros/README.md`。

### 全身控制测试面板
先启动机器人的 `ros2_control` 全身控制栈，再运行：
```bash
source scripts/env.sh
ros2 launch robot_bringup whole_body_dashboard.launch.py
```

浏览器打开 `http://<机器人 IP>:8200`。该入口只包装 `robot_test_dashboard`，不会启动硬件接口、控制器管理器或底层 G1 控制；缺少 `/controller_manager` 等前置接口时，页面会保持等待状态。参数和安全约束见 `src/robot_test_dashboard/README.md`。

各包细节见 `sdk/CAN-SDK/README.md`、`src/can_bridge_ros/README.md`、`src/camera_node/README.zh.md`、`src/kwr57_ros/README.md` 和 `src/robot_bringup/README.md`。
