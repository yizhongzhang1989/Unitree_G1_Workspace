# Unitree_G1_Workspace

Unitree G1 项目的 ROS 2 工作区。

## 末端执行器与相机

末端执行器（力传感器 + 夹爪）的 ROS 2 集成采用“CAN 总线作为共享资源”的分层架构。`can_bridge_ros` 独占物理 CAN；KWR57 作为 bridge 进程中的独立 ROS node 处理高频原始帧，Gloria-M 作为独立进程订阅专属 ROS Frame 话题。左右 IP 相机各由一个 `camera_node` 进程读取 RTSP 并发布图像。末端子系统由 `robot_bringup` 的 `end_effectors_*` 入口统一启动；整机 URDF 和控制器测试网页是独立功能，不包含 G1 本体控制。

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

`unitree_g1_description` 提供整机 URDF、`/lowstate` 到 `/joint_states` 的状态适配，以及面向 `robot_test_dashboard` 的受限 MIT 位置控制节点。该节点提供一个 forward-position 控制器外观，将网页给出的 31 个实际关节位置分发为 G1 `/lowcmd` 和左右 Gloria-M `MitCommand`。它不是通用 `ros2_control` 硬件接口或运动控制器。


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
    ├── unitree_g1_description/   整机 description 包（model/ 为 URDF submodule）
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
    ros-foxy-controller-manager-msgs \
    ros-foxy-rosidl-generator-dds-idl \
    ros-foxy-robot-state-publisher ros-foxy-xacro \
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
    robot_test_dashboard unitree_g1_description
source scripts/env.sh
```

CANalyst-II 需 udev 权限（VID:PID 04d8:0053），见 `src/can_bridge_ros/README.md`。

`scripts/env.sh` 会依次加载 ROS 2 Foxy、`~/cyclonedds_ws` 和项目安装环境（其中包含 Unitree G1 消息），同时将 `CAN-SDK`、`KWR57-SDK` 与 Gloria submodule 的源码目录加入 `PYTHONPATH`。若要在仓库外独立使用 SDK，可选择安装：
```bash
python3 -m pip install -e './sdk/CAN-SDK[canalystii]'
python3 -m pip install -e ./sdk/KWR57-SDK
```


## 启动入口
每个终端先执行：
```bash
source scripts/env.sh
```

生产运行分为两个阶段：先启动数据节点，再按需手动启动 Dashboard。唯一推荐的数据入口是 `robot_bringup/all_data.launch.py`。

### 阶段一：启动数据
只启动末端数据（CAN、KWR57、Gloria-M、左右相机）：
```bash
ros2 launch robot_bringup all_data.launch.py scope:=end_effectors topology:=dual
```

启动全部数据（末端数据，再增加 `/lowstate` 转换、左右夹爪状态汇入、整机 URDF、`/robot_description`、统一 `/joint_states` 和 TF）：
```bash
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual
```

参数含义：

| 参数 | 可选值 | 默认值 | 含义 |
|---|---|---|---|
| `scope` | `end_effectors` / `whole_body` | `whole_body` | 只启动末端，或启动末端加整机状态数据 |
| `topology` | `single` / `dual` | `dual` | CANalyst-II 单通道四设备，或双通道每臂两设备 |
| `lowstate_topic` | ROS 话题 | `/lowstate` | Unitree `unitree_hg/LowState` 输入，仅 `whole_body` 使用 |
| `joint_states_topic` | ROS 话题 | `/joint_states` | G1 与左右夹爪最新状态缓存的统一输出话题 |
| `robot_description_topic` | ROS 话题 | `/robot_description` | 组装后的 URDF 输出 |
| `require_pr_mode` | `true` / `false` | `true` | 默认拒绝 `mode_pr != 0`，防止把 A/B 脚踝电机量误作 Pitch/Roll 关节量 |
| `joint_state_publish_rate_hz` | Hz | `100.0` | 统一 `/joint_states` 的固定发布频率；输入回调只更新对应缓存 |
| `enable_grippers_on_start` | `true` / `false` | `true` | 生产拓扑启动后自动配置并使能两只 Gloria-M |
| `use_sim_time` | `true` / `false` | `false` | 是否使用仿真时钟 |

两种 scope 都不启动 8770 或 8200 Dashboard。左右 `camera_node` 按现有设计同时提供 ROS 图像和内置相机页面，因此 8010/8011 会随数据节点启动；这是相机节点自身的一体化能力。

`bash scripts/run_end_effectors.sh single|dual` 是 `scope:=end_effectors` 的清理型快捷脚本，Ctrl-C 时会释放 CAN 进程。

### 阶段二：手动启动 Dashboard
末端总面板必须与阶段一使用相同的 `topology`：
```bash
ros2 launch robot_bringup end_effectors_dashboard.launch.py topology:=dual
# http://<机器人 IP>:8770
```

全身 MIT 控制测试面板：
```bash
ros2 launch robot_bringup whole_body_dashboard.launch.py
# http://<机器人 IP>:8200
```

该入口同时启动 `unitree_g1_description/mit_position_controller` 和 8200 页面。`robot_bringup` 的启动 wrapper 在进程内将 Dashboard 使用的新版本 controller-manager 字段映射到 Foxy 的 `start_controllers`、`stop_controllers`、`start_asap` 与 `claimed_interfaces`；`robot_test_dashboard` submodule 保持原状。页面 Engage `whole_body_controller` 后，统一节点在 controller start 服务内通过 Unitree MotionSwitcher 释放当前运动模式，等待旧 `/lowcmd` 流停止，成功后才从最新反馈姿态开始按固定顺序分发 31 个关节目标到 G1 与两只夹爪。Disengage 会先停止并排空本节点的命令流，再在默认 10 秒窗口内重试恢复 Engage 前的运动模式（当前机器通常为 `ai`），最终以 CheckMode 的实际状态为准；若无法确认恢复，低层输出保持停止，`/controller_manager/switch_controller` 返回失败，避免与迟到的 SelectMode 切换形成双发布。当前 Dashboard 的 Disengage 接口不会透传该失败；实机操作后需独立确认 controller 为 `inactive` 且 CheckMode 已恢复预期模式。未 Engage 时节点不发布运动命令。若使用外部 `ros2_control`，传入 `use_mit_controller:=false`。

### Launch 完整清单
以下逐项列出源码树中的全部 launch。标记为“底层/调试”的入口不应替代 `all_data.launch.py` 作为生产启动方式。

#### `robot_bringup`

| Launch | 启动的资源 | 适用场景 |
|---|---|---|
| `all_data.launch.py` | `scope:=end_effectors` 时包含一个末端拓扑；`scope:=whole_body` 时再包含 `g1_data.launch.py`，将本体与两只夹爪状态发布到统一 `/joint_states`。不含 8770/8200 Dashboard | **推荐生产入口**；一键启动末端或全部数据 |
| `end_effectors_single_bus.launch.py` | 1 个 `can_bridge_ros` 进程；进程内 `/ft_left`、`/ft_right` KWR57 handler；独立 `/grip_left`、`/grip_right`；左右相机及其 8010/8011 内置页面 | 底层单总线拓扑实现；调试 `all_data topology:=single` |
| `end_effectors_dual_bus.launch.py` | 1 个双通道 bridge；进程内 `/ft_arm0`、`/ft_arm1`；独立 `/grip_arm0`、`/grip_arm1`；左右相机及其内置页面 | 底层双总线拓扑实现；调试 `all_data topology:=dual` |
| `end_effectors_dashboard.launch.py` | 仅 `/end_effectors_dashboard` Web 节点；订阅末端话题并代理相机 8010/8011；默认端口 8770 | 数据节点运行后，手动查看和联调左右末端；`topology` 必须匹配数据入口 |
| `whole_body_dashboard.launch.py` | 包含统一 MIT 位置控制节点与主仓库 Foxy 兼容 wrapper；wrapper 原样运行 `robot_test_dashboard`，默认端口 8200 | `scope:=whole_body` 运行后查看和点动整机；可关闭内置控制节点接外部控制栈 |

#### `unitree_g1_description`

| Launch | 启动的资源 | 适用场景 |
|---|---|---|
| `g1_data.launch.py` | 在进程内缓存 `/lowstate` 与左右夹爪最新状态，以固定频率发布一条统一 `JointState`；同时包含唯一的 `description.launch.py` | `all_data scope:=whole_body` 使用的整机数据组件；单独排查 G1 状态链 |
| `description.launch.py` | 仅 `robot_state_publisher`；发布 `/robot_description` 和 TF，订阅已有 `/joint_states` | 已有标准关节状态时，只加载 mode15 整机模型 |
| `mit_control.launch.py` | 提供 dashboard 所需的控制器管理服务和 31 关节位置入口，分发 G1/Gloria MIT 命令 | 单独调试统一控制节点；通常由 `whole_body_dashboard.launch.py` 包含 |

#### `kwr57_ros`

| Launch | 启动的资源 | 适用场景 |
|---|---|---|
| `ft_sensor_debug.launch.py` | 1 个 CAN bridge；默认 KWR57 handler 在 bridge 进程内并发布 Wrench；`use_frame_handler:=false` 时包含 `ft_sensor.launch.py` | **仅用于单只 KWR57 独占硬件调试**；会独占 CANalyst-II，不应与 `all_data` 或其他 bridge 同时运行 |
| `ft_sensor.launch.py` | 仅 1 个独立 KWR57 ROS 节点，不打开 CAN 设备 | 已有外部 bridge 且已配置 RX 路由时，调试 ROS Frame 兼容路径 |
| `web_demo.launch.py` | 仅 `web_wrench`，订阅已有 Wrench，默认端口 8765 | 数据节点运行后手动查看单只力传感器；不占用 CAN |

#### `gloria_ros`

| Launch | 启动的资源 | 适用场景 |
|---|---|---|
| `gripper_debug.launch.py` | 1 个 CAN bridge，并包含 `gripper.launch.py` 启动 1 个 Gloria-M 节点，无网页 | **仅用于单只夹爪独占硬件调试**；会独占 CANalyst-II，不应与 `all_data` 或其他 bridge 同时运行 |
| `gripper.launch.py` | 仅 1 个 Gloria-M 节点，不打开 CAN 设备 | 已有外部 bridge 且已配置专属 RX 路由时调试夹爪节点 |
| `web_gripper.launch.py` | 仅 `gloria_web_gripper`，连接已有目标节点，默认端口 8766 | 数据节点运行后手动查看/控制单只夹爪；不占用 CAN |

#### `can_bridge_ros` 与 `robot_test_dashboard`

| Launch | 启动的资源 | 适用场景 |
|---|---|---|
| `can_bridge_ros/can_bridge_ros.launch.py` | 仅通用 CAN bridge，默认读取 `single_bus.yaml` | 单独验证 CANalyst-II、路由和原始 Frame；独占 CAN 设备 |
| `robot_test_dashboard/dashboard.launch.py` | 仅 8200 Web 节点，默认参数面向通用机器人 | 已有 `/robot_description`、`/joint_states`、TF、`/controller_manager` 的通用 ros2_control 系统；G1 优先使用 bringup 包装入口 |

#### `camera_node`

`camera_node` 当前把 RTSP 采集、ROS Image 发布和 Flask 页面封装在同一节点中，因此以下每个入口都会启动内置 Web；本次不拆分该节点。

| Launch | 启动的资源 | 适用场景 |
|---|---|---|
| `double_camera_launch.py` | `192.168.1.100/.101` 两个相机节点，端口 8010/8011，默认不发布 ROS Image | 历史双相机低带宽预览配置；不是当前 G1 左右手相机配置 |
| `robot_arm_cam_launch.py` | 1 个可参数化相机节点，默认 `192.168.1.102`、端口 8012、发布 `/robot_arm_camera/image_raw` | 通用机械臂单相机调试 |
| `single_stream_test.py` | 1 个固定 `192.168.1.102` 测试节点，端口 8081并发布 ROS Image | 单路 RTSP/ROS 图像快速测试 |
| `ur10e_cam_launch.py` | 从外部 `common.config_manager` 读取 UR10e 相机配置 | 原上游 UR10e 部署；当前 G1 工作区未提供该配置包 |
| `ur15_cam_launch.py` | 从外部 `common.config_manager` 读取 UR15 相机配置 | 原上游 UR15 部署；当前 G1 工作区未提供该配置包 |

### 运行约束

- 同一时刻只能有一个 `can_bridge_ros` 进程独占同一台 CANalyst-II。不要同时运行 `all_data`、单设备 `*_debug.launch.py` 或独立 bridge。
- 单总线下非共享活动 CAN ID 必须互不冲突；双总线下不同物理通道可以复用 CAN ID。
- 末端 Dashboard 的 `topology` 必须与 `all_data` 一致，否则会订阅错误的节点名和话题。
- `LowState.motor_state[0:29]` 按官方 G1 29 轴索引映射到 mode15 URDF。LowState 与左右夹爪回调只更新各自缓存；定时器按 `joint_state_publish_rate_hz` 发布一条当前最新快照，并统一使用发布时刻，避免不同输入频率造成分体 TF。
- 默认只接受 `LowState.mode_pr == 0`。仓库没有可信的 A/B→Pitch/Roll 逆解，不能把 AB 电机角直接发布为 URDF 脚踝关节角。
- MIT 控制节点只在控制器已激活、`LowState` 与全部 31 个关节反馈新鲜、目标在 URDF 限位内且连续时发布。非法目标会被丢弃并继续保持；网页命令超过默认 0.25 秒未更新时改为保持最新反馈姿态。状态反馈丢失时退出低层控制并尝试恢复原运动模式。
- G1 的 29 组 `kp/kd` 来自 `unitree_g1_description/config/default_29dof_param.yaml`。Gloria-M 默认 `kp=10`、`kd=5`；其 SDK 将 `kd` 固定映射到 `[0,5]` 的 12 bit 字段，因此 5 是协议允许的最大值。
- `all_data.launch.py` 的生产拓扑默认自动使能两只 Gloria-M；需要上电保持失能时传入 `enable_grippers_on_start:=false`。独立 `gloria_ros` 调试入口仍默认不自动使能。
- BEST_EFFORT 高频话题可使用 `ros2 topic echo --qos-reliability best_effort`；1 kHz 控制订阅建议采用 `rclcpp`、BEST_EFFORT 和 `KEEP_LAST(64)`。

各包细节见 `sdk/CAN-SDK/README.md`、`src/can_bridge_ros/README.md`、`src/camera_node/README.zh.md`、`src/kwr57_ros/README.md` 和 `src/robot_bringup/README.md`。
