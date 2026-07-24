# Unitree_G1_Workspace

Unitree G1 的 ROS 2 Foxy 工作区，覆盖整机位置控制、IK、双夹爪、双六轴力传感器、双相机和 Web 调试。生产入口由 `robot_bringup` 统一启动；各设备包仍可单独用于调试。

| 模块 | 负责 |
|---|---|
| `robot_bringup` | 组合整机或末端设备的生产 launch |
| `unitree_g1_ros2_control` | 把 FPC/JTC 的关节位置补齐为 G1 `LowCmd` 和夹爪 MIT 命令，并接入状态反馈 |
| `canalystii_native_bridge`、`gloria_ros`、`camera_node`、`can_bridge_ros` | CAN 适配器、夹爪协议和相机设备通信<br>已经被 `canalystii_native_bridge` 取代 |
| `unitree_g1_description` | URDF、mesh、关节限位和 ros2_control 资源声明 |
| `inverse_kinematics_toolkit`、Dashboards | 将末端目标或人工操作转换为控制器命令；不直接驱动硬件 |

常用入口：
```bash
source scripts/env.sh
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual
```

下面先说明各子系统的边界，再给出目录、安装和调试细节。

## 末端执行器与相机

末端执行器（力传感器 + 夹爪）的生产链由 `canalystii_native_bridge` 独占 CANalyst-II。该 C++ 进程直接用 libusb 管理双通道，在进程内完成 KWR57 三帧组包，并通过 `can_msgs/Frame` 路由连接独立的 Gloria-M 节点。左右 IP 相机各由一个 `camera_node` 进程读取 RTSP 并发布图像。`can_bridge_ros`、`can_sdk` 和 `kwr57_ros` 保留为独立 Python 调试/兼容入口，不参与生产末端 launch。

双总线生产配置保持每通道 8 个异步 RX transfer，并保持 `io_diagnostics` 关闭。2026-07-23 的双 KWR57 + 双 Gloria-M + active FPC 30 秒实机验收中，左右 KWR57 ROS receive 最大 gap 为 `7.027/7.433 ms`，实际 CAN TX 为 `99.999/100.001 Hz`。根因、测试边界和禁止项见 [`canalystii_native_bridge/README.md`](src/canalystii_native_bridge/README.md)。

设备：2 个力传感器（KWR57）+ 2 个夹爪（Gloria-M）+ 2 个 IP 相机（左手 `192.168.123.97`、右手 `192.168.123.98`）。CAN 设备支持两种接线：
- **单总线**：所有设备都在 CANalyst-II 的统一 CAN 上（`/can0` 或者 `/can1`）。
- **双总线**：一个力传感器 + 一个夹爪为一组（一个手臂），分别接两条总线（`/can0`、`/can1`）。

```
第1层 native bridge  : C++/libusb 独占 CANalyst-II，双通道收发与 KWR57 组包
第2层 设备节点        : gloria_ros 订阅专属 RX；camera_node 读取 RTSP
第3层 end effectors   : 生成 native KWR57 配置、Gloria 路由和左右相机节点
调试链                : can_sdk + can_bridge_ros + kwr57_ros，不与生产 bridge 同时运行
```

消息契约使用上游 ROS 2 [`can_msgs`](https://index.ros.org/p/can_msgs/) 包提供的 `can_msgs/Frame`（与 [ros2_socketcan](https://index.ros.org/p/ros2_socketcan/) 一致）。它是 ROS 消息定义，不属于 `python-can` 或本项目的 `can_sdk`；Foxy 对应系统包为 `ros-foxy-can-msgs`。


## 宇树 G1

机器人本体使用官方 [`unitree_ros2`](https://github.com/unitreerobotics/unitree_ros2) 消息定义。G1 只需要以下两个包：

- `unitree_api`：机器人服务请求/响应消息。
- `unitree_hg`：G1/H1 系列的状态与控制消息。

本项目不编译 `unitree_go` 和 `unitree_ros2_example`。

`unitree_g1_description` 只提供整机模型资源。`unitree_g1_ros2_control` 提供统一硬件插件、互斥 FPC/JTC 和状态 broadcaster：FPC 只校验全量位置命令的维度与有限值，controller 将目标写入 position interface，`G1TopicSystem` 再按模型 command-interface 范围做最终 clamp、补齐 MIT 参数，并生成 G1 `/lowcmd` 与左右 Gloria-M `MitCommand`。


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
    ├── canalystii_native_bridge/ 生产用 C++ CANalyst-II/KWR57 bridge
    ├── camera_node/              左右 IP 相机 RTSP、ROS 图像与 Web 预览
    ├── kwr57_ros/                力传感器 ROS 设备节点（import kwr57_sensor）
    ├── gloria_ros/               夹爪 ROS 设备节点 + MIT/PV 消息（复用 Gloria SDK 协议）
    ├── inverse_kinematics_toolkit/ git submodule（Pinocchio IK、Pose Commander 与 Dashboard）
    ├── robot_bringup/            全身控制与末端设备的分层 launch 编排
    ├── robot_test_dashboard/     git submodule（机器人测试 Dashboard）
    ├── unitree_g1_description/   整机 description 包（model/ 为 URDF submodule）
    ├── unitree_g1_ros2_control/  G1/Gloria/KWR57 统一硬件插件和互斥 FPC/JTC
    └── unitree_ros2/             git submodule（仅构建 unitree_api、unitree_hg）
```

- SDK 保留：`CAN-SDK`（模块 `can_sdk`）、`KWR57-SDK`（模块 `kwr57_sensor`）和 `gloria_m_sdk` 均可脱离 ROS 使用；ROS 封装只复用基础 I/O 和设备协议，不重复实现。
- 三个 SDK 均不作为 ROS 包，且不由 colcon 构建；`scripts/env.sh` 统一把它们的源码目录加入 `PYTHONPATH`。ROS 节点不需要先安装本地 SDK，也不在节点代码中修改 `sys.path`。
- `can_sdk` 刻意不提供多订阅：直连 SDK 的 `recv()` 是单消费者语义；它和 `can_bridge_ros` 只用于独立调试，生产多设备系统由 `canalystii_native_bridge` 成为唯一 USB 所有者。
- 夹爪 SDK 整体声明 Python≥3.11；本项目只使用其 `protocol_mit`/`types` 逻辑并已做 Python 3.8 静态语法检查，不打开 SDK 的串口转 CAN 传输层。由于 Python 导入包子模块时仍会执行上游 `gloria_m_sdk/__init__.py`，运行环境当前仍需提供上游依赖 `pyserial`。


## 环境与安装
ROS 2 节点跑在 **Foxy 系统 `python3`（3.8）**；运行用 **CycloneDDS**（默认 FastRTPS 会刷 `std::bad_alloc`）。机器人出厂环境已在 `~/cyclonedds_ws` 安装兼容版本，项目继续复用该环境；仓库内的 `src/unitree_ros2/cyclonedds_ws` 只用于构建 Unitree 消息包。

```bash
# ROS/Python 图像栈统一使用 Ubuntu 软件包，避免与 Foxy cv_bridge 产生 ABI 冲突；
# ffprobe 和 ffplay 均由 ffmpeg 软件包提供。
sudo apt-get install -y ros-foxy-can-msgs \
    ros-foxy-cv-bridge ros-foxy-rmw-cyclonedds-cpp \
    ros-foxy-controller-manager-msgs ros-foxy-ros2-control \
    ros-foxy-ros2-controllers ros-foxy-joint-trajectory-controller \
    ros-foxy-rosidl-generator-dds-idl \
    ros-foxy-robot-state-publisher ros-foxy-xacro \
    ffmpeg libusb-1.0-0-dev libyaml-cpp-dev \
    python3-flask python3-opencv python3-numpy
source /opt/ros/foxy/setup.bash
# Foxy 使用 Python 3.8；固定 pip/NumPy/Pinocchio，避免 NumPy 2.x 覆盖 ROS 图像栈。
python3 -m pip install --user 'pip==23.3.2'
python3 -m pip install --user 'numpy<1.24' 'pin==2.6.21'
# 仅 Python CAN/KWR57 调试入口需要这些依赖
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
    ikt_common ikt_core ikt_interfaces ikt_inverse_kinematics ikt_pose_commander \
    camera_node can_bridge_ros canalystii_native_bridge gloria_ros kwr57_ros robot_bringup \
    robot_test_dashboard unitree_g1_description unitree_g1_ros2_control
source scripts/env.sh
```

CANalyst-II 需 udev 权限（VID:PID `04d8:0053`），见 `src/canalystii_native_bridge/README.md`。

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

### 推荐整机启动
终端 A 启动全部硬件、唯一 controller manager 和两个 inactive motion controller：
```bash
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual
```

终端 B 按需启动 Dashboard：
```bash
ros2 launch robot_bringup whole_body_dashboard.launch.py
# http://<机器人 IP>:8200
```

第一条命令已经包含 `unitree_g1_ros2_control/control.launch.py`。不要再单独启动第二个 `control.launch.py`，也不要单独重复启动 Gloria-M、KWR57 或 CAN bridge。Dashboard 只连接已有 `/controller_manager`，不会创建第二套 manager。

启动后先检查 controller 保持 inactive：
```bash
ros2 control list_controllers --controller-manager /controller_manager
ros2 control list_hardware_interfaces --controller-manager /controller_manager
```

应看到 `joint_state_broadcaster`、`pelvis_imu_broadcaster` 为 `active`，`forward_position_controller`（FPC）和 `joint_trajectory_controller`（JTC）均为 `inactive`，31 个 position command interface 均为 `unclaimed`。Engage 后只允许一个 motion controller 为 `active`，31 个接口都应显示 `claimed`。确认现场安全、反馈正常且外部 `/lowcmd` 已停止后，才可 Engage。

终端 C 可启动 IKT Pose Commander 与 8180 Dashboard：
```bash
ros2 launch robot_bringup ikt_pose_commander.launch.py
# http://<机器人 IP>:8180
```

该入口默认控制 `right_gripper_base`、以 `torso_link` 为参考帧并保持 disabled。**Track robot** 使用 FPC；**Snap robot** 与 `return_to_start` 使用 JTC。Commander 通过 `/controller_manager/switch_controller` 一停一启，两个 controller 对相同资源的 claim 提供真实互斥。只启动 Commander、不启动 8180 页面时传入 `enable_dashboard:=false`。

### 其他启动方式
只启动末端数据（CAN、双 KWR57、双 Gloria-M、左右相机）：
```bash
ros2 launch robot_bringup all_data.launch.py scope:=end_effectors topology:=dual
```

`scope:=whole_body` 在同一末端拓扑之外启动唯一的真实 `controller_manager`、统一硬件插件、`robot_state_publisher`、100 Hz JointState/IMU broadcaster，以及保持 `inactive` 的 FPC/JTC。manager 的更新率为 500 Hz；未 Engage 时 31 个 command interface 均未 claim，插件不会发布 `/lowcmd` 或 Gloria-M MIT 命令。

| 参数 | 可选值 | 默认值 | 含义 |
|---|---|---|---|
| `scope` | `end_effectors` / `whole_body` | `whole_body` | 只启动末端，或额外启动整机 ros2_control 栈 |
| `topology` | `single` / `dual` | `dual` | CANalyst-II 单通道四设备，或双通道每臂两设备 |
| `enable_grippers_on_start` | `true` / `false` | `true` | 末端设备启动后是否预先使能 Gloria-M；不等同于激活 controller |
| `controller_manager` | ROS 节点路径 | `/controller_manager` | Dashboard 和 spawner 连接的唯一 manager |
| `lowstate_topic` | ROS 话题 | `/lowstate` | Unitree `LowState` 输入 |
| `joint_states_topic` | ROS 话题 | `/joint_states` | 31 轴标准状态输出 |
| `robot_description_topic` | ROS 话题 | `/robot_description` | 展开后的整机 URDF 输出 |
| `require_pr_mode` | `true` / `false` | `true` | Engage 时要求 `mode_pr == 0` |
| `use_sim_time` | `true` / `false` | `false` | 是否使用仿真时钟 |

两种 scope 都不启动 8770/8200 Dashboard。相机链保持原设计，左右 `camera_node` 随末端拓扑启动并继续提供 ROS Image 与 8010/8011 内置页面。

已有匹配 `topology` 的 CAN bridge、Gloria-M 和 KWR57 节点时，才单独启动控制栈：

```bash
ros2 launch unitree_g1_ros2_control control.launch.py topology:=dual
```

该入口不打开 CAN、不创建设备节点；缺少 `/lowstate`、Gloria `JointState` 或夹爪服务时，controller 会保持 inactive 并拒绝 Engage。

### 阶段二：按需启动 Dashboard
末端页面必须使用与数据入口相同的拓扑：
```bash
ros2 launch robot_bringup end_effectors_dashboard.launch.py topology:=dual
# http://<机器人 IP>:8770
```

8770 默认是监视模式：显示相机、KWR57 和 Gloria 反馈，但不创建 `MitCommand` publisher，也不调用夹爪 enable/disable。它可以和 8200 同时运行。仅在 `scope:=end_effectors`、没有任何 ros2_control 夹爪 controller 时，才可显式追加 `allow_gripper_control:=true` 恢复独立末端控制；不要在整机控制期间打开该参数。

整机 Dashboard 只发现 controller、执行 Engage/Disengage，并按类型向 FPC 的 `/forward_position_controller/commands` 或 JTC 的轨迹接口发送目标，不创建 manager 或控制适配器。Foxy 兼容 wrapper 仅映射 `SwitchController` 字段，并从 inactive controller 的 `joints` 参数补全页面所需接口元数据。

Engage 会依次检查 31 轴反馈 freshness 与 PR mode、释放现有 MotionSwitcher 模式、等待外部 `/lowcmd` 静默、使能所 claim 的 Gloria-M，并在二次状态检查后才开放输出。Disengage 先阻止低层输出、失能夹爪，再恢复接管前的运动模式。任一步失败都保持输出关闭并返回切换失败。完整事务见 [unitree_g1_ros2_control/README.md](src/unitree_g1_ros2_control/README.md)。

### 状态与资源检查
```bash
ros2 control list_controllers --controller-manager /controller_manager
ros2 control list_hardware_interfaces --controller-manager /controller_manager
ros2 topic hz /joint_states
ros2 topic hz /pelvis_imu_broadcaster/imu
```

正常启动后应看到两个 broadcaster 为 `active`、FPC/JTC 均为 `inactive`，31 个 position command interface 全部 `unclaimed`。KWR57 原始 Wrench 继续由设备节点以 1 kHz 发布；默认不启动 FT broadcaster，避免重复 DDS 流。确需标准 ros2_control FT 输出时可手动 spawn `left_ft_broadcaster`/`right_ft_broadcaster`。

### 通信路径与 DDS 开销
ros2_control 插入的是同进程控制抽象，不是一个 DDS relay。`controller_manager`、`ForwardCommandController`、`JointTrajectoryController` 和 `G1TopicSystem` 都加载在同一个 `ros2_control_node` 进程中；active controller 的 `update()` 写 command interface，随后 manager 直接调用硬件插件的 `write()`，这段是 C++ 内存访问，没有 ROS 消息、序列化或 DDS hop。

| 路径 | 边界 | 是否 DDS | 说明 |
|---|---|---|---|
| Dashboard/IK -> `/forward_position_controller/commands` | 外部应用 -> controller | 是 | 外部目标输入；controller 收到后进入实时缓冲 |
| Commander -> `/joint_trajectory_controller/follow_joint_trajectory` | 外部应用 -> controller | 是 | Snap robot 与 return-to-start 的标准 JTC action |
| controller -> command interface -> `G1TopicSystem::write()` | 同一 `ros2_control_node` | 否 | 直接内存接口与函数调用 |
| `G1TopicSystem` -> `/lowcmd` -> G1 | PC2 -> Unitree 低层 | 是 | Unitree 官方低层接口；LowCmd 组包和 CRC 在硬件插件进程内完成 |
| `G1TopicSystem` -> `MitCommand` -> Gloria 节点 | ros2_control -> 独立夹爪进程 | 是，100 Hz | 默认 controller claim 两侧 eccentric interface；单侧反馈超时只停止该侧 |
| Gloria 节点 <-> bridge | 独立进程之间 | 是，`can_msgs/Frame` | 为保留独立 Gloria 节点和调试入口而保留的原有边界 |
| CAN -> native KWR57 device | 同一 C++ bridge 进程 | 否 | 原始三帧直接组包，不发布中间 CAN Frame |
| native KWR57 device -> raw Wrench -> `G1TopicSystem` | bridge 进程 -> ros2_control | 是，1 kHz | 插件只是已有 raw Wrench 的订阅者，不再转发 |

因此，controller 层本身没有增加 DDS hop。上层通过 FPC commands 或 JTC action 进入当前 active controller；controller 到 hardware interface 仍是进程内接口。KWR57 默认不启动 FT broadcaster，所以不会自动产生第二条 1 kHz Wrench 流。

### 主要 Launch

| Launch | 启动的资源 | 适用场景 |
|---|---|---|
| `robot_bringup/all_data.launch.py` | 末端拓扑；`scope:=whole_body` 时再包含唯一 ros2_control 栈 | 推荐生产入口 |
| `robot_bringup/end_effectors_*_bus.launch.py` | 单/双总线 bridge、KWR57、Gloria-M 和相机 | 末端拓扑底层入口 |
| `robot_bringup/end_effectors_dashboard.launch.py` | 8770 末端监视网页；默认不创建夹爪命令源 | 可与 8200 同时运行 |
| `robot_bringup/whole_body_dashboard.launch.py` | 仅 8200 controller Dashboard | 已有真实 manager 时联调 |
| `robot_bringup/ikt_pose_commander.launch.py` | Foxy 兼容 Commander、可选 8180 Dashboard | FPC 连续跟踪、JTC Snap/return-to-start |
| `unitree_g1_ros2_control/control.launch.py` | 唯一 manager、硬件插件、RSP、broadcaster、inactive FPC/JTC | 独立整机控制入口 |
| `unitree_g1_description/description.launch.py` | 仅模型、RSP 和 TF | 已有 `/joint_states` 时查看模型 |

单设备调试入口仍由 `kwr57_ros`、`gloria_ros`、`can_bridge_ros` 和 `camera_node` 各包提供；相机相关 launch 未改变。

### 运行约束
- 同一时刻只能有一个 `can_bridge_ros` 进程独占同一台 CANalyst-II；不要同时运行 `all_data`、单设备 `*_debug.launch.py` 或独立 bridge。
- 同一 ROS graph 只能启动一个目标路径相同的 `controller_manager`。Dashboard 不负责启动或代理 manager。
- 单总线下非共享活动 CAN ID 必须互不冲突；双总线下不同物理通道可以复用 CAN ID。
- 双总线 Gloria-M 明确发布 `left_eccentric_joint`、`right_eccentric_joint`；硬件插件仍校验消息位置为有限值。
- 启动反馈到达前，broadcaster 使用有限的零位关节状态和单位 IMU 四元数，避免污染 TF；这些中性值不会绕过 `received` 与 freshness 安全门，controller 仍无法 Engage。
- 默认只允许 `LowState.mode_pr == 0`。仓库没有可信的 A/B 到 Pitch/Roll 逆解，不能把 AB 电机角直接作为 URDF 脚踝角。
- G1 的 29 组 `kp/kd` 位于 `unitree_g1_ros2_control/config/default_29dof_param.yaml`。Gloria-M 默认 `kp=10`、`kd=5`，其中 `kd=5` 是协议编码上限。
- `enable_grippers_on_start:=true` 只配置并使能设备；controller 激活仍会重新执行完整安全事务。需要上电保持失能时传入 `false`。
- BEST_EFFORT 高频话题可使用 `ros2 topic echo --qos-reliability best_effort`；KWR57 的 1 kHz 订阅建议使用 `rclcpp`、BEST_EFFORT 和 `KEEP_LAST(64)`。

各包细节见 [robot_bringup/README.md](src/robot_bringup/README.md)、[unitree_g1_ros2_control/README.md](src/unitree_g1_ros2_control/README.md)、[unitree_g1_description/README.md](src/unitree_g1_description/README.md)、[kwr57_ros/README.md](src/kwr57_ros/README.md) 和 [camera_node/README.zh.md](src/camera_node/README.zh.md)。
