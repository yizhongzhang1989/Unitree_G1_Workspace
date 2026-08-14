# Unitree_G1_Workspace

Unitree G1 的 ROS 2 Humble 工作区，覆盖整机位置控制、IK、双夹爪、双六轴力传感器、双相机和 Web 调试。生产入口由 `robot_bringup` 统一启动；各设备包仍可单独用于调试。

| 模块 | 负责 |
|---|---|
| `robot_bringup` | 组合整机或末端设备的生产 launch |
| `unitree_g1_ros2_control` | 把 FPC/JTC 的关节位置补齐为 G1 `LowCmd` 和夹爪 MIT 命令，并接入状态反馈 |
| `g1_motion_control` | FPC 之上的整机 31 轴运动控制层：下肢 15 轴走 ONNX 策略（输入 `vx/vy/w/h`），上肢 14 轴走双臂 IK，夹爪 2 轴透传；内附 VR / 键盘遥操 |
| `canalystii_native_bridge`、`gloria_ros`、`camera_node`、`can_bridge_ros` | CAN 适配器、夹爪协议和相机设备通信<br>已经被 `canalystii_native_bridge` 取代 |
| `head_sensors` | 头部传感器：Livox MID-360 雷达接入（TF、空点过滤、IMU 单位修正）+ RealSense D435i（官方 realsense2_camera） |
| `unitree_g1_description` | URDF、mesh、关节限位和 ros2_control 资源声明 |
| `arm_gravity_compensation` | 基于 LowState/头部 IMU、Pinocchio 和纯 `tau` LowCmd 的双臂重力参数标定 |
| `inverse_kinematics_toolkit`、Dashboards | 将末端目标或人工操作转换为控制器命令；不直接驱动硬件 |
| `g1_motion_control` | 整机 31 轴运动控制层 + 遥操台（在 FPC 之上） |


## 快速开始

**开发和运行都在 Docker 容器里进行。** 宿主是 Jetson 的 Ubuntu 20.04（JetPack 5），上面只有 ROS 2 Foxy；Humble 由 `.devcontainer/` 下的镜像提供。工作区固定挂到容器里的 `/workspace`。

```bash
# 0) 首次使用需在 docker 组里；刚加完组要重新登录，或临时用 sg docker -c "..."
sudo usermod -aG docker "$USER"

# 1) 构建开发镜像（约 2 GB，只需做一次；改了 Dockerfile 再重跑）
cd ~/Unitree_G1_Workspace
.devcontainer/dev.sh build

# 2) 进容器
.devcontainer/dev.sh

# 3) 以下都在容器里（工作目录 /workspace）
colcon build --symlink-install --packages-ignore unitree_go unitree_ros2_example
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual
```

ROS 环境由容器自动装配，**不需要手动 `source`**。只有当 `install/` 是这次新建出来的、当前 shell 还没加载到它时，才需要 `source scripts/env.sh` 刷一下。

编译优化由工作区根目录的 [`colcon_defaults.yaml`](colcon_defaults.yaml) 统一给到 `RelWithDebInfo`（`-O2 -g`）。colcon 默认不设 `CMAKE_BUILD_TYPE`，那样一个 `-O` 都没有——实测控制环上的代码差 24 倍。**命令行显式传 `--cmake-args` 会整体覆盖该文件而不是合并**，临时加参数时要把 build type 一起写全。

也可以不进交互 shell，直接一次性执行：
```bash
.devcontainer/dev.sh colcon build --symlink-install --packages-ignore unitree_go unitree_ros2_example
.devcontainer/dev.sh ros2 control list_controllers
```

**用 VS Code 开发**：装 **Dev Containers** 扩展（`ms-vscode-remote.remote-containers`，跟 Docker/Containers 扩展不是一个东西），先跑一次上面的 `dev.sh build`，然后 "Reopen in Container"。VS Code Server 会装进容器，`/opt/ros/humble` 就是普通本地路径，补全直接可用。

下面先说明各子系统的边界，再给出目录、容器和调试细节。

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

消息契约使用上游 ROS 2 [`can_msgs`](https://index.ros.org/p/can_msgs/) 包提供的 `can_msgs/Frame`（与 [ros2_socketcan](https://index.ros.org/p/ros2_socketcan/) 一致）。它是 ROS 消息定义，不属于 `python-can` 或本项目的 `can_sdk`；对应系统包为 `ros-humble-can-msgs`，已装在开发镜像里。


## 头部传感器

头顶两个传感器由 [`head_sensors`](src/head_sensors/README.md) 接入，两者的驱动来源完全不同：

- **Livox MID-360 雷达**（`192.168.123.120`）：由机器人内部的 `lidar_driver` 服务驱动，直接以 DDS 发 `/utlidar/cloud_livox_mid360`（10 Hz）和 `/utlidar/imu_livox_mid360`（200 Hz），**本机不装 Livox SDK**。`head_lidar_node` 只补 `mid360_link → livox_frame` 静态 TF、过滤约 55% 的无回波空点、并把 Livox 以 g 为单位的 IMU 加速度换算成 m/s²。
- **RealSense D435i 深度相机**：USB 直连本机 NX，用官方 `realsense2_camera` 驱动，话题收在 `/head/camera/*`，默认 424x240x30。依赖 `ros-humble-librealsense2`、`ros-humble-realsense2-camera{,-msgs}`、`ros-humble-realsense2-description`，已写进 `.devcontainer/Dockerfile`。`head_camera.launch.py` 额外补一条 `d435_link → camera_link` 挂载 TF —— 官方驱动只发相机自己那棵子树，不知道相机装在机器人哪里。

头是可转的，撞歪了用 `verify_head_view` 校：它拍一张实拍图、读当前关节角、用 URDF 渲染同一视角，把轮廓叠到实拍图上，对不齐就转头再拍。注意投影时必须从 TF 取 `d435_link → camera_color_optical_frame` 的实测外参（彩色镜头偏离挂载原点 15.3 mm），当成纯旋转会让渲染整体横移十几个像素。

`head_sensors` 里的 URDF 渲染部分（`urdf_view.py` + `render_head_view.py`）**不依赖 ROS**，可以单独拷走交付，只要 pinocchio / numpy / opencv。

## 宇树 G1

机器人本体使用官方 [`unitree_ros2`](https://github.com/unitreerobotics/unitree_ros2) 消息定义。G1 只需要以下两个包：

- `unitree_api`：机器人服务请求/响应消息。
- `unitree_hg`：G1/H1 系列的状态与控制消息。

本项目不编译 `unitree_go` 和 `unitree_ros2_example`。

`unitree_g1_description` 只提供整机模型资源。`unitree_g1_ros2_control` 提供统一硬件插件、互斥 FPC/JTC 和状态 broadcaster：FPC 只校验全量位置命令的维度与有限值，controller 将目标写入 position interface，`G1TopicSystem` 不限制有限 target 的数据范围，直接补齐 MIT 参数并生成 G1 `/lowcmd` 与左右 Gloria-M `MitCommand`。这是因为阻抗控制需要通过目标位置相对反馈位置的偏移产生力矩。


## 目录

```
Unitree_G1_Workspace/             一个 colcon workspace
├── README.md
├── .gitignore
├── .gitmodules
├── .devcontainer/                Humble 开发容器（Dockerfile / docker-compose.yml / devcontainer.json / 脚本）
├── scripts/                      env.sh（进入环境）
├── sdk/                          纯 Python SDK（不参与 colcon 构建）
|   ├── CAN-SDK/                  通用 CAN 基础库（无 ROS、无设备协议）
|   ├── KWR57-SDK/                力传感器 SDK（纯Python，pip 安装；非ROS可用）
|   └── Gloria-M-SDK/             git submodule（云犀夹爪 SDK）
└── src/                          colcon 扫描的 ROS2 包
    ├── can_bridge_ros/           通用 ROS 2 CAN bridge（多通道）【已经被 canalystii_native_bridge 取代】
    ├── canalystii_native_bridge/ 生产用 C++ CANalyst-II/KWR57 bridge
    ├── camera_node/              左右 IP 相机 RTSP、ROS 图像与 Web 预览
    ├── head_sensors/             头部 Livox MID-360 雷达接入与 RealSense D435i 集成
    ├── kwr57_ros/                力传感器 ROS 设备节点（import kwr57_sensor）【已经被 canalystii_native_bridge 取代】
    ├── gloria_ros/               夹爪 ROS 设备节点 + MIT/PV 消息（复用 Gloria SDK 协议）
    ├── inverse_kinematics_toolkit/ [git submodule] Pinocchio IK、Pose Commander 与 Dashboard
    ├── robot_bringup/            全身控制与末端设备的分层 launch 编排
    ├── robot_test_dashboard/     [git submodule] 机器人测试 Dashboard
    ├── unitree_g1_description/   整机 description 包（model/ 为 URDF submodule）
    ├── arm_gravity_compensation/ 双臂重力参数采点、EM 标定与 Web 工作流
    ├── g1_motion_control/     整机 31 轴运动控制层 + 遥操台（在 FPC 之上）
    ├── unitree_g1_ros2_control/  G1/Gloria/KWR57 统一硬件插件和互斥 FPC/JTC
    └── unitree_ros2/             [git submodule] 官方消息结构（仅构建 unitree_api、unitree_hg）
```

- SDK 保留：`CAN-SDK`（模块 `can_sdk`）、`KWR57-SDK`（模块 `kwr57_sensor`）和 `gloria_m_sdk` 均可脱离 ROS 使用；ROS 封装只复用基础 I/O 和设备协议，不重复实现。
- 三个 SDK 均不作为 ROS 包，且不由 colcon 构建；`scripts/env.sh` 统一把它们的源码目录加入 `PYTHONPATH`。ROS 节点不需要先安装本地 SDK，也不在节点代码中修改 `sys.path`。
- `can_sdk` 刻意不提供多订阅：直连 SDK 的 `recv()` 是单消费者语义；它和 `can_bridge_ros` 只用于独立调试，生产多设备系统由 `canalystii_native_bridge` 成为唯一 USB 所有者。
- 夹爪 SDK 整体声明 Python≥31.1；本项目只使用其 `protocol_mit`/`types` 逻辑，不打开 SDK 的串口转 CAN 传输层。由于 Python 导入包子模块时仍会执行上游 `gloria_m_sdk/__init__.py`，运行环境当前仍需提供上游依赖 `pyserial`。


## 开发容器

宿主是 Jetson（JetPack 5 / L4T R35，Ubuntu 20.04 / Python 3.8），只带 ROS 2 Foxy。本项目跑在 **ROS 2 Humble + Python 3.10** 上，由 `.devcontainer/Dockerfile` 提供：基于官方 `ros:humble-ros-base`（arm64），apt 换 USTC 源。宿主的 Foxy 不受影响，也不再使用。

`.devcontainer/` 的构成：

| 文件 | 作用 |
|---|---|
| `Dockerfile` | 镜像定义 |
| `docker-compose.yml` | **运行参数的唯一来源**，devcontainer 与 `dev.sh` 共用，不抄两份 |
| `devcontainer.json` | 只写 `dockerComposeFile` + `service` + 扩展列表 |
| `dev.sh` | `dev.sh build` 构建镜像，`dev.sh [命令]` 进容器；给普通 SSH 会话、开机自启和 CI 用，VS Code 里用不到 |

ROS 环境不在这里：它定义在工作区的 [`scripts/env.sh`](scripts/env.sh)，镜像的 ENTRYPOINT 和 `~/.bashrc` 都引用它。那份文件**不 COPY 进镜像**，直接用挂载进来的那一份，改了立刻生效、不用重建镜像。

### 构建镜像
```bash
.devcontainer/dev.sh build       # 产出 g1-humble:latest，约 2 GB
```
本机直连不到镜像站（`mirrors.aliyun.com` 只能经代理访问，`mirrors.ustc.edu.cn` 和 `pypi.org` 则完全不可达），`dev.sh build` 会自动把宿主的 `ALL_PROXY`（如 `socks5h://127.0.0.1:1080`）作为构建期 `http_proxy`/`https_proxy` 传入；这些只在构建期生效，不会写进镜像。代理地址不同时用 `G1_BUILD_PROXY=... .devcontainer/dev.sh build` 覆盖。

compose 里 `build.network: host` 是必需的——BuildKit 的 RUN 步骤默认跑在自己的网络沙箱里，`127.0.0.1` 不是宿主的 loopback，代理会连不上。

VS Code 的 "Reopen in Container" 不经过 `dev.sh`，拿不到 `G1_BUILD_PROXY`，所以 compose 里把默认值写成了 `socks5h://127.0.0.1:1080`；代理地址不是这个的话，先跑一次 `dev.sh build` 把镜像做好再进容器。

### 进容器
```bash
.devcontainer/dev.sh                # 交互 shell
.devcontainer/dev.sh <命令...>      # 一次性执行，退出即销毁
```
关键运行参数（都在 `docker-compose.yml` 里）：

| 参数 | 为什么 |
|---|---|
| `privileged: true` + `/dev:/dev` | USB（CANalyst-II）和相机设备直通 |
| `network_mode: host` | 与机器人同一网络栈，DDS 能在 `eth0` 上发现 `/lowstate` |
| `ipc: host` | DDS 共享内存 |
| `ulimits: rtprio 99 / memlock -1` | 让 `ros2_control_node` 拿到 SCHED_FIFO 和 mlockall |
| `cpu_rt_runtime: 200000` | cgroup 实时带宽。**依赖下一节的宿主配置，没配就得删掉这行，否则容器起不来** |
| `..:/workspace` | 固定挂到 `/workspace`，不依赖宿主克隆路径 |
| `~/.ros` 挂载 | 重力标定结果（`~/.ros/arm_gravity_compensation/`）持久化 |
| 镜像内用户与宿主同 UID/GID | bind mount 里新建的文件不会变成 root 所有 |

`build/`、`install/`、`log/` 都落在宿主工作区，容器销毁不丢。`devcontainer.json` 里设了 `shutdownAction: none`，关掉 VS Code 窗口**不会**停容器——机器人上不该因为关个编辑器就把控制栈杀了；要停用 `docker compose -f .devcontainer/docker-compose.yml down`。

### 实时调度（一次性宿主配置）
宿主内核开了 `CONFIG_RT_GROUP_SCHED`（cgroup v1），**非根 cgroup 的实时带宽默认为 0**，此时 `sched_setscheduler()` 一律返回 EPERM——`privileged` 和 `RLIMIT_RTPRIO` 都救不了。`ros2_control_node` 会因此退回 SCHED_OTHER，500 Hz 控制环和 Flask、IK、相机编码抢同一批时间片。

dockerd 必须先拿到实时带宽，它才会在建容器时把配额递归写到 `/docker` 父 cgroup：
```bash
sudo cp /etc/docker/daemon.json /etc/docker/daemon.json.bak
# 往 daemon.json 里加 "cpu-rt-period": 1000000 与 "cpu-rt-runtime": 950000
sudo systemctl restart docker
```

验证：
```bash
.devcontainer/dev.sh chrt -f 50 true && echo OK
# 起控制栈后应看到：Successful set up FIFO RT scheduling policy with priority 50.
```

注意 `cpu_rt_runtime` 是**静态预算**，每个容器都从父级的 950000 里扣，扣完下一个容器就起不来。实测控制环那个 FIFO 线程只占 3% CPU（整进程 45% 是 DDS executor 线程，不吃 RT 配额），所以配了 200000，可并存 4 个容器。怀疑被限流时看 `dmesg | grep "RT throttling"`。

### 构建项目
`sdk/` 下的纯 Python SDK 不参与 colcon 构建。`unitree_go` 和官方示例是没用的包，被排除在外：
```bash
# 容器内
colcon build --symlink-install --packages-ignore unitree_go unitree_ros2_example
```

Docker下 ROS 环境不用手动 source：[`scripts/env.sh`](scripts/env.sh) 会加载 Humble、工作区 `install/setup.bash`、三个 SDK 的 `PYTHONPATH`，以及 `RMW_IMPLEMENTATION` 和 `CYCLONEDDS_URI`。它同时挂在镜像的 ENTRYPOINT 和 `~/.bashrc` 上，因为 **VS Code 开的终端不走 ENTRYPOINT**。首次构建出 `install/` 后，当前 shell 需要 `source scripts/env.sh` 刷一次。

该文件可以被 source（只设环境），也可以被执行（设完环境再 exec 传入的命令），所以一份定义同时服务两种入口。

### CycloneDDS
运行必须用 **CycloneDDS**（默认 FastRTPS 会刷 `std::bad_alloc`）。**Humble 自带的 CycloneDDS 0.10.x 就是 Unitree 要求的版本**，不再需要官方那套从源码编译的 `~/cyclonedds_ws`。

唯一要配的是网卡绑定：不指定时 Cyclone 会挑到 `wlan0`，表现为收不到 `/lowstate`。配置直接内联在 `CYCLONEDDS_URI` 里（Cyclone 接受 XML 文本，不只是 `file://`），省掉一个配置文件；默认 `eth0`，换网卡只需设环境变量：
```bash
G1_DDS_INTERFACE=enp1s0 source scripts/env.sh    # 网卡名用 ip -br addr 确认
```

验证连通性：
```bash
# 容器内，source scripts/env.sh 之后
ros2 topic hz /lowstate        # 应约 1 kHz
ros2 topic echo /secondary_imu --once --field rpy
```

### 其他
`git clone --recurse-submodules` 拉取仓库（已克隆则 `git submodule update --init --recursive`）。CANalyst-II 需 udev 权限（VID:PID `04d8:0053`），见 `src/canalystii_native_bridge/README.md`；容器是 `--privileged` 且 `-v /dev:/dev`，规则写在宿主上即可。

若要在仓库外独立使用 SDK，可选择安装：
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
终端 A 启动全部硬件、唯一 controller manager 和 motion controller：
```bash
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual
```
已经包含 `unitree_g1_ros2_control/control.launch.py`。不要再单独启动第二个 `control.launch.py`，也不要单独重复启动 Gloria-M、KWR57 或 CAN bridge。

终端 B 按需启动 Dashboard，只连接已有 `/controller_manager`：
```bash
ros2 launch robot_bringup whole_body_dashboard.launch.py
# http://<机器人 IP>:8200
```

终端 C 可启动 IKT Pose Commander Dashboard：
```bash
ros2 launch robot_bringup ikt_pose_commander.launch.py
# http://<机器人 IP>:8180
```

该入口默认控制 `right_gripper_base`、以 `torso_link` 为参考帧并保持 disabled。**Track robot** 使用 FPC；**Snap robot** 与 `return_to_start` 使用 JTC。Commander 通过 `/controller_manager/switch_controller` 一停一启，两个 controller 对相同资源的 claim 提供真实互斥。只启动 Commander、不启动 8180 页面时传入 `enable_dashboard:=false`。

### 其他启动方式
双臂重力参数初始化与手拉采点页面（默认不允许输出 `/lowcmd`）：
```bash
ros2 launch arm_gravity_compensation gravity_calibration.launch.py
# http://<机器人 IP>:8310
```

机械臂已可靠支撑、FPC/JTC 均 inactive 且无其他 `/lowcmd` 源时，才显式开放自动纯扭矩标定：
```bash
ros2 launch arm_gravity_compensation gravity_calibration.launch.py \
    allow_torque_output:=true
```

完整流程、参数可辨识边界和输出文件见 [arm_gravity_compensation/README.md](src/arm_gravity_compensation/README.md)。

只启动末端数据（CAN、双 KWR57、双 Gloria-M、左右相机）：
```bash
ros2 launch robot_bringup all_data.launch.py scope:=end_effectors topology:=dual
```

头部雷达与相机（不在 `all_data` 里，需要时单独起）：
```bash
ros2 launch head_sensors head_sensors.launch.py                # 雷达 + 相机
ros2 launch head_sensors head_sensors.launch.py camera:=false  # 只要雷达
ros2 launch head_sensors head_camera.launch.py                 # 只要相机
```

点云和图像要挂到机器人模型上需要整机 TF，先起 `all_data.launch.py scope:=whole_body`（提供 `robot_state_publisher`）。头部挂载角的校准步骤见 [head_sensors/README.md](src/head_sensors/README.md)。

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

相机链保持原设计，左右 `camera_node` 随末端拓扑启动并继续提供 ROS Image 与 8010/8011 内置页面。

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

整机 Dashboard 只发现 controller、执行 Engage/Disengage，并按类型向 FPC 的 `/forward_position_controller/commands` 或 JTC 的轨迹接口发送目标，不创建 manager 或控制适配器。G1 wrapper 只把切换超时放宽到 30 秒并做切换后状态校验（硬件接管会阻塞做夹爪使能和运控释放）。

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
| `robot_bringup/ikt_pose_commander.launch.py` | G1 适配 Commander、可选 8180 Dashboard | FPC 连续跟踪、JTC Snap/return-to-start |
| `unitree_g1_ros2_control/control.launch.py` | 唯一 manager、硬件插件、RSP、broadcaster、inactive FPC/JTC | 独立整机控制入口 |
| `unitree_g1_description/description.launch.py` | 仅模型、RSP 和 TF | 已有 `/joint_states` 时查看模型 |
| `head_sensors/head_sensors.launch.py` | 头部雷达节点 + RealSense，可用 `lidar` / `camera` 单开 | 头部传感器，不在 `all_data` 里 |
| `head_sensors/head_camera.launch.py` | 仅 RealSense + `d435_link → camera_link` 挂载 TF | 只要头部相机 |

单设备调试入口仍由 `kwr57_ros`、`gloria_ros`、`can_bridge_ros` 和 `camera_node` 各包提供；相机相关 launch 未改变。

### 运行约束
- 同一时刻只能有一个 `can_bridge_ros` 进程独占同一台 CANalyst-II；不要同时运行 `all_data`、单设备 `*_debug.launch.py` 或独立 bridge。
- D435i 同样只能被一个进程打开。起第二份时的报错完全不指向根因（`xioctl(UVCIOC_CTRL_QUERY) failed`、`Cannot open '/dev/video6'`），还会连累第一份；`head_camera.launch.py` 启动前会检查并直接拒绝。
- 同一 ROS graph 只能启动一个目标路径相同的 `controller_manager`。Dashboard 不负责启动或代理 manager。
- 单总线下非共享活动 CAN ID 必须互不冲突；双总线下不同物理通道可以复用 CAN ID。
- 双总线 Gloria-M 明确发布 `left_eccentric_joint`、`right_eccentric_joint`；硬件插件仍校验消息位置为有限值。
- 启动反馈到达前，broadcaster 使用有限的零位关节状态和单位 IMU 四元数，避免污染 TF；这些中性值不会绕过 `received` 与 freshness 安全门，controller 仍无法 Engage。
- 默认只允许 `LowState.mode_pr == 0`。仓库没有可信的 A/B 到 Pitch/Roll 逆解，不能把 AB 电机角直接作为 URDF 脚踝角。
- G1 的 29 组 `kp/kd` 位于 `unitree_g1_ros2_control/config/default_29dof_param.yaml`。Gloria-M 默认 `kp=10`、`kd=5`，其中 `kd=5` 是协议编码上限。
- `enable_grippers_on_start:=true` 只配置并使能设备；controller 激活仍会重新执行完整安全事务。需要上电保持失能时传入 `false`。
- BEST_EFFORT 高频话题可使用 `ros2 topic echo --qos-reliability best_effort`；KWR57 的 1 kHz 订阅建议使用 `rclcpp`、BEST_EFFORT 和 `KEEP_LAST(64)`。

各包细节见 [robot_bringup/README.md](src/robot_bringup/README.md)、[unitree_g1_ros2_control/README.md](src/unitree_g1_ros2_control/README.md)、[unitree_g1_description/README.md](src/unitree_g1_description/README.md)、[kwr57_ros/README.md](src/kwr57_ros/README.md) 和 [camera_node/README.zh.md](src/camera_node/README.zh.md)。
