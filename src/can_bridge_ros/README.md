# can_bridge_ros

通用 ROS 2 CAN 总线桥接：一个节点独占一个物理 USB-CAN 设备，将收到的帧发布为 `can_msgs/msg/Frame`，并订阅命令帧下发。支持 CANalyst-II 单设备多通道，以及按 `channel + CAN ID` 将高频帧分流到设备专属 RX 话题。

本包只负责 ROS 参数、消息转换、线程调度和话题分发；无 ROS 的总线创建、CANalyst-II `libusb` 准备及权限检查统一由 [`CAN-SDK`](../../sdk/CAN-SDK/README.md) 提供。

`CAN-SDK` 是位于根目录 `sdk/` 的纯 Python 包，不在 colcon 默认扫描的 `src/` 下。运行本节点前应 source 工作区的 `scripts/env.sh`，由它通过 `PYTHONPATH` 暴露 SDK 源码；无需安装本地 SDK。

## `can_msgs` 来源

`from can_msgs.msg import Frame` 来自上游 ROS 2 **`can_msgs`** 消息包，不是 `python-can`、`can_sdk` 或本仓库定义的类型。该包的源码位于 [`ros-industrial/ros_canopen`](https://github.com/ros-industrial/ros_canopen/tree/dashing-devel/can_msgs)，
用于定义 CAN 相关 ROS 消息；`Frame` 是由 ROS 接口生成器生成的 Python 消息类。

ROS2 Foxy 安装命令：
```bash
sudo apt-get install -y ros-foxy-can-msgs
```

依赖同时在本包的 `package.xml` 中声明为 `<exec_depend>can_msgs</exec_depend>`。
其中 `python-can`/`can_sdk` 负责访问物理 CAN 总线，`can_msgs/Frame` 只负责 ROS 节点间传输一帧 CAN 数据。

```text
第1层 can_sdk        : python-can 后端与基础 I/O（无 ROS、无设备协议）
第2层 can_bridge_ros : 独占物理总线，按可选 ID 路由发布 RX，订阅 /canX/tx
第3层设备节点        : 订阅默认 /canX/rx 或自己的专属 RX 话题
```

## 话题与参数

| 方向 | 话题 | 类型 | QoS |
|---|---|---|---|
| 发布 | `/<bus_name>/rx` 或路由目标 | `can_msgs/Frame` | BEST_EFFORT, KEEP_LAST(depth) |
| 订阅 | `/<bus_name>/tx` | `can_msgs/Frame` | RELIABLE, KEEP_LAST |

| 参数 | 默认 | 说明 |
|---|---|---|
| `interface` | `canalystii` | python-can 后端 |
| `channel` | `"0"` | 单通道；多通道如 `"0,1"` |
| `bitrate` | `1000000` | 比特率 |
| `channel_ids` | `[0]` | `Message.channel` 值 |
| `bus_names` | `["can0"]` | 对应 ROS 总线名 |
| `rx_queue_depth` | `128` | 接收发布队列深度；约 40 ms 的单 KWR57 帧窗口 |
| `receive_own_messages` | `false` | 是否回显发送帧 |
| `rx_routes` | `[""]` | `channel:can_id:topic` 字符串数组；支持十进制或 `0x` ID |

`rx_routes` 是 bridge 的**启动参数**。节点启动时解析并建立路由表，运行期间不动态修改。
命中时采用**转发而非镜像**：该帧只发布到专属话题，不再发布到 `/<bus_name>/rx`。未命中的帧仍走默认 RX，因此 KWR57 高频流可以和 Gloria-M 等低频设备隔离。例如：
```yaml
rx_routes:
  - "0:0x15:/can0/ft_left/rx"
  - "0:0x16:/can0/ft_left/rx"
  - "0:0x17:/can0/ft_left/rx"
```

路由中的通道必须出现在 `channel_ids` 中。同一通道的同一 CAN ID 可以配置多个**不同**
目标话题，bridge 会按顺序扇出；完全相同的重复规则会被拒绝。例如 Gloria-M 的共享
反馈 ID `0x000` 可以同时路由到两台夹爪：
```yaml
rx_routes:
  - "0:0x0:/can0/grip_left/rx"
  - "0:0x0:/can0/grip_right/rx"
```

仓库自带的 `config/single_bus.yaml` 和 `config/dual_bus.yaml` 只描述适配器、通道、波特率和队列等**物理总线默认值**，不包含具体设备路由。完整机器人启动时，`robot_bringup` 根据当次 launch 中的 `Kwr57Device` 和 `GloriaDevice` 清单生成 `rx_routes`，并以 ROS 参数覆盖传给 bridge。参数只在启动时解析一次，运行中的路由仍是字典查询；仅当一个 ID 配置多个目标时，才会产生协议所需的额外发布。

## 启动

```bash
source ~/end_effector_ros/scripts/env.sh
ros2 launch can_bridge_ros can_bridge_ros.launch.py config:=single_bus.yaml
ros2 launch can_bridge_ros can_bridge_ros.launch.py config:=dual_bus.yaml
```

单独运行上述 bridge 时没有专属路由，所有普通接收帧都发布到默认 `/canX/rx`；这是通用
调试模式。生产部署使用 `robot_bringup`，不要在物理 YAML 中写死设备名称和 CAN ID。

CANalyst-II 是一个 USB 设备。双通道必须用同一进程通过 `channel="0,1"` 打开，两个独立 Bus 可能产生 `Resource busy`。

## CANalyst-II Linux 权限
```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="04d8", ATTR{idProduct}=="0053", MODE="0666", GROUP="plugdev"' \
  | sudo tee /etc/udev/rules.d/99-canalystii.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 设备节点约定
- 高频设备优先订阅 bringup 通过 `rx_routes` 分配的专属话题；其他设备订阅 `/<bus_name>/rx`。
- 发布命令到 `/<bus_name>/tx`（RELIABLE）。
- `can_msgs/Frame.data` 是定长 8 字节整数列表，`dlc` 表示有效长度。
- ROS 设备节点不直接打开物理 CAN；需要台架直连时，只能停止 bridge 后使用独立 SDK 工具。
