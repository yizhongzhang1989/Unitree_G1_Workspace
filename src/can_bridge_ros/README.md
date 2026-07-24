# can_bridge_ros

本包让一个 ROS 2 节点独占 USB-CAN 适配器，并在物理 CAN 帧与 `can_msgs/Frame` 之间转发；多个设备驱动因此可以共享同一个 CANalyst-II，而不会重复打开硬件。

> 简单理解：设备节点发布 TX Frame，`can_bridge_ros` 把它发到 CAN；适配器收到 CAN 帧后，本包再按通道、CAN ID 路由给对应设备节点。高频设备也可用进程内 handler 跳过中间 ROS Frame。

本包只负责总线所有权、帧收发和路由，不解释 KWR57、Gloria-M 等设备协议。无 ROS 的总线创建、CANalyst-II `libusb` 准备及权限检查由 [`CAN-SDK`](../../sdk/CAN-SDK/README.md) 提供。

`CAN-SDK` 是位于根目录 `sdk/` 的纯 Python 包，不在 colcon 默认扫描的 `src/` 下。运行本节点前应 source 工作区的 `scripts/env.sh`，由它通过 `PYTHONPATH` 暴露 SDK 源码；无需安装本地 SDK。

## `can_msgs` 来源

`from can_msgs.msg import Frame` 来自上游 ROS 2 **`can_msgs`** 消息包，不是 `python-can`、`can_sdk` 或本仓库定义的类型。源码位于 [`ros-industrial/ros_canopen`](https://github.com/ros-industrial/ros_canopen/tree/dashing-devel/can_msgs)。

ROS2 Foxy 安装命令：
```bash
sudo apt-get install -y ros-foxy-can-msgs
```

依赖同时在 `package.xml` 中声明；`python-can`/`can_sdk` 负责物理总线，`can_msgs/Frame` 只负责 ROS 节点间传输。

```text
第1层 can_sdk        : python-can 后端与基础 I/O（无 ROS、无设备协议）
第2层 can_bridge_ros : 独占物理总线，按可选 ID 路由 RX，并提供受约束的进程内 handler
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
| `frame_handler_specs` | `[""]` | 受信任的进程内 handler JSON 字符串数组；默认禁用 |
| `rx_processing_queue_depth` | `2048` | 每通道处理缓冲深度；满时丢弃最旧帧 |
| `rx_processing_batch_size` | `128` | 每个处理线程单次取出的最大帧数 |
| `tx_batch_size` | `64` | USB I/O 每轮最多发送的帧数，避免 RX 饥饿 |

`rx_routes` 在启动时解析，运行期间不动态修改。命中时采用转发而非镜像：帧只发布到专属话题；未命中时发布到 `/<bus_name>/rx`。例如：
```yaml
rx_routes:
  - "0:0x101:/can0/grip_left/rx"
```

路由中的通道必须出现在 `channel_ids` 中。同一通道的同一 CAN ID 可以配置多个不同目标话题，bridge 会按顺序扇出；完全相同的重复规则会被拒绝。例如 Gloria-M 的共享反馈 ID `0x000` 可以同时路由到两台夹爪：
```yaml
rx_routes:
  - "0:0x0:/can0/grip_left/rx"
  - "0:0x0:/can0/grip_right/rx"
```

`config/single_bus.yaml` 和 `config/dual_bus.yaml` 只描述物理适配器。生产 `robot_bringup` 为 KWR57 生成 handler specs，为 Gloria-M 生成 `rx_routes`；KWR57 有效帧在进程内消费，不创建中间 ROS Frame。

## 进程内帧 handler

`frame_handler_specs` 用于高频设备的可选快路径。每个元素是启动时解析的 JSON 对象：

```yaml
frame_handler_specs:
  - >-
    {"factory":"kwr57_ros.bridge_handler:create_frame_handler",
     "config":{"channel_id":0,"data_base_id":21}}
```

`factory` 必须使用 `module:function` 格式。bridge 只负责动态导入并调用该工厂，不导入任何具体设备协议；`config` 的字段和含义由设备包验证。该参数能够导入 Python 代码，因此只能来自受信任的 launch/config，不能直接接受远程或终端用户输入。

工厂接收通用 `FrameHandlerContext`，其中包含 logger、直接进入 bridge TX 队列的 `send_frame(channel_id, can_id, data)` 和当前 ROS context；返回 `FrameHandlerRegistration`，声明：

- handler 名称和唯一的 `(channel_id, can_id)` 集合；
- 每帧回调，以及 `FORWARD` 或 `CONSUME` 结果；
- 可选的辅助 ROS 节点及 `start`/`stop` 生命周期。

bridge 在启动时检查空注册、非法/重复 CAN ID 和多个 handler 抢占同一 key。运行时使用字典按 `(channel_id, can_id)` 做 O(1) 查找。`FORWARD` 继续原有 `rx_routes`/默认 RX 发布，`CONSUME` 跳过该帧的 ROS `can_msgs/Frame` 发布。回调连续失败 3 次会被禁用，后续帧恢复原话题路由；正常关闭时先停止分发和设备流，再排空 TX 队列并释放总线。

bridge 使用一个 USB I/O 线程持续收发 CAN，并为每个物理通道创建一个处理线程。USB I/O 只执行有限批量 TX、`recv()` 和入队；handler、协议组包及普通 ROS RX 路由在对应通道的处理线程中执行。因此一个通道的 Python 回调不会阻塞另一个通道的 USB 接收轮询。相同 handler 注册实例由独立锁保持串行语义，不同 handler 可以并行。

每通道处理缓冲为有界 oldest-drop 队列；过载时保留最新帧，不会让延迟无限增长。handler 回调仍必须同步、非阻塞，不能执行文件/网络 I/O、`sleep` 或等待其他线程。bridge 不为每帧创建超时线程；运行期隔离依靠独立通道线程、严格工厂注册和异常熔断。

Web demo 为兼容方案 A 保留 KWR57 专属路由；生产 bringup 不创建 KWR57 专属路由。handler 拒绝帧或被熔断后，帧会落到默认 `/canX/rx`；bridge 不会动态创建独立设备进程。通用 bridge launch 默认不加载 handler，生产 `robot_bringup` 始终为所有 KWR57 生成 handler specs。

## 启动

```bash
source scripts/env.sh
ros2 launch can_bridge_ros can_bridge_ros.launch.py config:=single_bus.yaml
ros2 launch can_bridge_ros can_bridge_ros.launch.py config:=dual_bus.yaml
```

单独运行 bridge 时没有专属路由，所有普通接收帧都发布到默认 `/canX/rx`。生产部署使用 `robot_bringup`，不要在物理 YAML 中写死设备名称和 CAN ID。

CANalyst-II 是一个 USB 设备。双通道必须用同一进程通过 `channel="0,1"` 打开，两个独立 Bus 可能产生 `Resource busy`。

## CANalyst-II Linux 权限
```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="04d8", ATTR{idProduct}=="0053", MODE="0666", GROUP="plugdev"' \
  | sudo tee /etc/udev/rules.d/99-canalystii.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 设备节点约定
- 高频设备优先使用进程内 handler；低频设备可订阅 `rx_routes` 分配的专属话题。
- 发布命令到 `/<bus_name>/tx`（RELIABLE）。
- `can_msgs/Frame.data` 是定长 8 字节整数列表，`dlc` 表示有效长度。
- ROS 设备节点不直接打开物理 CAN；需要台架直连时，只能停止 bridge 后使用独立 SDK 工具。
