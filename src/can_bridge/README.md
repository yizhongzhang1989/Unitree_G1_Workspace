# can_bridge

通用 CAN 总线桥接（第2层 "Bus Manager"）：一个节点独占**一个物理 USB-CAN 设备**
（基于 [python-can]，支持 CANalyst-II / SocketCAN / slcan 等），把收到的**所有帧**发布为
`can_msgs/msg/Frame`，并订阅命令帧下发。**与设备无关**，供任意 CAN 设备的 ROS 节点复用。

支持**多通道**：一个进程可同时桥接一个设备的多条 CAN 通道（如 CANalyst-II 的 CAN1/CAN2
= 通道 0/1），按 `msg.channel` 路由到 `/can0`、`/can1`。

```
第1层 CAN Driver : python-can 后端（canbus_backend.py，含 CANalyst-II libusb 处理）
第2层 can_bridge : 本包，独占设备；RX 线程发布所有帧、订阅 tx 帧下发   ← Bus Manager
第3层 设备节点   : 纯 ROS，订阅 /canX/rx 过滤自己的 ID，发命令到 /canX/tx
```

消息用标准 `can_msgs/msg/Frame`（与 [ros2_socketcan] 一致）：换 SocketCAN 硬件可直接换官方桥。

---

## 话题 / 参数

| 方向 | 话题 | 类型 | QoS |
|---|---|---|---|
| 发布 | `/<bus_name>/rx` | `can_msgs/Frame` | BEST_EFFORT, KEEP_LAST(depth) |
| 订阅 | `/<bus_name>/tx` | `can_msgs/Frame` | RELIABLE, KEEP_LAST |

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `interface` | string | `canalystii` | python-can 后端 |
| `channel` | string | `0` | python-can 通道；多通道用 `"0,1"` |
| `bitrate` | int | `1000000` | 比特率 |
| `channel_ids` | int[] | `[0]` | `msg.channel` 值（与 bus_names 平行）|
| `bus_names` | string[] | `["can0"]` | 每个通道对应的 ROS 总线名 |
| `rx_queue_depth` | int | `1000` | rx 发布队列深度 |
| `receive_own_messages` | bool | `false` | 是否回显自己发送的帧 |

配置文件：`config/single_bus.yaml`（单通道 → /can0）、`config/dual_bus.yaml`（双通道 → /can0、/can1）。

```bash
ros2 launch can_bridge can_bridge.launch.py config:=single_bus.yaml
ros2 launch can_bridge can_bridge.launch.py config:=dual_bus.yaml
```

> **为什么必须一个进程开多通道**：CANalyst-II 是**一个 USB 设备**，每个 python-can Bus
> 各建一个 CanalystDevice 独占该 USB 设备，两个 Bus 会 `Resource busy`；一个 Bus 用
> `channel="0,1"` 只建一个 device、初始化两个通道，收发用 `Message.channel` 区分。

---

## CANalyst-II udev 权限（一次性）

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="04d8", ATTR{idProduct}=="0053", MODE="0666", GROUP="plugdev"' \
  | sudo tee /etc/udev/rules.d/99-canalystii.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 快速验证

```bash
# 看到原始帧（BEST_EFFORT 需加 --qos-reliability）
ros2 topic echo --qos-reliability best_effort /can0/rx
```

## 给设备节点作者

- 订阅 `/<bus_name>/rx`（BEST_EFFORT），按 `frame.id` 过滤自己的 CAN ID。
- 下发命令：发布 `can_msgs/Frame` 到 `/<bus_name>/tx`（RELIABLE），`data` 为 8 字节定长
  **list of int**（不是 bytes），`dlc` 为有效长度。
- 一设备一节点；多设备就多起几个节点，各自过滤各自 ID。

[python-can]: https://python-can.readthedocs.io
[ros2_socketcan]: https://github.com/autowarefoundation/ros2_socketcan
