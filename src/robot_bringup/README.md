# robot_bringup

`robot_bringup` 是末端设备系统的部署编排包。它不实现 CAN 协议或设备驱动，而是把
`can_bridge_ros`、`kwr57_ros` 和 `gloria_ros` 按实际接线组合成一套可启动系统。

## 作用

- 声明 CAN 通道与 ROS 总线名的对应关系。
- 用一份设备清单声明每台 KWR57 和 Gloria-M 的总线、CAN ID、专属 RX 话题及节点参数。
- 根据设备清单生成 `can_bridge_ros` 的启动参数 `rx_routes`。
- 使用同一批设备对象创建 KWR57 和 Gloria-M 节点，避免 bridge 路由与节点参数分别维护。
- 在启动前检查重复节点名、重复专属话题，以及同一通道上的关键 CAN ID 冲突。

物理适配器参数仍由 `can_bridge_ros/config/single_bus.yaml` 和 `dual_bus.yaml` 提供。这两份
YAML 只保存适配器、通道、波特率和队列深度，不包含机器人设备名称或 CAN ID。

## 路由模型

bringup 在 launch 文件执行时构造：

- `CanBus`：ROS 总线名和 python-can 通道编号；
- `Kwr57Device`：命令 ID、连续三个数据 ID、专属 RX 话题和 wrench 输出；
- `GloriaDevice`：命令 ID、反馈 ID、专属 RX 话题和夹爪节点参数。

`build_bridge_parameters()` 将清单转换成 bridge 参数：

```text
channel_id:can_id:dedicated_rx_topic
```

这些参数在 `can_bridge_ros` 节点启动时解析一次，系统运行期间保持不变。这里的“运行时
路由支持”是指 bridge 在接收循环中按照启动时建立的路由表转发帧，并不是节点运行后动态
增删路由。

命中路由的普通 CAN 数据帧只发布到专属话题，不再进入默认 `/canX/rx`；未命中的帧仍发布
到默认话题。相同的 `channel + CAN ID` 可以配置多个不同目标，用于协议要求的扇出。
Gloria-M 兼容 CAN ID `0x000` 返回状态，因此同总线多个夹爪都会接收这一个共享 ID，再按
payload `Data[0]` 的低 4 位设备号过滤。寄存器回包不携带可用于共享 ID 分流的设备号，
因此生产 bringup 要求每台夹爪使用非零专属 `feedback_id`；CAN ID `0x000` 只作为状态
反馈兼容路径。`command_id` 的低 4 位也不能为 `0`，该值为共享反馈协议保留。其他专属
ID 不会在夹爪之间广播。

## 性能

没有专属路由时，每个设备节点都会接收并过滤整条总线的所有 ROS CAN 消息。两个 1 kHz
KWR57 会产生约 6000 frame/s，这会让每个 Python 节点承担大量无关 DDS 回调和数据复制。

启用 bringup 路由后：

- 每台 KWR57 只接收自己的三个数据 ID；
- 每台 Gloria-M 只接收自己的反馈 ID、兼容命令 ID 和共享 `0x000`；
- 未归属设备的帧保留在默认 `/canX/rx`；
- bridge 热路径仍是一次字典查询，仅在协议确实需要扇出时多做目标发布。

因此专属路由通常会明显降低 Python/DDS 负载，尤其能防止 KWR57 高频数据持续唤醒夹爪
节点。路由来源是 YAML、launch 还是其他启动参数不会改变查询性能。

## 当前拓扑

### 单总线

`single_bus.launch.py` 将四个设备放在 CANalyst-II 通道 0：

| 设备 | 命令 ID | 接收/数据 ID | 专属 RX |
|---|---:|---|---|
| `ft_left` | `0x10` | `0x15/0x16/0x17` | `/can0/ft_left/rx` |
| `ft_right` | `0x11` | `0x18/0x19/0x1A` | `/can0/ft_right/rx` |
| `grip_left` | `0x01` | `0x101/0x01/0x000` | `/can0/grip_left/rx` |
| `grip_right` | `0x02` | `0x102/0x02/0x000` | `/can0/grip_right/rx` |

同一总线上的设备活动 ID 必须与实物配置一致且不能产生非预期冲突。
Gloria-M 的 `command_id & 0x0F` 必须非零且唯一，固定请求 ID `0x7FF` 不能被任何同通道
设备占用。

### 双总线

`dual_bus.launch.py` 每条总线放一台 KWR57 和一台 Gloria-M：

| 总线 | 设备 | 命令 ID | 接收/数据 ID | 专属 RX |
|---|---|---:|---|---|
| `can0` / 通道 0 | `ft_arm0` | `0x10` | `0x15/0x16/0x17` | `/can0/ft_arm0/rx` |
| `can0` / 通道 0 | `grip_arm0` | `0x01` | `0x101/0x01/0x000` | `/can0/grip_arm0/rx` |
| `can1` / 通道 1 | `ft_arm1` | `0x10` | `0x15/0x16/0x17` | `/can1/ft_arm1/rx` |
| `can1` / 通道 1 | `grip_arm1` | `0x01` | `0x101/0x01/0x000` | `/can1/grip_arm1/rx` |

不同物理 CAN 通道可以复用相同 CAN ID。

## 启动

```bash
source scripts/env.sh

ros2 launch robot_bringup single_bus.launch.py
ros2 launch robot_bringup dual_bus.launch.py
```

也可以使用工作区脚本：

```bash
bash scripts/run.sh single
bash scripts/run.sh dual
```

## 修改部署

1. 在对应 launch 文件中修改或新增 `CanBus`、`Kwr57Device`、`GloriaDevice`。
2. 确保清单中的 CAN ID 与实物一致；Gloria-M 使用非零 `feedback_id`，同通道低 4 位
	设备号唯一，并保留固定请求 ID `0x7FF`。
3. 不要把设备路由写入 `can_bridge_ros/config/*.yaml`。
4. 重新启动 bringup；不支持系统运行中动态切换路由。

因为 bridge 参数和设备节点参数来自同一对象，修改 `data_base_id`、`feedback_id` 或
`rx_topic` 后无需在其他文件重复维护映射。

## 文件

- `launch/single_bus.launch.py`：单总线设备清单。
- `launch/dual_bus.launch.py`：双总线设备清单。
- `robot_bringup/topology.py`：设备模型、路由生成和冲突检查。
- `robot_bringup/nodes.py`：把设备模型转换为 ROS 2 launch Node。
- `test/test_topology.py`：无 ROS 依赖的拓扑单元测试。
