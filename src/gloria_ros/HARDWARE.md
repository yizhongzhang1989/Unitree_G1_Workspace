# Gloria-M 硬件与协议技术文档
本文面向在本工作区中接入 Synria Robotics 云犀 Gloria-M 平行二指夹爪的开发者，总结硬件接口、CAN 协议、SDK 参数与 `gloria_ros` 的实际行为。本文依据以下资料整理：

- [Gloria-M 官方产品介绍](https://docs.sparklingrobo.com/docs/gloria-m-series)
- [官方安装与使用指南](https://docs.sparklingrobo.com/docs/gloria-m-series/doc_01_installation_guide)
- [官方通信协议](https://docs.sparklingrobo.com/docs/gloria-m-series/protocol/doc_00_intro)
- [官方 Python SDK 文档](https://docs.sparklingrobo.com/docs/gloria-m-series/doc_02_sdk_communication)
- [`Gloria-M-SDK`](../../sdk/Gloria-M-SDK/README.zh-CN.md) 源码
- 本仓库 [`gloria_ros`](README.md) 与 [`can_bridge_ros`](../can_bridge_ros/README.md)

整理日期：2026-07-11

> **资料边界**：Gloria-M 硬件使用 24 V DC 供电（问淘宝客服得到的；已经实测检验过）。官网公开页面尚未给出峰值电流、重量、连接器针脚定义、额定夹持力或完整安装孔位表。接线和上电前仍须核对设备铭牌、随货资料或厂家确认值，不能从 SDK 默认参数或线缆颜色反推其余电气规格。

## 1. 机械尺寸

官网公开的 100 mm 夹爪尺寸图：

![Gloria-M 100 mm 夹爪尺寸图](https://download.sparklingrobo.com/avatar/hZzTGOjQUWnw6OZoyqf8EgJxvM0ufgimage/png)

*图 1：Gloria-M 100 mm 夹爪尺寸，来源：[官方产品介绍](https://docs.sparklingrobo.com/docs/gloria-m-series)。*

图中标注如下，单位均为 mm。尺寸是图示值，不代表公差、安装孔位或碰撞包络已经
完整定义。

| 项目 | 图示值 | 说明 |
|---|---:|---|
| 指间内开口 | 100 | 图示张开状态两指内侧间距 |
| 图示总宽 | 157 | 图示张开状态外廓宽度 |
| 图示总高 | 130 | 从指端至上部电机/驱动组件 |
| 指端宽度 | 43 | 单个指端沿图示纵向的宽度 |
| 中心圆弧 | R28.5 | 中央机构标注半径 |

## 2. 电气接口与通信路径

夹爪由独立的 24 V DC 电源供电。官方接线图标出了电源、CAN 与串口调试接口：

![Gloria-M 接口与接线图](https://download.sparklingrobo.com/avatar/vqtcbKodHmtgUWnw6OZoyqf8EgJxvM0ufgimage/png)

*图 2：Gloria-M 接口与官方串口调试链路，来源：[官方安装与使用指南](https://docs.sparklingrobo.com/docs/gloria-m-series/doc_01_installation_guide)。*

| 接口 | 标注用途 | 本工作区中的用途 |
|---|---|---|
| XT30 电源接口 | 24 V DC 工作供电 | 独立给夹爪供电；接线前确认极性和电源容量 |
| XT30(2+2) 接口 | CAN 通信 | 连接 CANalyst-II 所在 CAN 总线 |
| GH1.25 3PIN | 串口通信 | 可配合官方串口调试链路；ROS 运行时不使用 |
| Type-C/调试板 | 上位机串口转 CAN 链路 | `Gloria-M-SDK` 的 `SerialCanAdapter` 使用该类链路 |

本仓库支持两条互斥的软件访问路径：

```text
官方 SDK 直连：上位机 -> Type-C 串口转 CAN 适配器 -> Gloria-M
本仓库 ROS：  ROS 设备节点 -> /canX/tx,/canX/rx -> can_bridge_ros
              -> CANalyst-II -> XT30(2+2) CAN -> Gloria-M
```

`gloria_ros` 只复用 SDK 的 `protocol_mit.py` 和 `types.py`，不会打开 SDK 的串口。
物理 CAN 设备由一个 `can_bridge_ros` 进程独占。运行 ROS 时不要再启动会直接打开同一适配器的 SDK 程序。

### 2.1 两种“波特率”不能混淆

| 参数 | 当前值 | 来源与含义 |
|---|---:|---|
| 串口波特率 | 921600 bit/s | SDK `SerialCanAdapter` 的 USB 串口速率 |
| CAN 总线比特率 | 1000000 bit/s | 本仓库 `single_bus.yaml`/`dual_bus.yaml` 配置值 |

`921600` 不是 CAN 比特率。`1000000` 是本项目配置，不应被当作所有 Gloria-M 固件的无条件出厂值。夹爪寄存器中存在 `can_br`（RID 35），但 SDK 未给出枚举值与实际 bit/s 的换算表。接入前应使用厂家调试工具确认设备 CAN 速率与 bridge 一致。

### 2.2 总线接线原则
- 断电完成电源和 CAN 接线，确认电源极性后再上电。
- 不要按官方示意图中的线色猜测 XT30(2+2) 针脚，必须查实物标识或厂家针脚表。
- CAN_H、CAN_L 和参考地应正确连接，双绞线尽量短并远离电机动力线。
- 按 CAN 总线规范，仅在物理总线两端配置终端电阻，避免星形长支线。
- 同一总线的所有节点必须使用相同比特率，且活动 CAN ID 不能产生非预期冲突。
- 本项目默认使用标准 11-bit CAN 数据帧，不接受扩展帧、RTR 帧或错误帧作为设备反馈。

## 3. ROS 硬件架构
```mermaid
flowchart LR
    App[ROS 控制应用] -->|命令话题/服务| Gripper[gloria_ros]
    Gripper -->|can_msgs/Frame| Bridge[can_bridge_ros]
    Bridge --> CanSdk[CAN-SDK]
    CanSdk --> UsbCan[CANalyst-II]
    UsbCan -->|1 Mbit/s CAN| Gloria[Gloria-M]
    Gloria --> UsbCan
    Gripper -->|JointState 与 diagnostics| App
```

职责边界：

| 层 | 组件 | 职责 |
|---|---|---|
| 设备协议 | `Gloria-M-SDK` | MIT 打包/反馈解包、量程类型和寄存器定义 |
| ROS 设备节点 | `gloria_ros` | 模式确认、命令校验、安全限位、反馈健康度和 ROS 接口 |
| ROS 总线桥 | `can_bridge_ros` | 独占 USB-CAN、收发 `can_msgs/Frame`、按 ID 路由 |
| 物理 I/O | `CAN-SDK`/CANalyst-II | CAN 通道初始化和帧收发 |

## 4. CAN 标识符规划
协议使用标准 11-bit CAN ID。以下默认值来自当前 SDK/ROS 配置，不一定等于每台实物的出厂设置。

| 用途 | 计算/默认值 | `command_id=0x01` 示例 |
|---|---:|---:|
| MIT、使能、失能、设零命令 | `command_id` | `0x001` |
| PV 命令 | `0x100 + command_id` | `0x101` |
| 参数读写、保存、状态请求 | 固定广播 ID | `0x7FF` |
| 状态/参数反馈 | 设备 `Master ID`，ROS 参数为 `feedback_id` | 默认配置 `0x101` |

官网协议说明反馈帧 ID 由 `Master ID` 设置且默认值为 0；当前 SDK 和 ROS 则默认使用 `feedback_id=0x101`。因此 `feedback_id` 必须按实机读取值配置，不能只依赖默认值。
当前 ROS 节点兼容反馈 CAN ID 为 `feedback_id`、`command_id` 或 `0x000` 的固件。
多设备 `robot_bringup` 要求每台夹爪使用非零专属 `feedback_id`，因为寄存器回包没有
payload 设备号，无法在共享 CAN ID `0x000` 上安全区分来源。`command_id` 低 4 位设备号
必须非零且在同通道唯一；值 `0` 为共享状态反馈保留。
节点按 SDK 定义的完整回包特征识别寄存器帧：`Data[0:2]` 为保留零字节且
`Data[2]` 为 `0x33/0x55`。MIT 状态帧中的位置字节即使偶然等于该操作码，也不会被误判。

多夹爪共总线时，应检查每台设备的完整活动 ID 集合，而不只是 `command_id`。例如：

| 设备 | `command_id` | PV ID | `feedback_id` |
|---|---:|---:|---:|
| 左夹爪 | `0x01` | `0x101` | `0x101` |
| 右夹爪 | `0x02` | `0x102` | `0x102` |

不同物理总线上的设备可以复用同一组 ID。本仓库的单总线 bringup 使用两组 ID，双总线 bringup 则允许两臂都使用 `0x01/0x101`。

## 5. 控制协议
除特别说明外，控制与反馈均为 DLC 8 的标准 CAN 数据帧。

### 5.1 通用控制命令
通用命令发送到 `command_id`，数据前 7 字节均为 `0xFF`：

| 命令 | Data[7] | 上电/运行语义 | ROS 支持 |
|---|---:|---|---|
| 清除错误 | `0xFB` | 清除可恢复故障 | 未暴露服务 |
| 使能 | `0xFC` | 上电自检后进入主动控制 | `~/enable` |
| 失能 | `0xFD` | 停止主动驱动 | `~/disable` |
| 保存/设置位置零点 | `0xFE` | 将当前位置作为零点 | `~/set_zero`，默认禁用 |

上电默认处于失能状态。必须先确认控制模式和量程，再使能和发送运动命令。

SDK 还使用以下状态请求帧，发送到 `0x7FF`：

```text
Data = [command_id低字节, command_id高字节, 0xCC, 0, 0, 0, 0, 0]
```

### 5.2 MIT 阻抗/力矩模式
MIT 命令发送到 `command_id`，5 个变量压缩在 8 字节中：

| 字节 | 位布局 |
|---|---|
| 0..1 | `p_des`，16 bit |
| 2 | `v_des[11:4]` |
| 3 | `v_des[3:0] | kp[11:8]` |
| 4 | `kp[7:0]` |
| 5 | `kd[11:4]` |
| 6 | `kd[3:0] | t_ff[11:8]` |
| 7 | `t_ff[7:0]` |

固件目标力矩关系为：

$$
\tau_{ref} = k_p(p_{des}-p_{fb}) + k_d(v_{des}-v_{fb}) + \tau_{ff}
$$

协议范围：

| 量 | 范围 | 单位/说明 |
|---|---:|---|
| `p_des` | `[-PMAX, PMAX]` | rad，16 bit |
| `v_des` | `[-VMAX, VMAX]` | rad/s，12 bit |
| `kp` | `[0, 500]` | 12 bit |
| `kd` | `[0, 5]` | 12 bit |
| `t_ff` | `[-TMAX, TMAX]` | N m，12 bit |

典型组合：

- 位置阻抗：`kp > 0`、`kd > 0`，`tau = 0`。
- 速度控制：`kp = 0`、`kd > 0`，设置 `dq`。
- 力矩前馈：`kp = 0`，使用低幅值 `tau`；工程上仍建议保留适当 `kd` 抑制冲击。

官网明确警告：进行位置控制时 `kd` 不能为 0，否则可能震荡甚至失控。
### 5.3 PV 位置速度模式

PV 命令发送到 `0x100 + command_id`：

```text
Data[0:4] = p_des，IEEE-754 float32，小端序，单位 rad
Data[4:8] = v_des，IEEE-754 float32，小端序，单位 rad/s
```

`p_des` 是目标位置，`v_des` 是轨迹最大绝对速度。固件内部使用位置环、速度环和电流环。
当前 ROS 节点额外要求 `velocity` 位于 `[0, vmax]`，即速度字段作为非负速度上限使用。
官网建议阻尼因子设置为非零正数；出现震荡时应同时检查阻尼、加速度和减速度设置。

### 5.4 反馈帧
所有控制模式共用 MIT 风格状态反馈：

| 字节 | 字段 | 含义 |
|---|---|---|
| 0 | `ERR[3:0] | ID[3:0]` | 高 4 位状态码，低位设备 ID |
| 1..2 | `POS` | 16-bit 位置 |
| 3..4 高 4 位 | `VEL` | 12-bit 速度 |
| 4 低 4 位..5 | `T` | 12-bit 力矩 |
| 6 | `T_MOS` | 驱动 MOS 平均温度，摄氏度 |
| 7 | `T_Rotor` | 电机线圈平均温度，摄氏度 |

官网状态码：

| `ERR` | 状态 |
|---:|---|
| `0x0` | 失能 |
| `0x1` | 使能 |
| `0x8` | 超压 |
| `0x9` | 欠压 |
| `0xA` | 过电流 |
| `0xB` | MOS 过温 |
| `0xC` | 电机线圈过温 |
| `0xD` | 通信丢失 |
| `0xE` | 过载 |

**当前软件限制**：`gloria_ros` 已按 Data[0] 低 4 位识别设备，因此高 4 位的使能或故障
状态不会再导致整帧被丢弃；但 SDK 的 `unpack_mit_feedback()` 和 ROS 节点目前仍只输出位置、
速度和力矩，没有发布 `ERR`、`T_MOS` 或 `T_Rotor`。ROS `/diagnostics` 中的
`enabled_requested` 是主机侧请求状态，不是设备 `ERR=1` 的确认。生产系统不能仅凭当前
`/diagnostics` 判定硬件无故障，应增加状态码和温度解析或使用厂家工具监控。

### 5.5 定点映射与量化
SDK 对位置、速度和力矩使用相同的线性映射。对范围 `[x_min, x_max]` 和 `n` 位整数：

$$
u = \left\lfloor\frac{x-x_{min}}{x_{max}-x_{min}}(2^n-1)\right\rfloor
$$

$$
x = x_{min} + \frac{u}{2^n-1}(x_{max}-x_{min})
$$

SDK 默认 `PMAX=3.14`、`VMAX=10`、`TMAX=12` 时，理论量化步长约为：

| 反馈量 | 步长 |
|---|---:|
| 位置 | `9.58e-5 rad`，约 `0.00549 deg` |
| 速度 | `4.88e-3 rad/s` |
| 力矩 | `5.86e-3 N m` |

`PMAX/VMAX/TMAX` 是主机与固件共同使用的**编解码量程**。它们不是机械安全行程，也不是建议工作载荷。任一值不一致都会导致同一比特流在主机和设备上表示不同的物理量。

## 6. 寄存器协议
寄存器请求均发送到标准 CAN ID `0x7FF`。SDK 实际使用的 8 字节格式为：

```text
读取：[ID_L, ID_H, 0x33, RID, 0, 0, 0, 0]
写入：[ID_L, ID_H, 0x55, RID, value0, value1, value2, value3]
保存：[ID_L, ID_H, 0xAA, 0,   0, 0, 0, 0]
```

4 字节值使用小端序。SDK 将以下 RID 解码为 `uint32`：`7..10`、`13..16`、`35..36`；其他已知参数按 `float32` 解码。

### 6.1 集成常用寄存器

| RID | SDK 名称 | 类型 | 含义 |
|---:|---|---|---|
| 0 | `UV_Value` | float32 | 欠压保护阈值 |
| 2 | `OT_Value` | float32 | 过温保护阈值 |
| 3 | `OC_Value` | float32 | 过流设置 |
| 4 | `ACC` | float32 | 加速度，官网单位 Krad/s² |
| 5 | `DEC` | float32 | 减速度，官网单位 Krad/s² |
| 6 | `MAX_SPD` | float32 | 理论速度限制 |
| 7 | `MST_ID` | uint32 | 反馈帧 Master ID |
| 8 | `ESC_ID` | uint32 | 电机命令 CAN ID |
| 9 | `TIMEOUT` | uint32 | CAN 超时周期，官网计量单位 50 us |
| 10 | `CTRL_MODE` | uint32 | 控制模式 |
| 11 | `Damp` | float32 | 速度环阻尼因子 |
| 13..15 | `hw_ver/sw_ver/SN` | uint32 | 硬件版本、软件版本、序列号 |
| 16 | `NPP` | uint32 | 电机极对数 |
| 17..20 | `Rs/LS/Flux/Gr` | float32 | 相电阻、相电感、磁链、减速比 |
| 21 | `PMAX` | float32 | 位置编解码量程，rad |
| 22 | `VMAX` | float32 | 速度编解码量程，rad/s |
| 23 | `TMAX` | float32 | 力矩编解码量程，N m |
| 24 | `I_BW` | float32 | 电流环带宽 |
| 25..26 | `KP_ASR/KI_ASR` | float32 | 速度环 PI 参数 |
| 27..28 | `KP_APR/KI_APR` | float32 | 位置环 PI 参数 |
| 29 | `OV_Value` | float32 | 过压保护阈值 |
| 30 | `GREF` | float32 | 齿轮力矩传递系数 |
| 35 | `can_br` | uint32 | CAN 比特率配置枚举，映射未在 SDK 中公开 |

完整枚举见
[`registers.py`](../../sdk/Gloria-M-SDK/src/gloria_m_sdk/registers.py)。`Deta`、`V_BW`、`IQ_c1`、`VL_c1`、校准偏置和 `p_m/xout` 等条目的完整语义未在当前公开资料中定义，不建议在不了解固件版本的情况下写入。

### 6.2 控制模式值

| 值 | SDK 枚举 | 当前 ROS 支持 |
|---:|---|---|
| 1 | `MIT` | 是，`control_mode:=mit` |
| 2 | `POS_VEL` | 是，`control_mode:=pos_vel` |
| 3 | `VEL` | 否 |
| 4 | `TORQUE_POS` | 否，公开协议页未给出详细控制帧 |

`gloria_ros` 的 `~/configure` 会写入 RID 10 并等待回读，然后读取 RID 21、22、23 与 ROS 参数比对。它不会修改或保存 `PMAX/VMAX/TMAX`。量程不匹配时使能失败，这是为了避免错误缩放直接产生危险运动。

SDK 的 `GloriaGripper.connect()` 默认会写入并保存量程。直连实机做只读检查时应考虑使用 `connect(apply_limits=False)`；需要变更量程时，应停止 ROS bridge、核对参数，并只在必要时保存一次，避免无意义的 Flash 写入。

## 7. ROS 2 接口映射
完整接口见 [`README.md`](README.md)，硬件相关映射如下：

| ROS 接口 | 设备动作 |
|---|---|
| `~/mit_command` | 发送 MIT 8 字节控制帧到 `command_id` |
| `~/pv_command` | 发送两个 float32 到 `0x100 + command_id` |
| `~/command` | 按当前模式生成位置命令；MIT 使用固定 `kp/kd`，PV 使用固定速度 |
| `~/joint_states` | 发布反馈位置、速度和力矩到 position/velocity/effort |
| `~/configure` | 写控制模式并回读，校验 PMAX/VMAX/TMAX |
| `~/enable` | 配置模式、校验量程、使能并等待任意有效状态反馈 |
| `~/disable` | 发送失能；固件协议没有独立确认帧 |
| `~/refresh` | 发送 `0xCC` 状态请求并等待反馈 |
| `~/set_zero` | 发送 `0xFE`；默认禁用，且要求主机侧处于失能 |
| `/diagnostics` | 报告主机请求状态、反馈新鲜度、模式和三项运动反馈 |

重要默认参数：

| 参数 | 默认值 | 硬件含义 |
|---|---:|---|
| `command_id` | `0x01` | 电机命令 ID |
| `feedback_id` | `0x101` | 期望反馈 ID，必须与设备 Master ID 对应 |
| `pmax/vmax/tmax` | `3.14/10/12` | MIT 编解码量程，必须与固件一致 |
| `safe_position_min/max` | `0/2.77` | ROS 机械软件限位，必须按实机校准 |
| `feedback_timeout_s` | `0.5` | 反馈超过该时间视为失联 |
| `state_poll_period_s` | `0.1` | 使能后主动请求状态周期 |
| `enable_on_start` | `false` | 默认要求人工确认后使能 |

节点默认拒绝以下命令：非有限值、模式未确认、未使能、反馈过期、MIT 增益越界、
MIT 速度/力矩越界和 PV 速度越界。位置超出 `safe_position_min/max` 时会被夹紧到安全范围。

## 8. 方向、零点与机械限位标定
当前上游资料存在方向约定冲突：

- SDK `constants.py` 注释称 `0 rad` 为完全张开，正方向趋向闭合。
- SDK demos 和官方 SDK 页面通常使用 `open_q=2.5/2.77`、`close_q=0/0.003`，并约定正力矩张开、负力矩闭合。

因此不能把任一默认方向当作所有型号和装配的硬件事实。首次调试应：

1. 保持 `enable_on_start=false`，移除夹持物并准备急停/断电手段。
2. 先调用 `~/refresh`，确认 CAN ID、反馈量程和当前位置合理。
3. 在 PV 模式以很低速度发送一个很小的位置增量，观察实际开合方向。
4. 记录实机 `q_open` 和 `q_closed`，将二者的数值边界写入安全限位，并在上层单独保存
   “开/闭”语义。
5. MIT 力矩方向只用低幅值、短时间脉冲验证，并保留非零阻尼。
6. 未建立可靠机械基准前不要调用 `set_zero`。该操作会改变后续所有绝对位置语义。

`safe_position_min/max` 只是数值区间，不表达哪一端是张开或闭合；它也不能替代机械限位、驱动器保护、急停或上层碰撞检测。

## 9. 力矩反馈与夹持力估算
反馈 `torque` 是按 `TMAX` 解码的电机/减速器输出力矩估计，不是指端六维力传感器读数。连杆夹爪的近似指端夹持力依赖位置相关等效力臂：

$$
F_{grip}(q) \approx \frac{\tau_{contact}(q)}{r(q)}
$$

其中 `r(q)` 必须通过实际机构几何或标定得到。SDK 示例还会用空载基线扣除摩擦、重力和机构阻力。固定 `radius_mm=12`、示例目标力和接触阈值都只是演示初值，不是设备保证精度。更换指端、姿态、润滑状态或夹爪型号后应重新标定，并用独立测力计验证。

## 10. 推荐上电与调试流程
1. 核对实物型号，确认 24 V DC 电源的极性与供电能力，并检查 CAN 针脚和终端电阻。
2. 断电连接夹爪电源与 CAN，确认总线上没有 ID 冲突。
3. 确认夹爪 CAN 比特率与本仓库 bridge 配置一致。
4. 启动 bridge 和夹爪节点，但保持自动使能关闭：

   ```bash
   source scripts/env.sh
   ros2 launch can_bridge_ros can_bridge_ros.launch.py config:=single_bus.yaml
   ros2 launch gloria_ros gripper.launch.py \
     command_id:=1 feedback_id:=257 enable_on_start:=false
   ```

5. 请求一次反馈并观察状态：

   ```bash
   ros2 service call /gloria_gripper/refresh std_srvs/srv/Trigger '{}'
   ros2 topic echo /gloria_gripper/joint_states --qos-reliability best_effort
   ros2 topic echo /diagnostics
   ```

6. 在失能状态配置模式并校验固件量程：

   ```bash
   ros2 service call /gloria_gripper/configure std_srvs/srv/Trigger '{}'
   ```

7. 清空运动范围，以低速、小位移或小力矩使能测试：

   ```bash
   ros2 service call /gloria_gripper/enable std_srvs/srv/Trigger '{}'
   ros2 topic pub --once /gloria_gripper/command std_msgs/msg/Float64 '{data: 0.1}'
   ```

8. 测试结束立即失能：

   ```bash
   ros2 service call /gloria_gripper/disable std_srvs/srv/Trigger '{}'
   ```

上例 `0.1 rad` 仅演示消息格式，必须先根据当前零点、方向和机械位置选择安全目标。

## 11. 共总线约束
本工作区可让 Gloria-M 与 KWR57 力传感器共享 CAN：

- 单总线配置中，两台夹爪使用不同的命令/反馈 ID。
- 双总线配置中，不同通道可复用 ID。
- KWR57 高频帧通过 `rx_routes` 转发到专属话题，Gloria-M 使用默认 `/canX/rx`。
- 两个 1 kHz KWR57 在 1 Mbit/s 总线上已占用约 66.6% 的无填充带宽，最坏位填充估算约 81%。夹爪状态请求和命令频率应保守设置，并监控丢帧与总线错误。
- `0x7FF` 是设备参数/状态请求广播 ID，多个主机或配置工具同时写参数会产生竞态。

## 12. 故障排查

| 现象 | 优先检查 |
|---|---|
| 完全无反馈 | 供电、急停、CAN_H/L、参考地、终端、CAN 比特率、`feedback_id` |
| `refresh` 超时 | 设备 Master ID、命令 ID、bridge RX 话题、是否误用串口速率作为 CAN 速率 |
| `configure` 模式超时 | RID 10 回包 ID、固件兼容性、总线负载、是否有其他主机占用 |
| `PMAX/VMAX/TMAX mismatch` | 读取厂家配置；让 ROS 参数匹配固件，或在受控条件下重新配置固件 |
| 使能后立即失能 | 首帧未在 `response_timeout_s` 内到达，或反馈在 `feedback_timeout_s` 后过期 |
| 开合方向相反 | 实机重新标定 `q_open/q_closed`、力矩符号和零点，不要套用示例方向 |
| 位置/速度/力矩数值异常 | 主机与固件量程不一致，或反馈帧 ID/格式不匹配 |
| 运动抖动 | `kd`/阻尼过低、增益过高、加减速度不合适、总线丢帧或机构卡滞 |
| 夹持力估计偏差大 | TMAX、空载基线、力臂曲线、指端和型号不匹配 |
| `/diagnostics` 正常但设备报警 | 当前节点未解析 `ERR` 和温度，使用厂家工具或扩展反馈解析 |
