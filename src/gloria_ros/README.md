# gloria_ros
Gloria-M 夹爪的 ROS 2 设备节点。节点不打开 Gloria SDK 自带的串口转 CAN
适配器，而是通过 `can_bridge_ros` 使用项目统一管理的 CAN 设备；可与 KWR57
节点共享总线。本包使用 `ament_cmake_python`，同时包含 Python 节点和 MIT/PV
强类型消息，不需要单独的接口包。

## 功能
- MIT 阻抗/扭矩模式与 PV 位置速度模式。
- 使能前设置控制模式，并等待设备寄存器回读确认。
- 使能、归零和状态刷新等待真实反馈，超时返回失败。
- 机械安全位置限幅、反馈超时保护、非有限值检查。
- 发布 `JointState` 和 `/diagnostics`。
- 节点退出时自动发送失能命令。
- 兼容设备使用反馈 CAN ID、命令 CAN ID 或 CAN ID 0 回传状态的固件。

## 前置条件
先启动 `can_bridge_ros`。运行终端需要执行：
```bash
source scripts/env.sh
```

该脚本把 Gloria-M-SDK submodule 的源码目录加入 `PYTHONPATH`。上游包入口会
导入其串口适配器，因此运行环境仍需安装上游声明的 `pyserial`，但本节点不会打开或使用该串口适配器。

## 启动
```bash
# MIT 模式，默认不自动使能
ros2 launch gloria_ros gripper.launch.py \
  command_id:=1 feedback_id:=257 control_mode:=mit \
  safe_position_min:=0.0 safe_position_max:=2.77

# PV 模式
ros2 launch gloria_ros gripper.launch.py \
  control_mode:=pos_vel pv_velocity:=0.5
```

生产环境建议保持 `enable_on_start:=false`，在确认 bridge、供电和机械环境安全后调用：
```bash
ros2 service call /gloria_gripper/enable std_srvs/srv/Trigger '{}'
```

服务会先设置并确认控制模式，然后使能并确认状态反馈。未确认模式、未使能或反馈
过期时，运动命令默认被拒绝。

## ROS 接口

### 订阅

| 名称 | 类型 | 说明 |
|---|---|---|
| `~/command` | `std_msgs/Float64` | 兼容位置接口；MIT 使用固定 kp/kd，PV 使用固定速度 |
| `~/mit_command` | `gloria_ros/msg/MitCommand` | `q/dq/kp/kd/tau` 阻抗和扭矩前馈命令 |
| `~/pv_command` | `gloria_ros/msg/PvCommand` | `position/velocity` 位置速度命令 |
| 配置的 `rx_topic` | `can_msgs/Frame` | bridge 接收帧 |

### 发布

| 名称 | 类型 | 说明 |
|---|---|---|
| `~/joint_states` | `sensor_msgs/JointState` | 位置、速度和反馈扭矩 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 在线、模式、反馈年龄和状态 |
| 配置的 `tx_topic` | `can_msgs/Frame` | 发往 bridge 的 CAN 帧 |

### 服务

| 名称 | 说明 |
|---|---|
| `~/configure` | 写入并确认控制模式，同时读取校验 PMAX/VMAX/TMAX |
| `~/enable` | 配置模式、校验量程、使能并等待反馈 |
| `~/disable` | 下发失能；固件没有独立确认响应 |
| `~/refresh` | 请求一次状态并等待反馈 |
| `~/set_zero` | 重设机械零点；默认禁用且要求夹爪已失能 |

## 重要参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `control_mode` | `mit` | `mit` 或 `pos_vel` |
| `pmax/vmax/tmax` | `3.14/10/12` | 必须与设备寄存器中的 MIT 编解码量程一致 |
| `safe_position_min/max` | `0/2.77` | 独立机械安全范围；应按实际夹爪型号校准 |
| `kp/kd` | `10/1` | 兼容位置接口在 MIT 模式下使用的增益 |
| `pv_velocity` | `1.0` | 兼容位置接口在 PV 模式下使用的速度 |
| `enable_on_start` | `false` | 延时启动后是否自动配置和使能 |
| `feedback_timeout_s` | `0.5` | 超过该时间认为反馈过期 |
| `response_timeout_s` | `0.5` | 服务等待设备确认的超时 |
| `state_poll_period_s` | `0.1` | 使能时主动请求状态的周期 |
| `verify_limits_on_configure` | `true` | 使能前读取固件 PMAX/VMAX/TMAX 并与 ROS 参数比对 |
| `allow_set_zero` | `false` | 是否开放危险的机械零点重设服务 |
| `disable_on_feedback_timeout` | `true` | 反馈过期时发送失能并阻断后续运动 |
| `require_enabled_for_command` | `true` | 未经成功使能时拒绝命令 |
| `require_fresh_feedback` | `true` | 反馈过期时拒绝命令 |
| `disable_on_shutdown` | `true` | 正常退出时发送失能 |

## 共总线注意事项
- 同一 CAN 总线上的命令 ID、反馈 ID 和 KWR57 数据 ID 必须互不冲突。
- KWR57 一个样本占三帧；两个 1 kHz KWR57 加两个夹爪不建议放在同一条 1 Mbps
  总线上。优先使用 `robot_bringup` 的双总线配置，或降低 KWR57 上传频率。
- KWR57 的 `direct_bus` 模式会独占物理设备，与 bridge 架构一起运行时必须关闭。
