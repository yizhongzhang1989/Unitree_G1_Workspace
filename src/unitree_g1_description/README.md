# unitree_g1_description

`unitree_g1_description` 是纯模型资源包。它提供 Unitree G1 29 轴本体、双 KWR57 安装件、双 Gloria-M 夹爪的组合模型，以及硬件无关的 ros2_control 资源声明；不包含硬件插件、controller、controller-manager facade 或 `/lowstate` 适配节点。

## 内容

| 路径 | 用途 |
|---|---|
| `model/G1-with-dual-Gloria-M.urdf.xacro` | 双夹爪整机组合模型 |
| `model/final.urdf` | G1 29 轴基础模型与关节限位来源 |
| `model/ros2_control_resources.macro.xacro` | 31 轴 command/state、双 FT 与 pelvis IMU 资源宏 |
| `model/meshes/` | 模型 mesh |
| `launch/description.launch.py` | 仅启动 `robot_state_publisher`，订阅已有 `/joint_states` |

资源宏声明：

- 31 个 position command interface：G1 29 轴、`left_eccentric_joint`、`right_eccentric_joint`；
- 每个关节的 position、velocity、effort state interface，共 93 个；
- `left_ft_sensor`、`right_ft_sensor` 各 6 个 force/torque state interface；
- `pelvis_imu` 的四元数、角速度和线加速度，共 10 个 state interface。

宏只描述资源名称、接口和由模型继承的控制限位。插件类型、话题、服务、增益、超时和 MotionSwitcher 策略属于部署配置，保存在 `unitree_g1_ros2_control` 包中。

## 只启动模型
已有标准 `/joint_states` 时，可独立启动模型与 TF：
```bash
source scripts/env.sh
ros2 launch unitree_g1_description description.launch.py
```

可覆盖的话题参数：
```bash
ros2 launch unitree_g1_description description.launch.py \
  joint_states_topic:=/joint_states \
  robot_description_topic:=/robot_description
```

这一路径不会打开 CAN、订阅 `/lowstate`、创建 controller manager 或发送任何运动命令。

## 整机控制
推荐由 bringup 一次启动设备和真实 Foxy ros2_control 栈：
```bash
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual
```

仅当 CAN bridge、Gloria-M 和 KWR57 节点已经由外部部署启动时，才单独启动控制栈：
```bash
ros2 launch unitree_g1_ros2_control control.launch.py topology:=dual
```

两条命令不可在同一 ROS graph 同时使用；前者已经 include 后者。

硬件接口、controller、发布频率和安全事务见 [unitree_g1_ros2_control/README.md](../unitree_g1_ros2_control/README.md)。
