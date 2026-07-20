# unitree_g1_description
`unitree_g1_description` 提供 Unitree G1 全身模型、状态适配和统一 MIT 位置控制能力。该包把 G1 的 29 路本体关节与两只 Gloria-M 夹爪组合成标准的 31 路 `/joint_states`，并可将同一顺序的 31 个位置目标拆分为：
- G1 前 29 路 `/lowcmd` MIT 命令；
- 左、右 Gloria-M 各一路 MIT 命令。

控制节点只在 controller 已激活、反馈新鲜且目标通过限位与连续性检查时发送命令。默认由 MotionSwitcher 在 Engage 时释放当前高层运动模式，等待原 `/lowcmd` 流静默后接管；Disengage 时先停止低层输出，再恢复接管前的运动模式。

## 主要内容

| 路径 | 用途 |
| --- | --- |
| `model/` | G1、Gloria-M 及组合后的 URDF/Xacro 模型 |
| `config/default_29dof_param.yaml` | G1 本体 29 轴 MIT `kp`、`kd` 默认值 |
| `unitree_g1_description/lowstate_to_joint_states_node.py` | 将 `/lowstate` 与双夹爪状态合并为 31 路 `/joint_states` |
| `unitree_g1_description/mit_position_controller_node.py` | controller-manager facade 和 31 路统一 MIT 位置控制节点 |
| `unitree_g1_description/motion_switcher.py` | Unitree MotionSwitcher topic 客户端 |
| `unitree_g1_description/mit_command.py` | 增益与 URDF 限位加载、G1 `LowCmd` CRC |

## 启动
仅启动模型、状态合并和 TF，不发送运动命令：
```bash
source scripts/env.sh
ros2 launch unitree_g1_description g1_data.launch.py
```

在已有 `/lowstate`、双夹爪状态和 `/joint_states` 数据源时，启动统一 MIT 控制节点：
```bash
source scripts/env.sh
ros2 launch unitree_g1_description mit_control.launch.py
```

节点提供以下 controller-manager 兼容接口：
- `/controller_manager/list_controllers`
- `/controller_manager/switch_controller`
- `/whole_body_controller/commands`

`/whole_body_controller/commands` 使用 `std_msgs/msg/Float64MultiArray`，必须包含 31 个位置目标，并严格采用 `/joint_states` 中定义的控制顺序：前 29 项为 G1 本体关节，最后两项依次为 `left_eccentric_joint` 和 `right_eccentric_joint`。

完整数据链与测试 Dashboard 的启动方式见工作区根目录的 [README](../../README.md) 和 [robot_bringup/README](../robot_bringup/README.md)。

## `default_29dof_param.yaml`

### 出处
G1 29 轴默认增益整理自 Unitree 官方 `unitree_rl_mjlab` 仓库的部署参数：
[unitree_rl_mjlab/deploy/robots/g1/config/policy/velocity/v0/params/deploy.yaml](https://github.com/unitreerobotics/unitree_rl_mjlab/blob/main/deploy/robots/g1/config/policy/velocity/v0/params/deploy.yaml)

上游文件通过 `joint_ids_map: [0, ..., 28]` 标识 29 路关节，并将 `stiffness`、`damping` 用作部署时的关节 PD 增益；它们不是策略网络的权重。本包保留这两组数值，将其分别映射为 G1 MIT 命令的 `kp`、`kd`，并增加显式的 `joint_names` 列表，用于防止增益数组因关节顺序变化而错配。该文件只描述 G1 本体 29 轴，不包含两只 Gloria-M 夹爪；夹爪增益由 `mit_control.launch.py` 的 `gripper_kp`、`gripper_kd` 参数独立配置，默认分别为 `10` 和 `5`。

### 字段含义

| 字段 | 长度 | 含义 |
| --- | ---: | --- |
| `joint_names` | 29 | 每组增益对应的 G1 关节名及固定顺序，必须与代码中的 `G1_JOINT_NAMES` 完全一致 |
| `stiffness` | 29 | MIT 命令的比例增益 `kp`，表示位置刚度，近似单位为 `N·m/rad` |
| `damping` | 29 | MIT 命令的微分增益 `kd`，表示速度阻尼，近似单位为 `N·m·s/rad` |

统一控制节点对每个 G1 关节发送：
$$
\tau_{cmd} = k_p(q_{target} - q) + k_d(\dot q_{target} - \dot q) + \tau_{ff}
$$

当前实现设置 $\dot q_{target}=0$、$\tau_{ff}=0$，因此：
- `stiffness` 越大，关节对位置误差的纠正越强，表现为更“硬”；
- `damping` 越大，对关节速度的抑制越强，可减小振荡，但过大会使响应迟钝或产生较大的阻尼力矩；
- 每个数组下标必须始终对应同一 `joint_names` 下标，不能只调整列表顺序而不同时调整两组增益。

节点启动时会校验三个数组均为 29 项、数值有限且 `joint_names` 顺序完全匹配；任一条件不满足都会拒绝启动。修改增益后应先在机器人可靠支撑、目标保持当前反馈位姿且现场可急停的条件下验证，避免直接用未经验证的高增益驱动实机。

## 默认发布频率与安全参数
- G1 `/lowcmd`：500 Hz；
- 双 Gloria-M MIT 命令：100 Hz；
- Gloria-M：`kp=10`、`kd=5`，其中 `kd=5` 是 SDK/固件 12 bit MIT 编码范围的上限；
- Dashboard 命令超时：0.25 s，超时后保持最新反馈姿态；
- 运动模式恢复确认窗口：10 s；
- 未激活、反馈过期、目标越界或状态异常时不发送新的低层命令。

具体参数及默认值见 [mit_control.launch.py](launch/mit_control.launch.py)。