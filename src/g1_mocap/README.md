# g1_mocap

PICO 4 Ultra 全身动捕（5 个 Motion Tracker，24 关节 SMPL 骨架）到 G1 29 轴关节角的重定向。
数据经 **WiFi** 从头显上的动作捕捉桥接软件 [madderscientist/PicoBridge](https://github.com/madderscientist/PicoBridge) APK 过来。

## 实时动捕

```bash
ros2 launch g1_mocap mocap.launch.py
ros2 launch g1_mocap dashboard.launch.py
```

头显连接 **本机 IP:18000**；面板访问 `http://<本机IP>:18080`。
自然站直后，**双摇杆同时按下**或在面板校准，随后发布 `/mocap/frame`、`/mocap/joint_states` 和 `/mocap/status`。
动作帧保留原始 72/90 Hz，下游按时间戳插值，不要先降采样。

## 录制动作

```bash
ros2 launch g1_mocap record_motion.launch.py \
  output_dir:=/home/unitree/motions_dataset \
  category:=locomotion action:=walk_forward model_confirmed:=true
```

头显改连 **本机 IP:18001**。采集源独立于实时控制，不要向两者同时发送。
确认模型与下游一致后才设 `model_confirmed:=true`；换模型用 `urdf_path:=绝对路径`。

1. 全部 tracker 跟踪有效，自然站直约 3 秒，双摇杆按下校准。
2. **右手 A** 开始，再按 **A** 停止并验证保存；**右手 B** 丢弃。
3. 每段独立保存，编号递增、不覆盖。换动作名需结束当前片段后重启 launch。

至少 2 秒，推荐 4 至 30 秒；**不会在 30 秒自动结束**，默认到 60 秒硬上限结束保存。
所有动作均允许腾空。Ctrl-C 丢弃未完成片段；失败后需重新按 A 开始，头显重连后需重新校准。

也可调用服务（`controller_buttons:=false` 关闭按键）：

```bash
ros2 service call /motion_capture/calibrate std_srvs/srv/Trigger '{}'
ros2 service call /motion_capture/start std_srvs/srv/Trigger '{}'
ros2 service call /motion_capture/stop std_srvs/srv/Trigger '{}'
ros2 service call /motion_capture/discard std_srvs/srv/Trigger '{}'
ros2 service call /motion_capture/capture_status std_srvs/srv/Trigger '{}'
```

采集预览，浏览器访问 `http://<本机IP>:18081`：

```bash
ros2 launch g1_mocap dashboard.launch.py \
  frame_topic:=/motion_capture/frame status_topic:=/motion_capture/status \
  calibrate_service:=/motion_capture/calibrate dashboard_port:=18081
```

## 输出

```text
motions_dataset/
  motions/locomotion_walk_forward_001.csv
  metadata.csv
```

- 无表头 CSV，每行 36 个有限浮点数：`pelvis_pos_xyz(3), pelvis_quat_xyzw(4), joint_pos(29)`。
- 固定 **50 Hz**，位置/关节线性插值，四元数 SLERP；不补断流、不镜像。
- 世界系 X 前、Y 左、Z 上，地面 Z=0，单位米/弧度；人机尺度和站立高度由校准对齐。
- 关节顺序见 [config/mocap.yaml](config/mocap.yaml)，启动时严格核验。
- metadata 列：`file_name,action,duration_seconds,fps,num_frames`，时长为 `num_frames/50`。


## 重定向

- **姿态求解**：人体关节位置决定肢体方向和屈伸；厂商关节朝向补充腕/踝自转，并在腿伸直时稳定膝轴。
- **模型约定**：关节轴、零位几何和限位从 G1 URDF 计算，不把人体关节角直接当作机器人关节角；输出角度限制在模型行程内。
- **站立校准**：按腿长比缩放位移，对齐骨盆高度、修正盆骨/躯干姿态偏置，再将人的站姿映射到 `default_joint_pos`，之后按动作增量重定向。因此必须站直校准，站姿不等于所有关节归零。
- **坐标与输出**：OpenXR 坐标转换为 X 前、Y 左、Z 上；根节点是 G1 pelvis，其他刚体位置由重定向关节角通过 G1 正运动学（FK）计算，不直接复制人体位置。

实现见 [g1_mocap/retarget.py](g1_mocap/retarget.py)，模型闭环测试见 [test/test_retarget.py](test/test_retarget.py)。真人体型差异与极端姿态仍需回放验收。


## 注意事项

- 默认使用 G1 `g1_29dof_mode_15.urdf`，轴、零位、限位取自模型。自定义模型须以 pelvis 为根、含指定 29 轴，脚部碰撞几何支持 sphere/box。面板显示模型需另行核对。
- 录制中跟踪失效、LIMITED、朝向缺失、掉帧、跳变或尝试校准会废弃整段。关节速度超过 30 rad/s、超限、非有限值和严重穿地也会拒绝保存。
- 禁止录制中 Home/recenter 或重设地面。建议使用 STAGE；小幅坐标重置可能漏检，发生时主动丢弃。
- 开录前预览左右映射和脚底接触，交付前回放检查顶限位、动作失真及不合理悬空。只采集能安全完成的动作。
- 实时模式允许 LIMITED、缺朝向时退化为位置解法；采集模式更严格。开放网络请设置 `token`。

## 开发

重定向实现：[g1_mocap/retarget.py](g1_mocap/retarget.py)。

```bash
cd src/g1_mocap && python3 -m pytest test/ -q
```