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

面板右侧列出已保存动作，点击即回放，可暂停、拖动进度、倍速或循环，点“返回实时”退出。动作右侧可二次点击确认归档；归档会同步移出 `metadata.csv`，并将原 CSV 及其 source sidecar 保留在 `motions/.trash/`，其编号不会复用。
默认读取 `/home/unitree/motions_dataset/motions/*.csv`；自定义录制目录时给面板加 `motions_dir:=录制目录`。回放不控制机器人。

每次成功保存都会同时得到实时解析结果和原始 PICO sidecar。机器人本机只负责录制与预览，不运行 PyRoki 等重型离线重定向。

## 输出

```text
motions_dataset/
  motions/locomotion_walk_forward_001.csv
  motions/locomotion_walk_forward_001.source.npz
  metadata.csv
```

- 无表头 CSV 是实时解析重定向的 50 Hz 结果，供现场回放和 dashboard 预览；每行 36 个有限浮点数：`pelvis_pos_xyz(3), pelvis_quat_xyzw(4), joint_pos(29)`。
- 同名 `.source.npz` 是离线处理的原始数据，保存 PICO 24 点位置、旋转、原始时间戳、跟踪状态和本次校准快照；CSV 与 sidecar 同事务保存、回滚和归档。
- 固定 **50 Hz**，位置/关节线性插值，四元数 SLERP；不补断流、不镜像。
- 世界系 X 前、Y 左、Z 上，地面 Z=0，单位米/弧度；人机尺度和站立高度由校准对齐。
- 关节顺序见 [config/mocap.yaml](config/mocap.yaml)，启动时严格核验。
- metadata 列：`file_name,action,duration_seconds,fps,num_frames`，时长为 `num_frames/50`。


## 重定向

- **姿态求解**：人体关节位置决定肢体方向和屈伸；厂商关节朝向补充腕/踝自转，并稳定膝轴和肘轴，避免肢体伸直或手臂越过躯干后方时发生翻转。
- **模型约定**：关节轴、零位几何和限位从 G1 URDF 计算，不把人体关节角直接当作机器人关节角；输出角度限制在模型行程内。
- **站立校准**：按腿长比缩放位移，对齐骨盆高度、修正盆骨/躯干姿态偏置，再将人的站姿映射到 `default_joint_pos`，之后按动作增量重定向。因此必须站直校准，站姿不等于所有关节归零。
- **动作贴合校正**：解析解作为种子，再用 G1 FK 和解析 Jacobian 默认迭代两步，校正四肢球窝三轴与铰链轴，使 G1 上下臂和大小腿的方向变化贴合原始 PICO 相对校准站姿的动作变化。它不添加固定姿态偏置，不追随其他重定向器的外观，也不改 pelvis、腰、踝/腕末端自转语义。
- **坐标与输出**：OpenXR 坐标转换为 X 前、Y 左、Z 上；根节点是 G1 pelvis，其他刚体位置由重定向关节角通过 G1 正运动学（FK）计算，不直接复制人体位置。

面向新手的完整原理和数据流见 [retarget_and_postprocess_guide.md](retarget_and_postprocess_guide.md)。实现见 [g1_mocap/retarget.py](g1_mocap/retarget.py)，模型闭环测试见 [test/test_retarget.py](test/test_retarget.py)。真人体型差异与极端姿态仍需回放验收。


## 注意事项

- 默认使用 G1 `g1_29dof_mode_15.urdf`，轴、零位、限位取自模型。自定义模型须以 pelvis 为根、含指定 29 轴，脚部碰撞几何支持 sphere/box。面板显示模型需另行核对。
- 录制中跟踪失效、LIMITED、朝向缺失、掉帧、根位姿跳变或尝试校准会废弃整段。关节超限和非有限值也会拒绝保存；不按关节速度丢弃或限速。脚底穿地不会阻止原始片段保存，须通过回放检查并按需执行离线接触修正。
- 禁止录制中 Home/recenter 或重设地面。建议使用 STAGE；小幅坐标重置可能漏检，发生时主动丢弃。
- 开录前预览左右映射和脚底接触，交付前回放检查顶限位、动作失真及不合理悬空。只采集能安全完成的动作。
- 实时模式允许 LIMITED、缺朝向时退化为位置解法；采集模式更严格。开放网络请设置 `token`。

## 简单离线修正

### 为什么需要后处理

实时重定向逐帧映射人体姿态，骨盆位移与腿部关节分别计算；人机比例、站姿偏置和关节限位会使原本踩地的脚在 G1 上滑动、倾斜或穿地。**CSV 格式合格、关节不超限，不代表接触合理。** 用于 motion track 的动作应检查这些问题，必要时通过后处理联合修正脚底接触。

后处理读取完整片段，推断连续支撑区间并平滑处理离地/落地。它是独立的离线步骤，**不在录制时计算，也不会在停止录制后自动执行**；保留原文件，修正版另存，便于对照验收。后续应把 CSV、同名 `.source.npz` 和录制使用的 URDF 复制到外部工作站处理，不占用机器人计算机。

### 导出与使用

1. 结束录制，确认日志提示 `Saved`，将 CSV、同名 `.source.npz` 和录制使用的 URDF 复制到装有本工作区依赖的外部工作站，然后执行：

```bash
source <外部工作区>/install/setup.bash
python3 -m g1_mocap.contact_refine \
  <输入目录>/locomotion_balance_001.csv \
  --source <输入目录>/locomotion_balance_001.source.npz \
  --output-dir <输出目录>/contact_refined
```

带 `--source` 时，程序读取原始 PICO 24 点和录制时的校准快照，逐帧调用实时使用的 `Retargeter.solve()`（沿用 sidecar 记录的 DLS 迭代次数；旧版按两步），再按实时导出规则重采样到 50 Hz，最后执行足底接触修正。这是需要重新应用当前重定向算法时的推荐路径。它会要求重放结果与输入 CSV 帧数一致，并在报告中记录 sidecar 哈希和重放相对旧 CSV 的最大关节变化。

不带 `--source` 时保持兼容行为：**只读取输入 CSV**，不会读取同名 sidecar，也不会恢复或重新拟合原始人体动作，只在该 CSV 的 root 和双腿上做足底修正。将输入路径替换为其他已保存动作即可，所有动作使用同一框架，不需要 `--mode`。自定义模型加 `--urdf /绝对路径/model.urdf`，须与录制时一致；建议原版和修正版使用不同目录。

2. 等待命令完成。输出保留原来的 50 Hz、帧数和 36 列格式：

```text
contact_refined/
  motions/refined_locomotion_balance_001_001.csv
  metadata.csv
  refined_locomotion_balance_001_001_report.json
```

输出名包含输入文件名，末尾编号自动递增、不覆盖。报告记录收敛情况、修正前后滑移/穿地/速度及姿态改变量；求解未收敛时会报错，不导出该次修正版。

3. 在外部工作站启动单独的 dashboard，或只将修正版 CSV 复制回机器人数据集的 `motions/` 目录。运行中的录制 dashboard 会自动刷新动作列表，不需要覆盖原版：

```bash
ros2 launch g1_mocap dashboard.launch.py \
  motions_dir:=<输出目录>/contact_refined \
  dashboard_port:=18084
```

浏览器访问 `http://<本机IP>:18084`，点击右侧动作播放；端口被占用时换一个。与原版对照脚底滑移、穿地、抬脚幅度和接触切换处的抖动，同时检查报告，不能只看锁脚效果。验收通过后交付输出目录下的 `motions/` 和 `metadata.csv`，原版和指标报告自行留存。

### 算法原理

- 用 G1 URDF 正运动学计算脚底碰撞接触点的世界位置，根据高度和垂直速度逐点判断支撑；使用滞回抑制状态抖动，不填补短暂腾空。
- 为连续支撑区间建立符合刚性脚掌几何的固定锚点；平脚支撑对齐地面，仅脚尖/脚跟支撑且无整脚支撑证据时保留脚掌倾角。接触首尾平滑进入和释放，过渡长度随短接触段缩短。
- 使用带关节限位的非线性最小二乘，联合调整根平移和双腿 12 轴，同时惩罚接触偏差、穿地、偏离原动作和修正量突变。求解器先按 Jacobian 缩放米/弧度混合变量，仅失败帧从同一初值回退单位尺度。脚点跟踪权重随接触置信度平方变化，稳定支撑点权重为 1000，非接触点保留权重 1；根修正的时间权重高于腿关节。根姿态、腰和上肢不变；无接触且无穿地的帧原样保留。

在 `locomotion_walk_forward_001` 的 810 帧实录上，原始 sidecar 重放和接触修正 810 帧全部收敛。相对只基于旧实时 CSV 的 `_003`，新统一候选在相同接触标签和锚点下：四肢段方向相对原始 PICO 动作增量的误差 p50 从 6.76 降至 0.825 mm、p95 从 32.56 降至 27.74 mm；最坏支撑段滑移从 5.47 降至 4.98 mm，支撑 XY 速度 p95 从 6.39 降至 6.29 mm/s；最大穿地从 1.096 增至 1.125 mm。关节速度 p95 从 1.214 降至 1.201 rad/s，腿关节加速度 p95 从 35.17 降至 34.43 rad/s²。接触修正会让腿偏离逐帧人体方向目标，但上肢逐位保留实时 DLS 结果；候选仍需视觉和真机可跟踪性验收。

当前范围是**平地足部接触的运动学修正**：已验证真实 balance 和合成多接触动作，尚未完成真实跳跃、手撑地、跪地及动力学验收。接触来自推断，支撑约束是数值惩罚，不保证严格零滑移，也不保证平衡、接触力或真机可跟踪性。

## PyRoki 外机处理

本包不再安装、封装或运行 ProtoMotions/PyRoki。当前轻量修正仅吸收其置信度加权和分组时间正则思路，不包含全片段联合优化。需要从原始 PICO 骨骼重新做全身优化时，将 CSV、同名 `.source.npz` 和录制使用的 URDF 搬到带 NVIDIA GPU 的工作站，按 [pyroki_offboard.md](./pyroki_offboard.md) 转换数据并运行官方工具。

官方 PyRoki 不能直接读取本项目的 `.source.npz`，也不能直接输出本项目的 36 列 CSV；两端都必须按文档做适配和严格校验。只有旧 CSV、没有同名 sidecar 时无法恢复人体关键点。候选须另存并与实时原版并排回放，不能覆盖原始录制。

PyRoki 的 G1 `foot_contact` 是软代价：支撑期主要惩罚 ankle/toe 代理点的帧间速度和两点高度差，不把真实足底碰撞点钉在 `z=0`，也不把整块脚掌刚性锁在世界系。适配 URDF 中新增的 `*_foot_link` 只是关键点代理，不是足底碰撞几何。因此直接输出仍可能悬空、穿地或滑动；`foot_contact` 权重非零不表示接触成立。若同一原始片段上 PyRoki 的踩地和滑移劣于实时 CSV，应直接判候选不通过并保留实时导出，不以关键点误差更小作为替换理由。需要继续试验时，先转换为 36 列，再把它作为独立输入运行“简单离线修正”并重新 A/B；后处理也不是必然能救回该候选。

## 开发

重定向实现：[g1_mocap/retarget.py](g1_mocap/retarget.py)，离线修正：[g1_mocap/contact_refine.py](g1_mocap/contact_refine.py)。

```bash
cd src/g1_mocap && python3 -m pytest test/ -q
```