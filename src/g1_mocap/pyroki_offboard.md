# 在外部 GPU 工作站使用 PyRoki

本文记录如何把 `g1_mocap` 录制的数据交给 ProtoMotions/PyRoki 做离线全身重定向。机器人计算机只保存数据和显示实时解析结果，不安装 JAX、PyRoki 或 ProtoMotions，也不执行本流程。

PyRoki 是可选实验工具。离线收敛、关键点误差降低或脚底指标改善均不代表动力学可行、控制策略可跟踪或真机安全。

## 输入与不可变原件

一次成功录制产生一对同名文件：

```text
locomotion_walk_forward_001.csv
locomotion_walk_forward_001.source.npz
```

同时保存录制时使用的 G1 URDF。把三者复制到外部工作站，并在处理前记录哈希：

```bash
sha256sum locomotion_walk_forward_001.csv \
  locomotion_walk_forward_001.source.npz \
  g1_29dof_mode_15.urdf > input.sha256
```

原文件只读保存。所有 PyRoki 中间文件、候选 CSV、报告和日志写到另一个目录。

CSV 是实时解析重定向的基线，固定 50 Hz、每行 36 列：

```text
pelvis_pos_xyz(3), pelvis_quat_xyzw(4), joint_pos(29)
```

`.source.npz` 才是重新重定向的输入真值。当前版本 2 的主要字段如下；读取器仍兼容
没有 `landmark_iterations` 的版本 1，并按两步校正重放：

| 字段 | 形状 | 含义 |
| --- | --- | --- |
| `format_version` | 标量 | 当前为 `2` |
| `landmark_iterations` | 标量 | 录制时实时肢体方向 DLS 的迭代次数 |
| `joint_names` | `(24,)` | PICO SMPL 关节顺序 |
| `timestamps` | `(N,)` | 头显单调时间戳 |
| `sequences` | `(N,)` | 原始连续帧序号 |
| `positions` | `(N,24,3)` | 未做人机缩放的骨骼位置，X 前、Y 左、Z 上，单位米 |
| `orientations` | `(N,24,3,3)` | 同坐标系下的关节旋转矩阵 |
| `statuses`, `messages` | `(N,)` | PICO 跟踪状态 |
| `calibration_scale` | 标量 | 人到机器人位移缩放 |
| `calibration_pelvis_ref_z` | 标量 | 校准时缩放后的骨盆高度 |
| `calibration_stand_height` | 标量 | G1 站立骨盆高度 |
| `calibration_pelvis_fix` | `(3,3)` | 骨盆朝向修正 |

完整字段以 [capture_stream.py](../g1_mocap/capture_stream.py) 中 `SourceClip.save()` 为准。读取时必须使用 `numpy.load(..., allow_pickle=False)`，检查有限值、严格递增时间戳、旋转矩阵正交性以及 `statuses == 1`、`messages == 0`。

## 工作站环境

建议使用 x86_64 Linux、NVIDIA GPU、足够的内存和显存。先安装匹配驱动/CUDA 的 JAX，再确认不是 CPU fallback：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install --upgrade "jax[cuda12]"
python -c 'import jax; print(jax.default_backend()); print(jax.devices())'
```

输出必须包含 `gpu` 和 `GpuDevice`。具体 JAX 安装命令应以工作站 CUDA 版本对应的官方 JAX 文档为准。

已在本项目做过接口核对的上游版本是：

```text
ProtoMotions 607ca7a0bb92e261120bcab8d9f97f28b3130ffc
PyRoki      388e43e1fc0d0ee382968d3dd72970fd62a0450c
jaxls       50a58be88c5ef74532f09e3f55268b4f02c490e3
```

按固定版本准备官方源码和 Python 包：

```bash
git clone https://github.com/NVlabs/ProtoMotions.git
git -C ProtoMotions checkout 607ca7a0bb92e261120bcab8d9f97f28b3130ffc

python -m pip install \
  tyro jaxlie jax_dataclasses jaxtyping loguru robot_descriptions \
  yourdfpy trimesh viser pyliblzfse
python -m pip install \
  "jaxls @ git+https://github.com/brentyi/jaxls.git@50a58be88c5ef74532f09e3f55268b4f02c490e3"
python -m pip install --no-deps \
  "pyroki @ git+https://github.com/chungmin99/pyroki.git@388e43e1fc0d0ee382968d3dd72970fd62a0450c"

python -c 'import jax, jaxls, pyroki, yourdfpy; print(jax.default_backend(), jax.devices())'
python -m pip freeze > environment.freeze.txt
nvidia-smi > gpu.txt
git -C ProtoMotions rev-parse HEAD > protomotions.commit.txt
```

最后一条 Python 命令仍须输出 `gpu` 和 `GpuDevice`。不要在没有重新做 A/B 验证的情况下静默升级上游。

## 转换为官方关键点输入

官方脚本 `pyroki/batch_retarget_to_g1_from_keypoints.py` 不能直接读取 `.source.npz`。本仓库刻意不再携带 PyRoki adapter；需要在工作站按以下稳定契约生成 `.npy` mapping，并把该 adapter 与处理结果一起版本化。

1. 以 CSV 行数 `M` 为输出长度，在 `0, 1/50, ..., (M-1)/50` 上重采样 sidecar。位置线性插值，旋转使用 SLERP；禁止先把 72/90 Hz 数据降采样后再插值。
2. 从 24 点中按以下顺序选择 15 点：

```text
pelvis,
left_hip, right_hip,
left_knee, right_knee,
left_ankle, right_ankle,
left_foot, right_foot,
left_shoulder, right_shoulder,
left_elbow, right_elbow,
left_wrist, right_wrist
```

3. 再追加三个辅助点，最终得到 18 点：

```text
left_hand_aux  = left_wrist  + 0.20 * unit(left_hand  - left_wrist)
right_hand_aux = right_wrist + 0.20 * unit(right_hand - right_wrist)
pelvis_aux     = pelvis      + 0.20 * calibrated_pelvis_forward
```

4. 对位置应用录制校准：

```text
positions *= calibration_scale
positions[..., 2] += calibration_stand_height - calibration_pelvis_ref_z
```

骨盆朝向由髋部左右轴和 `SPINE1 - PELVIS` 的上方向构造，再右乘 `calibration_pelvis_fix`。其余点使用重采样后的 PICO orientation。

5. 固定版本的官方 SMPL loader 会再次对根、下肢和上肢应用各向异性缩放。适配器必须预先做逆缩放，确保经过上游 loader 后恢复到上一步的目标位置；否则会发生二次缩放。
6. 生成 `left_foot_contacts`、`right_foot_contacts`，形状均为 `(M,2)`，两列分别表示 ankle 和 toe。接触标签应根据脚点高度与速度保守推断，保留真正腾空区间，不能填满短暂离地。
7. 保存 mapping：

```python
numpy.save(output_path, {
    "positions": positions_18,
    "orientations": orientations_18,
    "left_foot_contacts": left_contacts,
    "right_foot_contacts": right_contacts,
    "fps": numpy.array(50.0),
}, allow_pickle=True)
```

转换完成后至少检查：18 点形状、50 Hz 帧数、所有数值有限、旋转正交、首末时间覆盖以及足部接触没有跨越明显腾空段。

## 适配 G1 URDF

固定版本脚本要求三个目标 link。复制原 URDF，在副本中添加固定 link，不能修改录制原件：

```text
pelvis_contour_link  parent=pelvis                 xyz=0 0 0
left_foot_link       parent=left_ankle_roll_link   xyz=0.15 0 0
right_foot_link      parent=right_ankle_roll_link  xyz=0.15 0 0
```

三个 fixed joint 的 `rpy` 均为 `0 0 0`。确认副本中仍恰好包含动作契约的 29 个可动关节，并能解析所有 mesh 路径。

这里的 `left_foot_link` / `right_foot_link` 是 PyRoki 用来匹配人体 toe 的**代理关键点**，不是 G1 足底接触点。当前 G1 模型的真实足底由 `ankle_roll_link` 下四个 sphere collision 定义：中心约在 `x=-0.05/0.12 m, z=-0.03 m`，半径 `0.005 m`，最低点约为踝系 `z=-0.035 m`；它与上面的 `x=0.15 m, z=0` 代理点不重合。不要用代理点高度判断是否踩地。

固定版本的 G1 `foot_contact` 也不是地面约束：它按接触置信度惩罚 ankle/toe 代理点的帧间速度，并约束两点高度接近；`foot_tilt` 只鼓励脚部 z 轴朝上。它既不把真实足底碰撞点约束到世界 `z=0`，也不保证支撑期脚掌保持固定世界位姿。自碰撞默认同样关闭。因此即使接触标签和权重均已生效，输出仍可能悬空、穿地或滑动。

## 运行 PyRoki

长动作应切成有重叠的窗口。GPU 显存足够时优先使用较大的窗口，避免机器人端曾使用的 40 帧小窗带来大量重复计算。每个窗口单独运行和验收，防止所有窗口算完后才发现限位错误：

```bash
python ProtoMotions/pyroki/batch_retarget_to_g1_from_keypoints.py \
  --no-visualize \
  --keypoints-folder-path <单个或一批同长度窗口目录> \
  --output-dir <窗口输出目录> \
  --urdf-path <适配后的URDF> \
  --mesh-dir <mesh根目录> \
  --source-type smpl \
  --input-fps 50 \
  --subsample-factor 1 \
  --target-raw-frames <窗口帧数> \
  --skip-existing
```

先用 2 至 4 秒片段做 smoke test，记录耗时、峰值显存和输出误差，再决定正式窗口长度。官方脚本把每个窗口从默认关节姿态独立初始化，并允许最多 800 次非线性迭代；不要把 CPU smoke test 的速度外推到 GPU，也不要把窗口数并行开到超过显存。

## 转回 36 列 CSV

每个官方输出包含：

```text
base_frame_pos      (K,3)
base_frame_wxyz     (K,4)
joint_angles        (K,J)
fps
```

转换时需要：

1. 把 `base_frame_wxyz` 重排为 CSV 的 `xyzw`。
2. 按本项目 [motion_capture.py](../g1_mocap/motion_capture.py) 中 `JOINT_NAMES` 的顺序重排 `joint_angles`，不能假设上游 URDF 可动关节顺序相同。
3. 每个窗口转换后立即对照录制 URDF 检查 29 轴限位、四元数单位长度、有限值和 50 Hz；任何窗口失败就停止，不继续消耗 GPU 时间。
4. 重叠区内对根位置和关节角线性融合，对根四元数做符号连续的 SLERP。拼接后再次检查总帧数与原 CSV 完全相同。
5. 输出新的候选文件，例如 `pyroki_locomotion_walk_forward_001.csv`，绝不覆盖原 CSV 或 `.source.npz`。

## A/B 验收与回传

至少比较以下项目：

- G1 FK 后的髋、膝、踝、肩、肘、腕关键点对原始 PICO 目标的 p50、p95 和最大误差。
- 脚底穿地深度、支撑段滑移、接触切换处速度和加速度。
- candidate 相对实时 CSV 的根位移、关节 RMSE 和最大关节改变量。
- 关节顶限位、四肢姿态、动作语义、腾空是否保留以及窗口接缝是否可见。

实时 CSV 是质量基线，不是等待 PyRoki 覆盖的临时产物。只要 PyRoki 候选出现更明显的脚悬空、穿地或支撑滑移，就判为不通过；不要靠继续增大 `foot_contact` 权重掩盖代理点与真实足底几何不一致的问题。可在转成 36 列后另跑一次 `g1_mocap.contact_refine` 作为第三个独立候选，但必须重新比较接触、速度尾部和动作失真，不能假定后处理一定优于实时原版。

验收通过后，只把候选 CSV 和报告复制回机器人；原始 CSV 与 `.source.npz` 不动。把候选 CSV 放进 `/home/unitree/motions_dataset/motions/`，运行中的 dashboard 会自动在动作列表中显示它。回放仍只是运动学预览，真机执行前必须经过独立的动力学和控制安全验收。