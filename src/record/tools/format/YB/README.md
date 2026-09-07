# YB 数据集格式 v0.1

上肢双臂遥操作的模仿学习数据集。**一份 YB 就是一个 dataset**，里面是若干条 episode，
每条 = **一个 h5 + 三路相机各一个 mp4**。

**一份 dataset 通常由多次采集（session）合并而成**，不是一次采集一份 —— 交付单位是
数据集，不是某一天的采集。合并后 `meta.json` 与 `episodes_all.*` 全局各只有一份，
episode 文件序号跨采集连续（§3）。

导出工具就在旁边：`export.py`；统一入口是 `tools/convert.py --to yb`，它的第一个位置
参数可以给多个 session 目录：

```bash
python3 convert.py <session A> <session B> <session C> --to yb -o <输出目录> \
        --urdf final.urdf
```

---

## 0. 规格一览

| | |
| --- | --- |
| **采样率** | **统一 30 Hz**。h5 的每一行、mp4 的每一帧，都落在同一条 30 Hz 栅格上 |
| **对齐方式** | **h5 第 k 行 ⇔ mp4 第 k 帧**，同一时刻。不用查时间戳、不用插值 |
| **一条 episode** | 1 个 `.h5` + 3 个同名 `.mp4`（headcam / leftcam / rightcam） |
| **世界坐标系** | **`torso_link`**，跟着机器人一起动。右手系，+X 前 / +Y 左 / +Z 上（REP-103） |
| **长度单位** | 米 |
| **角度单位** | 弧度 |
| **四元数顺序** | `xyzw`（**不是** wxyz） |
| **关节** | 29 个，顺序读 `meta/joint_space/names`，绝对角不是增量 |
| **夹爪** | 连续值 `[0, 1]`，`0 = 完全张开`，`1 = 完全夹紧` |
| **相机外参** | `world_xyz = extrinsic @ camera_xyz`（相机在世界系下的位姿） |
| **相机光学系** | OpenCV 惯例：+X 图像右 / +Y 下 / +Z 进入场景 |
| **视频** | H.264 mp4，30 fps，默认高 360（源分辨率可选） |
| **缺数据** | `state` 侧填 `NaN` 并置 `valid_mask=0`，**绝不填 0** |
| **episode 边界** | 已掉掉首尾的无动作空转，各留 1 s（§1.1） |
| **收录范围** | **只有采集时标注为成功的 episode**，fail / discard 一律不导（§1.2） |
| **数据集组成** | 一份 dataset = 若干次采集合并，共用一份 `meta.json` 与 `episodes_all.*`（§3） |

---

## 1. 转换脚本把原始数据变成了什么

**原始采集是一堆各跑各的异步流**，速率不同、时间戳不同、起止不同。
实测一次 104 秒的采集：

| 原始 | 速率 | |
| --- | --- | --- |
| `joint_states` | 100.5 Hz | 抖动 ±9.8 ms |
| `motion_control_status`（末端实测） | 49.9 Hz | |
| `motion_control_command`（末端目标） | 49.5 Hz | **只在遥操作那 59 秒里有** |
| 力/力矩 ×4 路 | 138~175 Hz | |
| IMU ×2 路 | 81.7 / 98.5 Hz | |
| head 相机 | 24.9 fps | 三路相机**帧率互不相同** |
| wrist_left 相机 | 29.8 fps | |
| wrist_right 相机 | 24.9 fps | |

**YB 把这些全部投影到一条 30 Hz 的公共时间栅格上**，然后按 episode 切段：

```
原始：  关节 ~100Hz  ────•──•───•──•────•──•──
        指令  ~50Hz  ──•─────•─────•─────•────
        视频 ~25fps  ─•────•────•────•────•───
                          ↓  重采样到公共栅格
YB：     30 Hz       ├────┼────┼────┼────┼────┤
                     0    1    2    3    4    5   ← h5 行号 == mp4 帧号
```

具体做了五件事：

1. **重采样**：零阶保持（沿用上一个样本），**不做插值** —— 四元数、离散状态位、
   指令都不能线性插，与其一个字段一套规则不如统一保持。上一个样本超过 100 ms
   没更新就判无效（见 §6）。
2. **切段**：按采集时标注的 episode 边界切，每条独立成文件，带上指令文本和成败。
3. **掐掉两头的空转**：见下。
4. **补算**：末端位姿和相机外参原始数据里没有，用 URDF + 标定做 FK 现算
   （所以导出要传 `--urdf`）。同时给一份跨机器人统一约定的 `pose_unified`（§5.3）。
5. **转视频**：三路各自重采样到 30 fps 并降到 360p，**帧与 h5 行严格一一对应**。

**没做的事**：不做滤波、不做去噪、不做归一化（夹爪除外）、不丢弃异常值。
除了重采样和单位约定，数值都是原样。

### 1.1 两头的空转是怎么掐的

采集时操作者要先点「开始本轮」、走回工位、做完、再走回来点成败，所以每条 episode
的头尾都挂着几秒纯发呆。**实测一次 13 条的采集，183.1 s 里有 77.2 s（42%）是这种空转。**
照原样导出，模型会在「什么都不动」上花掉四成算力。

判据是**遥操作目标在不在变**（`motion_control_status` 的 `limited.*` 末端目标 +
`grip.*` 夹爪开度），不是实测位姿 —— 实测值有伺服抖动和噪声底，目标值静止时是
**精确的 0**。取 ±0.1 s 窗口，任一只手超过阈值就算在动：

| 常量 | 值 | |
| --- | --- | --- |
| `IDLE_POS_M_S` | 0.005 m/s | 位姿看**速率**。实测空转 p05 = 0.000 m/s，操作中 p95 = 0.11~0.23 m/s，差四个数量级 |
| `IDLE_GRIP_RAD` | 0.28 rad | 夹爪看**幅度**（行程的 10%），原因见下 |
| `IDLE_WINDOW_S` | 0.2 s | |
| `DEFAULT_KEEP_IDLE_S` | 1.0 s | 首尾各留这么多，别把起手和收势削秃 |

夹爪的阈值为什么不能只看开合、也不能看速率：

* 「左手把东西递给右手」这类指令操作者**全程夹着**，按开合与否判一秒都裁不掉。
* VR 扬机是模拟量，手指碰一下能在 0.19 s 里跑出 **0.054 rad（行程的2%）= 0.29 rad/s**，
  按速率判就是真动作 —— 实测就是它让一条 episode 开头留了 2.3 s 的静止画面。
  真的开合一定走完全行程，峰值窗口幅度 p50 = 1.18 rad，高出一个数量级。

夹爪这一项在实测的 13 条 episode 上**一次也没改变过边界**（夹爪动的时候手臂也在动），
留着是为了「手臂停着只合爪」那种真实情形。

裁了多少会在导出日志里逐条打出来。`--no-trim` 关掉，`--keep-idle` 调余量。
没录 `motion_control_status`、或整条都没动时，原样保留不裁。

### 1.2 只收成功的

采集时每条 episode 当场标 `success` / `fail` / `discard`，**导出只取 `success`**。
失败轨迹对模仿学习是负样本，混在里面等于教模型把东西碰倒。丢掉几条会在导出日志
第一行打出来。被丢掉的 episode **在原始采集里一样不动**，要的话可以自己改脚本重导。

> 文件名里的序号在整份 dataset 内连续，而 `round0_episode2` 这段是采集时的原始编号 ——
> 中间跳号就是那几条没做成。

---

## 2. 一分钟上手

```python
import h5py, json, numpy as np

meta = json.load(open('meta.json'))                     # 整个数据集共用的约定
eps  = json.load(open('episodes_all.json'))['episodes'] # 有哪些 episode

ep = eps[0]
name = ep['episode_name']                       # h5 和三个 mp4 共用这一个基名
f  = h5py.File(f'data/{name}.h5', 'r')

q      = f['state/joint_space/position'][:]     # (N, 29) 关节角，rad
pose    = f['state/end_space/pose_unified'][:]  # (N, 2, 7) 双臂末端 xyz + 四元数
grip    = f['state/actuator_space/value'][:]    # (N, 2) 夹爪 0=张开 1=夹紧
target  = f['action/end_space/pose_unified'][:] # (N, 2, 7) 同一时刻的目标位姿
video   = f'video_headcam/{name}.mp4'           # 第 k 帧 == 第 k 行
```

**唯一必须记住的一条：`h5` 的第 k 行和 `mp4` 的第 k 帧是同一时刻。**
不用查时间戳、不用对齐、不用插值。全部数据已经重采样到 **30 Hz 的统一栅格**上了。

---

## 3. 目录长什么样

```
meta.json                                数据集级约定（机器人、坐标系、夹爪定义…）
episodes_all.json                        全部 episode 的索引
episodes_all.h5                          同一份索引的 h5 版（只有帧区间与指令）
data/<name>.h5                           一条 episode 一个
episode/<name>.json                      同一条的索引，单独一份
video_headcam/<name>.mp4                 头部相机
video_leftcam/<name>.mp4                 左腕相机
video_rightcam/<name>.mp4                右腕相机
```

**上面这四个顶层文件/目录整个 dataset 各只有一份**，合并了多少次采集都一样：
`meta.json` 和 `episodes_all.*` 覆盖全部 episode，`data/` 与三个 `video_*/` 是平铺的，
**不按采集分子目录**。哪一条来自哪次采集看文件名里的采集批次（见下）。

**相机的名字用对面样例的词**（`headcam`/`leftcam`/`rightcam`），不是采集端的流名。
对面还有一路 `frontcam`（机器人前方的外部相机），我们没有，所以是三路不是四路 ——
顺序一律读 `meta/camera_space/names`，别按位置硬编。

同名的 `.h5` 和三个 `.mp4` 属于同一条 episode。文件名格式：

```
00000001-20260821_102522__g1__round0_episode0
└──┬───┘ └──────┬──────┘  ┬   └──┬─┘ └───┬──┘
  序号      采集批次      机器人  第几轮   该轮第几条
```

**序号在整个 dataset 内从 1 开始连续**，跨采集批次一起编下去（前一次采集导完接着往下排），
所以它就是这份 dataset 里的唯一编号。**采集批次在一份 dataset 里会有多个值** ——
合并了几次采集就有几种，按它就能把 episode 归回原始采集。跨 dataset 不保证唯一，
要全局唯一请用整个文件名。

---

## 4. h5 里有什么

一条 episode 的 h5 是四组东西。`N` = 帧数 = `end_frame - start_frame + 1`。

```
timestamp                 (N,)          float64   Unix 时间（秒），间隔恒为 1/30
state/…                   机器人实际处在什么状态（从传感器来）
action/…                  同一时刻下发的目标是什么（从控制指令来）
meta/…                    上面这些数组的名字、单位、约定
```

`state` 和 `action` 的结构一一对应，都分成四个「空间」：

| 空间 | 说的是 | 有 state | 有 action |
| --- | --- | :-: | :-: |
| `joint_space` | 29 个关节 | ✓ | ✓ |
| `end_space` | 双臂末端位姿 | ✓ | ✓ |
| `actuator_space` | 双夹爪开合 | ✓ | ✓ |
| `camera_space` | 相机内外参 + 帧号 | ✓ | — |

### 4.1 `joint_space` — 关节

| 数据集 | 形状 | 单位 | 说明 |
| --- | --- | --- | --- |
| `state/joint_space/position` | (N, 29) | rad | 关节角 |
| `state/joint_space/velocity` | (N, 29) | rad/s | 角速度 |
| `state/joint_space/effort` | (N, 29) | N·m | 力矩 |
| `action/joint_space/position` | (N, 29) | rad | 下发的目标角 |
| `*/joint_space/valid_mask` | (N,) | uint8 | 见 §6 |

关节顺序在 `meta/joint_space/names`（29 个字符串）。别自己按名字猜顺序，
**读 `names`**。分组索引在 `meta/joint_space/roles/`：

```python
names = [s.decode() for s in f['meta/joint_space/names'][:]]
left  = f['meta/joint_space/roles/left_arm'][:]     # [15..21]
q_left_arm = f['state/joint_space/position'][:, left]
```

`roles` 有 `left_arm` `right_arm` `left_leg` `right_leg` `waist`。
`position` 是**绝对角**不是增量（`meta/joint_space/state_position_type = "absolute"`）。

> ⚠️ **本批数据 `action/joint_space` 全是 NaN**，`valid_mask` 全 0。
> 遥操作是在末端空间下指令的，没有关节级目标。要用目标就用 `action/end_space`。
> 每次导出都会打一行有效率，**导出时看一眼**，别到训练时才发现整列是 NaN。

### 4.2 `end_space` — 末端位姿

| 数据集 | 形状 | 说明 |
| --- | --- | --- |
| `state/end_space/pose` | (N, 2, 7) | 原始末端系 |
| `state/end_space/pose_unified` | (N, 2, 7) | **统一约定，优先用这个** |
| `action/end_space/pose` | (N, 2, 7) | 目标，原始末端系 |
| `action/end_space/pose_unified` | (N, 2, 7) | 目标，统一约定 |

- 中间那一维：`0` = 左臂，`1` = 右臂（`meta/end_space/names`）。
- 最后一维 7 个数：`[x, y, z, qx, qy, qz, qw]`，位置单位 **米**，
  四元数 **xyzw 顺序**（不是 wxyz）。
- 参考系是 `torso_link`（§5.1），写在 `meta.json → world_frame`；
  h5 里 `meta/end_space/state_pose_reference = "robot"` 指的就是它。
- `pose` 用的是 URDF 里 `left/right_gripper_base` 这个 link 的原生朝向；
  `pose_unified` 把它转成了统一约定，两者位置相同、只差一个固定旋转。详见 §5.3。

### 4.3 `actuator_space` — 夹爪

| 数据集 | 形状 | 说明 |
| --- | --- | --- |
| `state/actuator_space/value` | (N, 2) | 实际开合（CAN 实测角） |
| `action/actuator_space/value` | (N, 2) | 目标开合（遥操作扳机） |
| `*/actuator_space/valid_mask` | (N,) | 见 §6 |

**连续值，范围 [0, 1]，`0 = 完全张开`，`1 = 完全夹紧`**。不是二值开关。
（`meta.json → robot.gripper.normalized` 里也写着这句。）
中间那一维 `0` = 左、`1` = 右，见 `meta/actuator_space/names`。

原始量是偏心关节角 0…2.7638 rad，方向相反，导出时取补归一化成
`clip(1 - rad / 2.7638, 0, 1)`；两列同公式同常量，可以直接相减。
原始定义留在 `meta.json → robot.gripper.stored_raw`。

**两列的差就是「夹住了」**：堵转时 `state` 停在中间而 `action` 已经到 1.0，
空夹时两者收敛。别把 `state` 当成 `action` 的延迟版本。实测偶尔超出行程
（量到过 2.889 rad）会被裁到 0，所以 `state` 的 0 不是精确的机械零位。

### 4.4 `camera_space` — 相机

| 数据集 | 形状 | 说明 |
| --- | --- | --- |
| `state/camera_space/intrinsic` | (N, 3, 3, 3) | 每路相机的 K 矩阵 |
| `state/camera_space/extrinsic` | (N, 3, 3, 4) | 每路相机的位姿（3×4） |
| `state/camera_space/frame_index` | (N, 3) | 对应**原始采集**里的帧号 |

第二维的 3 是相机，顺序在 `meta/camera_space/names`：
`['headcam', 'leftcam', 'rightcam']`。

`frame_index` 指的是**原始 mkv** 里的帧号，不是导出 mp4 的帧号。
只有要回溯到原始素材时才需要它；用导出的 mp4 时直接按行号取帧即可。
`-1` 表示那一刻这路相机没有画面（掉线或还没开始）。

> ⚠️ **K 对应的是相机原生分辨率，不是 mp4 的分辨率。**
> 本批 K 是 1280×720（头）和 1920×1080（腕），而 mp4 是 640×360。
> 直接拿 K 往 mp4 像素上套会差 2~3 倍。换算：
>
> ```python
> K  = f['state/camera_space/intrinsic'][0, 0]           # headcam
> sw, sh = f['meta/camera_space/intrinsic_size'][0]      # [1280, 720]
> vw, vh = 640, 360                                      # mp4 自己的尺寸，解一帧就知道
> K = K * np.array([[vw/sw, vw/sw, vw/sw],
>                   [vh/sh, vh/sh, vh/sh],
>                   [1,     1,     1    ]])
> ```
>
> 之所以不在导出时替你缩好：相机的档位里既有缩放也有裁剪（848×480 → 640×480 是
> 裁剪，`fx` 根本不变），按比例硬换出来的 K 看着正常但是错的。所以只给精确值 + 尺寸。

`static_intrinsic` / `static_extrinsic`（各 3 个 0/1）标明该路参数是否全程不变。
本批：内参三路都是常量；外参是 `[1, 0, 0]` —— **头部相机固连 `torso_link`，外参逐帧不变**，
两个腕相机跟着手臂走，必须逐帧读。

---

## 5. 坐标系

**这一节是最容易搞错的地方，请照着读，不要按习惯猜。**

### 5.1 世界系

**世界系 = `torso_link`。** 右手系，`+X 向前，+Y 向左，+Z 向上`（REP-103）。
末端位姿就是拿实测关节角在这条链上做一次 FK，没有任何额外变换。

为什么根选在 `torso_link` 上：它是两条手臂链的共同根，也是 IK 的 `base_frame`，
末端位姿在它下面是纯手臂的量，不随腰腿姿态漂。

零位实测：左肩在 `+Y 100 mm`、右肩 `−Y 100 mm`、双夹爪 `+X 303 mm`、
头相机 `+Z 428 mm`。

> ⚠️ **`torso_link` 跟着机器人一起动。** 实测遥操时躯干姿态 p95 **7.31°**，
> 折到 0.5 m 力臂上是 **63.8 mm** —— 桌上一个没动的物体在数据里会跟着漂。
> 要把这部分除掉得自己做（比如拿外部定位把位姿换到固定系下），导出不管。

### 5.2 相机外参的方向

`extrinsic` 是 3×4 矩阵 `[R | t]`，**定义就是下面这一行**：

```
world_xyz = extrinsic @ camera_xyz
```

也就是**把相机系下的点变到世界系**，等价于「相机在世界系下的位姿」。
矩阵是 3×4，所以实际算的时候记得补齐次项：`world = R @ cam + t`，即

```python
world = extrinsic[:, :3] @ camera_xyz + extrinsic[:, 3]
```

方向名叫 `base_T_cam`，写在 `meta/camera_space/extrinsic_direction`，
上面那行公式原样写在 `meta.json → camera_space.extrinsic_formula`。

> ⚠️ **不要拿「world2camera」「camera_to_world」这类叫法当依据。**
> CV 圈和机器人圈把这些词指向相反的两边，我们已经因此弄反过一次。
> 认公式，不认名字。
>
> 自查办法一：腕相机是拧在夹爪上的，把 `extrinsic` 的平移列当成相机位置，
> 量它到同侧末端的距离，**必须全程是常数**（本机 7.4 / 7.7 cm）。
> 自查办法二：按公式的逆把末端位置投回头相机画面，十字要落在夹爪上。

相机自身的光学系是 **OpenCV 惯例**：`+X 图像向右，+Y 向下，+Z 向前进入场景`。
零位实测确认过，**不需要再做任何轴变换**。

### 5.3 末端的统一约定

同一个「夹爪末端」，不同机器人的 URDF 朝向五花八门。这里给两份：

- **`pose`** —— URDF 原生的 `left/right_gripper_base`。实测：两指沿 **±X** 开合
  （张开 ±41.1 mm、夹紧 ±17.3 mm），指尖在 **+Z 61 mm**，腕相机在 **+Y 43 mm**。
- **`pose_unified`** —— 统一到 `approach_z_closing_y`：

  | 轴 | 指向 |
  | --- | --- |
  | `+Z` | **approach**：从夹爪尾部指向指尖，也就是「伸过去抓」的方向 |
  | `+Y` | **closing**：两指闭合的方向 |
  | `+X` | 朝腕相机那一侧（右手系补齐，X = Y × Z） |

两者只差一个固定旋转 `R_fix = Rz(+90°)`，位置完全相同：

```
q_unified = q_raw ⊗ R_fix,   R_fix(xyzw) = (0, 0, √2/2, √2/2)
```

`R_fix` 和目标约定名都在 `meta.json → ee_convention` 里，逐臂给。

**跨机器人训练用 `pose_unified`，回放到本机用 `pose`。**

---

## 6. `valid_mask` —— 请务必看，state 和 action 的含义不一样

`joint_space`、`end_space`、`actuator_space` 各带一个 `valid_mask`，形状 `(N,)`，
`uint8`。**同一个名字，在 `state` 和 `action` 下是两件事**：

| | `1` 的含义 | `0` 时数值是什么 |
| --- | --- | --- |
| `state/*/valid_mask` | 这一行的读数是新鲜的 | **`NaN`** |
| `action/*/valid_mask` | **这一拍有新指令发出** | 上一条指令的保持值（**是真实数字**） |

### state 侧

数据是把各路异步消息重采样到 30 Hz 栅格上得到的，采用零阶保持（沿用上一个样本）。
**上一个样本超过 100 ms 没更新就判为无效**，而不是继续拖一个陈旧的值。
无效处填 `NaN` 而不是 0 —— 0 是一个合法的关节角，填 0 下游看不出是缺的。

```python
m = f['state/end_space/valid_mask'][:].astype(bool)
pose = f['state/end_space/pose_unified'][:][m]      # 只取可信的行
```

### action 侧

`action` 的 `valid_mask` 回答的是**「这一拍机器人被下了新指令吗」**，
不是「这个数能不能用」。指令原始速率约 50 Hz，比 30 Hz 的栅格还密，所以绝大多数
拍都有新指令；偶尔有一拍正好没赶上，那一行就是 0，
而**数值仍然是上一条指令的保持值，是可以直接用的真实数字**。
本批 623 行里只有 5 行是 0，且都等于上一行。

所以：**训练时不要用 `action` 的 mask 去筛行**，那会把完全正常的样本丢掉。
它的用途是「想知道指令的真实下发时刻」时才查。

**唯一的例外**：某个空间从头到尾一条指令都没有（本批的 `action/joint_space`），
那么 mask 全 0 且数值全是 `NaN`。判断方式是看整列而不是单行：

```python
if np.isnan(f['action/joint_space/position'][:]).all():
    ...   # 这个空间没有数据，别用
```

各空间的 mask 是**独立**的，某一行可能关节有效而末端无效。

---

## 7. 视频

- H.264 / mp4，**30 fps**，和 h5 行一一对应。
- 默认导出高度 360（`--video-height 0` 可以保持原始分辨率）。实际尺寸解一帧就知道。
- 某一刻相机没画面时，**重复上一帧**，不插黑帧 —— 黑帧是训练时一眼看不出的脏数据。
  这类帧在导出日志里以「重复帧 N」计数。
- 腕相机画面里烧着一行日期时间。相机固件关不掉（`DeleteOSD` 是空壳、
  `SetOSD` 不让改类型），所以改成**让它显示正确的时间**（已对到机载 NTP）。
  介意的话在训练时裁掉那一条，不要指望画面里没有它。

---

## 8. 索引文件

`episode/<name>.json` 是单条，四个键：

```jsonc
{"episodes": [{
  "episode_id":  "00000001-0000",  // 前半段 = 文件名那个序号，后半段 = 文件内第几段
  "start_frame": 0,
  "end_frame":   622,              // 闭区间，帧数 = end - start + 1
  "instruction": ["Pick up the black square box with the left arm"]
}]}
```

`episodes_all.json` 是全部，每条多一个 `episode_name`（不带扩展名，`data/` 与三个
`video_*/` 都用它）。单条里不给，因为**文件名逐字就是它** —— 对面的样例也是这么分的。

**这四个键是 `episodefile_template.json` 和对面样例的交集。** 模板里那个 `label`、
样例里那个 `instruction_eval` 都只有一边有，我们两个都不给：没有独立的评测说法集，
`label` 的内容也全都能从文件名和 `meta.json` 里推出来。采集现场的标注（物品编号、
指令的结构化拆分、lint 告警、绝对时刻、裁剪痕迹）**一律不导出** —— 它们是采集端
自己的记录，在 session 里原样存着，用 `tools/inspect_session.py` 看。视频路径也不给：
三路都是 `video_<camera>/<episode_name>.mp4`，拼得出来。

`instruction` 是**列表**：规范允许一条 episode 有多种说法，我们目前每条只有一句，
按单元素列表给。

`episodes_all.h5` 是同一份索引的 h5 版，四个 dataset：`episode_id`、`start_frame`、
`end_frame`、`instruction`。对面样例里有同名文件（他们那份还多一个
`instruction_eval`）。指令是 `(N, K)` 的定宽表 —— h5 存不了锯齿数组。

> `episodes_all.*` 里没有一个字节是新的：`episode_name` 就是 `episode/` 下的文件名，
> 其余字段逐条拼起来即可。留着它们是因为**对面的样例里就有这两个文件**，
> 删掉等于单方面改交付布局。

**`episode_id` 的前半段必须和文件名的序号逐字相同**，不然拿到 id 找不回文件；
后半段是「该文件内第几段」，我们一个 h5 只放一条 episode，所以恒为 `0000`。

---

## 9. `meta.json` 里值得看的几项

| 字段 | 说明 |
| --- | --- |
| `format` / `format_version` | `"YB"` / `"0.1"`，认这个而不是靠目录结构猜 |
| `dataset_name` | 单次采集就是那个采集批次；合并多次时是 `首--尾 (N sessions)` |
| `sampling.hz` | 30，h5 和视频同一个值 |
| `world_frame` | §5.1 那套，附实测值 |
| `camera_space.extrinsic_formula` | §5.2 那一行公式 |
| `ee_convention` | §5.3 的 `R_fix`，逐臂给 |
| `robot.gripper` | 归一化前后的对应关系 |
| `scale` | episode 条数与总时长（小时），**是掐完的时长**，合并的话是全部采集之和 |

h5 里 `meta/end_space/fk_provenance` 记着算末端位姿用的 URDF 的 sha256，以及这次采集
自带的 `camera_params.yaml` 的 sha256。
两次导出结果对不上时先比这个。**合并多次采集时每条 episode 记的是自己那份** ——
相机被碰过的前后两批可以合成一份 dataset，各用各的外参。

h5 的 `meta` attrs 只有三个：`version`、`gripper_unified="v1"`（夹爪归一化约定的版号，
就是 §0 那条 0=张开 1=夹紧）、`patches`（对面用来记「这份数据事后打过哪些补丁」，
我们没打过，是空表）。**导出参数不写进 h5**，说明性的约定统一在 `meta.json` 里给。

---

## 10. 已知限制

- **`action/joint_space` 是 NaN**（见 §4.1）。
- **图像不在 h5 里**，只有 mp4 + `frame_index`。对齐关系已经钉死，
  以后要塞进 h5 只是补取像素这一步。
- **腕相机内参是 1920×1080 档**，取自这次采集自带的 `camera_params.yaml`（里面只留
  当时实际录制的那一档）。头部相机是出厂值，不是现场标的。
- **标定精度**：640×360 重投影 RMS 1.74 / 1.57 px，外参残差 5.85 / 6.87 mm；
  左右腕相机的 z 相差 15 mm，尽管两侧安装件相同。要求更高的话得重标。
- 相机管线延迟按固定值 110 ms 补偿（腕）、0 ms（头）。这个值是靠运动相关性标的，
  与 OSD 时间戳法测出的 234 ms 有约 80 ms 的分歧，尚未定论。
- **合并多次采集时，`meta.json` 里的内参出处只报第一次采集的**（h5 逐条用的仍是各自
  采集当时那份）。几次采集之间重标过相机的话，以 h5 里 `meta/camera_space/intrinsic`
  为准。`head_optical` 不一致时导出直接拒绝，不会悄悄按第一次的算。
