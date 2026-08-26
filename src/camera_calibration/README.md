# camera_calibration

G1 三个相机的内参标定，和两个腕相机相对手臂 link 的外参标定。一个 dashboard 跑完两阶段。

本包只负责**拍照和算**。机器人本体、头部相机、腕相机都由各自的 launch 起，
手臂姿态你自己遥操，这里不动电机。

```bash
ros2 launch camera_calibration calibration.launch.py
# 打开 http://<机器>:8300
```

---

## 标定板

参数全在 `config/board.yaml`，代码里不写死任何尺寸。

```yaml
board:
  squares_x: 12
  squares_y: 9
  square_size: 0.030
  marker_size: 0.0225
  dictionary: DICT_5X5_100
```

**字典不能随便填**。12×9 = 108 格意味着 54 个 marker，`DICT_5X5_50` 只有 50 个，装不下，
`Board` 会直接抛错而不是悄悄少认几个。

拿不准实物板的字典和朝向，就在 dashboard 上点**探测板参数**：拍一张遍历所有候选。
判据是**共面残差**，不是角点数 —— 朝向猜错时角点数一样多，只有残差能区分。
下面是拿头部相机对着实物板测出来的：

| 字典 | 格数 | 角点 | 共面残差 |
|---|---|---|---|
| DICT_5X5_100 | **12×9** | 88/88 | **2.21 px** |
| DICT_5X5_100 | 9×12 | 84/88 | 100.14 px |

板是平面、成像是投影，所以正确的 id 映射下单应几乎是精确的，只剩镜头畸变那几个像素。
映射错了残差立刻上到几十上百像素。

`DICT_5X5_100` / `250` / `1000` 的残差完全相同（后者是前者的超集），取能装下 54 个 marker
的最小那个。

核对实物：`ros2 run camera_calibration make_board --dpi 300 --out /tmp/board.png`，
打印时**关掉缩放**，打完拿尺子量一格。尺寸填错的话标定照样跑完、内参看着也正常，
只有涉及米的量（外参平移）会整体差一个比例。

---

## 第一阶段：内参

每个相机的**每个分辨率档位单独标**，因为 —— 

### 换分辨率内参会跟着变，而且不一定是等比缩放

判据是看 `fx`：

- **fx 按分辨率比例变** = 缩放。FOV 不变，K 可以整体换算，畸变系数不动。
- **fx 一点不变、只有主点动** = 传感器裁剪。FOV 变小，只能平移 cx/cy，**按比例缩放是错的**。

头部 D435i 两种情况都有（`head_sensors/README.md` 里的实测表）：

| 档位 | fx | cx | HFOV | 与 848×480 的关系 |
|---|---|---|---|---|
| 424×240 | 304.226 | 215.043 | 69.74° | 缩放（fx 正好减半）|
| 640×480 | 608.451 | 326.086 | 55.48° | 裁剪（fx 不变，cx 差 104 =(848−640)/2）|
| 848×480 | 608.451 | 430.086 | 69.74° | — |

所以 `storage.find_intrinsic()` 只做**精确匹配**；只有实测判定为缩放关系并写进
`profile_relations` 的档位之间才允许换算，其余一律返回 `None`，宁可不发 camera_info
也不发一组悄悄错掉的。

判定方法：两个档位都标完并保存后，调 `/api/relate`，它比较 `fx_low` 和 `fx_high × ratio`。

### 要标的档位

| 相机 | 档位 | 说明 |
|---|---|---|
| 头部 D435i | 1920×1080 / 1280×720 / 960×540 / 848×480 / 640×480 / 640×360 / 424×240 / 320×240 / 320×180 | `rs-enumerate-devices` 实测的全部彩色流原生档位。帧率不影响内参，统一挂 30 |
| 腕（左/右）| 1920×1080 | stream0，h264@30。`record` 采集用的就是它，`-c copy` 直存 |
| 腕（左/右）| 640×360 | stream1，hevc@30。`camera_node` 默认拉的 |

腕相机硬件**只有这两路 RTSP 流**（stream2 返 404、stream3 超时，探过了），而且**是两路独立的流**，
不是同一路的缩放，所以必须分别标、再用「判定档位关系」实测它俩到底是不是缩放关系。

`camera_node` 的 `image_width`/`image_height` 能让 ffmpeg 缩到任意尺寸，但**缩放档不标**。
真跑在缩放尺寸上时（比如 `image_height:=240` 出来的 426×240），`camera_node` 找不到对应内参，
只 warn 一次、不发 `camera_info` —— 不会给出一组假的。要 `camera_info` 就用原生档位跑。

### 切档位

dashboard 上每个相机有档位下拉 + 「切换」按钮，切完会**等到真收到对应尺寸的图**才返回，
不会拿旧分辨率的残帧去拍。

- 腕相机：改 `camera_node` 的 `rtsp_url`（宽高给 0，用流的原生尺寸）
- 头部：**先 `enable_color=false`，再改 `rgb_camera.color_profile`，再 `enable_color=true`**。
  实测直接改 `color_profile` 会返回「设置成功」但传感器根本不重开，出图尺寸一直不变 ——
  静默失败。必须关掉彩色流才会真的重建。

### 怎么拍

板子在画面里换角度、换远近，**务必也放到四个角上** —— 畸变系数全靠边缘的点约束，
都堆在中间是标不出来的。每档位至少 15 张。

- 合格线：RMS < 0.5 px（1080p）/ < 0.3 px（640×360、424×240）
- 「整体覆盖」是所有视图角点合起来的凸包占画面的比例，低于 60% 说明还没铺开
- 「删离群」按重投影误差挑出明显偏高的张（多半拍糊了），删掉重解
- 头部标完会自动和 `camera_info` 里的**出厂内参**并排对比。fx/fy 差 > 1% 或
  cx/cy 差 > 3 px 说明流程有问题，先怀疑板子尺寸填错，别直接采信自己标的值

---

## 第二阶段：外参

把板固定在三个相机都看得见的地方，**整个过程别动板子**。换若干个手臂姿态，每次停稳后采一组。

### 三条独立的路子，都算一遍再对比

**联合最小二乘**（默认，板可动，**且不依赖头部外参准不准**）。把所有组一起做 LM，
未知量是 X 和头部外参的偏差 ΔH：

```
min over (X, ΔH)   Σᵢ ‖ log( (T_base_ref·ΔH·T_ref_boardᵢ)⁻¹ · (T_base_linkᵢ·X·T_cam_boardᵢ) ) ‖²
```

板每组挪到哪都行 —— 头部每组各测一次板位姿，板的位置不进未知量。至少 2 组。

**把 ΔH 放开之后，头部外参再不准也不影响 X。** 约束可以写成 `ΔH = Cᵢ·X·Dᵢ`
（`Cᵢ = T_base_ref⁻¹·T_base_linkᵢ`，`Dᵢ = M ʷᵢ(M ʰᵢ)⁻¹`），两组相除后 `T_base_ref` 被约掉，
剩下 `(T_base_link_j⁻¹ T_base_link_i)·X = X·(…)` —— 又一个不含头部外参的 AX=XB。合成数据实测：

| 头部实际偏差 | 参考相机法 | 联合(信头部) | 联合(放开 ΔH) | ΔH 估计 |
|---|---|---|---|---|
| 1° | 0.900° / 5.4 mm | 0.846° / 5.5 mm | **1.1e-3°** | 1.000° |
| 3° | 2.701° / 16.3 mm | 2.540° / 16.6 mm | **1.1e-3°** | 3.000° |
| 10° | 9.006° / 54.5 mm | 8.492° / 55.4 mm | **0** | 10.000° |

代价是它继承了 AX=XB 的那个前提：**各姿态之间的转轴不能都平行**，否则 12 个未知量撑不起来。
代码算 Jacobian 的条件数并报 `well_posed`，撑不起来时会明说别当真（`test_joint_flags_parallel_rotation_axes_as_ill_posed`）。

头部仍然不可少 —— 它是唯一能每组把板位姿钉死的东西，只是它**准不准**不再重要。

**注意别把它当成"精度更高"**：只估 X 的话，位姿空间等权最小二乘数学上就是在求均值，实测和
逐组平均差不到 2%（`test_joint_matches_averaging_when_only_solving_x` 就是钉这条的）。
要在精度上真的赢过平均，得把残差写成像素重投影，那需要把角点也存进 meta。

**参考相机法**（板可动，但**头部偏多少它就偏多少**）。逐组各解一个再取平均：

```
T_link_cam = T_base_link⁻¹ · T_base_ref · T_ref_board · T_cam_board⁻¹
```

一张就能解，但结果里带着头部外参的全部误差。

**AX=XB 手眼**（`cv2.calibrateHandEye`，**要求板全程不动**）。只用腕相机自己看板 + 手臂 FK，
**完全不碰头部外参**，所以能拿来独立验证头部。

```
T_base_link_i · X · T_cam_board_i = T_base_board = 常量
```

推导第一步就用了"板在 base 下不动"。板每挪一次就多 6 个未知量、也只多 6 个方程 ——
**永远欠定**。所以代码会先算板在各组之间的位姿差（`board_stability`），超过 20 mm / 3°
就直接拒绝出解，而不是安静地给个错的。另外还要求至少 3 个姿态且**姿态之间手腕的转轴不能都平行**
（都绕同一根轴转的话旋转部分欠定，代码里做了秩检查）。

**结论**：想要交叉验证就把板固定住采一轮 —— 那时三条路都能跑。板动着也能标，
但只剩联合解和参考相机法，且头部是否可信只能靠联合解里的 ΔH。

### 怎么采

三路实时画面带检测叠加就在页面上 —— 板子摆得对不对、有没有被夹爪挡住、离得够不够近，
看图比看角点数直观。

解板位姿用的内参按这个顺序取：

1. `config/calibration.yaml` 里**当前档位**的标定值
2. 取不到时，退回相机自己发的 `camera_info`（**只有头部 RealSense 有**，且分辨率要对得上）
3. 还没有就拒绝采集，说清楚缺哪个

所以头部即使没走完第一阶段也能直接开始标外参，用的是 D435i 的出厂内参。用了哪一种会写进
`meta.json` 的 `intrinsic_source`，页面上的姿态列表和采集前检查也都标出来，事后能查。
腕相机没有 `camera_info`，没标就是没标，不会编一个出来。

### 质量指标

- **板位姿残差**：拿候选外参反推板在 `torso_link` 下的位姿。板没动过，所以这些值该重合。
  它们的离散度就是这组外参的实测残差 —— 两种方法可以直接比这个数。目标 < 5 mm rms。
- **各姿态解的离散**：参考相机法每个姿态单独解一次，离散度 > 1° 说明有问题。
  注意这个信号**只有在手臂姿态拉得开时才存在**，只拍一张的话根本看不出来。
- **头部定板抖动**：头部单独看板的位姿在各次采集之间的抖动，反映头部 PnP 噪声。
- 两法差异合格线：< 2° 且 < 10 mm。

### 为什么拍照时必须手臂静止

`camera_node` 打的是**收到帧的时刻**，不是曝光时刻；RTSP 还叠着编解码和网络延迟。
动着拍出来的图，和同一时刻查到的 TF 根本不是一回事。

所以 dashboard 盯着 `/joint_states`：所有关节速度都低于 `max_joint_speed`（默认 0.1 rad/s）
并持续 `settle_seconds`（默认 0.5 s）才放行拍照，否则按钮给出具体原因。阈值在
`config/cameras.yaml` 的 `motion_gate` 里。

阈值不是越小越好：手臂真停住时速度读数本身就在 0.02~0.05 rad/s 上下跳，卡到 0.01
会永远放不了行。真在动的时候远不止 0.1。

---

## 结果怎么用

标定结果写 `config/calibration.yaml`（dashboard 上显示实际写入的绝对路径）。

```yaml
intrinsics:
  camera_left:
    - {width: 1920, height: 1080, camera_matrix: [...], distortion_coefficients: [...], rms: 0.31, ...}
extrinsics:
  camera_left: {parent: left_gripper_base, child: camera_left, translation: [...], rotation: [...], ...}
profile_relations:
  camera_left:
    - {from: [1920, 1080], to: [640, 360], kind: scale, ...}
```

**内参** —— 给 `camera_node` 传 `calib_file`，它会按当前输出分辨率挑对应条目，
和 `Image` 同 header 发 `~/camera_info`：

```bash
ros2 launch camera_node camera.launch.py \
  calib_file:=/workspace/src/camera_calibration/config/calibration.yaml
```

挑不到对应档位时只 warn 一次，**不发**。发一组全零的假内参下游是看不出来的。
改 `calib_file` 会重新加载但不重开流（它不在 `_STREAM_PARAMS` 里）。

**外参** —— 三个相机的 TF **全部由 URDF 出**，不发 static TF。同一个 child frame
两个 publisher，tf2 不保证取哪个。保存时写进 `calibration.yaml` 的 `urdf_overrides`，
控制栈展开 URDF 时打进内存 DOM：

```bash
ros2 launch unitree_g1_ros2_control control.launch.py   # use_camera_calibration 默认 true
ros2 run tf2_ros tf2_echo left_camera_mount_link camera_left
```

**不改 `unitree_g1_description/model/final.urdf`** —— 那是 submodule，改了会和上游
分叉，submodule 一更新就没了。不想用标定值就 `use_camera_calibration:=false`。

三个相机在 URDF 里的处境不一样，覆盖方式也不同：

| 相机 | URDF 里已有什么 | 怎么落地 |
| --- | --- | --- |
| 头部 | `d435_joint: torso_link → d435_link`，光心那段由 realsense-ros 发 | 只改 `d435_joint` 的 origin |
| 左/右腕 | 只有 `left/right_camera_mount_link`（支架的**可视化模型**），没有光心 link | 新插一条 `mount_link → camera_left/right` |

头部的修正量估的是 `T_base←optical`，而 URDF 里能改的是 `T_parent←mount`，中间
`mount → optical` 是相机自己的几何，不动。所以要共轭过去：

$$T_{\text{parent}\leftarrow\text{mount}}^{\text{new}} = T_{\text{parent}\leftarrow\text{mount}} \cdot M \cdot \Delta H \cdot M^{-1}, \quad M = T_{\text{mount}\leftarrow\text{optical}}$$

`cameras.yaml` 里每台相机声明 `mount_joint` / `mount_parent`（腕部再加 `mount_create: true`）。
`mount_parent` 必须和 URDF 里该关节的 `<parent link>` 一致 —— 存的是
`T_parent←child`，写错了会静静存一个差一段变换的 origin。launch 侧会校验，
对不上就 warn 跳过。

`child` 必须等于 `camera_node` 的 `frame_id` 参数（默认 = 节点名，即 `camera_left`），
否则图像的 frame_id 和 TF 对不上。

不跑控制栈时可以用 `calib_tf_node` 发 static TF 顶一下。它默认
`skip_urdf_overrides:=true`，会跳过已经进了 `urdf_overrides` 的 frame，免得和
`robot_state_publisher` 抢；要单独发就设成 `false`。

### 保存完必须重启控制栈

`control.launch.py` **只在启动那一刻读一次** `calibration.yaml`，之后不再看。
控制栈先起、标定后存，那份 `/robot_description` 里就没有相机 —— 表现是 8200/8180
页面和 rviz 里**找不到 `camera_left` / `camera_right`，`d435_joint` 还是名义值**。

```bash
LP=$(pgrep -f 'ros2 launch robot_bringup all_data' | head -1)
kill -INT -- -"$(ps -o pgid= -p "$LP" | tr -d ' ')"
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual
```

停控制栈**绝不能 `pkill`**，也不能只把 SIGINT 发给 launch 进程（它不转发），
原因见 [`robot_bringup/README.md`](../robot_bringup/README.md)。

一眼确认现在跑的是哪一版：

```bash
ros2 param get /robot_state_publisher robot_description | grep -oE 'link name="camera_[a-z]+"'
```

---

## 采集数据

默认在 `~/camera_calib_data/`，可以整个拷走离线重跑：

```
intrinsic/<相机>/<宽x高>/0001.png + 0001.json   # json 里是检测好的角点
extrinsic/pose_001/{head,camera_left,camera_right}.png + meta.json
```

图一律存 PNG。JPEG 的块效应会把角点推偏零点几个像素，而标定的全部精度就在这零点几个像素上。

编号取已有最大值 +1，不是文件数 +1 —— 后者在删掉中间某一张之后会撞名，新图直接盖掉旧的。

---

## 实现上的几个决定

**没用参考仓库 `camera_calibration_toolkit` 做 submodule。** 它的 requirements 写死
`opencv-contrib-python>=4.8.0`，用的是 `CharucoDetector` 那套新接口；系统装的是
Ubuntu 22.04 自带的 **4.5.4**，只有 `CharucoBoard_create` / `interpolateCornersCharuco`
这套旧接口，两边不兼容。在 Jetson aarch64 上 pip 装 4.8 又要和 cv_bridge、realsense 抢 so。
所有 `cv2.aruco` 调用收敛在 `board.py` 一个文件里，将来换 OpenCV 只改这一处。

**没用 cv_bridge。** 它内部对 `uint8[]` 逐元素处理，1080p 一帧要几百毫秒。
直接 `np.frombuffer` 按 `step` 切片。注意**头部 RealSense 出 rgb8、腕相机出 bgr8**，
`image_to_bgr()` 按 encoding 分支，弄反了颜色是错的。

**检测跑在独立线程里**，HTTP 请求只取缓存。1080p 上 ChArUco 检测要上百毫秒，
放进请求里会把页面卡住；而且 `/api/state` 里的"看得见板/角点数"本来就要用这份结果。

**板坐标系的 y 轴在板面上朝上、+z 指向观察者**，所以正对着板拍时 `T_cam←board` 的
`R[2][2]` 约等于 −1，不是 +1。看数别以为解错了。

---

## 测试

```bash
cd src/camera_calibration && python3 -m pytest test -q
```

不需要机器人。合成一张"拍出来"的板图跑通检测→位姿；用注入的真值验证两种外参解法；
其中一条专门验证「头部外参偏 3° 时，参考相机法跟着偏而 AX=XB 不受影响」。
