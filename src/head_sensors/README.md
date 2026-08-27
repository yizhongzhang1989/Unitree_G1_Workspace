# head_sensors

G1 头部两个传感器的调用层：头顶 Livox MID-360 雷达 + 头顶 RealSense D435i 深度相机。

两者的接入方式完全不同，这是本包存在的理由：

| 传感器 | 位置 | 接法 | 谁提供驱动 |
|---|---|---|---|
| Livox MID-360 | 头顶，URDF `mid360_link` | 以太网 192.168.123.120 | **机器人内部的 `lidar_driver` 服务**，直接发 DDS 标准话题；本机不装 Livox SDK |
| RealSense D435i | 头顶，URDF `d435_link` | USB 直连本机 NX | **官方 `realsense2_camera`**（apt: `ros-humble-realsense2-camera`） |

官方文档：<https://support.unitree.com/home/zh/G1_developer/depth_camera_instruction>

## 依赖
驱动已写进 [`.devcontainer/Dockerfile`](../../.devcontainer/Dockerfile)，重建镜像即可：
```
ros-humble-librealsense2           2.58.x
ros-humble-realsense2-camera       4.58.x
ros-humble-realsense2-camera-msgs
ros-humble-realsense2-description
```

容器是 `privileged` + `/dev:/dev`，USB 直通已经有了，不需要额外的 udev 规则。

## 运行
每个终端先 `source scripts/env.sh`。
```bash
ros2 launch head_sensors head_sensors.launch.py                  # 雷达 + 相机
ros2 launch head_sensors head_sensors.launch.py camera:=false    # 只要雷达
ros2 launch head_sensors head_camera.launch.py                   # 只要相机
ros2 launch head_sensors head_camera.launch.py pointcloud:=true align_depth:=true
```

RViz 里要让点云/图像挂到机器人模型上，需要整机 TF，另起一个终端：
```bash
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual
```

### 校准头部安装角
头是可转的，撞歪了就这么校：**跑 `verify_head_view` 看轮廓 → 手动转头 → 再跑一次**，直到轮廓贴合。四个终端，前三个一直开着：
```bash
# 1. 整机栈 —— verify_head_view 要 /robot_description 和 /joint_states
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual
# 2. 头部相机 —— 同时提供 d435_link -> camera_color_optical_frame 的 TF
ros2 launch head_sensors head_camera.launch.py
# 3. 让手臂进画。手推到任意姿态，不进画就没东西可比
ros2 launch robot_bringup gravity_float_demo.launch.py
# 4. 拍照对照。转一次头跑一次
ros2 run head_sensors verify_head_view --out /tmp/verify
```

看 `/tmp/verify_overlay.png`：**左臂绿、右臂红**的 URDF 轮廓压在实拍图上，对不齐就是头歪了。

## 雷达
原始话题（机器人固件发布，本机不用装任何驱动）：

| 话题 | 类型 | 实测 |
|---|---|---|
| `/utlidar/cloud_livox_mid360` | `sensor_msgs/PointCloud2` | 9.98 Hz，19968 点/帧，`frame_id=livox_frame` |
| `/utlidar/imu_livox_mid360` | `sensor_msgs/Imu` | 200 Hz，`frame_id=livox_frame` |

点云布局 `point_step=22`：`x,y,z,intensity`(float32) + `ring`(uint16 @16) + `time`(float32 @18)。

直接用原始话题有三个坑，`head_lidar_node` 负责补上：
1. **TF 断了**：`frame_id` 是 `livox_frame`，URDF 里叫 `mid360_link`，两者之间没有变换。发一条恒等静态 TF `mid360_link → livox_frame`，点云才挂得到机器人模型上。
2. **一半点是废的**：每帧约 55% 是无回波的 `(0,0,0)` 占位（实测 10960/19968），但 `is_dense=true`。按 `min_range`/`max_range` 过滤后发到 `/head/lidar/points`。
3. **IMU 单位是 g**：不是 m/s²（静止实测模长 ≈ 0.99），`orientation` 全零也没标无效。换算成 m/s² 并按 REP-145 把 `orientation_covariance[0]` 置 -1，发到 `/head/lidar/imu`。

MID-360 在 URDF 里是**倒装**的（`mid360_link` 相对 `torso_link` 的 `rpy` roll = π，pitch = 0.0511），IMU 静止读到 `az ≈ -0.97 g` 与之吻合。

点云出**两路**，同样的距离过滤、只差打包布局：

| 话题 | `point_step` | 字段 | 给谁用 |
|---|---|---|---|
| `/head/lidar/points` | 16 | `x,y,z,intensity` | 只要坐标的下游 |
| `/head/lidar/points_full` | 22 | 再加 `ring` + `time` | 激光惯性里程计（`g1_localization`） |

里程计靠逐点 `time` 做运动去畸变，瘦身布局喂不了它。两路都只在**有订阅者时**才打包，
没人订就只走统计，所以默认两路全开也不额外花钱。

> **`PointCloud2.data` 只能用 `array.array('B', ...)` 赋值。**
> rclpy 对 `uint8[]` 只对 `array.array` 短路，赋 `bytes` 会走逐元素 `isinstance` 断言 ——
> 实测一帧 9500 点（209 KB）**29.9 ms vs 0.044 ms，678 倍**，10 Hz 下就是 30% 单核，
> 还会把同节点里 200 Hz 的 IMU 回调饿死（实测掉到 164 Hz）。`pointcloud.py` 的
> `_as_payload()` 统一处理，别绕过它。点云与 IMU 另外分在不同的回调组 +
> `MultiThreadedExecutor`，两件事都做了 IMU 才回到 200 Hz。

在自己的节点里用：
```python
from head_sensors.pointcloud import cloud_to_xyzi, filter_range

xyzi = cloud_to_xyzi(msg)            # (N, 4) float32，零拷贝解析
pts, dist = filter_range(xyzi, 0.1, 70.0)
```

要保留 `ring`/`time` 就走 `cloud_to_structured` + `range_mask` + `repack_like`。

`cloud_to_xyzi` 按 `msg.fields` 现场构造 numpy dtype，不写死 22 字节布局，换固件也不用改。

实测的雷达视野（2026-08-25，见 `g1_localization` 的 README）：
**水平只有 285° 有回波**，正前方 `az ∈ [-45°, +45°]` 是盲区（推测头部外壳遮挡）；
垂直向上 6.2° ~ 向下 53.8°，与 MID-360 标称的 -7°~52° 倒装后吻合。
近处 0~0.3 m 有 6.14% 的点是雷达自己所在的头部外壳（平均距离 0.154 m）。

## 相机
`head_camera.launch.py` 只做两件事：转发参数给官方 `rs_launch.py`，再补一条挂载 TF。

话题（`camera_namespace:=head`、`camera_name:=camera`）：

| 话题 | 说明 |
|---|---|
| `/head/camera/color/image_raw` + `camera_info` | RGB，默认 424x240x30 |
| `/head/camera/depth/image_rect_raw` + `camera_info` | 深度 Z16，默认 424x240x30 |
| `/head/camera/aligned_depth_to_color/image_raw` | `align_depth:=true` 时才有 |
| `/head/camera/depth/color/points` | `pointcloud:=true` 时才有 |

`color_profile` / `depth_profile` 的格式是 `宽 x 高 x 帧率`，第三段是 fps。默认 **424x240x30**。

### 换分辨率时内参必须跟着换
`fx` `fy` 是焦距（单位**像素**），`cx` `cy` 是主点（光轴穿过成像面的位置，不等于画面几何中心，实测偏 6~8 px）。FOV 不是独立参数，是 `2·atan(w / 2fx)` 算出来的。出厂标定的彩色档位（`rs-enumerate-devices -c` 读出）：

| 分辨率 | fx | fy | cx | cy | 水平 FOV |
|---|---|---|---|---|---|
| **424x240** | **304.226** | **304.385** | **215.043** | **123.774** | **69.74°** |
| 640x360 | 456.338 | 456.578 | 324.565 | 185.661 | 70.07° |
| 640x480 | 608.451 | 608.771 | 326.086 | 247.547 | 55.48° |
| 848x480 | 608.451 | 608.771 | 430.086 | 247.547 | 69.74° |
| 960x540 | 684.508 | 684.867 | 486.847 | 278.491 | 70.07° |
| 1280x720 | 912.677 | 913.156 | 649.129 | 371.321 | 70.07° |
| 1920x1080 | 1369.015 | 1369.734 | 973.694 | 556.982 | 70.08° |

加粗行是当前默认，垂直 FOV 全档恒定 43.03°。

**别按比例缩放这些数**。对比 `640x480` 和 `848x480`：`fx` `fy` `cy` 一模一样，只有 `cx` 差 104 px，正好是 `(848-640)/2` —— 4:3 那两档是从 16:9 传感器**横向裁**出来的，不是缩放，水平 FOV 因此从 69.74° 掉到 55.48°（1 m 处横向可视范围 1.39 m → 1.05 m，少 25%）。判断办法就看 `fx`：不变 = 裁剪，按比例变 = 缩放。

选 `424x240` 正是因为它是 `848x480` 的严格一半：`fx` 608.451 → 304.226、`cx` 430.086 → 215.043 都整除，FOV 一点没变，带宽只剩四分之一。**别改成 `320x240`**，那是裁出来的。

要拿当前实际值：
```bash
ros2 topic echo /head/camera/color/camera_info --once
```
`k` 那个 9 元素数组的第 0、4、2、5 项就是 `fx fy cx cy`。

**挂载 TF**：驱动只发相机自己那棵子树（`camera_link → camera_*_optical_frame`），不知道相机装在机器人哪儿。launch 里补一条 `d435_link → camera_link`，默认恒等 —— 两者确实同为 x 前 / y 左 / z 上，朝向没问题。`d435_link` 相对 `torso_link` 是 `xyz 0.0576235 0.01753 0.42987`、pitch `0.8307767` rad（低头 47.6°）。

但**别把 `camera_link` 当成成像中心**：彩色镜头在 `camera_color_optical_frame`，相对 `d435_link` 偏 `[-0.8 +15.3 0.0] mm`，要投影就得从 TF 取这一段，不能只用理想的光学旋转。

### 接线：脑袋后那个 Type-C 就是相机口
D435i 不是内部走线，需要用一根 Type-C 线把**头部后方的 Type-C** 接到机器人顶部电气接口板上的 USB host 口。没接线时 `lsusb` 里根本不会出现 `8086:0b3a`，装什么驱动都没用。

**一定要确认协商到的是 USB3**（看驱动日志的 `Device USB type`）。实测同一台机器两种接法：

| 链路 | sysfs speed | 可用档位 |
|---|---|---|
| USB 2.1 | 480 Mbps | 彩色最高只能 `640x480x15`；`848x480x30` 被判 invalid 并回落 |
| USB 3.2 | 5000 Mbps | 彩色/深度 `848x480x30` 双流稳定满帧 |

现在默认的 `424x240x30` 带宽只有 848x480 的四分之一，USB2 下也跑得动。

USB2 接法下把两路都降到 15 fps 才不会互相抢带宽（实测彩色 14.1 Hz、深度 15.0 Hz）：
```bash
ros2 launch head_sensors head_camera.launch.py color_profile:=640x480x15 depth_profile:=640x480x15
```

### 别用 `ros2 topic hz` 判断相机掉没掉帧
`image_raw` 一帧 848x480x3 ≈ 1.2 MB，30 fps 就是 36 MB/s（当时的默认档位）。Python 写的 `ros2 topic hz` 订阅端根本吃不下，会报出一个偏低的假帧率。同一时刻实测：
```
/head/camera/color/camera_info   29.96 Hz   min 0.032 max 0.034   ← 传感器真实出帧
/head/camera/color/image_raw     22.59 Hz   min 0.031 max 0.537   ← 订阅端掉的
```

要判断传感器有没有掉帧，看**同一路的 `camera_info`**（几十字节，不受传输影响）。

### 深度要和彩色比对时必须开 `align_depth`
`depth/image_rect_raw` 和 `color/image_raw` 分辨率相同，很容易以为能直接逐像素相减。**不能** —— 它们是两颗不同的传感器（下表是 848x480 档位下测的，换档位同理）：

| 流 | 内参 | FOV | frame |
|---|---|---|---|
| color | fx=608.5 cx=430.1 | 69.7° x 43.0° | `camera_color_optical_frame` |
| depth | fx=431.5 cx=420.2 | 89.0° x 58.2° | `camera_depth_optical_frame` |

直接相减实测得到 -0.854 m 的假中位差（拿夹爪的深度去减地板的深度）。要比对就开 `align_depth:=true`，用 `/head/camera/aligned_depth_to_color/image_raw`。

**相机 IMU** 默认关（`enable_gyro`/`enable_accel`）——机器人本体已经有盆骨和躯干两个 IMU。

## 实机验证挂载 TF
`verify_head_view` 拍一张实拍图 + 读当前关节角 → URDF 拍照 → 把渲染轮廓叠上去。命令见上面「校准头部安装角」。

| 决定 | 为什么 |
|---|---|
| **不摆姿势** | 摆姿势已经有 `gravity_float_demo`，重新发明一遍等于多一份要维护的控制代码和安全面 |
| 模型取自 **`/robot_description`** 而不是磁盘 | 广播那份才是机器人实际在跑的模型。磁盘 `final.urdf` 写的是相对 mesh 路径，广播的是 `package://`，两份不是同一个文件。收不到广播直接报错退出 |
| 关节角在窗口内取均值 | **不是为了降噪** —— 静止时 `/joint_states` 标准差实测只有 0.00002 rad，平均毫无意义。窗口是用来发现「拍照那一刻手臂还在动」，峰峰值超 `--still` 就警告 |
| 外参从 **TF** 读 `d435_link -> camera_color_optical_frame` | 彩色镜头不在 `d435_link` 原点上，实测偏 `[-0.8 +15.3 0.0] mm`。当成纯旋转会让渲染整体横移十几个像素 |
| **只出一张 `_overlay.png`** | 要判的就是「轮廓贴不贴得上」。深度图、部件图、原图都是中间产物，真要它们直接用 `render_head_view` |

## 从 URDF 渲染相机视角
回答「相机在某个姿态下能看到自己身体的哪些部分」。**不需要仿真器** —— MuJoCo / PyBullet 的价值在动力学积分，这里一步物理都不用算，只是 FK + 投影 + 光栅化。

| 文件 | 依赖 | 职责 |
|---|---|---|
| `urdf_view.py` | pinocchio / numpy / opencv | 渲染全部能力，核心是 `shoot(关节角) -> Shot` |
| `render_head_view.py` | 同上 | 薄 CLI |
| `verify_head_view.py` | **rclpy 等** | 实物对照，本包唯一碰 ROS 的文件 |

前两个拷走就能跑，已实测 `env -i` 剥掉所有环境变量、只给 pinocchio 的 `PYTHONPATH` 照样出图：
```bash
cp urdf_view.py render_head_view.py /somewhere/
python3 render_head_view.py --urdf final.urdf --pose shot.json --out /tmp/hv
```

`--urdf` 必填，mesh 路径按 URDF 所在目录解析（G1 写的是 `g1_description/meshes/x.STL`，所以交付时要把 `model/` 整个带上）。`--pose` 是姿态 JSON（字段见 `render_head_view.py` 模块文档），没列到的关节保持中立位；不给 `--pose` 就全中立位加默认内参。换个视角：`--frame mid360_link --hfov 120 --vfov 90 --width 640 --height 480`。

在代码里就直接调函数：
```python
from urdf_view import shoot

shot = shoot({'left_elbow_joint': 0.5}, urdf='final.urdf', out='/tmp/x')
shot.depth   # float32 米，未命中为 inf
shot.label   # 命中的几何体下标，配 shot.names 用
```

输出 `_depth16.png`（16UC1 毫米，与 RealSense 深度同格式，可直接与实拍相减）、`_depth.png`（JET 着色）、`_parts.png`（按几何体上色，看哪个 link 挡住了视野）。中立位下命中 14.6% 像素、深度 0.352~0.532 m —— 视野被自己的双手夹爪占掉约 15%。

### 两条光栅化路径，按要不要逐像素深度选

`render()` 是逐像素 z-buffer，透视正确地插值逆深度；代价是那个逐三角形的 Python
循环（一帧有九万多个三角形能活下来），**640x360 实测 12.4 s/帧**。要深度图就只能走它。

`silhouette()` 只出 `label`，**0.32 s/帧，快约 40 倍**：闭合网格的正面三角形之并就是
它的剪影，所以单个几何体内部不需要比深度，整块交给一次 `cv2.fillPoly`；几何体之间
按最近深度从远到近画家算法覆盖。两者命中像素实测重合 99.7%。只要轮廓的场合
（叠图对齐、自遮挡掩膜）用它；`record` 的 `verify_alignment` 就是这么干的。

相机朝向默认按 **ROS 光学坐标系**（x 右 / y 下 / z 前），加 `--link-frame` 则按 link 坐标系（x 前 / y 左 / z 上）。要往场景里放桌子、材质和光照，那才轮到真正的渲染器。

## 排障
**相机没枚举出来**：`lsusb | grep 8086` 没有 `8086:0b3a` 就是头部那根 Type-C 线没接到顶部接口板，装驱动解决不了。驱动本身是否正常可单独验证：
```bash
ros2 run realsense2_camera realsense2_camera_node --ros-args -p wait_for_device_timeout:=5.0
```

设备不在时它会打 `No RealSense devices were found!`。

**雷达有没有数据**：起 `head_sensors.launch.py`，`head_lidar_node` 每 5 秒会打一行 `点云 10.0 Hz，平均有效点 8997；IMU 200 Hz`。一直不打就是 `/utlidar/*` 没数据。

**两条可以无视的报错**：

| 日志 | 真相 |
|---|---|
| 退出时 `process has died [... exit code -11 ...]` | SIGSEGV，但发生在**收到 SIGINT 之后**的清理阶段。realsense2_camera 4.58 已知问题，不影响已落盘的数据 |
| 启动时 `Could not set param: rgb_camera.power_line_frequency with 3 Range: [0, 2]` | realsense-ros 自己的默认值越界，不影响出图 |

判断相机是否真的起来了，看日志里有没有 `RealSense Node Is Up!` 和两行 `Open profile`。
