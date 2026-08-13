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

在已有容器里临时补装：

```bash
sudo apt-get update && sudo apt-get install -y \
  ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs ros-humble-realsense2-description
```

容器是 `privileged` + `/dev:/dev`，USB 直通已经有了，不需要额外的 udev 规则。

## 运行

```bash
ros2 launch head_sensors head_sensors.launch.py                  # 雷达 + 相机
ros2 launch head_sensors head_sensors.launch.py camera:=false    # 只要雷达
ros2 launch head_sensors head_camera.launch.py pointcloud:=true align_depth:=true
ros2 run head_sensors head_sensors_probe                         # 一次性体检
```

RViz 里要让点云/图像挂到机器人模型上，需要同时有整机 TF，即先起
`ros2 launch robot_bringup all_data.launch.py scope:=whole_body`（提供 `robot_state_publisher`）。

## 雷达

原始话题（机器人固件发布，本机不用装任何驱动）：

| 话题 | 类型 | 实测（2026-08-13） |
|---|---|---|
| `/utlidar/cloud_livox_mid360` | `sensor_msgs/PointCloud2` | 9.98 Hz，19968 点/帧，`frame_id=livox_frame` |
| `/utlidar/imu_livox_mid360` | `sensor_msgs/Imu` | 200 Hz，`frame_id=livox_frame` |

点云布局 `point_step=22`：`x,y,z,intensity`(float32) + `ring`(uint16 @16) + `time`(float32 @18)。

直接用原始话题有三个坑，`head_lidar_node` 负责补上：

1. **TF 断了**：`frame_id` 是 `livox_frame`，URDF 里的链接叫 `mid360_link`，两者之间没有变换。
   节点发一条恒等静态 TF `mid360_link → livox_frame`，点云才挂得到机器人模型上。
2. **一半点是废的**：每帧约 55% 是无回波的 `(0,0,0)` 占位（实测 10960/19968），但 `is_dense=true`。
   节点按 `min_range`/`max_range` 过滤后发到 `/head/lidar/points`。
3. **IMU 单位是 g**：Livox 的 `linear_acceleration` 是重力加速度倍数（静止实测模长 ≈ 0.99），
   不是 m/s²；`orientation` 全零也没标无效。节点换算成 m/s² 并按 REP-145 把
   `orientation_covariance[0]` 置 -1，发到 `/head/lidar/imu`。

MID-360 在 URDF 里是**倒装**的（`mid360_link` 相对 `torso_link` 的 `rpy` roll = π，pitch = 0.0511），
IMU 静止读到 `az ≈ -0.97 g` 与之吻合。

### 在自己的节点里用

```python
from head_sensors.pointcloud import cloud_to_xyzi, filter_range

def on_cloud(msg):                       # sensor_msgs/PointCloud2
    xyzi = cloud_to_xyzi(msg)            # (N, 4) float32，零拷贝解析
    pts, dist = filter_range(xyzi, 0.1, 70.0)
    print('最近障碍 %.2f m' % dist.min())
```

`cloud_to_xyzi` 按 `msg.fields` 现场构造 numpy dtype，不写死 22 字节布局，换固件也不用改。

## 相机

`head_camera.launch.py` 只做两件事：转发参数给官方 `rs_launch.py`，再补一条挂载 TF。

话题（`camera_namespace:=head`、`camera_name:=camera`）：

| 话题 | 说明 |
|---|---|
| `/head/camera/color/image_raw` + `camera_info` | RGB，默认 848x480x30（需 USB3） |
| `/head/camera/depth/image_rect_raw` + `camera_info` | 深度 Z16，默认 848x480x30 |
| `/head/camera/aligned_depth_to_color/image_raw` | `align_depth:=true` 时才有 |
| `/head/camera/depth/color/points` | `pointcloud:=true` 时才有 |

`color_profile` / `depth_profile` 的格式是 `宽 x 高 x 帧率`，第三段是 fps。

**挂载 TF**：驱动只发相机自己那棵子树（`camera_link → camera_*_optical_frame`），
不知道相机装在机器人哪儿。launch 里补一条 `d435_link → camera_link`，默认恒等。
`d435_link` 在 URDF 里相对 `torso_link` 是 `xyz 0.0576235 0.01753 0.42987`、
pitch `0.8307767` rad（低头 47.6°）。恒等的前提是 `d435_link` 与 RealSense `camera_link`
同为 x 向前 / y 向左 / z 向上；如果实测点云姿态不对，改 launch 的 `camera_base_frame`
或在这条静态变换上加旋转，不要去改 URDF。

### 接线：脑袋后那个 Type-C 就是相机口

D435i 不是内部走线，需要用一根 Type-C 线把**头部后方的 Type-C** 接到机器人顶部电气接口板上的
USB host 口（官方接口表里的 6/7/8 号 Type-C 支持 USB3.0 host，9 号是 Alt Mode）。
没接线时 `lsusb` 里根本不会出现 `8086:0b3a`，装什么驱动都没用。

**一定要确认协商到的是 USB3**。实测同一台机器两种接法差别很大：

| 链路 | sysfs speed | `Device USB type` | 可用档位 |
|---|---|---|---|
| USB 2.1（`1-3`） | 480 Mbps | 2.1，驱动打 `Reduced performance is expected` | 彩色最高只能 `640x480x15`；`848x480x30` 被判 invalid 并回落 |
| USB 3.2（`2-2.2`） | 5000 Mbps | 3.2 | 彩色/深度 `848x480x30` 双流稳定满帧 |

查当前链路：

```bash
grep -H . /sys/bus/usb/devices/*/speed 2>/dev/null | grep -B0 . # 或直接看驱动日志的 "Device USB type"
```

USB2 接法下把两路都降到 15 fps 才不会互相抢带宽（实测彩色 14.1 Hz、深度 15.0 Hz）：

```bash
ros2 launch head_sensors head_camera.launch.py color_profile:=640x480x15 depth_profile:=640x480x15
```

### 别用 `ros2 topic hz` 判断相机掉没掉帧

`image_raw` 一帧 848x480x3 ≈ 1.2 MB，30 fps 就是 36 MB/s。Python 写的 `ros2 topic hz`
订阅端根本吃不下，会报出一个偏低的假帧率。同一时刻实测：

```
/head/camera/color/camera_info   29.96 Hz   min 0.032 max 0.034   ← 传感器真实出帧
/head/camera/color/image_raw     22.59 Hz   min 0.031 max 0.537   ← 订阅端掉的
```

要判断传感器有没有掉帧，看**同一路的 `camera_info`**（几十字节，不受传输影响）。

**相机 IMU** 默认关（`enable_gyro`/`enable_accel`）——机器人本体已经有盆骨和躯干两个 IMU。

## 参数

`head_lidar_node`：`input_cloud_topic` `input_imu_topic` `output_cloud_topic` `output_imu_topic`
`lidar_frame` `mount_frame` `publish_static_tf` `min_range` `max_range`
`imu_acceleration_in_g` `stats_period` `data_timeout`

`head_camera.launch.py`：`camera_namespace` `camera_name` `color_profile` `depth_profile`
`align_depth` `pointcloud` `enable_gyro` `enable_accel` `mount_frame` `camera_base_frame`
`wait_for_device_timeout` `reconnect_timeout`

## 排障

`head_sensors_probe` 会打印 USB 枚举结果。看到

```
USB : 没有枚举到 Intel(8086) 设备 —— 相机没接在本机 NX 上
```

就是头部那根 Type-C 线没接到顶部接口板，装驱动解决不了；接好后 `lsusb` 里应当出现
`8086:0b3a`（D435i）。驱动本身是否正常可以单独验证：

```bash
ros2 run realsense2_camera realsense2_camera_node --ros-args -p wait_for_device_timeout:=5.0
```

设备不在时它会打 `No RealSense devices were found!`。

启动时这条 warning 可以忽略，是 realsense-ros 自己的默认值越界，不影响出图：

```
Could not set param: rgb_camera.power_line_frequency with 3 Range: [0, 2]
```
