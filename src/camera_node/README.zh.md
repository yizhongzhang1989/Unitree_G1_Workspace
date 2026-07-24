# `camera_node`

## 来源与文档差异
本目录最初复制自 [`yizhongzhang1989/robot_dc`](https://github.com/yizhongzhang1989/robot_dc) 的 `colcon_ws/src/camera_node`，原始英文说明保留在 [`README.md`](README.md)。该来源记录不表示本目录会与原仓库持续同步；后续适配和部署配置由当前 `Unitree_G1_Workspace` 独立维护。

本文档不是原 `README.md` 的逐句翻译，而是面向当前工作区的部署说明，主要差异如下：

- 原 README 面向 ROS 2 Humble 和通用 `192.168.1.x` 示例；本文档以当前 ROS 2 Foxy 环境为准。
- 原 README 主要介绍包内自带的单相机、双相机和机械臂测试 launch；本文档重点记录 `robot_bringup` 实际启动的左右相机，包括 IP、RTSP URL、Web 端口和 ROS 图像话题。
- 本文档按当前 Ubuntu/ROS 环境说明 apt、`rosdep` 和 `ffmpeg` 的安装关系，避免把 `ffprobe` 当作独立 apt 包，也避免 pip 版 OpenCV/NumPy 与 Foxy 的 `cv_bridge` 产生兼容问题。
- 本文档以当前源码为准修正接口说明：图像话题由 `ros_topic_name` 明确指定，不会根据 `camera_name` 自动生成。
- 原 README 仍可用于了解上游设计和历史示例；在本工作区构建、启动与排障时应以本文档和 `robot_bringup` 当前配置为准。

通用 ROS 2 IP 相机节点。节点通过 FFmpeg 读取 RTSP 视频流，按新帧事件发布 `sensor_msgs/Image`，同时提供 Flask Web 预览、启停、重连、截图和 ROS 图像发布控制。

## 概览
本包把 IP 相机的 RTSP 视频流变成 ROS 2 图像和浏览器预览。`robot_bringup` 用它分别连接左右手相机，每台相机对应一个节点、图像话题和 Web 端口。

> 简单理解：FFmpeg 从相机取帧，`camera_node` 保存最新帧，再交给 `sensor_msgs/Image` publisher、MJPEG Web 页面和截图服务。

本包负责连接、断流重连、图像发布、预览和截图；不负责目标识别、视觉定位、深度估计或相机标定。

## 功能
- 使用 FFmpeg TCP 传输读取 RTSP 主码流，并在连接失败时提供重试和错误状态。
- RTSP 相机未连接或中途断流时保持 ROS/Web 节点运行，并在后台定时重连；相机恢复后继续使用原 ROS 发布器和 Web 地址。
- 使用 `cv_bridge` 发布 ROS 2 图像；QoS 为 RELIABLE、VOLATILE、KEEP_LAST(1)。
- 每个相机提供独立的 Web 页面、MJPEG 视频流和状态 API。
- 通过 ROS 2 服务或 Web 页面截图，文件保存到 `/tmp/<camera_name>_snapshots/`。
- 支持多个相机实例；节点名、Web 端口、图像话题和服务名必须互不冲突。

## 数据流
```mermaid
flowchart LR
  Camera[IP 相机] -->|RTSP / TCP| FFmpeg[FFmpeg 子进程]
  FFmpeg --> Stream[RTSPStream]
  Stream -->|sensor_msgs/Image| ROS[ROS 2 订阅者]
  Stream -->|MJPEG| Web[Flask Web 页面]
  Service[Trigger 服务] -->|截图 / 重启| Stream
```

FFmpeg 读取线程保存最新帧；ROS 发布线程仅在新帧到达时转换并发布图像；Flask 在独立线程中提供 Web 接口。Web 预览和 ROS 发布共享同一条 RTSP 输入，不会为每个浏览器连接重复打开相机。

## 依赖与构建
本工作区使用 ROS 2 Foxy。`ffprobe` 随 Ubuntu 的 `ffmpeg` 软件包安装：
```bash
cd ~/Unitree_G1_Workspace
sudo apt-get update
sudo apt-get install -y ffmpeg
rosdep install --from-paths src/camera_node --ignore-src -r -y

colcon build --symlink-install --packages-select camera_node robot_bringup
source scripts/env.sh
```

主要运行依赖为 `rclpy`、`sensor_msgs`、`std_srvs`、`cv_bridge`、Flask、OpenCV 和 NumPy，完整声明见 `package.xml`。

## 通过 bringup 启动

`robot_bringup` 的末端设备入口固定启动左右两个相机。单总线、双总线以及末端设备 Dashboard 均使用同一份配置：

| 侧别 | ROS 节点 / `camera_name` | IP | RTSP URL | Web | 图像话题 |
|---|---|---|---|---:|---|
| 左手 | `camera_left` | `192.168.123.97` | `rtsp://admin:123456@192.168.123.97/stream0` | `8010` | `/camera_left/image_raw` |
| 右手 | `camera_right` | `192.168.123.98` | `rtsp://admin:123456@192.168.123.98/stream0` | `8011` | `/camera_right/image_raw` |

```bash
source scripts/env.sh
ros2 launch robot_bringup end_effectors_single_bus.launch.py
# 或
ros2 launch robot_bringup end_effectors_dual_bus.launch.py
```

浏览器访问：
- 左手：`http://<机器人 IP>:8010`
- 右手：`http://<机器人 IP>:8011`

部署值定义在 `robot_bringup/robot_bringup/end_effectors/nodes.py`。当前 RTSP 用户名、密码和路径沿用本包原有相机约定；若相机端配置变化，应同时更新对应 URL。

## 单独启动

使用节点默认值启动：
```bash
source scripts/env.sh
ros2 run camera_node camera_node
```

指定一个相机：
```bash
ros2 run camera_node camera_node --ros-args \
  -r __node:=camera_test \
  -p camera_name:=camera_test \
  -p rtsp_url_main:=rtsp://admin:123456@192.168.123.97/stream0 \
  -p camera_ip:=192.168.123.97 \
  -p server_port:=8080 \
  -p ros_topic_name:=/camera_test/image_raw \
  -p publish_ros_image:=true
```

本包还保留从原项目复制的测试 launch：
```bash
ros2 launch camera_node single_stream_test.py
ros2 launch camera_node double_camera_launch.py
ros2 launch camera_node robot_arm_cam_launch.py
```

这些测试 launch 使用原有的 `192.168.1.x` 地址，不属于本工作区的生产 bringup。`ur10e_cam_launch.py` 和 `ur15_cam_launch.py` 依赖原项目的 `common.config_manager`，当前工作区未提供该模块。

## 参数

| 参数 | 类型 | 节点默认值 | 说明 |
|---|---|---|---|
| `camera_name` | string | `GenericCamera` | 相机标识，也用于生成服务名和截图目录 |
| `rtsp_url_main` | string | `rtsp://admin:123456@192.168.1.100/stream0` | RTSP 主码流 URL |
| `camera_ip` | string | `192.168.1.100` | 用于状态显示的相机 IP；实际连接使用 `rtsp_url_main` |
| `server_port` | int | `8010` | Flask Web 监听端口 |
| `stream_fps` | int | `25` | Web MJPEG 目标帧率 |
| `jpeg_quality` | int | `75` | Web JPEG 质量，范围 `1-100` |
| `max_width` | int | `800` | Web 图像最大宽度，保持原始宽高比 |
| `publish_ros_image` | bool | `true` | 启动时是否发布 ROS 图像 |
| `ros_topic_name` | string | `/camera/image_raw` | 图像发布话题；不会从 `camera_name` 自动推导 |
| `auto_reconnect` | bool | `true` | 期望运行但无有效视频帧时是否后台自动重连 |
| `reconnect_interval_s` | float | `5.0` | 自动重连检查周期，必须大于 `0` |

## ROS 2 接口

### 发布

| 名称 | 类型 | 说明 |
|---|---|---|
| `<ros_topic_name>` | `sensor_msgs/Image` | 新帧到达时发布 BGR 图像 |

### 服务

假设 `camera_name=camera_left`：

| 名称 | 类型 | 说明 |
|---|---|---|
| `/camera_left/take_snapshot` | `std_srvs/srv/Trigger` | 保存当前帧到 `/tmp/camera_left_snapshots/` |
| `/restart_camera_left_node` | `std_srvs/srv/Trigger` | 重启 RTSP 流 |

```bash
ros2 service call /camera_left/take_snapshot std_srvs/srv/Trigger '{}'
ros2 service call /restart_camera_left_node std_srvs/srv/Trigger '{}'
```

## Web 接口

> [!INFO]
> 每个 `camera_node` 实例启动时都会自动启动一个内置 Flask Web 服务，不需要另行启动网页节点。访问该实例的 `server_port` 根路径即可打开相机控制页；当前 `robot_bringup` 中左、右相机分别使用 `8010` 和 `8011`。`end_effectors_dashboard.launch.py` 启动的 `8770` 统一联调面板是另一个 Web 服务，它通过相机内置服务的 `/status` 和 `/video_feed` 接口获取状态和代理视频。

| 路径 | 方法 | 说明 |
|---|---|---|
| `/` | GET | 控制页面 |
| `/video_feed` | GET | MJPEG 视频流 |
| `/status` | GET | RTSP 运行状态 |
| `/start` | POST | 启动视频流 |
| `/stop` | POST | 停止视频流 |
| `/restart` | POST | 重启视频流 |
| `/snapshot` | POST | 保存截图 |
| `/download_snapshot` | GET | 下载最近一次截图 |
| `/ros_image_publish` | GET / POST | 查询或切换 ROS 图像发布 |

Web 服务监听 `0.0.0.0`，当前没有身份认证。RTSP URL 也包含明文凭据，只应在可信隔离网络中使用，不应把 `8010/8011` 暴露到互联网。

## 验证与排障
左右相机均位于 `192.168.123.0/24`。先确认机器人主机具有到该子网的路由，并能分别访问两台相机：
```bash
ip route get 192.168.123.97
ip route get 192.168.123.98
ping -c 3 192.168.123.97
ping -c 3 192.168.123.98
```

直接验证 RTSP 和 Web 状态：
```bash
ffprobe -rtsp_transport tcp \
  rtsp://admin:123456@192.168.123.97/stream0
curl http://127.0.0.1:8010/status
curl http://127.0.0.1:8011/status
```

检查 ROS 接口：
```bash
ros2 node list | grep camera
ros2 topic hz /camera_left/image_raw
ros2 topic hz /camera_right/image_raw
ros2 service list | grep camera
```

若节点启动但没有图像，依次检查主机路由、相机账号密码、`/stream0` 路径和 FFmpeg 输出。若 Web 无法监听，使用 `ss -ltnp | grep -E ':8010|:8011'` 检查端口占用。
