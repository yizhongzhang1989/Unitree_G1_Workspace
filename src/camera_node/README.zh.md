# `camera_node`

IP 相机 RTSP → `sensor_msgs/Image`。**没人要图的时候不拉流**。

原始实现复制自
[`yizhongzhang1989/robot_dc@94d4030`](https://github.com/yizhongzhang1989/robot_dc/tree/94d4030db170edaa986b8d1243fd8ae27d45cffd/colcon_ws/src/camera_node)，
2026-08-17 按本工作区需要重写，不再与上游同步；改了什么见 [`change.log`](change.log)。

```mermaid
flowchart LR
  cam[IP 相机] -->|RTSP/TCP| ff["ffmpeg 子进程<br/>fps + scale 都在这里做"]
  ff -->|裸 BGR 管道| pump[读帧线程]
  pump -->|有订阅者才发| ros["sensor_msgs/Image"]
  pump -->|有人在看才留| web["MJPEG 预览页<br/>server_port > 0 才开"]
  sup["监管线程 1 Hz<br/>订阅数 / 观众数 / 帧龄"] --> ff
```

## 按需拉流

监管线程每 `poll_period_s` 看一眼 `get_subscription_count()` 和网页观众数：

| 情况 | 动作 |
|---|---|
| 都是 0 | kill ffmpeg，一个字节都不走网 |
| 有订阅者 或 网页在看 | 拉起 ffmpeg |
| ffmpeg 退了 / `stale_timeout_s` 没新帧 | kill 掉重连 |
| 只有网页在看 | 拉流但**不**组装 `Image` 消息 |

代价是订阅者接上后要等 1~3 s（轮询周期 + RTSP 握手）才有第一帧。`vla_bridge`
那边的 `image_timeout_s` 会正常重试，不用管。

## 性能：钱花在哪（Orin NX 实测）

单台相机，`stream1` 640x360 HEVC → 426x240 BGR @15 fps，稳态 `/proc` 增量：

| 环节 | 单核占比 |
|---|---:|
| ffmpeg HEVC 解码 | 8.7% |
| ffmpeg 缩放 + yuv→bgr24 | 1.7% |
| CycloneDDS 后台线程（**按进程算，不是按节点**） | 4.6% |
| 读管道 | 0.7% |
| 组包 + publish（正比于字节数） | ~2% |
| **单台合计** | **18.5%**（两台 = 8 核机器的 4.6%） |

三条设计前提，实测数字和取舍过程见 [`change.log`](change.log)：

- **`msg.data` 只能赋 `array.array('B', raw)`。** 赋 `bytes` 会走 rclpy 的逐元素
  断言，640x360 一帧 102 ms、1080p 约 920 ms。`cv_bridge` 内部就是 `tobytes()`，
  所以这里不用它——这是旧实现「帧率很小」的真凶。
- **缩放和限帧都在 ffmpeg 里做**，管道上只走目标分辨率。`scale` 用 `flags=area`
  （下采样盒式平均，比默认 bicubic 省 12% 且不 aliasing）。**只适合下采样**，
  别把目标尺寸设得比原生大。
- **优先用子码流。** 同样出 426x240，走 `stream1` 是 12.2% 单核，
  走 `stream0`(1080p H.264) 是 42.5%。

**GPU 硬解在本容器用不了**：NVDEC 设备节点随 `privileged` 挂进来了，但 tegra 用户态
库、CUDA、gstreamer 插件全缺，`hevc_cuvid` 直接 segfault。要开得改 `.devcontainer`
并重建容器，而 NVDEC 最多拿走两台合计 17.4% 单核（机器的 2.2%），当前分辨率下不划算。

## 分辨率：只给高度就行，且可以运行时改

`image_width` / `image_height` 只给一边时，另一边**按原生宽高比自动补**（取偶数）：
640x360 给 `image_height:=240` 就是 426x240，恰好是 `vla_bridge` 发给模型的尺寸。

参数表里标了 ✓ 的都**运行时可改**，不用重启节点：

```bash
ros2 param set /camera_left image_height 120     # 下一拍（≤ 1 s）重开 ffmpeg
ros2 param set /camera_left rtsp_url rtsp://admin:123456@192.168.123.97/stream0
```

监管线程每拍拿当前参数和已生效的一组比，不一致就 kill 掉重开。负数和空 `rtsp_url`
在 `on_set_parameters` 里就被拒，`ros2 param set` 会直接报失败。

## 参数

| 参数 | 类型 | 默认 | 运行时可改 | 说明 |
|---|---|---|:-:|---|
| `rtsp_url` | string | —— | ✓ | 必填，空则启动失败 |
| `image_width` / `image_height` | int | `0` | ✓ | 只给一边就按宽高比补另一边；都为 0 走原生 |
| `fps` | int | `0` | ✓ | `0` = 不限帧 |
| `jpeg_quality` | int | `60` | ✓ | 预览页 JPEG 质量 |
| `image_topic` | string | `~/image_raw` | | 发布话题 |
| `frame_id` | string | 节点名 | | `header.frame_id` |
| `server_port` | int | `0` | | **`0` 就不开预览页**，也不 import cv2 |
| `poll_period_s` | double | `1.0` | | 监管线程周期 |
| `stale_timeout_s` | double | `5.0` | | 多久没新帧算断流 |

QoS 固定 RELIABLE / VOLATILE / KEEP_LAST(1)——`vla_bridge` 订阅端是 RELIABLE，
改成 BEST_EFFORT 会直接匹配不上。

## 跑起来

`robot_bringup` 的末端设备入口固定起左右两个相机，配置在
`robot_bringup/robot_bringup/end_effectors/nodes.py`：

| 侧别 | 节点 / `frame_id` | IP | RTSP | 话题 | 预览页 |
|---|---|---|---|---|---:|
| 左手 | `camera_left` | `192.168.123.97` | `/stream1` | `/camera_left/image_raw` | `8010` |
| 右手 | `camera_right` | `192.168.123.98` | `/stream1` | `/camera_right/image_raw` | `8011` |

两台都是 `image_height:=240` + `fps:=15`，实际出图 **426x240**。

```bash
source scripts/env.sh
ros2 launch robot_bringup end_effectors_single_bus.launch.py
```

单独起一台（默认**不开**预览页）：

```bash
ros2 launch camera_node camera.launch.py \
  name:=camera_left rtsp_url:=rtsp://admin:123456@192.168.123.97/stream1 \
  image_topic:=/camera_left/image_raw image_height:=240 fps:=15
# 要看画面再加 server_port:=8010
```

## 预览页

只有 `server_port > 0` 才起，三个路由：`/` 页面、`/video_feed` MJPEG、`/status` JSON。
用标准库 `http.server`，不依赖 Flask；cv2 也是这时候才 import。
**开预览页的代价是内存不是 CPU**：RSS 53 → 153 MB（全是 cv2 一个库的 116 MB），
CPU 在噪声内。

`robot_bringup` 的 8770 面板 **不订阅 ROS 图像话题**，它是 HTTP 代理这两条路由的：

```
浏览器 → 8770 /api/cameras/left/video_feed → dashboard 代理 → 8010 /video_feed
```

所以 `server_port: 0` 会让 8770 上的腕相机那两栏变成离线占位。

- **只有 `/video_feed` 算观众，`/status` 轮询不算** —— 8770 开着但没人看时流不会起。
- `/status` 的 `state` 有 `idle` / `streaming` / `error`，`is_running` 只在 `error`
  时为 false —— 否则 8770 会把「没人看所以没拉流」误报成相机离线。
- **断连检测不能只靠 write。** 没帧可写时（比如相机正好断线）循环一个字节也不发，
  浏览器关掉也发现不了，观众数就永远挂着。超时那一拍会用
  `MSG_DONTWAIT | MSG_PEEK` 探一下对端（`preview.connected`）。

## 已知坑

- **相机 IP 相关的路由必须存在**：本机要能到 `192.168.123.0/24`。
- `stream1` 是 HEVC，ffmpeg 偶尔刷 `Could not find ref with POC nn`，是丢包后
  重建参考帧，不影响出图，已用 `-loglevel error` 压掉大部分噪声。
- 相机侧的码流参数在相机自己的 Web 界面上改（`http://192.168.123.97`）。
- `ros2 bag record`、`rqt_image_view`、`ros2 topic hz` 都算订阅者，会把流拉起来。

## 验收

```bash
python3 -m pytest src/camera_node/test -q
python3 -m pycodestyle --max-line-length=120 src/camera_node/camera_node \
  src/camera_node/launch src/camera_node/test
```

`test/` 只覆盖不依赖 ROS 运行时和相机的部分：ffmpeg 命令拼装、按需拉流判据、
读帧线程的整帧切分与 kill 唤醒、`Image` 载荷类型。链路本身在实机上验。
