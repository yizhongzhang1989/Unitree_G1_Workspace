# 双总线 CAN 占用与 KWR57 频率
当前 `end_effectors_dual_bus.launch.py` 在 CAN0、CAN1 各连接一台 KWR57 和一台 Gloria-M。两个 CAN 通道均为 1 Mbps，并分别承载一套设备，因此两条总线的占用不能相加为单条总线利用率。

## CAN 占用
KWR57 在 1 kHz 下每个样本使用 3 个 8-byte 标准 CAN 帧，即每台产生 3000 frame/s。标准帧计入帧间隔后按 111 bit 计算，考虑最坏位填充时按 135 bit 计算：
$$
U = \frac{N_{frame} \times (111\text{ 至 }135)}{1\,000\,000}
$$

Web 夹爪往返默认以 100 Hz 发送 MIT 命令。按每条命令和 10 Hz 主动状态请求均产生一帧反馈估算，每台 Gloria-M 约产生 220 frame/s。

| 每条物理总线上的设备 | 帧率 | 无填充占用 | 保守上界 |
|---|---:|---:|---:|
| KWR57，1 kHz | 3000 frame/s | 33.300% | 40.500% |
| Gloria-M，100 Hz 往返 | 约 220 frame/s | 2.442% | 2.970% |
| **合计** | **约 3220 frame/s** | **35.742%** | **43.470%** |

因此当前配置仍有约 56.5% 的保守线速余量，CAN 仲裁不是双路 1 kHz 的限制。

## 1 kHz 数据路径
生产 `canalystii_native_bridge` 使用一个 libusb event 线程处理异步 RX completion/重提交、一个独立 TX owner 线程执行阻塞 USB OUT，并为 CAN0/CAN1 各配置一个 RX worker 和有界 packet 队列。event 线程不执行 TX、KWR57 三帧组包或 ROS 回调，因此阻塞发送和任一通道的处理不会占住 USB completion 路径或另一通道的 worker。生产 KWR57 在对应 RX worker 中组包，不发布中间 `can_msgs/Frame`，每个完整样本只发布一次 `WrenchStamped`。

CANalyst-II 固件约返回 `9.5k` 个 completion/s/通道，其中大量 64 字节 USB 包的 frame count 为 0。零帧包是合法协议数据，但无需进入 RX 队列；completion 回调现在只计数，不加队列锁、不通知条件变量，也不唤醒 worker。默认保持 8 个异步 RX transfer/通道；4-transfer A/B 更差，不能降低固件空包率。

临时最大时延计数和独立测频工具不参与生产运行，已从代码库和 `/tmp` 清理。`io_diagnostics` 只保留可选基础吞吐计数并默认关闭；紧时延验收中不得开启。

## 实测与推荐配置
2026-07-23 在 Unitree G1 PC2 上运行双 KWR57、双 Gloria-M、active FPC 和生产 100 Hz hold。测试持续 30 秒，使用默认 8 RX transfers/通道、关闭 `io_diagnostics`，不启动相机：

| 指标 | CAN0 / 左侧 | CAN1 / 右侧 |
|---|---:|---:|
| KWR57 source 最大 gap | 6.860 ms | 7.322 ms |
| KWR57 ROS receive 最大 gap | 7.027 ms | 7.433 ms |
| MIT 命令平均频率 | 100.000 Hz | 100.000 Hz |
| 实际 CAN TX 平均频率 | 99.999 Hz | 100.001 Hz |

双 KWR57 source/receive 最大 gap 均低于 10 ms，四设备生产场景通过。该结果不包含相机负载；相机并发应按具体部署单独复测。

KWR57 推荐配置保持：
```python
period_ms = 1
sample_rate_hz = 1000
publish_rate = 0.0
```

控制节点建议使用 `rclcpp`、BEST_EFFORT 和 `KEEP_LAST(64)`，并在回调中避免日志、阻塞 I/O 和无界分配。Dashboard 采用相同深度的 raw `WrenchStamped` 订阅，只在 HTTP 快照时反序列化最新样本；它显示的是 3 秒平均接收吞吐，不代表每个样本都满足 1 ms deadline。实时控制仍应自行检查数据时效、延迟、抖动和 deadline miss。
