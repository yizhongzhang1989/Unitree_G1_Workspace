# 双总线 CAN 占用与 KWR57 频率
当前 `dual_bus.launch.py` 在 CAN0、CAN1 各连接一台 KWR57 和一台 Gloria-M。两个 CAN 通道均为 1 Mbps，并分别承载一套设备，因此两条总线的占用不能相加为单条总线利用率。

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
bridge 使用一个 USB I/O 线程收发 CAN，并为每个通道配置独立处理线程和有界 oldest-drop 缓冲。USB 线程不执行 KWR57 三帧组包；每个通道的 handler 独立处理，TX 也限制单次批量，避免发送挤占接收。生产 KWR57 使用进程内 handler，不发布中间 `can_msgs/Frame`，每个完整样本只发布一次 `WrenchStamped`。

这些改动解决的是共享 USB/主机处理路径的串行阻塞，属于达到双路 1 kHz 所需的生产代码。周期吞吐日志和独立测频工具不参与运行，已从代码库删除。

## 实测与推荐配置
2026-07-15 在 Unitree G1 PC2 上运行双 KWR57、双 Gloria-M 100 Hz 往返、Web dashboard 和相机重连时：

- 两台 KWR57 的组包与发布均约为 1 kHz；
- Web 接收频率在完整负载下左右均约为 1 kHz；
- 独立 C++ 消费者实测双路约 1 kHz，确认结果不只来自 Web 显示。

KWR57 推荐配置保持：
```python
period_ms = 1
sample_rate_hz = 1000
publish_rate = 0.0
```

控制节点建议使用 `rclcpp`、BEST_EFFORT 和 `KEEP_LAST(64)`，并在回调中避免日志、阻塞 I/O 和无界分配。Dashboard 采用相同深度的 raw `WrenchStamped` 订阅，只在 HTTP 快照时反序列化最新样本；它显示的是 3 秒平均接收吞吐，不代表每个样本都满足 1 ms deadline。实时控制仍应自行检查数据时效、延迟、抖动和 deadline miss。
