# Online RL 人工评分

同包独立 bridge/CLI，接口为 `/online_rl_bridge/*`，复用普通 bridge 的观测、历史、坐标转换与控制保护。

## 启动
```bash
source install/setup.bash
ros2 launch g1_vla_bridge online_rl.launch.py \
  rl_directory:=/workspace/log/vla_rl/experiment_001 \
  proxy:=socks5h://127.0.0.1:1080
```

服务地址沿用原 CogACT 配置，无需重复传 `server_url`；服务须支持 Online RL 接口。

另一个终端：
```bash
ros2 run g1_vla_bridge online_rl_cli
```

## 交互与评分

输入任务 → Enter 执行一段 → `1/2/3/4/5` 评分，映射 `-1/-0.5/0/+0.5/+1`；**评分时 Enter 默认 3（零分）**。
确认后 Enter 继续，或 `/home`、换任务。评分绑定显示的 chunk ID；未观察用 `/null`，中断或未执行只能提交 null。
`/home` 保留任务，停止 VLA、下发原双臂 home 目标，夹爪保持且不确认到位；再次 Enter 沿用原 start 行为。
`/start /stop /engage /estop /status /quit` 可用；待评分时仍可停止和急停。退出本 CLI 会请求 stop，停止不是卸力。

## 执行范围

仅支持 manual 逐点执行。前缀取 `execution_chunk_size`、返回长度和正数 `action_horizon` 的最小值，其它控制设置不变。
`executed_steps` 是已下发的模型步数，不是 30 Hz 插值次数或物理到位确认。
`rl_max_observation_age_s` 默认 3 秒，按真实观测时刻检查响应；过期不执行并提交 null。这不是安全保证，不影响普通 bridge。

## 记录与恢复

- 训练数据在服务端。SQLite 长期仅存 ID、版本/批次、时间、执行结果与评分确认；动作只在内存。
  请求 JSON/JPEG 只为原样重试临时保存，结果确定或作废后原子释放，保留响应摘要检查冲突；旧账本下次独占启动自动精简并回收空间。
- 推理断网或 409 pending 重试原字节；503 updating/stale_observation 作废旧请求，collecting 后 Enter 采新观测。
  反馈断网只重试原评分，不重执行；冲突、error、未知协议停止放行。429 pending_limit 要排查积压，不新建请求绕过。
- 重启不续播：收到但未执行的动作跳过并提交 null；执行中崩溃为 uncertain，人工核实后 `/resolve N`，再 `/null`，不得猜 0。
- 同目录单写者，停止客户端后备份目录。home/reset/start 不清 RL 账本，客户端不调用服务端 reset/train/retry。
