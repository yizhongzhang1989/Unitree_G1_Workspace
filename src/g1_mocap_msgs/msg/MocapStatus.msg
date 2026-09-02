# 动捕链路状态。低频（1 Hz），给人看也给看门狗用。
#
# 结构化而不是一行文本：下游要拿它做急停判据（断流后参考会钳在最后一帧，
# 机器人保持姿势站着，看起来毫无异样），解析字符串太脆。

std_msgs/Header header

# 头显的 WebSocket 是否连着。
bool connected

# 人机比例是否已经标过。没标过时重定向结果无意义。
bool calibrated

# 累计收到 / 丢弃的帧数。dropped 持续增长说明 body_status 一直不可用，
# 或者骨架里出现了零长肢体（tracker 短暂丢失）。
uint64 frames
uint64 dropped

# 最近一帧的 body 状态，取值同 MocapFrame。
uint8 body_status
uint8 body_message
string body_message_text

# 最近一次内部错误，正常为空串。
string last_error

# 校准出来的人机比例，标过之后才有意义。
float64 scale
float64 stand_height
