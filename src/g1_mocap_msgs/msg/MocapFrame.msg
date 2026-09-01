# 一帧动捕的重定向结果。**原子**：这里的每个字段都出自同一帧骨架，
# 拆成多个 topic 发就会引入时间同步问题，而配错了不报错、只是姿态悄悄不对。
#
# 发布频率跟随头显（72/90 Hz），**不做降采样**。下游要 50 Hz 就自己按 stamp 插值——
# 先降采样再插值会把原始时间分辨率丢掉，参考窗口那种靠差分求速度的用法直接毁掉。

# 采样时刻，已经从头显时钟对齐到本机 ROS 时钟。
# 用的是头显的 predictedDisplayTime 做帧间增量、首帧到达时刻做锚点，所以帧间隔干净，
# 只有锚点那一次吃了 WiFi 抖动。**按时间插值时用它，不要用接收时刻。**
std_msgs/Header header

# 头显给的帧号，单调递增。丢帧能从这里看出来。
uint32 seq

# XrBodyTrackingStatusPICO：0=INVALID 1=VALID 2=LIMITED
uint8 BODY_INVALID = 0
uint8 BODY_VALID = 1
uint8 BODY_LIMITED = 2
uint8 body_status

# XrBodyTrackingMessagePICO，0 为正常。7=追踪姿态错误（头显没戴好时常见）。
uint8 body_message

# --- 重定向结果 -------------------------------------------------------------
# 本话题只在校准完成后才开始发，收到帧本身就意味着人机比例已经标过。
# 要判断“当前标没标”看 MocapStatus.calibrated。
# 关节名随帧一起发，是为了让消息自描述：只发数值的话，下游一旦顺序对不上
# 就是左右腿互换级别的事故，而且不会报任何错。
string[] joint_names
float64[] joint_positions

# 骨盆（root）与锚刚体（torso_link）在动捕坐标系下的位姿。
# 坐标系是 X 前 / Y 左 / Z 上，原点由校准帧决定，**不是**世界绝对坐标。
geometry_msgs/Pose root
geometry_msgs/Pose anchor

# key body 的笛卡尔位置，由 G1 自己的正运动学算出，和上面的关节角自洽。
# 顺序即 key_body_names，两者一一对应。
string[] key_body_names
geometry_msgs/Point[] key_body_positions

# --- 原始输入 ---------------------------------------------------------------
# 人的 24 关节骨架，SMPL 顺序（即 XrBodyJointBD 的枚举值 0..23）。
# 已换到机器人坐标系，但**未按人机比例缩放**——它是原始观测，不是重定向产物。
geometry_msgs/Point[] human_joints
