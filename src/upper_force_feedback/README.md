# 双臂力反馈

## 实现
`upper_force_feedback` 是接入 `g1_motion_session` 的独立上肢任务。每收到一帧原始 G1 `/lowstate`（`unitree_hg/msg/LowState`），立即取该帧左右七轴实测角度，以映射后的右臂位置作为左臂目标、映射后的左臂位置作为右臂目标，直接向 FPC 发布双臂 14 轴掩码，不经过独立定时器降采样或 IK，也不以上一拍目标代替实测位置。耦合尽力处理收到的原始反馈，不固定输出周期，也不人为限制为 500 Hz；实际输出频率取决于原始反馈到达速率、DDS 和执行器调度。没有新反馈时不重复发布旧的耦合目标。任务不订阅 `/joint_states`，不依赖夹爪反馈；夹爪不登记、不控制，由会话默认输出保持。

## 镜像与反馈
七轴顺序为肩 pitch、肩 roll、肩 yaw、肘、腕 roll、腕 pitch、腕 yaw。启动配置 `arm_signs` 默认 `[+1, -1, -1, +1, -1, +1, +1]`：第 2、3、5 轴（肩 roll、肩 yaw、腕 roll）反号，其余轴同号，双向使用同一符号表，每项只能为 `-1.0` 或 `1.0`。第二轴用于对称张开大臂：左臂向左张开，右臂向右张开。[G1 URDF](../unitree_g1_description/model/final.urdf) 正运动学测试验证肩 pitch 同号时两手前后运动一致，肩 roll 反号时两手向两侧张开。

两侧绝对角度按符号直接交换，不添加初始相对偏移、滤波、额外限速或力矩指令。默认预备姿态的左肩 roll `+0.6`、右肩 roll `-0.6` 符合第二轴映射关系，不会因耦合而互换成内收姿态；其他实测角度若不满足映射关系，启用时仍会主动互相靠拢。关节耦合不是笛卡尔耦合，固定符号表无法保证任意姿态下两只手的完整三维位移同向。

反馈来自现有底层位置 PD 和重力补偿：一侧受力而偏离目标时，其实测位移传给另一侧，形成双向位置耦合。这不是直接测量或精确复制末端力；力度与阻尼取决于现有硬件 PD、负载、摩擦和通信延迟。本包不修改硬件增益。真机首次验证应由操作者使用已确认的低刚度配置，并保持急停可用；双向耦合的稳定性和反馈力度仍需真机验证。

## 启动
在工作区根目录构建并加载：
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to upper_force_feedback robot_bringup --symlink-install
source install/setup.bash
```

### 完整启动
终端 1：启动硬件与唯一控制栈，`arm_stiffness_scale:=4.0` 是高刚度示例。
```bash
source scripts/env.sh
ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual arm_stiffness_scale:=4.0
```

终端 2：启动会话和双臂力反馈任务，不启动下肢任务。
```bash
ros2 launch robot_bringup motion_tasks.launch.py lower_task:=none upper_task:=force_feedback
```

终端 3：等待登记和反馈就绪后 Engage，准备完成自动进入双向耦合，不需要另调 Start。
```bash
ros2 service call /motion_control/engage std_srvs/srv/Trigger '{}'
```

停止耦合并请求停用控制器：
```bash
ros2 service call /motion_control/estop std_srvs/srv/Trigger '{}'
```

也可以单独启动（与上面的终端2互斥）：

终端2：
```bash
ros2 launch g1_motion_session session.launch.py
```

终端3：
```bash
ros2 launch upper_force_feedback force_feedback.launch.py
```