# 被动颈轴标定

颈部结构与传感器安装不变时无需重标。日常掰头由双 IMU 估计角度，更新 TF。

## 原理

约定 `T_A_B` 将 B 系坐标转换到 A 系。腕部相机使用已有内外参，三台相机观察同一块 ChArUco 板：

```text
T_torso_board = T_torso_wrist_camera * T_wrist_camera_board
T_torso_head_camera = T_torso_board * inverse(T_head_camera_board)
```

两个不同头姿态的空间相对变换 `T_2 * inverse(T_1) = (R, t)` 给出旋转轴方向，轴心满足 `(I-R)c=t`。沿轴的位置不唯一，本次约定 torso 系下 `c_y=0`。

直头时记录头部和躯干 IMU 的重力矢量作为零位参考。结合名义安装朝向对齐坐标系，将运行时的重力投影到颈轴垂直平面，由有向夹角估计头角 q。消去头部转动，得到固定安装变换：

```text
T_neck_camera = inverse(T_torso_neck(q)) * T_torso_camera
```

`T_torso_neck(q)` 的平移为轴心，旋转为绕颈轴转动 q。相机变换必须包含 d435 到真实光学中心的平移。

## 流程

1. 确认腕部已有内外参和头相机内参，保留采集时配置。
2. 头部摆正，静止记录双 IMU，定义零位。
3. 在两个不同头部俯仰姿态下，分别静止采集多帧三相机图像、CameraInfo、双 IMU 和 TF。
4. 逐张估计板位姿，按图像时间查询腕部 TF，计算头相机在 torso 下的位姿。
5. 用两组相对运动求轴，左右腕等权；结合 IMU 头角求固定安装外参。
6. 可额外采集第三姿态检查预测结果，并在不同头角下检查模型与实拍轮廓叠加。
7. 更新源 URDF 并重新生成 final；两个 IMU 零位矢量保存在 [calibration.yaml](config/calibration.yaml) 的 `head_imu_reference` 中，几何参数不重复保存。

## 运行

`head_sensors` 读取零位配置，并订阅实际 `robot_description`，从 `head_pitch_joint` 获取父子 link、轴方向、轴心及限位，再发布颈部动态 TF。当前估计要求关节 origin 的 rpy 为零；模型缺失或无效时不发布。消费者按传感器时间戳查完整 TF，不需要头部 JointState。离线 FK 需保存头角或完整相机变换。

头部零位矢量单位为 g，躯干为 m/s²。它们对应本机 IMU 安装，不是所有机器人通用。重力无法单独确定 yaw，运动加速度也会影响估计。

腕部标定程序及其结果继续保留。当前 neck 依赖原腕部外参；本次第三姿态参与过内参拟合，不是完全独立验证。不要用互相依赖的标定结果证明绝对精度。