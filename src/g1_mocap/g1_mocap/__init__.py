"""PICO 全身动捕 -> G1 关节角。与任何具体策略无关。

链路全程 WiFi（不用 adb）：头显上的 PicoBridge APK 把 24 关节 SMPL 骨架推过来，
本包把它重定向成 G1 的 29 轴关节角，并用 G1 自己的 URDF 做正运动学补出各刚体位姿。
接入方式和精度数据见 ``README.md``。

四个模块：

* :mod:`.skeleton`   —— 报文解析、XR 坐标系换到机器人坐标系、发送端时钟对齐
* :mod:`.kinematics` —— pinocchio 封装；零位几何常量全部从 URDF 现算，不硬编码
* :mod:`.retarget`   —— SMPL 24 关节位置 -> G1 29 轴关节角（纯几何，不用厂商的关节朝向）
* :mod:`.stream`     —— 后台线程收帧 + 环形缓冲 + 按时间插值

下游怎么用这些数据是下游的事。``g1_rgmt_tracking_global`` 里的 ``MocapClip`` 把这条流
装配成 RGMT 的参考窗口，那部分逻辑属于策略契约，不在本包里。
"""

from .kinematics import G1Kinematics
from .retarget import RetargetCalibration, Retargeter, RetargetResult
from .skeleton import SMPL_JOINTS, BodyFrame, parse_body
from .stream import MocapStream, SampleBatch

__all__ = [
    'BodyFrame',
    'G1Kinematics',
    'MocapStream',
    'RetargetCalibration',
    'RetargetResult',
    'Retargeter',
    'SMPL_JOINTS',
    'SampleBatch',
    'parse_body',
]
