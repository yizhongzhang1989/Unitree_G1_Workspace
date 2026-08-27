"""G1 头部雷达世界定位。

对外只有两个东西：

* ``~/set_origin``  （``std_srvs/Trigger``）把此刻的躯干位姿钉成世界原点
* ``~/torso_pose``  （``nav_msgs/Odometry``）躯干在世界系的位姿、速度与协方差

其余都是实现细节，可以整体替换掉 Point-LIO 而不影响下游。
"""
