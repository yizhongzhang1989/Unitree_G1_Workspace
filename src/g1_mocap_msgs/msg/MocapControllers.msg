# 双手柄状态，跟随头显帧率（72/90 Hz）。
#
# **从连上头显就发，不等校准，骨架坏了也照发。** 这条流是给急停一类的操作用的，
# 而最需要按急停的时刻恰恰是数据流出问题的时刻：tracker 全丢（body_status=0）、
# 骨架里出现非有限值、或者压根还没标定——那几种情况下 ~/frame 都是不发的。
# 把按键挂在 ~/frame 上等于让急停依赖动捕是好的，正好反了。
#
# 本包**不解释**这些按键的语义。「双手同时按 B/Y 推进状态机」那种约定属于下游。

std_msgs/Header header

MocapController left
MocapController right
