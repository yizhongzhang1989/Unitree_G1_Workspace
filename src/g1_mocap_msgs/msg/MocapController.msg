# 一只手柄的按键与摇杆。位姿不在这里——需要的时候再加，现在没有消费者。

# 手柄有没有连上。为 false 时下面的值全部无意义。
bool connected

# 扳机与侧握的模拟量，0~1。
float32 trigger
float32 squeeze

bool trigger_click
bool squeeze_click
bool thumbstick_pressed

# 右手是 A/B，左手是 X/Y——同一个物理位置，PICO 就是这么命名的。
bool a_x
bool b_y

# 只有左手柄有。
bool menu

# 摇杆 x, y，各 -1~1。
float32[2] thumbstick

# 看着像 0~5 的档位，不是 0~1。
float32 battery
