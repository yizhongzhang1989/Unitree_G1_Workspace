"""Gloria-M CAN 反馈帧归属判断，不依赖 ROS"""

from typing import Optional, Sequence


REGISTER_REPLY_OPCODES = (0x33, 0x55)


def is_register_reply(data: Sequence[int], command_id: Optional[int] = None) -> bool:
    """按地址前缀和操作码判断寄存器读写回包"""
    if len(data) != 8 or int(data[2]) not in REGISTER_REPLY_OPCODES:
        return False
    # 不同固件可能使用保留地址 0，也可能回显设备命令地址
    address = int(data[0]) | (int(data[1]) << 8)
    return (
        address == 0
        or command_id is not None and address == command_id
    )


def register_reply_belongs_to_device(can_id: int, command_id: int, feedback_id: int) -> bool:
    """寄存器回包没有 payload 设备号，只接受设备的非零专属 CAN ID"""
    return can_id != 0 and can_id in (command_id, feedback_id)


def state_feedback_belongs_to_device(
    can_id: int, data: Sequence[int],
    command_id: int, feedback_id: int
) -> bool:
    """按仲裁 ID 和 Data[0] 低 4 位设备号判断状态帧归属"""
    if len(data) != 8 or can_id not in (feedback_id, command_id, 0x00):
        return False
    # Data[0] 高 4 位是设备状态，不能参与设备号比较
    return (int(data[0]) & 0x0F) == (command_id & 0x0F)