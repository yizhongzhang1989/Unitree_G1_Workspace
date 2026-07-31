"""机器人端消费示例：订阅位姿流，把右手柄 grip 位姿映射成增量指令。

用法：
    python example_consumer.py --url ws://localhost:8000/ws/subscribe
"""

from __future__ import annotations

import argparse
import asyncio
import json

import aiohttp

# WebXR 坐标系：右手系，Y 轴向上，-Z 为正前方（面朝方向）
# 常见机器人基坐标系：X 前、Y 左、Z 上
def webxr_to_robot(p: list[float]) -> tuple[float, float, float]:
    x, y, z = p
    return (-z, -x, y)


async def main(url: str) -> None:
    async with aiohttp.ClientSession() as sess:
        async with sess.ws_connect(url) as ws:
            origin = None
            print("connected, 按住右手柄扳机开始跟随")
            async for msg in ws:
                if msg.type is not aiohttp.WSMsgType.TEXT:
                    continue
                frame = json.loads(msg.data)
                right = frame.get("right")
                if not right or not right.get("grip"):
                    continue

                pos = webxr_to_robot(right["grip"]["position"])
                trigger = right.get("buttons", {}).get("trigger", 0.0)
                gripper = right.get("buttons", {}).get("squeeze", 0.0)

                if trigger > 0.5:
                    if origin is None:
                        origin = pos          # 扳机按下瞬间锁定参考原点
                    delta = tuple(round(c - o, 4) for c, o in zip(pos, origin))
                    print(f"delta_xyz={delta}  gripper={gripper:.2f}")
                    # TODO: 在这里下发到你的机器人（IK / 伺服 / SDK 调用）
                else:
                    origin = None


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--url", default="ws://localhost:8000/ws/subscribe")
    asyncio.run(main(p.parse_args().url))
