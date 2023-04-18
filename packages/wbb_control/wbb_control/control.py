#!/usr/bin/env python

import asyncio
from enum import IntEnum

import rclpy
import websockets
from rclpy.node import Node

from wbb_msgs.msg import Control

rclpy.init()


class Direction(IntEnum):
    FWD = 0
    BACK = 1
    STOP = 2


class ControlNode(Node):
    def __init__(self):
        super(ControlNode, self).__init__("control")
        self.declare_parameter("esp32_ip", "192.168.1.121")
        self.sub = self.create_subscription(Control, "movement", self.send, 10)

    async def init_conn(self):
        ip = self.get_parameter("esp32_ip").get_parameter_value().string_value
        self.ws = await websockets.connect("wss://" + ip + "/ws", port=80, ssl=False)

    async def send(self, msg):
        d = Direction.STOP
        if msg.velocity > 0:
            d = Direction.FWD
        elif msg.velocity < 0:
            d = Direction.BACK

        await self.ws.send(
            str(int(d)) + ";" + str(round(float(msg.curvature), 6)) + ";"
        )


node = ControlNode()


async def loop():
    await node.init_conn()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(0.0001)


def main():
    asyncio.run(loop())


if __name__ == "__main__":
    main()
