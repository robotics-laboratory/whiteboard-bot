#!/usr/bin/env python

import asyncio
from enum import IntEnum

import rclpy
import websockets
from rclpy.node import Node

from wbb_msgs.msg import Control, Eraser


class Command(IntEnum):
    MOVE = 0
    ERASER_UP = 1
    ERASER_DOWN = 2


class ControlNode(Node):
    def __init__(self):
        super(ControlNode, self).__init__("control")
        self.declare_parameter("esp32_ip", "192.168.1.121")
        self.move_sub = self.create_subscription(Control, "/movement", self.send, 10)
        self.eraser_sub = self.create_subscription(Eraser, "/eraser", self.send, 10)
        self.eraser_up = True
        self.lock = asyncio.Lock()

    async def init_conn(self):
        ip = self.get_parameter("esp32_ip").value
        self.ws = await websockets.connect("wss://" + ip + "/ws", port=80, ssl=False)

    async def send(self, msg):
        await self.ws.send(
            str(int(Command.MOVE)) + ";" + str(round(float(msg.curvature), 6)) + ";" + str(round(float(msg.velocity), 6))
        )

    async def move_eraser(self, msg):
        async with self.lock:
            if msg.toggle:
                self.eraser_up = not self.eraser_up
            else:
                self.eraser_up = msg.up

        cmd = Command.ERASER_DOWN
        if self.eraser_up:
            cmd = Command.ERASER_UP

        await self.ws.send(
            str(int(Command.MOVE)) + ";" + str(round(float(msg.curvature), 6)) + ";" + str(
                round(float(msg.velocity), 6))
        )


async def loop(node):
    await node.init_conn()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(0.0001)


def main():
    rclpy.init()
    node = ControlNode()
    asyncio.run(loop(node))


if __name__ == "__main__":
    main()
