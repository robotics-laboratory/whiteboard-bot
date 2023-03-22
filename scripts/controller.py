import argparse
import asyncio
import sys
from enum import IntEnum

import websockets
from pynput import keyboard


class Dir(IntEnum):
    FWD = 0
    BACK = 1
    STOP = 2
    LEFT = 3
    RIGHT = 4


class Controller:
    def __init__(self):
        self.direction = 0
        self.radius = 10000
        self.increment = 10
        self.last_state = Dir.FWD

    def rotate(self, d):
        if d == 1:
            self.radius = 0.00001
        else:
            self.radius = -0.00001
        
        #if d == 1 and self.radius < 10000:
        #    self.radius += 100
        #elif self.radius > -10000:
        #    self.radius -= 100
        
        #if d == 1 and self.direction < 90:
        #    self.direction = min(self.direction + self.increment, 90)
        #elif d == -1 and self.direction > -90:
        #    self.direction = max(self.direction - self.increment, -90)

    async def init_ip(self, ip):
        print("Connecting...")
        self.ws = await websockets.connect("wss://" + ip + "/ws", port=80, ssl=False)
        print("Connected")

    async def send(self, msg):
        if msg == Dir.LEFT:
            self.rotate(-1)
            msg = self.last_state
        elif msg == Dir.RIGHT:
            self.rotate(1)
            msg = self.last_state
        else:
            self.radius = 10000

        if msg == Dir.FWD or msg == Dir.BACK:
            self.last_state = msg
        
        if self.radius == 0:
            self.radius = 1
        await self.ws.send(str(int(msg)) + ";" + str(round(1/self.radius, 6)) + ";")


ctr = Controller()

keys_bind = {
    "Key.left": Dir.LEFT,
    "Key.right": Dir.RIGHT,
    "Key.up": Dir.FWD,
    "Key.down": Dir.BACK,
}


def init_listener():
    queue = asyncio.Queue()
    loop = asyncio.get_event_loop()

    def press_event(key):
        action = keys_bind.get(str(key))
        if action is not None:
            loop.call_soon_threadsafe(queue.put_nowait, action)

    def release_event(key):
        loop.call_soon_threadsafe(queue.put_nowait, Dir.STOP)

    listener = keyboard.Listener(on_press=press_event, on_release=release_event)

    listener.start()

    return queue


async def main(args):
    ip = args.ip
    await ctr.init_ip(ip)

    queue = init_listener()
    while True:
        msg = await queue.get()
        await ctr.send(msg)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="controller.py", description="Manually control the whiteboard bot"
    )
    parser.add_argument("-i", "--ip", required=True, help="Bot IP address")
    args = parser.parse_args()

    asyncio.run(main(args))
