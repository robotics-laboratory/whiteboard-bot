import websockets
import asyncio
from pynput import keyboard
import sys

class Controller:
    def __init__(self):
        self.direction = 0
        self.increment = 10
        self.last_state = "0"

    def rotate(self, d):
        if d == 1 and self.direction < 90:
            self.direction = min(self.direction + self.increment, 90)
        elif d == -1 and self.direction > -90:
            self.direction = max(self.direction - self.increment, -90)

    async def init_ip(self, ip):
        print("Connecting...")
        self.ws = await websockets.connect("wss://"+ip+"/ws", port=80, ssl=False)
        print("Connected")

    async def send(self, msg):
        if msg == "left":
            self.rotate(-1)
            msg = self.last_state
        elif msg == "right":
            self.rotate(1)
            msg = self.last_state

        self.last_state = msg
        await self.ws.send(msg + ";" + str(self.direction + 90) + ";")

ctr = Controller()

keys_bind = {"Key.left": "left", "Key.right": "right", "Key.up": "0", "Key.down": "1"}

def init_listener():
    queue = asyncio.Queue()
    loop = asyncio.get_event_loop()

    def press_event(key):
        action = keys_bind.get(str(key))
        if action is not None:
            loop.call_soon_threadsafe(queue.put_nowait, action)

    def release_event(key):
        loop.call_soon_threadsafe(queue.put_nowait, "2")

    listener = keyboard.Listener(
        on_press=press_event,
        on_release=release_event)
    
    listener.start()

    return queue

async def main():
    ip = sys.argv[1]
    await ctr.init_ip(ip)

    queue = init_listener()
    while True:
        msg = await queue.get()
        await ctr.send(msg)

if __name__ == "__main__":
    asyncio.run(main())

