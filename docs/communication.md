# Controlling the bot using websockets

## Connecting

The server is running on `<ip>/ws` on port 80.

Note: on the local network it usually appears to be `192.168.1.121`

## Movement

The movement command has following structure:

`<code>;<direction>;`

`code`: 0 to move forward, 1 to move backward, 2 to stop. The bot continues to move until the stop command is received

`direction`: from 0 to 180, where 0 makes full left rotation, 90 makes forward movement and 180 makes full right rotation