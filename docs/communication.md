# Controlling the bot using websockets

## Connecting

The server is running on `<ip>/ws` on port 80.

Note: on the local network it usually appears to be `192.168.1.121`

## Movement

The movement command has the following structure:

`<command>;<parameters>`

`command`: (int) 0 to move, 1 to move the eraser up, 2 to move it down

### Movement command:

`0;<curvature>;<velocity>`

`curvature`: (float, precision: 6) 1/radius, 0.0 means going forward

`velocity`: (float, precision: 6) from -1.0 to 1.0

### Eraser commands:

`1;`: move eraser up

`2;`: move eraser down