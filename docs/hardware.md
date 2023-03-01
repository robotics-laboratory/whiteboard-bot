# Compiling hardware

## Requirements

* Platformio framework (platformio-cli)

## Environment

Open `.env` file and insert WiFi SSID and password

## Build

`pio run --target upload` - Build and upload the firmware (may be on another port)

## Troubleshooting

#### In case of permission denied errors, you need to install board udev rules

[Link to the documentation](https://docs.platformio.org/en/stable/core/installation/udev-rules.html)

A quick fix would be to `sudo chown $USER /dev/ttyUSB0`, but you would need to execute it each time you connect the board.
