# Compiling hardware

## Requirements

* Platformio framework (platformio-cli)

## Environment

Open `hardware/wbb/.env` and insert WiFi SSID and password

## Build

`cd hardware/wbb`

`pio run --target upload --upload-port /dev/ttyUSB0` - Build and upload the firmware (may be on another port)

## Troubleshooting

#### In case of permission denied errors, you need to install board udev rules

`curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules`

For most of the systems:

`sudo cp ./99-platformio-udev.rules /etc/udev/rules.d/99-platformio-udev.rules`

For systems with udev rules placed in /lib/udev/rules.d:

`sudo cp ./99-platformio-udev.rules /lib/udev/rules.d/99-platformio-udev.rules`

```
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Ubuntu/Debian:

```
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER
```

Arch:

```
sudo usermod -a -G uucp $USER
sudo usermod -a -G lock $USER
```

Logout and reconnect the board to apply the changes

A quick fix would be to `sudo chown $USER /dev/ttyUSB0`, but you would need to execute it each time you connect the board.
