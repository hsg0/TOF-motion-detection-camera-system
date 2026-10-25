# OLED

Source: `oledDisplay.ino`
Runs on: Uno Q STM32, inside `motorController`

The display is an Adafruit FeatherWing 4650, an SH1107, 128×64, on Qwiic `Wire1` at `0x3C` (`0x3D` is the alternate). It only shows state. It does not read a sensor of its own.

Each command byte goes out as its own I2C packet. The picture is sent 32 bytes per loop pass so the IMU is not stalled.

| Line | Source |
|---|---|
| 1 | GPS module. Coordinates, or zeros and satellite counts |
| 2 | IMU acceleration |
| 3 | Barometer pressure and Pico distance |
| 4 | Camera on or off, from the byte the Pi bridge sends |

Line 4 drops to off if that heartbeat is missing for about 3.5 seconds.
