# Sensor stack

This is the bench setup that ties the flight-controller sensors, the vision computer, and a USB cellular modem into one system. It collects attitude, pressure, GPS, and range on the controller board, shows them on a small display, and lets the Pi watch the camera and publish the same telemetry. It does not fly the aircraft.

Account names, passwords, phone numbers, and the cellular provider are not written here. Those live only in the local config on the Pi.

The folders next to this file are the project layout. Each folder is one board, or one piece of code on that board. The notes inside say which source file that piece is, and where that file actually runs.

```text
oct-25/
  raspberry-pi-5/
    camera-imx500/
    ai-hat-hailo-10h/
    dashboard/
    unoq-bridge/
    cellular-modem/
  uno-q/
    linux-mpu/
    mcu/
      imu-magnetometer/
      barometer/
      gps/
      oled/
      pico-link/
  pico-w/
    vl53l0x/
    spi-slave/
```

## How the pieces fit

```text
Pi 5  ---- USB ----  Uno Q Linux MPU ---- serial bridge ---- STM32U585
  |                                                      |
  | camera + NPU                                         | Qwiic (Wire1)
  |                                                      |   IMU + magnetometer
  | USB                                                  |   barometer
  |                                                      |   OLED
  cellular modem                                         |
                                                         | Serial1
                                                         |   GPS
                                                         |
                                                         | JSPI
                                                         |   Pico W ---- I2C ---- VL53L0X
```

The Pi is the vision computer and the dashboard. The Uno Q is the sensor board. The Pico only reads the distance sensor and hands that reading to the Uno Q.

## Raspberry Pi 5

The Pi runs the camera, the neural processor, the dashboard, and the link to the Uno Q. It is powered from a 5 V, 5 A USB-C supply. A weaker supply undervolts the board when the camera starts.

| Part | Role |
|---|---|
| Raspberry Pi 5 | Host computer |
| Sony IMX500 | Camera on the CSI connector |
| Hailo-10H on the AI HAT+ 2 | Runs the person detector |
| SIM7600G-H | USB cellular modem |

Software on the Pi:

| Software | What it does |
|---|---|
| Raspberry Pi OS | Host system |
| `libcamera` and `picamera2` | Opens the IMX500 at 1280×720 |
| `hailo-h10-all` 5.1.1 and `python3-h10-hailort` | Drives `/dev/hailo0` |
| `yolov11m_h10.hef` | Person and object detection |
| `watcher.py` | Dashboard, detection, and alert mail |
| `unoq_bridge.py` | Reads Uno Q telemetry and tells the OLED whether the camera is on |
| `adb` | Reaches the Uno Q Linux side over USB |
| Ollama | Local model runtime. It is installed and is not part of the sensor loop |
| ModemManager and `libqmi` | Talks to the cellular modem |
| NetworkManager | Brings up Wi-Fi and the cellular data session |
| `modem-manager-gui` | Desktop window for the modem |

`watcher.py` and `unoq_bridge.py` are started by hand. They do not start on their own after a reboot.

The dashboard listens on port 8080. The left side is the live camera with detection boxes. The right side is the telemetry from the Uno Q: time, GPS, IMU, barometer, distance, a copy of the four OLED lines, and power. A snapshot is mailed only when the detector sees a person and the distance sensor has stayed at or inside 30 cm for 2 seconds. Mail is sent through SMTP. The recipient is stored in a local config file and is not copied into this note. Repeat mail is held for 100 seconds.

`unoq_bridge.py` forwards the Uno Q serial monitor and writes the latest telemetry text for the dashboard. About once a second it sends `C` while the camera program is running and `c` when it is not. The OLED uses that for the camera line.

## Cellular modem

The SIM7600G-H plugs into a USB port on the Pi. The kernel exposes serial ports `ttyUSB0` through `ttyUSB4` and a data interface, `cdc-wdm0` / `wwan0`. ModemManager uses the QMI port. NetworkManager keeps a cellular connection configured.

Wi-Fi is the preferred route while it is connected. The cellular route has a worse metric, so it carries traffic only when Wi-Fi is down. The data session can stay up in the background so the switch is immediate. Voice calls are started from ModemManager on the command line. This modem does not present a USB sound card. Call audio is on the modem board's own headset jack, not on the Pi.

The provider, the data APN, and any phone number are omitted here.

## Arduino Uno Q

The Uno Q has two processors. A Linux MPU enumerates over USB. An STM32U585 runs the sensor sketch. The Pi uses `adb` to reach a serial-monitor proxy on the Uno Q. The sketch prints one-line sensor summaries on that monitor. The bridge on the Pi saves them.

The sketch is the `motorController` program. It has no `setup` or `loop` of its own inside the sensor files. The main file calls each sensor's setup and update function. Float formatting with `%f` does not work in this toolchain, so displayed decimals are built with integer arithmetic.

| File | Owns |
|---|---|
| `imu-lsm6dsox-lis3mdl-qwiic.ino` | Accelerometer, gyro, magnetometer |
| `baro-pressure-bme280.ino` | Pressure, temperature, altitude |
| `gps-gt-u7.ino` | GPS UART and NMEA parsing |
| `oledDisplay.ino` | Four-line status display |
| `pico1Link.ino` | SPI master toward the Pico |
| `motorController.ino` | Calls the modules above |

### Qwiic bus

`Wire1` is the Qwiic connector. These boards share it:

| Device | Address | Role |
|---|---|---|
| LSM6DSOX | `0x6A` | Accelerometer and gyroscope |
| LIS3MDL | `0x1C` | Magnetometer |
| BME280 | `0x77` (alternate `0x76`) | Barometer |
| Adafruit FeatherWing 4650, SH1107 OLED | `0x3C` (alternate `0x3D`) | 128×64 display |

The IMU module and the BME280 are chained on this connector. The OLED is on the same bus. The sketch starts `Wire1` once. A sensor that does not answer does not stop the others. GPS is not on this bus, and a GPS failure must not stop the IMU or the barometer.

The OLED is an SH1107. Each command byte is sent as its own I2C packet. The framebuffer is pushed 32 bytes at a time so a full-screen refresh does not stall the IMU. The four lines are:

1. GPS latitude and longitude, or zeros plus satellites in use and in view when there is no fix.
2. IMU acceleration.
3. Barometer pressure and distance in centimeters.
4. `CAM: ON` or `CAM: OFF`, from the Pi heartbeat. The line goes off if that heartbeat is missing for about 3.5 seconds.

### GPS

The receiver is a GT-U7, a u-blox 7, talking NMEA at 9600 baud. The sketch parses it with TinyGPSPlus.

| GT-U7 | Uno Q |
|---|---|
| TXD | D0, `Serial1` RX |
| micro-USB red (5 V) | 5 V |
| micro-USB black | GND |

Leave the receiver RXD and PPS disconnected. D1 is the Uno Q console transmit pin. If it is wired to the receiver, boot text corrupts the NMEA stream. Do not connect the USB data pair (green and white) to D0 or D1. Those wires are not a UART. The black wire has to land on Uno Q ground or the receiver has power and still delivers nothing.

The sketch reports bytes received, good and bad checksums, satellites in use, satellites in view, and the strongest signal. A position needs a fix. Time and a satellite count can be valid while latitude and longitude are still empty. Indoors that is the usual state. The last numbers are kept for logging, and they are marked stale about 2 seconds after the receiver drops the fix.

### Link to the Pico

The Pico is on the JSPI header, not on the normal SPI pins D11–D13. The hardware SPI driver cannot see this header, so `pico1Link.ino` bit-bangs the clock. It polls once a second, SPI mode 0.

| Uno Q | Pico W |
|---|---|
| JSPI MOSI, D23 | GP12 |
| JSPI MISO, D22 | GP11 |
| JSPI SCK, D24 | GP10 |
| D10, chip select | GP9 |
| JSPI 5 V | VSYS |
| JSPI GND | GND |

JSPI pin 5 is reset. The sketch does not drive it.

The Pico answers with 8 bytes: two sync bytes `0xA5`, distance in centimeters (low byte first), a second 16-bit field that currently carries the distance in millimeters, a fresh flag, and a checksum. A bad checksum is discarded. On the telemetry line that second field is labeled strength. It is millimeters, not an optical signal level.

## Pico W

Pico 1 is a Raspberry Pi Pico W. The program is C++ built with the Pico SDK for `PICO_BOARD=pico_w`, and it is flashed as `pico1Main.uf2` while the board is in BOOTSEL mode. The boot drive is named `RPI-RP2`.

| File | Role |
|---|---|
| `pico1Main.cpp` | Main loop. Reads the sensor and serves SPI. Does not command motors |
| `pico1Luna.cpp` | VL53L0X driver |
| `pico1SpiLink.cpp` | Bit-banged SPI slave |

The distance sensor is a VL53L0X at I2C address `0x29` on `i2c0`.

| VL53L0X | Pico W |
|---|---|
| SDA | GP0, physical pin 1 |
| SCL | GP1, physical pin 2 |
| VIN | 3V3, physical pin 36 |
| GND | GND |

There is no TF-Luna lidar on this Pico. The firmware accepts a range only when the sensor reports a valid status, and only between 30 mm and 2000 mm. Weak returns are rejected by raising the signal floor above the chip default. The published distance is the median of the last five valid samples. A sample older than half a second is not fresh.

## What is running where

| Work | Where it runs |
|---|---|
| Camera, person detection, dashboard, mail | Pi, `watcher.py` |
| Telemetry copy and camera heartbeat | Pi, `unoq_bridge.py` |
| IMU, magnetometer, barometer, GPS, OLED, Pico poll | Uno Q STM32 sketch |
| Distance sensor and SPI reply | Pico W |
| Cellular data session | Pi, ModemManager and NetworkManager |

Source for the Pi programs is `oct-24/watcher.py` and `oct-24/unoq_bridge.py` in this project. The copies that the Pi runs are under the Desktop `scripts/oct-24` folder on that machine. The Uno Q and Pico sources are in the flight-controller tree: `FLIGHT-CONTROLLER/UNO-Q/MCU/motorController` and `FLIGHT-CONTROLLER/UNO-Q/MCU/PICO-1`.
# TOF-motion-detection-camera-system
