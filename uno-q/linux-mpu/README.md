# Uno Q Linux side

The Linux MPU enumerates on the Pi as a USB device. `adb` on the Pi reaches it. An on-board router exposes the MCU serial monitor at port 7500. The Pi bridge forwards that port and is the only program that reads it.

No sensor driver runs on this Linux side. IMU, barometer, GPS, OLED, and the Pico poll are all on the STM32.
