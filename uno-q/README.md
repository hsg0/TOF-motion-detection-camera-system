# Arduino Uno Q

The Uno Q is the sensor computer. It has two processors in one board, connected to the Pi by one USB cable.

| Side | Folder | What runs |
|---|---|---|
| Linux MPU | `linux-mpu` | The USB gadget and the serial-monitor proxy |
| STM32U585 | `mcu` | The `motorController` sketch and every sensor module |

The Qwiic sensors, the GPS, the OLED, and the Pico link are all code on the STM32. They are folders under `mcu`, because that is where those files execute.
