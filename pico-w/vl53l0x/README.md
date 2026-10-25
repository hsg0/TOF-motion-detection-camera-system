# VL53L0X

Source: `PICO-1-SENSORS/pico1Luna.cpp` and `pico1Luna.h`
Runs on: Pico W, called from `pico1Main.cpp`

I2C0 at address `0x29`, 100 kHz.

| VL53L0X | Pico W |
|---|---|
| SDA | GP0, physical pin 1 |
| SCL | GP1, physical pin 2 |
| VIN | 3V3, physical pin 36 |
| GND | GND |

There is no TF-Luna on this board. A sample is kept only when the range status is valid and the distance is between 30 mm and 2000 mm. The signal floor is set above the chip default so weak reflections are dropped. The distance sent to the Uno Q is the median of the last five valid samples. A reading older than half a second is not fresh.

The Uno Q side of this number is `uno-q/mcu/pico-link`.
