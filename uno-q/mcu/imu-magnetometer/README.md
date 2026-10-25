# IMU and magnetometer

Source: `imu-lsm6dsox-lis3mdl-qwiic.ino`
Runs on: Uno Q STM32, inside `motorController`

| Chip | Address | Measures |
|---|---|---|
| LSM6DSOX | `0x6A` | Acceleration and rotation |
| LIS3MDL | `0x1C` | Magnetic field |

Both chips are on the Qwiic connector, `Wire1`. The BME280 and the OLED share that same bus. The main sketch calls `Wire1.begin()` once.

The module publishes acceleration, sample rates, FIFO health, and attitude and heading health. The OLED second line is the acceleration. The dashboard reads the same summary line from the serial text.
