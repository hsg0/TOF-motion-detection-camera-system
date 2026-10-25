# Barometer

Source: `baro-pressure-bme280.ino`
Runs on: Uno Q STM32, inside `motorController`

The BME280 is chained on Qwiic, `Wire1`, with the IMU module. The address in use is `0x77`. `0x76` is the alternate if the board is strapped that way.

The module reports pressure, temperature, and a relative altitude after it has a baseline. The OLED third line shows pressure in hectopascals next to the Pico distance. The dashboard shows the same pressure, temperature, and altitude from the serial summary.
