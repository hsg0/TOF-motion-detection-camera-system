# Pico W

Pico 1 is a Raspberry Pi Pico W. It has one job: read the VL53L0X and answer the Uno Q on SPI. It does not talk to the Pi, and it does not command motors.

The program is C++ with the Pico SDK, built for `PICO_BOARD=pico_w`, and flashed as `pico1Main.uf2` in BOOTSEL mode. The boot drive is named `RPI-RP2`.

Source tree: `FLIGHT-CONTROLLER/UNO-Q/MCU/PICO-1`

| Folder here | Source file | Role |
|---|---|---|
| this folder | `pico1Main.cpp` | Setup, then sensor update and SPI service |
| `vl53l0x` | `PICO-1-SENSORS/pico1Luna.cpp` | Distance sensor |
| `spi-slave` | `pico1SpiLink.cpp` | Reply toward the Uno Q |

`pico1Main.cpp` calls the sensor update and the SPI update. The two folders below are the two files it uses.
