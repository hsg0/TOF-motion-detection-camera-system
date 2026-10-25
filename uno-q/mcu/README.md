# STM32U585 sketch

Source tree: `FLIGHT-CONTROLLER/UNO-Q/MCU/motorController` in the flight-controller project.

The sketch is one program with one `setup` and one `loop` in `motorController.ino`. The other files are modules. They do not contain their own `setup` or `loop`. The main file calls each module's setup function once and its update function every pass.

| Folder here | Source file | Bus |
|---|---|---|
| `imu-magnetometer` | `imu-lsm6dsox-lis3mdl-qwiic.ino` | Qwiic, `Wire1` |
| `barometer` | `baro-pressure-bme280.ino` | Qwiic, `Wire1` |
| `oled` | `oledDisplay.ino` | Qwiic, `Wire1` |
| `gps` | `gps-gt-u7.ino` | `Serial1` on D0 |
| `pico-link` | `pico1Link.ino` | JSPI, bit-banged |

`%f` does not print on this toolchain. The modules format decimals with integer arithmetic.

A failure in one module must not stop the others. GPS in particular is not required for the IMU or the barometer.

The sketch prints one-line summaries to the serial monitor. That text is what the Pi bridge stores and what the dashboard displays.

Each module folder holds a copy of its `.ino` file. The Arduino build treats every `.ino` in one directory as a single sketch, so a rebuild means gathering `motorController.ino` and the five module files into one sketch folder. The copies here are the backup, split the same way as the notes.
