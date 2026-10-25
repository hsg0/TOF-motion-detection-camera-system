# GPS

Source: `gps-gt-u7.ino`
Runs on: Uno Q STM32, inside `motorController`

The receiver is a GT-U7 (u-blox 7). This file owns `Serial1`. Nothing else should read that port. TinyGPSPlus parses the NMEA sentences into a `GpsData` structure: position, altitude, ground speed, course, satellite count, HDOP, UTC, fix validity, and link health.

| GT-U7 | Uno Q |
|---|---|
| TXD | D0, `Serial1` RX, 9600 baud |
| micro-USB red | 5 V |
| micro-USB black | GND |
| RXD | Leave disconnected |
| PPS | Leave disconnected |

D1 is the MCU console transmit pin. Do not connect it to the receiver. Do not connect the USB green or white wires to D0 or D1. The black wire must share ground with the Uno Q.

The OLED first line shows latitude and longitude when a fix is valid. With no fix it shows zeros plus satellites in use and satellites in view. A clock lock and a satellite count can be real while the coordinates are still empty. A fix needs enough satellites in the open. The module marks the fix stale about 2 seconds after the receiver drops it.
