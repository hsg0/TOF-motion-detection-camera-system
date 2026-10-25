# Pico link (Uno Q side)

Source: `pico1Link.ino`
Runs on: Uno Q STM32, inside `motorController`

The Pico is on the JSPI header. The chip's hardware SPI is fixed to D11, D12, and D13, so this file clocks the JSPI pins itself, mode 0, once a second.

| Uno Q | Pico W |
|---|---|
| JSPI MOSI, D23 | GP12 |
| JSPI MISO, D22 | GP11 |
| JSPI SCK, D24 | GP10 |
| D10 chip select | GP9 |
| JSPI 5 V | VSYS |
| JSPI GND | GND |

JSPI pin 5 is reset. This file does not drive it.

The reply is 8 bytes: `0xA5 0xA5`, distance in centimeters (low byte first), a 16-bit field that is the distance in millimeters, a fresh flag, and a checksum. A bad frame is dropped. The telemetry label for that 16-bit field says strength. It is millimeters, not an optical signal level.

The matching Pico code is `pico-w/spi-slave`.
