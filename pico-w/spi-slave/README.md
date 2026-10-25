# SPI slave

Source: `pico1SpiLink.cpp` and `pico1SpiLink.h`
Runs on: Pico W, called from `pico1Main.cpp`

Bit-banged SPI slave, mode 0, most significant bit first. The reply byte is set up while the Uno Q clock is low.

| Pico W | Uno Q |
|---|---|
| GP12 | JSPI MOSI, D23 |
| GP11 | JSPI MISO, D22 |
| GP10 | JSPI SCK, D24 |
| GP9 | D10 chip select, active low |

The frame is the same 8 bytes `pico1Link.ino` expects: two `0xA5` sync bytes, centimeters, millimeters, a fresh flag, and the low 8 bits of the sum of the earlier bytes. Change one side only and the Uno Q will reject every frame.

The master that clocks this slave is `uno-q/mcu/pico-link`.
