/*
WHAT:
  The Pico 1 side of the SPI link to the Uno Q flight controller.

WHY:
  The flight controller has to be able to tell that this board is alive
  before it trusts any sensor reading from it. This file is that answer.

HOW:
  The Uno Q owns the clock. This board is the slave. While chip select
  is low the Uno clocks eight bytes:

    0..1   0xA5 0xA5
    2..3   distance in centimeters, low byte first
    4..5   signal strength, low byte first
    6      1 when the Luna sample is fresh, otherwise 0
    7      checksum, the low 8 bits of the sum of bytes 0..6

  updatePico1Spi() watches the clock and chip-select pins. It never
  waits, so the TF-Luna reader gets a turn on every pass.

IMPORTANT:
  Chip select is required. GP11 stays silent while chip select is high.
  An open chip-select wire, or a Pico that is not running, makes the
  Uno read FF.

  The clock rests low. The bit is already on GP11 before the clock
  rises, which is when the Uno reads it. The next bit is placed when
  the clock falls.

  The hardware SPI block was loaded with 0xA5 and the Uno read 0xAA.
  These pins are driven directly so the bit on the wire is the bit
  in 0xA5.

  JSPI pin 5 is the Uno Q reset pin. Do not connect it.
  This board must never command motors.

  setPico1LunaSample() stores the next reply. The reply already in
  progress is left alone until chip select rises.
*/

#pragma once

#include <cstdint>

void setupPico1Spi();
void updatePico1Spi();
void setPico1LunaSample(
    std::uint16_t distanceCentimeters,
    std::uint16_t signalStrength,
    bool fresh);
