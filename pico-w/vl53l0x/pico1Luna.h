/*
WHAT:
  The TF-Luna distance reading for Pico 1.

WHY:
  Pico 1 carries one TF-Luna. The flight controller decides what the
  distance means. This file only collects sensor data and publishes the
  latest sample.

HOW:
  Luna data pins on Pico GP4 and GP5 (physical pins 6 and 7):

    Luna pin 1  power  -> Pico pin 39  VSYS  (or 40 VBUS when USB-powered)
    Luna pin 2  data   -> Pico pin 6   GP4
    Luna pin 3  data   -> Pico pin 7   GP5
    Luna pin 4  GND    -> Pico pin 38  GND
    Luna pin 5  mode   -> GND for I2C, open for UART, before power-up
    Luna pin 6  ready  -- left open

  Firmware tries I2C (both wire orders), then UART1 on GP4/GP5, then
  UART listen on GP4 if sensor TX is swapped onto that pin.

IMPORTANT:
  Do not invent a distance. A failed read leaves fresh false.
  A sample older than 250 ms is not fresh.
*/

#pragma once

#include <cstdint>

struct LunaReading {
  std::uint16_t distanceCentimeters;
  std::uint16_t signalStrength;
  std::int32_t temperatureDecidegrees;
  bool fresh;
};

void setupLuna();
void updateLuna();
LunaReading currentLunaReading();
