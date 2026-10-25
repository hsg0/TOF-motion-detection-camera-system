/*
WHAT:
  Bit-by-bit SPI slave that answers the Uno Q on GPIO pins.

WHY:
  The reply has to be the byte 0xA5, with each bit stable before the
  Uno's clock rises. Placing the bit from this loop makes that timing
  ours.

HOW:
  Mode 0, 8 bits, most significant bit first.

    Uno Q JSPI pin 4  MOSI  -> GP12  input
    Uno Q JSPI pin 1  MISO  <- GP11  reply output
    Uno Q JSPI pin 3  SCK   -> GP10  input
    Uno Q D10         SS    -> GP9   input, active low

  GP11 is released while chip select is high. When chip select falls,
  the eight-byte reply is copied and bit 7 of its first byte is driven.
  Each time the clock falls, the next bit is driven.

IMPORTANT:
  updatePico1Spi() has to run again while the clock is still low, so
  the next bit is waiting before the clock rises. The Uno keeps the
  clock low long enough. This file never waits for a byte.
*/

#include "pico1SpiLink.h"

#include "hardware/gpio.h"

constexpr uint PICO1_RX_PIN = 12;
constexpr uint PICO1_TX_PIN = 11;
constexpr uint PICO1_SCK_PIN = 10;
constexpr uint PICO1_CS_PIN = 9;

// Eight bytes, agreed with pico1Link.ino. Do not change one side only.
constexpr uint8_t PICO1_FRAME_BYTES = 8;
constexpr uint8_t PICO1_FRAME_BITS = PICO1_FRAME_BYTES * 8;
constexpr uint8_t PICO1_SYNC_BYTE = 0xA5;

uint8_t stagedFrame[PICO1_FRAME_BYTES] = {PICO1_SYNC_BYTE, PICO1_SYNC_BYTE, 0, 0, 0, 0, 0, 0x4A};
uint8_t activeFrame[PICO1_FRAME_BYTES] = {PICO1_SYNC_BYTE, PICO1_SYNC_BYTE, 0, 0, 0, 0, 0, 0x4A};
uint8_t replyBitNumber = 0;
bool chipSelected = false;
bool clockWasHigh = false;


void writeLunaFrame(
    uint8_t frame[PICO1_FRAME_BYTES],
    uint16_t distanceCentimeters,
    uint16_t signalStrength,
    bool fresh
) {
  frame[0] = PICO1_SYNC_BYTE;
  frame[1] = PICO1_SYNC_BYTE;
  frame[2] = static_cast<uint8_t>(distanceCentimeters & 0xFF);
  frame[3] = static_cast<uint8_t>(distanceCentimeters >> 8);
  frame[4] = static_cast<uint8_t>(signalStrength & 0xFF);
  frame[5] = static_cast<uint8_t>(signalStrength >> 8);
  frame[6] = fresh ? 1 : 0;

  uint8_t checksum = 0;
  for (uint8_t index = 0; index < PICO1_FRAME_BYTES - 1; index++) {
    checksum = static_cast<uint8_t>(checksum + frame[index]);
  }
  frame[7] = checksum;
}


void driveReplyBit() {
  uint8_t bitValue = 0;
  if (replyBitNumber < PICO1_FRAME_BITS) {
    uint8_t frameByte = activeFrame[replyBitNumber / 8];
    uint8_t bitInByte = 7 - (replyBitNumber % 8);
    bitValue = (frameByte >> bitInByte) & 1;
  }
  gpio_put(PICO1_TX_PIN, bitValue);
  gpio_set_dir(PICO1_TX_PIN, GPIO_OUT);
}


void releaseReplyPin() {
  gpio_set_dir(PICO1_TX_PIN, GPIO_IN);
  gpio_disable_pulls(PICO1_TX_PIN);
  replyBitNumber = 0;
}


void setupPico1Spi() {
  gpio_init(PICO1_RX_PIN);
  gpio_set_dir(PICO1_RX_PIN, GPIO_IN);
  gpio_pull_down(PICO1_RX_PIN);

  gpio_init(PICO1_SCK_PIN);
  gpio_set_dir(PICO1_SCK_PIN, GPIO_IN);
  gpio_pull_down(PICO1_SCK_PIN);

  gpio_init(PICO1_CS_PIN);
  gpio_set_dir(PICO1_CS_PIN, GPIO_IN);
  gpio_pull_up(PICO1_CS_PIN);

  gpio_init(PICO1_TX_PIN);
  gpio_set_dir(PICO1_TX_PIN, GPIO_IN);
  gpio_disable_pulls(PICO1_TX_PIN);
  gpio_set_drive_strength(PICO1_TX_PIN, GPIO_DRIVE_STRENGTH_8MA);

  chipSelected = false;
  clockWasHigh = false;
  replyBitNumber = 0;
}


void updatePico1Spi() {
  bool csLow = gpio_get(PICO1_CS_PIN) == 0;
  bool clockHigh = gpio_get(PICO1_SCK_PIN) != 0;

  if (!csLow) {
    if (chipSelected) {
      releaseReplyPin();
      chipSelected = false;
    }
    clockWasHigh = clockHigh;
    return;
  }

  if (!chipSelected) {
    for (uint8_t index = 0; index < PICO1_FRAME_BYTES; index++) {
      activeFrame[index] = stagedFrame[index];
    }
    replyBitNumber = 0;
    driveReplyBit();
    chipSelected = true;
    clockWasHigh = clockHigh;
    return;
  }

  // The Uno reads the current bit while the clock is high. When the
  // clock falls, that read is finished, so the next bit can be placed.
  if (clockWasHigh && !clockHigh) {
    if (replyBitNumber < PICO1_FRAME_BITS) {
      replyBitNumber++;
    }
    driveReplyBit();
  }
  clockWasHigh = clockHigh;
}


void setPico1LunaSample(
    uint16_t distanceCentimeters,
    uint16_t signalStrength,
    bool fresh
) {
  // A transfer already copied stagedFrame. Leave that copy stable.
  if (chipSelected) {
    return;
  }
  writeLunaFrame(stagedFrame, distanceCentimeters, signalStrength, fresh);
}
