/*
WHAT:
  The Uno Q side of the link to Pico 1.

WHY:
  Pico 1 is wired to the JSPI connector, not to the header SPI pins.
  The Arduino SPI driver is fixed to D11, D12 and D13, so it cannot
  see this wiring. This file clocks the JSPI pins itself and asks
  whether Pico 1 answers.

HOW:
  Once a second, D10 (chip select) goes low and eight bytes are clocked
  on JSPI. The reply is:

    0..1   0xA5 0xA5
    2..3   TF-Luna distance in centimeters, low byte first
    4..5   signal strength, low byte first
    6      1 when that sample is fresh, otherwise 0
    7      checksum, the low 8 bits of the sum of bytes 0..6

  The two 0xA5 bytes and a matching checksum mean Pico 1 is present.

    JSPI pin 4 MOSI  pin 23  output  -> Pico GP12
    JSPI pin 1 MISO  pin 22  input   <- Pico GP11
    JSPI pin 3 SCK   pin 24  output  -> Pico GP10
    D10 chip select  pin 10  output  -> Pico GP9
    JSPI pin 6 GND
    JSPI pin 2 +5V   -> Pico VSYS

IMPORTANT:
  JSPI pin 5 is the Uno Q reset pin. This file never touches it.
  The clock rests low. A bit is read as the clock rises. That is SPI
  mode 0, the same mode Pico 1 uses.
  The transfer takes a fraction of a millisecond and runs once a
  second, so it does not sit inside the attitude path.
*/

// ---------------------------------------------------------------------------
// JSPI pins. Numbers are the Uno Q digital pin indexes for those pads.
// ---------------------------------------------------------------------------

constexpr uint8_t PICO1_MISO_PIN = 22;
constexpr uint8_t PICO1_MOSI_PIN = 23;
constexpr uint8_t PICO1_SCK_PIN = 24;
constexpr uint8_t PICO1_CS_PIN = 10;

constexpr uint8_t PICO1_SYNC_BYTE = 0xA5;
constexpr uint8_t PICO1_FRAME_BYTES = 8;
constexpr uint32_t PICO1_CHECK_INTERVAL_MS = 1000;
// Long enough for the Pico to place the next bit while the clock is low.
constexpr uint32_t PICO1_BIT_DELAY_US = 10;

// ---------------------------------------------------------------------------
// Link state
// ---------------------------------------------------------------------------

bool pico1Present = false;
bool pico1LunaFresh = false;
uint16_t pico1DistanceCentimeters = 0;
uint16_t pico1SignalStrength = 0;
uint8_t pico1LastBytes[PICO1_FRAME_BYTES] = {0, 0, 0, 0, 0, 0, 0, 0};
uint32_t pico1LastCheckMs = 0;


void setupPico1Link() {
  pinMode(PICO1_MOSI_PIN, OUTPUT);
  pinMode(PICO1_SCK_PIN, OUTPUT);
  // Pull the return wire high when the Pico is not driving it.
  // An open wire then reads FF. A wire held low still reads 00.
  pinMode(PICO1_MISO_PIN, INPUT_PULLUP);
  pinMode(PICO1_CS_PIN, OUTPUT);

  // Idle bus: clock low, chip select released, data low.
  digitalWrite(PICO1_SCK_PIN, LOW);
  digitalWrite(PICO1_MOSI_PIN, LOW);
  digitalWrite(PICO1_CS_PIN, HIGH);
}


uint8_t clockOneByteToPico1(uint8_t byteOut) {
  uint8_t byteIn = 0;

  // Most significant bit first, matching the Pico.
  for (int bitNumber = 7; bitNumber >= 0; bitNumber--) {
    digitalWrite(PICO1_MOSI_PIN, (byteOut >> bitNumber) & 1);
    delayMicroseconds(PICO1_BIT_DELAY_US);

    digitalWrite(PICO1_SCK_PIN, HIGH);
    delayMicroseconds(PICO1_BIT_DELAY_US);
    byteIn = (byteIn << 1) | (digitalRead(PICO1_MISO_PIN) ? 1 : 0);

    digitalWrite(PICO1_SCK_PIN, LOW);
  }

  return byteIn;
}


bool askPico1() {
  digitalWrite(PICO1_CS_PIN, LOW);
  // The Pico places bit 7 when chip select falls. Wait until that bit
  // is on the wire before the first clock rises.
  delayMicroseconds(50);

  for (uint8_t byteNumber = 0; byteNumber < PICO1_FRAME_BYTES; byteNumber++) {
    pico1LastBytes[byteNumber] = clockOneByteToPico1(0x00);
  }

  digitalWrite(PICO1_CS_PIN, HIGH);
  digitalWrite(PICO1_MOSI_PIN, LOW);

  uint8_t checksum = 0;
  for (uint8_t index = 0; index < PICO1_FRAME_BYTES - 1; index++) {
    checksum = static_cast<uint8_t>(checksum + pico1LastBytes[index]);
  }

  bool frameOk = pico1LastBytes[0] == PICO1_SYNC_BYTE &&
      pico1LastBytes[1] == PICO1_SYNC_BYTE &&
      checksum == pico1LastBytes[PICO1_FRAME_BYTES - 1];

  if (!frameOk) {
    pico1LunaFresh = false;
    return false;
  }

  pico1DistanceCentimeters = static_cast<uint16_t>(
      pico1LastBytes[2] | (static_cast<uint16_t>(pico1LastBytes[3]) << 8));
  pico1SignalStrength = static_cast<uint16_t>(
      pico1LastBytes[4] | (static_cast<uint16_t>(pico1LastBytes[5]) << 8));
  pico1LunaFresh = pico1LastBytes[6] == 1;
  return true;
}


void updatePico1Link() {
  uint32_t nowMs = millis();

  if (
      pico1LastCheckMs != 0 &&
      static_cast<uint32_t>(nowMs - pico1LastCheckMs) < PICO1_CHECK_INTERVAL_MS
  ) {
    return;
  }

  pico1LastCheckMs = nowMs;
  pico1Present = askPico1();
  if (!pico1Present) {
    delayMicroseconds(40);
    pico1Present = askPico1();
  }
}


bool pico1LinkPresent() {
  return pico1Present;
}


bool pico1LunaDistanceFresh() {
  return pico1Present && pico1LunaFresh;
}


uint16_t pico1LunaDistanceCm() {
  return pico1DistanceCentimeters;
}


void printPico1Summary() {
  char line[96];
  int length = 0;

  if (pico1Present && pico1LunaFresh) {
    length = snprintf(line, sizeof(line),
        "PICO1 yes | %u cm | strength %u | fresh yes\n",
        pico1DistanceCentimeters, pico1SignalStrength);
  } else if (pico1Present) {
    length = snprintf(line, sizeof(line), "PICO1 yes | luna silent\n");
  } else {
    length = snprintf(line, sizeof(line),
        "PICO1 no %02X %02X %02X %02X %02X %02X %02X %02X\n",
        pico1LastBytes[0], pico1LastBytes[1], pico1LastBytes[2], pico1LastBytes[3],
        pico1LastBytes[4], pico1LastBytes[5], pico1LastBytes[6], pico1LastBytes[7]);
  }

  if (length > 0) {
    writeFlightLog(line, length);
  }
}
