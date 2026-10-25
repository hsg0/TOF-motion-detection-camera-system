/*
 * WHAT:
 * Non-blocking SH1107 128x64 OLED FeatherWing (Adafruit P4650F, address 0x3C)
 * status display on the shared Qwiic bus (Wire1) for the Arduino UNO Q.
 *
 * Displays 4 lines:
 *   Line 1: GPS lat & lon rounded (plus satellites in use / in view)
 *   Line 2: IMU accel X/Y/Z data
 *   Line 3: Baro pressure (hPa) & Pico VL53L0X ToF distance (cm)
 *   Line 4: Pi 5 Camera ON or OFF
 *
 * WHY:
 * 1. On the STM32U585 Zephyr toolchain, %f in snprintf is disabled and prints
 *    blank strings. All decimals here are formatted with integer math.
 * 2. On the SH1107 controller, each command byte must be sent in its own
 *    command packet (0x00, cmd) so every register command takes effect.
 * 3. Flushing 1024 bytes in one loop() pass would stall the IMU FIFO, so the
 *    display flushes one 32-byte half-page per loop() pass (< 1 ms per pass).
 */

#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

constexpr uint8_t OLED_I2C_ADDR = 0x3C;
constexpr uint8_t OLED_I2C_ADDR_ALT = 0x3D;
constexpr uint32_t OLED_REFRESH_INTERVAL_MS = 250;
constexpr uint32_t OLED_RETRY_INTERVAL_MS = 2000;
constexpr uint32_t CAMERA_HEARTBEAT_TIMEOUT_MS = 3500;

namespace {

uint8_t activeOledAddr = OLED_I2C_ADDR;
bool oledInitialized = false;
uint32_t lastOledAttemptMs = 0;
uint32_t lastOledRenderMs = 0;

uint8_t sh1107Ram[16][64];

bool flushingPages = false;
uint8_t flushPageIdx = 0;
uint8_t flushColHalf = 0;

bool piCameraOn = false;
uint32_t lastCameraHeartbeatMs = 0;

char cachedLine1[24] = "";
char cachedLine2[24] = "";
char cachedLine3[24] = "";
char cachedLine4[24] = "";

// Compact 5x7 ASCII font (0x20..0x7E)
const uint8_t FONT5X7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // ' '
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // '!'
    {0x00, 0x07, 0x00, 0x07, 0x00}, // '"'
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // '#'
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // '$'
    {0x23, 0x13, 0x08, 0x64, 0x62}, // '%'
    {0x36, 0x49, 0x55, 0x22, 0x50}, // '&'
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '''
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // '('
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // ')'
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // '*'
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // '+'
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ','
    {0x08, 0x08, 0x08, 0x08, 0x08}, // '-'
    {0x00, 0x60, 0x60, 0x00, 0x00}, // '.'
    {0x20, 0x10, 0x08, 0x04, 0x02}, // '/'
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // '0'
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // '1'
    {0x42, 0x61, 0x51, 0x49, 0x46}, // '2'
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // '3'
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // '4'
    {0x27, 0x45, 0x45, 0x45, 0x39}, // '5'
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // '6'
    {0x01, 0x71, 0x09, 0x05, 0x03}, // '7'
    {0x36, 0x49, 0x49, 0x49, 0x36}, // '8'
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // '9'
    {0x00, 0x36, 0x36, 0x00, 0x00}, // ':'
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ';'
    {0x08, 0x14, 0x22, 0x41, 0x00}, // '<'
    {0x14, 0x14, 0x14, 0x14, 0x14}, // '='
    {0x00, 0x41, 0x22, 0x14, 0x08}, // '>'
    {0x02, 0x01, 0x51, 0x09, 0x06}, // '?'
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // '@'
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // 'A'
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // 'B'
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // 'C'
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // 'D'
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // 'E'
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // 'F'
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // 'G'
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // 'H'
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // 'I'
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // 'J'
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // 'K'
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // 'L'
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // 'M'
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // 'N'
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // 'O'
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // 'P'
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // 'Q'
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // 'R'
    {0x46, 0x49, 0x49, 0x49, 0x31}, // 'S'
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // 'T'
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // 'U'
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // 'V'
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // 'W'
    {0x63, 0x14, 0x08, 0x14, 0x63}, // 'X'
    {0x07, 0x08, 0x70, 0x08, 0x07}, // 'Y'
    {0x61, 0x51, 0x49, 0x45, 0x43}, // 'Z'
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // '['
    {0x02, 0x04, 0x08, 0x10, 0x20}, // '\'
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // ']'
    {0x04, 0x02, 0x01, 0x02, 0x04}, // '^'
    {0x40, 0x40, 0x40, 0x40, 0x40}, // '_'
    {0x00, 0x01, 0x02, 0x04, 0x00}, // '`'
    {0x20, 0x54, 0x54, 0x54, 0x78}, // 'a'
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // 'b'
    {0x38, 0x44, 0x44, 0x44, 0x20}, // 'c'
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // 'd'
    {0x38, 0x54, 0x54, 0x54, 0x18}, // 'e'
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // 'f'
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, // 'g'
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // 'h'
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // 'i'
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // 'j'
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // 'k'
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // 'l'
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // 'm'
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // 'n'
    {0x38, 0x44, 0x44, 0x44, 0x38}, // 'o'
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // 'p'
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // 'q'
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // 'r'
    {0x48, 0x54, 0x54, 0x54, 0x20}, // 's'
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // 't'
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // 'u'
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // 'v'
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // 'w'
    {0x44, 0x28, 0x10, 0x28, 0x44}, // 'x'
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // 'y'
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // 'z'
    {0x00, 0x08, 0x36, 0x41, 0x00}, // '{'
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // '|'
    {0x00, 0x41, 0x36, 0x08, 0x00}, // '}'
    {0x08, 0x04, 0x08, 0x10, 0x08}, // '~'
};

bool oledSendSingleCommand(uint8_t addr, uint8_t cmd) {
  Wire1.beginTransmission(addr);
  Wire1.write(0x00);  // Co = 0, D/C# = 0 (single command byte)
  Wire1.write(cmd);
  return Wire1.endTransmission() == 0;
}

bool oledSendCommandSequence(uint8_t addr, const uint8_t* cmds, size_t count) {
  for (size_t i = 0; i < count; i++) {
    if (!oledSendSingleCommand(addr, cmds[i])) {
      return false;
    }
  }
  return true;
}

bool probeAndInitOled() {
  const uint8_t candidates[2] = {OLED_I2C_ADDR, OLED_I2C_ADDR_ALT};
  uint8_t foundAddr = 0;
  for (int i = 0; i < 2; i++) {
    Wire1.beginTransmission(candidates[i]);
    if (Wire1.endTransmission() == 0) {
      foundAddr = candidates[i];
      break;
    }
  }
  if (foundAddr == 0) {
    return false;
  }
  activeOledAddr = foundAddr;

  // Official Adafruit_SH1107 64x128 FeatherWing (P4650) init sequence
  const uint8_t initCmds[] = {
      0xAE,             // Display OFF
      0xD5, 0x51,       // Clock divide ratio / oscillator freq
      0x20,             // Page addressing mode
      0x81, 0x4F,       // Contrast
      0xAD, 0x8A,       // DC-DC converter ON
      0xA0,             // Segment remap normal
      0xC0,             // COM scan direction normal
      0xDC, 0x00,       // Display start line = 0
      0xD3, 0x60,       // Display offset = 0x60 (FeatherWing 4650)
      0xD9, 0x22,       // Pre-charge/dis-charge period
      0xDB, 0x35,       // VCOMH deselect level
      0xA8, 0x3F,       // Multiplex ratio = 63 (0x3F)
      0xA4,             // Entire display ON from RAM
      0xA6,             // Normal display
      0xAF,             // Display ON
  };

  if (!oledSendCommandSequence(activeOledAddr, initCmds, sizeof(initCmds))) {
    return false;
  }

  memset(sh1107Ram, 0, sizeof(sh1107Ram));
  flushingPages = true;
  flushPageIdx = 0;
  flushColHalf = 0;
  return true;
}

// Sets a single pixel at landscape coordinates:
//   x in 0..127 (left to right along the 128px long side)
//   y in 0..63  (top to bottom along the 64px short side)
void setLandscapePixel(int x, int y) {
  if (x < 0 || x >= 128 || y < 0 || y >= 64) {
    return;
  }
  int nativeCol = 63 - y;
  int nativePage = x >> 3;
  int nativeBit = x & 7;
  sh1107Ram[nativePage][nativeCol] |= static_cast<uint8_t>(1u << nativeBit);
}

void drawCharLandscape(int x0, int y0, char c) {
  if (c < 0x20 || c > 0x7E) {
    c = '?';
  }
  const uint8_t* glyph = FONT5X7[c - 0x20];
  for (int col = 0; col < 5; col++) {
    uint8_t bits = glyph[col];
    for (int row = 0; row < 7; row++) {
      if ((bits >> row) & 0x01) {
        // Render 1x width, 1.5x/clean spacing or 1x crisp text:
        // Let's double the vertical thickness (each font row = 2 screen rows, 14px tall)
        // so the 4 lines are big, bright, and super easy to read across the 64px height!
        setLandscapePixel(x0 + col, y0 + row * 2);
        setLandscapePixel(x0 + col, y0 + row * 2 + 1);
      }
    }
  }
}

void drawLineLandscape(int lineIndex, const char* text) {
  int y0 = lineIndex * 16 + 1;
  int x = 0;
  for (size_t i = 0; text[i] != '\0' && x + 5 <= 128; i++) {
    drawCharLandscape(x, y0, text[i]);
    x += 6;
  }
}

void formatFixed1(char* out, size_t outSize, float val) {
  const char* sign = (val < 0.0f) ? "-" : "";
  float mag = fabsf(val);
  long scaled = static_cast<long>(mag * 10.0f + 0.5f);
  long whole = scaled / 10;
  long frac = scaled % 10;
  snprintf(out, outSize, "%s%ld.%ld", sign, whole, frac);
}

void formatFixed2(char* out, size_t outSize, double val) {
  const char* sign = (val < 0.0) ? "-" : "";
  double mag = fabs(val);
  long scaled = static_cast<long>(mag * 100.0 + 0.5);
  long whole = scaled / 100;
  long frac = scaled % 100;
  snprintf(out, outSize, "%s%ld.%02ld", sign, whole, frac);
}

void pollCameraHeartbeatSerial() {
  while (Serial.available() > 0) {
    int ch = Serial.read();
    if (ch == 'C' || ch == '1') {
      piCameraOn = true;
      lastCameraHeartbeatMs = millis();
    } else if (ch == 'c' || ch == '0') {
      piCameraOn = false;
      lastCameraHeartbeatMs = millis();
    }
  }
  while (Serial3.available() > 0) {
    int ch = Serial3.read();
    if (ch == 'C' || ch == '1') {
      piCameraOn = true;
      lastCameraHeartbeatMs = millis();
    } else if (ch == 'c' || ch == '0') {
      piCameraOn = false;
      lastCameraHeartbeatMs = millis();
    }
  }
  if (piCameraOn && static_cast<uint32_t>(millis() - lastCameraHeartbeatMs) > CAMERA_HEARTBEAT_TIMEOUT_MS) {
    piCameraOn = false;
  }
}

void renderFourLinesToBuffer() {
  memset(sh1107Ram, 0, sizeof(sh1107Ram));

  const GpsData& gps = getGpsData();
  const FlightSensorData& imu = getFlightSensorData();
  const BarometerData& baro = getBarometerData();

  // Line 1: GPS lat and lon rounded (using integer math, never %f)
  if (gpsFixValid() || (gps.latitudeDeg != 0.0 || gps.longitudeDeg != 0.0)) {
    char latBuf[12];
    char lonBuf[12];
    formatFixed2(latBuf, sizeof(latBuf), gps.latitudeDeg);
    formatFixed2(lonBuf, sizeof(lonBuf), gps.longitudeDeg);
    snprintf(cachedLine1, sizeof(cachedLine1), "GPS %s %s", latBuf, lonBuf);
  } else {
    snprintf(cachedLine1, sizeof(cachedLine1), "GPS 0.00 0.00 s%lu/%lu",
             static_cast<unsigned long>(gps.satellites),
             static_cast<unsigned long>(gps.satellitesInView));
  }

  // Line 2: IMU data (accel X/Y/Z in m/s^2 formatted with integer math)
  char axBuf[8];
  char ayBuf[8];
  char azBuf[8];
  formatFixed1(axBuf, sizeof(axBuf), imu.accelerationMps2.x);
  formatFixed1(ayBuf, sizeof(ayBuf), imu.accelerationMps2.y);
  formatFixed1(azBuf, sizeof(azBuf), imu.accelerationMps2.z);
  snprintf(cachedLine2, sizeof(cachedLine2), "IMU %s %s %s", axBuf, ayBuf, azBuf);

  // Line 3: Baro & ToF distance
  int pressureHpa = static_cast<int>((baro.pressurePa + 50.0f) / 100.0f);
  if (pico1LunaDistanceFresh()) {
    snprintf(cachedLine3, sizeof(cachedLine3), "B:%dhPa ToF:%ucm",
             pressureHpa,
             static_cast<unsigned>(pico1LunaDistanceCm()));
  } else {
    snprintf(cachedLine3, sizeof(cachedLine3), "B:%dhPa ToF:--cm", pressureHpa);
  }

  // Line 4: Camera ON or OFF
  snprintf(cachedLine4, sizeof(cachedLine4), "CAM: %s", piCameraOn ? "ON" : "OFF");

  drawLineLandscape(0, cachedLine1);
  drawLineLandscape(1, cachedLine2);
  drawLineLandscape(2, cachedLine3);
  drawLineLandscape(3, cachedLine4);
}

void flushNextOledChunk() {
  if (!flushingPages) {
    return;
  }

  uint8_t colOffset = flushColHalf * 32;
  if (!oledSendSingleCommand(activeOledAddr, static_cast<uint8_t>(0xB0 | (flushPageIdx & 0x0F))) ||
      !oledSendSingleCommand(activeOledAddr, static_cast<uint8_t>(0x00 | (colOffset & 0x0F))) ||
      !oledSendSingleCommand(activeOledAddr, static_cast<uint8_t>(0x10 | ((colOffset >> 4) & 0x07)))) {
    oledInitialized = false;
    flushingPages = false;
    return;
  }

  Wire1.beginTransmission(activeOledAddr);
  Wire1.write(0x40);  // Co = 0, D/C# = 1 (data stream)
  for (uint8_t i = 0; i < 32; i++) {
    Wire1.write(sh1107Ram[flushPageIdx][colOffset + i]);
  }
  if (Wire1.endTransmission() != 0) {
    oledInitialized = false;
    flushingPages = false;
    return;
  }

  flushColHalf++;
  if (flushColHalf >= 2) {
    flushColHalf = 0;
    flushPageIdx++;
    if (flushPageIdx >= 16) {
      flushingPages = false;
    }
  }
}

}  // namespace

void setupOledDisplay() {
  lastOledAttemptMs = millis();
  oledInitialized = probeAndInitOled();
  if (oledInitialized) {
    renderFourLinesToBuffer();
    flushingPages = true;
    flushPageIdx = 0;
    flushColHalf = 0;
  }
}

void updateOledDisplay() {
  pollCameraHeartbeatSerial();

  uint32_t nowMs = millis();
  if (!oledInitialized) {
    if (static_cast<uint32_t>(nowMs - lastOledAttemptMs) >= OLED_RETRY_INTERVAL_MS) {
      lastOledAttemptMs = nowMs;
      oledInitialized = probeAndInitOled();
    }
    return;
  }

  if (flushingPages) {
    flushNextOledChunk();
    return;
  }

  if (static_cast<uint32_t>(nowMs - lastOledRenderMs) >= OLED_REFRESH_INTERVAL_MS) {
    lastOledRenderMs = nowMs;
    renderFourLinesToBuffer();
    flushingPages = true;
    flushPageIdx = 0;
    flushColHalf = 0;
  }
}

bool oledDisplayPresent() {
  return oledInitialized;
}

void printOledSummary() {
  char line[160];
  int length = snprintf(line, sizeof(line),
      "OLED 0x%02X %s | L1:[%s] L2:[%s] L3:[%s] L4:[%s]\n",
      activeOledAddr,
      oledInitialized ? "ok" : "missing",
      cachedLine1,
      cachedLine2,
      cachedLine3,
      cachedLine4);
  if (length > 0) {
    if (length > (int)sizeof(line) - 1) {
      length = (int)sizeof(line) - 1;
    }
    writeFlightLog(line, length);
  }
}
