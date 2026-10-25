/*
WHAT:
  VL53L0X Time-of-Flight (ToF) reader on Pico W GP0 (SDA) and GP1 (SCL) over I2C0.
  Also retains fallback support for TF-Luna on GP4/GP5.

WHY:
  Pico 1 reads the VL53L0X (address 0x29) wired to GP0 (pin 1) and GP1 (pin 2)
  powered from 3V3 (pin 36) and GND, and publishes the latest distance in
  centimeters (and millimeters in signalStrength) over the JSPI link to the Uno Q.
*/

#include "pico1Luna.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/sync.h"
#include "pico/time.h"

constexpr uint VL53_SDA_PIN = 0;  // GP0 (physical pin 1)
constexpr uint VL53_SCL_PIN = 1;  // GP1 (physical pin 2)
constexpr uint8_t VL53_ADDR = 0x29;
constexpr uint VL53_BAUD_HZ = 100000;
constexpr uint VL53_I2C_TIMEOUT_US = 30000;
constexpr uint32_t VL53_STALE_AFTER_US = 500000;
// Q9.7 counts. 0.40 MCPS rejects weak reflections the 0.25 default still accepts.
constexpr uint16_t VL53_SIGNAL_LIMIT_Q9_7 = 51;
constexpr uint16_t VL53_MIN_VALID_MM = 30;
constexpr uint16_t VL53_MAX_VALID_MM = 2000;
constexpr uint8_t VL53_MEDIAN_WINDOW = 5;

namespace {

uint16_t distanceCentimeters = 0;
uint16_t distanceMillimeters = 0;
uint32_t lastGoodReadUs = 0;
bool haveGoodRead = false;
spin_lock_t* lunaLock = nullptr;
uint16_t medianWindow[VL53_MEDIAN_WINDOW] = {};
uint8_t medianCount = 0;
uint8_t medianPos = 0;

bool vl53Initialized = false;
uint8_t stopVariable = 0;

uint16_t medianMillimeters() {
  uint16_t ordered[VL53_MEDIAN_WINDOW];
  uint8_t count = medianCount;
  for (uint8_t i = 0; i < count; i++) {
    ordered[i] = medianWindow[i];
  }
  for (uint8_t i = 1; i < count; i++) {
    uint16_t value = ordered[i];
    uint8_t j = i;
    while (j > 0 && ordered[j - 1] > value) {
      ordered[j] = ordered[j - 1];
      j--;
    }
    ordered[j] = value;
  }
  return ordered[count / 2];
}

void pushSample(uint16_t rangeMm) {
  medianWindow[medianPos] = rangeMm;
  medianPos = static_cast<uint8_t>((medianPos + 1) % VL53_MEDIAN_WINDOW);
  if (medianCount < VL53_MEDIAN_WINDOW) {
    medianCount++;
  }
}

void storeSample(uint16_t distCm, uint16_t distMm) {
  uint32_t saved = spin_lock_blocking(lunaLock);
  distanceCentimeters = distCm;
  distanceMillimeters = distMm;
  lastGoodReadUs = time_us_32();
  haveGoodRead = true;
  spin_unlock(lunaLock, saved);
}

void busRecovery(uint sdaPin, uint sclPin) {
  i2c_deinit(i2c0);
  gpio_init(sdaPin);
  gpio_init(sclPin);
  gpio_set_function(sdaPin, GPIO_FUNC_SIO);
  gpio_set_function(sclPin, GPIO_FUNC_SIO);
  gpio_pull_up(sdaPin);
  gpio_pull_up(sclPin);
  gpio_set_dir(sdaPin, GPIO_IN);
  gpio_set_dir(sclPin, GPIO_OUT);

  for (int pulse = 0; pulse < 9; pulse++) {
    gpio_put(sclPin, 0);
    busy_wait_us(5);
    gpio_put(sclPin, 1);
    busy_wait_us(5);
  }

  i2c_init(i2c0, VL53_BAUD_HZ);
  gpio_set_function(sdaPin, GPIO_FUNC_I2C);
  gpio_set_function(sclPin, GPIO_FUNC_I2C);
  gpio_pull_up(sdaPin);
  gpio_pull_up(sclPin);
}

bool writeReg(uint8_t reg, uint8_t val) {
  uint8_t buf[2] = {reg, val};
  int ret = i2c_write_timeout_us(i2c0, VL53_ADDR, buf, 2, false, VL53_I2C_TIMEOUT_US);
  return ret == 2;
}

bool writeReg16(uint8_t reg, uint16_t val) {
  uint8_t buf[3] = {
      reg,
      static_cast<uint8_t>((val >> 8) & 0xFF),
      static_cast<uint8_t>(val & 0xFF),
  };
  int ret = i2c_write_timeout_us(i2c0, VL53_ADDR, buf, 3, false, VL53_I2C_TIMEOUT_US);
  return ret == 3;
}

bool writeMulti(uint8_t reg, const uint8_t* src, uint8_t count) {
  uint8_t buf[16];
  if (count > 15) {
    return false;
  }
  buf[0] = reg;
  for (uint8_t i = 0; i < count; i++) {
    buf[i + 1] = src[i];
  }
  int ret = i2c_write_timeout_us(i2c0, VL53_ADDR, buf, count + 1, false, VL53_I2C_TIMEOUT_US);
  return ret == (count + 1);
}

bool readReg(uint8_t reg, uint8_t* out) {
  int w = i2c_write_timeout_us(i2c0, VL53_ADDR, &reg, 1, true, VL53_I2C_TIMEOUT_US);
  if (w != 1) {
    return false;
  }
  int r = i2c_read_timeout_us(i2c0, VL53_ADDR, out, 1, false, VL53_I2C_TIMEOUT_US);
  return r == 1;
}

bool readReg16(uint8_t reg, uint16_t* out) {
  int w = i2c_write_timeout_us(i2c0, VL53_ADDR, &reg, 1, true, VL53_I2C_TIMEOUT_US);
  if (w != 1) {
    return false;
  }
  uint8_t buf[2] = {0, 0};
  int r = i2c_read_timeout_us(i2c0, VL53_ADDR, buf, 2, false, VL53_I2C_TIMEOUT_US);
  if (r != 2) {
    return false;
  }
  *out = static_cast<uint16_t>((static_cast<uint16_t>(buf[0]) << 8) | buf[1]);
  return true;
}

bool readMulti(uint8_t reg, uint8_t* dst, uint8_t count) {
  int w = i2c_write_timeout_us(i2c0, VL53_ADDR, &reg, 1, true, VL53_I2C_TIMEOUT_US);
  if (w != 1) {
    return false;
  }
  int r = i2c_read_timeout_us(i2c0, VL53_ADDR, dst, count, false, VL53_I2C_TIMEOUT_US);
  return r == count;
}

bool setFlag(uint8_t reg, uint8_t bit, bool state) {
  uint8_t v = 0;
  if (!readReg(reg, &v)) {
    return false;
  }
  if (state) {
    v |= static_cast<uint8_t>(1u << bit);
  } else {
    v &= static_cast<uint8_t>(~(1u << bit));
  }
  return writeReg(reg, v);
}

bool getSpadInfo(uint8_t* countOut, bool* isApertureOut) {
  if (!writeReg(0x80, 0x01) ||
      !writeReg(0xFF, 0x01) ||
      !writeReg(0x00, 0x00) ||
      !writeReg(0xFF, 0x06)) {
    return false;
  }
  if (!setFlag(0x83, 3, true)) {
    return false;
  }
  if (!writeReg(0xFF, 0x07) ||
      !writeReg(0x81, 0x01) ||
      !writeReg(0x80, 0x01) ||
      !writeReg(0x94, 0x6B) ||
      !writeReg(0x83, 0x00)) {
    return false;
  }

  bool ready = false;
  for (int i = 0; i < 500; i++) {
    uint8_t v = 0;
    if (!readReg(0x83, &v)) {
      return false;
    }
    if (v != 0) {
      ready = true;
      break;
    }
    busy_wait_us(1000);
  }
  if (!ready) {
    return false;
  }

  if (!writeReg(0x83, 0x01)) {
    return false;
  }
  uint8_t tmp = 0;
  if (!readReg(0x92, &tmp)) {
    return false;
  }
  *countOut = static_cast<uint8_t>(tmp & 0x7F);
  *isApertureOut = (tmp & 0x80) != 0;

  if (!writeReg(0x81, 0x00) ||
      !writeReg(0xFF, 0x06)) {
    return false;
  }
  if (!setFlag(0x83, 3, false)) {
    return false;
  }
  if (!writeReg(0xFF, 0x01) ||
      !writeReg(0x00, 0x01) ||
      !writeReg(0xFF, 0x00) ||
      !writeReg(0x80, 0x00)) {
    return false;
  }
  return true;
}

bool performSingleRefCalibration(uint8_t vhvInitByte) {
  if (!writeReg(0x00, static_cast<uint8_t>(0x01 | vhvInitByte))) {
    return false;
  }
  bool done = false;
  for (int i = 0; i < 500; i++) {
    uint8_t st = 0;
    if (!readReg(0x13, &st)) {
      return false;
    }
    if ((st & 0x07) != 0) {
      done = true;
      break;
    }
    busy_wait_us(1000);
  }
  if (!done) {
    return false;
  }
  if (!writeReg(0x0B, 0x01) || !writeReg(0x00, 0x00)) {
    return false;
  }
  return true;
}

bool initVl53l0x() {
  busRecovery(VL53_SDA_PIN, VL53_SCL_PIN);

  uint8_t modelId = 0;
  if (!readReg(0xC0, &modelId) || modelId != 0xEE) {
    return false;
  }

  // 2V8 I/O mode
  if (!setFlag(0x89, 0, true)) {
    return false;
  }

  // Standard I2C mode and stop_variable read
  if (!writeReg(0x88, 0x00) ||
      !writeReg(0x80, 0x01) ||
      !writeReg(0xFF, 0x01) ||
      !writeReg(0x00, 0x00) ||
      !readReg(0x91, &stopVariable) ||
      !writeReg(0x00, 0x01) ||
      !writeReg(0xFF, 0x00) ||
      !writeReg(0x80, 0x00)) {
    return false;
  }

  // Disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  if (!setFlag(0x60, 1, true) || !setFlag(0x60, 4, true)) {
    return false;
  }

  // Provisional 0.1 MCPS. The tuning table overwrites register 0x44;
  // the limit that actually sticks is written again after tuning.
  if (!writeReg16(0x44, 12)) {
    return false;
  }

  if (!writeReg(0x01, 0xFF)) {
    return false;
  }

  uint8_t spadCount = 0;
  bool spadIsAperture = false;
  if (!getSpadInfo(&spadCount, &spadIsAperture)) {
    return false;
  }

  uint8_t refSpadMap[6] = {0};
  if (!readMulti(0xB0, refSpadMap, 6)) {
    return false;
  }

  if (!writeReg(0xFF, 0x01) ||
      !writeReg(0x4F, 0x00) ||
      !writeReg(0x4E, 0x2C) ||
      !writeReg(0xFF, 0x00) ||
      !writeReg(0xB6, 0xB4)) {
    return false;
  }

  uint8_t firstSpadToEnable = spadIsAperture ? 12 : 0;
  uint8_t spadsEnabled = 0;
  for (uint8_t i = 0; i < 48; i++) {
    if (i < firstSpadToEnable || spadsEnabled == spadCount) {
      refSpadMap[i / 8] &= static_cast<uint8_t>(~(1u << (i % 8)));
    } else if ((refSpadMap[i / 8] >> (i % 8)) & 0x1) {
      spadsEnabled++;
    }
  }

  if (!writeMulti(0xB0, refSpadMap, 6)) {
    return false;
  }

  // Default tuning settings
  const uint8_t tuning[][2] = {
      {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x00}, {0x09, 0x00},
      {0x10, 0x00}, {0x11, 0x00}, {0x24, 0x01}, {0x25, 0xFF},
      {0x75, 0x00}, {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00},
      {0x30, 0x20}, {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00},
      {0x31, 0x04}, {0x32, 0x03}, {0x40, 0x83}, {0x46, 0x25},
      {0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00},
      {0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00},
      {0x62, 0x00}, {0x64, 0x00}, {0x65, 0x00}, {0x66, 0xA0},
      {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF},
      {0x4A, 0x00}, {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00},
      {0x78, 0x21}, {0xFF, 0x01}, {0x23, 0x34}, {0x42, 0x00},
      {0x44, 0xFF}, {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40},
      {0x0E, 0x06}, {0x20, 0x1A}, {0x43, 0x40}, {0xFF, 0x00},
      {0x34, 0x03}, {0x35, 0x44}, {0xFF, 0x01}, {0x31, 0x04},
      {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04}, {0xFF, 0x00},
      {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28},
      {0x67, 0x00}, {0x70, 0x04}, {0x71, 0x01}, {0x72, 0xFE},
      {0x76, 0x00}, {0x77, 0x00}, {0xFF, 0x01}, {0x0D, 0x01},
      {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8}, {0xFF, 0x01},
      {0x8E, 0x01}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00},
  };
  for (size_t i = 0; i < sizeof(tuning) / sizeof(tuning[0]); i++) {
    if (!writeReg(tuning[i][0], tuning[i][1])) {
      return false;
    }
  }

  // Tuning leaves the final-range floor at 0.25 MCPS (0x0020) and would
  // overwrite any earlier write to register 0x44. Set 0.40 MCPS after it.
  if (!writeReg16(0x44, VL53_SIGNAL_LIMIT_Q9_7)) {
    return false;
  }

  // Interrupt config: new sample ready
  if (!writeReg(0x0A, 0x04) ||
      !setFlag(0x84, 4, false) ||
      !writeReg(0x0B, 0x01)) {
    return false;
  }

  // Reference calibrations
  if (!writeReg(0x01, 0x01) || !performSingleRefCalibration(0x40)) {
    return false;
  }
  if (!writeReg(0x01, 0x02) || !performSingleRefCalibration(0x00)) {
    return false;
  }
  if (!writeReg(0x01, 0xE8)) {
    return false;
  }

  // Start continuous back-to-back ranging
  if (!writeReg(0x80, 0x01) ||
      !writeReg(0xFF, 0x01) ||
      !writeReg(0x00, 0x00) ||
      !writeReg(0x91, stopVariable) ||
      !writeReg(0x00, 0x01) ||
      !writeReg(0xFF, 0x00) ||
      !writeReg(0x80, 0x00) ||
      !writeReg(0x00, 0x02)) {
    return false;
  }

  return true;
}

bool readVl53l0xOnce() {
  if (!vl53Initialized) {
    vl53Initialized = initVl53l0x();
    if (!vl53Initialized) {
      return false;
    }
  }

  uint8_t intStatus = 0;
  if (!readReg(0x13, &intStatus)) {
    vl53Initialized = false;
    return false;
  }

  if ((intStatus & 0x07) == 0) {
    return false;
  }

  uint8_t statusReg = 0;
  if (!readReg(0x14, &statusReg)) {
    vl53Initialized = false;
    return false;
  }
  uint8_t rangeStatus = static_cast<uint8_t>((statusReg >> 3) & 0x0F);

  uint16_t rangeMm = 0;
  if (!readReg16(0x14 + 10, &rangeMm)) {
    vl53Initialized = false;
    return false;
  }
  writeReg(0x0B, 0x01);

  // 0 is Range Valid. 11 is Range Valid Merged Pulse. Anything else is a
  // weak, clipped, or wrapped return and must not move the published range.
  bool statusOk = rangeStatus == 0 || rangeStatus == 11;
  if (!statusOk || rangeMm < VL53_MIN_VALID_MM || rangeMm > VL53_MAX_VALID_MM) {
    return false;
  }

  pushSample(rangeMm);
  if (medianCount < 3) {
    return false;
  }

  uint16_t filteredMm = medianMillimeters();
  uint16_t distCm = static_cast<uint16_t>((filteredMm + 5) / 10);
  storeSample(distCm, filteredMm);
  return true;
}

}  // namespace

void setupLuna() {
  lunaLock = spin_lock_instance(spin_lock_claim_unused(true));
  busRecovery(VL53_SDA_PIN, VL53_SCL_PIN);
}

void updateLuna() {
  readVl53l0xOnce();
}

LunaReading currentLunaReading() {
  uint32_t saved = spin_lock_blocking(lunaLock);
  uint16_t distCm = distanceCentimeters;
  uint16_t distMm = distanceMillimeters;
  uint32_t lastUs = lastGoodReadUs;
  bool haveRead = haveGoodRead;
  spin_unlock(lunaLock, saved);

  LunaReading reading;
  reading.distanceCentimeters = distCm;
  reading.signalStrength = distMm;
  reading.temperatureDecidegrees = 0;
  if (!haveRead) {
    reading.fresh = false;
  } else {
    uint32_t ageUs = time_us_32() - lastUs;
    reading.fresh = ageUs <= VL53_STALE_AFTER_US;
  }
  return reading;
}
