/*
 * WHAT:
 * Production BME280 barometric sensor acquisition module
 * for the Arduino UNO Q.
 *
 * Collects and prepares:
 *
 *   - Atmospheric pressure
 *   - Temperature
 *   - Humidity
 *   - Relative altitude
 *   - Filtered relative altitude
 *   - Estimated vertical speed
 *   - Baseline pressure
 *   - Sample timestamp
 *   - Sample count
 *   - Measured sample rate
 *   - Sample interval
 *   - Timing jitter
 *   - Freshness
 *   - Health state
 *   - I2C discovery state
 *   - Runtime recovery state
 *   - Error counters
 *
 * WHY:
 * The future flight controller must not directly communicate
 * with the BME280.
 *
 * This file owns the barometer hardware and exposes a clean,
 * prepared BarometerData structure for:
 *
 *   - altitude estimation
 *   - vertical-motion estimation
 *   - altitude hold
 *   - sensor fusion
 *   - flight-control decisions
 *
 * HOW:
 *
 *   BME280
 *      ↓
 *   updateBarometer()
 *      ↓
 *   validation / filtering / health
 *      ↓
 *   BarometerData
 *      ↓
 *   flight-control / motor layer
 *
 * IMPORTANT:
 *
 * This file does NOT contain setup() or loop().
 *
 * The main sketch calls:
 *
 *   setupBarometer();
 *   updateBarometer();
 *
 * The flight-control code reads:
 *
 *   getBarometerData();
 *
 * This module is independent of the IMU module in code.
 * A dead BME280 never stops the LSM6DSOX/LIS3MDL path,
 * and a dead IMU never stops this one.
 * They are no longer electrically isolated: both modules
 * share Wire1 because the BME280 is chained on Qwiic
 * through the IMU board.
 *
 * Adafruit BME280 product 2652, the board in hand:
 *
 *   I2C bus = Wire1
 *
 *   Uno Q Qwiic -> LSM6DSOX/LIS3MDL board -> BME280
 *
 * Supported addresses, from Adafruit_BME280.h:
 *
 *   0x77 = BME280_ADDRESS, default
 *   0x76 = BME280_ADDRESS_ALTERNATE
 *
 * The main tab already calls Wire1.begin() before
 * setupBarometer(). This file does not call Wire.begin()
 * or Wire1.begin().
 */


 #include <Wire.h>
 #include <math.h>
 #include <stdio.h>
 #include <Adafruit_BME280.h>
 
 
 // ==================================================
 // CONFIGURATION
 // ==================================================
 
 /*
  * Confirmed in Adafruit_BME280.h:
  * BME280_ADDRESS is 0x77, BME280_ADDRESS_ALTERNATE is 0x76.
  */
 
 
 /*
  * Barometric altitude does not need to run at the
  * gyroscope rate.
  *
  * Bosch's indoor-navigation preset (pressure x16,
  * temperature x2, humidity x1) takes about 40 ms
  * per measurement. That oversampling choice is why
  * the expected rate is 25 Hz, not 50 Hz.
  *
  *     one acquisition every 40 ms
  */
 constexpr float BAROMETER_EXPECTED_RATE_HZ =
     25.0f;
 
 constexpr uint32_t BAROMETER_EXPECTED_PERIOD_US =
     40000UL;
 
 
 /*
  * Health limit.
  *
  * This is NOT the flight-controller loop period.
  */
 constexpr uint32_t BAROMETER_STALE_AFTER_US =
     100000UL;
 
 
 /*
  * How often to retry a missing/failed BME280.
  */
 constexpr uint32_t BAROMETER_RETRY_MS =
     250;
 
 
 /*
  * Number of valid samples used to determine
  * initial ground pressure.
  *
  * At 25 Hz:
  *
  *     100 samples ≈ 4 seconds
  */
 constexpr uint16_t BAROMETER_BASELINE_SAMPLES =
     100;
 
 
 /*
  * Runtime failures tolerated before the sensor is
  * marked unavailable and initialization is retried.
  */
 constexpr uint8_t BAROMETER_FAILURE_LIMIT =
     5;
 
 
 /*
  * Published pressure window, in pascals.
  */
 constexpr float MIN_VALID_PRESSURE_PA =
     30000.0f;
 
 constexpr float MAX_VALID_PRESSURE_PA =
     125000.0f;
 
 
 /*
  * Basic software filtering.
  *
  * Higher values:
  *     faster response
  *
  * Lower values:
  *     smoother response
  */
 constexpr float ALTITUDE_FILTER_ALPHA =
     0.15f;
 
 constexpr float VERTICAL_SPEED_FILTER_ALPHA =
     0.10f;
 
 
 // ==================================================
 // DATA TYPES
 // ==================================================
 
 struct BarometerTiming {
 
   bool valid = false;
 
   uint32_t lastIntervalUs = 0;
 
   uint32_t minimumIntervalUs = 0;
 
   uint32_t maximumIntervalUs = 0;
 
   float averageIntervalUs = 0;
 
   uint32_t maximumJitterUs = 0;
 };
 
 
 struct BarometerErrors {
 
   /*
    * Neither BME280 address responded on Wire1.
    */
   uint32_t addressProbeFailures = 0;
 
 
   /*
    * Device responded on I2C but Adafruit begin()
    * rejected it or failed communication.
    * begin() requires chip id 0x60.
    */
   uint32_t beginFailures = 0;
 
 
   /*
    * Sensor began successfully but configuration
    * failed.
    */
   uint32_t configurationFailures = 0;
 
 
   uint32_t invalidPressureReads = 0;
 
   uint32_t invalidTemperatureReads = 0;
 
   uint32_t invalidHumidityReads = 0;
 
   uint32_t baselineFailures = 0;
 
 
   /*
    * Number of times runtime failure detection
    * forced the sensor offline.
    */
   uint32_t runtimeDisconnects = 0;
 };
 
 
 struct BarometerHealth {
 
   bool initialized = false;
 
   bool i2cDeviceDetected = false;
 
   bool baselineReady = false;
 
   bool pressureValid = false;
 
   bool temperatureValid = false;
 
   bool humidityValid = false;
 
   bool fresh = false;
 
   bool healthy = false;
 };
 
 
 struct BarometerData {
 
   uint32_t sequence = 0;
 
   uint32_t sampleCount = 0;
 
 
   // ------------------------------------------------
   // ADDRESS / CONNECTION
   // ------------------------------------------------
 
   uint8_t i2cAddress = 0;
 
 
   // ------------------------------------------------
   // TIMING
   // ------------------------------------------------
 
   uint64_t timestampUs = 0;
 
   uint32_t lastSampleUs = 0;
 
   float sampleRateHz = 0;
 
   BarometerTiming timing;
 
 
   // ------------------------------------------------
   // SENSOR VALUES
   // ------------------------------------------------
 
   float pressurePa = 0;
 
   float temperatureC = 0;
 
   float humidityPercent = 0;
 
 
   // ------------------------------------------------
   // ALTITUDE
   // ------------------------------------------------
 
   float baselinePressurePa = 0;
 
   float relativeAltitudeM = 0;
 
   float filteredAltitudeM = 0;
 
   float verticalSpeedMps = 0;
 
 
   // ------------------------------------------------
   // HEALTH
   // ------------------------------------------------
 
   BarometerHealth health;
 
 
   // ------------------------------------------------
   // ERRORS
   // ------------------------------------------------
 
   BarometerErrors errors;
 };
 
 
 // ==================================================
 // TIMING ACCUMULATOR
 // ==================================================
 
 struct BarometerTimingAccumulator {
 
   bool havePrevious = false;
 
   uint32_t previousUs = 0;
 
   uint32_t lastIntervalUs = 0;
 
   uint32_t minimumIntervalUs = 0;
 
   uint32_t maximumIntervalUs = 0;
 
   uint64_t intervalTotalUs = 0;
 
   uint32_t intervalCount = 0;
 
   uint32_t maximumJitterUs = 0;
 };
 
 
 // ==================================================
 // SENSOR OBJECTS
 // ==================================================
 
 /*
  * One driver object. begin() takes the address, so the
  * 0x77 / 0x76 choice is made at discovery time.
  *
  * read8() is protected, so the subclass can read the
  * control registers back after setSampling().
  */
 
 class FlightBme280 : public Adafruit_BME280 {
 
  public:
 
   /*
    * setSampling() and write8() return void, so a
    * read-back is the only way to know the config landed.
    *
    * ctrl_meas at 0xF4, the whole register:
    *   bits 7:5 temperature oversampling
    *   bits 4:2 pressure oversampling
    *   bits 1:0 mode
    *
    * ctrl_hum at 0xF2:
    *   bits 2:0 humidity oversampling
    *   bits 7:3 are unused and are not compared
    */
   bool requestedSamplingLanded() {
 
     uint8_t ctrlMeas =
         read8(BME280_REGISTER_CONTROL);
 
 
     uint8_t ctrlHum =
         read8(BME280_REGISTER_CONTROLHUMID);
 
 
     uint8_t expectedCtrlMeas =
         static_cast<uint8_t>(
             (SAMPLING_X2 << 5) |
             (SAMPLING_X16 << 2) |
             MODE_NORMAL
         );
 
 
     uint8_t humidityOversampling =
         ctrlHum & 0x07;
 
 
     return
         ctrlMeas == expectedCtrlMeas &&
         humidityOversampling ==
             SAMPLING_X1;
   }
 
 };
 
 
 FlightBme280 bme280;
 
 
 FlightBme280 *activeBme280 =
     nullptr;
 
 
 // ==================================================
 // PUBLISHED DATA
 // ==================================================
 
 BarometerData barometerData;
 
 
 // ==================================================
 // INTERNAL STATE
 // ==================================================
 
 BarometerTimingAccumulator
     barometerTimingAccumulator;
 
 
 bool barometerReady = false;
 
 bool barometerConfigured = false;
 
 
 uint8_t consecutiveBarometerFailures = 0;
 
 
 uint32_t nextBarometerRetryMs = 0;
 
 
 uint32_t barometerRateWindowStartMs = 0;
 
 uint32_t barometerRateWindowSampleStart = 0;
 
 
 /*
  * Used to enforce the 25 Hz software poll rate.
  * 40000 us comes from the oversampling choice above.
  */
 uint32_t lastBarometerPollUs = 0;
 
 
 // ==================================================
 // MONOTONIC TIME
 // ==================================================
 
 uint32_t lastMicrosReadingBarometer = 0;
 
 uint64_t barometerMonotonicUs = 0;
 
 bool haveBarometerMonotonicTime = false;
 
 
 // ==================================================
 // BASELINE
 // ==================================================
 
 float baselinePressureSum = 0;
 
 uint16_t baselinePressureCount = 0;
 
 
 // ==================================================
 // FILTER STATE
 // ==================================================
 
 bool haveFilteredAltitude = false;
 
 float previousFilteredAltitudeM = 0;
 
 uint32_t previousAltitudeUs = 0;
 
 
 // ==================================================
 // TIME
 // ==================================================
 
 uint64_t barometerMonotonicMicroseconds() {
 
   uint32_t now =
       micros();
 
 
   if (haveBarometerMonotonicTime) {
 
     barometerMonotonicUs +=
         static_cast<uint32_t>(
             now -
             lastMicrosReadingBarometer
         );
   }
 
 
   lastMicrosReadingBarometer =
       now;
 
 
   haveBarometerMonotonicTime =
       true;
 
 
   return barometerMonotonicUs;
 }
 
 
 // ==================================================
 // I2C HELPERS
 // ==================================================
 
 bool barometerAddressResponds(
     uint8_t address
 ) {
 
   Wire1.beginTransmission(
       address
   );
 
 
   uint8_t result =
       Wire1.endTransmission();
 
 
   return result == 0;
 }
 
 
 // ==================================================
 // I2C BUS SCAN
 // ==================================================
 
 void printBarometerBusScan() {
 
   Serial.println();
 
   Serial.println(
       "BME280 Wire1 BUS SCAN"
   );
 
   Serial.println(
       "====================="
   );
 
 
   /*
    * Seeing 0x6A and 0x1C is normal and is proof
    * the Qwiic chain is alive. Those are the
    * LSM6DSOX and LIS3MDL on the board this
    * BME280 is chained through.
    */
 
   uint8_t deviceCount = 0;
 
 
   for (
       uint8_t address = 1;
       address < 127;
       address++
   ) {
 
     Wire1.beginTransmission(
         address
     );
 
 
     uint8_t error =
         Wire1.endTransmission();
 
 
     if (error != 0) {
 
       continue;
     }
 
 
     Serial.print(
         "Wire1 device found at 0x"
     );
 
 
     if (address < 0x10) {
 
       Serial.print("0");
     }
 
 
     Serial.print(
         address,
         HEX
     );
 
 
     if (address == 0x6A) {
 
       Serial.print(
           "  LSM6DSOX"
       );
 
     } else if (address == 0x1C) {
 
       Serial.print(
           "  LIS3MDL"
       );
 
     } else if (
         address == BME280_ADDRESS ||
         address == BME280_ADDRESS_ALTERNATE
     ) {
 
       Serial.print(
           "  BME280"
       );
     }
 
 
     Serial.println();
 
 
     deviceCount++;
   }
 
 
   Serial.print(
       "Wire1 devices found: "
   );
 
 
   Serial.println(
       deviceCount
   );
 
 
   Serial.println();
 }
 
 
 // ==================================================
 // BME280 ADDRESS DISCOVERY
 // ==================================================
 
 uint8_t detectBarometerAddress() {
 
   bool defaultAddressFound =
       barometerAddressResponds(
           BME280_ADDRESS
       );
 
 
   if (defaultAddressFound) {
 
     barometerData.health
         .i2cDeviceDetected =
         true;
 
 
     barometerData.i2cAddress =
         BME280_ADDRESS;
 
 
     return BME280_ADDRESS;
   }
 
 
   bool alternateAddressFound =
       barometerAddressResponds(
           BME280_ADDRESS_ALTERNATE
       );
 
 
   if (alternateAddressFound) {
 
     barometerData.health
         .i2cDeviceDetected =
         true;
 
 
     barometerData.i2cAddress =
         BME280_ADDRESS_ALTERNATE;
 
 
     return BME280_ADDRESS_ALTERNATE;
   }
 
 
   barometerData.health
       .i2cDeviceDetected =
       false;
 
 
   barometerData.i2cAddress =
       0;
 
 
   return 0;
 }
 
 
 // ==================================================
 // SELECT SENSOR OBJECT
 // ==================================================
 
 bool selectBarometerDevice(
     uint8_t address
 ) {
 
   if (
       address == BME280_ADDRESS ||
       address == BME280_ADDRESS_ALTERNATE
   ) {
 
     activeBme280 =
         &bme280;
 
 
     return true;
   }
 
 
   activeBme280 =
       nullptr;
 
 
   return false;
 }
 
 
 // ==================================================
 // TIMING
 // ==================================================
 
 void recordBarometerTiming(
     uint32_t sampleUs
 ) {
 
   if (
       !barometerTimingAccumulator
            .havePrevious
   ) {
 
     barometerTimingAccumulator
         .previousUs =
         sampleUs;
 
 
     barometerTimingAccumulator
         .havePrevious =
         true;
 
 
     return;
   }
 
 
   uint32_t intervalUs =
       static_cast<uint32_t>(
           sampleUs -
           barometerTimingAccumulator
               .previousUs
       );
 
 
   barometerTimingAccumulator
       .previousUs =
       sampleUs;
 
 
   barometerTimingAccumulator
       .lastIntervalUs =
       intervalUs;
 
 
   if (
       barometerTimingAccumulator
               .intervalCount ==
           0 ||
       intervalUs <
           barometerTimingAccumulator
               .minimumIntervalUs
   ) {
 
     barometerTimingAccumulator
         .minimumIntervalUs =
         intervalUs;
   }
 
 
   if (
       intervalUs >
       barometerTimingAccumulator
           .maximumIntervalUs
   ) {
 
     barometerTimingAccumulator
         .maximumIntervalUs =
         intervalUs;
   }
 
 
   barometerTimingAccumulator
       .intervalTotalUs +=
       intervalUs;
 
 
   barometerTimingAccumulator
       .intervalCount++;
 
 
   uint32_t jitterUs;
 
 
   if (
       intervalUs >=
       BAROMETER_EXPECTED_PERIOD_US
   ) {
 
     jitterUs =
         intervalUs -
         BAROMETER_EXPECTED_PERIOD_US;
 
   } else {
 
     jitterUs =
         BAROMETER_EXPECTED_PERIOD_US -
         intervalUs;
   }
 
 
   if (
       jitterUs >
       barometerTimingAccumulator
           .maximumJitterUs
   ) {
 
     barometerTimingAccumulator
         .maximumJitterUs =
         jitterUs;
   }
 }
 
 
 // ==================================================
 // PUBLISH TIMING
 // ==================================================
 
 void publishBarometerTiming() {
 
   if (
       barometerTimingAccumulator
           .intervalCount ==
       0
   ) {
 
     barometerData.timing.valid =
         false;
 
 
     barometerData.timing
         .lastIntervalUs =
         0;
 
 
     barometerData.timing
         .minimumIntervalUs =
         0;
 
 
     barometerData.timing
         .maximumIntervalUs =
         0;
 
 
     barometerData.timing
         .averageIntervalUs =
         0;
 
 
     barometerData.timing
         .maximumJitterUs =
         0;
 
 
     return;
   }
 
 
   barometerData.timing.valid =
       true;
 
 
   barometerData.timing
       .lastIntervalUs =
       barometerTimingAccumulator
           .lastIntervalUs;
 
 
   barometerData.timing
       .minimumIntervalUs =
       barometerTimingAccumulator
           .minimumIntervalUs;
 
 
   barometerData.timing
       .maximumIntervalUs =
       barometerTimingAccumulator
           .maximumIntervalUs;
 
 
   barometerData.timing
       .averageIntervalUs =
       static_cast<float>(
           barometerTimingAccumulator
               .intervalTotalUs
       ) /
       static_cast<float>(
           barometerTimingAccumulator
               .intervalCount
       );
 
 
   barometerData.timing
       .maximumJitterUs =
       barometerTimingAccumulator
           .maximumJitterUs;
 
 
   /*
    * Preserve:
    *
    *     havePrevious
    *     previousUs
    *
    * so timing remains continuous across
    * reporting windows.
    */
 
   barometerTimingAccumulator
       .lastIntervalUs =
       0;
 
 
   barometerTimingAccumulator
       .minimumIntervalUs =
       0;
 
 
   barometerTimingAccumulator
       .maximumIntervalUs =
       0;
 
 
   barometerTimingAccumulator
       .intervalTotalUs =
       0;
 
 
   barometerTimingAccumulator
       .intervalCount =
       0;
 
 
   barometerTimingAccumulator
       .maximumJitterUs =
       0;
 }
 
 
 // ==================================================
 // ALTITUDE
 // ==================================================
 
 float calculateRelativeAltitudeM(
     float pressurePa,
     float referencePressurePa
 ) {
 
   if (
       pressurePa <= 0 ||
       referencePressurePa <= 0
   ) {
 
     return 0;
   }
 
 
   float pressureRatio =
       pressurePa /
       referencePressurePa;
 
 
   return
       44330.0f *
       (
           1.0f -
           powf(
               pressureRatio,
               0.19029495f
           )
       );
 }
 
 
 // ==================================================
 // ALTITUDE FILTERING
 // ==================================================
 
 void updateAltitudeFiltering(
     float relativeAltitudeM,
     uint32_t sampleUs
 ) {
 
   if (!haveFilteredAltitude) {
 
     barometerData
         .filteredAltitudeM =
         relativeAltitudeM;
 
 
     previousFilteredAltitudeM =
         relativeAltitudeM;
 
 
     previousAltitudeUs =
         sampleUs;
 
 
     barometerData
         .verticalSpeedMps =
         0;
 
 
     haveFilteredAltitude =
         true;
 
 
     return;
   }
 
 
   float filteredAltitudeM =
       barometerData
           .filteredAltitudeM +
       ALTITUDE_FILTER_ALPHA *
       (
           relativeAltitudeM -
           barometerData
               .filteredAltitudeM
       );
 
 
   uint32_t deltaUs =
       static_cast<uint32_t>(
           sampleUs -
           previousAltitudeUs
       );
 
 
   if (deltaUs > 0) {
 
     float deltaSeconds =
         deltaUs /
         1000000.0f;
 
 
     float rawVerticalSpeed =
         (
             filteredAltitudeM -
             previousFilteredAltitudeM
         ) /
         deltaSeconds;
 
 
     barometerData
         .verticalSpeedMps =
         barometerData
             .verticalSpeedMps +
         VERTICAL_SPEED_FILTER_ALPHA *
         (
             rawVerticalSpeed -
             barometerData
                 .verticalSpeedMps
         );
   }
 
 
   barometerData
       .filteredAltitudeM =
       filteredAltitudeM;
 
 
   previousFilteredAltitudeM =
       filteredAltitudeM;
 
 
   previousAltitudeUs =
       sampleUs;
 }
 
 
 // ==================================================
 // BASELINE
 // ==================================================
 
 void updatePressureBaseline(
     float pressurePa
 ) {
 
   if (
       barometerData.health
           .baselineReady
   ) {
 
     return;
   }
 
 
   baselinePressureSum +=
       pressurePa;
 
 
   baselinePressureCount++;
 
 
   if (
       baselinePressureCount <
       BAROMETER_BASELINE_SAMPLES
   ) {
 
     return;
   }
 
 
   barometerData
       .baselinePressurePa =
       baselinePressureSum /
       baselinePressureCount;
 
 
   if (
       barometerData
               .baselinePressurePa <
           MIN_VALID_PRESSURE_PA ||
       barometerData
               .baselinePressurePa >
           MAX_VALID_PRESSURE_PA
   ) {
 
     barometerData.errors
         .baselineFailures++;
 
 
     baselinePressureSum =
         0;
 
 
     baselinePressureCount =
         0;
 
 
     return;
   }
 
 
   barometerData.health
       .baselineReady =
       true;
 }
 
 
 // ==================================================
 // FAILURE HANDLING
 // ==================================================
 
 void markBarometerOffline() {
 
   barometerReady =
       false;
 
 
   barometerConfigured =
       false;
 
 
   activeBme280 =
       nullptr;
 
 
   barometerData.health
       .initialized =
       false;
 
 
   barometerData.health
       .pressureValid =
       false;
 
 
   barometerData.health
       .temperatureValid =
       false;
 
 
   barometerData.health
       .humidityValid =
       false;
 
 
   barometerData.health
       .fresh =
       false;
 
 
   barometerData.health
       .healthy =
       false;
 
 
   /*
    * Keep the established baseline.
    *
    * If the sensor briefly disconnects during flight,
    * we do not want to redefine ground altitude.
    */
 
   haveFilteredAltitude =
       false;
 
 
   consecutiveBarometerFailures =
       0;
 
 
   nextBarometerRetryMs =
       millis();
 
 
   barometerData.errors
       .runtimeDisconnects++;
 }
 
 
 // ==================================================
 // CONFIGURE SENSOR
 // ==================================================
 
 bool configureBarometer() {
 
   if (activeBme280 == nullptr) {
 
     return false;
   }
 
 
   /*
    * Bosch indoor-navigation preset.
    *
    *   pressure oversampling x16
    *   temperature oversampling x2
    *   humidity oversampling x1
    *   IIR filter x16
    *   standby 0.5 ms, the shortest standby_duration
    *
    * The measurement time from that oversampling is
    * what makes the output about 25 Hz.
    *
    * setSampling() returns void. requestedSamplingLanded()
    * reads ctrl_meas and ctrl_hum back, and a mismatch
    * is the configuration failure.
    */
 
   activeBme280->setSampling(
       Adafruit_BME280::MODE_NORMAL,
       Adafruit_BME280::SAMPLING_X2,
       Adafruit_BME280::SAMPLING_X16,
       Adafruit_BME280::SAMPLING_X1,
       Adafruit_BME280::FILTER_X16,
       Adafruit_BME280::STANDBY_MS_0_5
   );
 
 
   return activeBme280->requestedSamplingLanded();
 }
 
 
 // ==================================================
 // SENSOR INITIALIZATION
 // ==================================================
 
 bool bringUpBarometer() {
 
   if (barometerReady) {
 
     return true;
   }
 
 
   // ------------------------------------------------
   // FIND ADDRESS
   // ------------------------------------------------
 
   uint8_t detectedAddress =
       detectBarometerAddress();
 
 
   if (detectedAddress == 0) {
 
     barometerData.errors
         .addressProbeFailures++;
 
 
     barometerData.health
         .initialized =
         false;
 
 
     nextBarometerRetryMs =
         millis();
 
 
     return false;
   }
 
 
   if (
       !selectBarometerDevice(
           detectedAddress
       )
   ) {
 
     barometerData.errors
         .addressProbeFailures++;
 
 
     nextBarometerRetryMs =
         millis();
 
 
     return false;
   }
 
 
   // ------------------------------------------------
   // DRIVER INITIALIZATION
   // ------------------------------------------------
 
   if (
       activeBme280 ==
           nullptr ||
       !activeBme280->begin(
           detectedAddress,
           &Wire1
       )
   ) {
 
     barometerData.errors
         .beginFailures++;
 
 
     barometerData.health
         .initialized =
         false;
 
 
     nextBarometerRetryMs =
         millis();
 
 
     return false;
   }
 
 
   // ------------------------------------------------
   // SENSOR CONFIGURATION
   // ------------------------------------------------
 
   if (!configureBarometer()) {
 
     barometerData.errors
         .configurationFailures++;
 
 
     barometerData.health
         .initialized =
         false;
 
 
     nextBarometerRetryMs =
         millis();
 
 
     return false;
   }
 
 
   barometerConfigured =
       true;
 
 
   barometerReady =
       true;
 
 
   consecutiveBarometerFailures =
       0;
 
 
   barometerData.health
       .initialized =
       true;
 
 
   barometerData.health
       .i2cDeviceDetected =
       true;
 
 
   barometerRateWindowStartMs =
       millis();
 
 
   barometerRateWindowSampleStart =
       barometerData.sampleCount;
 
 
   lastBarometerPollUs =
       micros();
 
 
   /*
    * Restart filtering after hardware recovery.
    *
    * Baseline is intentionally preserved.
    */
 
   haveFilteredAltitude =
       false;
 
 
   return true;
 }
 
 
 // ==================================================
 // SETUP
 // ==================================================
 
 bool setupBarometer() {
 
   /*
    * BME280 is on Wire1, chained through the IMU
    * board. The main tab already called Wire1.begin().
    * Do not call Wire.begin() or Wire1.begin() here.
    */
 
   /*
    * Print one complete Wire1 scan at startup.
    *
    * 0x6A and 0x1C from the IMU board are expected.
    * Seeing them is proof the chain is alive.
    */
 
   printBarometerBusScan();
 
 
   bool started =
       bringUpBarometer();
 
 
   Serial.print(
       "BME280 address: "
   );
 
 
   if (
       barometerData.i2cAddress ==
       0
   ) {
 
     Serial.println(
         "not detected"
     );
 
   } else {
 
     Serial.print(
         "0x"
     );
 
 
     Serial.println(
         barometerData.i2cAddress,
         HEX
     );
   }
 
 
   Serial.print(
       "BME280 startup: "
   );
 
 
   Serial.println(
       started ?
       "OK" :
       "FAILED"
   );
 
 
   Serial.println();
 
 
   return started;
 }
 
 
 // ==================================================
 // VALIDATION
 // ==================================================
 
 bool pressureIsValid(
     float pressurePa
 ) {
 
   return
       isfinite(pressurePa) &&
       pressurePa >=
           MIN_VALID_PRESSURE_PA &&
       pressurePa <=
           MAX_VALID_PRESSURE_PA;
 }
 
 
 bool temperatureIsValid(
     float temperatureC
 ) {
 
   return
       isfinite(temperatureC) &&
       temperatureC >
           -50.0f &&
       temperatureC <
           100.0f;
 }
 
 
 bool humidityIsValid(
     float humidityPercent
 ) {
 
   return
       isfinite(humidityPercent) &&
       humidityPercent >=
           0.0f &&
       humidityPercent <=
           100.0f;
 }
 
 
 // ==================================================
 // SAMPLE RATE
 // ==================================================
 
 void updateBarometerRate() {
 
   uint32_t nowMs =
       millis();
 
 
   uint32_t elapsedMs =
       static_cast<uint32_t>(
           nowMs -
           barometerRateWindowStartMs
       );
 
 
   if (
       elapsedMs <
       1000
   ) {
 
     return;
   }
 
 
   uint32_t samples =
       barometerData.sampleCount -
       barometerRateWindowSampleStart;
 
 
   barometerData.sampleRateHz =
       (
           samples *
           1000.0f
       ) /
       elapsedMs;
 
 
   publishBarometerTiming();
 
 
   barometerRateWindowStartMs =
       nowMs;
 
 
   barometerRateWindowSampleStart =
       barometerData.sampleCount;
 }
 
 
 // ==================================================
 // HEALTH
 // ==================================================
 
 void updateBarometerHealth() {
 
   if (
       !barometerReady ||
       barometerData.sampleCount ==
           0
   ) {
 
     barometerData.health
         .fresh =
         false;
 
 
     barometerData.health
         .healthy =
         false;
 
 
     return;
   }
 
 
   uint32_t ageUs =
       static_cast<uint32_t>(
           micros() -
           barometerData.lastSampleUs
       );
 
 
   barometerData.health.fresh =
       ageUs <=
       BAROMETER_STALE_AFTER_US;
 
 
   /*
    * Humidity is not part of healthy. A bad humidity
    * sample must not make altitude look unavailable.
    */
   barometerData.health.healthy =
       barometerData.health
               .initialized &&
       barometerData.health
               .i2cDeviceDetected &&
       barometerData.health
               .pressureValid &&
       barometerData.health
               .temperatureValid &&
       barometerData.health
               .baselineReady &&
       barometerData.health
               .fresh;
 }
 
 
 // ==================================================
 // HANDLE FAILED SAMPLE
 // ==================================================
 
 void noteBarometerReadFailure() {
 
   consecutiveBarometerFailures++;
 
 
   if (
       consecutiveBarometerFailures <
       BAROMETER_FAILURE_LIMIT
   ) {
 
     return;
   }
 
 
   markBarometerOffline();
 }
 
 
 // ==================================================
 // UPDATE
 // ==================================================
 
 void updateBarometer() {
 
   // ------------------------------------------------
   // RETRY OFFLINE SENSOR
   // ------------------------------------------------
 
   if (!barometerReady) {
 
     uint32_t nowMs =
         millis();
 
 
     if (
         static_cast<uint32_t>(
             nowMs -
             nextBarometerRetryMs
         ) >=
         BAROMETER_RETRY_MS
     ) {
 
       bringUpBarometer();
     }
 
 
     updateBarometerHealth();
 
 
     return;
   }
 
 
   // ------------------------------------------------
   // LIMIT POLLING TO 25 HZ
   //
   // This is important.
   //
   // updateBarometer() may be called thousands of
   // times per second by loop(), but the BME280
   // measurement from the oversampling choice is
   // about 40 ms, so it is not read any faster.
   // ------------------------------------------------
 
   uint32_t nowUs =
       micros();
 
 
   if (
       static_cast<uint32_t>(
           nowUs -
           lastBarometerPollUs
       ) <
       BAROMETER_EXPECTED_PERIOD_US
   ) {
 
     updateBarometerHealth();
 
 
     return;
   }
 
 
   lastBarometerPollUs =
       nowUs;
 
 
   // ------------------------------------------------
   // READ SENSOR
   //
   // Adafruit_BME280.cpp: readPressure() calls
   // readTemperature() itself before using t_fine,
   // and readHumidity() does the same. Compensation
   // does not depend on the caller reading
   // temperature first. This read is the value we
   // publish; the library reads temperature again
   // inside the pressure call.
   // ------------------------------------------------
 
   float temperatureC =
       activeBme280->readTemperature();
 
 
   float pressurePa =
       activeBme280->readPressure();
 
 
   uint32_t sampleUs =
       micros();
 
 
   bool validPressure =
       pressureIsValid(
           pressurePa
       );
 
 
   bool validTemperature =
       temperatureIsValid(
           temperatureC
       );
 
 
   // ------------------------------------------------
   // PRESSURE FAILURE
   // ------------------------------------------------
 
   if (!validPressure) {
 
     barometerData.errors
         .invalidPressureReads++;
 
 
     barometerData.health
         .pressureValid =
         false;
 
 
     noteBarometerReadFailure();
 
 
     updateBarometerHealth();
 
 
     return;
   }
 
 
   // ------------------------------------------------
   // TEMPERATURE FAILURE
   // ------------------------------------------------
 
   if (!validTemperature) {
 
     barometerData.errors
         .invalidTemperatureReads++;
 
 
     barometerData.health
         .temperatureValid =
         false;
 
 
     noteBarometerReadFailure();
 
 
     updateBarometerHealth();
 
 
     return;
   }
 
 
   /*
    * Successful sample.
    */
 
   consecutiveBarometerFailures =
       0;
 
 
   // ------------------------------------------------
   // PUBLISH SENSOR SAMPLE
   // ------------------------------------------------
 
   barometerData.pressurePa =
       pressurePa;
 
 
   barometerData.temperatureC =
       temperatureC;
 
 
   barometerData.timestampUs =
       barometerMonotonicMicroseconds();
 
 
   barometerData.lastSampleUs =
       sampleUs;
 
 
   barometerData.sampleCount++;
 
 
   barometerData.sequence++;
 
 
   barometerData.health
       .pressureValid =
       true;
 
 
   barometerData.health
       .temperatureValid =
       true;
 
 
   // ------------------------------------------------
   // HUMIDITY
   //
   // A bad humidity read must not discard the
   // pressure sample already stored above, and it
   // must not count toward taking the chip offline.
   // ------------------------------------------------
 
   float humidityPercent =
       activeBme280->readHumidity();
 
 
   if (
       !humidityIsValid(
           humidityPercent
       )
   ) {
 
     barometerData.errors
         .invalidHumidityReads++;
 
 
     barometerData.health
         .humidityValid =
         false;
 
   } else {
 
     barometerData
         .humidityPercent =
         humidityPercent;
 
 
     barometerData.health
         .humidityValid =
         true;
   }
 
 
   // ------------------------------------------------
   // TIMING
   // ------------------------------------------------
 
   recordBarometerTiming(
       sampleUs
   );
 
 
   // ------------------------------------------------
   // GROUND PRESSURE BASELINE
   // ------------------------------------------------
 
   updatePressureBaseline(
       pressurePa
   );
 
 
   // ------------------------------------------------
   // ALTITUDE
   // ------------------------------------------------
 
   if (
       barometerData.health
           .baselineReady
   ) {
 
     barometerData
         .relativeAltitudeM =
         calculateRelativeAltitudeM(
             pressurePa,
             barometerData
                 .baselinePressurePa
         );
 
 
     updateAltitudeFiltering(
         barometerData
             .relativeAltitudeM,
         sampleUs
     );
   }
 
 
   // ------------------------------------------------
   // RATE
   // ------------------------------------------------
 
   updateBarometerRate();
 
 
   // ------------------------------------------------
   // HEALTH
   // ------------------------------------------------
 
   updateBarometerHealth();
 }
 
 
 // ==================================================
 // PUBLIC INTERFACE
 // ==================================================
 
 const BarometerData &getBarometerData() {
 
   return barometerData;
 }
 
 
 bool barometerHealthy() {
 
   return
       barometerData.health
           .healthy;
 }
 
 
 bool barometerAltitudeReady() {
 
   return
       barometerData.health
               .baselineReady &&
       barometerData.health
               .fresh &&
       barometerData.health
               .pressureValid;
 }
 
 
 // ==================================================
 // DIAGNOSTICS
 // ==================================================
 
 /*
  * One-line status for the normal flight log.
  *
  * Serial on the UNO Q is the Linux-side monitor, so every print call
  * is a blocking bridge call. This builds the line in memory and makes
  * exactly one write. Decimals use integer maths because %f is not
  * dependable on this toolchain.
  *
  * Baseline progress is on this line on purpose. Altitude hold stays
  * blocked until the baseline finishes, and without the count it looks
  * like a dead sensor instead of a sensor that is still averaging.
  */
 void printBarometerSummary() {

   char line[160];


   float pressureHpa =
       barometerData.pressurePa / 100.0f;


   /*
    * Temperature and altitude both go negative, below freezing and
    * below the takeoff baseline. Split the sign off and format the
    * magnitude, because truncating a negative float gives a negative
    * fraction and prints as "-3.-4".
    */

   float temperatureC =
       barometerData.temperatureC;

   const char *temperatureSign =
       temperatureC < 0.0f ? "-" : "";

   float temperatureMagnitude =
       fabsf(temperatureC);


   float altitudeM =
       barometerData.filteredAltitudeM;

   const char *altitudeSign =
       altitudeM < 0.0f ? "-" : "";

   float altitudeMagnitude =
       fabsf(altitudeM);


   int length = snprintf(line, sizeof(line),
       "BARO 0x%02X %ld.%ld hPa | %s%ld.%ld C | base %u/%u | alt %s%ld.%02ld m | ready %s\n",
       (unsigned)barometerData.i2cAddress,
       (long)pressureHpa,
       (long)((pressureHpa - (long)pressureHpa) * 10.0f),
       temperatureSign,
       (long)temperatureMagnitude,
       (long)((temperatureMagnitude - (long)temperatureMagnitude) * 10.0f),
       (unsigned)baselinePressureCount,
       (unsigned)BAROMETER_BASELINE_SAMPLES,
       altitudeSign,
       (long)altitudeMagnitude,
       (long)((altitudeMagnitude - (long)altitudeMagnitude) * 100.0f),
       barometerAltitudeReady() ? "yes" : "no");


   if (length <= 0) {

     return;
   }


   if (length > (int)sizeof(line) - 1) {

     length = (int)sizeof(line) - 1;
   }


   writeFlightLog(line, length);
 }
 
 
 void printBarometerDiagnostics() {
 
   const BarometerData &data =
       getBarometerData();
 
 
   Serial.println(
       "BAROMETER"
   );
 
 
   // ------------------------------------------------
   // ADDRESS
   // ------------------------------------------------
 
   Serial.print(
       "I2C detected: "
   );
 
 
   Serial.println(
       data.health.i2cDeviceDetected ?
       "yes" :
       "no"
   );
 
 
   Serial.print(
       "I2C address: "
   );
 
 
   if (
       data.i2cAddress ==
       0
   ) {
 
     Serial.println(
         "none"
     );
 
   } else {
 
     Serial.print(
         "0x"
     );
 
 
     Serial.println(
         data.i2cAddress,
         HEX
     );
   }
 
 
   // ------------------------------------------------
   // HEALTH
   // ------------------------------------------------
 
   Serial.print(
       "Ready: "
   );
 
 
   Serial.println(
       barometerReady ?
       "yes" :
       "no"
   );
 
 
   Serial.print(
       "Configured: "
   );
 
 
   Serial.println(
       barometerConfigured ?
       "yes" :
       "no"
   );
 
 
   Serial.print(
       "Healthy: "
   );
 
 
   Serial.println(
       data.health.healthy ?
       "yes" :
       "no"
   );
 
 
   Serial.print(
       "Fresh: "
   );
 
 
   Serial.println(
       data.health.fresh ?
       "yes" :
       "no"
   );
 
 
   // ------------------------------------------------
   // PRESSURE
   // ------------------------------------------------
 
   Serial.print(
       "Pressure: "
   );
 
 
   Serial.print(
       data.pressurePa,
       2
   );
 
 
   Serial.println(
       " Pa"
   );
 
 
   // ------------------------------------------------
   // TEMPERATURE
   // ------------------------------------------------
 
   Serial.print(
       "Temperature: "
   );
 
 
   Serial.print(
       data.temperatureC,
       2
   );
 
 
   Serial.println(
       " C"
   );
 
 
   // ------------------------------------------------
   // HUMIDITY
   // ------------------------------------------------
 
   Serial.print(
       "Humidity: "
   );
 
 
   Serial.print(
       data.humidityPercent,
       2
   );
 
 
   Serial.println(
       " %"
   );
 
 
   Serial.print(
       "Humidity valid: "
   );
 
 
   Serial.println(
       data.health.humidityValid ?
       "yes" :
       "no"
   );
 
 
   // ------------------------------------------------
   // BASELINE
   // ------------------------------------------------
 
   Serial.print(
       "Baseline pressure: "
   );
 
 
   Serial.print(
       data.baselinePressurePa,
       2
   );
 
 
   Serial.println(
       " Pa"
   );
 
 
   Serial.print(
       "Baseline samples: "
   );
 
 
   Serial.print(
       baselinePressureCount
   );
 
 
   Serial.print(
       " / "
   );
 
 
   Serial.println(
       BAROMETER_BASELINE_SAMPLES
   );
 
 
   Serial.print(
       "Baseline ready: "
   );
 
 
   Serial.println(
       data.health.baselineReady ?
       "yes" :
       "no"
   );
 
 
   // ------------------------------------------------
   // ALTITUDE
   // ------------------------------------------------
 
   Serial.print(
       "Relative altitude: "
   );
 
 
   Serial.print(
       data.relativeAltitudeM,
       3
   );
 
 
   Serial.println(
       " m"
   );
 
 
   Serial.print(
       "Filtered altitude: "
   );
 
 
   Serial.print(
       data.filteredAltitudeM,
       3
   );
 
 
   Serial.println(
       " m"
   );
 
 
   Serial.print(
       "Vertical speed: "
   );
 
 
   Serial.print(
       data.verticalSpeedMps,
       3
   );
 
 
   Serial.println(
       " m/s"
   );
 
 
   // ------------------------------------------------
   // RATE
   // ------------------------------------------------
 
   Serial.print(
       "Sample rate: "
   );
 
 
   Serial.print(
       data.sampleRateHz,
       1
   );
 
 
   Serial.println(
       " Hz"
   );
 
 
   Serial.print(
       "Sample count: "
   );
 
 
   Serial.println(
       data.sampleCount
   );
 
 
   Serial.print(
       "Sequence: "
   );
 
 
   Serial.println(
       data.sequence
   );
 
 
   // ------------------------------------------------
   // TIMING
   // ------------------------------------------------
 
   Serial.print(
       "Timing valid: "
   );
 
 
   Serial.println(
       data.timing.valid ?
       "yes" :
       "no"
   );
 
 
   if (data.timing.valid) {
 
     Serial.print(
         "Interval last/min/max/mean: "
     );
 
 
     Serial.print(
         data.timing
             .lastIntervalUs
     );
 
 
     Serial.print(
         " / "
     );
 
 
     Serial.print(
         data.timing
             .minimumIntervalUs
     );
 
 
     Serial.print(
         " / "
     );
 
 
     Serial.print(
         data.timing
             .maximumIntervalUs
     );
 
 
     Serial.print(
         " / "
     );
 
 
     Serial.print(
         data.timing
             .averageIntervalUs,
         1
     );
 
 
     Serial.println(
         " us"
     );
 
 
     Serial.print(
         "Maximum jitter: "
     );
 
 
     Serial.print(
         data.timing
             .maximumJitterUs
     );
 
 
     Serial.println(
         " us"
     );
   }
 
 
   // ------------------------------------------------
   // FAILURE STATE
   // ------------------------------------------------
 
   Serial.print(
       "Consecutive read failures: "
   );
 
 
   Serial.print(
       consecutiveBarometerFailures
   );
 
 
   Serial.print(
       " / "
   );
 
 
   Serial.println(
       BAROMETER_FAILURE_LIMIT
   );
 
 
   // ------------------------------------------------
   // ERRORS
   // ------------------------------------------------
 
   Serial.print(
       "Address probe failures: "
   );
 
 
   Serial.println(
       data.errors
           .addressProbeFailures
   );
 
 
   Serial.print(
       "Begin failures: "
   );
 
 
   Serial.println(
       data.errors
           .beginFailures
   );
 
 
   Serial.print(
       "Configuration failures: "
   );
 
 
   Serial.println(
       data.errors
           .configurationFailures
   );
 
 
   Serial.print(
       "Invalid pressure reads: "
   );
 
 
   Serial.println(
       data.errors
           .invalidPressureReads
   );
 
 
   Serial.print(
       "Invalid temperature reads: "
   );
 
 
   Serial.println(
       data.errors
           .invalidTemperatureReads
   );
 
 
   Serial.print(
       "Invalid humidity reads: "
   );
 
 
   Serial.println(
       data.errors
           .invalidHumidityReads
   );
 
 
   Serial.print(
       "Baseline failures: "
   );
 
 
   Serial.println(
       data.errors
           .baselineFailures
   );
 
 
   Serial.print(
       "Runtime disconnects: "
   );
 
 
   Serial.println(
       data.errors
           .runtimeDisconnects
   );
 
 
   Serial.println();
 }
 
 
 // ==================================================
 // RESET BASELINE
 // ==================================================
 
 void resetBarometerBaseline() {
 
   baselinePressureSum =
       0;
 
 
   baselinePressureCount =
       0;
 
 
   barometerData
       .baselinePressurePa =
       0;
 
 
   barometerData
       .relativeAltitudeM =
       0;
 
 
   barometerData
       .filteredAltitudeM =
       0;
 
 
   barometerData
       .verticalSpeedMps =
       0;
 
 
   barometerData.health
       .baselineReady =
       false;
 
 
   haveFilteredAltitude =
       false;
 }
 