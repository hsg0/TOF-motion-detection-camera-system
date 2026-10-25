/*
 * WHAT:
 * Production GT-U7 GPS acquisition and parsing module
 * for the Arduino UNO Q.
 *
 * Collects and prepares:
 *
 *   - Latitude
 *   - Longitude
 *   - GPS altitude
 *   - Ground speed
 *   - Course over ground
 *   - Satellite count
 *   - HDOP
 *   - UTC date
 *   - UTC time
 *   - Fix validity
 *   - Data freshness
 *   - UART activity
 *   - NMEA parser statistics
 *   - Update timestamp
 *   - Sample/update count
 *   - Health state
 *   - Error counters
 *
 * WHY:
 * The future motorController / flight-controller layer must not
 * directly read Serial1 or parse NMEA sentences.
 *
 * This file owns the GPS UART and exposes a clean GpsData
 * structure for:
 *
 *   - navigation
 *   - position hold
 *   - ground speed
 *   - course
 *   - GPS altitude reference
 *   - sensor fusion
 *   - mission logic
 *   - failsafe decisions
 *
 * HOW:
 *
 *   GT-U7
 *      ↓
 *   Serial1
 *      ↓
 *   TinyGPSPlus
 *      ↓
 *   updateGps()
 *      ↓
 *   validation / freshness / health
 *      ↓
 *   GpsData
 *      ↓
 *   future motorController / navigation layer
 *
 * IMPORTANT:
 *
 * This file does NOT contain setup() or loop().
 *
 * The main sketch calls:
 *
 *   setupGps();
 *   updateGps();
 *
 * The future controller reads:
 *
 *   getGpsData();
 *
 * WIRING - three wires, confirmed working on this airframe:
 *
 *   GT-U7 TXD      -> UNO Q D0 / USART1_RX     data
 *   micro-USB red  -> UNO Q 5V                 power
 *   micro-USB black -> UNO Q GND               power and UART ground
 *
 *   GT-U7 RXD      -> leave DISCONNECTED
 *   GT-U7 PPS      -> leave disconnected, not used yet
 *
 * WHY RXD STAYS DISCONNECTED:
 *
 * We never send this receiver anything. It streams NMEA on its own,
 * so the return wire earns nothing.
 *
 * It also costs something. D1 is USART1_TX, and on the UNO Q that
 * same usart1 is the Zephyr console, so MCU boot and console text
 * leaves on D1. Wired to RXD, that text lands in the receiver's
 * parser at the wrong baud, and it answers every time with
 *
 *   $GPTXT,01,01,01,NMEA unknown msg
 *
 * Those complaints crowded out the $GPGGA and $GPRMC sentences we
 * actually need. At 9600 baud there are only about 960 bytes per
 * second to spend, so this mattered.
 *
 * POWER NOTES:
 *
 * The 5 V comes in through the micro-USB socket rather than the VCC
 * header pin. A USB cable has four wires and only two are power:
 * red is VBUS, black is ground. White is D- and green is D+, a USB
 * differential pair for the receiver's own USB interface.
 *
 * IMPORTANT:
 * Never wire D+ or D- to D0/D1. They are USB signalling, not a TTL
 * UART. USART1 cannot decode them.
 *
 * The black USB wire does double duty: it powers the board and gives
 * the UART its common ground reference. Without a shared ground the
 * receiver looks powered and still delivers nothing.
 *
 * Budget about 70 mA, with 150 mA peaks while acquiring.
 *
 * UART:
 *
 *   Serial1
 *   9600 baud
 *
 * The receiver confirmed this itself over USB, through a UBX-CFG-PRT
 * poll of its UART1 port:
 *
 *   UART1 baud=9600 in=UBX,NMEA,RTCM out=UBX,NMEA
 *
 * Hardware is a u-blox 7, UBX-G70xx, NMEA protocol version 14.00.
 *
 * IMPORTANT FLIGHT-CONTROL RULE:
 *
 * GPS is NOT required for basic attitude stabilization.
 *
 * A GPS failure must NOT stop:
 *
 *   - LSM6DSOX
 *   - LIS3MDL
 *   - BME280
 *   - attitude stabilization
 *
 * GPS health is independent.
 *
 * TinyGPSPlus keeps the last committed fix after the receiver
 * drops it. health.fixValid, altitudeValid, speedValid, and
 * courseValid are therefore false once that fix is older than
 * GPS_FIX_STALE_AFTER_MS. The last numbers stay stored for
 * logging. gpsFixValid() returns health.fixValid, and the
 * "Fix valid" diagnostic prints that same flag.
 */
 #include <TinyGPSPlus.h>
 #include <math.h>
 #include <stdio.h>
 
 constexpr uint32_t GPS_BAUD = 9600;
 constexpr uint32_t GPS_DATA_STALE_AFTER_MS = 2000;
 constexpr uint32_t GPS_FIX_STALE_AFTER_MS = 2000;
 constexpr uint32_t GPS_STARTUP_GRACE_MS = 30000;
 constexpr uint16_t GPS_MAX_BYTES_PER_LOOP = 128;
 constexpr double GPS_MIN_LATITUDE_DEG = -90.0;
 constexpr double GPS_MAX_LATITUDE_DEG = 90.0;
 constexpr double GPS_MIN_LONGITUDE_DEG = -180.0;
 constexpr double GPS_MAX_LONGITUDE_DEG = 180.0;
 constexpr float GPS_MIN_ALTITUDE_M = -1000.0f;
 constexpr float GPS_MAX_ALTITUDE_M = 60000.0f;
 constexpr float GPS_MAX_GROUND_SPEED_MPS = 300.0f;
 constexpr float GPS_GOOD_HDOP = 2.5f;
 constexpr uint8_t GPS_GOOD_SATELLITE_COUNT = 6;
 
 struct GpsUtcDate {
   bool valid = false;
   uint16_t year = 0;
   uint8_t month = 0;
   uint8_t day = 0;
 };
 
 struct GpsUtcTime {
   bool valid = false;
   uint8_t hour = 0;
   uint8_t minute = 0;
   uint8_t second = 0;
   uint8_t centisecond = 0;
 };
 
 struct GpsErrors {
   uint32_t invalidLocation = 0;
   uint32_t invalidAltitude = 0;
   uint32_t invalidSpeed = 0;
   uint32_t invalidCourse = 0;
   uint32_t checksumFailureEvents = 0;
   uint32_t communicationDropouts = 0;
   uint32_t fixDropouts = 0;
 };
 
 struct GpsHealth {
   bool initialized = false;
   bool communicationHealthy = false;
   bool fixValid = false;
   bool fixFresh = false;
   bool altitudeValid = false;
   bool speedValid = false;
   bool courseValid = false;
   bool satellitesValid = false;
   bool hdopValid = false;
   bool navigationQualityGood = false;
   bool healthy = false;
 };
 
 struct GpsData {
   uint32_t sequence = 0;
   uint32_t updateCount = 0;
   uint64_t timestampUs = 0;
   uint32_t lastByteMs = 0;
   uint32_t lastFixMs = 0;
   uint32_t locationAgeMs = 0;
   double latitudeDeg = 0.0;
   double longitudeDeg = 0.0;
   float altitudeM = 0.0f;
   float groundSpeedMps = 0.0f;
   float groundSpeedKph = 0.0f;
   float courseDeg = 0.0f;
   uint32_t satellites = 0;
   uint32_t satellitesInView = 0;
   uint32_t bestSnrDb = 0;
   float hdop = 0.0f;
   GpsUtcDate date;
   GpsUtcTime time;
   uint32_t charactersProcessed = 0;
   uint32_t sentencesWithFix = 0;
   uint32_t passedChecksum = 0;
   uint32_t failedChecksum = 0;
   uint32_t bytesReceived = 0;
   uint32_t bytesProcessedLastPass = 0;
   GpsHealth health;
   GpsErrors errors;
 };
 
 TinyGPSPlus gps;
 GpsData gpsData;

 char nmeaLineBuf[96];
 uint8_t nmeaLineLen = 0;
 char lastGsvSummary[48] = "none";
 char lastRmcOrGga[84] = "none";
 uint32_t gsvRunningMaxSnr = 0;

 void parseGsvSentence(const char *line) {
   // Format: $GPGSV,totalMsgs,msgNum,satsInView,prn1,el1,az1,snr1,prn2,el2,az2,snr2,...*CS
   int fieldIdx = 0;
   const char *p = line;
   int msgNum = 0;
   int totalMsgs = 0;
   int inView = -1;

   while (*p && *p != '*') {
     if (*p == ',') {
       fieldIdx++;
       p++;
       int val = 0;
       bool hasDigits = false;
       while (*p >= '0' && *p <= '9') {
         val = val * 10 + (*p - '0');
         hasDigits = true;
         p++;
       }
       if (fieldIdx == 1 && hasDigits) {
         totalMsgs = val;
       } else if (fieldIdx == 2 && hasDigits) {
         msgNum = val;
         if (msgNum == 1) {
           gsvRunningMaxSnr = 0;
         }
       } else if (fieldIdx == 3 && hasDigits) {
         inView = val;
       } else if (fieldIdx >= 7 && ((fieldIdx - 7) % 4 == 0) && hasDigits) {
         if ((uint32_t)val > gsvRunningMaxSnr) {
           gsvRunningMaxSnr = (uint32_t)val;
         }
       }
     } else {
       p++;
     }
   }

   if (inView >= 0) {
     gpsData.satellitesInView = (uint32_t)inView;
   }
   if (totalMsgs > 0 && msgNum == totalMsgs) {
     gpsData.bestSnrDb = gsvRunningMaxSnr;
   } else if (gsvRunningMaxSnr > gpsData.bestSnrDb) {
     gpsData.bestSnrDb = gsvRunningMaxSnr;
   }
 }

 void recordRawNmeaChar(char c) {
   if (c == '$') {
     nmeaLineBuf[0] = '$';
     nmeaLineLen = 1;
     return;
   }
   if (nmeaLineLen == 0) {
     return;
   }
   if (c == '\r' || c == '\n') {
     nmeaLineBuf[nmeaLineLen] = '\0';
     if (nmeaLineLen > 6) {
       if (strncmp(nmeaLineBuf + 3, "GSV,", 4) == 0) {
         parseGsvSentence(nmeaLineBuf);
       } else if (strncmp(nmeaLineBuf + 3, "GGA,", 4) == 0 ||
                  strncmp(nmeaLineBuf + 3, "RMC,", 4) == 0) {
         strncpy(lastRmcOrGga, nmeaLineBuf, sizeof(lastRmcOrGga) - 1);
         lastRmcOrGga[sizeof(lastRmcOrGga) - 1] = '\0';
       }
     }
     nmeaLineLen = 0;
     return;
   }
   if (nmeaLineLen < sizeof(nmeaLineBuf) - 1) {
     nmeaLineBuf[nmeaLineLen++] = c;
   } else {
     nmeaLineLen = 0;
   }
 }
 
 uint32_t gpsStartedMs = 0;
 uint32_t previousFailedChecksumCount = 0;
 bool previousCommunicationHealthy = false;
 bool previousFixFresh = false;
 uint32_t lastGpsMicrosReading = 0;
 uint64_t gpsMonotonicUs = 0;
 bool haveGpsMonotonicTime = false;
 
 bool setupGps();
 void updateGps();
 const GpsData &getGpsData();
 bool gpsCommunicationHealthy();
 bool gpsFixValid();
 bool gpsNavigationHealthy();
 void printGpsDiagnostics();
 
 uint64_t gpsMonotonicMicroseconds() {
   uint32_t now = micros();
   if (haveGpsMonotonicTime) {
     gpsMonotonicUs += static_cast<uint32_t>(now - lastGpsMicrosReading);
   }
   lastGpsMicrosReading = now;
   haveGpsMonotonicTime = true;
   return gpsMonotonicUs;
 }
 
 bool gpsLocationIsSane(double latitudeDeg, double longitudeDeg) {
   return isfinite(latitudeDeg) &&
       isfinite(longitudeDeg) &&
       latitudeDeg >= GPS_MIN_LATITUDE_DEG &&
       latitudeDeg <= GPS_MAX_LATITUDE_DEG &&
       longitudeDeg >= GPS_MIN_LONGITUDE_DEG &&
       longitudeDeg <= GPS_MAX_LONGITUDE_DEG;
 }
 
 bool gpsAltitudeIsSane(float altitudeM) {
   return isfinite(altitudeM) &&
       altitudeM >= GPS_MIN_ALTITUDE_M &&
       altitudeM <= GPS_MAX_ALTITUDE_M;
 }
 
 bool gpsSpeedIsSane(float speedMps) {
   return isfinite(speedMps) &&
       speedMps >= 0.0f &&
       speedMps <= GPS_MAX_GROUND_SPEED_MPS;
 }
 
 bool gpsCourseIsSane(float courseDeg) {
   return isfinite(courseDeg) &&
       courseDeg >= 0.0f &&
       courseDeg <= 360.0f;
 }
 
 void updateGpsDate() {
   if (!gps.date.isValid()) {
     gpsData.date.valid = false;
     return;
   }
   gpsData.date.year = gps.date.year();
   gpsData.date.month = gps.date.month();
   gpsData.date.day = gps.date.day();
   gpsData.date.valid = true;
 }
 
 void updateGpsTime() {
   if (!gps.time.isValid()) {
     gpsData.time.valid = false;
     return;
   }
   gpsData.time.hour = gps.time.hour();
   gpsData.time.minute = gps.time.minute();
   gpsData.time.second = gps.time.second();
   gpsData.time.centisecond = gps.time.centisecond();
   gpsData.time.valid = true;
 }
 
 void updateGpsLocation() {
   if (!gps.location.isValid()) {
     gpsData.health.fixValid = false;
     return;
   }
 
   bool locationWasUpdated = gps.location.isUpdated();
   uint32_t locationAgeMs = gps.location.age();
   double latitudeDeg = gps.location.lat();
   double longitudeDeg = gps.location.lng();
 
   if (!gpsLocationIsSane(latitudeDeg, longitudeDeg)) {
     gpsData.errors.invalidLocation++;
     gpsData.health.fixValid = false;
     return;
   }
 
   gpsData.latitudeDeg = latitudeDeg;
   gpsData.longitudeDeg = longitudeDeg;
   gpsData.locationAgeMs = locationAgeMs;
   gpsData.health.fixValid = true;
 
   if (locationWasUpdated) {
     gpsData.lastFixMs = millis();
     gpsData.sequence++;
     gpsData.updateCount++;
   }
 }
 
 void updateGpsAltitude() {
   if (!gps.altitude.isValid() || gps.altitude.age() > GPS_FIX_STALE_AFTER_MS) {
     gpsData.health.altitudeValid = false;
     return;
   }
 
   float altitudeM = static_cast<float>(gps.altitude.meters());
 
   if (!gpsAltitudeIsSane(altitudeM)) {
     gpsData.errors.invalidAltitude++;
     gpsData.health.altitudeValid = false;
     return;
   }
 
   gpsData.altitudeM = altitudeM;
   gpsData.health.altitudeValid = true;
 }
 
 void updateGpsSpeed() {
   if (!gps.speed.isValid() || gps.speed.age() > GPS_FIX_STALE_AFTER_MS) {
     gpsData.health.speedValid = false;
     return;
   }
 
   float speedMps = static_cast<float>(gps.speed.mps());
 
   if (!gpsSpeedIsSane(speedMps)) {
     gpsData.errors.invalidSpeed++;
     gpsData.health.speedValid = false;
     return;
   }
 
   gpsData.groundSpeedMps = speedMps;
   gpsData.groundSpeedKph = static_cast<float>(gps.speed.kmph());
   gpsData.health.speedValid = true;
 }
 
 void updateGpsCourse() {
   if (!gps.course.isValid() || gps.course.age() > GPS_FIX_STALE_AFTER_MS) {
     gpsData.health.courseValid = false;
     return;
   }
 
   float courseDeg = static_cast<float>(gps.course.deg());
 
   if (!gpsCourseIsSane(courseDeg)) {
     gpsData.errors.invalidCourse++;
     gpsData.health.courseValid = false;
     return;
   }
 
   gpsData.courseDeg = courseDeg;
   gpsData.health.courseValid = true;
 }
 
 void updateGpsQuality() {
   if (gps.satellites.isValid()) {
     gpsData.satellites = gps.satellites.value();
     gpsData.health.satellitesValid = true;
   } else {
     gpsData.health.satellitesValid = false;
   }
 
   if (gps.hdop.isValid()) {
     gpsData.hdop = static_cast<float>(gps.hdop.hdop());
     gpsData.health.hdopValid = true;
   } else {
     gpsData.health.hdopValid = false;
   }
 }
 
 void updateGpsParserStatistics() {
   gpsData.charactersProcessed = gps.charsProcessed();
   gpsData.sentencesWithFix = gps.sentencesWithFix();
   gpsData.passedChecksum = gps.passedChecksum();
   gpsData.failedChecksum = gps.failedChecksum();
 
   if (gpsData.failedChecksum > previousFailedChecksumCount) {
     gpsData.errors.checksumFailureEvents +=
         (gpsData.failedChecksum - previousFailedChecksumCount);
     previousFailedChecksumCount = gpsData.failedChecksum;
   }
 }
 
 void updateGpsHealth() {
   uint32_t nowMs = millis();
 
   bool communicationHealthy =
       gpsData.bytesReceived > 0 &&
       static_cast<uint32_t>(nowMs - gpsData.lastByteMs) <= GPS_DATA_STALE_AFTER_MS;
 
   bool startupGrace =
       static_cast<uint32_t>(nowMs - gpsStartedMs) < GPS_STARTUP_GRACE_MS;
 
   if (previousCommunicationHealthy && !communicationHealthy && !startupGrace) {
     gpsData.errors.communicationDropouts++;
   }
 
   if (!startupGrace || communicationHealthy) {
     previousCommunicationHealthy = communicationHealthy;
   }
 
   gpsData.health.communicationHealthy = communicationHealthy;
 
   bool fixFresh =
       gpsData.health.fixValid &&
       gpsData.lastFixMs != 0 &&
       static_cast<uint32_t>(nowMs - gpsData.lastFixMs) <= GPS_FIX_STALE_AFTER_MS;
 
   if (previousFixFresh && !fixFresh) {
     gpsData.errors.fixDropouts++;
   }
 
   previousFixFresh = fixFresh;
   gpsData.health.fixFresh = fixFresh;
 
   if (!fixFresh) {
     gpsData.health.fixValid = false;
     gpsData.health.altitudeValid = false;
     gpsData.health.speedValid = false;
     gpsData.health.courseValid = false;
   }
 
   gpsData.health.navigationQualityGood =
       gpsData.health.fixFresh &&
       gpsData.health.satellitesValid &&
       gpsData.health.hdopValid &&
       gpsData.satellites >= GPS_GOOD_SATELLITE_COUNT &&
       gpsData.hdop <= GPS_GOOD_HDOP;
 
   gpsData.health.healthy =
       gpsData.health.initialized &&
       gpsData.health.communicationHealthy &&
       gpsData.health.fixFresh;
 }
 
 bool setupGps() {
   Serial1.begin(GPS_BAUD);
   gpsStartedMs = millis();
   gpsData.health.initialized = true;
   gpsData.timestampUs = gpsMonotonicMicroseconds();
 
   Serial.println("GT-U7 GPS UART started.");
   Serial.print("GPS baud: ");
   Serial.println(GPS_BAUD);
   Serial.println("Waiting for NMEA data and satellite fix.");
   Serial.println();
 
   return true;
 }
 
 void updateGps() {
   gpsData.timestampUs = gpsMonotonicMicroseconds();
   uint16_t processedThisPass = 0;
 
   while (Serial1.available() && processedThisPass < GPS_MAX_BYTES_PER_LOOP) {
     char incomingByte = static_cast<char>(Serial1.read());
     recordRawNmeaChar(incomingByte);
     gps.encode(incomingByte);
     processedThisPass++;
     gpsData.bytesReceived++;
     gpsData.lastByteMs = millis();
   }
 
   gpsData.bytesProcessedLastPass = processedThisPass;
 
   updateGpsLocation();
   updateGpsAltitude();
   updateGpsSpeed();
   updateGpsCourse();
   updateGpsQuality();
   updateGpsDate();
   updateGpsTime();
   updateGpsParserStatistics();
   updateGpsHealth();
 }
 
 const GpsData &getGpsData() {
   return gpsData;
 }
 
 bool gpsCommunicationHealthy() {
   return gpsData.health.communicationHealthy;
 }
 
 bool gpsFixValid() {
   return gpsData.health.fixValid;
 }
 
 bool gpsNavigationHealthy() {
   return gpsData.health.healthy;
 }
 
 /*
  * One-line status for the normal flight log.
  *
  * Serial on the UNO Q is the Linux-side monitor, so every print call
  * is a blocking bridge call. This builds the line in memory and makes
  * exactly one write.
  *
  * Raw byte count comes first on purpose. It separates the two very
  * different failures:
  *
  *   bytes 0          -> nothing is arriving on D0. Wiring, power, or
  *                       baud. Satellites are not the problem yet.
  *   bytes rising,
  *   fix no           -> the receiver is talking and still searching
  *                       for satellites. Take it outside.
  */
 void printGpsSummary() {

   char line[200];

   int length = snprintf(line, sizeof(line),
       "GPS bytes %lu | ok %lu | bad %lu | sats %lu/%lu | snr %ludB | link %s | fix %s | %s\n",
       (unsigned long)gpsData.bytesReceived,
       (unsigned long)gpsData.passedChecksum,
       (unsigned long)gpsData.failedChecksum,
       (unsigned long)gpsData.satellites,
       (unsigned long)gpsData.satellitesInView,
       (unsigned long)gpsData.bestSnrDb,
       gpsData.health.communicationHealthy ? "ok" : "--",
       gpsData.health.fixFresh ? "yes" : "no",
       lastRmcOrGga);

   if (length <= 0) {
     return;
   }

   if (length > (int)sizeof(line) - 1) {
     length = (int)sizeof(line) - 1;
   }

   writeFlightLog(line, length);
 }

 void printGpsDiagnostics() {
   const GpsData &data = getGpsData();

   Serial.println("GPS");
 
   Serial.print("Initialized: ");
   Serial.println(data.health.initialized ? "yes" : "no");
 
   Serial.print("Communication healthy: ");
   Serial.println(data.health.communicationHealthy ? "yes" : "no");
 
   Serial.print("Bytes received: ");
   Serial.println(data.bytesReceived);
 
   Serial.print("Bytes processed last pass: ");
   Serial.println(data.bytesProcessedLastPass);
 
   Serial.print("Fix valid: ");
   Serial.println(data.health.fixValid ? "yes" : "no");
 
   Serial.print("Fix fresh: ");
   Serial.println(data.health.fixFresh ? "yes" : "no");
 
   Serial.print("Navigation healthy: ");
   Serial.println(data.health.healthy ? "yes" : "no");
 
   Serial.print("Navigation quality good: ");
   Serial.println(data.health.navigationQualityGood ? "yes" : "no");
 
   Serial.print("Latitude: ");
   if (data.health.fixFresh) {
     Serial.println(data.latitudeDeg, 7);
   } else {
     Serial.println("unavailable");
   }
 
   Serial.print("Longitude: ");
   if (data.health.fixFresh) {
     Serial.println(data.longitudeDeg, 7);
   } else {
     Serial.println("unavailable");
   }
 
   Serial.print("Location age: ");
   Serial.print(data.locationAgeMs);
   Serial.println(" ms");
 
   Serial.print("GPS altitude: ");
   if (data.health.altitudeValid) {
     Serial.print(data.altitudeM, 2);
     Serial.println(" m");
   } else {
     Serial.println("unavailable");
   }
 
   Serial.print("Altitude valid: ");
   Serial.println(data.health.altitudeValid ? "yes" : "no");
 
   Serial.print("Ground speed: ");
   if (data.health.speedValid) {
     Serial.print(data.groundSpeedMps, 2);
     Serial.println(" m/s");
   } else {
     Serial.println("unavailable");
   }
 
   Serial.print("Ground speed: ");
   if (data.health.speedValid) {
     Serial.print(data.groundSpeedKph, 2);
     Serial.println(" km/h");
   } else {
     Serial.println("unavailable");
   }
 
   Serial.print("Course: ");
   if (data.health.courseValid) {
     Serial.print(data.courseDeg, 2);
     Serial.println(" deg");
   } else {
     Serial.println("unavailable");
   }
 
   Serial.print("Satellites: ");
   Serial.println(data.satellites);
 
   Serial.print("HDOP: ");
   Serial.println(data.hdop, 2);
 
   Serial.print("UTC date: ");
   if (data.date.valid) {
     Serial.print(data.date.year);
     Serial.print("-");
     if (data.date.month < 10) {
       Serial.print("0");
     }
     Serial.print(data.date.month);
     Serial.print("-");
     if (data.date.day < 10) {
       Serial.print("0");
     }
     Serial.println(data.date.day);
   } else {
     Serial.println("unavailable");
   }
 
   Serial.print("UTC time: ");
   if (data.time.valid) {
     if (data.time.hour < 10) {
       Serial.print("0");
     }
     Serial.print(data.time.hour);
     Serial.print(":");
     if (data.time.minute < 10) {
       Serial.print("0");
     }
     Serial.print(data.time.minute);
     Serial.print(":");
     if (data.time.second < 10) {
       Serial.print("0");
     }
     Serial.println(data.time.second);
   } else {
     Serial.println("unavailable");
   }
 
   Serial.print("Sequence: ");
   Serial.println(data.sequence);
 
   Serial.print("Location updates: ");
   Serial.println(data.updateCount);
 
   Serial.print("NMEA characters: ");
   Serial.println(data.charactersProcessed);
 
   Serial.print("Sentences with fix: ");
   Serial.println(data.sentencesWithFix);
 
   Serial.print("Passed checksums: ");
   Serial.println(data.passedChecksum);
 
   Serial.print("Failed checksums: ");
   Serial.println(data.failedChecksum);
 
   Serial.print("Invalid location values: ");
   Serial.println(data.errors.invalidLocation);
 
   Serial.print("Invalid altitude values: ");
   Serial.println(data.errors.invalidAltitude);
 
   Serial.print("Invalid speed values: ");
   Serial.println(data.errors.invalidSpeed);
 
   Serial.print("Invalid course values: ");
   Serial.println(data.errors.invalidCourse);
 
   Serial.print("Checksum failures observed: ");
   Serial.println(data.errors.checksumFailureEvents);
 
   Serial.print("Communication dropouts: ");
   Serial.println(data.errors.communicationDropouts);
 
   Serial.print("Fix dropouts: ");
   Serial.println(data.errors.fixDropouts);
 
   Serial.println();
 }