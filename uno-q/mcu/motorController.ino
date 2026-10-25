/*
 * WHAT:
 * Main flight-controller file for the Arduino UNO Q.
 *
 * This file owns:
 *
 *   - setup()
 *   - loop()
 *   - sensor orchestration
 *   - the aircraft decision: what the drone is allowed to do right now
 *
 * Sensor modules, each one collecting and publishing its own data:
 *
 *   imu-lsm6dsox-lis3mdl-qwiic.ino
 *     -> LSM6DSOX + LIS3MDL
 *     -> FlightSensorData
 *
 *   baro-pressure-bme280.ino
 *     -> BME280
 *     -> BarometerData
 *
 *   gps-gt-u7.ino
 *     -> GT-U7 GPS
 *     -> GpsData
 *
 * WHY:
 * Hardware talk belongs in the sensor modules. Decisions belong here.
 *
 * This file never touches I2C registers, NMEA sentences, or a sensor
 * driver. It reads prepared data through each module's public
 * functions and decides from that.
 *
 * HOW:
 *
 *   setup()
 *      |
 *      +--> Wire1.begin()
 *      +--> setupFlightSensors()
 *      +--> setupBarometer()
 *      +--> setupGps()
 *
 *   loop()
 *      |
 *      +--> updateSensors()          collect from all three modules
 *      +--> updateMotorController()  decide what is allowed
 *      +--> diagnostics              optional Serial report
 *      +--> clearFlightSensorEvents()
 *
 * IMPORTANT:
 *
 * Every sensor update runs on every pass. One failed sensor must not
 * stop the others.
 *
 * Accelerometer and gyroscope are the critical attitude sensors.
 * Without them the aircraft must not fly.
 *
 * Movement uses those two plus the magnetometer and the barometer.
 * GPS adds a tighter position when it has a fix. The TF-Luna adds
 * distance awareness when a frame is arriving. Neither one is
 * required before the aircraft may move.
 *
 * No motor output exists yet. ESC pins are not chosen and the frame
 * mixing is not defined, so this file decides and reports only. The
 * motor mixer and ESC output are the next vertical slice, and they
 * belong in this file behind decidedCapabilities.
 */


 #include <Wire.h>
 #include <stdio.h>


 // ==================================================
 // SENSOR DATA FORWARD DECLARATIONS
 // ==================================================

 /*
  * The full structures are owned by the sensor files.
  *
  * Arduino compiles this main file before those definitions, so this
  * file may only hold references to these types, never read their
  * members. Every value this file needs arrives through a function
  * below.
  */

 struct FlightSensorData;

 struct BarometerData;

 struct GpsData;


 /*
  * These are internal types of the IMU module, not data this file
  * reads.
  *
  * Arduino builds one program out of every tab and copies a prototype
  * for every function into this file, at the top. Some IMU helper
  * functions take these types, so their prototypes land here before
  * the IMU tab defines them. Naming the types here is what lets those
  * prototypes compile.
  *
  * Add a line here if a new sensor-tab helper takes a type that the
  * same tab defines.
  */

 enum class OrientationFace : uint8_t;

 struct Vector3Float;

 struct Vector3Int16;

 struct SampleTiming;

 struct TimingAccumulator;


 // ==================================================
 // IMU / MAGNETOMETER PUBLIC INTERFACE
 // ==================================================

 bool setupFlightSensors();

 void updateFlightSensors();

 const FlightSensorData &getFlightSensorData();

 bool flightAttitudeSensorsHealthy();

 bool flightHeadingSensorHealthy();

 bool flightFreeFallDetected();

 void clearFlightSensorEvents();

 uint32_t totalSensorErrors();

 void printFlightSensorStartupReport();

 void printFlightSensorSummary();

 void printFlightSensorErrorBreakdown();

 void printFlightSensorDiagnostics();


 // ==================================================
 // BME280 PUBLIC INTERFACE
 // ==================================================

 bool setupBarometer();

 void updateBarometer();

 const BarometerData &getBarometerData();

 bool barometerHealthy();

 bool barometerAltitudeReady();

 void printBarometerSummary();

 void printBarometerDiagnostics();


 // ==================================================
 // GT-U7 GPS PUBLIC INTERFACE
 // ==================================================

 bool setupGps();

 void updateGps();

 const GpsData &getGpsData();

 bool gpsCommunicationHealthy();

 bool gpsFixValid();

 bool gpsNavigationHealthy();

 void printGpsSummary();

 void printGpsDiagnostics();


 // ==================================================
 // PICO 1 PUBLIC INTERFACE
 // ==================================================

 void setupPico1Link();

 void updatePico1Link();

 bool pico1LinkPresent();

 bool pico1LunaDistanceFresh();

 uint16_t pico1LunaDistanceCm();

 void printPico1Summary();


 // ==================================================
 // OLED DISPLAY PUBLIC INTERFACE
 // ==================================================

 void setupOledDisplay();

 void updateOledDisplay();

 bool oledDisplayPresent();

 void printOledSummary();


 // ==================================================
 // MOTOR CONTROLLER CONFIGURATION
 // ==================================================

 constexpr uint32_t SERIAL_BAUD =
     115200;


 /*
  * Serial is the USB log. Serial3 is the same log on the pins marked
  * SDA and SCL: D20 receives from the Pi, D21 sends to the Pi.
  * Both sides use 115200. D0 and D1 stay with the GPS.
  */
 void writeFlightLog(const char* line, int length) {

   if (line == nullptr || length <= 0) {

     return;
   }

   Serial.write(line, length);
   Serial3.write(line, length);
 }


 constexpr uint32_t SERIAL_WAIT_MS =
     500;


 /*
  * The one diagnostics switch for the whole sketch.
  *
  * The sensor modules print whenever they are asked to. This file is
  * the only place that decides when, so there is one switch to find
  * instead of one per module.
  *
  * This prints four compact lines per interval, one Serial write each.
  */
 constexpr bool MOTOR_CONTROLLER_DEBUG =
     true;


 /*
  * The full per-sensor dump. Leave this false unless you are bench
  * testing one sensor.
  *
  * IMPORTANT - WHY THIS IS OFF:
  * On the UNO Q, Serial is not a UART. It is the Linux-side monitor
  * reached over the Arduino bridge, and every single print call is a
  * blocking RPC round trip. The dump makes a few hundred of those, so
  * it stalls loop() for seconds. Measured effect of leaving it on:
  *
  *   accel/gyro/mag sample rate fell to 0.3 Hz
  *   accel interval stretched to about 3,330,000 us
  *   the LSM6DSOX FIFO overran at 512 words
  *   the barometer baseline never finished, so altitude hold stayed
  *   blocked
  *
  * None of that was a hardware fault. It was Serial cost inside the
  * control loop. Keep heavy printing out of the flight path.
  */
 constexpr bool MOTOR_CONTROLLER_VERBOSE_DEBUG =
     false;


 constexpr uint32_t MOTOR_CONTROLLER_DEBUG_INTERVAL_MS =
     1000;


 // ==================================================
 // AIRCRAFT DECISION
 // ==================================================

 /*
  * What the aircraft is allowed to do on this pass.
  *
  * Each field answers one question, and each answer comes from
  * sensor health rather than from a hopeful default. Everything
  * starts false so a sensor we never heard from can never permit
  * flight.
  */
 struct FlightCapabilities {

   bool attitudeStabilizationAllowed = false;

   bool headingCorrectionAllowed = false;

   bool altitudeHoldAllowed = false;

   // IMU, magnetometer, and barometer together. This is the permission
   // to move. GPS and the lidar are not part of it.
   bool movementAllowed = false;

   bool positionHoldAllowed = false;

   bool returnToHomeAllowed = false;

   // Extras. False means that extra is absent, not that flight is stopped.
   bool gpsPositioningAvailable = false;

   bool lunaAwarenessAvailable = false;

   bool emergencyStopRequired = true;
 };


 // ==================================================
 // CONTROLLER STATE
 // ==================================================

 bool flightSensorsStarted =
     false;


 bool barometerStarted =
     false;


 bool gpsStarted =
     false;


 FlightCapabilities decidedCapabilities;


 uint32_t lastControllerDebugMs =
     0;


 // ==================================================
 // SENSOR SETUP
 // ==================================================

 void setupSensors() {

   /*
    * Wire1 is the shared Qwiic bus, started once here.
    *
    * Current devices:
    *
    *   0x6A -> LSM6DSOX
    *   0x1C -> LIS3MDL
    *   0x77 -> BME280
    *
    * The sensor modules must not call Wire1.begin() again.
    */

   Wire1.begin();


   flightSensorsStarted =
       setupFlightSensors();


   barometerStarted =
       setupBarometer();


   /*
    * setupGps() starts Serial1 at the GPS baud rate.
    *
    * True means the UART opened. It does not mean we have
    * satellites yet.
    */

   gpsStarted =
       setupGps();
 }


 // ==================================================
 // STARTUP REPORT
 // ==================================================

 void printControllerStartupStatus() {

   Serial.println();

   Serial.println(
       "========================================"
   );

   Serial.println(
       "UNO Q MOTOR CONTROLLER"
   );

   Serial.println(
       "========================================"
   );


   Serial.print(
       "IMU / magnetometer startup: "
   );

   Serial.println(
       flightSensorsStarted ?
       "OK" :
       "FAILED - sensor module will keep retrying"
   );


   printFlightSensorStartupReport();


   Serial.print(
       "BME280 startup: "
   );

   Serial.println(
       barometerStarted ?
       "OK" :
       "FAILED - sensor module will keep retrying"
   );


   Serial.print(
       "GT-U7 startup: "
   );

   Serial.println(
       gpsStarted ?
       "UART STARTED / waiting for fix" :
       "UART FAILED"
   );


   Serial.println();

   Serial.println(
       "Motor output: NOT WIRED"
   );

   Serial.println(
       "Sensor acquisition running."
   );

   Serial.println(
       "========================================"
   );

   Serial.println();
 }


 // ==================================================
 // SENSOR ACQUISITION
 // ==================================================

 void updateSensors() {

   /*
    * Always call every updater, every pass.
    *
    * Never guard one sensor behind another sensor's health. Each
    * module owns its own retry timer, so a module that is down
    * recovers by being called.
    */

   updateFlightSensors();

   updateBarometer();

   updateGps();
 }


 // ==================================================
 // FLIGHT DECISION
 // ==================================================

 /*
  * Turns sensor health into permissions.
  *
  * Attitude is the gate. Accelerometer and gyroscope carry the
  * aircraft, so nothing is permitted without them, and free fall
  * overrides everything because the airframe is already falling.
  *
  * Movement is attitude plus a healthy magnetometer and a ready
  * barometer. Position hold and return to home use that same
  * permission. A GPS fix and a fresh Luna frame add depth on top.
  * They never take movement away.
  */
 void updateMotorController() {

   /*
    * Start from the blocked defaults every pass, so a permission
    * granted earlier can never survive the sensor that justified it.
    */

   decidedCapabilities =
       FlightCapabilities();


   bool attitudeReady =
       flightAttitudeSensorsHealthy();


   bool falling =
       flightFreeFallDetected();


   decidedCapabilities.emergencyStopRequired =
       !attitudeReady ||
       falling;


   if (decidedCapabilities.emergencyStopRequired) {

     /*
      * Every capability stays false. Stop the aircraft rather than
      * fall back to a lesser flight mode.
      */

     return;
   }


   decidedCapabilities.attitudeStabilizationAllowed =
       true;


   decidedCapabilities.headingCorrectionAllowed =
       flightHeadingSensorHealthy();


   decidedCapabilities.altitudeHoldAllowed =
       barometerAltitudeReady();


   decidedCapabilities.movementAllowed =
       decidedCapabilities.headingCorrectionAllowed &&
       decidedCapabilities.altitudeHoldAllowed;


   decidedCapabilities.positionHoldAllowed =
       decidedCapabilities.movementAllowed;


   decidedCapabilities.returnToHomeAllowed =
       decidedCapabilities.movementAllowed;


   decidedCapabilities.gpsPositioningAvailable =
       gpsNavigationHealthy();


   decidedCapabilities.lunaAwarenessAvailable =
       pico1LunaDistanceFresh();


   /*
    * Motors are not wired yet, so there is nothing to drive and
    * nothing to stop. decidedCapabilities is the input the motor
    * mixer will read once ESC pins exist:
    *
    *   emergencyStopRequired        -> all ESCs to zero throttle
    *   attitudeStabilizationAllowed -> run the attitude loop
    *   movementAllowed              -> fly on IMU, magnetometer, barometer
    *   gpsPositioningAvailable      -> tighten that flight with GPS
    *   lunaAwarenessAvailable       -> use the lidar distance
    */
 }


 // ==================================================
 // CONTROLLER DIAGNOSTICS
 // ==================================================

 void printMotorControllerDiagnostics() {

   if (!MOTOR_CONTROLLER_DEBUG) {

     return;
   }


   uint32_t nowMs =
       millis();


   if (
       lastControllerDebugMs != 0 &&
       static_cast<uint32_t>(
           nowMs -
           lastControllerDebugMs
       ) <
       MOTOR_CONTROLLER_DEBUG_INTERVAL_MS
   ) {

     return;
   }


   lastControllerDebugMs =
       nowMs;


   /*
    * One line per sensor module, then one line for the decision.
    * Four Serial writes per interval, which the loop can absorb.
    */

   printFlightSensorSummary();

   printFlightSensorErrorBreakdown();

   printBarometerSummary();

   printGpsSummary();

   printPico1Summary();

   printOledSummary();


   char line[160];

   int length = snprintf(line, sizeof(line),
       "CTRL stop %s | move %s | att %s | hdg %s | alt %s | pos %s | rth %s | gps %s | luna %s\n",
       decidedCapabilities.emergencyStopRequired ? "YES" : "no",
       decidedCapabilities.movementAllowed ? "ok" : "--",
       decidedCapabilities.attitudeStabilizationAllowed ? "ok" : "--",
       decidedCapabilities.headingCorrectionAllowed ? "ok" : "--",
       decidedCapabilities.altitudeHoldAllowed ? "ok" : "--",
       decidedCapabilities.positionHoldAllowed ? "ok" : "--",
       decidedCapabilities.returnToHomeAllowed ? "ok" : "--",
       decidedCapabilities.gpsPositioningAvailable ? "ok" : "--",
       decidedCapabilities.lunaAwarenessAvailable ? "ok" : "--");


   if (length > 0) {

     if (length > (int)sizeof(line) - 1) {

       length = (int)sizeof(line) - 1;
     }

     writeFlightLog(line, length);
   }


   if (!MOTOR_CONTROLLER_VERBOSE_DEBUG) {

     return;
   }


   /*
    * Bench mode only. This is the few-hundred-call dump that stalls
    * the loop, so it is never part of normal running.
    */

   printFlightSensorDiagnostics();

   printBarometerDiagnostics();

   printGpsDiagnostics();
 }


 // ==================================================
 // SETUP
 // ==================================================

 void setup() {

   Serial.begin(
       SERIAL_BAUD
   );

   Serial3.begin(
       SERIAL_BAUD
   );


   uint32_t serialStartMs =
       millis();


   while (
       !Serial &&
       static_cast<uint32_t>(
           millis() -
           serialStartMs
       ) <
       SERIAL_WAIT_MS
   ) {

     delay(10);
   }


   Serial.println();

   Serial.println(
       "Starting UNO Q flight system..."
   );


   setupSensors();

   setupPico1Link();

   setupOledDisplay();


   printControllerStartupStatus();


   lastControllerDebugMs =
       millis();
 }


 // ==================================================
 // LOOP
 // ==================================================

 void loop() {

   /*
    * Nothing here may block. No delay(). Every pass services all
    * three sensor modules and then decides.
    */


   // ------------------------------------------------
   // 1. COLLECT
   // ------------------------------------------------

   updateSensors();

   updatePico1Link();

   updateOledDisplay();


   // ------------------------------------------------
   // 2. DECIDE
   // ------------------------------------------------

   updateMotorController();


   // ------------------------------------------------
   // 3. REPORT
   // ------------------------------------------------

   printMotorControllerDiagnostics();


   // ------------------------------------------------
   // 4. CONSUME LATCHED EVENTS
   // ------------------------------------------------

   /*
    * The IMU module latches tap, tilt, free fall, and significant
    * motion until someone clears them. The decision above has read
    * them, so clearing here keeps every flag meaning "seen since
    * the last pass" instead of "seen once, long ago".
    */

   clearFlightSensorEvents();
 }
