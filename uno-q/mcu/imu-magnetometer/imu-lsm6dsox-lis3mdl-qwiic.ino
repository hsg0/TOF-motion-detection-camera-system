/*
 * WHAT:
 * Reads the LSM6DSOX + LIS3MDL flight set on an Arduino Uno Q:
 *
 *   LSM6DSOX:
 *     - Accelerometer X/Y/Z
 *     - Gyroscope X/Y/Z
 *     - Temperature
 *     - Data-ready flags (accel, gyro, temperature)
 *     - FIFO level, watermark, overrun, newest accel/gyro/temperature word
 *     - Hardware timestamp counter and FIFO timestamp words
 *     - Raw accelerometer and gyroscope register counts
 *     - WHO_AM_I identity check and measured sample rate
 *     - Single tap, double tap
 *     - Step count and step detector
 *     - Tilt, free-fall, 6D orientation, significant motion
 *
 *   LIS3MDL:
 *     - Magnetic field X/Y/Z
 *     - Temperature
 *     - Data-ready and overrun
 *     - Raw magnetic register counts
 *     - WHO_AM_I identity check and measured sample rate
 *
 *   Diagnostics:
 *     - Per-stream sample rate
 *     - Interval last, minimum, maximum, mean, and jitter
 *     - Shared LSM6DSOX timestamp interval
 *     - Bus error counters
 *     - Freshness and health
 *
 * WHY:
 * The continuous 9-axis samples are not enough for the flight log.
 *
 * A future flight controller needs:
 *     - converted engineering units
 *     - raw sensor values
 *     - accurate timestamps
 *     - FIFO state
 *     - data-ready information
 *     - sensor identity / health information
 *     - actual measured sample rate
 *
 * This gives us a much stronger base for GPS-denied attitude
 * estimation and later sensor fusion.
 *
 * HOW:
 * The LSM6DSOX and LIS3MDL share the UNO Q Qwiic I2C bus using Wire1.
 * The BME280 is chained on that same connector, after the IMU board.
 *
 * motorController.ino owns setup() and loop(). It calls
 * setupFlightSensors() once and updateFlightSensors() every pass.
 *
 * printFlightSensorDiagnostics() always prints when called.
 * motorController decides when, so Serial output never controls
 * the sample rate.
 *
 * Embedded functions are programmed through the sensor's
 * embedded-function bank.
 *
 * IMPORTANT:
 * LSM6DSOX I2C address = 0x6A
 * LIS3MDL I2C address  = 0x1C
 *
 * LSM6DSOX WHO_AM_I expected = 0x6C
 * LIS3MDL WHO_AM_I expected  = 0x3D
 *
 * The LSM6DSOX hardware timestamp has a resolution of 25 us/tick.
 * Accel and gyro share that one 25 us clock.
 *
 * This sketch never halts. Each chip is brought up on its own and
 * retried about every 100 ms. A dead LSM6DSOX does not skip the
 * LIS3MDL, and the reverse.
 *
 * The IMU module and the BME280 module share one physical Wire1
 * bus. They stay independent in code, each with its own retry
 * timer and health, but they are no longer electrically isolated.
 *
 * This file is a sensor module only. It has no setup() and no
 * loop(). Those belong to motorController.ino, which is the one
 * file that decides what the aircraft does.
 *
 * IMU_STALE_AFTER_US and MAG_STALE_AFTER_US are health limits,
 * not control-loop timing.
 * A failed user-bank select skips IMU output reads for that pass.
 */


/*
 * ==================================================
 * HOW TO READ THIS FILE
 * ==================================================
 *
 * Two chips sit on one wire pair. This file does four things with
 * them, in this order, and the file is laid out in that same order:
 *
 *   1. Describe the chips      names and numbers from their manuals
 *   2. Describe our data       the shapes we store readings in
 *   3. Wake the chips up       bringUpImu() and bringUpMag()
 *   4. Read them, forever      updateFlightSensors(), once per pass
 *
 * WORDS USED FOR NUMBER TYPES
 *
 * C++ makes you say how big a number is and whether it can be
 * negative. The names look cryptic but they are simple:
 *
 *   uint8_t    whole number, 0 to 255.          One byte.
 *                "u" = unsigned = never negative.
 *                "int" = integer = whole number.
 *                "8" = eight bits.
 *   uint16_t   whole number, 0 to about 65,000.
 *   uint32_t   whole number, 0 to about 4.29 billion.
 *   uint64_t   whole number, enormous.
 *   int16_t    whole number that CAN be negative,
 *                about -32,000 to +32,000. Sensor readings use this
 *                because tilting one way is negative.
 *   float      a number with a decimal point, like 9.81.
 *   bool       true or false, nothing else.
 *   char       one letter or symbol.
 *
 * OTHER WORDS
 *
 *   constexpr  a value fixed when the program is built and never
 *                changed afterwards. A permanent label for a number.
 *   struct     a group of related values kept under one name.
 *   enum class a short list of allowed names, like the six faces of
 *                a box.
 *   &          "the real thing, not a copy". Passing a big struct
 *                this way avoids copying it.
 *   *          a pointer, meaning "the address where a value lives".
 *   0x6A       a number written in hex. Chip manuals use hex.
 *                0x6A is 106 in ordinary counting.
 *
 * TIME UNITS, WHICH THE NAMES ALWAYS SPELL OUT
 *
 *   Us  = microseconds.  One millionth of a second.
 *   Ms  = milliseconds.  One thousandth of a second.
 *   Hz  = times per second.
 *
 * So IMU_STALE_AFTER_US is "how many microseconds before an IMU
 * reading counts as too old", and SENSOR_RETRY_MS is "how many
 * milliseconds to wait before trying a dead chip again".
 *
 * A REGISTER, WHICH IS THE WHOLE TRICK OF TALKING TO A CHIP
 *
 * A sensor chip is a box full of numbered mailboxes. Each mailbox is
 * called a register and holds one byte. To use the chip you write a
 * byte into a mailbox to set something up, or read a byte out of a
 * mailbox to get an answer. Every REG_ name below is one mailbox
 * number, copied from the chip's manual.
 */


/*
 * Libraries this file borrows.
 *
 * Code we did not write, so these names stay in the style their
 * authors chose.
 */

#include <Wire.h>                     // Arduino's I2C two-wire talking.
#include <stdio.h>                    // snprintf(), builds text into memory.
#include <string.h>                   // Standard C text helpers.
#include <Adafruit_LSM6DSOX.h>        // Driver for the accelerometer and gyroscope chip.
#include <Adafruit_LIS3MDL.h>         // Driver for the magnetometer chip.
#include <Adafruit_Sensor.h>          // Shared Adafruit constants, including gravity.
#include <Adafruit_BusIO_Register.h>  // Lets us poke single registers ourselves.


// ==================================================
// CONFIGURATION
// ==================================================

/*
 * Our own choices live here. Every one of these is a decision, not a
 * fact about the chips.
 */

/*
 * How old a reading may be before we stop trusting it.
 *
 * These answer "is this sensor still alive and talking", not "how
 * fast must the flight loop run". A reading older than this means
 * something stopped, so the health flags go false.
 */

// 50,000 microseconds is 50 thousandths of a second.
// The accelerometer and gyroscope produce 104 readings a second, so
// one arrives about every 9.6 ms. 50 ms means roughly five missed in
// a row before we complain.
constexpr uint32_t IMU_STALE_AFTER_US = 50000;

// 100,000 microseconds is a tenth of a second.
// The magnetometer is slower and matters less, so it gets more slack.
constexpr uint32_t MAG_STALE_AFTER_US = 100000;

// If a chip did not wake up, wait this long before trying again.
// We retry forever and never give up, because a loose wire pushed
// back in should start working without a reboot.
constexpr uint32_t SENSOR_RETRY_MS = 100;

// Read the hardware step counter at most ten times a second.
// Reading it is expensive because it needs a register bank switch,
// and nobody needs step counts faster than this.
constexpr uint32_t STEP_READ_INTERVAL_MS = 100;

// Work out the measured readings-per-second once a second.
// Count the readings, wait a second, divide. That is the whole method.
constexpr uint32_t RATE_WINDOW_MS = 1000;

/*
 * Facts about the chips now, straight from their manuals. These are
 * not ours to choose.
 */

// Every chip on an I2C wire pair needs its own address so we can say
// who we are talking to. The accelerometer and gyroscope answer here.
constexpr uint8_t IMU_ADDRESS = 0x6A;

// The magnetometer answers at a different address, so both chips can
// share the same two wires without confusion.
constexpr uint8_t MAG_ADDRESS = 0x1C;

/*
 * The identity check.
 *
 * Each chip holds one register with a fixed value burned into it. We
 * read it and compare. If the number comes back wrong we are talking
 * to the wrong chip, or to a bad connection, and we should not
 * pretend its readings mean anything.
 */

// A real LSM6DSOX always answers 0x6C.
constexpr uint8_t LSM6DSOX_WHO_AM_I_VALUE = 0x6C;

// A real LIS3MDL always answers 0x3D.
constexpr uint8_t LIS3MDL_WHO_AM_I_VALUE = 0x3D;

/*
 * The speeds we ask the chips to run at.
 *
 * We tell the chip these numbers during startup, then measure what
 * actually arrives. Comparing the two is how we notice trouble.
 */

// We ask the accelerometer for 104 readings every second.
constexpr uint32_t ACCEL_EXPECTED_RATE_HZ = 104;

// Same for the gyroscope. They share one chip and one clock.
constexpr uint32_t GYRO_EXPECTED_RATE_HZ = 104;

/*
 * The LIS3MDL ran at 155 Hz and the loop reads it about 147 times a
 * second, so the chip overwrote samples the loop had not read yet and
 * set its overrun flag a few times a second.
 *
 * 80 Hz is below the loop rate with room to spare, so every sample now
 * gets read. The magnetometer only corrects heading, which drifts
 * slowly, so 80 Hz is still far more than the controller needs.
 *
 * Keep this matched to setDataRate() in bringUpMag(). It is what the
 * timing and freshness diagnostics measure against.
 */
constexpr uint32_t MAG_EXPECTED_RATE_HZ = 80;

/*
 * Turn "readings per second" into "microseconds between readings",
 * because the timing checks measure gaps, not rates.
 *
 * There are 1,000,000 microseconds in a second, so divide that by the
 * rate. 1,000,000 / 104 is about 9,615 microseconds, near 9.6 ms.
 * The UL on the end just tells C++ the number is a big one.
 *
 * The compiler does this division once while building. It costs the
 * drone nothing in the air.
 */

// About 9,615 microseconds between accelerometer readings.
constexpr uint32_t ACCEL_EXPECTED_PERIOD_US = 1000000UL / ACCEL_EXPECTED_RATE_HZ;

// The same 9,615 microseconds for the gyroscope.
constexpr uint32_t GYRO_EXPECTED_PERIOD_US = 1000000UL / GYRO_EXPECTED_RATE_HZ;

// 1,000,000 / 80 is exactly 12,500 microseconds, or 12.5 ms.
constexpr uint32_t MAG_EXPECTED_PERIOD_US = 1000000UL / MAG_EXPECTED_RATE_HZ;


// ==================================================
// LSM6DSOX MAILBOX NUMBERS
// ==================================================

/*
 * Every number below is a mailbox address from the chip's manual. We
 * give them names so the code reads as English instead of hex.
 *
 * THE TWO BANKS, WHICH TRIP PEOPLE UP
 *
 * This chip has more mailboxes than its numbering allows, so it keeps
 * two sets and shows one at a time. Switching sets is called
 * selecting a bank.
 *
 *   The user bank holds the ordinary readings.
 *   The embedded bank holds the step counter and motion features.
 *
 * The danger is simple: if we forget to switch back, mailbox number 5
 * means something completely different and we read nonsense while
 * believing it. So every trip into the embedded bank switches back
 * immediately, and updateFlightSensors() re-selects the user bank
 * before reading anything.
 */

// Bank number 0 is the everyday readings bank.
constexpr uint8_t BANK_USER = 0;

// Bank number 2 is the special features bank.
constexpr uint8_t BANK_EMBEDDED = 2;

// The mailbox that chooses which bank is showing.
constexpr uint8_t REG_FUNC_CFG_ACCESS = 0x01;

// Holds the fixed identity number, 0x6C. Our "are you really you" check.
constexpr uint8_t REG_WHO_AM_I = 0x0F;

// A settings mailbox. We use one bit of it, called IF_INC.
constexpr uint8_t REG_CTRL3_C = 0x12;

// Another settings mailbox. One bit here starts the chip's own clock.
constexpr uint8_t REG_CTRL10_C = 0x19;

// Reports wake-up and free-fall. Free fall means the drone is falling.
constexpr uint8_t REG_WAKE_UP_SRC = 0x1B;

// Reports taps, as in someone tapping the board.
constexpr uint8_t REG_TAP_SRC = 0x1C;

// Reports which face is pointing up. 6D means six directions.
constexpr uint8_t REG_D6D_SRC = 0x1D;

// The "is fresh data waiting" mailbox. We check this before reading.
constexpr uint8_t REG_STATUS = 0x1E;

/*
 * The first mailbox of the readings block.
 *
 * The chip lays fourteen bytes in a row starting here: temperature,
 * then the gyroscope's three axes, then the accelerometer's three.
 * We grab all fourteen in one go so every value belongs to the same
 * instant. Reading them one at a time would smear the readings across
 * different moments and corrupt attitude.
 */
constexpr uint8_t REG_OUT_TEMP_L = 0x20;

// Reports the motion features, readable without a bank switch.
constexpr uint8_t REG_EMB_FUNC_STATUS_MAINPAGE = 0x35;

/*
 * The FIFO, which is the chip's own waiting room.
 *
 * FIFO means first in, first out, like a queue at a counter. The chip
 * can stack readings here while we are busy. We mostly read live
 * values instead, but the queue depth is a useful warning light: if it
 * fills up, our loop is running too slowly.
 */

// First of two mailboxes reporting how full the queue is.
constexpr uint8_t REG_FIFO_STATUS1 = 0x3A;

// Second one. Also carries the "it overflowed" flag.
constexpr uint8_t REG_FIFO_STATUS2 = 0x3B;

/*
 * The chip's own clock.
 *
 * Four mailboxes in a row hold a counter that ticks steadily. It is
 * better than asking our own processor what time it is, because this
 * clock belongs to the chip taking the readings.
 */
constexpr uint8_t REG_TIMESTAMP0 = 0x40;

// Each tick of that counter is 25 microseconds. 40 ticks is 1 ms.
constexpr uint32_t TIMESTAMP_TICK_US = 25;

/*
 * Settings mailboxes for the extra detectors. We write thresholds
 * into these during startup and then leave them alone.
 */

// Chooses which axes may report a tap.
constexpr uint8_t REG_TAP_CFG0 = 0x56;

// How hard a tap must be, on the X axis.
constexpr uint8_t REG_TAP_CFG1 = 0x57;

// Tap strength again, plus the master switch for all these detectors.
constexpr uint8_t REG_TAP_CFG2 = 0x58;

// Shared mailbox: tap strength on Z, and the tilt angle for 6D.
constexpr uint8_t REG_TAP_THS_6D = 0x59;

// Timing for taps: how long a tap lasts, how long to wait for a second.
constexpr uint8_t REG_INT_DUR2 = 0x5A;

// How much movement counts as "significant motion".
constexpr uint8_t REG_WAKE_UP_THS = 0x5B;

// How long that movement must last to count.
constexpr uint8_t REG_WAKE_UP_DUR = 0x5C;

// How weak gravity must get, and for how long, to call it free fall.
constexpr uint8_t REG_FREE_FALL = 0x5D;

/*
 * FIFO settings mailboxes.
 */

// Low byte of the "tell me when the queue reaches this depth" level.
constexpr uint8_t REG_FIFO_CTRL1 = 0x07;

// High bit of that same level.
constexpr uint8_t REG_FIFO_CTRL2 = 0x08;

// How often accelerometer and gyroscope readings enter the queue.
constexpr uint8_t REG_FIFO_CTRL3 = 0x09;

// Queue mode, plus temperature and timestamp queueing.
constexpr uint8_t REG_FIFO_CTRL4 = 0x0A;

/*
 * Reading here pulls the next item out of the queue.
 *
 * Seven bytes arrive: one label saying what the item is, then six
 * bytes of reading. The label matters because accelerometer,
 * gyroscope, temperature and clock entries all come through this one
 * mailbox mixed together.
 */
constexpr uint8_t REG_FIFO_DATA_OUT_TAG = 0x78;

/*
 * Embedded-bank mailboxes. Only reachable after switching banks.
 */

// Switches on the step counter, tilt and significant-motion features.
constexpr uint8_t REG_EMB_FUNC_EN_A = 0x04;

// First of two mailboxes holding the running step total.
constexpr uint8_t REG_STEP_COUNTER_L = 0x62;

// Used to reset the step count back to zero.
constexpr uint8_t REG_EMB_FUNC_SRC = 0x64;

// Tells a feature to start up after it has been switched on.
constexpr uint8_t REG_EMB_FUNC_INIT_A = 0x66;

/*
 * The labels on queue items.
 *
 * The first byte of each seven-byte item carries one of these, which
 * tells us how to read the six bytes that follow.
 */

// Label 1: the six bytes are gyroscope X, Y, Z.
constexpr uint8_t FIFO_TAG_GYRO = 1;

// Label 2: the six bytes are accelerometer X, Y, Z.
constexpr uint8_t FIFO_TAG_ACCEL = 2;

// Label 3: the bytes are a temperature.
constexpr uint8_t FIFO_TAG_TEMPERATURE = 3;

// Label 4: the bytes are a clock reading.
constexpr uint8_t FIFO_TAG_TIMESTAMP = 4;

/*
 * Settings values written into the FIFO mailboxes above. These are
 * codes from the manual, not plain numbers. Code 4 happens to mean
 * 104 Hz; it does not mean the number four.
 */

// Code meaning "queue accelerometer and gyroscope at 104 Hz".
constexpr uint8_t FIFO_BATCH_104_HZ = 4;

// Mode 0 switches the queue off. We use it briefly to clear the queue.
constexpr uint8_t FIFO_MODE_BYPASS = 0;

// Mode 6 keeps the newest items and drops the oldest when full.
// For flight that is the right choice: stale readings are worthless.
constexpr uint8_t FIFO_MODE_STREAM = 6;

// Only bother emptying the queue once 16 items are waiting.
constexpr uint8_t FIFO_WATERMARK_WORDS = 16;

/*
 * Never read more than 32 items in a single pass.
 *
 * This is a safety rail for the flight loop. A full queue holds 512
 * items, and reading all of them at once would stall everything else.
 * Better to take a bite each pass and keep flying.
 */
constexpr uint16_t FIFO_MAX_DRAIN_PER_PASS = 32;

// Code meaning "queue temperature at 52 Hz".
constexpr uint8_t FIFO_TEMP_BATCH_52_HZ = 3;

// Code meaning "queue every clock reading, skip none".
constexpr uint8_t FIFO_TIMESTAMP_NO_DECIMATION = 1;


// ==================================================
// LIS3MDL MAILBOX NUMBERS
// ==================================================

/*
 * The magnetometer is a much simpler chip. It has no banks and no
 * queue. These four numbers are nearly all of it.
 */

// Settings mailbox. One bit switches on its thermometer.
constexpr uint8_t LIS3MDL_CTRL_REG1 = 0x20;

// "Is fresh data waiting", and "did I overwrite data you never read".
constexpr uint8_t LIS3MDL_STATUS = 0x27;

// First of six mailboxes holding the magnetic reading, X then Y then Z.
constexpr uint8_t LIS3MDL_OUT_X_L = 0x28;

// Low byte of its temperature.
constexpr uint8_t LIS3MDL_TEMP_OUT_L = 0x2E;

// High byte of its temperature.
constexpr uint8_t LIS3MDL_TEMP_OUT_H = 0x2F;

/*
 * Add this to a mailbox number to read several mailboxes in a row.
 *
 * Without it this chip hands back the same first byte six times over,
 * so a reading of X, Y and Z would come out as X, X, X. It is a quirk
 * of the LIS3MDL and the cause of a classic bug.
 */
constexpr uint8_t LIS3MDL_AUTO_INCREMENT = 0x80;


// ==================================================
// TURNING CHIP COUNTS INTO REAL UNITS
// ==================================================

/*
 * Sensors do not report metres or degrees. They report counts, which
 * are plain whole numbers like 8380. Each count stands for a fixed
 * amount, and these constants are those amounts, taken from the
 * manuals for the ranges we chose at startup.
 *
 * LSB means "least significant bit", which is one count.
 */

// At the plus-or-minus 4 g range, one count is 0.122 thousandths of a g.
// "g" is one Earth gravity. The f on the end means it is a float.
constexpr float ACCEL_MG_PER_LSB = 0.122f;

// At the plus-or-minus 500 degrees-per-second range, one count is
// 17.5 thousandths of a degree per second of turning.
constexpr float GYRO_MDPS_PER_LSB = 17.50f;

// At the plus-or-minus 4 gauss range, 6842 counts make one gauss.
// Gauss measures magnetic strength.
constexpr float LIS3MDL_LSB_PER_GAUSS_4G = 6842.0f;

// One gauss is 100 microtesla. Microtesla is the unit we publish,
// because it is what magnetic field strength is normally quoted in.
constexpr float MICROTESLA_PER_GAUSS = 100.0f;


// ==================================================
// TYPES
// ==================================================

/*
 * The shapes we store readings in.
 *
 * Nothing here does any work. These are empty containers, described
 * now so the reading code further down has somewhere to put things.
 *
 * Every field is given a starting value, usually 0 or false. That is
 * deliberate. An unset number in C++ holds whatever rubbish was in
 * that memory before, and rubbish that looks like a real attitude
 * reading is exactly the kind of thing that crashes aircraft.
 */

/*
 * Which face of the board is pointing up.
 *
 * An enum class is a short list of allowed names. A variable of this
 * type can only ever hold one of these seven words, so it is
 * impossible to store a meaningless value in it.
 *
 * Unknown comes first on purpose: it is what you get before the chip
 * has told us anything.
 */
enum class OrientationFace : uint8_t {
  Unknown,     // We have not been told yet.
  XPositive,   // The positive X edge is up.
  XNegative,   // The negative X edge is up.
  YPositive,   // The positive Y edge is up.
  YNegative,   // The negative Y edge is up.
  ZPositive,   // The board is flat, face up.
  ZNegative    // The board is flat, upside down.
};


/*
 * Three numbers with decimal points, for a reading in real units.
 *
 * Used for acceleration in metres per second squared, turning speed
 * in radians per second, and magnetic strength in microtesla.
 */
struct Vector3Float {

  float x = 0;  // Side to side.

  float y = 0;  // Forwards and backwards.

  float z = 0;  // Up and down.

};


/*
 * The same three axes, but as raw chip counts before conversion.
 *
 * int16_t because the counts can be negative: tilt one way and the
 * number goes below zero.
 *
 * We keep the raw counts as well as the converted ones because they
 * are the honest record. If a conversion constant is ever wrong, the
 * raw numbers still tell us what the chip actually said.
 */
struct Vector3Int16 {

  int16_t x = 0;  // Raw count, side to side.

  int16_t y = 0;  // Raw count, forwards and backwards.

  int16_t z = 0;  // Raw count, up and down.

};


/*
 * The finished timing report for one stream of readings.
 *
 * This answers "are readings arriving steadily". A wobbling gap
 * between readings means the loop is struggling, and that shows up
 * here before it shows up as bad flying.
 */
struct SampleTiming {

  bool valid = false;
  // false means not enough readings yet to say anything. The printing
  // code shows "waiting" rather than a misleading zero.

  uint32_t lastIntervalUs = 0;
  // Microseconds between the two most recent readings.

  uint32_t minimumIntervalUs = 0;
  // The shortest gap seen during the last second.

  uint32_t maximumIntervalUs = 0;
  // The longest gap seen. This is the one that exposes a stall.

  float averageIntervalUs = 0;
  // The typical gap. A float because dividing rarely comes out whole.

  uint32_t maximumJitterUs = 0;
  // The worst miss against the gap we expected. Small is good.

};


/*
 * The scratchpad used while building the report above.
 *
 * Kept separate from SampleTiming for one reason: this one is reset
 * every second, while SampleTiming has to stay still long enough to
 * be read and printed. Mixing the two would mean reading numbers
 * while they are being cleared.
 */
struct TimingAccumulator {

  bool havePrevious = false;
  // Have we seen a reading yet? You need two to measure a gap, so the
  // very first reading only records its time and measures nothing.

  uint32_t previousMark = 0;
  // When the previous reading arrived, so we can subtract.

  uint32_t lastIntervalUs = 0;
  // The gap we just worked out.

  uint32_t minimumIntervalUs = 0;
  // Smallest gap so far this second.

  uint32_t maximumIntervalUs = 0;
  // Largest gap so far this second.

  uint32_t sumIntervalUs = 0;
  // All the gaps added together, so we can work out the average.

  uint32_t intervalCount = 0;
  // How many gaps went into that sum. Sum divided by count is the average.

  uint32_t maximumJitterUs = 0;
  // Worst distance from the expected gap, so far this second.

};


/*
 * Things that happened, as opposed to things that are measured.
 *
 * An important detail about these flags: once set true they stay true
 * until motorController calls clearFlightSensorEvents(). They are a
 * message that something occurred since it last looked, not a
 * description of right now.
 *
 * That is why clearing them matters. Without it, one free-fall event
 * at power-on would read as "still falling" for the whole flight.
 */
struct FlightSensorEvents {

  bool singleTap = false;
  // Someone or something tapped the board once.

  bool doubleTap = false;
  // Two taps in quick succession.

  bool stepDetected = false;
  // The chip's pedometer counted a footstep. A leftover from the
  // chip's phone-and-fitness origins, not useful to a drone.

  bool tilt = false;
  // The board tilted past the angle we configured.

  bool freeFall = false;
  // Gravity nearly vanished, which means the board is falling.
  // This one is a genuine safety signal and motorController reads it.

  bool significantMotion = false;
  // Real movement, not just vibration.

  bool orientationChanged = false;
  // A different face is now pointing up.

  OrientationFace orientation = OrientationFace::Unknown;
  // Which face that is.

  uint16_t stepCount = 0;
  // The running step total held by the chip itself.

};


/*
 * What we found in the chip's waiting room.
 *
 * Remember the queue is a backup and a warning light, not our main
 * source. The live readings are what flies the drone. unreadWords is
 * the number worth watching: if it climbs, we are too slow.
 */
struct FlightSensorFifo {

  bool enabled = false;
  // Did the queue set up correctly at startup?

  uint16_t unreadWords = 0;
  // How many items are waiting. Near 512 means the queue is full and
  // readings are being thrown away. Single digits is healthy.

  bool watermark = false;
  // True once the queue reaches the depth we said to tell us about.

  bool overrun = false;
  // True if the queue filled and the chip had to discard readings.

  bool haveAccelerometer = false;
  // Have we ever pulled an accelerometer item out of the queue?

  Vector3Float accelerometerMps2;
  // The newest accelerometer reading that came from the queue.

  bool haveGyroscope = false;
  // Have we ever pulled a gyroscope item out?

  Vector3Float gyroscopeRadPerSec;
  // The newest gyroscope reading from the queue.

  bool haveTemperature = false;
  // Have we ever pulled a temperature item out?

  float temperatureC = 0;
  // The newest temperature from the queue, in degrees Celsius.

  bool haveTimestamp = false;
  // Have we ever pulled a clock item out?

  uint32_t timestampTicks = 0;
  // That clock reading, in 25 microsecond ticks.

  uint64_t timestampUs = 0;
  // The same reading converted to microseconds. uint64_t because
  // ticks times 25 grows large enough to overflow a smaller type.

};


/*
 * Tally marks for everything that can go wrong.
 *
 * One counter per kind of failure, and nothing here ever stops the
 * program. A chip that fails is retried, and the attempt is counted.
 *
 * Separate counters rather than one total because the name of the
 * counter that is climbing tells you where to look. We found the
 * magnetometer rate problem exactly this way: magnetometerOverrunCount
 * was the only one moving.
 */
struct FlightSensorErrors {

  uint32_t userBankSelect = 0;
  // Could not switch back to the everyday bank. Serious, because it
  // means later readings would come from the wrong mailboxes.

  uint32_t imuBegin = 0;
  // The accelerometer and gyroscope chip would not start.

  uint32_t imuWhoAmI = 0;
  // That chip gave the wrong identity number.

  uint32_t ifInc = 0;
  // Could not switch on reading several mailboxes in a row.

  uint32_t imuStatusRead = 0;
  // Could not ask whether fresh data was waiting.

  uint32_t imuSampleRead = 0;
  // Could not fetch the fourteen bytes of readings.

  uint32_t imuTimestampRead = 0;
  // Could not read the chip's clock.

  uint32_t magBegin = 0;
  // The magnetometer would not start.

  uint32_t magWhoAmI = 0;
  // The magnetometer gave the wrong identity number.

  uint32_t magnetometerStatusRead = 0;
  // Could not ask the magnetometer whether data was ready.

  uint32_t magnetometerSampleRead = 0;
  // Could not fetch the magnetic reading.

  uint32_t fifoStatusRead = 0;
  // Could not ask how full the queue was.

  uint32_t fifoWordRead = 0;
  // Could not pull an item out of the queue.

  uint32_t tapRead = 0;
  // Could not read the tap report.

  uint32_t freeFallRead = 0;
  // Could not read the free-fall report.

  uint32_t orientationRead = 0;
  // Could not read which face is up.

  uint32_t embeddedStatusRead = 0;
  // Could not read the motion-feature report.

  uint32_t stepCountRead = 0;
  // Could not read the step total.

  uint32_t magnetometerOverrunCount = 0;
  // The magnetometer overwrote a reading before we collected it.
  // Not a broken wire. It means the chip is faster than our loop,
  // which is why its rate was lowered from 155 Hz to 80 Hz.

  uint32_t fifoOverrunCount = 0;
  // The queue filled up and readings were lost.

};


/*
 * The short answer to "can we trust these sensors right now".
 *
 * This is the part motorController actually reads. It does not want
 * raw bytes, it wants a yes or no on each capability.
 *
 * Note the difference between two words used below:
 *
 *   Valid  the chip started up and answered. It exists.
 *   Fresh  a reading arrived recently. It is still talking.
 *
 * Valid without Fresh is the dangerous case: a chip that woke up at
 * power-on and has since gone quiet. Checking only Valid would let us
 * fly on a frozen reading, so the health flags that matter require
 * Fresh too.
 */
struct FlightSensorHealth {

  bool initialized = false;
  // At least one of the two chips came up at some point.

  bool imuIdentityValid = false;
  // The accelerometer and gyroscope chip proved who it is.

  bool magnetometerIdentityValid = false;
  // The magnetometer proved who it is.

  bool hardwareTimestampEnabled = false;
  // The chip's own clock is running.

  bool accelerometerValid = false;
  // The accelerometer started up.

  bool gyroscopeValid = false;
  // The gyroscope started up. Same chip, so it moves with the above.

  bool magnetometerValid = false;
  // The magnetometer started up.

  bool accelerometerFresh = false;
  // An accelerometer reading arrived within the last 50 ms.

  bool gyroscopeFresh = false;
  // A gyroscope reading arrived within the last 50 ms.

  bool magnetometerFresh = false;
  // A magnetic reading arrived within the last 100 ms.

  bool attitudeHealthy = false;
  // Accelerometer AND gyroscope are both fresh.
  // This is the one that decides whether the drone may fly at all.

  bool headingHealthy = false;
  // The magnetometer is fresh, so compass heading can be trusted.

  bool allSensorsHealthy = false;
  // Everything above is good. Nice to have, not required for flight.

};


/*
 * EVERYTHING, in one box.
 *
 * This is the whole published result of this file. One of these
 * exists, called flightSensorData, and getFlightSensorData() is how
 * the rest of the program looks at it.
 *
 * It gathers the smaller structs above and adds the readings
 * themselves. Think of it as the clipboard this file keeps filling in.
 */
struct FlightSensorData {

  uint32_t sequence = 0;
  // Counts up by one every time anything new is stored. A reader can
  // compare this against last time to ask "is there anything new".

  uint64_t timestampUs = 0;
  // Our own processor's time, in microseconds, at the start of this
  // pass. Keeps counting up and never goes backwards.

  /*
   * The accelerometer. It senses which way gravity pulls, which is
   * how the drone knows which way is down.
   */

  Vector3Int16 rawAccelerometer;
  // Straight chip counts, unconverted.

  Vector3Float accelerationMps2;
  // The same reading in metres per second squared. Sitting still and
  // flat, Z reads about 9.81, which is gravity.

  uint32_t accelerometerLastUpdateUs = 0;
  // When the newest reading arrived. Freshness is measured from here.

  uint32_t accelerometerSampleCount = 0;
  // Total readings ever collected. Used to work out the rate.

  float accelerometerRateHz = 0;
  // Measured readings per second. Should sit near 104.

  SampleTiming accelerometerTiming;
  // The gap-between-readings report for this stream.

  /*
   * The gyroscope. It senses turning speed, which is how the drone
   * knows it is starting to roll before it has rolled far.
   */

  Vector3Int16 rawGyroscope;
  // Straight chip counts.

  Vector3Float gyroscopeRadPerSec;
  // Turning speed in radians per second. Still means near zero.

  uint32_t gyroscopeLastUpdateUs = 0;
  // When the newest turning reading arrived.

  uint32_t gyroscopeSampleCount = 0;
  // Total turning readings collected.

  float gyroscopeRateHz = 0;
  // Measured rate, also near 104.

  SampleTiming gyroscopeTiming;
  // Gap report for turning readings.

  /*
   * The magnetometer. It senses Earth's magnetic field, which gives a
   * compass heading. Needed because the gyroscope alone slowly drifts.
   */

  Vector3Int16 rawMagnetometer;
  // Straight chip counts.

  Vector3Float magneticFieldMicrotesla;
  // Magnetic strength per axis, in microtesla.

  uint32_t magnetometerLastUpdateUs = 0;
  // When the newest magnetic reading arrived.

  uint32_t magnetometerSampleCount = 0;
  // Total magnetic readings collected.

  float magnetometerRateHz = 0;
  // Measured rate, near 80 since we lowered it.

  SampleTiming magnetometerTiming;
  // Gap report for magnetic readings.

  // Accel and gyro share this one 25 us clock.
  SampleTiming chipTimestampTiming;
  // Gap report measured by the chip's own clock instead of ours. More
  // trustworthy, because it is not affected by how busy we are.

  uint32_t imuTimestampTicks = 0;
  // The chip's clock reading, in 25 microsecond ticks.

  uint64_t imuTimestampUs = 0;
  // The same, in microseconds.

  bool haveImuTimestamp = false;
  // Have we successfully read that clock at least once?

  uint32_t imuPollCycles = 0;
  // How many times we have fetched a full reading block.

  float imuTemperatureC = 0;
  // Temperature inside the accelerometer chip, in Celsius. Matters
  // because these sensors drift as they warm up.

  float magnetometerTemperatureC = 0;
  // Temperature inside the magnetometer.

  bool haveMagnetometerTemperature = false;
  // Did its thermometer switch on successfully?

  uint8_t imuStatusRegister = 0;
  // The raw "data ready" byte, kept for troubleshooting.

  bool accelerometerDataReady = false;
  // Was fresh accelerometer data waiting on the last check?

  bool gyroscopeDataReady = false;
  // Was fresh gyroscope data waiting?

  bool imuTemperatureDataReady = false;
  // Was fresh temperature data waiting?

  bool magnetometerDataReady = false;
  // Was fresh magnetic data waiting?

  uint32_t lastImuReadUs = 0;
  // When we last finished reading the accelerometer chip.

  uint32_t lastMagReadUs = 0;
  // When we last finished reading the magnetometer.

  uint8_t imuWhoAmI = 0;
  // The identity byte the accelerometer chip gave us. Should be 0x6C.

  uint8_t magnetometerWhoAmI = 0;
  // The identity byte from the magnetometer. Should be 0x3D.

  FlightSensorEvents events;
  // Taps, tilt, free fall, steps.

  FlightSensorFifo fifo;
  // The waiting-room report.

  FlightSensorHealth health;
  // The trust summary. This is what motorController reads.

  FlightSensorErrors errors;
  // The tally of everything that has gone wrong.

};


// ==================================================
// IMU
// ==================================================

/*
 * Our own version of the Adafruit chip driver, with extra abilities.
 *
 * "public Adafruit_LSM6DSOX" means this starts as a copy of their
 * driver and keeps everything it can already do. We are only adding.
 *
 * WHY WE NEED TO ADD ANYTHING:
 * Their driver covers the ordinary readings, but not the step counter,
 * the taps, the free-fall detector or the queue. Reaching those means
 * writing to individual mailboxes ourselves, and the plumbing for that
 * is hidden inside their class. Inheriting is how we get at it.
 *
 * Every function here answers true for success and false for failure.
 * Nothing throws errors or stops the program.
 */
class FlightLsm6dsox : public Adafruit_LSM6DSOX {

 public:
  /*
   * Turn a raw temperature count into degrees Celsius.
   *
   * The chip reports 0 at 25 degrees and counts up or down from there,
   * so we scale and then add 25 back on. temperature_sensitivity comes
   * from the Adafruit driver we inherited.
   */
  float temperatureCFromRaw(int16_t raw) {

    return (raw / temperature_sensitivity) + 25.0f;

  }

  /*
   * Change just a few bits inside one mailbox, then check it worked.
   *
   * Mailboxes hold eight bits and often pack several unrelated
   * settings together. Writing the whole byte would wipe the settings
   * we did not mean to touch, so we change only our few bits.
   *
   *   address   which mailbox
   *   bitCount  how many bits belong to our setting
   *   shift     how far along the byte those bits sit
   *   value     what to put in them
   *   verify    read it back afterwards to confirm. Default true.
   *
   * verify exists because some mailboxes clear themselves the instant
   * they are written. Reading those back always disagrees, so callers
   * pass false for them rather than counting a false failure.
   */
  bool writeField(uint8_t address, uint8_t bitCount, uint8_t shift, uint8_t value, bool verify = true) {

    // Point the library at the mailbox we want.
    Adafruit_BusIO_Register reg(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, address);

    // Narrow that down to just our few bits inside it.
    Adafruit_BusIO_RegisterBits field(&reg, bitCount, shift);

    // Try the write. Give up straight away if the wire failed.
    if (!field.write(value)) {

      return false;

    }

    // Caller said not to check, so take the write at its word.
    if (!verify) {

      return true;

    }

    // Read it back. Equal means the setting really stuck.
    return field.read() == value;

  }

  /*
   * Write a whole mailbox at once, then check it.
   *
   * Used where every bit in the byte is ours to set, so there is
   * nothing to preserve.
   */
  bool writeRegister(uint8_t address, uint8_t value) {

    Adafruit_BusIO_Register reg(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, address);

    if (!reg.write(value)) {

      return false;

    }

    // Somewhere to put what we read back.
    uint8_t got = 0;

    if (!reg.read(&got)) {

      return false;

    }

    // Did the chip keep what we gave it?
    return got == value;

  }

  /*
   * Read one mailbox.
   *
   * The answer is handed back through value, which is a pointer,
   * meaning "the address of a number to fill in". The returned
   * true or false says whether the reading itself succeeded.
   *
   * Two separate things, which is the point: a reading of zero is
   * perfectly valid, so success cannot be signalled by the value.
   */
  bool readRegister(uint8_t address, uint8_t *value) {

    Adafruit_BusIO_Register reg(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, address);

    return reg.read(value);

  }

  /*
   * Read several mailboxes in a row in one go.
   *
   * This is how the fourteen bytes of readings arrive together. All of
   * them come from the same instant, which is what makes the attitude
   * maths honest.
   *
   *   buffer  where to put the bytes
   *   length  how many to fetch
   */
  bool readRegisters(uint8_t address, uint8_t *buffer, uint8_t length) {

    Adafruit_BusIO_Register reg(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, address, length);

    return reg.read(buffer, length);

  }

  /*
   * Switch which set of mailboxes is showing.
   *
   * The two bits that choose the bank live at the top of the byte, so
   * bitCount is 2 and shift is 6.
   *
   * "bank & 0x03" keeps only the lowest two bits of whatever was
   * passed in. It is a guard: a silly value cannot spill over and
   * disturb the other settings in that mailbox.
   */
  bool selectBank(uint8_t bank) {

    return writeField(REG_FUNC_CFG_ACCESS, 2, 6, bank & 0x03);

  }

};


// ==================================================
// THE MODULE'S OWN MEMORY
// ==================================================

/*
 * These live for as long as the program runs, so they remember things
 * between one pass of the flight loop and the next.
 *
 * They belong to this file alone. motorController cannot touch them,
 * and that is on purpose. It reads the finished data through the
 * functions near the bottom instead.
 */

/*
 * The chips themselves, as objects we can command.
 */

// The accelerometer and gyroscope, using our extended class.
FlightLsm6dsox imu;

// The magnetometer, using Adafruit's driver as it comes.
Adafruit_LIS3MDL magnetometer;

/*
 * A second, direct line to the magnetometer.
 *
 * Adafruit's magnetometer driver does not expose single mailboxes, and
 * we need that for the status byte, the auto-increment trick and the
 * thermometer. So we keep this alongside it.
 *
 * Wire1 is the Qwiic connector on the UNO Q. The & means "the real
 * one", because there is only one Qwiic bus and all the chips share it.
 */
Adafruit_I2CDevice magRegs(MAG_ADDRESS, &Wire1);

/*
 * The clipboard. Everything this file learns ends up here, and this is
 * the one copy that getFlightSensorData() hands out.
 */
FlightSensorData flightSensorData;

/*
 * Startup progress for the accelerometer and gyroscope chip.
 *
 * Waking a chip takes several steps that can each fail on their own.
 * Each flag remembers one step that is already done, so a retry picks
 * up where it left off instead of starting over.
 */

// Has the chip answered at all?
bool imuBeginOk = false;

// Has it proved its identity?
bool imuIdentityOk = false;

// Have we switched on reading several mailboxes in a row?
bool imuIfIncOk = false;

// Have ranges, rates and features been set?
bool imuConfigured = false;

// All of the above done. Safe to read from.
bool imuReady = false;

/*
 * The same startup progress for the magnetometer.
 */

// Has Adafruit's driver started it?
bool magBeginOk = false;

// Has our direct mailbox line started?
bool magRegsOk = false;

// Has it proved its identity?
bool magIdentityOk = false;

// Have its mode, rate and range been set?
bool magConfigured = false;

// All done. Safe to read from.
bool magReady = false;

// Is the chip's own 25 microsecond clock running?
bool hardwareTimestampOn = false;

/*
 * Which optional detectors actually switched on.
 *
 * Not all of them always do, and a feature that failed must not be
 * read. Reading a detector that was never enabled returns stale
 * rubbish that looks like a real event, so every read is guarded by
 * the matching flag here.
 */

// Tap detection is working.
bool featureTap = false;

// The step counter is working.
bool featureSteps = false;

// Tilt detection is working.
bool featureTilt = false;

// Free-fall detection is working. This is the safety-relevant one.
bool featureFreeFall = false;

// Which-face-is-up detection is working.
bool featureOrientation = false;

// Significant-motion detection is working.
bool featureSignificantMotion = false;

// The queue is working.
bool featureFifo = false;

// The magnetometer's thermometer is working.
bool featureMagnetometerTemperature = false;

/*
 * Remembering whether we already counted an overflow.
 *
 * An overflow flag stays raised in the chip until the cause clears, so
 * it would be seen on every pass. Without these we would count one
 * problem hundreds of times a second. We only count the moment it
 * first appears.
 */

// Was the magnetometer overflowing last time we looked?
bool sawMagnetometerOverrun = false;

// Was the queue overflowing last time we looked?
bool sawFifoOverrun = false;

/*
 * When to try a sleeping chip again, so retries do not hog the loop.
 */

// Earliest time we may retry the accelerometer chip, in milliseconds.
uint32_t nextImuRetryMs = 0;

// Earliest time we may retry the magnetometer.
uint32_t nextMagRetryMs = 0;

// Earliest time we may read the step counter again.
uint32_t nextStepReadMs = 0;

/*
 * Working out the real readings-per-second.
 *
 * The method is to note the counters, wait a second, note them again,
 * and divide the difference by the time. These remember the start.
 */

// When this one-second window began.
uint32_t rateWindowStartMs = 0;

// The accelerometer count when it began.
uint32_t rateWindowAccelStart = 0;

// The gyroscope count when it began.
uint32_t rateWindowGyroStart = 0;

// The magnetometer count when it began.
uint32_t rateWindowMagStart = 0;

// Is a window running? False only before the very first one.
bool rateWindowOpen = false;

/*
 * The scratchpads for the gap-between-readings maths, one per stream.
 */

// Gaps between accelerometer readings.
TimingAccumulator accelerometerAccumulator;

// Gaps between gyroscope readings.
TimingAccumulator gyroscopeAccumulator;

// Gaps between magnetic readings.
TimingAccumulator magnetometerAccumulator;

// Gaps measured by the chip's own clock rather than ours.
TimingAccumulator chipTimestampAccumulator;

/*
 * Building a clock that never goes backwards.
 *
 * The Arduino micros() counter runs out of room after about 71 minutes
 * and wraps back to zero. A flight log whose time suddenly jumps
 * backwards is useless, so monotonicMicroseconds() adds up the small
 * steps instead of trusting the raw number. These three remember where
 * it had got to.
 */

// What micros() said last time we looked.
uint32_t lastMicrosReading = 0;

// Our running total, which keeps climbing past the wrap.
uint64_t monotonicUs = 0;

// Have we looked at least once? The first look has nothing to compare.
bool haveMonotonicUs = false;


// ==================================================
// SMALL HELPERS
// ==================================================

/*
 * Add one to an error tally.
 *
 * The & means the real counter is handed in, not a copy, so adding one
 * here changes the original. Without the & we would increase a copy
 * and then throw it away.
 *
 * A one-line function earns its place by giving every failure in this
 * file the same obvious shape: countError(whichever one).
 */
void countError(uint32_t &counter) {

  counter++;

}


/*
 * Join two bytes into one number that may be negative.
 *
 * Sensors send a reading as two separate bytes because one byte only
 * reaches 255, nowhere near enough. The low byte carries the small
 * part, the high byte the large part.
 *
 * "<< 8" shifts the high byte up by eight places, which multiplies it
 * by 256. The | then drops the low byte into the space underneath.
 * So 0x7F and 0x20 become 0x207F.
 *
 * We assemble as unsigned, then convert to int16_t at the end so the
 * top bit correctly means "negative" rather than "very large".
 */
int16_t bytesToInt16(uint8_t lowByte, uint8_t highByte) {

  return (int16_t)((uint16_t)lowByte | ((uint16_t)highByte << 8));

}


/*
 * Join four bytes into one large positive number.
 *
 * Same idea, scaled up. Used for the chip's clock, which needs all
 * four bytes. The shifts are 0, 8, 16 and 24 places, each one eight
 * further along than the last.
 *
 * const in front of the pointer promises we will only read the bytes,
 * never alter the caller's array.
 */
uint32_t bytesToUint32(const uint8_t *bytes) {

  return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) | ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);

}


/*
 * Chip counts into metres per second squared.
 *
 * Three steps in one line:
 *   times ACCEL_MG_PER_LSB        counts become thousandths of a g
 *   times SENSORS_GRAVITY_STANDARD g becomes metres per second squared
 *   divided by 1000               undoes the "thousandths"
 *
 * SENSORS_GRAVITY_STANDARD is 9.80665, from the Adafruit library.
 *
 * A worked example: a raw count of 8380 lying flat gives
 * 8380 x 0.122 x 9.80665 / 1000, which is about 10.0. Close to
 * gravity, which is exactly what a level board should read.
 */
float accelToMps2(int16_t raw) {

  return raw * ACCEL_MG_PER_LSB * SENSORS_GRAVITY_STANDARD / 1000.0f;

}


/*
 * Chip counts into radians per second of turning.
 *
 * Same shape. SENSORS_DPS_TO_RADS converts degrees to radians, which
 * is the unit the later attitude maths will want.
 *
 * A radian is about 57.3 degrees, so these numbers look small. Sitting
 * still they should hover near zero.
 */
float gyroToRadPerSec(int16_t raw) {

  return raw * GYRO_MDPS_PER_LSB * SENSORS_DPS_TO_RADS / 1000.0f;

}


/*
 * Chip counts into microtesla of magnetic strength.
 *
 * Multiply by 100 to turn gauss into microtesla, divide by 6842
 * because that many counts make a gauss.
 *
 * Earth's field is roughly 25 to 65 microtesla depending where you
 * stand, so readings in the tens are normal.
 */
float magnetometerToMicrotesla(int16_t raw) {

  return raw * MICROTESLA_PER_GAUSS / LIS3MDL_LSB_PER_GAUSS_4G;

}


/*
 * A clock that never goes backwards.
 *
 * THE PROBLEM:
 * micros() counts microseconds since power-on, but it only has room
 * for about 71 minutes. Then it wraps to zero. Anything comparing
 * timestamps across that moment sees time run backwards.
 *
 * THE FIX:
 * Never use the raw number. Each visit, work out how much it moved
 * since last time and add that to our own larger total.
 *
 * WHY SUBTRACTING STILL WORKS AT THE WRAP:
 * now - lastMicrosReading is done in uint32_t arithmetic, which wraps
 * the same way the counter does. If last was near the maximum and now
 * is 5, the subtraction still yields the true small gap. This is a
 * standard trick, and it is why the cast is written out explicitly.
 */
uint64_t monotonicMicroseconds() {

  // What the hardware counter says right now.
  uint32_t now = micros();

  // Skip this on the very first call, when there is nothing to compare.
  if (haveMonotonicUs) {

    // Add only the small step, so the wrap never reaches our total.
    monotonicUs += (uint32_t)(now - lastMicrosReading);

  }

  // Remember this reading for next time.
  lastMicrosReading = now;

  haveMonotonicUs = true;

  return monotonicUs;

}


/*
 * Read one mailbox from the magnetometer.
 *
 * The same job as the IMU's readRegister, but through our direct line
 * because Adafruit's magnetometer driver does not offer this.
 */
bool readMagnetometerRegister(uint8_t address, uint8_t *value) {

  Adafruit_BusIO_Register reg(&magRegs, address);

  return reg.read(value);

}


/*
 * Read several magnetometer mailboxes in a row.
 *
 * Note the | LIS3MDL_AUTO_INCREMENT. That sets the top bit of the
 * address, which is this chip's way of being told "move along as you
 * read". Leave it out and the chip hands back its first byte over and
 * over, so X, Y and Z all come out as X.
 */
bool readMagnetometerRegisters(uint8_t address, uint8_t *buffer, uint8_t length) {

  // A multi-byte LIS3MDL read repeats the first byte unless bit 7 is set.
  Adafruit_BusIO_Register reg(&magRegs, address | LIS3MDL_AUTO_INCREMENT, length);

  return reg.read(buffer, length);

}


// ==================================================
// MEASURING HOW STEADILY READINGS ARRIVE
// ==================================================

/*
 * File one measured gap into a scratchpad.
 *
 * Called once per reading. It keeps the newest gap, the smallest, the
 * largest, a running total for the average, and the worst miss against
 * what we expected.
 *
 *   accumulator  the scratchpad to update, handed in by reference
 *   intervalUs   the gap we just measured, in microseconds
 *   expectedUs   the gap we were hoping for
 *
 * Why bother: a steady gap means the sensor and the loop are both
 * keeping up. A gap that sometimes stretches is the early warning that
 * something is stealing time, which is how the Serial stall was found.
 */
void recordTiming(TimingAccumulator &accumulator, uint32_t intervalUs, uint32_t expectedUs) {

  // Always remember the most recent gap.
  accumulator.lastIntervalUs = intervalUs;

  /*
   * Smallest gap so far.
   *
   * The intervalCount == 0 test matters: on the very first gap the
   * stored minimum is 0, and nothing is ever smaller than 0, so
   * without this the minimum would stay stuck at 0 forever.
   */
  if (accumulator.intervalCount == 0 || intervalUs < accumulator.minimumIntervalUs) {

    accumulator.minimumIntervalUs = intervalUs;

  }

  // Largest gap so far. Starting at 0 is fine here, since any real gap beats it.
  if (intervalUs > accumulator.maximumIntervalUs) {

    accumulator.maximumIntervalUs = intervalUs;

  }

  /*
   * Add this gap to the running total, but refuse to overflow.
   *
   * 0xFFFFFFFF is the largest number a uint32_t holds, about 4.29
   * billion. If adding would pass it, the number wraps to a tiny value
   * and the average becomes nonsense.
   *
   * So we ask first: is the remaining headroom smaller than the gap we
   * want to add? If so, park at the maximum. An average that reads
   * "impossibly large" is honest; one that silently wrapped is a lie.
   */
  if (accumulator.sumIntervalUs > (0xFFFFFFFFUL - intervalUs)) {

    accumulator.sumIntervalUs = 0xFFFFFFFFUL;

  } else {

    accumulator.sumIntervalUs += intervalUs;

  }

  // One more gap went into the total.
  accumulator.intervalCount++;

  /*
   * How far off the expected gap were we?
   *
   * This is distance, so it must never be negative. The ? : is a
   * compact if-else: if the gap was longer than expected subtract one
   * way, otherwise subtract the other. Both give a positive answer.
   *
   * Plain intervalUs - expectedUs would be wrong here, because these
   * are unsigned and a negative result would wrap to a huge number.
   */
  uint32_t difference = (intervalUs > expectedUs) ? (intervalUs - expectedUs) : (expectedUs - intervalUs);

  // Keep only the worst miss. That is what jitter means.
  if (difference > accumulator.maximumJitterUs) {

    accumulator.maximumJitterUs = difference;

  }

}


/*
 * A reading just arrived. Work out the gap since the last one.
 *
 *   completedUs  our processor's time when the read finished
 *   expectedUs   the gap we were hoping for
 *
 * We time the moment the read finished rather than when it started,
 * because that is the instant the numbers became ours.
 *
 * The first ever reading measures nothing. You need two arrivals to
 * have a gap between them, so the first just notes its time.
 */
// Delivery intervals are MCU time at the completed read.
// uint32 subtraction stays correct when micros() wraps.
void noteDelivery(TimingAccumulator &accumulator, uint32_t completedUs, uint32_t expectedUs) {

  // Only measure if there is an earlier arrival to measure from.
  if (accumulator.havePrevious) {

    // now minus then is the gap. Safe across the counter wrap.
    recordTiming(accumulator, completedUs - accumulator.previousMark, expectedUs);

  }

  // This arrival becomes the mark the next one measures from.
  accumulator.previousMark = completedUs;

  accumulator.havePrevious = true;

}


/*
 * The same gap measurement, but using the chip's own clock.
 *
 * WHY HAVE BOTH:
 * The version above measures when WE got round to reading. If our loop
 * hesitates, that shows up as a sensor problem when really it is ours.
 * The chip's clock ticks regardless of how busy we are, so it reports
 * what the sensor truly did. Comparing the two separates "the sensor
 * is struggling" from "we are struggling".
 *
 * The accelerometer and gyroscope share this single clock. There is
 * not one each.
 */
// Accel and gyro share this one 25 us clock. They do not have separate hardware clocks.
// uint32 subtraction stays correct when the chip timestamp wraps.
void noteChipTimestamp(uint32_t ticks) {

  if (chipTimestampAccumulator.havePrevious) {

    // How many ticks passed. Safe across the chip's own wrap.
    uint32_t deltaTicks = ticks - chipTimestampAccumulator.previousMark;

    /*
     * Ticks into microseconds, 25 each.
     *
     * Worked out as uint64_t because ticks times 25 can exceed what a
     * uint32_t holds. Multiplying first and worrying later is how
     * overflow bugs happen, so the large type is used up front.
     */
    uint64_t deltaUs64 = (uint64_t)deltaTicks * TIMESTAMP_TICK_US;

    // Squeeze back into the smaller type, capping rather than wrapping.
    uint32_t deltaUs = (deltaUs64 > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)deltaUs64;

    recordTiming(chipTimestampAccumulator, deltaUs, ACCEL_EXPECTED_PERIOD_US);

  }

  chipTimestampAccumulator.previousMark = ticks;

  chipTimestampAccumulator.havePrevious = true;

}


/*
 * Copy a finished second's worth of timing into the published report,
 * then clear the scratchpad for the next second.
 *
 *   accumulator  the scratchpad being emptied
 *   timing       the published report being filled
 *
 * Two separate structures exist precisely so this hand-off can happen
 * cleanly: readers see a complete, still report while the scratchpad
 * starts over.
 */
void publishTiming(TimingAccumulator &accumulator, SampleTiming &timing) {

  /*
   * No gaps measured at all this second.
   *
   * Publish zeros and mark the report not valid, so the printing code
   * says "waiting" instead of showing a convincing-looking 0 us.
   */
  if (accumulator.intervalCount == 0) {

    timing.valid = false;

    timing.lastIntervalUs = 0;

    timing.minimumIntervalUs = 0;

    timing.maximumIntervalUs = 0;

    timing.averageIntervalUs = 0.0f;

    timing.maximumJitterUs = 0;

    return;

  }

  // We have real numbers, so the report can be trusted.
  timing.valid = true;

  timing.lastIntervalUs = accumulator.lastIntervalUs;

  timing.minimumIntervalUs = accumulator.minimumIntervalUs;

  timing.maximumIntervalUs = accumulator.maximumIntervalUs;

  /*
   * The average: total divided by how many.
   *
   * Both are turned into float first. Dividing two whole numbers in
   * C++ throws away the remainder, so 9615 / 104 would come out as 92
   * rather than 92.45. Converting first keeps the decimals.
   */
  timing.averageIntervalUs = (float)accumulator.sumIntervalUs / (float)accumulator.intervalCount;

  timing.maximumJitterUs = accumulator.maximumJitterUs;

  /*
   * Now wipe the scratchpad so the next second starts clean.
   *
   * Notice what is NOT wiped: previousMark and havePrevious. Those
   * hold when the last reading arrived. Clearing them would throw away
   * one gap at every second boundary, so they deliberately survive.
   */
  // previousMark and havePrevious are preserved so interval continuity carries across reporting windows.
  accumulator.lastIntervalUs = 0;

  accumulator.minimumIntervalUs = 0;

  accumulator.maximumIntervalUs = 0;

  accumulator.sumIntervalUs = 0;

  accumulator.intervalCount = 0;

  accumulator.maximumJitterUs = 0;

}


/*
 * Once a second, work out the real readings-per-second.
 *
 * Called on every pass, but does almost nothing most of the time. It
 * only acts when a full second has gone by.
 *
 * THE METHOD:
 * Note each counter. Wait a second. Note them again. The difference,
 * divided by how long it really took, is the rate.
 *
 * We divide by the measured time rather than assuming exactly 1000 ms,
 * because the pass that notices the second has elapsed is always a
 * little late. Using the real elapsed time keeps the answer honest.
 */
void updateSampleRateDiagnostics() {

  // Milliseconds since power-on.
  uint32_t now = millis();

  /*
   * First ever call. There is no window to close yet, so open one:
   * note the time and the counters, and come back in a second.
   */
  if (!rateWindowOpen) {

    rateWindowOpen = true;

    rateWindowStartMs = now;

    rateWindowAccelStart = flightSensorData.accelerometerSampleCount;

    rateWindowGyroStart = flightSensorData.gyroscopeSampleCount;

    rateWindowMagStart = flightSensorData.magnetometerSampleCount;

    return;

  }

  // How long this window has actually been running.
  uint32_t elapsedMs = now - rateWindowStartMs;

  // Not a second yet, so leave everything alone and come back later.
  if (elapsedMs < RATE_WINDOW_MS) {

    return;

  }

  /*
   * A second has passed. Readings divided by seconds gives the rate.
   *
   * The times 1000.0f is because elapsedMs is in milliseconds and we
   * want an answer per second. Writing it as 1000.0f rather than 1000
   * forces float maths, which keeps the decimal part. In whole-number
   * maths 103 readings in 1000 ms would come out as 0.
   */
  flightSensorData.accelerometerRateHz = ((flightSensorData.accelerometerSampleCount - rateWindowAccelStart) * 1000.0f) / elapsedMs;

  flightSensorData.gyroscopeRateHz = ((flightSensorData.gyroscopeSampleCount - rateWindowGyroStart) * 1000.0f) / elapsedMs;

  flightSensorData.magnetometerRateHz = ((flightSensorData.magnetometerSampleCount - rateWindowMagStart) * 1000.0f) / elapsedMs;

  // Start the next window from here, with fresh starting counts.
  rateWindowStartMs = now;

  rateWindowAccelStart = flightSensorData.accelerometerSampleCount;

  rateWindowGyroStart = flightSensorData.gyroscopeSampleCount;

  rateWindowMagStart = flightSensorData.magnetometerSampleCount;

  /*
   * Same moment, same job for the gap reports: publish the finished
   * second and clear the scratchpads. Keeping both on one schedule
   * means the rate and the gaps in a log line describe the same second.
   */
  publishTiming(accelerometerAccumulator, flightSensorData.accelerometerTiming);

  publishTiming(gyroscopeAccumulator, flightSensorData.gyroscopeTiming);

  publishTiming(magnetometerAccumulator, flightSensorData.magnetometerTiming);

  publishTiming(chipTimestampAccumulator, flightSensorData.chipTimestampTiming);

}


// ==================================================
// DECIDING WHETHER THE SENSORS CAN BE TRUSTED
// ==================================================

/*
 * Is this stream's newest reading recent enough to use?
 *
 *   lastUpdateUs  when the newest reading arrived
 *   sampleCount   how many readings have ever arrived
 *   limitUs       how old is too old
 *
 * The sampleCount check is the important one. With no readings ever,
 * lastUpdateUs is still 0, and 0 can look convincingly recent just
 * after power-on. Without this test a chip that never once answered
 * could be reported as fresh. So no readings means not fresh, full stop.
 */
bool sampleIsFresh(uint32_t lastUpdateUs, uint32_t sampleCount, uint32_t limitUs) {

  if (sampleCount == 0) {

    return false;

  }

  // Age is now minus then. Safe across the micros() wrap.
  uint32_t ageUs = (uint32_t)micros() - lastUpdateUs;

  return ageUs <= limitUs;

}


/*
 * Fill in the trust summary.
 *
 * Runs at the end of every pass. It turns the scattered flags and
 * timestamps into the short yes-or-no answers motorController reads.
 *
 * The chain of reasoning, built from the bottom up:
 *
 *   the chip started        -> Valid
 *   plus a recent reading   -> Fresh
 *   accel AND gyro fresh    -> attitudeHealthy, so flight is allowed
 *   magnetometer fresh      -> headingHealthy, so compass is allowed
 *
 * Requiring Fresh and not merely Valid is the whole safety point: a
 * chip that woke at power-on and then went silent must not be trusted.
 */
void updateSensorHealth() {

  // Copy across the identity results from startup.
  flightSensorData.health.imuIdentityValid = imuIdentityOk;

  flightSensorData.health.magnetometerIdentityValid = magIdentityOk;

  flightSensorData.health.hardwareTimestampEnabled = hardwareTimestampOn;

  /*
   * Valid simply mirrors "the chip finished starting up".
   *
   * The accelerometer and gyroscope share one chip, so one flag
   * decides both. If that chip is up, both are up.
   */
  flightSensorData.health.accelerometerValid = imuReady;

  flightSensorData.health.gyroscopeValid = imuReady;

  flightSensorData.health.magnetometerValid = magReady;

  /*
   * Fresh asks for more: the chip is up AND a reading arrived recently.
   *
   * && means both must be true. The chip check comes first on purpose,
   * because if it is false C++ skips the rest of the line entirely.
   * No point asking the age of readings from a chip that never woke.
   */
  flightSensorData.health.accelerometerFresh = imuReady && sampleIsFresh(flightSensorData.accelerometerLastUpdateUs, flightSensorData.accelerometerSampleCount, IMU_STALE_AFTER_US);

  flightSensorData.health.gyroscopeFresh = imuReady && sampleIsFresh(flightSensorData.gyroscopeLastUpdateUs, flightSensorData.gyroscopeSampleCount, IMU_STALE_AFTER_US);

  // The magnetometer gets the longer 100 ms allowance.
  flightSensorData.health.magnetometerFresh = magReady && sampleIsFresh(flightSensorData.magnetometerLastUpdateUs, flightSensorData.magnetometerSampleCount, MAG_STALE_AFTER_US);

  /*
   * The single most important line in this file.
   *
   * Attitude needs the accelerometer for which way is down and the
   * gyroscope for how fast it is turning. Both, fresh, or the drone
   * must not fly. motorController reads this through
   * flightAttitudeSensorsHealthy() and blocks everything when false.
   */
  flightSensorData.health.attitudeHealthy = flightSensorData.health.accelerometerFresh && flightSensorData.health.gyroscopeFresh;

  // Heading rests on the magnetometer alone. Useful, never required.
  flightSensorData.health.headingHealthy = flightSensorData.health.magnetometerFresh;

  // Everything good at once. For reporting, not for permission.
  flightSensorData.health.allSensorsHealthy = flightSensorData.health.attitudeHealthy && flightSensorData.health.headingHealthy;

  /*
   * || means "either one". Once either chip has worked we remember
   * that this module got going, and we never take it back. It records
   * history, so it cannot be used to decide whether to fly now.
   */
  if (imuReady || magReady) {

    flightSensorData.health.initialized = true;

  }

}


// ==================================================
// WHICH WAY UP
// ==================================================

/*
 * Turn a face into short text for the log.
 *
 * A switch is a tidy multi-way if. It compares face against each case
 * and runs the matching one. return leaves immediately, so no break
 * statements are needed here.
 *
 * "const char *" means text that we promise not to change. These are
 * fixed words built into the program, which is why handing them back
 * is safe.
 */
const char *orientationName(OrientationFace face) {

  switch (face) {

    case OrientationFace::XPositive: return "X+";

    case OrientationFace::XNegative: return "X-";

    case OrientationFace::YPositive: return "Y+";

    case OrientationFace::YNegative: return "Y-";

    case OrientationFace::ZPositive: return "Z+";

    case OrientationFace::ZNegative: return "Z-";

    // Covers Unknown and anything unexpected. Never guess a direction.
    default: return "unknown";

  }

}


/*
 * Read the chip's orientation byte and name the face.
 *
 * The chip packs six yes-or-no answers into one byte, one bit per
 * face. "source & 0x20" tests a single bit: & keeps only the bits set
 * in both numbers, so the result is non-zero only when that one bit is
 * raised.
 *
 * The bits are, from the manual:
 *   0x20  Z up        0x10  Z down
 *   0x08  Y up        0x04  Y down
 *   0x02  X up        0x01  X down
 *
 * Tested in that order, so if the chip ever raises two bits at once we
 * answer predictably rather than at random. No bits raised means it
 * does not know, and we say so instead of guessing.
 */
OrientationFace decodeOrientation(uint8_t source) {

  if (source & 0x20) return OrientationFace::ZPositive;

  if (source & 0x10) return OrientationFace::ZNegative;

  if (source & 0x08) return OrientationFace::YPositive;

  if (source & 0x04) return OrientationFace::YNegative;

  if (source & 0x02) return OrientationFace::XPositive;

  if (source & 0x01) return OrientationFace::XNegative;

  return OrientationFace::Unknown;

}


/*
 * Store the orientation, and notice if it changed.
 *
 * Two deliberate decisions here:
 *
 * An Unknown answer is thrown away rather than stored. Forgetting a
 * good answer because the chip went briefly quiet would be worse than
 * keeping the last known one.
 *
 * The changed flag is only raised when the face is genuinely different
 * from what we had. Otherwise it would be raised constantly and mean
 * nothing.
 */
void noteOrientation(uint8_t source) {

  OrientationFace face = decodeOrientation(source);

  // No usable answer, so keep whatever we already knew.
  if (face == OrientationFace::Unknown) {

    return;

  }

  // != means "is not equal to". Only a real change counts.
  if (face != flightSensorData.events.orientation) {

    flightSensorData.events.orientationChanged = true;

    flightSensorData.events.orientation = face;

  }

}


// ==================================================
// SWITCHING ON THE OPTIONAL DETECTORS
// ==================================================

/*
 * Print one line about whether a feature switched on.
 *
 * Runs only during startup, never in the flight loop, so the many
 * separate Serial.print calls are affordable here. Inside the loop
 * they would be far too slow, which is why the log lines further down
 * are built in memory and written once.
 *
 *   name     the feature, in plain words
 *   enabled  did it work
 *   reason   if not, which register write failed
 *
 * The reason text is the valuable part. "skipped" alone would leave
 * you guessing; naming the exact register is what makes a failure
 * fixable.
 */
void reportFeature(const char *name, bool enabled, const char *reason) {

  Serial.print("Feature ");

  Serial.print(name);

  Serial.print(": ");

  if (enabled) {

    Serial.println("enabled");

  } else {

    Serial.print("skipped, ");

    Serial.println(reason);

  }

}


/*
 * Set up tap detection.
 *
 * Each line writes one setting and gives up at the first failure. The
 * ! means "not", so "if (!write...) return false" reads as "if that
 * write did not work, stop here and report failure".
 *
 * Stopping early matters. Carrying on after a failed write would leave
 * the detector half configured, which is worse than off, because it
 * would report rubbish that looks real.
 *
 * Taps are not useful to a drone. They are here because the chip
 * offers them and they prove the detector plumbing works.
 */
bool configureTap() {

  // Let all three axes report taps. 0x07 is the lowest three bits.
  if (!imu.writeField(REG_TAP_CFG0, 3, 1, 0x07)) return false;

  // How hard a tap must be, on each axis in turn.
  // At +/-4 g, 1 tap LSB is FS/32 = 125 mg. 9 LSB is about 1.125 g.
  if (!imu.writeField(REG_TAP_CFG1, 5, 0, 9)) return false;

  if (!imu.writeField(REG_TAP_CFG2, 5, 0, 9)) return false;

  if (!imu.writeField(REG_TAP_THS_6D, 5, 0, 9)) return false;

  /*
   * Tap timing, three settings packed into one byte.
   *
   * 7 << 4 puts 7 in the top half, 1 << 2 puts 1 in the middle, and 2
   * sits at the bottom. The | glues them together. They are how long
   * to wait for a second tap, how long a tap may last, and the quiet
   * time required afterwards.
   */
  if (!imu.writeRegister(REG_INT_DUR2, (uint8_t)((7 << 4) | (1 << 2) | 2))) return false;

  // The last bit says double taps may be reported as well as single.
  return imu.writeField(REG_WAKE_UP_THS, 1, 7, 1);

}


/*
 * Set up which-face-is-up detection.
 *
 * One write. The value 2 selects a 60 degree tilt before the chip
 * decides a new face is up. Shallower would flicker between faces
 * whenever the board wobbled.
 */
bool configureOrientation() {

  // TAP_THS_6D bits 6:5 = 2 is the 60 degree threshold. D4D stays off.
  return imu.writeField(REG_TAP_THS_6D, 2, 5, 2);

}


/*
 * Set up free-fall detection.
 *
 * This one genuinely matters. Free fall means gravity has nearly
 * vanished, which for an aircraft means it is dropping. motorController
 * treats it as an emergency and blocks every capability.
 *
 * How the chip detects it: in free fall all three axes read near zero,
 * because nothing is pushing back against the sensor.
 */
bool configureFreeFall() {

  // How weak gravity must get. 312 mg is about a third of normal.
  // FREE_FALL bits 2:0 = 3 is the 312 mg threshold.
  if (!imu.writeField(REG_FREE_FALL, 3, 0, 3)) return false;

  // How long it must stay that weak, so a jolt is not mistaken for falling.
  if (!imu.writeField(REG_FREE_FALL, 5, 3, 6)) return false;

  // Clear a bit that would otherwise interfere with this timing.
  return imu.writeField(REG_WAKE_UP_DUR, 1, 7, 0);

}


/*
 * Set up significant-motion detection.
 *
 * The point is to tell real movement apart from motor vibration, so
 * both a strength and a duration are needed.
 */
bool configureSignificantMotion() {

  // How much movement counts. 2 counts is about 125 mg.
  // At +/-4 g, 1 wake LSB is FS/64 = 62.5 mg.
  if (!imu.writeField(REG_WAKE_UP_THS, 6, 0, 2)) return false;

  // How long it must last. Filters out single sharp knocks.
  return imu.writeField(REG_WAKE_UP_DUR, 2, 5, 1);

}


/*
 * The master switch for all those detectors.
 *
 * Every detector above also needs this one bit set, or none of them
 * report anything. Configuring a detector and forgetting this leaves
 * it silent, which is why the failure messages distinguish "its own
 * setup failed" from "the master switch failed".
 */
bool enableBasicInterruptFunctions() {

  return imu.writeField(REG_TAP_CFG2, 1, 7, 1);

}


/*
 * Set up the chip's waiting room.
 *
 * ORDER MATTERS HERE, and that is the one thing to remember.
 *
 * The queue is switched OFF first, by setting bypass mode. That throws
 * away anything stale sitting in it. Then all the settings are
 * written. Only at the very end is it switched into stream mode and
 * allowed to start collecting.
 *
 * Changing settings on a running queue leaves a mixture of items
 * recorded under old and new rules, and nothing downstream can tell
 * them apart.
 */
bool configureFifo() {

  // Off first, which also empties it.
  if (!imu.writeField(REG_FIFO_CTRL4, 3, 0, FIFO_MODE_BYPASS)) return false;

  /*
   * One byte carries both rates: accelerometer in the top half,
   * gyroscope in the bottom. << 4 moves the first value up four
   * places to make room, and | joins them.
   */
  uint8_t batch = (uint8_t)((FIFO_BATCH_104_HZ << 4) | FIFO_BATCH_104_HZ);

  if (!imu.writeRegister(REG_FIFO_CTRL3, batch)) return false;

  // Tell us once 16 items are waiting.
  if (!imu.writeRegister(REG_FIFO_CTRL1, FIFO_WATERMARK_WORDS)) return false;

  // The ninth bit of that depth, which we want as 0.
  if (!imu.writeField(REG_FIFO_CTRL2, 1, 0, 0)) return false;

  // Also queue temperature, more slowly, at 52 Hz.
  if (!imu.writeField(REG_FIFO_CTRL4, 2, 4, FIFO_TEMP_BATCH_52_HZ)) return false;

  // Also queue every clock reading, skipping none.
  if (!imu.writeField(REG_FIFO_CTRL4, 2, 6, FIFO_TIMESTAMP_NO_DECIMATION)) return false;

  // Now start it, keeping newest and dropping oldest when full.
  return imu.writeField(REG_FIFO_CTRL4, 3, 0, FIFO_MODE_STREAM);

}


/*
 * Switch on the features that live in the other bank.
 *
 *   allowPedometer  may we enable the step counter
 *
 * THE SHAPE OF THIS FUNCTION:
 * switch banks, do the work, switch back. The switch back is not
 * optional and not skippable, because every later reading in the whole
 * program depends on the everyday bank being selected.
 *
 * Each feature needs two writes: one to enable it, one to initialise
 * it. The initialise mailbox clears itself the moment it is written,
 * so those writes pass false for verify. Reading them back would
 * always disagree and report a failure that did not happen.
 */
bool configureEmbeddedFunctions(bool allowPedometer) {

  // Switch to the features bank. Nothing else works until this does.
  if (!imu.selectBank(BANK_EMBEDDED)) {

    return false;

  }

  // Track each result so the caller can report precisely what failed.
  bool pedometerEnabled = false;

  bool pedometerInit = false;

  if (allowPedometer) {

    // Bit 3 is the step counter.
    pedometerEnabled = imu.writeField(REG_EMB_FUNC_EN_A, 1, 3, 1);

    // EMB_FUNC_INIT_A bits clear themselves, so they are not read back.
    pedometerInit = imu.writeField(REG_EMB_FUNC_INIT_A, 1, 3, 1, false);

    // Both worked, so reset the step total to zero for a clean start.
    if (pedometerEnabled && pedometerInit) {

      imu.writeField(REG_EMB_FUNC_SRC, 1, 7, 1, false);

    }

  }

  // Bit 4 is tilt detection.
  bool tiltEnabled = imu.writeField(REG_EMB_FUNC_EN_A, 1, 4, 1);

  bool tiltInit = imu.writeField(REG_EMB_FUNC_INIT_A, 1, 4, 1, false);

  // Bit 5 is significant motion.
  bool motionEnabled = imu.writeField(REG_EMB_FUNC_EN_A, 1, 5, 1);

  bool motionInit = imu.writeField(REG_EMB_FUNC_INIT_A, 1, 5, 1, false);

  /*
   * Back to the everyday bank, and this is the critical line.
   *
   * Fail here and we report failure even though the features above
   * may have worked, because leaving the wrong bank selected would
   * make every later reading garbage.
   */
  if (!imu.selectBank(BANK_USER)) {

    return false;

  }

  /*
   * A feature counts as working only if every step for it worked.
   * Being strict here is what stops us reading a detector that is not
   * actually running.
   */
  featureSteps = allowPedometer && pedometerEnabled && pedometerInit;

  featureTilt = tiltEnabled && tiltInit;

  featureSignificantMotion = motionEnabled && motionInit;

  return true;

}


/*
 * Switch on every optional detector, and say what happened to each.
 *
 * Called once, from bringUpImu(), during startup.
 *
 * THE PATTERN REPEATED BELOW:
 * Each detector needs two things to work: its own settings, and the
 * master switch. So for each one we ask, in order, "did its settings
 * fail", "did the master switch fail", otherwise "it works". That is
 * why there are three branches per feature instead of a simple yes
 * or no.
 *
 * The point of all this branching is the startup log. Knowing a
 * feature is off is nearly useless; knowing which register write
 * failed tells you where to look.
 *
 * Nothing here can stop the module. Every detector is optional, and
 * the accelerometer and gyroscope keep working regardless.
 */
void setupImuFeatures() {

  /*
   * Two failure messages used repeatedly below, written once here.
   * Both name the exact register, because that is the detail that
   * makes a failure diagnosable.
   */

  const char *interruptReason = "TAP_CFG2 (0x58) INTERRUPTS_ENABLE write failed";

  const char *bankReason = "FUNC_CFG_ACCESS (0x01) embedded bank select failed";

  /*
   * Configure everything first, remembering each result, then report.
   * Doing all the writes before any of the printing keeps the slow
   * Serial work out of the middle of the chip conversation.
   */

  bool tapConfigured = configureTap();

  bool orientationConfigured = configureOrientation();

  bool fallConfigured = configureFreeFall();

  bool wakeConfigured = configureSignificantMotion();

  // The shared master switch. If this failed, nothing below reports.
  bool interruptsOk = enableBasicInterruptFunctions();

  // Taps need their own settings AND the master switch.
  featureTap = tapConfigured && interruptsOk;

  if (!tapConfigured) {

    reportFeature("single tap", false, "tap register write failed");

    reportFeature("double tap", false, "tap register write failed");

  } else if (!interruptsOk) {

    reportFeature("single tap", false, interruptReason);

    reportFeature("double tap", false, interruptReason);

  } else {

    reportFeature("single tap", true, "");

    reportFeature("double tap", true, "");

  }

  featureOrientation = orientationConfigured && interruptsOk;

  if (!orientationConfigured) {

    reportFeature("6D orientation", false, "TAP_THS_6D (0x59) SIXd_THS write failed");

  } else if (!interruptsOk) {

    reportFeature("6D orientation", false, interruptReason);

  } else {

    reportFeature("6D orientation", true, "");

  }

  featureFreeFall = fallConfigured && interruptsOk;

  if (!fallConfigured) {

    reportFeature("free-fall", false, "FREE_FALL (0x5D) write failed");

  } else if (!interruptsOk) {

    reportFeature("free-fall", false, interruptReason);

  } else {

    reportFeature("free-fall", true, "");

  }

  /*
   * The step counter has a hardware requirement the others do not:
   * the accelerometer must be running at 26 Hz or faster, or the chip
   * refuses to count. We ask the current rate and check before trying,
   * so a refusal gets an accurate explanation rather than a vague one.
   */
  bool pedometerRateOk = imu.getAccelDataRate() >= LSM6DS_RATE_26_HZ;

  // One trip into the other bank sets up steps, tilt and motion together.
  bool bankOk = configureEmbeddedFunctions(pedometerRateOk);

  if (!pedometerRateOk) {

    reportFeature("step counter", false, "accel ODR is below 26 Hz");

    reportFeature("step detector", false, "accel ODR is below 26 Hz");

  } else if (!bankOk) {

    reportFeature("step counter", false, bankReason);

    reportFeature("step detector", false, bankReason);

  } else if (!featureSteps) {

    reportFeature("step counter", false, "EMB_FUNC_EN_A (0x04) PEDO_EN write failed");

    reportFeature("step detector", false, "EMB_FUNC_EN_A (0x04) PEDO_EN write failed");

  } else {

    reportFeature("step counter", true, "");

    reportFeature("step detector", true, "");

  }

  if (!bankOk) {

    featureTilt = false;

    reportFeature("tilt", false, bankReason);

  } else if (!featureTilt) {

    reportFeature("tilt", false, "EMB_FUNC_EN_A (0x04) TILT_EN write failed");

  } else {

    reportFeature("tilt", true, "");

  }

  if (!wakeConfigured) {

    featureSignificantMotion = false;

    reportFeature("significant motion", false, "WAKE_UP_THS (0x5B) write failed");

  } else if (!interruptsOk) {

    featureSignificantMotion = false;

    reportFeature("significant motion", false, interruptReason);

  } else if (!bankOk) {

    featureSignificantMotion = false;

    reportFeature("significant motion", false, bankReason);

  } else if (!featureSignificantMotion) {

    reportFeature("significant motion", false, "EMB_FUNC_EN_A (0x04) SIGN_MOTION_EN write failed");

  } else {

    reportFeature("significant motion", true, "");

  }

  /*
   * The queue last, because it does not need the master switch and is
   * configured in one go. Its result is also copied into the published
   * data so a reader can see whether queue numbers mean anything.
   */
  featureFifo = configureFifo();

  flightSensorData.fifo.enabled = featureFifo;

  reportFeature("FIFO accel+gyro+temperature+timestamp", featureFifo, featureFifo ? "" : "FIFO_CTRL3/FIFO_CTRL4 write failed");

}


/*
 * Switch on the magnetometer's built-in thermometer.
 *
 * Done by hand because Adafruit's magnetometer driver does not offer
 * it. We open our own line to one mailbox and set one bit.
 *
 * Why bother with temperature: magnetic readings drift as the chip
 * warms, so knowing the temperature allows that to be corrected later.
 */
bool enableMagnetometerTemperature() {

  // Make sure our direct line to the chip is open.
  if (!magRegs.begin()) {

    return false;

  }

  // Point at the settings mailbox.
  Adafruit_BusIO_Register ctrl1(&magRegs, LIS3MDL_CTRL_REG1);

  // Narrow to the single top bit, which is the thermometer switch.
  Adafruit_BusIO_RegisterBits tempEn(&ctrl1, 1, 7);

  if (!tempEn.write(1)) {

    return false;

  }

  // Read it back. Only a confirmed 1 counts as success.
  return tempEn.read() == 1;

}


/*
 * Read the chip's own clock.
 *
 * Four bytes in a row, joined into one number, then multiplied by 25
 * because each tick is 25 microseconds.
 *
 * This clock is more trustworthy than our own for measuring how
 * steadily the sensor produces readings, because it keeps ticking at
 * the same pace no matter how busy our processor gets.
 */
bool updateImuTimestamp() {

  // Room for four bytes, all set to zero so nothing is left to chance.
  uint8_t bytes[4] = {0, 0, 0, 0};

  if (!imu.readRegisters(REG_TIMESTAMP0, bytes, 4)) {

    return false;

  }

  // Four bytes become one number.
  flightSensorData.imuTimestampTicks = bytesToUint32(bytes);

  // Ticks become microseconds. uint64_t first, because this grows big.
  flightSensorData.imuTimestampUs = (uint64_t)flightSensorData.imuTimestampTicks * TIMESTAMP_TICK_US;

  flightSensorData.haveImuTimestamp = true;

  // Feed it to the timing maths so gaps can be measured chip-side.
  noteChipTimestamp(flightSensorData.imuTimestampTicks);

  return true;

}


/*
 * Make sense of one item pulled out of the queue.
 *
 *   word  the seven bytes just read
 *
 * Byte 0 is a label. Bytes 1 to 6 are the reading. The label is what
 * tells us whether those six bytes are acceleration, turning speed,
 * temperature or a clock value, because all four kinds come out of
 * the same mailbox mixed together.
 *
 * Anything with an unrecognised label is ignored rather than guessed
 * at. Guessing would put a temperature into the acceleration field.
 */
void processFifoWord(const uint8_t *word) {

  /*
   * Dig the label out of byte 0.
   *
   * >> 3 shifts the byte right three places, throwing away three bits
   * we do not care about. & 0x1F then keeps the lowest five bits,
   * which is where the label lives. Both steps come from the manual's
   * layout of this byte.
   */
  uint8_t tag = (uint8_t)((word[0] >> 3) & 0x1F);

  if (tag == FIFO_TAG_ACCEL) {

    flightSensorData.fifo.accelerometerMps2.x = accelToMps2(bytesToInt16(word[1], word[2]));

    flightSensorData.fifo.accelerometerMps2.y = accelToMps2(bytesToInt16(word[3], word[4]));

    flightSensorData.fifo.accelerometerMps2.z = accelToMps2(bytesToInt16(word[5], word[6]));

    flightSensorData.fifo.haveAccelerometer = true;

  } else if (tag == FIFO_TAG_GYRO) {

    flightSensorData.fifo.gyroscopeRadPerSec.x = gyroToRadPerSec(bytesToInt16(word[1], word[2]));

    flightSensorData.fifo.gyroscopeRadPerSec.y = gyroToRadPerSec(bytesToInt16(word[3], word[4]));

    flightSensorData.fifo.gyroscopeRadPerSec.z = gyroToRadPerSec(bytesToInt16(word[5], word[6]));

    flightSensorData.fifo.haveGyroscope = true;

  } else if (tag == FIFO_TAG_TEMPERATURE) {

    flightSensorData.fifo.temperatureC = imu.temperatureCFromRaw(bytesToInt16(word[1], word[2]));

    flightSensorData.fifo.haveTemperature = true;

  } else if (tag == FIFO_TAG_TIMESTAMP) {

    uint8_t timestampBytes[4] = {word[1], word[2], word[3], word[4]};

    flightSensorData.fifo.timestampTicks = bytesToUint32(timestampBytes);

    flightSensorData.fifo.timestampUs = (uint64_t)flightSensorData.fifo.timestampTicks * TIMESTAMP_TICK_US;

    flightSensorData.fifo.haveTimestamp = true;

  }

}


/*
 * Pull a set number of items out of the queue.
 *
 *   words  how many to take
 *
 * The 512 clamp is a safety rail. 512 is the most the queue can hold,
 * so a larger request could only come from a corrupted status read,
 * and acting on it would spin here far too long.
 *
 * A failed read breaks out immediately instead of carrying on. If the
 * wire has gone quiet, the remaining attempts would fail too, and
 * each one costs flight-loop time.
 */
void drainFifo(uint16_t words) {

  if (words > 512) {

    words = 512;

  }

  // A for loop: start at 0, keep going while below words, add one each time.
  for (uint16_t i = 0; i < words; i++) {

    // Seven bytes per item: one label plus six of reading.
    uint8_t word[7];

    if (!imu.readRegisters(REG_FIFO_DATA_OUT_TAG, word, 7)) {

      countError(flightSensorData.errors.fifoWordRead);

      // Stop early. The wire is not answering.
      break;

    }

    processFifoWord(word);

  }

}


/*
 * Check the queue, and empty some of it if it is filling up.
 *
 * Called every pass. Mostly it just looks at the depth, which is our
 * early warning that the loop is falling behind.
 */
void updateFifo() {

  // The queue never set up, so its numbers would be meaningless.
  if (!featureFifo) {

    return;

  }

  // Two status mailboxes to read.
  uint8_t status1 = 0;

  uint8_t status2 = 0;

  /*
   * || means "or". If either read fails the whole thing failed.
   *
   * C++ stops at the first failure here, so a dead wire does not get
   * asked twice.
   */
  if (!imu.readRegister(REG_FIFO_STATUS1, &status1) || !imu.readRegister(REG_FIFO_STATUS2, &status2)) {

    countError(flightSensorData.errors.fifoStatusRead);

    return;

  }

  /*
   * The depth is nine bits spread across two mailboxes, because 512
   * will not fit in one byte.
   *
   * status1 holds the lower eight. The lowest two bits of status2 hold
   * the top, so they are shifted up eight places and joined on.
   */
  uint16_t unread = (uint16_t)status1 | (uint16_t)((status2 & 0x03) << 8);

  flightSensorData.fifo.unreadWords = unread;

  // Bit 0x80 means the queue reached the depth we asked to be told about.
  flightSensorData.fifo.watermark = (status2 & 0x80) != 0;

  // Bit 0x40 means it filled completely and readings were discarded.
  bool overrun = (status2 & 0x40) != 0;

  flightSensorData.fifo.overrun = overrun;

  /*
   * Count the overflow only when it first appears.
   *
   * The flag stays raised while the cause persists, so counting it
   * every pass would log one problem hundreds of times a second and
   * bury everything else.
   */
  if (overrun && !sawFifoOverrun) {

    countError(flightSensorData.errors.fifoOverrunCount);

  }

  sawFifoOverrun = overrun;

  /*
   * Only bother emptying it once 16 items have gathered, and never
   * take more than 32 in one pass.
   *
   * The reasoning is a deliberate trade. Live readings are what fly
   * the drone; the queue is only history. Spending a long time
   * emptying it would starve the very thing it is meant to support.
   *
   * Stream mode drops the oldest items when full, so what gets lost
   * under pressure is the stale end of the history. For flight that
   * is the right thing to lose.
   */
  // The live accel and gyro path has priority over FIFO history.
  // Stream mode means the oldest words are the ones lost.
  if (unread >= FIFO_WATERMARK_WORDS) {

    uint16_t drainCount = unread;

    if (drainCount > FIFO_MAX_DRAIN_PER_PASS) {

      drainCount = FIFO_MAX_DRAIN_PER_PASS;

    }

    drainFifo(drainCount);

  }

}


/*
 * Read the hardware step total.
 *
 *   count  where to put the answer
 *
 * The step total lives in the other bank, so this has to switch over
 * and switch back. Notice the failure path switches back too, even
 * though the read never happened. Leaving the wrong bank selected
 * would corrupt every later reading, so getting back is more important
 * than reporting this one failure cleanly.
 *
 * Success needs both the read AND the return trip to work.
 */
bool readStepCount(uint16_t *count) {

  if (!imu.selectBank(BANK_EMBEDDED)) {

    // Still try to get back, in case the switch half happened.
    imu.selectBank(BANK_USER);

    return false;

  }

  uint8_t bytes[2] = {0, 0};

  bool ok = imu.readRegisters(REG_STEP_COUNTER_L, bytes, 2);

  // Always switch back, whatever the read did.
  bool back = imu.selectBank(BANK_USER);

  if (!ok || !back) {

    return false;

  }

  // Two bytes into one number, low byte first.
  *count = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
  return true;

}


// ==================================================
// READING THE SENSORS, WHICH IS THE REAL WORK
// ==================================================

/*
 * Read the accelerometer and gyroscope once.
 *
 * This is the function the whole file exists for. Everything above is
 * preparation, everything below is reporting.
 *
 * THE FOUR STEPS:
 *   1. Ask whether fresh data is waiting.
 *   2. If not, leave. This is normal, not a fault.
 *   3. If so, read all fourteen bytes in one go.
 *   4. Convert them, store them, and note the time.
 *
 * Step 1 matters because reading the same values twice would be
 * counted as two readings and make the measured rate a lie.
 *
 * Step 3 is one read rather than several because every value then
 * belongs to the same instant. Reading axis by axis would mix
 * different moments together and corrupt the attitude maths.
 */
void updateImu() {

  // Somewhere for the "is data ready" byte.
  uint8_t statusReg = 0;

  if (!imu.readRegister(REG_STATUS, &statusReg)) {

    countError(flightSensorData.errors.imuStatusRead);

    return;

  }

  // Bit 0 is the accelerometer, bit 1 the gyroscope.
  bool accelReady = (statusReg & 0x01) != 0;

  bool gyroReady = (statusReg & 0x02) != 0;

  // Keep the raw byte and the unpacked flags for troubleshooting.
  flightSensorData.imuStatusRegister = statusReg;

  flightSensorData.accelerometerDataReady = accelReady;

  flightSensorData.gyroscopeDataReady = gyroReady;

  flightSensorData.imuTemperatureDataReady = (statusReg & 0x04) != 0;

  /*
   * Nothing new yet, so stop here.
   *
   * This happens constantly and is completely normal: the loop runs
   * faster than 104 times a second, so most passes find nothing new.
   * It is NOT counted as an error, because counting it would bury the
   * real failures under noise.
   */
  // A clear ready bit is not a sample and not a bus error.
  if (!accelReady && !gyroReady) {

    return;

  }

  /*
   * Fourteen bytes, in this order:
   *   0,1    temperature
   *   2..7   gyroscope X, Y, Z
   *   8..13  accelerometer X, Y, Z
   */
  uint8_t bytes[14];

  // sizeof(bytes) is 14, worked out by the compiler. Changing the
  // array size above cannot leave this number stale.
  if (!imu.readRegisters(REG_OUT_TEMP_L, bytes, sizeof(bytes))) {

    countError(flightSensorData.errors.imuSampleRead);

    return;

  }

  /*
   * Stamp the time now, right after the bytes arrived.
   *
   * Taken after the read, not before, because this is when the
   * numbers became ours. Timing from before would include however
   * long the wire took.
   */
  uint32_t completedUs = (uint32_t)micros();

  int16_t rawTemperature = bytesToInt16(bytes[0], bytes[1]);

  /*
   * Unpack the raw counts. Each axis is two bytes, low byte first.
   * Gyroscope comes before accelerometer, which is the chip's order,
   * not an obvious one.
   */

  flightSensorData.rawGyroscope.x = bytesToInt16(bytes[2], bytes[3]);

  flightSensorData.rawGyroscope.y = bytesToInt16(bytes[4], bytes[5]);

  flightSensorData.rawGyroscope.z = bytesToInt16(bytes[6], bytes[7]);

  flightSensorData.rawAccelerometer.x = bytesToInt16(bytes[8], bytes[9]);

  flightSensorData.rawAccelerometer.y = bytesToInt16(bytes[10], bytes[11]);

  flightSensorData.rawAccelerometer.z = bytesToInt16(bytes[12], bytes[13]);

  /*
   * Now convert those counts into real units, keeping both. The raw
   * numbers stay as the honest record of what the chip said.
   */

  flightSensorData.gyroscopeRadPerSec.x = gyroToRadPerSec(flightSensorData.rawGyroscope.x);

  flightSensorData.gyroscopeRadPerSec.y = gyroToRadPerSec(flightSensorData.rawGyroscope.y);

  flightSensorData.gyroscopeRadPerSec.z = gyroToRadPerSec(flightSensorData.rawGyroscope.z);

  flightSensorData.accelerationMps2.x = accelToMps2(flightSensorData.rawAccelerometer.x);

  flightSensorData.accelerationMps2.y = accelToMps2(flightSensorData.rawAccelerometer.y);

  flightSensorData.accelerationMps2.z = accelToMps2(flightSensorData.rawAccelerometer.z);

  flightSensorData.imuTemperatureC = imu.temperatureCFromRaw(rawTemperature);

  // ++ means add one. Counts how many full reads we have done.
  flightSensorData.imuPollCycles++;

  flightSensorData.lastImuReadUs = completedUs;

  // Tells any reader that something changed since they last looked.
  flightSensorData.sequence++;

  /*
   * Count the two streams separately, and only when that stream
   * really had new data.
   *
   * Both share one read, but they can become ready at different
   * moments. Counting both every time would inflate the rate of
   * whichever one was not actually new, and the rate is how we detect
   * trouble, so it has to stay truthful.
   */

  if (accelReady) {

    noteDelivery(accelerometerAccumulator, completedUs, ACCEL_EXPECTED_PERIOD_US);

    flightSensorData.accelerometerSampleCount++;

    // Freshness is judged from this moment.
    flightSensorData.accelerometerLastUpdateUs = completedUs;

  }

  if (gyroReady) {

    noteDelivery(gyroscopeAccumulator, completedUs, GYRO_EXPECTED_PERIOD_US);

    flightSensorData.gyroscopeSampleCount++;

    flightSensorData.gyroscopeLastUpdateUs = completedUs;

  }

  /*
   * Read the chip's clock too, if it is running.
   *
   * The && stops early: if the clock was never enabled, the read is
   * not even attempted, so nothing is counted as failing.
   */
  if (hardwareTimestampOn && !updateImuTimestamp()) {

    countError(flightSensorData.errors.imuTimestampRead);

  }

}


/*
 * Read the magnetometer once.
 *
 * Same shape as the accelerometer read: ask if data is ready, leave if
 * not, otherwise read and convert. Simpler though, because this chip
 * has no banks and no queue.
 *
 * It does have one extra thing worth understanding, which is the
 * overflow flag explained below.
 */
void updateMagnetometer() {

  uint8_t statusReg = 0;

  if (!readMagnetometerRegister(LIS3MDL_STATUS, &statusReg)) {

    countError(flightSensorData.errors.magnetometerStatusRead);

    return;

  }

  /*
   * Bit 0x80 means the chip overwrote a reading we never collected.
   *
   * THIS IS THE ONE THAT CAUGHT US OUT.
   * It is not a wiring fault. It means the chip is producing readings
   * faster than our loop collects them. It used to run at 155 Hz while
   * the loop managed about 147, so a few readings a second were lost
   * and this counter climbed steadily.
   *
   * The fix was to slow the chip to 80 Hz rather than to ignore the
   * flag. See MAG_EXPECTED_RATE_HZ. Now the counter should stay at
   * zero, which makes it a useful warning again: if it ever moves, the
   * loop has slowed down.
   */
  bool overrun = (statusReg & 0x80) != 0;

  // Count it only when it first appears, as with the queue.
  if (overrun && !sawMagnetometerOverrun) {

    countError(flightSensorData.errors.magnetometerOverrunCount);

  }

  sawMagnetometerOverrun = overrun;

  // Bit 0x08 means a fresh reading is waiting.
  bool ready = (statusReg & 0x08) != 0;

  flightSensorData.magnetometerDataReady = ready;

  // Nothing new. Normal, and not an error.
  if (!ready) {

    return;

  }

  // Six bytes: two each for X, Y and Z.
  uint8_t bytes[6];

  if (!readMagnetometerRegisters(LIS3MDL_OUT_X_L, bytes, sizeof(bytes))) {

    countError(flightSensorData.errors.magnetometerSampleRead);

    return;

  }

  uint32_t completedUs = (uint32_t)micros();

  flightSensorData.lastMagReadUs = completedUs;

  // Raw counts first, two bytes per axis.
  flightSensorData.rawMagnetometer.x = bytesToInt16(bytes[0], bytes[1]);

  flightSensorData.rawMagnetometer.y = bytesToInt16(bytes[2], bytes[3]);

  flightSensorData.rawMagnetometer.z = bytesToInt16(bytes[4], bytes[5]);

  // Then the same values in microtesla.
  flightSensorData.magneticFieldMicrotesla.x = magnetometerToMicrotesla(flightSensorData.rawMagnetometer.x);

  flightSensorData.magneticFieldMicrotesla.y = magnetometerToMicrotesla(flightSensorData.rawMagnetometer.y);

  flightSensorData.magneticFieldMicrotesla.z = magnetometerToMicrotesla(flightSensorData.rawMagnetometer.z);

  flightSensorData.magnetometerSampleCount++;

  flightSensorData.magnetometerLastUpdateUs = completedUs;

  flightSensorData.sequence++;

  noteDelivery(magnetometerAccumulator, completedUs, MAG_EXPECTED_PERIOD_US);

  // Its thermometer never switched on, so there is nothing more to read.
  if (!featureMagnetometerTemperature) {

    return;

  }

  /*
   * The temperature needs its own separate read.
   *
   * You might expect to grab it on the end of the XYZ read, since
   * 0x2E follows 0x28 closely. It does not work: the chip's
   * auto-increment wraps back to the start of the XYZ block when it
   * reaches the end, so a longer read returns XYZ again instead of
   * carrying on to the temperature. Hence a second trip.
   */
  // Auto-increment rolls over at the end of the output block, so TEMP_OUT_L/H
  // at 0x2E/0x2F cannot be read as a tail on the 0x28 XYZ burst.
  uint8_t temperatureBytes[2];

  if (!readMagnetometerRegisters(LIS3MDL_TEMP_OUT_L, temperatureBytes, sizeof(temperatureBytes))) {

    countError(flightSensorData.errors.magnetometerSampleRead);

    return;

  }

  // 8 LSB per degree C, and 0 LSB is 25 C.
  int16_t rawTemperature = bytesToInt16(temperatureBytes[0], temperatureBytes[1]);

  flightSensorData.magnetometerTemperatureC = (rawTemperature / 8.0f) + 25.0f;

  flightSensorData.haveMagnetometerTemperature = true;

}


/*
 * Collect anything the detectors have noticed.
 *
 * Every block is guarded by its feature flag, so a detector that never
 * switched on is never read. That guard is the point: an unconfigured
 * detector's mailbox still contains something, and that something
 * would look like a real event.
 *
 * Flags are only ever set true here, never false. They stay raised
 * until motorController calls clearFlightSensorEvents(), which is what
 * makes them mean "this happened since you last looked".
 */
void updateMotionEvents() {

  // Taps.
  if (featureTap) {

    uint8_t tap = 0;

    if (!imu.readRegister(REG_TAP_SRC, &tap)) {

      countError(flightSensorData.errors.tapRead);

    } else {

      // 0x10 is a double tap, 0x20 a single. Both can be set at once.
      if (tap & 0x10) flightSensorData.events.doubleTap = true;

      if (tap & 0x20) flightSensorData.events.singleTap = true;

    }

  }

  /*
   * Free fall. The one that matters for safety.
   *
   * motorController reads this through flightFreeFallDetected() and
   * treats it as an emergency, because gravity nearly vanishing means
   * the aircraft is dropping.
   */
  if (featureFreeFall) {

    uint8_t wake = 0;

    if (!imu.readRegister(REG_WAKE_UP_SRC, &wake)) {

      countError(flightSensorData.errors.freeFallRead);

    } else if (wake & 0x20) {

      flightSensorData.events.freeFall = true;

    }

  }

  // Which face is up.
  if (featureOrientation) {

    uint8_t d6d = 0;

    if (!imu.readRegister(REG_D6D_SRC, &d6d)) {

      countError(flightSensorData.errors.orientationRead);

    } else {

      noteOrientation(d6d);

    }

  }

  /*
   * Steps, tilt and significant motion all report through one shared
   * mailbox, so one read covers all three. The || asks "is any of them
   * switched on", because if none are there is nothing to read.
   */
  if (featureSteps || featureTilt || featureSignificantMotion) {

    uint8_t embedded = 0;

    if (!imu.readRegister(REG_EMB_FUNC_STATUS_MAINPAGE, &embedded)) {

      countError(flightSensorData.errors.embeddedStatusRead);

    } else {

      /*
       * Each line checks two things: that the feature is on, and that
       * its bit is raised. Checking the bit alone would report events
       * from a detector that is not even running.
       */

      if (featureSteps && (embedded & 0x08)) flightSensorData.events.stepDetected = true;

      if (featureTilt && (embedded & 0x10)) flightSensorData.events.tilt = true;

      if (featureSignificantMotion && (embedded & 0x20)) flightSensorData.events.significantMotion = true;

    }

  }

}


// ==================================================
// WAKING THE CHIPS UP
// ==================================================

/*
 * Get the accelerometer and gyroscope chip ready to use.
 *
 * Called once at startup, and then again from updateFlightSensors()
 * every 100 ms for as long as the chip is not ready. We never give up,
 * because a loose Qwiic cable pushed back in should start working
 * without needing a reboot.
 *
 * WHY EACH STEP HAS ITS OWN REMEMBERED FLAG:
 * Startup is five steps and any one can fail. The flags mean a retry
 * resumes where it stopped instead of redoing work, which matters
 * because this runs inside the flight loop.
 *
 * THE STEPS:
 *   1. Answer on the wire at all.
 *   2. Prove identity.
 *   3. Switch on multi-mailbox reads.
 *   4. Set ranges, rates and detectors.
 *   5. Declare ready.
 */
bool bringUpImu() {

  // Already done. Nothing to do.
  if (imuReady) {

    return true;

  }

  // Step 1: does the chip answer?
  if (!imuBeginOk) {

    // Wire1 is the Qwiic bus. This also sets up Adafruit's driver.
    imuBeginOk = imu.begin_I2C(IMU_ADDRESS, &Wire1);

    if (!imuBeginOk) {

      countError(flightSensorData.errors.imuBegin);

    }

  }

  /*
   * Step 2: is it the chip we think it is?
   *
   * Checked before trusting anything it says. A wrong answer means
   * either the wrong chip or a bad connection, and in both cases its
   * readings are meaningless.
   */
  if (!imuIdentityOk) {

    uint8_t whoAmI = 0;

    bool readOk = imu.readRegister(REG_WHO_AM_I, &whoAmI);

    // Store whatever came back, even if wrong. The actual value is the
    // useful clue when diagnosing.
    flightSensorData.imuWhoAmI = whoAmI;

    // Needs both a successful read AND the right value.
    imuIdentityOk = readOk && whoAmI == LSM6DSOX_WHO_AM_I_VALUE;

    flightSensorData.health.imuIdentityValid = imuIdentityOk;

    if (!imuIdentityOk) {

      countError(flightSensorData.errors.imuWhoAmI);

    }

  }

  /*
   * Either of the first two steps failed, so stop and arrange a retry.
   * Noting the time here is what spaces retries 100 ms apart instead
   * of hammering a dead chip every pass.
   */
  if (!imuBeginOk || !imuIdentityOk) {

    nextImuRetryMs = millis();

    return false;

  }

  /*
   * Step 3: switch on IF_INC, which lets one read walk through several
   * mailboxes.
   *
   * Without it, the fourteen-byte read returns the first byte fourteen
   * times, so temperature would appear as every axis. This one bit is
   * the difference between real readings and convincing nonsense,
   * which is why failure here marks the sensors invalid and bails out.
   */
  // Without IF_INC, a multi-byte read repeats OUT_TEMP_L.
  if (!imuIfIncOk) {

    imuIfIncOk = imu.writeField(REG_CTRL3_C, 1, 2, 1);

    if (!imuIfIncOk) {

      countError(flightSensorData.errors.ifInc);

      flightSensorData.health.accelerometerValid = false;

      flightSensorData.health.gyroscopeValid = false;

      nextImuRetryMs = millis();

      return false;

    }

  }

  // Step 4: all the settings. Done once only.
  if (!imuConfigured) {

    // Start the chip's own 25 microsecond clock.
    hardwareTimestampOn = imu.writeField(REG_CTRL10_C, 1, 5, 1);

    flightSensorData.health.hardwareTimestampEnabled = hardwareTimestampOn;

    Serial.println(hardwareTimestampOn ? "LSM6DSOX hardware timestamp enabled." : "LSM6DSOX hardware timestamp FAILED.");

    /*
     * Ranges and rates.
     *
     * 4 g suits a drone: enough headroom for hard manoeuvres while
     * keeping fine resolution. A wider range would waste precision.
     *
     * 500 degrees per second likewise covers normal rotation without
     * throwing away detail.
     *
     * These two choices are what ACCEL_MG_PER_LSB and
     * GYRO_MDPS_PER_LSB were copied from the manual to match. Change a
     * range here and those conversion constants must change too, or
     * every reading silently becomes wrong.
     */

    imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);

    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);

    imu.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);

    imu.setGyroDataRate(LSM6DS_RATE_104_HZ);

    // The optional detectors and the queue, with their own reporting.
    setupImuFeatures();

    imuConfigured = true;

  }

  // Step 5: ready. Readings from here on can be trusted.
  imuReady = true;

  flightSensorData.health.accelerometerValid = true;

  flightSensorData.health.gyroscopeValid = true;

  return true;

}


/*
 * Get the magnetometer ready to use.
 *
 * Same never-give-up pattern as above, and deliberately independent of
 * it. A dead magnetometer must not stop the accelerometer, because
 * attitude can be held without a compass. Losing heading costs a
 * capability, not the aircraft.
 *
 * This chip needs two connections opened, not one: Adafruit's driver
 * for the ordinary settings, and our own direct line for the mailboxes
 * their driver does not expose.
 */
bool bringUpMag() {

  if (magReady) {

    return true;

  }

  // Open Adafruit's driver.
  if (!magBeginOk) {

    magBeginOk = magnetometer.begin_I2C(MAG_ADDRESS, &Wire1);

    if (!magBeginOk) {

      countError(flightSensorData.errors.magBegin);

    }

  }

  // Open our own direct line to the same chip.
  if (!magRegsOk) {

    magRegsOk = magRegs.begin();

    if (!magRegsOk) {

      countError(flightSensorData.errors.magBegin);

    }

  }

  // Prove identity, using our direct line. Expect 0x3D.
  if (!magIdentityOk) {

    uint8_t whoAmI = 0;

    // The magRegsOk && first means we do not attempt the read unless
    // that line actually opened.
    bool readOk = magRegsOk && readMagnetometerRegister(REG_WHO_AM_I, &whoAmI);

    flightSensorData.magnetometerWhoAmI = whoAmI;

    magIdentityOk = readOk && whoAmI == LIS3MDL_WHO_AM_I_VALUE;

    flightSensorData.health.magnetometerIdentityValid = magIdentityOk;

    if (!magIdentityOk) {

      countError(flightSensorData.errors.magWhoAmI);

    }

  }

  // Any of the three failed, so arrange a retry in 100 ms.
  if (!magBeginOk || !magRegsOk || !magIdentityOk) {

    nextMagRetryMs = millis();

    flightSensorData.health.magnetometerValid = false;

    return false;

  }

  // All settings, done once.
  if (!magConfigured) {

    // Ultra-high performance means the chip averages more internally,
    // which gives quieter readings. Worth it on a slow sensor.
    magnetometer.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);

    // Continuous means it keeps measuring by itself, rather than
    // waiting to be asked each time.
    magnetometer.setOperationMode(LIS3MDL_CONTINUOUSMODE);

    /*
     * 80 Hz, not 155 Hz, so the chip never outruns the loop. See
     * MAG_EXPECTED_RATE_HZ for the reasoning.
     *
     * 80 Hz is a plain output rate, so it leaves the ultra-high
     * performance mode set above alone. Asking for 155 Hz would
     * reset that mode itself, because the library forces ultra-high
     * for its fast rates.
     */

    magnetometer.setDataRate(LIS3MDL_DATARATE_80_HZ);

    // 4 gauss is the most sensitive range, and Earth's field is weak,
    // so this is the right pick. It matches LIS3MDL_LSB_PER_GAUSS_4G.
    magnetometer.setRange(LIS3MDL_RANGE_4_GAUSS);

    featureMagnetometerTemperature = enableMagnetometerTemperature();

    reportFeature("LIS3MDL temperature", featureMagnetometerTemperature, featureMagnetometerTemperature ? "" : "CTRL_REG1 (0x20) TEMP_EN write failed");

    magConfigured = true;

  }

  magReady = true;

  flightSensorData.health.magnetometerValid = true;

  return true;

}


// ==================================================
// WHAT motorController CALLS
// ==================================================

/*
 * Start both chips. motorController calls this once, from setup().
 *
 * Both are attempted whatever happens, which is the whole point. The
 * two results are collected first and only then combined, so a failed
 * accelerometer cannot cause the magnetometer to be skipped.
 *
 * Returning false does not mean give up. It means "not everything came
 * up yet", and updateFlightSensors() keeps retrying in the background.
 */
bool setupFlightSensors() {

  bool imuUp = bringUpImu();

  bool magUp = bringUpMag();

  // Fill in the trust summary so the startup log can report it.
  updateSensorHealth();

  return imuUp && magUp;

}


/*
 * Read the step total, at most ten times a second.
 *
 *   nowMs  the current time, passed in so millis() is read once
 *
 * Rate limited because reading it requires two bank switches, which
 * are expensive, and nobody needs step counts quickly.
 *
 * Note the extra bank check at the end. readStepCount() already
 * switches back, so this is a belt-and-braces confirmation of the one
 * thing that must never be left wrong.
 */
void updateStepCounter(uint32_t nowMs) {

  // Too soon since the last read. Safe across the millis() wrap.
  if ((uint32_t)(nowMs - nextStepReadMs) < STEP_READ_INTERVAL_MS) {

    return;

  }

  nextStepReadMs = nowMs;

  // Start from the value we already have, so a failed read leaves the
  // last known total in place rather than zeroing it.
  uint16_t count = flightSensorData.events.stepCount;

  if (readStepCount(&count)) {

    flightSensorData.events.stepCount = count;

  } else {

    countError(flightSensorData.errors.stepCountRead);

  }

  // Confirm we really are back in the everyday bank.
  if (!imu.selectBank(BANK_USER)) {

    countError(flightSensorData.errors.userBankSelect);

  }

}


/*
 * One pass of sensor work. motorController calls this every loop.
 *
 * This is the busiest function in the file and it runs roughly 150
 * times a second, so nothing here is allowed to block or wait.
 *
 * THE ORDER, AND WHY IT IS THIS ORDER:
 *   1. Stamp the time.
 *   2. Retry any sleeping chip, but no more than every 100 ms.
 *   3. Make sure the everyday mailbox bank is selected.
 *   4. Read the accelerometer and gyroscope, events, queue, steps.
 *   5. Read the magnetometer.
 *   6. Work out rates once a second.
 *   7. Refresh the trust summary.
 *
 * Step 3 before step 4 is the safeguard that matters. Something else
 * may have left the other bank selected, and reading the everyday
 * mailboxes while the features bank is showing returns numbers that
 * look plausible and are entirely wrong.
 *
 * The magnetometer in step 5 sits outside the bank check on purpose.
 * It is a different chip with no banks, so an accelerometer problem
 * must not stop it being read.
 */
void updateFlightSensors() {

  // A time for this pass that never runs backwards.
  flightSensorData.timestampUs = monotonicMicroseconds();

  // Read the millisecond clock once and reuse it, so every decision
  // below is made against the same instant.
  uint32_t nowMs = millis();

  /*
   * Retry anything that has not woken up, no more often than every
   * 100 ms. Retrying on every pass would spend the flight loop's time
   * talking to a chip that is not there.
   */

  if (!imuReady && (uint32_t)(nowMs - nextImuRetryMs) >= SENSOR_RETRY_MS) {

    bringUpImu();

  }

  if (!magReady && (uint32_t)(nowMs - nextMagRetryMs) >= SENSOR_RETRY_MS) {

    bringUpMag();

  }

  /*
   * Confirm the everyday bank before reading anything from this chip.
   *
   * Starts as false, so if the chip is not ready we never even try to
   * read it.
   */
  bool userBankOk = false;

  if (imuReady) {

    userBankOk = imu.selectBank(BANK_USER);

    if (!userBankOk) {

      countError(flightSensorData.errors.userBankSelect);

    }

  }

  /*
   * Only read if we are certain of the bank. Skipping a pass is
   * harmless; reading from the wrong bank would feed the flight
   * controller fiction.
   */
  if (userBankOk) {

    // The readings that actually matter for flight.
    updateImu();

    // Taps, tilt, free fall.
    updateMotionEvents();

    // Queue depth, and empty some if needed.
    updateFifo();

    // Steps, if that feature came up. Rate limited inside.
    if (featureSteps) {

      updateStepCounter(nowMs);

    }

  }

  // The magnetometer, independent of everything above.
  if (magReady) {

    updateMagnetometer();

  }

  // Rates and gap reports, which only act once a second.
  updateSampleRateDiagnostics();

  // Last of all, refresh the yes-or-no answers motorController reads.
  updateSensorHealth();

}


/*
 * Hand out the whole clipboard, read-only.
 *
 * "const FlightSensorData &" means two things. The & avoids copying a
 * large structure on every call. The const promises the caller cannot
 * change it, so this file stays the only writer and nobody else can
 * quietly alter a reading.
 */
const FlightSensorData &getFlightSensorData() {

  return flightSensorData;

}


/*
 * May the aircraft fly?
 *
 * The single most important question this file answers. True needs the
 * accelerometer and gyroscope both working and both recently heard
 * from. motorController blocks every capability when this is false.
 */
bool flightAttitudeSensorsHealthy() {

  return flightSensorData.health.attitudeHealthy;

}


/*
 * Can the compass heading be trusted?
 *
 * Useful but never required. Losing this costs heading correction,
 * position hold and return to home. It does not stop the aircraft
 * holding itself level.
 */
bool flightHeadingSensorHealthy() {

  return flightSensorData.health.headingHealthy;

}


/*
 * Free fall seen since the last clearFlightSensorEvents() call.
 *
 * The event flags are latched inside this module, and motorController
 * cannot reach into FlightSensorData because Arduino compiles the main
 * file before this struct is defined. This accessor is how the
 * controller asks the safety question.
 */
bool flightFreeFallDetected() {

  return flightSensorData.events.freeFall;

}


/*
 * Is absolutely everything healthy?
 *
 * For reporting only. Never use this to decide whether to fly: it goes
 * false when the magnetometer hiccups, and that is not a reason to
 * ground an aircraft that can still hold itself level.
 */
bool allFlightSensorsHealthy() {

  return flightSensorData.health.allSensorsHealthy;

}


/*
 * Wipe the event flags.
 *
 * motorController calls this at the end of every pass, after it has
 * read them. That is what turns the flags from "this happened at some
 * point" into "this happened since you last looked".
 *
 * Without it, one free-fall event at power-on would read as "still
 * falling" for the rest of the flight.
 *
 * The step total is deliberately NOT cleared. It is a running count
 * owned by the chip, not an event.
 */
void clearFlightSensorEvents() {

  flightSensorData.events.singleTap = false;

  flightSensorData.events.doubleTap = false;

  flightSensorData.events.stepDetected = false;

  flightSensorData.events.tilt = false;

  flightSensorData.events.freeFall = false;

  flightSensorData.events.significantMotion = false;

  flightSensorData.events.orientationChanged = false;

}


/*
 * Add every error counter together into one number.
 *
 * Handy for a single log line, but one total cannot say what is wrong.
 * printFlightSensorErrorBreakdown() below names the counters that are
 * actually moving, which is what turns a number into a diagnosis.
 *
 * The local reference just saves writing flightSensorData.errors
 * seventeen times.
 */
uint32_t totalSensorErrors() {

  const FlightSensorErrors &errors = flightSensorData.errors;

  return errors.userBankSelect
      + errors.imuBegin
      + errors.imuWhoAmI
      + errors.ifInc
      + errors.imuStatusRead
      + errors.imuSampleRead
      + errors.imuTimestampRead
      + errors.magBegin
      + errors.magWhoAmI
      + errors.magnetometerStatusRead
      + errors.magnetometerSampleRead
      + errors.fifoStatusRead
      + errors.fifoWordRead
      + errors.tapRead
      + errors.freeFallRead
      + errors.orientationRead
      + errors.embeddedStatusRead
      + errors.stepCountRead
      + errors.magnetometerOverrunCount
      + errors.fifoOverrunCount;

}


// ==================================================
// PRINTING, FOR HUMANS ONLY
// ==================================================

/*
 * Nothing below this line affects flying. It is all reporting.
 *
 * ONE THING TO UNDERSTAND ABOUT Serial ON THIS BOARD:
 * On the UNO Q, Serial is not a plain wire. It is the Linux side of
 * the board, reached across a bridge, and every single print call
 * waits for an answer. The cost is per CALL, not per letter.
 *
 * That is why there are two styles here. The long dump below uses many
 * print calls and is only for bench testing. The one-line summaries at
 * the very bottom build their text in memory first and then make
 * exactly one call, which is what makes them safe to leave running.
 *
 * Leaving the long dump on once stalled the loop for 3.3 seconds a
 * time, dropped the sensors to 0.3 readings a second and overflowed
 * the queue. The sensors were fine. The printing was the fault.
 */

/*
 * Print a chip's identity byte next to the expected one.
 *
 *   name      which chip
 *   value     what it answered
 *   expected  what it should have answered
 *
 * The lone "0" is for padding. Serial prints hex without leading
 * zeros, so 0x0D would come out as "0xD". Printing the extra zero
 * keeps every value two digits wide and easy to compare.
 */
void printWhoAmI(const char *name, uint8_t value, uint8_t expected) {

  Serial.print(name);

  Serial.print(" WHO_AM_I: 0x");

  if (value < 0x10) {

    Serial.print("0");

  }

  Serial.print(value, HEX);

  Serial.print(" expected 0x");

  Serial.println(expected, HEX);

}


/*
 * Print one timing report.
 *
 * The not-valid branch prints "waiting" rather than a row of zeros. A
 * zero gap looks like a real measurement and would be read as a
 * terrible result, when the truth is simply that no measurement exists
 * yet.
 */
void printTiming(const char *name, const SampleTiming &timing) {

  Serial.print(name);

  Serial.print(" interval last/min/max/mean: ");

  if (!timing.valid) {

    Serial.println("waiting");

    Serial.print(name);

    Serial.println(" jitter: waiting");

    return;

  }

  Serial.print(timing.lastIntervalUs);

  Serial.print(" / ");

  Serial.print(timing.minimumIntervalUs);

  Serial.print(" / ");

  Serial.print(timing.maximumIntervalUs);

  Serial.print(" / ");

  Serial.print(timing.averageIntervalUs, 1);

  Serial.println(" us");

  Serial.print(name);

  Serial.print(" jitter: ");

  Serial.print(timing.maximumJitterUs);

  Serial.println(" us");

}


/*
 * Print an X, Y, Z reading in real units.
 *
 *   digits  how many decimal places
 *
 * The digit count varies by sensor because their useful precision
 * does. Turning speed is printed to five places since the numbers are
 * tiny; magnetic strength needs only three.
 */
void printVector(const char *name, const Vector3Float &value, int digits, const char *unit) {

  Serial.print(name);

  Serial.print(" ");

  Serial.print(value.x, digits);

  Serial.print(" ");

  Serial.print(value.y, digits);

  Serial.print(" ");

  Serial.print(value.z, digits);

  Serial.println(unit);

}


/*
 * Print an X, Y, Z reading as raw chip counts.
 *
 * No decimal places because counts are whole numbers. Printed
 * alongside the converted values so that if a conversion constant is
 * ever wrong, the honest numbers are still in the log.
 */
void printRawVector(const char *name, const Vector3Int16 &value) {

  Serial.print(name);

  Serial.print(" ");

  Serial.print(value.x);

  Serial.print(" ");

  Serial.print(value.y);

  Serial.print(" ");

  Serial.println(value.z);

}


/*
 * The full dump. Everything this module knows, in about forty lines.
 *
 * BENCH USE ONLY. DO NOT LEAVE THIS RUNNING.
 * Each line is several Serial calls and each call blocks, so this
 * takes long enough to wreck the flight loop. motorController keeps it
 * behind MOTOR_CONTROLLER_VERBOSE_DEBUG, which is off.
 *
 * It prints unconditionally when called. The decision about when
 * belongs to motorController, so there is one switch in the project
 * rather than one per sensor file.
 */
void printFlightSensorDiagnostics() {

  Serial.println("----------------------------------------");

  printWhoAmI("LSM6DSOX", flightSensorData.imuWhoAmI, LSM6DSOX_WHO_AM_I_VALUE);

  Serial.print("LSM6DSOX up: ");

  Serial.println(flightSensorData.health.accelerometerValid ? "yes" : "no");

  printWhoAmI("LIS3MDL", flightSensorData.magnetometerWhoAmI, LIS3MDL_WHO_AM_I_VALUE);

  Serial.print("LIS3MDL up: ");

  Serial.println(flightSensorData.health.magnetometerValid ? "yes" : "no");

  Serial.print("Accel samples per second: ");

  Serial.println(flightSensorData.accelerometerRateHz, 1);

  Serial.print("Gyro samples per second: ");

  Serial.println(flightSensorData.gyroscopeRateHz, 1);

  Serial.print("Mag samples per second: ");

  Serial.println(flightSensorData.magnetometerRateHz, 1);

  Serial.print("IMU poll cycles: ");

  Serial.println(flightSensorData.imuPollCycles);

  Serial.print("Sequence: ");

  Serial.println(flightSensorData.sequence);

  Serial.print("Total errors: ");

  Serial.println(totalSensorErrors());

  printTiming("Accel", flightSensorData.accelerometerTiming);

  printTiming("Gyro", flightSensorData.gyroscopeTiming);

  printTiming("Mag", flightSensorData.magnetometerTiming);

  Serial.println("Timestamp interval is the chip's own 25 us clock.");

  printTiming("Timestamp", flightSensorData.chipTimestampTiming);

  printVector("Accel m/s^2", flightSensorData.accelerationMps2, 4, "");

  printRawVector("Accel raw", flightSensorData.rawAccelerometer);

  printVector("Gyro rad/s", flightSensorData.gyroscopeRadPerSec, 5, "");

  printRawVector("Gyro raw", flightSensorData.rawGyroscope);

  printVector("Mag uT", flightSensorData.magneticFieldMicrotesla, 3, "");

  printRawVector("Mag raw", flightSensorData.rawMagnetometer);

  Serial.print("IMU temperature C: ");

  Serial.println(flightSensorData.imuTemperatureC, 2);

  Serial.print("Mag temperature C: ");

  if (flightSensorData.haveMagnetometerTemperature) {

    Serial.println(flightSensorData.magnetometerTemperatureC, 2);

  } else {

    Serial.println("unavailable");

  }

  Serial.print("Attitude healthy: ");

  Serial.println(flightSensorData.health.attitudeHealthy ? "yes" : "no");

  Serial.print("Heading healthy: ");

  Serial.println(flightSensorData.health.headingHealthy ? "yes" : "no");

  Serial.print("6D face: ");

  Serial.println(orientationName(flightSensorData.events.orientation));

  Serial.print("Step count: ");

  Serial.println(flightSensorData.events.stepCount);

  Serial.print("FIFO unread: ");

  Serial.println(flightSensorData.fifo.unreadWords);

  Serial.println();

}


/*
 * One-line status for the normal flight log.
 *
 * WHY THIS IS ONE LINE:
 * On the UNO Q, Serial is the Linux-side monitor, not a UART. Every
 * Serial.print() becomes a blocking RPC call across the bridge, so the
 * cost is per call, not per character. The full dump above makes a few
 * hundred calls and stalls the loop for seconds, which starves the
 * control rate and overruns the FIFO.
 *
 * So this builds the whole line in memory and hands Serial exactly one
 * write. Rates are printed as whole.tenths with integer maths because
 * %f is not dependable on this toolchain.
 */
void printFlightSensorSummary() {

  /*
   * Space for the finished text. 160 letters is generous for a line
   * that normally runs about 80.
   */
  char line[160];

  float accelRate = flightSensorData.accelerometerRateHz;

  float gyroRate = flightSensorData.gyroscopeRateHz;

  float magRate = flightSensorData.magnetometerRateHz;

  /*
   * snprintf builds text into memory instead of sending it anywhere.
   * The %ld and %u marks are gaps, filled in order by the values
   * listed after.
   *
   * PRINTING DECIMALS THE HARD WAY:
   * %f is not dependable on this board's toolchain, so each rate is
   * split into two whole numbers. (long)accelRate throws away the
   * decimals to give the whole part. Subtracting that from the
   * original leaves just the fraction, and multiplying by 10 turns it
   * into tenths. So 103.84 is printed as 103 and 8.
   *
   * sizeof(line) is handed in so snprintf can never write past the
   * end of the array, whatever the values turn out to be.
   */
  int length = snprintf(line, sizeof(line),
      "IMU acc %ld.%ld gyr %ld.%ld mag %ld.%ld Hz | fifo %u | att %s | hdg %s | err %lu\n",
      (long)accelRate, (long)((accelRate - (long)accelRate) * 10.0f),
      (long)gyroRate, (long)((gyroRate - (long)gyroRate) * 10.0f),
      (long)magRate, (long)((magRate - (long)magRate) * 10.0f),
      (unsigned)flightSensorData.fifo.unreadWords,
      flightSensorData.health.attitudeHealthy ? "ok" : "--",
      flightSensorData.health.headingHealthy ? "ok" : "--",
      (unsigned long)totalSensorErrors());

  // snprintf returns how many letters it wrote. Zero or less means it
  // failed, so there is nothing worth sending.
  if (length <= 0) {

    return;

  }

  /*
   * If the text would have been longer than the space, snprintf
   * reports the length it WANTED, not what it managed to store. Taking
   * that number at face value would send rubbish from past the end of
   * the array, so it is capped here.
   */
  if (length > (int)sizeof(line) - 1) {

    length = (int)sizeof(line) - 1;

  }

  // One write to USB and the same write to the Pi on Serial3.
  writeFlightLog(line, length);
}


/*
 * Names the error counters that are actually counting.
 *
 * WHY:
 * totalSensorErrors() is one number, so a climbing total says
 * something is wrong without saying what. This lists only the non-zero
 * counters, which is the difference between "err 13156" and knowing
 * the FIFO is overrunning.
 *
 * Still one Serial write, so it is safe to leave on while tuning.
 */
void printFlightSensorErrorBreakdown() {

  const FlightSensorErrors &errors = flightSensorData.errors;

  /*
   * A tiny pairing of a name with its number, used only here.
   *
   * Declared inside the function because nothing else needs it. It
   * lets the list below be walked through in a loop instead of writing
   * twenty near-identical if statements.
   */
  struct ErrorCounter {
    const char *name;
    uint32_t count;
  };

  const ErrorCounter counters[] = {
      {"bank", errors.userBankSelect},
      {"imuBegin", errors.imuBegin},
      {"imuWho", errors.imuWhoAmI},
      {"ifInc", errors.ifInc},
      {"imuStatus", errors.imuStatusRead},
      {"imuSample", errors.imuSampleRead},
      {"imuTime", errors.imuTimestampRead},
      {"magBegin", errors.magBegin},
      {"magWho", errors.magWhoAmI},
      {"magStatus", errors.magnetometerStatusRead},
      {"magSample", errors.magnetometerSampleRead},
      {"magOverrun", errors.magnetometerOverrunCount},
      {"fifoStatus", errors.fifoStatusRead},
      {"fifoWord", errors.fifoWordRead},
      {"fifoOverrun", errors.fifoOverrunCount},
      {"tap", errors.tapRead},
      {"freeFall", errors.freeFallRead},
      {"orientation", errors.orientationRead},
      {"embedded", errors.embeddedStatusRead},
      {"step", errors.stepCountRead},
  };

  /*
   * How many entries are in that list.
   *
   * Total size divided by the size of one entry. Written this way so
   * adding a counter above needs no change here.
   */
  const uint8_t counterCount = sizeof(counters) / sizeof(counters[0]);

  char line[200];

  /*
   * Start the line with the label, and remember how many letters have
   * been used so far. Each counter is then appended after that point.
   */
  int used = snprintf(line, sizeof(line), "IMUERR");

  for (uint8_t index = 0; index < counterCount; index++) {

    /*
     * Skip anything at zero. This is what makes the line useful: only
     * counters that are actually counting appear, so the name of the
     * problem jumps out instead of hiding in a wall of zeros.
     *
     * continue means "go straight to the next loop turn".
     */
    if (counters[index].count == 0) {

      continue;

    }

    // How much space is left in the line.
    int remaining = (int)sizeof(line) - used;

    // Full. Stop rather than write past the end.
    if (remaining <= 1) {

      break;

    }

    /*
     * Append after what is already there. "line + used" means "start
     * writing at this position", and remaining caps how much may be
     * added.
     */
    int written = snprintf(line + used, remaining, " %s %lu",
        counters[index].name, (unsigned long)counters[index].count);

    // Failed, or did not fit. Mark the line full and stop.
    if (written < 0 || written >= remaining) {

      used = (int)sizeof(line) - 1;

      break;

    }

    used += written;
  }

  /*
   * 6 is the length of "IMUERR" on its own, so nothing was appended
   * and there are no errors at all. Say so plainly rather than
   * printing a bare label that looks like something went missing.
   */
  if (used == 6) {

    // Nothing counted, so say so rather than printing a bare label.
    used = snprintf(line, sizeof(line), "IMUERR none");

  }

  /*
   * Add the line break by hand, if there is room.
   *
   * used++ stores the character at the current position and then moves
   * the position on by one, so the length stays correct.
   */
  int remaining = (int)sizeof(line) - used;

  if (remaining > 1) {

    line[used++] = '\n';

  }

  // USB and the Pi UART both get this one line.
  writeFlightLog(line, used);
}


/*
 * Chip identity report for startup.
 *
 * motorController calls this once after setupFlightSensors() so the
 * startup log names which chip answered. Only this file can read the
 * WHO_AM_I bytes, so the report lives here.
 */
void printFlightSensorStartupReport() {

  printWhoAmI("LSM6DSOX", flightSensorData.imuWhoAmI, LSM6DSOX_WHO_AM_I_VALUE);

  Serial.print("LSM6DSOX up: ");

  Serial.println(imuReady ? "yes" : "no");

  printWhoAmI("LIS3MDL", flightSensorData.magnetometerWhoAmI, LIS3MDL_WHO_AM_I_VALUE);

  Serial.print("LIS3MDL up: ");

  Serial.println(magReady ? "yes" : "no");
}


