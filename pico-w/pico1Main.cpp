/*
WHAT:
  Program for Pico 1, the sensor board wired to the Uno Q JSPI connector.

WHY:
  Pico 1 answers the flight controller and, when the TF-Luna is fitted,
  collects distance. It must never command motors. The flight controller
  makes the aircraft decisions.

HOW:
  The SPI reply is placed from the clock edges on the first core.
  The second core reads the lidar on GP4 and GP5 (I2C or UART), so a
  sensor read cannot stretch the SPI bit timing.

  Flight-controller wiring:

    Uno Q JSPI pin 4  MOSI  -> Pico GP12  SPI1 RX
    Uno Q JSPI pin 1  MISO  <- Pico GP11  SPI1 TX
    Uno Q JSPI pin 3  SCK   -> Pico GP10  SPI1 SCK
    Uno Q D10         SS    -> Pico GP9   SPI1 CSn
    Uno Q JSPI pin 6  GND   -- Pico GND
    Uno Q JSPI pin 2  +5V   -> Pico VSYS

  TF-Luna wiring on GP4 and GP5:

    Luna pin 1  power  -> Pico pin 39  VSYS  (Uno JSPI 5 V)
    Luna pin 2  data   -> Pico pin 6   GP4
    Luna pin 3  data   -> Pico pin 7   GP5
    Luna pin 4  GND    -> Pico pin 38  GND
    Luna pin 5  mode   -> GND = I2C, open = UART, before power-up
    Luna pin 6  ready  -- open
  GP13 is driven high while this program is running. It is the pin
  directly below GP12. A meter from GP13 to GND reads about 3.3 V
  when the CPU is in this program.

IMPORTANT:
  This is Pico SDK code for a Pico W (RP2040). It is not an Arduino
  sketch and it will not compile inside the Uno Q program.

  The finished program is written into the Pico's own flash. The boot
  ROM starts that copy on every power-up and every reset, including
  when the only power is the Uno's 5 V. It does not wait for a
  computer. The BOOT button is only for replacing the stored program.
  Holding BOOT stops the stored program from starting.

  Power the Pico from one 5 V source at a time. JSPI pin 5 is the Uno Q
  reset pin. Leave it unconnected. Leave the Pico RUN pin unconnected.

  The green LED is not a power light. It is wired to the Wi-Fi chip,
  so 5 V on VSYS can be present while that LED stays dark. That chip
  is started on the first core, and the LED is turned on before the
  lidar is read. The second core blinks it. A lidar read uses a short
  timeout, so a missing sensor cannot leave the LED dark.

  The CPU itself runs from the 3.3 V pin, physical pin 36. That pin
  sits two pins below the GND under VSYS. It should read about 3.3 V
  once VSYS has 5 V. Near 0 V means the CPU is off.
*/

#include "pico1Luna.h"
#include "pico1SpiLink.h"

#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

// The pin directly below GP12. High means this program is running.
constexpr uint PICO1_RUN_PIN = 13;

// Reset the board if the flight loop ever stops petting the watchdog.
// The LED core is not part of this. A stuck LED must not reset the link.
constexpr uint32_t PICO1_WATCHDOG_MS = 1000;

// Half of a blink. On for this long, then off for this long.
constexpr int64_t PICO1_LED_HALF_PERIOD_US = 250000;


void showProgramRunning() {
  gpio_init(PICO1_RUN_PIN);
  gpio_set_dir(PICO1_RUN_PIN, GPIO_OUT);
  gpio_put(PICO1_RUN_PIN, 1);
}


// The Wi-Fi chip's alarm pool lives on the core that starts it.
// The first core owns that pool. Starting the chip on the second
// core leaves the green LED off.
bool statusLedReady = false;


bool startStatusLed() {
  if (cyw43_arch_init() != 0) {
    return false;
  }
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
  return true;
}


void runLunaCore() {
  bool ledOn = true;
  absolute_time_t nextToggle = delayed_by_us(get_absolute_time(), PICO1_LED_HALF_PERIOD_US);
  while (true) {
    if (statusLedReady && absolute_time_diff_us(nextToggle, get_absolute_time()) >= 0) {
      ledOn = !ledOn;
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, ledOn);
      nextToggle = delayed_by_us(get_absolute_time(), PICO1_LED_HALF_PERIOD_US);
    }
    updateLuna();
    sleep_ms(20);
  }
}


int main() {
  // The boot ROM calls main from flash. Nothing above this line waits
  // for USB, and this function never returns.
  // Raised before the Wi-Fi chip is touched, so a meter can see that
  // the CPU has entered this program even while the green LED is dark.
  showProgramRunning();
  statusLedReady = startStatusLed();

  // I2C0 on GP4 and GP5 is started here. The second core reads it.
  setupLuna();
  setupPico1Spi();
  // Armed after the LED start. That start can take longer than the
  // watchdog, and the watchdog would reset the chip before the light
  // came on.
  watchdog_enable(PICO1_WATCHDOG_MS, true);

  multicore_launch_core1(runLunaCore);

  while (true) {
    LunaReading lunaReading = currentLunaReading();
    setPico1LunaSample(
        lunaReading.distanceCentimeters,
        lunaReading.signalStrength,
        lunaReading.fresh);
    updatePico1Spi();
    watchdog_update();
  }
}
