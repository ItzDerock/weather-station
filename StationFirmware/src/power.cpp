#include <Arduino.h>
#include "include/utilities.h"
#include "include/power.hpp"
#include "XPowersLib.h"
#include "esp_sleep.h"

XPowersPMU power::PMU;

/**
 * Starts the I2C communication with the XPowers PMU chip.
 */
void power::initialize() {
  while (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
    Serial.println("Failed to initialize power......");
    delay(5000);
  }

  // On power cycles, restart the modem fully
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) {
    PMU.disableDC3();
    delay(200);
  }

  // Set the working voltage
  // SIM7080 modem main power channel 2700~ 3400V
  PMU.setDC3Voltage(3000);  // DO NOT CHANGE
  PMU.enableDC3();

  // Modem GPS Power channel
  PMU.setBLDO2Voltage(3300);
  PMU.enableBLDO2();  // The antenna power must be turned on to use the GPS
                      // function

  // TS Pin detection must be disabled, otherwise it cannot be charged
  PMU.disableTSPinMeasure();
}

bool power::enableModem() {}
