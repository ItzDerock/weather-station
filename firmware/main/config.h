#pragma once

// Measurements for anemometer and wind vane are done by the ESP32S3's ULP
// co-processor. As such, the pins chosen MUST BE RTC-GPIOs.
#define ANEMOMETER_PIN 11
#define WIND_VANE_PIN 12
