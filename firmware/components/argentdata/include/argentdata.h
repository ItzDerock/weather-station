#pragma once

// Configuration
#define ULP_WAKEUP_PERIOD 3000000

// From datasheet:
// > A wind speed of 1.492 MPH (2.4 km/h) causes the switch to close once per
// > second.
extern const float ANEMOMETER_CALIBRATION_FACTOR; // 1.492MPH/Hz

// From datasheet:
// > Each 0.011”
// > (0.2794 mm) of rain causes one momentary contact closure that can
// > be recorded with a digital counter or microcontroller interrupt input.
extern const float RAIN_PER_TIP_FACTOR;        // mm/tip
extern const float REPORTING_INTERVAL_SECONDS; // 10 minutes
extern const float GUST_INTERVAL_SECONDS;

struct ArgentSensorData {
  // mph
  float wind_speed;
  float wind_speed_gust;

  // in/min
  float rainfall;
};

/**
 * Initializes the GPIO pins.
 */
void argentdata_init_gpio(void);

/**
 * Loads the ULP program and starts.
 */
void argentdata_init_ulp(void);

/**
 * Should be called every 10 minutes.
 * Resets some ULP variables so we can start accumulating the data for the next
 * 10 minute average window.
 */
void argentdata_reset_counts(void);

/**
 * Reads the data from the ULP and places it into the data pointer.
 */
void argentdata_read_values(struct ArgentSensorData *data);
