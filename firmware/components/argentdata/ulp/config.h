#pragma once

#include "hal/adc_types.h"

// Must be RTC GPIOs.
#define ULP_WAKEUP_PERIOD 3000000 /* 3s in us */
#define REPORT_PERIOD 10          /* minutes */
#define ANEMOMETER_GPIO GPIO_NUM_11
#define RAIN_GAUGE_GPIO GPIO_NUM_12
#define WIND_VANE_GPIO GPIO_NUM_13

// See https://randomnerdtutorials.com/esp32-s3-devkitc-pinout-guide/
// for ADC channel/unit -> GPIO map
// ADC Unit 1_2 maps to IO3
#define WIND_VANE_ADC_CHANNEL ADC_CHANNEL_2
#define WIND_VANE_ADC_UNIT ADC_UNIT_1
#define WIND_VANE_ADC_ATTEN ADC_ATTEN_DB_0
#define WIND_VANE_ADC_WIDTH ADC_BITWIDTH_12
#define WIND_VANE_KEEP_N 10

// Value of the fixed resistor in the voltage divider (Ohms)
#define WIND_VANE_FIXED_RESISTOR_OHMS 10000.0f

// ADC Reference Voltage (V) corresponding to the chosen attenuation.
// Per design the ESP32-S3 ADC reference voltage is 1100 mV
#define WIND_VANE_ADC_REF_VOLTAGE 1.1f

// ADC Bit Width
// If `ADC_BITWIDTH_DEFAULT` is used for WIND_VANE_ADC_WIDTH
// this must be explicity set to something (since DEFAULT=0)
#define WIND_VANE_ADC_BIT_WIDTH WIND_VANE_ADC_WIDTH

// Maximum raw ADC reading (2^bits - 1)
#define WIND_VANE_ADC_MAX_READING ((1 << WIND_VANE_ADC_BIT_WIDTH) - 1)

// If we wake up every 3 (ULP_WAKEUP_PERIOD) seconds, but want to have 10
// (WIND_VANE_KEEP_N) datapoints over the past 10 (REPORT_PERIOD) minutes, then
// we should measure the wind vane every (WIND_VANE_MEASURE_INTERVAL) wakeups
// Calculation:
// Total time span (us) = REPORT_PERIOD * 60 * 1,000,000
// Time between measurements (us) = Total time span (us) / WIND_VANE_KEEP_N
// Interval (wakeups) = Time between measurements (us) / ULP_WAKEUP_PERIOD (us)
// Interval = (REPORT_PERIOD*60000000) / (WIND_VANE_KEEP_N * ULP_WAKEUP_PERIOD)
#define WIND_VANE_MEASUREMENT_INTERVAL                                         \
  ((REPORT_PERIOD * 60 * 1000000UL) / (WIND_VANE_KEEP_N * ULP_WAKEUP_PERIOD))

// bounds check
#if WIND_VANE_MEASUREMENT_INTERVAL == 0
#error "WIND_VANE_MEASUREMENT_INTERVAL cannot be zero!"
#endif
