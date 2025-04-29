#include "argentdata.h"
#include "driver/rtc_io.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "hal/adc_types.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc.h"
#include "ulp/config.h"
#include "ulp_adc.h"
#include "ulp_riscv.h"
#include "ulp_vars.h"
#include <float.h>
#include <math.h>

extern const uint8_t
    ulp_main_bin_start[] asm("_binary_ulp_argentdata_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_argentdata_bin_end");

// ESP32S3 Technical Reference Manual,
// Register 6.51. RTC_GPIO_PINn_REG
// 2: falling edge
#define ULP_GPIO_WAKEUP_INT_TYPE 2

const float ANEMOMETER_CALIBRATION_FACTOR = 1.492f; // 1.492MPH/Hz
const float RAIN_PER_TIP_FACTOR = 0.2794f;          // mm/tip
const float REPORTING_INTERVAL_SECONDS = 600.0f;    // 10 minutes
const float GUST_INTERVAL_SECONDS = 3.0f;

// lookup table for wind vane,
// resistance (ohms) -> direction (degrees)
static struct DirectionLookup lookupTable[] = {
    {33000, 0.0f},    {6570, 22.5f},   {8200, 45.0f},   {891, 67.5f},
    {1000, 90.0f},    {688, 112.5f},   {2200, 135.0f},  {1410, 157.5f},
    {3900, 180.0f},   {3140, 202.5f},  {16000, 225.0f}, {14120, 247.5f},
    {120000, 270.0f}, {42120, 292.5f}, {64900, 315.0f}, {21880, 337.5f}};

static const int lookupTableSize = sizeof(lookupTable) / sizeof(lookupTable[0]);

/**
 * @brief Converts a raw ADC reading from the wind vane voltage divider
 *        to a wind direction in degrees.
 *
 * @param adc_reading The raw value read from the ADC.
 * @return float The calculated wind direction in degrees (0.0 - 337.5),
 *               or a negative value (-1.0f) if the reading is invalid or
 *               results in an undefined resistance.
 */
static float wind_vane_adc_to_direction(uint32_t adc_reading) {
  // 1. Convert ADC reading to voltage
  // Ensure adc_reading doesn't exceed max possible value
  if (adc_reading > WIND_VANE_ADC_MAX_READING) {
    adc_reading = WIND_VANE_ADC_MAX_READING; // Clamp to max
  }

  float voltage_out = ((float)adc_reading / (float)WIND_VANE_ADC_MAX_READING) *
                      WIND_VANE_ADC_REF_VOLTAGE;

  // 2. Calculate Sensor Resistance (Rs)
  float sensor_resistance;

  // Handle edge case: Voltage near or at reference voltage (implies infinite R)
  // Use a small tolerance to avoid floating point issues
  if (voltage_out >= WIND_VANE_ADC_REF_VOLTAGE * 0.999f) {
    // Corresponds to the highest resistance in the table (120k -> 270 deg)
    // Or could treat as an open circuit / error
    sensor_resistance = 120000.0f; // Assign highest known resistance
  }
  // Handle edge case: Voltage near zero (implies zero R)
  else if (voltage_out <= 0.001f) {
    // Resistance is effectively zero. Find the lowest resistance direction.
    // The lowest in the table is 688 Ohms. Assigning 0 might lead to choosing
    // 688 Ohms anyway. Let's calculate it, it should be near zero.
    sensor_resistance = 0.0f;
  } else {
    // Calculate Rs using the voltage divider formula rearranged:
    // Rs = R_fixed * Vout / (Vref - Vout)
    sensor_resistance =
        WIND_VANE_FIXED_RESISTOR_OHMS *
        (voltage_out / (WIND_VANE_ADC_REF_VOLTAGE - voltage_out));
  }

  // Check for calculation errors (e.g., negative resistance if Vout > Vref
  // somehow)
  if (sensor_resistance < 0) {
    // printf("Error: Calculated negative resistance (Vout=%.3f)\n",
    // voltage_out);
    return -1.0f; // Indicate error
  }

  // 3. Find Closest Match in Lookup Table
  float min_diff = FLT_MAX;
  float best_direction = -1.0f; // Default to error/not found

  for (int i = 0; i < lookupTableSize; ++i) {
    float diff = fabsf(lookupTable[i].resistanceOhms - sensor_resistance);
    if (diff < min_diff) {
      min_diff = diff;
      best_direction = lookupTable[i].degrees;
    }
  }

  // 4. Return the Best Matching Direction
  return best_direction;
}

RTC_DATA_ATTR uint32_t argentdata_sensors_last_reset = 0;

/**
 * @brief Initialize the GPIOs for the ULP wakeup
 */
void argentdata_init_gpio(void) {
  ESP_LOGI("argent", "Configuring GPIOs for ULP wakeup via registers...");

  gpio_num_t pins[] = {ANEMOMETER_GPIO, RAIN_GAUGE_GPIO};

  for (int i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
    gpio_num_t pin = pins[i];
    ESP_LOGI("argent", "Configuring GPIO %d...", pin);

    // Initialize RTC IO basic properties
    rtc_gpio_init(pin);
    rtc_gpio_set_direction(pin, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_en(pin); // Enable pull-up
    rtc_gpio_pulldown_dis(pin);
    rtc_gpio_hold_en(pin); // Keep config during sleep

    // Get the RTC IO number (0-21) corresponding to the GPIO number
    int rtc_io_idx = rtc_io_number_get(pin);
    if (rtc_io_idx < 0) {
      ESP_LOGE("argent", "GPIO %d is not an RTC IO", pin);
      continue; // Skip if not an RTC IO
    }

    // Configure wakeup and interrupt type via register
    // Normally, you'd use rtc_gpio_wakeup_enable(pin, ...);
    // But it does not support edge triggering despite the technical reference
    // manual saying it does.
    //
    // RTC_GPIO_PINn_INT_TYPE GPIO interrupt type selection. 0: GPIO interrupt
    // disabled; 1: rising edge trigger; 2: falling edge trigger; 3: any edge
    // trigger; 4: low level trigger; 5: high level trigger. (R/W)
    uint32_t reg_addr = RTC_GPIO_PIN0_REG + (rtc_io_idx * sizeof(uint32_t));
    REG_SET_BIT(reg_addr, RTC_GPIO_PIN0_WAKEUP_ENABLE_M);
    REG_SET_FIELD(reg_addr, RTC_GPIO_PIN0_INT_TYPE, ULP_GPIO_WAKEUP_INT_TYPE);

    ESP_LOGI("argent",
             "GPIO %d (RTC IO %d) configured for ULP wakeup (Type: %d)", pin,
             rtc_io_idx, ULP_GPIO_WAKEUP_INT_TYPE);

    // Latch the configuration
    rtc_gpio_hold_en(pin);

    // DO NOT CALL rtc_gpio_wakeup_enable(pin, ...)
    // It doesn't support edge triggering (panics) and would overwrite INT_TYPE
  }

  // RTC Peripheral power domain needs to be on to keep SAR ADC config
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

  // set up the ADC for wind vane
  ulp_adc_cfg_t adc_cfg = {.adc_n = WIND_VANE_ADC_UNIT,
                           .channel = WIND_VANE_ADC_CHANNEL,
                           .width = WIND_VANE_ADC_WIDTH,
                           .atten = WIND_VANE_ADC_ATTEN,
                           .ulp_mode = ADC_ULP_MODE_RISCV};

  ESP_ERROR_CHECK(ulp_adc_init(&adc_cfg));
}

/**
 * Starts the ULP program.
 */
void argentdata_init_ulp(void) {
  ESP_LOGI("argent", "Loading program into ULP.");
  esp_err_t err = ulp_riscv_load_binary(
      ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start));
  ESP_ERROR_CHECK(err);

  // Wake the ULP every 3 seconds
  ESP_ERROR_CHECK(ulp_set_wakeup_period(0, ULP_WAKEUP_PERIOD));

  // Start the program
  err = ulp_riscv_run();
  ESP_ERROR_CHECK(err);
  ESP_LOGI("argent", "ULP program started.");
}

void argentdata_read_values(struct ArgentSensorData *data) {
  ESP_LOGI("argent", "ULP Status is %u", ulp_error_flags);

  // Read the ULP data
  uint32_t max_anem_ticks_3_sec = ulp_max_anem_ticks_3_sec;
  uint32_t anem_ticks_total = ulp_anem_ticks_total;
  uint32_t rain_gauge_count = ulp_rain_gauge_count;

  // deltatime since last reset in seconds
  float delta_time =
      (float)(esp_timer_get_time() - argentdata_sensors_last_reset) /
      1000000.0f;

  // ESP_LOGD("argent", "delta_time: %f", delta_time);
  // ESP_LOGD("argent", "max_anem_ticks_3_sec: %lu", max_anem_ticks_3_sec);
  // ESP_LOGD("argent", "anem_ticks_total: %lu", anem_ticks_total);
  // ESP_LOGD("argent", "rain_gauge_count: %lu", rain_gauge_count);

  // calculate
  data->wind_speed_gust =
      (float)max_anem_ticks_3_sec / 3.0f * ANEMOMETER_CALIBRATION_FACTOR;

  data->wind_speed =
      (float)anem_ticks_total / delta_time * ANEMOMETER_CALIBRATION_FACTOR;

  data->rainfall = (float)rain_gauge_count / delta_time * RAIN_PER_TIP_FACTOR;

  // start at index+1, then wrap around
  uint8_t resultingIndex = 0;
  for (uint8_t i = (uint8_t)ulp_wind_direction_index + 1; i < WIND_VANE_KEEP_N;
       i++) {
    data->degrees[resultingIndex++] =
        wind_vane_adc_to_direction(ulp_wind_direction[i]);
  }

  for (uint8_t i = 0; i < WIND_VANE_KEEP_N; i++) {
    data->degrees[resultingIndex++] =
        wind_vane_adc_to_direction(ulp_wind_direction[i]);
  }
}

/**
 * Resets the ULP variables.
 */
void argentdata_reset_counts() {
  ulp_anem_ticks_total = 0;
  ulp_rain_gauge_count = 0;
  ulp_max_anem_ticks_3_sec = 0;
  argentdata_sensors_last_reset = esp_timer_get_time();
}
