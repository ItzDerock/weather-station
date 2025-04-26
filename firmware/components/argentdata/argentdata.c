#include "argentdata.h"
#include "driver/rtc_io.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hal/gpio_types.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc.h"
#include "ulp_riscv.h"

extern const uint8_t
    ulp_main_bin_start[] asm("_binary_ulp_argentdata_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_argentdata_bin_end");

#define ANEMOMETER_GPIO_NUM GPIO_NUM_11
#define RAIN_GAUGE_GPIO_NUM 12
// ESP32S3 Technical Reference Manual,
// Register 6.51. RTC_GPIO_PINn_REG
// 2: falling edge
#define ULP_GPIO_WAKEUP_INT_TYPE 2

const float ANEMOMETER_CALIBRATION_FACTOR = 1.492f; // 1.492MPH/Hz
const float RAIN_PER_TIP_FACTOR = 0.2794f;          // mm/tip
const float REPORTING_INTERVAL_SECONDS = 600.0f;    // 10 minutes
const float GUST_INTERVAL_SECONDS = 3.0f;

// from ULP
extern uint32_t ulp_max_anem_ticks_3_sec;
extern uint32_t ulp_anem_ticks_total;
extern uint32_t ulp_rain_gauge_count;
extern int8_t ulp_ulp_error_flags;

RTC_DATA_ATTR uint32_t argentdata_sensors_last_reset = 0;

/**
 * @brief Initialize the GPIOs for the ULP wakeup
 */
void argentdata_init_gpio(void) {
  ESP_LOGI("argent", "Configuring GPIOs for ULP wakeup via registers...");

  gpio_num_t pins[] = {ANEMOMETER_GPIO_NUM, RAIN_GAUGE_GPIO_NUM};

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
  ESP_LOGI("argent", "ULP Status is %d", ulp_ulp_error_flags);

  // Read the ULP data
  uint32_t max_anem_ticks_3_sec = ulp_max_anem_ticks_3_sec;
  uint32_t anem_ticks_total = ulp_anem_ticks_total;
  uint32_t rain_gauge_count = ulp_rain_gauge_count;

  // deltatime since last reset in seconds
  float delta_time =
      (float)(esp_timer_get_time() - argentdata_sensors_last_reset) /
      1000000.0f;

  ESP_LOGD("argent", "delta_time: %f", delta_time);
  ESP_LOGD("argent", "max_anem_ticks_3_sec: %lu", max_anem_ticks_3_sec);
  ESP_LOGD("argent", "anem_ticks_total: %lu", anem_ticks_total);
  ESP_LOGD("argent", "rain_gauge_count: %lu", rain_gauge_count);

  // calculate
  data->wind_speed_gust =
      (float)ulp_max_anem_ticks_3_sec / 3.0f * ANEMOMETER_CALIBRATION_FACTOR;

  data->wind_speed =
      (float)ulp_anem_ticks_total / delta_time * ANEMOMETER_CALIBRATION_FACTOR;

  data->rainfall =
      (float)ulp_rain_gauge_count / delta_time * RAIN_PER_TIP_FACTOR;
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
