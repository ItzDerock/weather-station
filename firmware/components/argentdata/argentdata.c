#include "argentdata.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_log_level.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_periph.h"
#include "soc/sens_reg.h"
#include "ulp_riscv.h"

extern const uint8_t
    ulp_main_bin_start[] asm("_binary_ulp_argentdata_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_argentdata_bin_end");

#define ANEMOMETER_GPIO_NUM GPIO_NUM_11
#define RAIN_GAUGE_GPIO_NUM 12

extern uint32_t ulp_anemometer_count;
extern uint32_t ulp_rain_gauge_count;

/**
 * @brief Initialize the GPIOs for the ULP wakeup
 */
void argentdata_init_gpio(void) {
  ESP_LOGI("argent", "Configuring GPIOs for ULP wakeup...");

  /* Initialize selected GPIO as RTC IO, enable input, disable pullup and
   * pulldown */
  rtc_gpio_init(ANEMOMETER_GPIO_NUM);
  rtc_gpio_set_direction(ANEMOMETER_GPIO_NUM, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pulldown_dis(ANEMOMETER_GPIO_NUM);
  rtc_gpio_pullup_en(ANEMOMETER_GPIO_NUM);
  rtc_gpio_hold_en(ANEMOMETER_GPIO_NUM);
  ESP_LOGI("argent", "GPIO %d configured as RTC IO", ANEMOMETER_GPIO_NUM);

  rtc_gpio_init(RAIN_GAUGE_GPIO_NUM);
  rtc_gpio_set_direction(RAIN_GAUGE_GPIO_NUM, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pulldown_dis(RAIN_GAUGE_GPIO_NUM);
  rtc_gpio_pullup_en(RAIN_GAUGE_GPIO_NUM);
  rtc_gpio_hold_en(RAIN_GAUGE_GPIO_NUM);
  ESP_LOGI("argent", "GPIO %d configured as RTC IO", RAIN_GAUGE_GPIO_NUM);
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
  ulp_set_wakeup_period(0, ULP_WAKEUP_PERIOD);

  // Start the program
  err = ulp_riscv_run();
  ESP_ERROR_CHECK(err);
  ESP_LOGI("argent", "ULP program started.");
}

void argentdata_read_sensors(struct ArgentSensorData *data) {
  // Read the ULP data
  uint32_t anemometer_raw_count = ulp_anemometer_count;
  uint32_t rain_gauge_raw_count = ulp_rain_gauge_count;

  ESP_LOGD("argent", "Anemometer count: %f", data->wind_speed);
  // data->rain_gauge = ulp_rain_gauge_count;
  // ESP_LOGI("argent", "Rain gauge count: %d", data->rain_gauge);
}
