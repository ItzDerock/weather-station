#include <stdio.h>

#include "argentdata.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "ulp_riscv.h"

void app_main(void) {
  char *taskName = pcTaskGetName(NULL);
  ESP_LOGI(taskName, "StationFirmware starting.");

  // Wait 1 second to allow for USB serial connection
  // so chip doesnt sleep before we can see the log
  vTaskDelay(pdMS_TO_TICKS(1000));

  // figure the wakeup cause
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  switch (cause) {
  case ESP_SLEEP_WAKEUP_ULP:
    ESP_LOGI(taskName, "Wakeup reason: ULP");
    break;

  default:
    ESP_LOGI(taskName, "Wakeup reason: %d", cause);

    // load the ULP firmware
    ulp_riscv_reset();
    argentdata_init_gpio();
    argentdata_init_ulp();

    break;
  }

  struct ArgentSensorData argentData = {0};
  argentdata_reset_counts();

  while (true) {
    argentdata_read_values(&argentData);
    ESP_LOGI(taskName, "Wind speed: %f mph", argentData.wind_speed);
    ESP_LOGI(taskName, "Wind speed (gust): %f mph", argentData.wind_speed_gust);
    ESP_LOGI(taskName, "Rainfall: %f in/min", argentData.rainfall);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
