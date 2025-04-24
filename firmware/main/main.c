#include <stdio.h>

#include "esp_log.h"
#include "freertos/idf_additions.h"

void app_main(void) {
  char *taskName = pcTaskGetName(NULL);
  ESP_LOGI(taskName, "StationFirmware starting.");

  while (true) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
