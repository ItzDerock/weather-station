#include <stdio.h>

#include "argentdata.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "hal/i2c_types.h"
#include "lps22.h"
#include "shtc3.h"
#include "soc/clk_tree_defs.h"
#include "ulp_riscv.h"

static void setup_i2c(i2c_master_bus_handle_t *bus_handle);

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

  // initialize i2c
  i2c_master_bus_handle_t bus_handle;
  setup_i2c(&bus_handle);

  // esp_task_wdt_reset();

  // initialize each device
  i2c_master_dev_handle_t lps22_handle;
  esp_err_t lpsok = lps22_init(bus_handle, &lps22_handle, LPS22_DEFAULT_ADDR);

  esp_err_t err = i2c_master_probe(bus_handle, SHTC3_I2C_ADDR, 200);
  if (err != ESP_OK) {
    ESP_LOGE(taskName, "Failed to probe SHTC3: %s", esp_err_to_name(err));
    return;
  } else {
    ESP_LOGI(taskName, "SHTC3 probe successful");
  }

  // TODO: gracefully handle failure
  i2c_master_dev_handle_t shtc3_handle = shtc3_device_create(
      bus_handle, SHTC3_I2C_ADDR, CONFIG_SHTC3_I2C_CLK_SPEED_HZ);

  struct ArgentSensorData argentData = {0};
  argentdata_reset_counts();

  while (true) {
    // test argentdata
    argentdata_read_values(&argentData);
    ESP_LOGI(taskName, "Wind speed: %f mph", argentData.wind_speed);
    ESP_LOGI(taskName, "Wind speed (gust): %f mph", argentData.wind_speed_gust);
    ESP_LOGI(taskName, "Rainfall: %f in/min", argentData.rainfall);

    // test lps22
    if (lpsok == ESP_OK) {
      float pressure_hpa = 0.0f;
      esp_err_t err = lps22_read_data(lps22_handle, &pressure_hpa);

      if (err == ESP_OK) {
        ESP_LOGI(taskName, "Pressure: %.2f hPa", pressure_hpa);
      } else {
        ESP_LOGE(taskName, "Failed to read LPS22 data: %s",
                 esp_err_to_name(err));
      }
    } else {
      ESP_LOGE(taskName, "Failed to initialize LPS22 sensor: %s",
               esp_err_to_name(lpsok));
    }

    // test shtc3
    // CSE = Clock Stretching Enabled
    // NM = Normal Mode (as opposed to low power mode)
    float temperature, humidity;
    shtc3_get_th(shtc3_handle, SHTC3_REG_T_CSE_NM, &temperature, &humidity);

    ESP_LOGI(taskName, "Temperature: %.2f C", temperature);
    ESP_LOGI(taskName, "Humidity: %.2f %%", humidity);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

static void setup_i2c(i2c_master_bus_handle_t *bus_handle) {
  i2c_master_bus_config_t config = {.sda_io_num = GPIO_NUM_45,
                                    .scl_io_num = GPIO_NUM_48,
                                    .clk_source = I2C_CLK_SRC_DEFAULT,
                                    .i2c_port = I2C_NUM_0,
                                    .glitch_ignore_cnt = 7,
                                    .flags.enable_internal_pullup = true};

  ESP_ERROR_CHECK(i2c_new_master_bus(&config, bus_handle));
}
