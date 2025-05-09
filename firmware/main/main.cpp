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
#include "ltr390uv.h"
#include "shtc3.h"
#include "sim7080g_driver_esp_idf.h"
#include "soc/clk_tree_defs.h"
#include "ulp_riscv.h"

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"

static void setup_i2c(i2c_master_bus_handle_t *bus_handle,
                      i2c_master_bus_handle_t *bus_pmu_handle);

extern "C" void app_main(void) {
  char *taskName = pcTaskGetName(nullptr);
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
  i2c_master_bus_handle_t bus_pmu_handle;
  setup_i2c(&bus_handle, &bus_pmu_handle);

  // initialize PMU chip
  // XPowersPMU pmu;
  // pmu.begin(bus_pmu_handle, AXP2101_SLAVE_ADDRESS);

  // initialize each device
  i2c_master_dev_handle_t lps22_handle;
  esp_err_t lpsok = lps22_init(bus_handle, &lps22_handle, LPS22_DEFAULT_ADDR);

  // TODO: gracefully handle failure
  i2c_master_dev_handle_t shtc3_handle = shtc3_device_create(
      bus_handle, SHTC3_I2C_ADDR, CONFIG_SHTC3_I2C_CLK_SPEED_HZ);

  // initialize ltr390
  ltr390uv_config_t dev_cfg = I2C_LTR390UV_CONFIG_DEFAULT;
  ltr390uv_handle_t dev_hdl;

  ltr390uv_init(bus_handle, &dev_cfg, &dev_hdl);
  if (dev_hdl == NULL) {
    ESP_LOGE(taskName, "ltr390uv handle init failed");
    assert(dev_hdl);
  }

  struct ArgentSensorData argentData = {0, 0, 0};
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
      float lps_temperature = 0.0f;
      lps22_read_pressure(lps22_handle, &pressure_hpa);
      lps22_read_temperature(lps22_handle, &lps_temperature);

      ESP_LOGI(taskName, "Pressure: %.2f hPa", pressure_hpa);
      ESP_LOGI(taskName, "Temperature (LPS22): %.2f C", lps_temperature);
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

    // test ltr390
    float uvi;
    esp_err_t ltrerr = ltr390uv_get_ultraviolet_index(dev_hdl, &uvi);
    if (ltrerr != ESP_OK) {
      ESP_LOGE(taskName, "ltr390uv device read failed (%s)",
               esp_err_to_name(ltrerr));
    } else {
      ESP_LOGI(taskName, "Ultraviolet index: %f", uvi);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

static void setup_i2c(i2c_master_bus_handle_t *bus_handle,
                      i2c_master_bus_handle_t *bus_pmu_handle) {
  // I2C bus for all the sensors
  i2c_master_bus_config_t config = {
      .i2c_port = I2C_NUM_0,
      .sda_io_num = GPIO_NUM_45,
      .scl_io_num = GPIO_NUM_48,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .intr_priority = 0,
      .trans_queue_depth = 0,
      .flags = {.enable_internal_pullup = true, .allow_pd = false}};

  ESP_ERROR_CHECK(i2c_new_master_bus(&config, bus_handle));

  // The onboard PMU chip uses SCK -> 7, SDA -> 15
  i2c_master_bus_config_t pmu_bus_config = {
      .i2c_port = I2C_NUM_1,
      .sda_io_num = GPIO_NUM_15,
      .scl_io_num = GPIO_NUM_7,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .intr_priority = 0,
      .trans_queue_depth = 0,
      .flags = {.enable_internal_pullup = true, .allow_pd = false}};

  ESP_ERROR_CHECK(i2c_new_master_bus(&pmu_bus_config, bus_pmu_handle));
}
