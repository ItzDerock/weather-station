#include <stdio.h>

#include <cstddef>

#define XPOWERS_CHIP_AXP2101

#include "XPowersLib.h"
#include "argentdata.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "hal/i2c_types.h"
#include "include/modem.hpp"
#include "include/sensors.hpp"
#include "lps22.h"
#include "ltr390uv.h"
#include "shtc3.h"
#include "sim7080g_driver_esp_idf.h"
#include "soc/clk_tree_defs.h"
#include "ulp_riscv.h"

#include "config.h"

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
  XPowersPMU pmu;
  pmu.begin(bus_pmu_handle, AXP2101_SLAVE_ADDRESS);

  // power on the sensors
  pmu.setDC5Voltage(3'300);
  pmu.enableDC5();

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

  // start modem
  sim7080g_handle_t sim7080g_handle;
  esp_err_t modem_err = modem_init(pmu, &sim7080g_handle);

  // data
  sensor_data_t sensor_data;
  sensor_data.argent = &argentData;

  // while (true) {
  // test argentdata
  argentdata_read_values(&argentData);
  ESP_LOGI(taskName, "Wind speed: %f mph", argentData.wind_speed);
  ESP_LOGI(taskName, "Wind speed (gust): %f mph", argentData.wind_speed_gust);
  ESP_LOGI(taskName, "Rainfall: %f in/min", argentData.rainfall);

  // test lps22
  if (lpsok == ESP_OK) {
    float pressure_hpa, lps_temperature;

    lps22_read_pressure(lps22_handle, &pressure_hpa);
    // Not actually used because of ±1.5 std.err.
    lps22_read_temperature(lps22_handle, &lps_temperature);

#if USE_IMPERIAL
    float pressure_inhg = pressure_hpa * 0.02953f;
    // Not actually used
    float lps_temperature_f = lps_temperature * 9.0f / 5.0f + 32.0f;
    ESP_LOGI(taskName, "Pressure: %.2f inHg", pressure_inhg);
    ESP_LOGI(taskName, "Temperature (LPS22): %.2f °F", lps_temperature_f);

    sensor_data.pressure = std::optional<float>(pressure_inhg);
#if LOG_METRIC
    ESP_LOGI(taskName, "Pressure: %.2f hPa", pressure_hpa);
    ESP_LOGI(taskName, "Pressure: %.2f hPa", lps_temperature);
#endif
#else
    ESP_LOGI(taskName, "Pressure: %.2f hPa", pressure_hpa);
    // Not actually used
    ESP_LOGI(taskName, "Temperature (LPS22): %.2f °C", lps_temperature);

    sensor_data.pressure = std::optional<float>(pressure_hpa);
#endif
  } else {
    sensor_data.pressure = std::nullopt;
    ESP_LOGE(taskName, "Failed to initialize LPS22 sensor: %s",
             esp_err_to_name(lpsok));
  }

  // test shtc3
  // CSE = Clock Stretching Enabled
  // NM = Normal Mode (as opposed to low power mode)
  float temperature, humidity;
  esp_err_t shtc3_err =
      shtc3_get_th(shtc3_handle, SHTC3_REG_T_CSE_NM, &temperature, &humidity);
  if (shtc3_err == ESP_OK) {
    // ESP_LOGI(taskName, "ARE WE USING IMPERIAL: %d", USE_IMPERIAL)
#if USE_IMPERIAL
    float temperature_f = temperature * 9.0f / 5.0f + 32.0f;

    sensor_data.temperature =
      shtc3_err == ESP_OK ? std::optional<float>(temperature_f) : std::nullopt;

    ESP_LOGI(taskName, "Temperature: %.2f F", temperature_f);
#if LOG_METRIC
    ESP_LOGI(taskName, "Temperature: %.2f C", temperature);
#endif
#else
    sensor_data.temperature =
      shtc3_err == ESP_OK ? std::optional<float>(temperature) : std::nullopt;

    ESP_LOGI(taskName, "Temperature: %.2f C", temperature);
#endif
    sensor_data.humidity =
      shtc3_err == ESP_OK ? std::optional<float>(humidity) : std::nullopt;

    ESP_LOGI(taskName, "Humidity: %.2f %%", humidity);
  } else {
    sensor_data.temperature = std::nullopt;
    sensor_data.humidity = std::nullopt;

    ESP_LOGE(taskName, "Failed to initialize shtc3 sensor: %s",
             esp_err_to_name(lpsok));
  }

  // test ltr390
  float uvi;
  esp_err_t ltrerr = ltr390uv_get_ultraviolet_index(dev_hdl, &uvi);
  if (ltrerr != ESP_OK) {
    ESP_LOGE(taskName, "ltr390uv device read failed (%s)",
             esp_err_to_name(ltrerr));

    sensor_data.uv_index = std::nullopt;
  } else {
    ESP_LOGI(taskName, "Ultraviolet index: %f", uvi);
    sensor_data.uv_index = std::optional<float>(uvi);
  }

  // read battery information
  if (pmu.isBatteryConnect()) {
    uint16_t voltage = pmu.getBattVoltage();
    int percent = pmu.getBatteryPercent();

    ESP_LOGI(taskName, "Battery voltage: %d mV", voltage);
    ESP_LOGI(taskName, "Battery percent: %d %%", percent);

    sensor_data.battery_voltage = std::optional<uint16_t>(voltage);
    sensor_data.battery_est_percentage = std::optional<int>(percent);
  } else {
    ESP_LOGE(taskName, "Battery not connected");
    sensor_data.battery_voltage = std::nullopt;
    sensor_data.battery_est_percentage = std::nullopt;
  }

  // read cellular information
  int8_t rssi = 0;
  uint8_t ber = 0;

  esp_err_t cellular_err =
      sim7080g_check_signal_quality(&sim7080g_handle, &rssi, &ber);

  if (cellular_err == ESP_OK) {
    ESP_LOGI(taskName, "Signal quality: %d dBm", rssi);
    ESP_LOGI(taskName, "Bit error rate: %d", ber);

    sensor_data.rssi = std::optional<int8_t>(rssi);
    sensor_data.ber = std::optional<uint8_t>(ber);
  } else {
    ESP_LOGE(taskName, "Failed to check signal quality: %s",
             esp_err_to_name(cellular_err));

    sensor_data.rssi = std::nullopt;
    sensor_data.ber = std::nullopt;
  }

  // publish to MQTT
  esp_err_t mqtt_err = mqtt_send_data(&sim7080g_handle, sensor_data);

  if (mqtt_err != ESP_OK) {
    ESP_LOGE(taskName, "Failed to send data to MQTT: %s",
             esp_err_to_name(mqtt_err));
  } else {
    ESP_LOGI(taskName, "Data sent to MQTT successfully");
  }

  ESP_LOGI(taskName, "Entering deep sleep...");

  // Deep sleep for 10 minutes
  // Hard coded. If changed, update the ULP config.h's `REPORT_PERIOD`
  modem_power_off(pmu);
  pmu.disableDC5();
  esp_sleep_enable_timer_wakeup(static_cast<long long>(10 * 60) * 1000000);
  vTaskDelay(pdMS_TO_TICKS(1'000));
  esp_deep_sleep_start();

  // }
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
