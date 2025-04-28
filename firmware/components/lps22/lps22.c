#include "include/lps22.h"
#include "FreeRTOSConfig.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_check.h"
#include "esp_log.h"
#include "portmacro.h"
#include <stdio.h>

static const int I2C_MASTER_TIMEOUT = 200 / portTICK_PERIOD_MS;
static const char *TAG = "lps22";

static esp_err_t write_lps22_register(i2c_master_dev_handle_t dev,
                                      uint8_t reg_addr, uint8_t data) {
  uint8_t write_buf[2] = {reg_addr, data};
  esp_err_t err = i2c_master_transmit(dev, write_buf, sizeof(write_buf),
                                      I2C_MASTER_TIMEOUT);
  if (err != ESP_OK) {
    ESP_LOGE("LPS22_WRITE", "Failed to write 0x%02X to register 0x%02X: %s",
             data, reg_addr, esp_err_to_name(err));
  }
  return err;
}

esp_err_t lps22_init(i2c_master_bus_handle_t bus_handle,
                     i2c_master_dev_handle_t *ret_device, uint8_t address) {
  i2c_device_config_t lps22_config = {
      .device_address = address,
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .scl_speed_hz = LPS22_SCL_SPEED_HZ,
  };

  esp_err_t err =
      i2c_master_bus_add_device(bus_handle, &lps22_config, ret_device);

  ESP_RETURN_ON_ERROR(err, TAG, "Failed to add LPS22 device at address 0x%02X",
                      address);

  // Set ODR (Output Data Rate) to 10 Hz
  uint8_t ctrl_reg1_value = 0x20;
  esp_err_t write_err =
      write_lps22_register(*ret_device, 0x10, ctrl_reg1_value);

  if (write_err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set ODR: %s", esp_err_to_name(write_err));
    return write_err;
  }

  return ESP_OK;
}

esp_err_t lps22_read_pressure(i2c_master_dev_handle_t handle,
                              float *pressure_hpa) {
  // Pressure is stored into 3 registers that need to be combined
  uint8_t start_reg = LPS22_PRESS_OUT_XL_REG;
  uint8_t read_buffer[3];

  // Read the 3 registers
  // LPS22 has auto-incrementing registers, so we can read all 3 at once
  esp_err_t err = i2c_master_transmit_receive(
      handle, &start_reg, 1, read_buffer, 3, I2C_MASTER_TIMEOUT);

  ESP_RETURN_ON_ERROR(err, TAG, "Failed to read data from LPS22 sensor!");

  // read_buffer[0] = LPS22_PRESS_OUT_XL_REG
  // read_buffer[1] = LPS22_PRESS_OUT_L_REG
  // read_buffer[2] = LPS22_PRESS_OUT_H_REG
  uint32_t pressure_raw = ((uint32_t)read_buffer[2] << 16) |
                          ((uint32_t)read_buffer[1] << 8) |
                          (uint32_t)read_buffer[0];

  // Convert to pressure in hPa
  *pressure_hpa = (float)pressure_raw / 4096.0f;

  ESP_LOGD("lps22", "Read raw pressure: %lu -> %.2f hPa", pressure_raw,
           *pressure_hpa);

  return ESP_OK;
}

esp_err_t lps22_read_temperature(i2c_master_dev_handle_t handle,
                                 float *temperature) {
  uint8_t start_reg = LPS22_TEMP_OUT_L_REG;
  uint8_t read_buffer[2];

  esp_err_t err = i2c_master_transmit_receive(
      handle, &start_reg, 1, read_buffer, 2, I2C_MASTER_TIMEOUT);

  ESP_RETURN_ON_ERROR(err, TAG, "Failed to read temperature from LPS22");

  uint16_t temperature_raw =
      ((uint16_t)read_buffer[1] << 8) | (uint16_t)read_buffer[0];

  *temperature = (float)temperature_raw / 100;

  return ESP_OK;
}
