#pragma once

#include "driver/i2c_types.h"
#include "esp_err.h"

#define LPS22_PRESS_OUT_H_REG 0x2A
#define LPS22_PRESS_OUT_L_REG 0x29
#define LPS22_PRESS_OUT_XL_REG 0x28
#define LPS22_DEFAULT_ADDR 0x5D
#define LPS22_SCL_SPEED_HZ 100000 // 100kHz

/**
 * Creates a new I2C device handle for the LPS22.
 * Address should default to LPS22_DEFAULT_ADDR.
 */
esp_err_t lps22_init(i2c_master_bus_handle_t bus_handle,
                     i2c_master_dev_handle_t *ret_device, uint8_t address);

/**
 * Reads the pressure and returns it in the pressure_hpa pointer.
 * The pressure is in hPa.
 */
esp_err_t lps22_read_data(i2c_master_dev_handle_t handle, float *pressure_hpa);
