#include "XPowersLib.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_sleep.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "hal/uart_types.h"
#include "portmacro.h"
#include "soc/clk_tree_defs.h"

#define BOARD_MODEM_PWR_PIN (41)
#define BOARD_MODEM_DTR_PIN (42)
#define BOARD_MODEM_RI_PIN (3)
#define BOARD_MODEM_RXD_PIN (4)
#define BOARD_MODEM_TXD_PIN (5)

void modem_init_power(XPowersAXP2101 &pmu) {
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) {
    // power cycle the modem on power cycles
    pmu.disableDC3();
    vTaskDelay(pdMS_TO_TICKS(200));
  }

  // set working voltage for modem
  pmu.setDC3Voltage(3'000);
  pmu.enableDC3();

  // power for the GPS
  pmu.setBLDO2Voltage(3'300);
  pmu.enableBLDO2();

  // disable ts pin detection to allow for charging
  pmu.disableTSPinMeasure();
}

esp_err_t modem_init_gpio() {
  // Modem exists via UART
  uart_config_t modem_uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 0,
      .source_clk = UART_SCLK_DEFAULT,
      .flags = {.allow_pd = 0, .backup_before_sleep = 0}};

  ESP_RETURN_ON_ERROR(uart_driver_install(UART_NUM_1, 2048, 0, 0, nullptr, 0));
  ESP_RETURN_ON_ERROR(uart_param_config(UART_NUM_1, &modem_uart_config));
  ESP_RETURN_ON_ERROR(uart_set_pin(UART_NUM_1, BOARD_MODEM_TXD_PIN,
                                   BOARD_MODEM_RXD_PIN, UART_PIN_NO_CHANGE,
                                   UART_PIN_NO_CHANGE));

  return ESP_OK;
}
