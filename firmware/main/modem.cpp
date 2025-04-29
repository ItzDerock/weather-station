#include "XPowersLib.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_sleep.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include "hal/uart_types.h"
#include "portmacro.h"
#include "sim7080g_driver_esp_idf.h"
#include "soc/clk_tree_defs.h"

#define BOARD_MODEM_PWR_PIN GPIO_NUM_41
#define BOARD_MODEM_DTR_PIN GPIO_NUM_42
#define BOARD_MODEM_RI_PIN GPIO_NUM_3
#define BOARD_MODEM_RXD_PIN GPIO_NUM_4
#define BOARD_MODEM_TXD_PIN GPIO_NUM_5

static const char *TAG = "modem";

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

esp_err_t modem_init(XPowersAXP2101 &pmu, sim7080g_handle_t *sim7080g_handle) {
  modem_init_power(pmu);

  // init gpios
  ESP_RETURN_ON_ERROR(gpio_reset_pin(BOARD_MODEM_PWR_PIN), TAG,
                      "Failed to reset pin %d", BOARD_MODEM_PWR_PIN);
  ESP_RETURN_ON_ERROR(gpio_reset_pin(BOARD_MODEM_DTR_PIN), TAG,
                      "Failed to reset pin %d", BOARD_MODEM_DTR_PIN);
  ESP_RETURN_ON_ERROR(gpio_reset_pin(BOARD_MODEM_RI_PIN), TAG,
                      "Failed to reset pin %d", BOARD_MODEM_RI_PIN);

  ESP_RETURN_ON_ERROR(gpio_set_direction(BOARD_MODEM_PWR_PIN, GPIO_MODE_OUTPUT),
                      TAG, "Failed to set pin %d as output",
                      BOARD_MODEM_PWR_PIN);
  ESP_RETURN_ON_ERROR(gpio_set_direction(BOARD_MODEM_DTR_PIN, GPIO_MODE_OUTPUT),
                      TAG, "Failed to set pin %d as output",
                      BOARD_MODEM_DTR_PIN);
  ESP_RETURN_ON_ERROR(gpio_set_direction(BOARD_MODEM_RI_PIN, GPIO_MODE_INPUT),
                      TAG, "Failed to set pin %d as input", BOARD_MODEM_RI_PIN);

  // Pull PWRKEY for 1 second to start modem
  ESP_RETURN_ON_ERROR(gpio_set_level(BOARD_MODEM_PWR_PIN, 0), TAG,
                      "Failed to set pin %d low", BOARD_MODEM_PWR_PIN);
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_RETURN_ON_ERROR(gpio_set_level(BOARD_MODEM_PWR_PIN, 1), TAG,
                      "Failed to set pin %d high", BOARD_MODEM_PWR_PIN);
  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_RETURN_ON_ERROR(gpio_set_level(BOARD_MODEM_PWR_PIN, 0), TAG,
                      "Failed to set pin %d low", BOARD_MODEM_PWR_PIN);

  // Modem exists via UART
  sim7080g_uart_config_t uart_config = {.gpio_num_tx = BOARD_MODEM_TXD_PIN,
                                        .gpio_num_rx = BOARD_MODEM_RXD_PIN,
                                        .port_num = UART_NUM_1};

  sim7080g_mqtt_config_t mqtt_config = {.broker_url = MQTT_BROKER_URL,
                                        .username = MQTT_USERNAME,
                                        .client_id = MQTT_CLIENT_ID,
                                        .client_password = MQTT_CLIENT_PASSWORD,
                                        .port = MQTT_PORT};

  ESP_RETURN_ON_ERROR(
      sim7080g_config(sim7080g_handle, uart_config, mqtt_config), TAG,
      "Failed to configure modem handle");

  ESP_RETURN_ON_ERROR(sim7080g_init(sim7080g_handle), TAG,
                      "Failed to init modem handle");

  ESP_RETURN_ON_ERROR(sim7080g_connect_to_network_bearer(sim7080g_handle, APN),
                      TAG, "Failed to connect to network");

  ESP_RETURN_ON_ERROR(sim7080g_mqtt_connect_to_broker(sim7080g_handle), TAG,
                      "Failed to connect to MQTT broker");

  return ESP_OK;
}
