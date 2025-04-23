#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>
#define XPOWERS_CHIP_AXP2101
#define TINY_GSM_RX_BUFFER 1024
#define TINY_GSM_MODEM_SIM7080
#include <TinyGsmClient.h>

namespace cellular {

#define AT_CMD_FORMAT_ERROR -2
#define MODEM_DEFAULT_TIMEOUT 1000

enum CellError {
  GENERIC_ERROR = -1,
  NO_SIM_CARD = -2,
  GPRS_NOT_CONNECTED = -3,
  BEARER_NOT_ACTIVATED = -4,
};

extern TinyGsm modem;

/**
 * Checks if the board is able to communicate with the modem.
 * does NOT check for registration status, use isRegistered() for that.
 */
bool isConnected();

int initialize();
int initializeAPN();

/**
 * Checks if the board has successfully registered to the network
 */
bool isRegistered();

int connectMQTT();

/**
 * @brief Formats and sends an AT command, then waits for a response.
 *
 * @param modem The modem instance (e.g., TinyGsm). Adjust type if needed.
 * @param buffer A character buffer to store the formatted command.
 * @param bufferSize The size of the buffer.
 * @param timeout The maximum time in milliseconds to wait for a response.
 * @param format The format string for the AT command (like printf).
 * @param ... Variable arguments for the format string.
 * @return int The result from modem.waitResponse() (typically 1 for OK, 0 for
 * ERROR, -1 for timeout), or AT_CMD_FORMAT_ERROR (-2) if vsnprintf failed.
 */
int sendATCommand(TinyGsm &modem, char *buffer, size_t bufferSize,
                  uint32_t timeout, const char *format, ...);

} // namespace cellular
