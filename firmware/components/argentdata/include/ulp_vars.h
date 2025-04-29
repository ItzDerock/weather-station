#pragma once
#include <stdint.h>

/**
 * @brief Tick counter for the current 1-second interval.
 * Reset at the start of each second by the ULP.
 */
extern uint32_t ulp_anem_ticks_current_second;

/**
 * @brief Tick counts for the second ending 1 second ago.
 */
extern uint32_t ulp_anem_ticks_sec_minus_1;

/**
 * @brief Tick counts for the second ending 2 seconds ago.
 */
extern uint32_t ulp_anem_ticks_sec_minus_2;

/**
 * @brief Tick counts for the second ending 3 seconds ago.
 * Used for the 3-second moving window calculation.
 */
extern uint32_t ulp_anem_ticks_sec_minus_3;

/**
 * @brief Maximum sum of ticks observed in any 3-second window since last reset.
 * Represents the peak gust speed interval.
 */
extern uint32_t ulp_max_anem_ticks_3_sec;

/**
 * @brief Total anemometer ticks since last main CPU reset.
 * Used for calculating the average wind speed over longer periods.
 */
extern uint32_t ulp_anem_ticks_total;

/**
 * @brief Total count from the rain gauge sensor since last reset.
 */
extern uint32_t ulp_rain_gauge_count;

/**
 * @brief Flag indicating whether the ULP variables have been initialized.
 * Typically set by the main CPU after setting initial values.
 */
extern uint8_t ulp_initialized;

/**
 * @brief Buffer to store recent wind direction readings (e.g., ADC values).
 * Size 10 allows for averaging or trend analysis.
 */
extern uint32_t ulp_wind_direction[10];

/**
 * @brief Current index for writing into the ulp_wind_direction buffer.
 * Wraps around using modulo arithmetic.
 */
extern uint8_t ulp_wind_direction_index;

/**
 * @brief Index potentially related to ULP wakeup conditions or sequencing.
 * (Exact purpose depends on the ULP program logic).
 */
extern uint8_t ulp_wakeup_index;

/**
 * @brief Flags indicating errors encountered by the ULP coprocessor.
 * A value of 0 indicates no errors. Bits are set according to definitions
 * below.
 */
extern int8_t ulp_error_flags;

/**
 * @brief Error flag bit: ULP encountered an illegal instruction.
 */
#define ULP_ERR_FLAG_ILLEGAL_INSN (1 << 1)

/**
 * @brief Error flag bit: ULP encountered a bus error (e.g., invalid memory
 * access).
 */
#define ULP_ERR_FLAG_BUS_ERROR (1 << 2)

/**
 * @brief Error flag bit: ULP encountered an unknown or unexpected interrupt.
 */
#define ULP_ERR_FLAG_UNKNOWN_IRQ (1 << 3)
