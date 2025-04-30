/**
 * The ULP has a timer interrupt, but all functions related are not implemented
 * in ESP-IDF.
 */

/* ulp_riscv_timer_utils.h */

#pragma once

#include "soc/rtc.h" // For rtc_clk_slow_freq_get_hz if available in ULP context
#include "soc/rtc_cntl_reg.h" // Include SoC register definitions
#include "soc/soc.h"          // For DR_REG_RTCCNTL_BASE if needed
#include <stdbool.h>
#include <stdint.h>

// --- Register Access Macros (Common in ESP-IDF) ---
// Ensure these are available or define them appropriately for ULP context
#ifndef REG_WRITE
#define REG_WRITE(_r, _v) (*(volatile uint32_t *)(_r)) = (_v)
#endif
#ifndef REG_READ
#define REG_READ(_r) (*(volatile uint32_t *)(_r))
#endif
#ifndef REG_SET_BIT
#define REG_SET_BIT(_r, _b) (*(volatile uint32_t *)(_r) |= (_b))
#endif
#ifndef REG_CLR_BIT
#define REG_CLR_BIT(_r, _b) (*(volatile uint32_t *)(_r) &= ~(_b))
#endif
#ifndef REG_SET_FIELD
#define REG_SET_FIELD(_r, _f, _v)                                              \
  (*(volatile uint32_t *)(_r)) =                                               \
      (((*(volatile uint32_t *)(_r)) & ~((_f##_V) << (_f##_S))) |              \
       (((_v) & (_f##_V)) << (_f##_S)))
#endif
#ifndef REG_GET_FIELD
#define REG_GET_FIELD(_r, _f)                                                  \
  (((*(volatile uint32_t *)(_r)) >> (_f##_S)) & (_f##_V))
#endif

// Standard RISC-V CSR Bits (Only mstatus needed now)
#define MSTATUS_MIE_BIT (1 << 3) // Machine Interrupt Enable bit in mstatus

#define ULP_CP_TIMER_REG RTC_CNTL_ULP_CP_TIMER_1_REG
#define ULP_CP_TIMER_SLP_CYCLE_MASK 0x0FFFFFFFUL // Mask for bits 0 through 27
#define ULP_CP_TIMER_SLP_CYCLE_SHIFT 0           // Field starts at bit 0

// Interrupt Enable/Status/Clear Registers (General RTC)
#define RTC_INT_ENA_REG RTC_CNTL_INT_ENA_REG
#define RTC_INT_ST_REG RTC_CNTL_INT_ST_REG
#define RTC_INT_CLR_REG RTC_CNTL_INT_CLR_REG

// The specific interrupt bits for the ULP_CP Timer within the general RTC
// registers
#define RTC_ULP_CP_TIMER_INT_ENA_BIT RTC_CNTL_ULP_CP_INT_ENA // Enable Bit
#define RTC_ULP_CP_TIMER_INT_ST_BIT RTC_CNTL_ULP_CP_INT_ST   // Status Bit
#define RTC_ULP_CP_TIMER_INT_CLR_BIT RTC_CNTL_ULP_CP_INT_CLR // Clear Bit

// The specific bit for the ULP Timer Interrupt (FIND THIS IN TRM!)
// It might be called RTC_CNTL_ULP_CP_INT_ENA / ST / CLR or similar.
// Let's *hypothesize* it's the main RTC timer bit for this example.
#define RTC_TIMER_INT_BIT                                                      \
  RTC_CNTL_MAIN_TIMER_INT_ENA // !!! GUESS - FIND ACTUAL BIT !!!

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calculates the number of RTC slow clock cycles for a given duration in
 * microseconds.
 *
 * @note This function needs to know the RTC slow clock frequency.
 *       It might be simpler to pass the pre-calculated cycle count if known.
 * @param duration_us Duration in microseconds.
 * @return Number of RTC slow clock cycles.
 */
static inline uint32_t ulp_riscv_timer_us_to_cycles(uint32_t duration_us) {
  // Get frequency (This function might not work directly in ULP context)
  // uint32_t slow_clk_hz = rtc_clk_slow_freq_get_hz(); // Needs checking
  // For simplicity/robustness in ULP, it might be better to use a fixed known
  // value or pass it from the main CPU via a shared variable. Let's assume a
  // common value like 150kHz (RTC_SLOW_CLK_FREQ_150K) for example:
  const uint32_t slow_clk_hz = 150000; // Example: Use 150kHz RC oscillator freq

  // Use 64-bit intermediate calculation to avoid overflow
  uint64_t cycles = ((uint64_t)duration_us * slow_clk_hz) / 1000000ULL;
  return (uint32_t)cycles;
}

// Define the mask and shift manually based on TRM info for the field
#define ULP_CP_TIMER_SLP_CYCLE_MASK 0x0FFFFFFFUL // Mask for bits 0 through 27
#define ULP_CP_TIMER_SLP_CYCLE_SHIFT 0           // Field starts at bit 0

/**
 * @brief Sets the ULP-CP timer period in RTC slow clock cycles.
 *        This determines how often the timer interrupt will fire.
 *
 * @param cycles Period in RTC slow clock cycles (only lower 28 bits used).
 */
static inline void ulp_riscv_timer_set_period_cycles(uint32_t cycles) {
  // Manual Read-Modify-Write
  uint32_t reg_val = REG_READ(ULP_CP_TIMER_REG);

  // Clear the existing field bits
  reg_val &= ~(ULP_CP_TIMER_SLP_CYCLE_MASK << ULP_CP_TIMER_SLP_CYCLE_SHIFT);

  // Set the new field bits (masking the input value just in case)
  reg_val |=
      ((cycles & ULP_CP_TIMER_SLP_CYCLE_MASK) << ULP_CP_TIMER_SLP_CYCLE_SHIFT);

  // Write the modified value back to the register
  REG_WRITE(ULP_CP_TIMER_REG, reg_val);
}

/**
 * @brief Sets the ULP-CP timer period in microseconds.
 *
 * @note Requires knowing the RTC slow clock frequency.
 * @param period_us Period in microseconds.
 */
static inline void ulp_riscv_timer_set_period_us(uint32_t period_us) {
  uint32_t cycles = ulp_riscv_timer_us_to_cycles(period_us);
  ulp_riscv_timer_set_period_cycles(cycles);
}

/**
 * @brief Clears the RTC timer interrupt status flag.
 *        MUST be called within the ISR to allow subsequent interrupts.
 *
 * @note Assumes RTC_TIMER_INT_BIT is the correct status/clear bit. VERIFY IN
 * TRM!
 */
static inline void ulp_riscv_timer_interrupt_clear(void) {
  // Writing 1 to the clear register bit usually clears the corresponding status
  // bit
  REG_WRITE(RTC_INT_CLR_REG, RTC_TIMER_INT_BIT);
}

/**
 * @brief Enables or disables the RTC timer interrupt source.
 *
 * @note Assumes RTC_TIMER_INT_BIT is the correct enable bit. VERIFY IN TRM!
 * @param enable True to enable, false to disable.
 */
static inline void ulp_riscv_timer_interrupt_enable_source(bool enable) {
  if (enable) {
    REG_SET_BIT(RTC_INT_ENA_REG, RTC_TIMER_INT_BIT);
  } else {
    REG_CLR_BIT(RTC_INT_ENA_REG, RTC_TIMER_INT_BIT);
  }
}

/**
 * @brief Sets the Machine Trap Vector (mtvec) register.
 *        This points the ULP core to the interrupt handler function.
 *
 * @param isr_func Pointer to the interrupt service routine function.
 *                 The function must be of type void (*)(void).
 */
static inline void ulp_riscv_isr_vector_init(void (*isr_func)(void)) {
  asm volatile("csrw mtvec, %0" ::"r"(isr_func));
}

/**
 * @brief Enables or disables global interrupts for the ULP-RISC-V core.
 *        Modifies the MIE bit in the MSTATUS (Machine Status) CSR.
 *
 * @param enable True to enable, false to disable.
 */
static inline void ulp_riscv_global_interrupt_enable(bool enable) {
  // This part remains the same, using standard CSR access
  uint32_t mstatus_val;
  asm volatile("csrr %0, mstatus" : "=r"(mstatus_val));
  if (enable) {
    mstatus_val |= MSTATUS_MIE_BIT;
  } else {
    mstatus_val &= ~MSTATUS_MIE_BIT;
  }
  asm volatile("csrw mstatus, %0" ::"r"(mstatus_val));
}

#ifdef __cplusplus
}
#endif
