/**
 * The ULP handles counting anemometer ticks.
 * This allows the main CPU to go into deep sleep and conserve battery, while
 * still collecting continuous data. Required because wind speed gusts are the
 * typically defined as the highest 3-second wind speed average over a 10-min
 * time interval.
 */

#include "esp_attr.h"
#include "esp_err.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc.h"
#include "ulp_riscv_gpio.h"
#include "ulp_riscv_utils.h"
#include <stdint.h>

// Must be RTC GPIOs.
#define ULP_WAKEUP_PERIOD 60 /* 1 minute */
#define ANEMOMETER_GPIO 11
#define RAIN_GAUGE_GPIO 12

// There's no hardware debouncing, so must do this ourselves
#define DEBOUNCE_CYCLES 50

// Current count for this period (between timer wakeups)
RTC_DATA_ATTR uint32_t current_anemometer_count = 0;
// Total count until main CPU resets (for 10-min averages)
RTC_DATA_ATTR uint32_t total_anemometer_count = 0;
// The maximum count since last main CPU Reset (for gusts)
RTC_DATA_ATTR uint32_t max_anemometer_count = 0;
// Total rain gauge count
RTC_DATA_ATTR uint32_t rain_gauge_count = 0;
RTC_DATA_ATTR uint8_t ulp_initialized = 0;

// Debug flags (0 = ok)
RTC_DATA_ATTR int8_t ulp_error_flags = 0;
#define ULP_ERR_FLAG_ILLEGAL_INSN (1 << 1)
#define ULP_ERR_FLAG_BUS_ERROR (1 << 2)
#define ULP_ERR_FLAG_UNKNOWN_IRQ (1 << 3)

static void anemometer_isr() { current_anemometer_count++; }
static void rain_gauge_isr() { rain_gauge_count++; }

// wakeup period defined in argentdata.h
static void ulp_timer_isr() {
  if (current_anemometer_count > max_anemometer_count) {
    max_anemometer_count = current_anemometer_count;
  }

  current_anemometer_count = 0;
}

/**
 * This function overrides the weak default implementation.
 * It is called by the assembly vector code (ulp_riscv_vectors.S)
 * https://github.com/espressif/esp-idf/blob/465b159cd8771ffab6be70c7675ecf6705b62649/components/ulp/ulp_riscv/ulp_core/ulp_riscv_interrupt.c#L16
 *
 * Default interrupt handler has "// TODO" for timer interrupts which we need.
 */
void _ulp_riscv_interrupt_handler(uint32_t cause_q1) {
  // cause_q1 contains the IRQ number (0 for timer, 31 for peripheral, etc.)

  if (cause_q1 == 0) { // IRQ 0: Internal Timer Interrupt
    ulp_timer_isr();
  } else if (cause_q1 & (1U << 31)) { // IRQ 31: RTC Peripheral Interrupt
    uint32_t rtc_io_status =
        REG_GET_FIELD(RTC_GPIO_STATUS_REG, RTC_GPIO_STATUS_INT);

    if (rtc_io_status & (1U << ANEMOMETER_GPIO)) {
      anemometer_isr();

      // Clear the specific GPIO interrupt status bit
      WRITE_PERI_REG(RTC_GPIO_STATUS_W1TC_REG, (1U << ANEMOMETER_GPIO));
    }

    if (rtc_io_status & (1U << RAIN_GAUGE_GPIO)) {
      rain_gauge_isr();

      // Clear the specific GPIO interrupt status bit
      WRITE_PERI_REG(RTC_GPIO_STATUS_W1TC_REG, (1U << RAIN_GAUGE_GPIO));
    }

    // Ignore other ISRs

  } else if (cause_q1 & (1U << 1)) { // IRQ 1: EBREAK/ECALL/Illegal Instruction
    ulp_error_flags |= ULP_ERR_FLAG_ILLEGAL_INSN;
  } else if (cause_q1 & (1U << 2)) { // IRQ 2: Bus Error
    ulp_error_flags |= ULP_ERR_FLAG_BUS_ERROR;
  } else {
    // Unknown interrupt cause
    ulp_error_flags |= ULP_ERR_FLAG_UNKNOWN_IRQ;
  }
}

int main(void) {
  // Our software debounce.
  // When ULP is awake, interrupts are dropped.
  ulp_riscv_delay_cycles(DEBOUNCE_CYCLES);

  // halts and waits for next interrupt
  return 0;
}
