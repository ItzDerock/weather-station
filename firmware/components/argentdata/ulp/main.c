/**
 * The ULP handles counting anemometer ticks.
 * This allows the main CPU to go into deep sleep and conserve battery, while
 * still collecting continuous data. Required because wind speed gusts are the
 * typically defined as the highest 3-second wind speed average over a 10-min
 * time interval.
 */

#include "config.h"
#include "esp_attr.h"
#include "hal/adc_types.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc.h"
#include "ulp_riscv.h"
#include "ulp_riscv_adc_ulp_core.h"
#include "ulp_riscv_gpio.h"
#include "ulp_riscv_interrupt.h"
#include "ulp_riscv_register_ops.h"
#include "ulp_riscv_timer.h"
#include "ulp_riscv_utils.h"
#include "ulp_vars.h"
#include <stdint.h>

// There's no hardware debouncing, so must do this ourselves
#define DEBOUNCE_CYCLES 500

/* Wind speed. Need a moving 3-second window. */

// Tick counter for the current 1-second interval
RTC_DATA_ATTR uint32_t anem_ticks_current_second = 0;

// Tick counts for the previous three seconds (sliding window)
RTC_DATA_ATTR uint32_t anem_ticks_sec_minus_1 = 0;
RTC_DATA_ATTR uint32_t anem_ticks_sec_minus_2 = 0;
RTC_DATA_ATTR uint32_t anem_ticks_sec_minus_3 = 0;

// Maximum sum of ticks observed in any 3-second window since last reset
RTC_DATA_ATTR uint32_t max_anem_ticks_3_sec = 0;

// Total anemometer ticks since last main CPU reset (for average speed)
RTC_DATA_ATTR uint32_t anem_ticks_total = 0;

/* Other sensors */

// Total rain gauge count
RTC_DATA_ATTR uint32_t rain_gauge_count = 0;
RTC_DATA_ATTR uint8_t initialized = 0;

// Direction of the wind
RTC_DATA_ATTR uint32_t wind_direction[10];
RTC_DATA_ATTR uint8_t wind_direction_index = 0;
RTC_DATA_ATTR uint8_t wakeup_index = 0;

// Debug flags (0 = ok)
RTC_DATA_ATTR int8_t error_flags = 0;
#define ULP_ERR_FLAG_ILLEGAL_INSN (1 << 1)
#define ULP_ERR_FLAG_BUS_ERROR (1 << 2)
#define ULP_ERR_FLAG_UNKNOWN_IRQ (1 << 3)

/* clang-format off */
// Interrupt flags
// Stored in Q1 register
#define ULP_RISCV_TIMER_INT                         (1 << 0U)   /* Internal Timer Interrupt */
#define ULP_RISCV_EBREAK_ECALL_ILLEGAL_INSN_INT     (1 << 1U)   /* EBREAK, ECALL or Illegal instruction */
#define ULP_RISCV_BUS_ERROR_INT                     (1 << 2U)   /* Bus Error (Unaligned Memory Access) */
#define ULP_RISCV_PERIPHERAL_INTERRUPT              (1 << 31U)  /* RTC Peripheral Interrupt */
#define ULP_RISCV_INTERNAL_INTERRUPT                (ULP_RISCV_TIMER_INT | ULP_RISCV_EBREAK_ECALL_ILLEGAL_INSN_INT | ULP_RISCV_BUS_ERROR_INT)
/* clang-format on */

static void anemometer_isr() {
  anem_ticks_current_second++; // Count for the current 1-sec interval
  anem_ticks_total++;          // Accumulate total count
}

static void rain_gauge_isr() { rain_gauge_count++; }

static void read_direction() {
  int32_t result =
      ulp_riscv_adc_read_channel(WIND_VANE_ADC_UNIT, WIND_VANE_ADC_CHANNEL);

  wind_direction[wind_direction_index] = result;
  wind_direction_index = (wind_direction_index + 1) % WIND_VANE_KEEP_N;
}

// wakeup period defined in argentdata.h
// currently every 1 second
static void ulp_timer_isr() {
  // Capture the count from the second that just ended
  uint32_t ticks_finished_second = anem_ticks_current_second;

  // Reset the counter for the next second (t+1)
  anem_ticks_current_second = 0;

  // Shift the sliding window variables
  // (Oldest count drops out)
  anem_ticks_sec_minus_3 = anem_ticks_sec_minus_2; // t-3 = t-2
  anem_ticks_sec_minus_2 = anem_ticks_sec_minus_1; // t-2 = t-1
  anem_ticks_sec_minus_1 = ticks_finished_second;  // Newest count enters

  // Calculate the sum over the last 3 seconds
  // Note: At startup, the first couple of sums will be incomplete until
  // the window fills. This is acceptable because it'll run 24/7.
  uint32_t current_3_sec_sum =
      anem_ticks_sec_minus_1 + anem_ticks_sec_minus_2 + anem_ticks_sec_minus_3;

  // Update the maximum 3-second sum if needed
  if (current_3_sec_sum > max_anem_ticks_3_sec) {
    max_anem_ticks_3_sec = current_3_sec_sum;
  }

  // Every nth wakeup period, measure wind direction
  error_flags = (int8_t)wakeup_index;
  if (wakeup_index++ == 0) {
    error_flags++;
    read_direction();
  }

  if (wakeup_index >= WIND_VANE_MEASUREMENT_INTERVAL) {
    wakeup_index = 0;
  }

  // instruct CPU to wake up again
  ulp_riscv_timer_interrupt_clear();
}

/**
 * This function overrides the weak default implementation.
 * It is called by the assembly vector code (ulp_riscv_vectors.S)
 * https://github.com/espressif/esp-idf/blob/465b159cd8771ffab6be70c7675ecf6705b62649/components/ulp/ulp_riscv/ulp_core/ulp_riscv_interrupt.c#L16
 *
 * Default interrupt handler has "// TODO" for timer interrupts which we need.
 */
void _ulp_riscv_interrupt_handler(uint32_t q1) {
  error_flags++;
  // cause_q1 contains the IRQ number (0 for timer, 31 for peripheral, etc.)

  // IRQ 0: Internal Timer Interrupt
  if (q1 & ULP_RISCV_INTERNAL_INTERRUPT) {
    ulp_timer_isr();
  }

  // IRQ 31: RTC Peripheral Interrupt
  if (q1 & ULP_RISCV_PERIPHERAL_INTERRUPT) {
    uint32_t rtc_io_status =
        REG_GET_FIELD(RTC_GPIO_STATUS_REG, RTC_GPIO_STATUS_INT);

    if (rtc_io_status & (1U << ANEMOMETER_GPIO)) {
      anemometer_isr();
    }

    if (rtc_io_status & (1U << RAIN_GAUGE_GPIO)) {
      rain_gauge_isr();
    }

    REG_SET_FIELD(RTC_GPIO_STATUS_W1TC_REG, RTC_GPIO_STATUS_INT_W1TC,
                  rtc_io_status);
  } else if (q1 & (1U << 1)) { // IRQ 1: EBREAK/ECALL/Illegal Instruction
    error_flags |= ULP_ERR_FLAG_ILLEGAL_INSN;
  } else if (q1 & (1U << 2)) { // IRQ 2: Bus Error
    error_flags |= ULP_ERR_FLAG_BUS_ERROR;
  } else {
    // Unknown interrupt cause
    error_flags |= ULP_ERR_FLAG_UNKNOWN_IRQ;
  }
}

int main(void) {
  ulp_timer_isr(); // run immediately to fill data at boot.

  // ulp_riscv_isr_vector_init(ulp_timer_isr);
  ulp_riscv_timer_set_period_us(ULP_WAKEUP_PERIOD);
  ulp_riscv_timer_interrupt_enable_source(true);
  ulp_riscv_global_interrupt_enable(true);

  while (1) {
    asm volatile("wfi"); // Wait For Interrupt
    error_flags++;
  }

  return 0;
}
