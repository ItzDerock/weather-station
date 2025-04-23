
// ulp_riscv.S - ESP32-S3 RISC-V ULP Pulse Counter Example

#include "soc/soc.h"
#include "soc/rtc_io_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/gpio_reg.h" // Needed for direct GPIO read on S3 ULP

// --- Configuration ---
// Define the GPIO pin the anemometer is connected to.
// IMPORTANT: This MUST be an RTC GPIO pin! Check ESP32-S3 datasheet.
// Example: GPIO10 is RTC_GPIO10
.set PULSE_PIN_NUM, 10

// --- RTC Slow Memory Data ---
// These labels are accessible from the main CPU sketch via 'ulp_variable_name'
.bss
.global ulp_pulse_count
ulp_pulse_count: .long 0 // Reserve 4 bytes (32 bits) for the count

.global ulp_last_pin_state
ulp_last_pin_state: .long 0 // Reserve 4 bytes for the last state

.global ulp_debounce_counter // Optional: for simple debouncing
ulp_debounce_counter: .long 0
.set DEBOUNCE_THRESHOLD, 2 // Require pin state to be stable for ~2 ULP cycles

// --- ULP Code ---
.text
.global entry
entry:
    // --- Read Current Pin State ---
    // For S3 RISC-V, we often read the main GPIO input register
    // Make sure the pin is configured as RTC IO and input in the main code
    // Load the GPIO input register address into t0
    li t0, DR_REG_GPIO_BASE // Base address of GPIO registers
    lw t1, GPIO_IN_REG_OFFSET(t0) // Read GPIO_IN_REG (offset 0x3C) into t1

    // Isolate the bit corresponding to our PULSE_PIN_NUM
    srli t2, t1, PULSE_PIN_NUM // Shift right so our pin's bit is the LSB
    andi t2, t2, 1           // Mask to get just the LSB (0 or 1) - t2 is current state

    // --- Load Previous State and Count ---
    // Load address of ulp_last_pin_state into t0
    la t0, ulp_last_pin_state
    lw t1, 0(t0) // Load the value of ulp_last_pin_state into t1 (last state)

    // Load address of ulp_pulse_count into t0
    la t0, ulp_pulse_count
    lw t3, 0(t0) // Load the value of ulp_pulse_count into t3 (current count)

    // --- Edge Detection ---
    // Compare current state (t2) with last state (t1)
    beq t1, t2, state_unchanged // If they are equal, jump to state_unchanged

    // --- State Changed ---
    // Optional: Simple Debounce - Reset debounce counter
    la t0, ulp_debounce_counter
    li t4, 0
    sw t4, 0(t0) // Reset debounce counter

    // Store the new potential state (t2) as the last state immediately
    la t0, ulp_last_pin_state
    sw t2, 0(t0) // Store current state (t2) into ulp_last_pin_state

    // We only increment on a RISING edge (last=0, current=1)
    // If last state (t1) was 1, it was a falling edge, so don't count
    bne t1, zero, state_changed_no_increment // If t1 != 0, jump

    // --- Rising Edge Detected ---
    // Increment the pulse count
    addi t3, t3, 1 // Increment count (t3)

    // Store the updated pulse count back to RTC memory
    la t0, ulp_pulse_count
    sw t3, 0(t0) // Store incremented count (t3) into ulp_pulse_count

state_changed_no_increment:
    // Jump to the end/halt instruction
    j end_ulp

state_unchanged:
    // --- Optional: Debounce Logic ---
    // If state hasn't changed, increment debounce counter
    la t0, ulp_debounce_counter
    lw t4, 0(t0) // Load debounce counter
    addi t4, t4, 1 // Increment
    // Check if debounce threshold is met (optional - can remove if no debounce needed)
    li t5, DEBOUNCE_THRESHOLD
    blt t4, t5, debounce_not_met // If counter < threshold, skip storing state
    // Debounce threshold met - ensure last_pin_state matches current stable state
    la t0, ulp_last_pin_state
    sw t2, 0(t0) // Store current stable state (t2)
debounce_not_met:
    // Store updated debounce counter
    la t0, ulp_debounce_counter
    sw t4, 0(t0) // Store debounce counter

end_ulp:
    // --- Halt the ULP ---
    // The ULP will be woken up again by its timer (configured in main code)
    // or by the main CPU.
    // Use ebreak (environment break) which acts like halt for ULP
    ebreak // Halt ULP execution

    // --- Loop (Alternative - Less Power Efficient) ---
    // If you wanted the ULP to loop internally instead of halting:
    // delay: // Simple delay loop (adjust count for desired delay)
    //  li t5, 1000 // Adjust loop count
    // delay_loop:
    //  addi t5, t5, -1
    //  bnez t5, delay_loop
    // j entry // Jump back to the beginning
