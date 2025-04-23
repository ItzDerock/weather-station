#include "include/sensors/Anemometer.h"

// This is singleton since we need quick access to count inside the ISR.
Anemometer *Anemometer::_instance = nullptr;

Anemometer::Anemometer(uint8_t pin)
    : _pin(pin), _pulseCount(0), _lastReadTime(0), _windSpeedMPH(0.0) {
  // Store the instance pointer (assuming only one instance)
  _instance = this;
}

Anemometer::~Anemometer() {
  // Cleanup: Detach the interrupt when the object is destroyed
  if (digitalPinToInterrupt(_pin) != NOT_AN_INTERRUPT) {
    detachInterrupt(digitalPinToInterrupt(_pin));
  }
}

void Anemometer::begin() {
  // Ensure pin is valid for interrupts
  if (digitalPinToInterrupt(_pin) == NOT_AN_INTERRUPT) {
    // Handle error - perhaps log to Serial or set an error flag
    // For now, we'll just prevent attaching the interrupt
    return;
  }

  // Set pin mode with internal pull-up resistor
  pinMode(_pin, INPUT_PULLUP);

  // Reset counters and time
  _pulseCount = 0;
  _windSpeedMPH = 0.0;
  _lastReadTime = millis(); // Initialize time

  // Attach the interrupt handler
  // Trigger on FALLING edge (pin goes from HIGH due to pull-up to LOW when
  // switch closes)
  attachInterrupt(digitalPinToInterrupt(_pin), pulseISR, FALLING);
}

void Anemometer::update() {
  unsigned long currentTime = millis();
  unsigned long timeElapsed = currentTime - _lastReadTime;

  // Calculate speed only if a reasonable time has passed (e.g., >= 1000ms)
  // to avoid division by zero or noisy readings on very short intervals.
  if (timeElapsed >= 1000) {
    // --- Critical section ---
    // Disable interrupts briefly to safely read and reset the pulse count
    // This prevents the ISR from modifying _pulseCount while we read it.
    noInterrupts();
    unsigned long count = _pulseCount;
    _pulseCount = 0; // Reset counter for the next interval
    interrupts();
    // --- End Critical section ---

    // Calculate frequency in Hz
    // (float) cast ensures floating point division
    float frequency = (float)count / (timeElapsed / 1000.0);

    // Calculate wind speed
    _windSpeedMPH = frequency * ANEMOMETER_MPH_PER_HZ;

    // Update the last read time
    _lastReadTime = currentTime;
  }
  // else: Not enough time has passed, _windSpeedMPH retains its previous value
}

float Anemometer::getWindSpeedMPH() const { return _windSpeedMPH; }

float Anemometer::getWindSpeedKmh() const {
  // Convert MPH to km/h (1 MPH = 1.60934 km/h)
  // Alternatively, use the direct factor: _windSpeedMPH /
  // ANEMOMETER_MPH_PER_HZ * ANEMOMETER_KMH_PER_HZ
  return _windSpeedMPH * 1.60934;
}

// --- Static ISR Implementation ---
// This function is called by the hardware interrupt
#ifdef ESP32
void IRAM_ATTR Anemometer::pulseISR() {
#else
void Anemometer::pulseISR() {
#endif
  // Check if the instance exists (safety measure)
  if (_instance != nullptr) {
    // Increment the pulse count of the single instance
    // This needs to be as fast as possible!
    _instance->_pulseCount++;
  }
}
