#include "include/sensors/UVI.h"
#include <Arduino.h>

// Reference sensitivity from datasheet: 2300 Counts/UVI @ Gain 18x, 20-bit (400ms)
const float REFERENCE_SENSITIVITY = 2300.0f;
const ltr390_gain_t REFERENCE_GAIN = LTR390_GAIN_18;
const ltr390_resolution_t REFERENCE_RESOLUTION = LTR390_RESOLUTION_20BIT;

// Define the ordered arrays for gain and resolution options (low to high sensitivity)
const ltr390_gain_t UVI_Calculator::gain_options_[] = {
  LTR390_GAIN_1, LTR390_GAIN_3, LTR390_GAIN_6, LTR390_GAIN_9,
  LTR390_GAIN_18
};
const int UVI_Calculator::num_gain_options_ =
  sizeof(gain_options_) / sizeof(gain_options_[0]);

const ltr390_resolution_t UVI_Calculator::res_options_[] = {
  LTR390_RESOLUTION_13BIT, LTR390_RESOLUTION_16BIT, LTR390_RESOLUTION_17BIT,
  LTR390_RESOLUTION_18BIT, LTR390_RESOLUTION_19BIT, LTR390_RESOLUTION_20BIT
};
const int UVI_Calculator::num_res_options_ =
  sizeof(res_options_) / sizeof(res_options_[0]);

// Constructor
UVI_Calculator::UVI_Calculator()
  : sensitivity_(0.0f),
    wfac_(1.0f),
    initialized_(false),
    last_uvs_count_(0),
    last_valid_uvi_(-1.0f),                // Initialize last valid UVI to -1 until first reading
    current_gain_(LTR390_GAIN_3),          // Default initial
    current_res_(LTR390_RESOLUTION_18BIT)  // Default initial
{}

// Initialization
bool UVI_Calculator::begin(ltr390_gain_t initial_gain,
                           ltr390_resolution_t initial_res) {
  initialized_ = false;
  last_uvs_count_ = 0;
  last_valid_uvi_ = -1.0f;  // Reset on begin

  if (!ltr_.begin()) {
    return false;
  }

  ltr_.setMode(LTR390_MODE_UVS);
  if (ltr_.getMode() != LTR390_MODE_UVS) {
    return false;
  }

  // Set initial Gain
  ltr_.setGain(initial_gain);
  current_gain_ = ltr_.getGain();  // Store the actual gain set

  // Set initial Resolution
  ltr_.setResolution(initial_res);
  current_res_ = ltr_.getResolution();  // Store the actual resolution set

  // Calculate initial sensitivity
  sensitivity_ = calculateSensitivity(current_gain_, current_res_);
  if (sensitivity_ <= 0) {
    return false;  // Should not happen if gain/res are valid
  }

  initialized_ = true;
  return true;
}

// Read UVI with optional auto-ranging
float UVI_Calculator::readUVI(bool auto_range) {
  if (!initialized_) {
    return -1.0f;  // Return -1 only if never initialized
  }

  // Check if new data is available from the sensor
  if (ltr_.newDataAvailable()) {
    last_uvs_count_ = ltr_.readUVS();  // Read the latest raw count

    // Perform auto-ranging *before* calculating UVI for this cycle
    // This means the UVI calculated now is based on the settings *before* the range check
    if (auto_range) {
      performAutoRange();  // Adjusts settings for the *next* reading
    }

    // Calculate UVI using the sensitivity that was active for *this* reading
    if (sensitivity_ > 0) {
      last_valid_uvi_ = ((float)last_uvs_count_ / sensitivity_) * wfac_;
    } else {
      // Should not happen if initialized, but as a safeguard
      last_valid_uvi_ = -1.0f;
    }
    return last_valid_uvi_;

  } else {
    // No new data available, return the previously calculated valid value
    return last_valid_uvi_;
  }
}

// --- Auto-Ranging Helper ---
void UVI_Calculator::performAutoRange() {
  if (!initialized_) return;

  uint32_t max_count = getMaxCountForResolution(current_res_);
  if (max_count == 0) return;  // Invalid resolution?

  float low_trigger = (float)max_count * UVI_AUTO_RANGE_LOW_PERCENT / 100.0f;
  float high_trigger = (float)max_count * UVI_AUTO_RANGE_HIGH_PERCENT / 100.0f;

  bool settings_changed = false;
  ltr390_gain_t new_gain = current_gain_;
  ltr390_resolution_t new_res = current_res_;

  int current_gain_idx = findGainIndex(current_gain_);
  int current_res_idx = findResolutionIndex(current_res_);

  if (last_uvs_count_ > high_trigger && last_uvs_count_ < max_count) {
    // Reading is too high (but not saturated), decrease sensitivity
    if (current_gain_idx > 0) {  // Try decreasing gain first
      new_gain = gain_options_[current_gain_idx - 1];
      settings_changed = true;
    } else if (current_res_idx > 0) {  // If gain is min, decrease resolution
      new_res = res_options_[current_res_idx - 1];
      settings_changed = true;
    }
  } else if (last_uvs_count_ < low_trigger) {
    // Reading is too low, increase sensitivity
    if (current_res_idx < num_res_options_ - 1) {  // Try increasing resolution first
      new_res = res_options_[current_res_idx + 1];
      settings_changed = true;
    } else if (current_gain_idx < num_gain_options_ - 1) {  // If res is max, increase gain
      new_gain = gain_options_[current_gain_idx + 1];
      settings_changed = true;
    }
  } else if (last_uvs_count_ >= max_count) {
    if (current_gain_idx > 0) {
      new_gain = gain_options_[current_gain_idx - 1];
      settings_changed = true;
    } else if (current_res_idx > 0) {
      new_res = res_options_[current_res_idx - 1];
      settings_changed = true;
    }
    // If already at min gain and min res, we can't do anything more
  }


  // Apply changes if any were decided
  if (settings_changed) {
    bool change_applied = false;
    if (new_gain != current_gain_) {
      ltr_.setGain(new_gain);
      current_gain_ = ltr_.getGain();  // Update with actual value set
      change_applied = true;
    }
    if (new_res != current_res_) {
      ltr_.setResolution(new_res);
      current_res_ = ltr_.getResolution();  // Update with actual value set
      change_applied = true;
    }

    // Recalculate sensitivity if settings actually changed
    if (change_applied) {
      sensitivity_ = calculateSensitivity(current_gain_, current_res_);
      // Optional: Serial print to indicate settings changed
      // Serial.println("Auto-range adjusted settings.");
    }
  }
}


// --- Other Public Methods ---
void UVI_Calculator::setWindowFactor(float wfac) {
  if (wfac > 0) {
    wfac_ = wfac;
  }
}

uint32_t UVI_Calculator::getLastRawUVS() {
  return last_uvs_count_;
}
ltr390_gain_t UVI_Calculator::getCurrentGain() {
  return current_gain_;
}
ltr390_resolution_t UVI_Calculator::getCurrentResolution() {
  return current_res_;
}
float UVI_Calculator::getCurrentSensitivity() {
  return sensitivity_;
}


// --- Private Helper Functions ---

float UVI_Calculator::calculateSensitivity(ltr390_gain_t gain,
                                           ltr390_resolution_t res) {
  float gain_factor = 1.0f;
  float res_factor = 1.0f;

  switch (gain) {
    case LTR390_GAIN_1: gain_factor = 1.0f / 18.0f; break;
    case LTR390_GAIN_3: gain_factor = 3.0f / 18.0f; break;
    case LTR390_GAIN_6: gain_factor = 6.0f / 18.0f; break;
    case LTR390_GAIN_9: gain_factor = 9.0f / 18.0f; break;
    case LTR390_GAIN_18: gain_factor = 18.0f / 18.0f; break;
    default: return 0.0f;
  }

  switch (res) {
    case LTR390_RESOLUTION_13BIT: res_factor = 12.5f / 400.0f; break;
    case LTR390_RESOLUTION_16BIT: res_factor = 25.0f / 400.0f; break;
    case LTR390_RESOLUTION_17BIT: res_factor = 50.0f / 400.0f; break;
    case LTR390_RESOLUTION_18BIT: res_factor = 100.0f / 400.0f; break;
    case LTR390_RESOLUTION_19BIT: res_factor = 200.0f / 400.0f; break;
    case LTR390_RESOLUTION_20BIT: res_factor = 400.0f / 400.0f; break;
    default: return 0.0f;
  }

  return REFERENCE_SENSITIVITY * gain_factor * res_factor;
}

uint32_t UVI_Calculator::getMaxCountForResolution(ltr390_resolution_t res) {
  switch (res) {
    // Max value is 2^bits - 1
    case LTR390_RESOLUTION_13BIT: return (1UL << 13) - 1;  // 8191
    case LTR390_RESOLUTION_16BIT: return (1UL << 16) - 1;  // 65535
    case LTR390_RESOLUTION_17BIT: return (1UL << 17) - 1;  // 131071
    case LTR390_RESOLUTION_18BIT: return (1UL << 18) - 1;  // 262143
    case LTR390_RESOLUTION_19BIT: return (1UL << 19) - 1;  // 524287
    case LTR390_RESOLUTION_20BIT: return (1UL << 20) - 1;  // 1048575
    default: return 0;
  }
}

int UVI_Calculator::findGainIndex(ltr390_gain_t gain) {
  for (int i = 0; i < num_gain_options_; ++i) {
    if (gain_options_[i] == gain) return i;
  }
  return -1;  // Not found
}

int UVI_Calculator::findResolutionIndex(ltr390_resolution_t res) {
  for (int i = 0; i < num_res_options_; ++i) {
    if (res_options_[i] == res) return i;
  }
  return -1;  // Not found
}
