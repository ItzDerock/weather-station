#pragma once
#include <Adafruit_LTR390.h>

// Define thresholds for auto-ranging (as percentage of max count for resolution)
#define UVI_AUTO_RANGE_LOW_PERCENT 10.0f
#define UVI_AUTO_RANGE_HIGH_PERCENT 90.0f

class UVI_Calculator {
  public:
    // Constructor
    UVI_Calculator();

    // Initialization function
    // Call this in your setup()
    // Returns true on success, false on failure
    // Allows setting initial gain/res, otherwise uses defaults
    bool begin(ltr390_gain_t initial_gain = LTR390_GAIN_3,
               ltr390_resolution_t initial_res = LTR390_RESOLUTION_18BIT);

    // Read the calculated UV Index
    // Implements auto-ranging and returns last valid value if no new data
    // Returns -1.0f ONLY if never initialized successfully.
    float readUVI(bool auto_range = true); // Enable/disable auto-ranging

    // Set the window factor (WFAC) - default is 1.0 (no window)
    void setWindowFactor(float wfac);

    // Get the last raw UVS count read (useful for debugging)
    uint32_t getLastRawUVS();

    // Get current settings (useful for debugging)
    ltr390_gain_t getCurrentGain();
    ltr390_resolution_t getCurrentResolution();
    float getCurrentSensitivity();


  private:
    Adafruit_LTR390 ltr_;          // Instance of the sensor library
    float sensitivity_;            // Calculated sensitivity in Counts/UVI
    float wfac_;                   // Window Factor
    bool initialized_;             // Flag to check if begin() was successful
    uint32_t last_uvs_count_;      // Store the last raw count
    float last_valid_uvi_;         // Store the last calculated UVI
    ltr390_gain_t current_gain_;   // Store the configured gain
    ltr390_resolution_t current_res_; // Store the configured resolution

    // Helper to calculate sensitivity based on gain and resolution
    float calculateSensitivity(ltr390_gain_t gain, ltr390_resolution_t res);

    // Helper to get max count for a given resolution
    uint32_t getMaxCountForResolution(ltr390_resolution_t res);

    // Helper to perform auto-ranging logic
    void performAutoRange();

    // Ordered arrays for gain/resolution to facilitate stepping
    static const ltr390_gain_t gain_options_[];
    static const int num_gain_options_;
    static const ltr390_resolution_t res_options_[];
    static const int num_res_options_;

    // Find index of current setting in options array
    int findGainIndex(ltr390_gain_t gain);
    int findResolutionIndex(ltr390_resolution_t res);
};
