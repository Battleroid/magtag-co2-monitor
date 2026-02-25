#pragma once

#include <stdint.h>

// ---------------------------------------------------------------------------
// Display mode enums — shared across display, input, and main modules.
// ---------------------------------------------------------------------------

enum DisplayMode : uint8_t {
    DISPLAY_MODE_COMBINED = 0,
    DISPLAY_MODE_CO2_ONLY,
    DISPLAY_MODE_TEMP_ONLY,
    DISPLAY_MODE_RH_ONLY,
    DISPLAY_MODE_TEMP_RH,
    DISPLAY_MODE_TEXT_ONLY,            // CO2 left + temp/RH right, no graphs
    DISPLAY_MODE_COUNT
};

enum DeepDisplayMode : uint8_t {
    DEEP_DISPLAY_MODE_COMBINED = 0,   // CO2 left half, temp+RH right half
    DEEP_DISPLAY_MODE_CO2_ONLY,       // CO2 centered full screen
    DEEP_DISPLAY_MODE_TEMP_ONLY,      // Temp centered full screen
    DEEP_DISPLAY_MODE_RH_ONLY,        // RH centered full screen
    DEEP_DISPLAY_MODE_COUNT
};
