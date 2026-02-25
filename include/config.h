#pragma once

// ===========================================================================
// MagTag CO2 Monitor — Centralized Configuration
//
// All tunable constants live here. Change timing, colors, thresholds,
// pin mappings, and display layout in one place.
// ===========================================================================

// ---------------------------------------------------------------------------
// Build metadata fallbacks (overridden by tools/build_metadata.py)
// ---------------------------------------------------------------------------
#ifndef BUILD_VERSION_STR
#define BUILD_VERSION_STR "dev"
#endif

#ifndef BUILD_HASH_STR
#define BUILD_HASH_STR "unknown"
#endif

#ifndef BUILD_EPOCH_UNIX
#define BUILD_EPOCH_UNIX 0UL
#endif

#ifndef BUILD_DIRTY_STR
#define BUILD_DIRTY_STR "unknown"
#endif

// ---------------------------------------------------------------------------
// Timing and scheduling (what runs when)
// ---------------------------------------------------------------------------
#define USB_SAMPLE_INTERVAL_MS          15000   // Faster sampling while externally powered.
#define USB_DISPLAY_INTERVAL_MS         30000   // More frequent display updates while on USB.
#define BATTERY_SAMPLE_INTERVAL_MS      60000   // Slower sampling to conserve battery.
#define BATTERY_DISPLAY_INTERVAL_MS    300000   // E-ink refresh interval on battery.
#define BATTERY_USB_POLL_INTERVAL_MS    10000   // USB-power poll period in battery mode.
#define POWER_CHECK_INTERVAL_MS          5000   // Battery/USB heuristic update period.

#define FLASH_DURATION_MS                 500   // Standard NeoPixel flash duration.
#define STARTUP_FLASH_MS                  220   // Startup flash duration.
#define INVERT_FLASH_MS                   150   // UI toggle feedback flash duration.
#define STARTUP_BUILD_INFO_MS            5000   // How long to show build metadata after startup image.

#define GRAPH_WINDOW_MINUTES               15   // Time span shown on graphs.

// ---------------------------------------------------------------------------
// Display layout and rendering (where things are drawn)
// ---------------------------------------------------------------------------
#define GRAPH_LINE_THICKNESS              1    // Line thickness used for history traces.

// Graph fill mode — choose exactly one:
//   GRAPH_FILL_NONE            0  No fill under the graph line.
//   GRAPH_FILL_UNDERLINE       1  Solid shade fill under the line (original behavior).
//   GRAPH_FILL_DITHER          2  Checkerboard dither pattern under the line (~50% density).
//   GRAPH_FILL_VLINES          3  Vertical lines every N columns under the line.
//   GRAPH_FILL_SPARSE_DITHER   4  Sparse dither — every other pixel on every other row (~25% density).
#define GRAPH_FILL_MODE                   4    // Active fill style (see list above).
#define GRAPH_FILL_SHADE           EPD_DARK    // Shade used by all fill modes: EPD_LIGHT, EPD_DARK, EPD_GRAY.
#define GRAPH_FILL_RISING_ONLY            0    // When enabled, only rising segments get fill.
#define GRAPH_FILL_VLINE_SPACING          2    // Column spacing for GRAPH_FILL_VLINES mode.

// Combined view layout presets on 296x128 panel.
#define TEXT_RIGHT_BALANCED             140    // Right edge of value text in balanced layout.
#define GRAPH_X_BALANCED                152    // Graphs start X in balanced layout.

#define TEXT_RIGHT_GRAPH_HEAVY          108    // Right edge of value text in graph-heavy layout.
#define GRAPH_X_GRAPH_HEAVY             120    // Graphs start X in graph-heavy layout.

#define GRAPH_H_CO2                      40    // CO2 graph height (emphasized metric).
#define GRAPH_H_STD                      30    // Temp/RH graph height.
#define GRAPH_GAP                         6    // Vertical spacing between stacked graphs.

// Screen-edge content padding for enclosure/bezel compensation.
#define SCREEN_PAD_LEFT                   0
#define SCREEN_PAD_RIGHT                 10
#define SCREEN_PAD_TOP                    0
#define SCREEN_PAD_BOTTOM                 0

// ---------------------------------------------------------------------------
// UI input mapping (which button controls what)
// ---------------------------------------------------------------------------
#define MODE_BTN                         12    // D12: cycle display mode.
#define CAROUSEL_BTN                     14    // D14: toggle carousel mode.
#define SLEEP_TOGGLE_BTN                 11    // D11: long-press to toggle deep/light sleep.
#define SLEEP_TOGGLE_HOLD_MS           2000    // Hold D11 for this long to toggle sleep mode.
#define STATS_BTN                        15    // D15: show battery/system stats overlay.
#define STATS_DISPLAY_MS               5000    // Duration to show stats screen before restoring.

// ---------------------------------------------------------------------------
// NeoPixel feedback colors (what each event looks like)
// ---------------------------------------------------------------------------
#define USB_SAMPLE_FLASH_R               15
#define USB_SAMPLE_FLASH_G               15
#define USB_SAMPLE_FLASH_B               15

#define STARTUP_FLASH_R                   0
#define STARTUP_FLASH_G                 180
#define STARTUP_FLASH_B                   0

#define USB_CONNECTED_FLASH_R             0
#define USB_CONNECTED_FLASH_G             0
#define USB_CONNECTED_FLASH_B           200

#define MODE_TOGGLE_FLASH_R              24
#define MODE_TOGGLE_FLASH_G              24
#define MODE_TOGGLE_FLASH_B              24

#define CAROUSEL_ON_FLASH_R             140
#define CAROUSEL_ON_FLASH_G               0
#define CAROUSEL_ON_FLASH_B             110

#define CAROUSEL_OFF_FLASH_R            255
#define CAROUSEL_OFF_FLASH_G              0
#define CAROUSEL_OFF_FLASH_B              0

#define DEEP_SLEEP_ON_FLASH_R             0    // Deep sleep toggle ON: dark blue (center 2 pixels).
#define DEEP_SLEEP_ON_FLASH_G             0
#define DEEP_SLEEP_ON_FLASH_B            80

#define DEEP_SLEEP_OFF_FLASH_R           80    // Deep sleep toggle OFF: light blue (all 4 pixels).
#define DEEP_SLEEP_OFF_FLASH_G          120
#define DEEP_SLEEP_OFF_FLASH_B          255

#define STATS_FLASH_R                    60    // Stats button: dim white flash.
#define STATS_FLASH_G                    60
#define STATS_FLASH_B                    60

#define GENERIC_RED_FLASH_R             255
#define GENERIC_RED_FLASH_G               0
#define GENERIC_RED_FLASH_B               0

// ---------------------------------------------------------------------------
// Battery behavior thresholds and alert timings
// ---------------------------------------------------------------------------
#define BATTERY_CAPACITY_MAH           4400    // Battery capacity in mAh — set to match your LiPo.

#define BATTERY_WARN_50_PERCENT          50    // Mid-level warning threshold.
#define BATTERY_WARN_50_DURATION_MS   30000UL // Mid-level warning message duration.

#define BATTERY_CRITICAL_PERCENT         10    // Critical battery threshold.
#define BATTERY_CRITICAL_INTERVAL_MS 1800000UL // Repeat interval for critical warning.
#define BATTERY_CRITICAL_DURATION_MS  60000UL // Critical warning message duration.
#define BATTERY_CRITICAL_BLINK_MS       600UL // Blink cadence during critical warning.

#define BATTERY_CRITICAL_LED_R           30
#define BATTERY_CRITICAL_LED_G            0
#define BATTERY_CRITICAL_LED_B            0

// Battery-level LED thresholds (%). Mapping:
// >80 => 4 green, 50..80 => 3 yellow, 30..49 => 2 orange, <30 => 1 red
#define BATTERY_LEVEL_4_MIN_PERCENT      81
#define BATTERY_LEVEL_3_MIN_PERCENT      50
#define BATTERY_LEVEL_2_MIN_PERCENT      30

// Battery-level LED colors.
#define BATTERY_LEVEL_4_LED_R             0
#define BATTERY_LEVEL_4_LED_G           180
#define BATTERY_LEVEL_4_LED_B             0
#define BATTERY_LEVEL_3_LED_R           220
#define BATTERY_LEVEL_3_LED_G           180
#define BATTERY_LEVEL_3_LED_B             0
#define BATTERY_LEVEL_2_LED_R           255
#define BATTERY_LEVEL_2_LED_G            90
#define BATTERY_LEVEL_2_LED_B             0
#define BATTERY_LEVEL_1_LED_R           255
#define BATTERY_LEVEL_1_LED_G             0
#define BATTERY_LEVEL_1_LED_B             0

// Battery-level LED animation timing.
#define BATTERY_LEVEL_FLASH_ON_MS       180
#define BATTERY_LEVEL_FLASH_OFF_MS      120
#define BATTERY_STARTUP_SWEEP_MS         90
#define BATTERY_STARTUP_LEVEL_FLASHES     3

#define BATTERY_DISPLAY_EVERY_N ((BATTERY_DISPLAY_INTERVAL_MS / BATTERY_SAMPLE_INTERVAL_MS) > 0 ? (BATTERY_DISPLAY_INTERVAL_MS / BATTERY_SAMPLE_INTERVAL_MS) : 1)

// ---------------------------------------------------------------------------
// Sensor protocol constants
// ---------------------------------------------------------------------------
#define SCD30_I2C_ADDR                 0x61   // SCD30 I2C slave address.
#define SCD30_CMD_STOP_MEASUREMENTS    0x0104 // SCD30 command: stop continuous measurement.
#define SCD30_WARMUP_MS               25000   // Cold-start warm-up before trusting readings.
