#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SCD30.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_ThinkInk.h>
#include <Adafruit_LIS3DH.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <USB.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <WiFi.h>
#ifdef CONFIG_BT_ENABLED
#include <esp_bt.h>
#endif

#if __has_include("startup_image.h")
#include "startup_image.h"
#define HAS_STARTUP_IMAGE 1
#else
#define HAS_STARTUP_IMAGE 0
#endif

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

#define GENERIC_RED_FLASH_R             255
#define GENERIC_RED_FLASH_G               0
#define GENERIC_RED_FLASH_B               0

// ---------------------------------------------------------------------------
// Battery behavior thresholds and alert timings
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// History ring buffer
//   Size is derived from graph window and fastest configured sample interval,
//   so changing sample cadence (USB/battery) or window duration stays in sync.
// ---------------------------------------------------------------------------
#define FASTEST_SAMPLE_INTERVAL_MS ((USB_SAMPLE_INTERVAL_MS < BATTERY_SAMPLE_INTERVAL_MS) ? USB_SAMPLE_INTERVAL_MS : BATTERY_SAMPLE_INTERVAL_MS)
#define HISTORY_LEN ((((GRAPH_WINDOW_MINUTES * 60UL * 1000UL) + FASTEST_SAMPLE_INTERVAL_MS - 1) / FASTEST_SAMPLE_INTERVAL_MS) + 1)
static float histCO2[HISTORY_LEN];
static float histTempF[HISTORY_LEN];
static float histRH[HISTORY_LEN];
static unsigned long histTs[HISTORY_LEN];
static int   histCount = 0;        // total samples ever recorded
static int   histHead  = 0;        // next write index

static void pushSample(float co2, float tempF, float rh) {
    histCO2[histHead]   = co2;
    histTempF[histHead] = tempF;
    histRH[histHead]    = rh;
    histTs[histHead]    = millis();
    histHead = (histHead + 1) % HISTORY_LEN;
    histCount++;
}

// Return the number of valid samples currently in the buffer.
static int histSamples() { return (histCount < HISTORY_LEN) ? histCount : HISTORY_LEN; }

// Get the i-th oldest valid sample (0 = oldest).
static float histGet(const float *buf, int i) {
    int n = histSamples();
    int idx = (histHead - n + i + HISTORY_LEN) % HISTORY_LEN;
    return buf[idx];
}

static unsigned long histGetTs(int i) {
    int n = histSamples();
    int idx = (histHead - n + i + HISTORY_LEN) % HISTORY_LEN;
    return histTs[idx];
}

static bool     graphHeavyLayout = true;

enum DisplayMode : uint8_t {
    DISPLAY_MODE_COMBINED = 0,
    DISPLAY_MODE_CO2_ONLY,
    DISPLAY_MODE_TEMP_ONLY,
    DISPLAY_MODE_RH_ONLY,
    DISPLAY_MODE_TEMP_RH,
    DISPLAY_MODE_TEXT_ONLY,            // CO2 left + temp/RH right, no graphs
    DISPLAY_MODE_COUNT
};

static uint8_t currentDisplayMode = DISPLAY_MODE_COMBINED;
static bool carouselModeEnabled = false;

enum DeepDisplayMode : uint8_t {
    DEEP_DISPLAY_MODE_COMBINED = 0,   // CO2 left half, temp+RH right half
    DEEP_DISPLAY_MODE_CO2_ONLY,       // CO2 centered full screen
    DEEP_DISPLAY_MODE_TEMP_ONLY,      // Temp centered full screen
    DEEP_DISPLAY_MODE_RH_ONLY,        // RH centered full screen
    DEEP_DISPLAY_MODE_COUNT
};

static uint8_t currentDeepDisplayMode = DEEP_DISPLAY_MODE_COMBINED;
static bool deepSleepEnabled = false;
static bool lightSleepWakePending = false;   // set after waking from light sleep

// Last displayed values (for immediate redraw on button toggles)
static float lastCO2   = 0;
static float lastTempF = 0;
static float lastRH    = 0;
static bool  hasReading = false;
static bool  displayUpdateInProgress = false;
static bool  modeCyclePending = false;
static bool  carouselTogglePending = false;
static portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

static bool usbPowerPresent = false;
static unsigned long lastSampleMs = 0;
static unsigned long lastDisplayMs = 0;
static bool batteryFilterInit = false;
static float batteryVoltageFiltered = 0.0f;
static unsigned long lastPowerCheckMs = 0;
static int usbHeuristicScore = 0;
static bool usbByHeuristic = false;

// USB event-driven mount tracking (set from TinyUSB task context)
static volatile bool usbMounted = false;

// FreeRTOS button task handles — created for USB mode, deleted on battery transition
static TaskHandle_t modeButtonTaskHandle     = nullptr;
static TaskHandle_t carouselButtonTaskHandle  = nullptr;
static TaskHandle_t sleepToggleTaskHandle     = nullptr;
RTC_DATA_ATTR static uint32_t batterySampleCycles = 0;
RTC_DATA_ATTR static uint32_t batterySampleElapsedMs = BATTERY_SAMPLE_INTERVAL_MS;
RTC_DATA_ATTR static bool batteryWarn50Shown = false;
RTC_DATA_ATTR static uint32_t batteryCriticalElapsedMs = 0;
RTC_DATA_ATTR static bool rtcHasUsbPowerState = false;
RTC_DATA_ATTR static bool rtcLastUsbPowerPresent = false;
RTC_DATA_ATTR static uint8_t rtcLastBatteryLedTier = 255;
RTC_DATA_ATTR static bool rtcBatteryTierFlashPending = false;
RTC_DATA_ATTR static uint8_t rtcPendingBatteryLedTier = 0;
RTC_DATA_ATTR static bool rtcNotified4to3 = false;
RTC_DATA_ATTR static bool rtcNotified2to1 = false;

// RTC-persisted state (survives deep sleep)
RTC_DATA_ATTR static uint8_t  rtcDisplayMode       = DISPLAY_MODE_COMBINED;
RTC_DATA_ATTR static bool     rtcCarouselEnabled    = false;
RTC_DATA_ATTR static float    rtcLastCO2            = 0;
RTC_DATA_ATTR static float    rtcLastTempF          = 0;
RTC_DATA_ATTR static float    rtcLastRH             = 0;
RTC_DATA_ATTR static bool     rtcHasReading         = false;
RTC_DATA_ATTR static bool     rtcDeepSleepEnabled   = false;
RTC_DATA_ATTR static uint8_t  rtcDeepDisplayMode    = DEEP_DISPLAY_MODE_COMBINED;

static bool wokeFromDeepSleep = false;
static uint8_t buttonWakeGPIO = 0;   // which button GPIO caused wake (0 = none)
static bool scd30Ready = false;      // whether SCD30 was initialized this wake cycle

static uint16_t fgColor() { return EPD_BLACK; }
static uint16_t bgColor() { return EPD_WHITE; }

static float readBatteryVoltage() {
    int raw = analogRead(BATT_MONITOR);
    return (raw / 4095.0f) * 3.3f * 2.0f;
}

static uint8_t batteryPercentFromVoltage(float voltage) {
    const float fullV = 4.20f;
    const float emptyV = 3.30f;
    float pct = (voltage - emptyV) * 100.0f / (fullV - emptyV);
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return (uint8_t)pct;
}

static bool stopScd30Measurements() {
    Wire.beginTransmission(SCD30_I2C_ADDR);
    Wire.write((uint8_t)((SCD30_CMD_STOP_MEASUREMENTS >> 8) & 0xFF));
    Wire.write((uint8_t)(SCD30_CMD_STOP_MEASUREMENTS & 0xFF));
    uint8_t err = Wire.endTransmission(true);
    delay(4);
    return (err == 0);
}

static void loadVisualPreferences() {
    graphHeavyLayout = true;
    Serial.println("Loaded prefs: layout=GRAPH_HEAVY");
}

static void adjustUsbHeuristicScore(float dv) {
    if (dv > 0.003f) {
        if (usbHeuristicScore < 3) usbHeuristicScore++;
    } else if (dv < -0.003f) {
        if (usbHeuristicScore > -3) usbHeuristicScore--;
    } else {
        if (usbHeuristicScore > 0) usbHeuristicScore--;
        else if (usbHeuristicScore < 0) usbHeuristicScore++;
    }

    if (usbHeuristicScore >= 2) usbByHeuristic = true;
    else if (usbHeuristicScore <= -2) usbByHeuristic = false;
}

static void playToneOnce(uint16_t freq, uint16_t durationMs, uint8_t volumePercent = 100) {
    if (volumePercent > 100) volumePercent = 100;

    digitalWrite(SPEAKER_SHUTDOWN, HIGH);
    delay(8);

    if (volumePercent >= 100) {
        tone(A0, freq);
        delay(durationMs);
    } else {
        const uint8_t gatePeriodMs = 2;
        uint8_t onMs = (gatePeriodMs * volumePercent) / 100;
        if (onMs < 1 && volumePercent > 0) onMs = 1;
        uint8_t offMs = gatePeriodMs - onMs;

        unsigned long tStart = millis();
        while (millis() - tStart < durationMs) {
            if (onMs > 0) {
                tone(A0, freq);
                delay(onMs);
            }
            if (offMs > 0) {
                noTone(A0);
                delay(offMs);
            }
        }
    }

    noTone(A0);
    digitalWrite(SPEAKER_SHUTDOWN, LOW);
    delay(10);
}

static void playStartupJingle() {
    playToneOnce(880, 110);
    delay(20);
    playToneOnce(1175, 110);
    delay(20);
    playToneOnce(1568, 170);
}

static void playLayoutToggleTone() {
    playToneOnce(988, 90);
}

static void playModeToggleTone() {
    playToneOnce(1318, 80);
}

static void playUsbConnectedJingle() {
    playToneOnce(1047, 70);
    playToneOnce(1318, 80);
    playToneOnce(1568, 100);
}

static void playUsbDisconnectedJingle() {
    playToneOnce(1568, 70);
    playToneOnce(1318, 80);
    playToneOnce(1047, 110);
}

static bool detectUsbPowerPresent() {
    // Primary: event-driven mount state (instant on plug/unplug)
    bool mounted = usbMounted;
    static bool prevMounted = false;

    // Secondary: voltage heuristic for dumb chargers that don't enumerate
    unsigned long now = millis();
    if (lastPowerCheckMs == 0 || now - lastPowerCheckMs >= POWER_CHECK_INTERVAL_MS) {
        float voltage = readBatteryVoltage();
        if (!batteryFilterInit) {
            batteryVoltageFiltered = voltage;
            batteryFilterInit = true;
        } else {
            float prev = batteryVoltageFiltered;
            batteryVoltageFiltered = 0.8f * batteryVoltageFiltered + 0.2f * voltage;
            adjustUsbHeuristicScore(batteryVoltageFiltered - prev);
        }
        lastPowerCheckMs = now;
    }

    if (mounted) {
        // USB enumerated — force heuristic high
        usbByHeuristic = true;
        usbHeuristicScore = 3;
    } else if (prevMounted && !mounted) {
        // USB just unmounted — immediately reset heuristic
        usbByHeuristic = false;
        usbHeuristicScore = 0;
        Serial.println("USB unmount — heuristic reset");
    }

    if (prevMounted != mounted) {
        Serial.printf("USB mount state: %s\n", mounted ? "MOUNTED" : "UNMOUNTED");
    }
    prevMounted = mounted;

    return mounted || usbByHeuristic;
}

extern Adafruit_NeoPixel pixels;

static uint8_t batteryLedTierFromPercent(uint8_t batteryPercent) {
    if (batteryPercent >= BATTERY_LEVEL_4_MIN_PERCENT) return 4;
    if (batteryPercent >= BATTERY_LEVEL_3_MIN_PERCENT) return 3;
    if (batteryPercent >= BATTERY_LEVEL_2_MIN_PERCENT) return 2;
    return 1;
}

static void batteryLedTierColor(uint8_t tier, uint8_t *red, uint8_t *green, uint8_t *blue) {
    switch (tier) {
        case 4:
            *red = BATTERY_LEVEL_4_LED_R; *green = BATTERY_LEVEL_4_LED_G; *blue = BATTERY_LEVEL_4_LED_B;
            break;
        case 3:
            *red = BATTERY_LEVEL_3_LED_R; *green = BATTERY_LEVEL_3_LED_G; *blue = BATTERY_LEVEL_3_LED_B;
            break;
        case 2:
            *red = BATTERY_LEVEL_2_LED_R; *green = BATTERY_LEVEL_2_LED_G; *blue = BATTERY_LEVEL_2_LED_B;
            break;
        default:
            *red = BATTERY_LEVEL_1_LED_R; *green = BATTERY_LEVEL_1_LED_G; *blue = BATTERY_LEVEL_1_LED_B;
            break;
    }
}

static void showBatteryLedFrame(uint8_t litCount, uint8_t red, uint8_t green, uint8_t blue) {
    const uint8_t batteryPixelOrder[4] = {3, 2, 1, 0};
    if (litCount > 4) litCount = 4;

    pixels.clear();
    for (uint8_t i = 0; i < litCount; i++) {
        pixels.setPixelColor(batteryPixelOrder[i], pixels.Color(red, green, blue));
    }
    pixels.show();
}

static void showBatteryStatusColorFrame(uint8_t litCount) {
    const uint8_t batteryPixelOrder[4] = {3, 2, 1, 0};
    if (litCount > 4) litCount = 4;

    pixels.clear();
    for (uint8_t i = 0; i < litCount; i++) {
        uint8_t red, green, blue;
        batteryLedTierColor(i + 1, &red, &green, &blue);
        pixels.setPixelColor(batteryPixelOrder[i], pixels.Color(red, green, blue));
    }
    pixels.show();
}

static void flashBatteryLedTier(uint8_t tier, uint8_t flashes, uint16_t onMs, uint16_t offMs) {
    uint8_t red, green, blue;
    batteryLedTierColor(tier, &red, &green, &blue);

    digitalWrite(NEOPIXEL_POWER, LOW);
    delay(10);

    for (uint8_t i = 0; i < flashes; i++) {
        showBatteryLedFrame(tier, red, green, blue);
        delay(onMs);
        pixels.clear();
        pixels.show();
        if (i + 1 < flashes) delay(offMs);
    }

    digitalWrite(NEOPIXEL_POWER, HIGH);
}

static void handleBatteryTierTransition(uint8_t batteryPercent) {
    uint8_t newTier = batteryLedTierFromPercent(batteryPercent);
    if (rtcLastBatteryLedTier == 255) {
        rtcLastBatteryLedTier = newTier;
        return;
    }

    if (rtcLastBatteryLedTier == 4 && newTier == 3 && !rtcNotified4to3) {
        rtcBatteryTierFlashPending = true;
        rtcPendingBatteryLedTier = newTier;
        rtcNotified4to3 = true;
    }

    if (rtcLastBatteryLedTier == 2 && newTier == 1 && !rtcNotified2to1) {
        rtcBatteryTierFlashPending = true;
        rtcPendingBatteryLedTier = newTier;
        rtcNotified2to1 = true;
    }

    rtcLastBatteryLedTier = newTier;
}

static void resetBatteryTierNotificationsForUsb() {
    rtcLastBatteryLedTier = 255;
    rtcBatteryTierFlashPending = false;
    rtcPendingBatteryLedTier = 0;
    rtcNotified4to3 = false;
    rtcNotified2to1 = false;
}

static void flashPendingBatteryTierIfAny() {
    if (!usbPowerPresent && rtcBatteryTierFlashPending) {
        flashBatteryLedTier(rtcPendingBatteryLedTier, 1, BATTERY_LEVEL_FLASH_ON_MS, BATTERY_LEVEL_FLASH_OFF_MS);
        rtcBatteryTierFlashPending = false;
    }
}

static void playBatteryStartupAnimation(uint8_t batteryPercent) {
    uint8_t tier = batteryLedTierFromPercent(batteryPercent);

    digitalWrite(NEOPIXEL_POWER, LOW);
    delay(10);

    for (uint8_t count = 1; count <= 4; count++) {
        showBatteryStatusColorFrame(count);
        delay(BATTERY_STARTUP_SWEEP_MS);
    }
    for (int8_t count = 3; count >= 1; count--) {
        showBatteryStatusColorFrame((uint8_t)count);
        delay(BATTERY_STARTUP_SWEEP_MS);
    }

    pixels.clear();
    pixels.show();
    for (uint8_t i = 0; i < BATTERY_STARTUP_LEVEL_FLASHES; i++) {
        showBatteryStatusColorFrame(tier);
        delay(BATTERY_LEVEL_FLASH_ON_MS);
        pixels.clear();
        pixels.show();
        if (i + 1 < BATTERY_STARTUP_LEVEL_FLASHES) delay(BATTERY_LEVEL_FLASH_OFF_MS);
    }

    digitalWrite(NEOPIXEL_POWER, HIGH);
}

static void disableRadios() {
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
#ifdef CONFIG_BT_ENABLED
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        esp_bt_controller_disable();
    }
#endif
}

static uint32_t currentSampleIntervalMs() {
    return usbPowerPresent ? USB_SAMPLE_INTERVAL_MS : BATTERY_SAMPLE_INTERVAL_MS;
}

static uint32_t currentDisplayIntervalMs() {
    return usbPowerPresent ? USB_DISPLAY_INTERVAL_MS : BATTERY_DISPLAY_INTERVAL_MS;
}

// ---------------------------------------------------------------------------
// Globals  (board-defined pins from MagTag variant header)
// ---------------------------------------------------------------------------
Adafruit_SCD30  scd30;
Adafruit_NeoPixel pixels(4, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
ThinkInk_290_Grayscale4_EAAMFGN display(EPD_DC, EPD_RESET, EPD_CS, -1, -1);
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

static void flashNeopixelsColor(uint8_t red, uint8_t green, uint8_t blue, uint16_t ms);
void updateDisplay(float co2, float tempF, float rh);

static void showBatteryWarningMessage(const char *line1, const char *line2, uint32_t durationMs, bool blinkDimRed) {
    display.clearBuffer();
    display.fillScreen(bgColor());
    display.setTextColor(fgColor());
    display.setTextWrap(false);

    int16_t x1, y1;
    uint16_t w1, h1;
    display.setTextSize(2);
    display.getTextBounds(line1, 0, 0, &x1, &y1, &w1, &h1);
    int16_t yTop = (display.height() / 2) - 20;
    int16_t line1X = (display.width() - (int16_t)w1) / 2;
    display.setCursor(line1X < 0 ? 0 : line1X, yTop);
    display.print(line1);

    if (line2 && line2[0] != '\0') {
        int16_t x2, y2;
        uint16_t w2, h2;
        display.setTextSize(2);
        display.getTextBounds(line2, 0, 0, &x2, &y2, &w2, &h2);
        int16_t line2X = (display.width() - (int16_t)w2) / 2;
        display.setCursor(line2X < 0 ? 0 : line2X, yTop + 28);
        display.print(line2);
    }

    display.display();

    unsigned long startMs = millis();
    unsigned long lastBlinkMs = startMs;
    bool ledOn = false;

    while (millis() - startMs < durationMs) {
        if (blinkDimRed && (millis() - lastBlinkMs >= BATTERY_CRITICAL_BLINK_MS)) {
            ledOn = !ledOn;
            pixels.clear();
            if (ledOn) {
                for (uint8_t i = 0; i < pixels.numPixels(); ++i) {
                    pixels.setPixelColor(i, pixels.Color(BATTERY_CRITICAL_LED_R, BATTERY_CRITICAL_LED_G, BATTERY_CRITICAL_LED_B));
                }
            }
            pixels.show();
            lastBlinkMs = millis();
        }
        delay(50);
    }

    pixels.clear();
    pixels.show();

    if (hasReading) {
        updateDisplay(lastCO2, lastTempF, lastRH);
        lastDisplayMs = millis();
    }
}

static void flashSampleIndicator() {
    if (usbPowerPresent) {
        flashNeopixelsColor(USB_SAMPLE_FLASH_R, USB_SAMPLE_FLASH_G, USB_SAMPLE_FLASH_B, FLASH_DURATION_MS);
    }
}

static void enterDeepSleepMs(uint32_t sleepMs) {
    // Persist state to RTC memory before sleeping
    rtcDisplayMode    = currentDisplayMode;
    rtcCarouselEnabled = carouselModeEnabled;
    rtcLastCO2        = lastCO2;
    rtcLastTempF      = lastTempF;
    rtcLastRH         = lastRH;
    rtcHasReading     = hasReading;
    rtcLastUsbPowerPresent = usbPowerPresent;
    rtcHasUsbPowerState = true;
    rtcDeepSleepEnabled = deepSleepEnabled;
    rtcDeepDisplayMode  = currentDeepDisplayMode;

    // Stop SCD30 continuous measurement (only if we started it)
    if (scd30Ready) {
        bool stopOk = stopScd30Measurements();
        if (!stopOk) {
            Serial.println("Warning: failed to send SCD30 stop command");
        } else {
            Serial.println("SCD30 measurement stopped before deep sleep");
        }
        scd30Ready = false;
    }

    disableRadios();

    // Power down e-ink display (software + hardware reset low)
    display.powerDown();
    digitalWrite(EPD_RESET, LOW);

    // Power off NeoPixels and speaker amp
    digitalWrite(NEOPIXEL_POWER, HIGH);
    digitalWrite(SPEAKER_SHUTDOWN, LOW);

    // --- Wake sources ---
    // Timer wake for periodic sampling
    esp_sleep_enable_timer_wakeup((uint64_t)sleepMs * 1000ULL);

    // Button wake: D14 (carousel), D12 (mode), D11 (sleep toggle) — active-low
    uint64_t buttonMask = (1ULL << CAROUSEL_BTN) | (1ULL << MODE_BTN) | (1ULL << SLEEP_TOGGLE_BTN);
    esp_sleep_enable_ext1_wakeup(buttonMask, ESP_EXT1_WAKEUP_ANY_LOW);

    // Keep RTC peripherals powered so internal pull-ups stay active
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    // Explicitly enable RTC-domain pull-ups on button GPIOs to prevent
    // floating inputs from triggering spurious ext1 wakes during sleep.
    const gpio_num_t btnPins[] = { (gpio_num_t)CAROUSEL_BTN, (gpio_num_t)MODE_BTN, (gpio_num_t)SLEEP_TOGGLE_BTN };
    for (auto pin : btnPins) {
        rtc_gpio_init(pin);
        rtc_gpio_set_direction(pin, RTC_GPIO_MODE_INPUT_ONLY);
        rtc_gpio_pullup_en(pin);
        rtc_gpio_pulldown_dis(pin);
    }

    float _battV = readBatteryVoltage();
    uint8_t _battPct = batteryPercentFromVoltage(_battV);
    Serial.printf("Deep sleep %lu ms | Batt: %.2fV (%u%%)\n", (unsigned long)sleepMs, _battV, _battPct);
    Serial.flush();
    esp_deep_sleep_start();
}

// ---------------------------------------------------------------------------
// Light sleep: CPU pauses, RAM preserved, execution resumes here on wake.
// Used as the default battery sleep mode to retain graph history in PSRAM.
// ---------------------------------------------------------------------------
static void enterLightSleepMs(uint32_t sleepMs) {
    // ── Power down peripherals (mirror deep-sleep path for consistency) ──

    // E-ink display: software power-down + hold RESET low
    display.powerDown();
    digitalWrite(EPD_RESET, LOW);

    // NeoPixels and speaker amp off
    digitalWrite(NEOPIXEL_POWER, HIGH);
    digitalWrite(SPEAKER_SHUTDOWN, LOW);

    // --- Wake sources ---
    esp_sleep_enable_timer_wakeup((uint64_t)sleepMs * 1000ULL);

    // Button wake: D14 (carousel), D12 (mode), D11 (sleep toggle) — active-low
    uint64_t buttonMask = (1ULL << CAROUSEL_BTN) | (1ULL << MODE_BTN) | (1ULL << SLEEP_TOGGLE_BTN);
    esp_sleep_enable_ext1_wakeup(buttonMask, ESP_EXT1_WAKEUP_ANY_LOW);

    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    const gpio_num_t btnPins[] = { (gpio_num_t)CAROUSEL_BTN, (gpio_num_t)MODE_BTN, (gpio_num_t)SLEEP_TOGGLE_BTN };
    for (auto pin : btnPins) {
        rtc_gpio_init(pin);
        rtc_gpio_set_direction(pin, RTC_GPIO_MODE_INPUT_ONLY);
        rtc_gpio_pullup_en(pin);
        rtc_gpio_pulldown_dis(pin);
    }

    Serial.printf("Light sleep %lu ms\n", (unsigned long)sleepMs);
    Serial.flush();

    esp_light_sleep_start();

    // ── Execution resumes here after light sleep ──

    // Restore button GPIOs from RTC mode
    for (auto pin : btnPins) {
        rtc_gpio_deinit(pin);
        pinMode((uint8_t)pin, INPUT_PULLUP);
    }

    // Re-initialize peripheral buses — ESP32-S2 APB clock stops during
    // light sleep which can leave SPI/I2C peripherals in an undefined state.
    SPI.begin();
    Wire.begin(SDA, SCL);

    lightSleepWakePending = true;
    Serial.printf("Woke from light sleep, cause: %d\n", (int)esp_sleep_get_wakeup_cause());
}

// Forward declarations
void updateDisplay(float co2, float tempF, float rh);
void updateDisplayDeepSleep(float co2, float tempF, float rh);
static void flashNeopixelsColor(uint8_t red, uint8_t green, uint8_t blue, uint16_t ms);
static void flashCenterNeopixels(uint8_t r, uint8_t g, uint8_t b, uint16_t ms);
static void advanceDisplayMode();
static void advanceDeepDisplayMode();
static const char *displayModeName(uint8_t mode);
static const char *displayModeNameDeep(uint8_t mode);
static void showSleepModeStatus();
static void handleSleepToggle();
static void enterLightSleepMs(uint32_t sleepMs);
void showStatus(const char *msg);

// ---------------------------------------------------------------------------
// Battery-mode helper: handle a button press by GPIO number.
// Applies the action (mode/carousel) and redraws if we have a reading.
// ---------------------------------------------------------------------------
static void applyButtonAction(uint8_t gpio) {
    if (gpio == MODE_BTN) {
        if (deepSleepEnabled && !usbPowerPresent) {
            advanceDeepDisplayMode();
            Serial.printf("Button → deep mode: %s\n", displayModeNameDeep(currentDeepDisplayMode));
        } else {
            advanceDisplayMode();
            Serial.printf("Button → mode: %s\n", displayModeName(currentDisplayMode));
        }
        flashNeopixelsColor(MODE_TOGGLE_FLASH_R, MODE_TOGGLE_FLASH_G, MODE_TOGGLE_FLASH_B, INVERT_FLASH_MS);
        playModeToggleTone();
    } else if (gpio == CAROUSEL_BTN) {
        carouselModeEnabled = !carouselModeEnabled;
        Serial.printf("Button → carousel: %s\n", carouselModeEnabled ? "ON" : "OFF");
        if (carouselModeEnabled) {
            flashNeopixelsColor(CAROUSEL_ON_FLASH_R, CAROUSEL_ON_FLASH_G, CAROUSEL_ON_FLASH_B, INVERT_FLASH_MS);
        } else {
            flashNeopixelsColor(CAROUSEL_OFF_FLASH_R, CAROUSEL_OFF_FLASH_G, CAROUSEL_OFF_FLASH_B, INVERT_FLASH_MS);
        }
        playModeToggleTone();
    } else {
        return;
    }

    if (hasReading) {
        if (deepSleepEnabled && !usbPowerPresent) {
            updateDisplayDeepSleep(lastCO2, lastTempF, lastRH);
        } else {
            updateDisplay(lastCO2, lastTempF, lastRH);
        }
        lastDisplayMs = millis();
    }
}

// ---------------------------------------------------------------------------
// Battery-mode helper: delay for `waitMs` while polling buttons every ~50 ms.
// Detects new presses (edge-triggered) and immediately processes them.
// ---------------------------------------------------------------------------
static void delayWithButtonPolling(uint32_t waitMs) {
    const uint8_t btnPins[] = { MODE_BTN, CAROUSEL_BTN, SLEEP_TOGGLE_BTN };
    const int btnCount = sizeof(btnPins) / sizeof(btnPins[0]);
    bool prevState[3] = { false, false, false };
    unsigned long d11DownAt = 0;
    bool d11Held = false;

    // Seed previous state so already-held buttons don't re-trigger
    for (int i = 0; i < btnCount; i++) {
        prevState[i] = (digitalRead(btnPins[i]) == LOW);
    }

    unsigned long start = millis();
    while (millis() - start < waitMs) {
        for (int i = 0; i < btnCount; i++) {
            bool pressed = (digitalRead(btnPins[i]) == LOW);
            if (btnPins[i] == SLEEP_TOGGLE_BTN) {
                // D11 uses long-press detection
                if (pressed && !prevState[i]) {
                    d11DownAt = millis();
                    d11Held = false;
                }
                if (pressed && !d11Held && (millis() - d11DownAt >= SLEEP_TOGGLE_HOLD_MS)) {
                    handleSleepToggle();
                    d11Held = true;
                    while (digitalRead(SLEEP_TOGGLE_BTN) == LOW) delay(20);
                }
                if (!pressed) d11Held = false;
            } else if (pressed && !prevState[i]) {
                // New press detected — debounce briefly then act
                delay(40);
                if (digitalRead(btnPins[i]) == LOW) {
                    applyButtonAction(btnPins[i]);
                    // Wait for release to avoid repeat-fire
                    while (digitalRead(btnPins[i]) == LOW) delay(20);
                }
            }
            prevState[i] = pressed;
        }
        delay(50);
    }
}

static const char *displayModeName(uint8_t mode) {
    switch (mode) {
        case DISPLAY_MODE_COMBINED: return "COMBINED";
        case DISPLAY_MODE_CO2_ONLY: return "CO2";
        case DISPLAY_MODE_TEMP_ONLY: return "TEMP";
        case DISPLAY_MODE_RH_ONLY: return "HUMIDITY";
        case DISPLAY_MODE_TEMP_RH: return "TEMP_RH";
        case DISPLAY_MODE_TEXT_ONLY: return "TEXT_ONLY";
        default: return "UNKNOWN";
    }
}

static void advanceDisplayMode() {
    currentDisplayMode = (uint8_t)((currentDisplayMode + 1) % DISPLAY_MODE_COUNT);
}

static void advanceDeepDisplayMode() {
    currentDeepDisplayMode = (uint8_t)((currentDeepDisplayMode + 1) % DEEP_DISPLAY_MODE_COUNT);
}

static const char *displayModeNameDeep(uint8_t mode) {
    switch (mode) {
        case DEEP_DISPLAY_MODE_COMBINED: return "DEEP_COMBINED";
        case DEEP_DISPLAY_MODE_CO2_ONLY: return "DEEP_CO2";
        case DEEP_DISPLAY_MODE_TEMP_ONLY: return "DEEP_TEMP";
        case DEEP_DISPLAY_MODE_RH_ONLY: return "DEEP_RH";
        default: return "UNKNOWN";
    }
}

static void handleModeCycleRequest() {
    bool canApplyNow;

    portENTER_CRITICAL(&stateMux);
    canApplyNow = !displayUpdateInProgress;
    if (!canApplyNow) {
        modeCyclePending = true;
        portEXIT_CRITICAL(&stateMux);
        return;
    }
    displayUpdateInProgress = true;
    portEXIT_CRITICAL(&stateMux);

    advanceDisplayMode();
    Serial.printf("Display mode: %s\n", displayModeName(currentDisplayMode));
    flashNeopixelsColor(MODE_TOGGLE_FLASH_R, MODE_TOGGLE_FLASH_G, MODE_TOGGLE_FLASH_B, INVERT_FLASH_MS);
    playModeToggleTone();

    if (hasReading) {
        updateDisplay(lastCO2, lastTempF, lastRH);
    }

    portENTER_CRITICAL(&stateMux);
    displayUpdateInProgress = false;
    portEXIT_CRITICAL(&stateMux);
}

static void applyPendingModeCycleIfAny() {
    bool shouldApply = false;
    portENTER_CRITICAL(&stateMux);
    if (modeCyclePending && !displayUpdateInProgress) {
        modeCyclePending = false;
        displayUpdateInProgress = true;
        shouldApply = true;
    }
    portEXIT_CRITICAL(&stateMux);

    if (!shouldApply) return;

    advanceDisplayMode();
    Serial.printf("Display mode (deferred): %s\n", displayModeName(currentDisplayMode));
    flashNeopixelsColor(MODE_TOGGLE_FLASH_R, MODE_TOGGLE_FLASH_G, MODE_TOGGLE_FLASH_B, INVERT_FLASH_MS);
    playModeToggleTone();

    if (hasReading) {
        updateDisplay(lastCO2, lastTempF, lastRH);
    }

    portENTER_CRITICAL(&stateMux);
    displayUpdateInProgress = false;
    portEXIT_CRITICAL(&stateMux);
}

static void handleCarouselToggleRequest() {
    bool canApplyNow;

    portENTER_CRITICAL(&stateMux);
    canApplyNow = !displayUpdateInProgress;
    if (!canApplyNow) {
        carouselTogglePending = true;
        portEXIT_CRITICAL(&stateMux);
        return;
    }
    displayUpdateInProgress = true;
    portEXIT_CRITICAL(&stateMux);

    carouselModeEnabled = !carouselModeEnabled;
    Serial.printf("Carousel: %s\n", carouselModeEnabled ? "ON" : "OFF");
    if (carouselModeEnabled) {
        flashNeopixelsColor(CAROUSEL_ON_FLASH_R, CAROUSEL_ON_FLASH_G, CAROUSEL_ON_FLASH_B, INVERT_FLASH_MS);
    } else {
        flashNeopixelsColor(CAROUSEL_OFF_FLASH_R, CAROUSEL_OFF_FLASH_G, CAROUSEL_OFF_FLASH_B, INVERT_FLASH_MS);
    }
    playModeToggleTone();

    if (hasReading) {
        updateDisplay(lastCO2, lastTempF, lastRH);
    }

    portENTER_CRITICAL(&stateMux);
    displayUpdateInProgress = false;
    portEXIT_CRITICAL(&stateMux);
}

static void applyPendingCarouselToggleIfAny() {
    bool shouldApply = false;
    portENTER_CRITICAL(&stateMux);
    if (carouselTogglePending && !displayUpdateInProgress) {
        carouselTogglePending = false;
        displayUpdateInProgress = true;
        shouldApply = true;
    }
    portEXIT_CRITICAL(&stateMux);

    if (!shouldApply) return;

    carouselModeEnabled = !carouselModeEnabled;
    Serial.printf("Carousel (deferred): %s\n", carouselModeEnabled ? "ON" : "OFF");
    if (carouselModeEnabled) {
        flashNeopixelsColor(CAROUSEL_ON_FLASH_R, CAROUSEL_ON_FLASH_G, CAROUSEL_ON_FLASH_B, INVERT_FLASH_MS);
    } else {
        flashNeopixelsColor(CAROUSEL_OFF_FLASH_R, CAROUSEL_OFF_FLASH_G, CAROUSEL_OFF_FLASH_B, INVERT_FLASH_MS);
    }
    playModeToggleTone();

    if (hasReading) {
        updateDisplay(lastCO2, lastTempF, lastRH);
    }

    portENTER_CRITICAL(&stateMux);
    displayUpdateInProgress = false;
    portEXIT_CRITICAL(&stateMux);
}

static void modeButtonTask(void *param) {
    (void)param;
    bool pressedPrev = false;
    unsigned long downAt = 0;

    while (true) {
        bool pressed = (digitalRead(MODE_BTN) == LOW);

        if (pressed && !pressedPrev) {
            downAt = millis();
        }

        if (!pressed && pressedPrev) {
            if (millis() - downAt >= 40) {
                handleModeCycleRequest();
            }
        }

        pressedPrev = pressed;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void carouselButtonTask(void *param) {
    (void)param;
    bool pressedPrev = false;
    unsigned long downAt = 0;

    Serial.printf("Carousel task started, GPIO %d, initial=%s\n",
                  CAROUSEL_BTN, digitalRead(CAROUSEL_BTN) == LOW ? "LOW" : "HIGH");

    while (true) {
        bool pressed = (digitalRead(CAROUSEL_BTN) == LOW);

        if (pressed && !pressedPrev) {
            downAt = millis();
            Serial.println("Carousel btn: pressed");
        }

        if (!pressed && pressedPrev) {
            unsigned long held = millis() - downAt;
            Serial.printf("Carousel btn: released (%lu ms)\n", held);
            if (held >= 40) {
                handleCarouselToggleRequest();
            }
        }

        pressedPrev = pressed;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ---------------------------------------------------------------------------
// D11 sleep toggle button task: monitors for 2 s hold to toggle sleep mode.
// Runs only in USB mode as a FreeRTOS task.
// ---------------------------------------------------------------------------
static void sleepToggleButtonTask(void *param) {
    (void)param;
    bool pressedPrev = false;
    unsigned long downAt = 0;
    bool holdFired = false;

    while (true) {
        bool pressed = (digitalRead(SLEEP_TOGGLE_BTN) == LOW);

        if (pressed && !pressedPrev) {
            downAt = millis();
            holdFired = false;
        }

        if (pressed && !holdFired && (millis() - downAt >= SLEEP_TOGGLE_HOLD_MS)) {
            handleSleepToggle();
            holdFired = true;
        }

        if (!pressed && pressedPrev) {
            holdFired = false;
        }

        pressedPrev = pressed;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void flashNeopixelsColor(uint8_t red, uint8_t green, uint8_t blue, uint16_t ms) {
    digitalWrite(NEOPIXEL_POWER, LOW);
    delay(10);
    for (int i = 0; i < 4; i++) {
        pixels.setPixelColor(i, pixels.Color(red, green, blue));
    }
    pixels.show();
    delay(ms);
    pixels.clear();
    pixels.show();
    digitalWrite(NEOPIXEL_POWER, HIGH);
}

// ---------------------------------------------------------------------------
// Flash all four neopixels red for FLASH_DURATION_MS, then turn them off.
// ---------------------------------------------------------------------------
void flashNeopixelsRed() {
    flashNeopixelsColor(GENERIC_RED_FLASH_R, GENERIC_RED_FLASH_G, GENERIC_RED_FLASH_B, FLASH_DURATION_MS);
}

// ---------------------------------------------------------------------------
// Flash only the two center neopixels (indices 1 and 2).
// ---------------------------------------------------------------------------
static void flashCenterNeopixels(uint8_t r, uint8_t g, uint8_t b, uint16_t ms) {
    digitalWrite(NEOPIXEL_POWER, LOW);
    delay(10);
    pixels.clear();
    pixels.setPixelColor(1, pixels.Color(r, g, b));
    pixels.setPixelColor(2, pixels.Color(r, g, b));
    pixels.show();
    delay(ms);
    pixels.clear();
    pixels.show();
    digitalWrite(NEOPIXEL_POWER, HIGH);
}

// ---------------------------------------------------------------------------
// Show "Deep Sleep" / "On" or "Off" status screen, centered on display.
// ---------------------------------------------------------------------------
static void showSleepModeStatus() {
    display.clearBuffer();
    display.fillScreen(bgColor());
    display.setTextColor(fgColor());
    display.setTextWrap(false);

    const char *line1 = "Deep Sleep";
    const char *line2 = deepSleepEnabled ? "On" : "Off";

    int16_t x1, y1;
    uint16_t w1, h1, w2, h2;

    display.setTextSize(2);
    display.getTextBounds(line1, 0, 0, &x1, &y1, &w1, &h1);
    display.getTextBounds(line2, 0, 0, &x1, &y1, &w2, &h2);

    int16_t gap = 8;
    int16_t totalH = (int16_t)h1 + gap + (int16_t)h2;
    int16_t startY = (display.height() - totalH) / 2;

    display.setCursor((display.width() - (int16_t)w1) / 2, startY);
    display.print(line1);

    display.setCursor((display.width() - (int16_t)w2) / 2, startY + (int16_t)h1 + gap);
    display.print(line2);

    display.display();
}

// ---------------------------------------------------------------------------
// Toggle deep/light sleep mode: flash LEDs, beep, show status, redraw.
// ---------------------------------------------------------------------------
static void handleSleepToggle() {
    deepSleepEnabled = !deepSleepEnabled;
    rtcDeepSleepEnabled = deepSleepEnabled;

    Serial.printf("Sleep mode toggled: %s\n", deepSleepEnabled ? "DEEP" : "LIGHT");

    // NeoPixel feedback
    if (deepSleepEnabled) {
        // Deep sleep ON: two center pixels flash dark blue
        flashCenterNeopixels(DEEP_SLEEP_ON_FLASH_R, DEEP_SLEEP_ON_FLASH_G, DEEP_SLEEP_ON_FLASH_B, FLASH_DURATION_MS);
    } else {
        // Deep sleep OFF: all four pixels flash light blue
        flashNeopixelsColor(DEEP_SLEEP_OFF_FLASH_R, DEEP_SLEEP_OFF_FLASH_G, DEEP_SLEEP_OFF_FLASH_B, FLASH_DURATION_MS);
    }

    // Audible feedback
    playModeToggleTone();

    // Reset display modes to defaults for the new sleep type
    currentDisplayMode = DISPLAY_MODE_COMBINED;
    currentDeepDisplayMode = DEEP_DISPLAY_MODE_COMBINED;

    // Show status screen
    showSleepModeStatus();
    delay(2000);

    // Redraw with current data in the appropriate mode.
    // Always restore the display — if no readings yet, show a waiting message
    // so the status screen doesn't persist on the e-ink.
    if (hasReading) {
        if (deepSleepEnabled && !usbPowerPresent) {
            updateDisplayDeepSleep(lastCO2, lastTempF, lastRH);
        } else {
            updateDisplay(lastCO2, lastTempF, lastRH);
        }
        lastDisplayMs = millis();
    } else {
        showStatus("Waiting for data...");
    }
}

// ---------------------------------------------------------------------------
// Helper: right-align text at a given x-right edge, y position.
//   Measures the string width with the current text size, then positions
//   the cursor so the text ends at xRight.
// ---------------------------------------------------------------------------
static void printRightAligned(int16_t xRight, int16_t y, const char *str) {
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(str, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(xRight - (int16_t)w, y);
    display.print(str);
}

// ---------------------------------------------------------------------------
// Draw a tiny line graph.
//   buf/getter: history data, n: number of samples,
//   gx,gy,gw,gh: graph bounding box.
// ---------------------------------------------------------------------------
static void drawGraph(const float *buf, int n,
                      int16_t gx, int16_t gy, int16_t gw, int16_t gh) {
    if (n < 2) return;

    const unsigned long windowMs = GRAPH_WINDOW_MINUTES * 60UL * 1000UL;
    unsigned long nowMs = millis();

    // Determine oldest sample still within the time window.
    int start = n - 1;
    for (int i = n - 1; i >= 0; --i) {
        if (nowMs - histGetTs(i) <= windowMs) {
            start = i;
        } else {
            break;
        }
    }

    int win = n - start;
    if (win < 2) return;

    // Find min/max for auto-scaling
    float lo = histGet(buf, start), hi = lo;
    for (int i = start + 1; i < n; i++) {
        float v = histGet(buf, i);
        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }
    // Ensure there's at least a small range so flat lines centre
    if (hi - lo < 0.1f) { lo -= 0.5f; hi += 0.5f; }

    uint16_t fg = fgColor();
    const int16_t graphBottom = gy + gh - 1;

    // Plot line segments, newest sample pinned to the right edge
    // and the graph shifts left as new samples arrive.
    auto yForVal = [&](float v) -> int16_t {
        float frac = (v - lo) / (hi - lo);               // 0..1
        return gy + gh - 1 - (int16_t)(frac * (gh - 1)); // top = high
    };

#if GRAPH_FILL_MODE != 0
    auto fillUnderSegment = [&](int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
        const uint16_t shade = GRAPH_FILL_SHADE;
        const int16_t graphRight = gx + gw - 1;

        if (x0 == x1) {
            if (x0 < gx || x0 > graphRight) return;
            if (y0 < gy) y0 = gy;
            if (y0 > graphBottom) y0 = graphBottom;
#if GRAPH_FILL_MODE == 1
            display.drawLine(x0, y0, x0, graphBottom, shade);
#elif GRAPH_FILL_MODE == 2
            for (int16_t py = y0; py <= graphBottom; ++py) {
                if ((x0 + py) & 1) display.drawPixel(x0, py, shade);
            }
#elif GRAPH_FILL_MODE == 3
            if (((x0 - gx) % GRAPH_FILL_VLINE_SPACING) == 0)
                display.drawLine(x0, y0, x0, graphBottom, shade);
#elif GRAPH_FILL_MODE == 4
            if (x0 & 1) {
                for (int16_t py = y0; py <= graphBottom; py += 2) {
                    display.drawPixel(x0, py, shade);
                }
            }
#endif
            return;
        }

        int16_t xStart = (x0 < x1) ? x0 : x1;
        int16_t xEnd = (x0 < x1) ? x1 : x0;
        if (xEnd < gx || xStart > graphRight) return;
        if (xStart < gx) xStart = gx;
        if (xEnd > graphRight) xEnd = graphRight;

        float dx = (float)(x1 - x0);
        for (int16_t x = xStart; x <= xEnd; ++x) {
            float t = (dx == 0.0f) ? 0.0f : ((float)(x - x0) / dx);
            int16_t y = (int16_t)(y0 + (y1 - y0) * t);
            if (y < gy) y = gy;
            if (y > graphBottom) y = graphBottom;
#if GRAPH_FILL_MODE == 1
            display.drawLine(x, y, x, graphBottom, shade);
#elif GRAPH_FILL_MODE == 2
            for (int16_t py = y; py <= graphBottom; ++py) {
                if ((x + py) & 1) display.drawPixel(x, py, shade);
            }
#elif GRAPH_FILL_MODE == 3
            if (((x - gx) % GRAPH_FILL_VLINE_SPACING) == 0)
                display.drawLine(x, y, x, graphBottom, shade);
#elif GRAPH_FILL_MODE == 4
            if (x & 1) {
                for (int16_t py = y; py <= graphBottom; py += 2) {
                    display.drawPixel(x, py, shade);
                }
            }
#endif
        }
    };
#endif

    // x positions are based on sample age over GRAPH_WINDOW_MINUTES,
    // so the graph remains truly time-based across variable sample intervals.
    for (int i = 1; i < win; i++) {
        unsigned long age0 = nowMs - histGetTs(start + i - 1);
        unsigned long age1 = nowMs - histGetTs(start + i);
        int16_t x0 = gx + gw - 1 - (int32_t)age0 * (gw - 1) / (int32_t)windowMs;
        int16_t x1 = gx + gw - 1 - (int32_t)age1 * (gw - 1) / (int32_t)windowMs;
        float v0 = histGet(buf, start + i - 1);
        float v1 = histGet(buf, start + i);
        int16_t y0 = yForVal(v0);
        int16_t y1 = yForVal(v1);
#if GRAPH_FILL_MODE != 0
    #if GRAPH_FILL_RISING_ONLY
        if (v1 > v0) {
            fillUnderSegment(x0, y0, x1, y1);
        }
    #else
        fillUnderSegment(x0, y0, x1, y1);
    #endif
#endif
        if (GRAPH_LINE_THICKNESS <= 1) {
            display.drawLine(x0, y0, x1, y1, fg);
        } else {
            int16_t half = GRAPH_LINE_THICKNESS / 2;
            for (int16_t off = -half; off <= half; ++off) {
                display.drawLine(x0, y0 + off, x1, y1 + off, fg);
            }
        }
    }
}

static bool getWindowMinMax(const float *buf, int n, float *outLo, float *outHi) {
    if (n < 1) return false;

    const unsigned long windowMs = GRAPH_WINDOW_MINUTES * 60UL * 1000UL;
    unsigned long nowMs = millis();
    int start = n - 1;
    for (int i = n - 1; i >= 0; --i) {
        if (nowMs - histGetTs(i) <= windowMs) start = i;
        else break;
    }

    float lo = histGet(buf, start);
    float hi = lo;
    for (int i = start + 1; i < n; ++i) {
        float v = histGet(buf, i);
        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }

    *outLo = lo;
    *outHi = hi;
    return true;
}

// ---------------------------------------------------------------------------
// Redraw the e-ink display.
//   Left side:  values right-aligned, vertically centred to each graph row.
//   Right side: three line graphs (CO2 / Temp / RH) over last 30 min.
// ---------------------------------------------------------------------------
void updateDisplay(float co2, float tempF, float rh) {
    uint16_t fg = fgColor();
    int16_t contentLeft = SCREEN_PAD_LEFT;
    int16_t contentTop = SCREEN_PAD_TOP;
    int16_t contentRight = display.width() - 1 - SCREEN_PAD_RIGHT;
    int16_t contentBottom = display.height() - 1 - SCREEN_PAD_BOTTOM;
    int16_t contentW = contentRight - contentLeft + 1;
    int16_t contentH = contentBottom - contentTop + 1;
    if (contentW < 32 || contentH < 32) return;

    display.clearBuffer();
    display.setTextColor(fg);

    char buf[32];
    int n = histSamples();

    if (currentDisplayMode == DISPLAY_MODE_COMBINED) {
        int16_t baseTextRight = graphHeavyLayout ? TEXT_RIGHT_GRAPH_HEAVY : TEXT_RIGHT_BALANCED;
        int16_t baseGraphX = graphHeavyLayout ? GRAPH_X_GRAPH_HEAVY : GRAPH_X_BALANCED;
        int16_t textRight = contentLeft + baseTextRight;
        int16_t graphX = contentLeft + baseGraphX;
        int16_t graphW = contentRight - graphX + 1;
        if (graphW < 24) graphW = 24;
        if (textRight >= graphX) textRight = graphX - 4;

        int16_t totalGraphH = GRAPH_H_CO2 + GRAPH_H_STD + GRAPH_H_STD + GRAPH_GAP + GRAPH_GAP;
        int16_t gyCO2  = contentTop + ((contentH > totalGraphH) ? (contentH - totalGraphH) / 2 : 0);
        int16_t gyTemp = gyCO2  + GRAPH_H_CO2 + GRAPH_GAP;
        int16_t gyRH   = gyTemp + GRAPH_H_STD + GRAPH_GAP;

        display.setTextSize(3);
        snprintf(buf, sizeof(buf), "%d", (int)co2);
        printRightAligned(textRight, gyCO2 + GRAPH_H_CO2 / 2 - 12, buf);

        display.setTextSize(2);
        snprintf(buf, sizeof(buf), "%.1f F", tempF);
        printRightAligned(textRight, gyTemp + GRAPH_H_STD / 2 - 8, buf);

        snprintf(buf, sizeof(buf), "%.1f %%", rh);
        printRightAligned(textRight, gyRH + GRAPH_H_STD / 2 - 8, buf);

        drawGraph(histCO2,   n, graphX, gyCO2,  graphW, GRAPH_H_CO2);
        drawGraph(histTempF, n, graphX, gyTemp, graphW, GRAPH_H_STD);
        drawGraph(histRH,    n, graphX, gyRH,   graphW, GRAPH_H_STD);
    } else if (currentDisplayMode == DISPLAY_MODE_CO2_ONLY ||
               currentDisplayMode == DISPLAY_MODE_TEMP_ONLY ||
               currentDisplayMode == DISPLAY_MODE_RH_ONLY) {
        const float *metricBuf = histCO2;
        float currentVal = co2;
        int decimals = 0;
        const char *suffix = "";
        if (currentDisplayMode == DISPLAY_MODE_TEMP_ONLY) {
            metricBuf = histTempF;
            currentVal = tempF;
            decimals = 1;
            suffix = "F";
        } else if (currentDisplayMode == DISPLAY_MODE_RH_ONLY) {
            metricBuf = histRH;
            currentVal = rh;
            decimals = 1;
            suffix = "%";
        }

        float lo = currentVal, hi = currentVal;
        getWindowMinMax(metricBuf, n, &lo, &hi);

        int16_t textRight = contentLeft + TEXT_RIGHT_GRAPH_HEAVY;
        int16_t graphX = contentLeft + GRAPH_X_GRAPH_HEAVY;
        int16_t graphW = contentRight - graphX + 1;
        if (graphW < 24) graphW = 24;
        if (textRight >= graphX) textRight = graphX - 4;

        int16_t totalGraphH = GRAPH_H_CO2 + GRAPH_H_STD + GRAPH_H_STD + GRAPH_GAP + GRAPH_GAP;
        int16_t gyCO2  = contentTop + ((contentH > totalGraphH) ? (contentH - totalGraphH) / 2 : 0);
        int16_t gyTemp = gyCO2 + GRAPH_H_CO2 + GRAPH_GAP;
        int16_t gyRH   = gyTemp + GRAPH_H_STD + GRAPH_GAP;

        int16_t yCurrent = gyCO2 + GRAPH_H_CO2 / 2 - 12;
        int16_t yHigh = gyTemp + GRAPH_H_STD / 2 - 8;
        int16_t yLow = gyRH + GRAPH_H_STD / 2 - 8;

        if (decimals == 0) snprintf(buf, sizeof(buf), "%d", (int)currentVal);
        else snprintf(buf, sizeof(buf), "%.1f", currentVal);
        {
            // Try size 3 for number; fall back to size 2 if it won't fit
            int16_t x1, y1;
            uint16_t numW, numH, sfxW = 0;
            int numSize = 3;
            display.setTextSize(3);
            display.getTextBounds(buf, 0, 0, &x1, &y1, &numW, &numH);
            if (suffix[0] != '\0') {
                display.setTextSize(2);
                uint16_t sfxH;
                display.getTextBounds(suffix, 0, 0, &x1, &y1, &sfxW, &sfxH);
            }
            if ((int16_t)numW + (int16_t)sfxW > textRight) {
                numSize = 2;
                display.setTextSize(2);
                display.getTextBounds(buf, 0, 0, &x1, &y1, &numW, &numH);
            }
            int16_t totalW = (int16_t)numW + (int16_t)sfxW;
            int16_t xStart = textRight - totalW;
            display.setTextSize(numSize);
            display.setCursor(xStart, yCurrent);
            display.print(buf);
            if (suffix[0] != '\0') {
                display.setTextSize(2);
                // Align baselines: size 3 = 21px, size 2 = 14px
                int16_t yOff = (numSize == 3) ? (21 - 14) : 0;
                display.setCursor(xStart + (int16_t)numW, yCurrent + yOff);
                display.print(suffix);
            }
        }

        display.setTextSize(2);
        if (decimals == 0) snprintf(buf, sizeof(buf), "%d%s", (int)hi, suffix);
        else snprintf(buf, sizeof(buf), "%.1f%s", hi, suffix);
        printRightAligned(textRight, yHigh, buf);

        if (decimals == 0) snprintf(buf, sizeof(buf), "%d%s", (int)lo, suffix);
        else snprintf(buf, sizeof(buf), "%.1f%s", lo, suffix);
        printRightAligned(textRight, yLow, buf);

        drawGraph(metricBuf, n, graphX, contentTop + 2, graphW, contentH - 4);
    } else if (currentDisplayMode == DISPLAY_MODE_TEMP_RH) {
        int16_t textRight = contentLeft + TEXT_RIGHT_GRAPH_HEAVY;
        int16_t graphX = contentLeft + GRAPH_X_GRAPH_HEAVY;
        int16_t graphW = contentRight - graphX + 1;
        if (graphW < 24) graphW = 24;
        if (textRight >= graphX) textRight = graphX - 4;
        int16_t graphGap = 8;
        int16_t halfH = (contentH - graphGap) / 2;
        int16_t upperY = contentTop;
        int16_t lowerY = upperY + halfH + graphGap;

        display.setTextSize(2);
        snprintf(buf, sizeof(buf), "%.1f F", tempF);
        printRightAligned(textRight, upperY + (halfH / 2) - 8, buf);
        snprintf(buf, sizeof(buf), "%.1f %%", rh);
        printRightAligned(textRight, lowerY + (halfH / 2) - 8, buf);

        drawGraph(histTempF, n, graphX, upperY, graphW, halfH);
        drawGraph(histRH,    n, graphX, lowerY, graphW, halfH);
    } else {
        // DISPLAY_MODE_TEXT_ONLY: CO2 left half, temp+RH right half (no graphs)
        int16_t screenW = display.width();
        int16_t screenH = display.height();
        int16_t halfW = screenW / 2;

        // Left half: CO2 + "ppm"
        display.setTextSize(3);
        snprintf(buf, sizeof(buf), "%d", (int)co2);
        int16_t bx, by; uint16_t bw, bh;
        display.getTextBounds(buf, 0, 0, &bx, &by, &bw, &bh);
        uint16_t co2NumW = bw, co2NumH = bh;

        display.setTextSize(2);
        uint16_t ppmW, ppmH;
        display.getTextBounds("ppm", 0, 0, &bx, &by, &ppmW, &ppmH);

        int16_t gap = 4;
        int16_t totalCO2H = (int16_t)co2NumH + gap + (int16_t)ppmH;
        int16_t co2StartY = (screenH - totalCO2H) / 2;

        display.setTextSize(3);
        display.setCursor((halfW - (int16_t)co2NumW) / 2, co2StartY);
        display.print(buf);

        display.setTextSize(2);
        display.setCursor((halfW - (int16_t)ppmW) / 2, co2StartY + (int16_t)co2NumH + gap);
        display.print("ppm");

        // Right half: temp + humidity
        display.setTextSize(2);
        snprintf(buf, sizeof(buf), "%.1f F", tempF);
        display.getTextBounds(buf, 0, 0, &bx, &by, &bw, &bh);
        uint16_t tempW = bw, tempH = bh;

        char rhBuf[32];
        snprintf(rhBuf, sizeof(rhBuf), "%.1f %%", rh);
        uint16_t rhW, rhH;
        display.getTextBounds(rhBuf, 0, 0, &bx, &by, &rhW, &rhH);

        uint16_t maxW = (tempW > rhW) ? tempW : rhW;
        int16_t rightGap = 8;
        int16_t totalRightH = (int16_t)tempH + rightGap + (int16_t)rhH;
        int16_t rightStartY = (screenH - totalRightH) / 2;
        int16_t rightStartX = halfW + (halfW - (int16_t)maxW) / 2;

        display.setCursor(rightStartX, rightStartY);
        display.print(buf);
        display.setCursor(rightStartX, rightStartY + (int16_t)tempH + rightGap);
        display.print(rhBuf);
    }

    display.display();
    flashPendingBatteryTierIfAny();
}

// ---------------------------------------------------------------------------
// Redraw the e-ink display in deep-sleep compact modes (no graphs).
//   Combined: CO2+ppm on left half, temp+RH on right half.
//   Single:   One metric centered with suffix below.
// ---------------------------------------------------------------------------
void updateDisplayDeepSleep(float co2, float tempF, float rh) {
    uint16_t fg = fgColor();
    display.clearBuffer();
    display.fillScreen(bgColor());
    display.setTextColor(fg);
    display.setTextWrap(false);

    char buf[32];
    int16_t x1, y1;
    uint16_t w, h;
    int16_t screenW = display.width();
    int16_t screenH = display.height();

    if (currentDeepDisplayMode == DEEP_DISPLAY_MODE_COMBINED) {
        // ── Left half: CO2 number + "ppm" suffix, centered ──
        int16_t halfW = screenW / 2;

        display.setTextSize(3);
        snprintf(buf, sizeof(buf), "%d", (int)co2);
        display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
        uint16_t co2NumW = w, co2NumH = h;

        display.setTextSize(2);
        uint16_t ppmW, ppmH;
        display.getTextBounds("ppm", 0, 0, &x1, &y1, &ppmW, &ppmH);

        int16_t gap = 4;
        int16_t totalCO2H = (int16_t)co2NumH + gap + (int16_t)ppmH;
        int16_t co2StartY = (screenH - totalCO2H) / 2;

        display.setTextSize(3);
        display.setCursor((halfW - (int16_t)co2NumW) / 2, co2StartY);
        display.print(buf);

        display.setTextSize(2);
        display.setCursor((halfW - (int16_t)ppmW) / 2, co2StartY + (int16_t)co2NumH + gap);
        display.print("ppm");

        // ── Right half: temp + humidity, left-aligned group centered ──
        display.setTextSize(2);
        snprintf(buf, sizeof(buf), "%.1f F", tempF);
        display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
        uint16_t tempW = w, tempH = h;

        char rhBuf[32];
        snprintf(rhBuf, sizeof(rhBuf), "%.1f %%", rh);
        uint16_t rhW, rhH;
        display.getTextBounds(rhBuf, 0, 0, &x1, &y1, &rhW, &rhH);

        uint16_t maxW = (tempW > rhW) ? tempW : rhW;
        int16_t rightGap = 8;
        int16_t totalRightH = (int16_t)tempH + rightGap + (int16_t)rhH;
        int16_t rightStartY = (screenH - totalRightH) / 2;
        int16_t rightStartX = halfW + (halfW - (int16_t)maxW) / 2;

        display.setCursor(rightStartX, rightStartY);
        display.print(buf);

        display.setCursor(rightStartX, rightStartY + (int16_t)tempH + rightGap);
        display.print(rhBuf);

    } else {
        // ── Single metric centered with suffix below ──
        float val;
        const char *suffix;
        int decimals;

        if (currentDeepDisplayMode == DEEP_DISPLAY_MODE_CO2_ONLY) {
            val = co2; suffix = "ppm"; decimals = 0;
        } else if (currentDeepDisplayMode == DEEP_DISPLAY_MODE_TEMP_ONLY) {
            val = tempF; suffix = "F"; decimals = 1;
        } else {
            val = rh; suffix = "%"; decimals = 1;
        }

        if (decimals == 0) snprintf(buf, sizeof(buf), "%d", (int)val);
        else                snprintf(buf, sizeof(buf), "%.1f", val);

        display.setTextSize(3);
        display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
        uint16_t numW = w, numH = h;

        display.setTextSize(2);
        uint16_t sfxW, sfxH;
        display.getTextBounds(suffix, 0, 0, &x1, &y1, &sfxW, &sfxH);

        int16_t gap = 4;
        int16_t totalH = (int16_t)numH + gap + (int16_t)sfxH;
        int16_t startY = (screenH - totalH) / 2;

        display.setTextSize(3);
        display.setCursor((screenW - (int16_t)numW) / 2, startY);
        display.print(buf);

        display.setTextSize(2);
        display.setCursor((screenW - (int16_t)sfxW) / 2, startY + (int16_t)numH + gap);
        display.print(suffix);
    }

    display.display();
    flashPendingBatteryTierIfAny();
}

// ---------------------------------------------------------------------------
// Show a single-line status message on the e-ink display.
// ---------------------------------------------------------------------------
void showStatus(const char *msg) {
    display.clearBuffer();
    display.setTextColor(fgColor());
    display.setTextSize(2);
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);
    int16_t contentW = display.width() - SCREEN_PAD_LEFT - SCREEN_PAD_RIGHT;
    int16_t contentH = display.height() - SCREEN_PAD_TOP - SCREEN_PAD_BOTTOM;
    int16_t x = SCREEN_PAD_LEFT + (contentW - (int16_t)w) / 2;
    int16_t y = SCREEN_PAD_TOP + (contentH - (int16_t)h) / 2;
    display.setCursor(x, y);
    display.print(msg);
    display.display();
}

void showStartupImage() {
#if HAS_STARTUP_IMAGE
    display.clearBuffer();
    display.drawBitmap(0, 0, STARTUP_IMAGE_BITMAP, STARTUP_IMAGE_WIDTH, STARTUP_IMAGE_HEIGHT, fgColor());
    display.display();
#else
    showStatus("Started");
#endif
}

static void showBuildInfoScreen() {
    char lineVersion[32];
    char lineHash[40];
    char lineEpoch[40];
    char lineState[40];

    snprintf(lineVersion, sizeof(lineVersion), "Build: %s", BUILD_VERSION_STR);
    snprintf(lineHash, sizeof(lineHash), "Hash: %s", BUILD_HASH_STR);
    snprintf(lineEpoch, sizeof(lineEpoch), "Epoch: %lu", (unsigned long)BUILD_EPOCH_UNIX);
    snprintf(lineState, sizeof(lineState), "State: %s", BUILD_DIRTY_STR);

    display.clearBuffer();
    display.fillScreen(bgColor());
    display.setTextColor(fgColor());
    display.setTextWrap(false);

    const int16_t baseX = SCREEN_PAD_LEFT + 8;
    const int16_t baseY = SCREEN_PAD_TOP;

    display.setTextSize(2);
    display.setCursor(baseX, baseY + 16);
    display.print("Build Info");

    display.setTextSize(1);
    display.setCursor(baseX, baseY + 52);
    display.print(lineVersion);
    display.setCursor(baseX, baseY + 72);
    display.print(lineHash);
    display.setCursor(baseX, baseY + 92);
    display.print(lineEpoch);
    display.setCursor(baseX, baseY + 112);
    display.print(lineState);
    display.display();

    delay(STARTUP_BUILD_INFO_MS);
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
    // ── CRITICAL: detect wake cause and capture button state BEFORE any delays ──
    // The ext1 wakeup status register is only valid immediately after wake;
    // button may already be released by the time Serial.begin returns.
    esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();
    wokeFromDeepSleep = (wakeCause != ESP_SLEEP_WAKEUP_UNDEFINED);

    // Configure button GPIOs and sample state immediately
    pinMode(MODE_BTN, INPUT_PULLUP);
    pinMode(CAROUSEL_BTN, INPUT_PULLUP);
    pinMode(SLEEP_TOGGLE_BTN, INPUT_PULLUP);

    // Detect which button caused wake from the ext1 status register,
    // then immediately verify the pin is still LOW to filter noise.
    // This runs within microseconds of boot — real presses (>50 ms)
    // are still held; noise glitches are not.
    buttonWakeGPIO = 0;
    if (wakeCause == ESP_SLEEP_WAKEUP_EXT1) {
        uint64_t wakeStatus = esp_sleep_get_ext1_wakeup_status();
        if (wakeStatus & (1ULL << SLEEP_TOGGLE_BTN))  buttonWakeGPIO = SLEEP_TOGGLE_BTN;
        else if (wakeStatus & (1ULL << MODE_BTN))      buttonWakeGPIO = MODE_BTN;
        else if (wakeStatus & (1ULL << CAROUSEL_BTN))  buttonWakeGPIO = CAROUSEL_BTN;

        // Immediate pin check — no delay, just verify the button is actually held
        if (buttonWakeGPIO != 0 && digitalRead(buttonWakeGPIO) != LOW) {
            buttonWakeGPIO = 0;  // spurious ext1 wake from noise
        }
    }

    // Restore RTC-persisted state immediately (no NVS needed)
    if (wokeFromDeepSleep) {
        currentDisplayMode      = rtcDisplayMode;
        carouselModeEnabled     = rtcCarouselEnabled;
        lastCO2                 = rtcLastCO2;
        lastTempF               = rtcLastTempF;
        lastRH                  = rtcLastRH;
        hasReading              = rtcHasReading;
        deepSleepEnabled        = rtcDeepSleepEnabled;
        currentDeepDisplayMode  = rtcDeepDisplayMode;
    }

    // Now safe to start serial (short delay for deep sleep wake)
    Serial.begin(115200);
    if (wokeFromDeepSleep) {
        delay(200);
    } else {
        delay(2000);
    }

    if (buttonWakeGPIO != 0) {
        Serial.printf("Button wake: GPIO %d\n", buttonWakeGPIO);
    }
    Serial.println("MagTag CO2 Monitor starting...");

    // Load visual defaults.
    if (!wokeFromDeepSleep) {
        loadVisualPreferences();
    }
    disableRadios();

    // ── Neopixels ──
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
    pixels.begin();
    pixels.setBrightness(50);
    pixels.clear();
    pixels.show();
    if (!wokeFromDeepSleep) {
        flashNeopixelsColor(STARTUP_FLASH_R, STARTUP_FLASH_G, STARTUP_FLASH_B, STARTUP_FLASH_MS);
    }

    // ── Speaker ──
    pinMode(SPEAKER_SHUTDOWN, OUTPUT);
    digitalWrite(SPEAKER_SHUTDOWN, LOW);
    if (!wokeFromDeepSleep) {
        playStartupJingle();
    }

    // ── Battery monitor ADC setup ──
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // Button GPIOs already configured above (before ext1 debounce)

    // ── LIS3DH accelerometer (on-board) — unused, power down immediately ──
    if (!lis.begin(0x19)) {
        Serial.println("WARNING: LIS3DH not found");
    } else {
        lis.setDataRate(LIS3DH_DATARATE_POWERDOWN);
        Serial.println("LIS3DH powered down (unused)");
    }

    // ── E-ink display ──
    display.begin(THINKINK_MONO);
    if (!wokeFromDeepSleep) {
        showStartupImage();
        Serial.println("Display: Startup image");
        showBuildInfoScreen();
    }

#if CONFIG_TINYUSB_ENABLED
    // Register USB mount/unmount event callbacks for instant power detection
    USB.onEvent(ARDUINO_USB_STARTED_EVENT, [](void *arg, esp_event_base_t base, int32_t id, void *data) {
        usbMounted = true;
        Serial.println("USB event: MOUNTED");
    });
    USB.onEvent(ARDUINO_USB_STOPPED_EVENT, [](void *arg, esp_event_base_t base, int32_t id, void *data) {
        usbMounted = false;
        Serial.println("USB event: UNMOUNTED");
    });
    USB.begin();

    // Seed mount state from current TinyUSB status — on ESP32-S2 with
    // ARDUINO_USB_CDC_ON_BOOT the stack is already running before setup(),
    // so the mount event may have already fired before our callback was
    // registered.  The event callbacks handle subsequent plug/unplug.
    usbMounted = (bool)USB;
    if (usbMounted) {
        Serial.println("USB already mounted at startup");
    }
#endif
    usbPowerPresent = detectUsbPowerPresent();
    float startupBattV = readBatteryVoltage();
    uint8_t startupBattPct = batteryPercentFromVoltage(startupBattV);

    if (usbPowerPresent) {
        resetBatteryTierNotificationsForUsb();
    } else {
        handleBatteryTierTransition(startupBattPct);
    }

    if (!wokeFromDeepSleep) {
        playBatteryStartupAnimation(startupBattPct);
    }

    if (wokeFromDeepSleep && rtcHasUsbPowerState && !rtcLastUsbPowerPresent && usbPowerPresent) {
        flashNeopixelsColor(USB_CONNECTED_FLASH_R, USB_CONNECTED_FLASH_G, USB_CONNECTED_FLASH_B, 180);
        playUsbConnectedJingle();
    }

    rtcLastUsbPowerPresent = usbPowerPresent;
    rtcHasUsbPowerState = true;

    lastSampleMs = millis();
    lastDisplayMs = millis();
    Serial.printf("Power mode: %s\n", usbPowerPresent ? "USB" : "Battery");

    // Button polling tasks are only useful in USB mode where we stay awake.
    // In battery mode, buttons wake the device via ext1 interrupt instead.
    if (usbPowerPresent) {
        xTaskCreate(modeButtonTask, "mode_btn", 8192, nullptr, 1, &modeButtonTaskHandle);
        xTaskCreate(carouselButtonTask, "carousel_btn", 8192, nullptr, 1, &carouselButtonTaskHandle);
        xTaskCreate(sleepToggleButtonTask, "sleep_btn", 8192, nullptr, 1, &sleepToggleTaskHandle);
    }

    // ── Handle D11 (sleep toggle) wake from deep sleep ──
    // Must happen after display + neopixel init but before SCD30 init,
    // so the toggle result can influence whether SCD30 is needed.
    if (wokeFromDeepSleep && buttonWakeGPIO == SLEEP_TOGGLE_BTN) {
        // Check for 2 s hold to toggle sleep mode
        unsigned long holdStart = millis();
        bool toggled = false;
        while (digitalRead(SLEEP_TOGGLE_BTN) == LOW) {
            if (millis() - holdStart >= SLEEP_TOGGLE_HOLD_MS) {
                handleSleepToggle();
                toggled = true;
                while (digitalRead(SLEEP_TOGGLE_BTN) == LOW) delay(20);
                break;
            }
            delay(20);
        }
        buttonWakeGPIO = 0;  // consumed
        Serial.printf("D11 wake: %s\n", toggled ? "toggled sleep mode" : "released early");
    }

    if (!wokeFromDeepSleep) {
        delay(5000);
    }

    // ── I2C + SCD-30 sensor ──
    Wire.begin(SDA, SCL);

    // In battery deep-sleep mode, only start the SCD30 when it's actually
    // sample time.  In light-sleep mode or USB, always start it.
    bool needScd30 = usbPowerPresent || !wokeFromDeepSleep || !deepSleepEnabled
                     || batterySampleElapsedMs >= BATTERY_SAMPLE_INTERVAL_MS;

    if (needScd30) {
        if (!scd30.begin()) {
            Serial.println("ERROR: SCD-30 not found – check wiring");
            showStatus("Sensor not found!");
            while (true) { delay(1000); }
        }
        scd30Ready = true;
        Serial.println("SCD-30 initialised");

        // After deep sleep the sensor was fully off; it needs ~25 s of
        // continuous measurement before the readings are accurate.
        // Poll buttons during warm-up so the UI stays responsive.
        if (!usbPowerPresent && wokeFromDeepSleep) {
            Serial.printf("SCD30 warm-up: %d ms (buttons active)\n", SCD30_WARMUP_MS);
            delayWithButtonPolling(SCD30_WARMUP_MS);
        }
    } else {
        Serial.println("SCD30 skipped (poll-only wake)");
    }
}

// ---------------------------------------------------------------------------
// Main loop – read → flash → display → sleep, every 20 s.
// ---------------------------------------------------------------------------
void loop() {
    unsigned long now = millis();

    bool usbNow = detectUsbPowerPresent();
    if (usbNow != usbPowerPresent) {
        usbPowerPresent = usbNow;
        float battVNow = readBatteryVoltage();
        uint8_t battPctNow = batteryPercentFromVoltage(battVNow);
        Serial.printf("Power mode changed: %s\n", usbPowerPresent ? "USB" : "Battery");
        if (usbPowerPresent) {
            resetBatteryTierNotificationsForUsb();
            flashNeopixelsColor(USB_CONNECTED_FLASH_R, USB_CONNECTED_FLASH_G, USB_CONNECTED_FLASH_B, 180);
            playUsbConnectedJingle();

            // Recreate button tasks for USB mode (if not already running)
            if (modeButtonTaskHandle == nullptr)
                xTaskCreate(modeButtonTask, "mode_btn", 8192, nullptr, 1, &modeButtonTaskHandle);
            if (carouselButtonTaskHandle == nullptr)
                xTaskCreate(carouselButtonTask, "carousel_btn", 8192, nullptr, 1, &carouselButtonTaskHandle);
            if (sleepToggleTaskHandle == nullptr)
                xTaskCreate(sleepToggleButtonTask, "sleep_btn", 8192, nullptr, 1, &sleepToggleTaskHandle);
        } else {
            // Delete USB button tasks — battery mode uses ext1 wake instead
            if (modeButtonTaskHandle != nullptr) {
                vTaskDelete(modeButtonTaskHandle);
                modeButtonTaskHandle = nullptr;
            }
            if (carouselButtonTaskHandle != nullptr) {
                vTaskDelete(carouselButtonTaskHandle);
                carouselButtonTaskHandle = nullptr;
            }
            if (sleepToggleTaskHandle != nullptr) {
                vTaskDelete(sleepToggleTaskHandle);
                sleepToggleTaskHandle = nullptr;
            }
            handleBatteryTierTransition(battPctNow);
            flashBatteryLedTier(batteryLedTierFromPercent(battPctNow), 1, BATTERY_LEVEL_FLASH_ON_MS, BATTERY_LEVEL_FLASH_OFF_MS);
            playUsbDisconnectedJingle();
        }
    }

    if (!usbPowerPresent) {
        float battV = readBatteryVoltage();
        uint8_t battPct = batteryPercentFromVoltage(battV);
        handleBatteryTierTransition(battPct);

        if (deepSleepEnabled) {
            // =================================================================
            // DEEP SLEEP BATTERY PATH — compact display, no graph samples
            // =================================================================
            const uint32_t sleepPollMs = BATTERY_USB_POLL_INTERVAL_MS;

            // Handle button wake: mode/carousel apply immediately
            if (buttonWakeGPIO != 0 && buttonWakeGPIO != SLEEP_TOGGLE_BTN) {
                applyButtonAction(buttonWakeGPIO);
                buttonWakeGPIO = 0;
            } else {
                buttonWakeGPIO = 0;
            }

            // ── Battery warnings ──
            bool showWarn50 = false;
            bool showCriticalWarn = false;

            if (battPct <= BATTERY_CRITICAL_PERCENT) {
                uint32_t newElapsed = batteryCriticalElapsedMs + sleepPollMs;
                if (newElapsed < batteryCriticalElapsedMs) newElapsed = BATTERY_CRITICAL_INTERVAL_MS;
                batteryCriticalElapsedMs = newElapsed;
                if (batteryCriticalElapsedMs >= BATTERY_CRITICAL_INTERVAL_MS) {
                    showCriticalWarn = true;
                    batteryCriticalElapsedMs = 0;
                }
            } else {
                batteryCriticalElapsedMs = 0;
            }

            if (battPct <= BATTERY_WARN_50_PERCENT && battPct > BATTERY_CRITICAL_PERCENT) {
                if (!batteryWarn50Shown) { showWarn50 = true; batteryWarn50Shown = true; }
            } else if (battPct > BATTERY_WARN_50_PERCENT) {
                batteryWarn50Shown = false;
            }

            if (showCriticalWarn) {
                Serial.printf("Critical battery warning: %.2fV (%u%%)\n", battV, battPct);
                showBatteryWarningMessage("Battery is at 10%,", "please recharge!", BATTERY_CRITICAL_DURATION_MS, true);
            } else if (showWarn50) {
                Serial.printf("Battery 50%% warning: %.2fV (%u%%)\n", battV, battPct);
                showBatteryWarningMessage("Battery at 50%", "", BATTERY_WARN_50_DURATION_MS, false);
            }

            // ── Sample timing (poll-only wakes don't read sensor) ──
            if (batterySampleElapsedMs < BATTERY_SAMPLE_INTERVAL_MS) {
                batterySampleElapsedMs += sleepPollMs;
                if (batterySampleElapsedMs > BATTERY_SAMPLE_INTERVAL_MS)
                    batterySampleElapsedMs = BATTERY_SAMPLE_INTERVAL_MS;
                enterDeepSleepMs(sleepPollMs);
                return;
            }
            batterySampleElapsedMs = 0;

            if (!scd30Ready) {
                Serial.println("SCD30 not ready at sample time – sleeping");
                enterDeepSleepMs(sleepPollMs);
                return;
            }

            // Wait for sensor data, poll buttons
            unsigned long waitStart = millis();
            while (!scd30.dataReady()) {
                if (millis() - waitStart > 5000) {
                    Serial.println("Timeout waiting for SCD-30 data");
                    break;
                }
                for (uint8_t pin : {MODE_BTN, CAROUSEL_BTN}) {
                    if (digitalRead(pin) == LOW) {
                        delay(40);
                        if (digitalRead(pin) == LOW) {
                            applyButtonAction(pin);
                            while (digitalRead(pin) == LOW) delay(20);
                        }
                    }
                }
                delay(50);
            }

            if (scd30.read()) {
                float co2   = scd30.CO2;
                float tempC = scd30.temperature;
                float tempF = tempC * 9.0f / 5.0f + 32.0f;
                float rh    = scd30.relative_humidity;

                if (co2 > 0.0f) {
                    Serial.printf("CO2: %.0f ppm | Temp: %.1f F | RH: %.1f %% | Batt: %.2fV (%u%%)\n",
                                   co2, tempF, rh, battV, battPct);

                    // Deep sleep: do NOT push graph samples (preserves NVS longevity)
                    bool firstBatteryDisplay = (batterySampleCycles == 0);
                    batterySampleCycles++;
                    bool periodicDisplay = ((batterySampleCycles % BATTERY_DISPLAY_EVERY_N) == 0);

                    lastCO2 = co2; lastTempF = tempF; lastRH = rh;
                    hasReading = true;

                    if (firstBatteryDisplay || periodicDisplay) {
                        updateDisplayDeepSleep(co2, tempF, rh);
                        lastDisplayMs = millis();
                        if (carouselModeEnabled) advanceDeepDisplayMode();
                    }
                }
            } else {
                Serial.println("Failed to read SCD-30");
            }

            enterDeepSleepMs(sleepPollMs);
            return;

        } else {
            // =================================================================
            // LIGHT SLEEP BATTERY PATH — full display with graphs, RAM preserved
            // =================================================================

            // Handle button input after light sleep wake.
            // Poll GPIO states directly rather than relying on
            // esp_sleep_get_ext1_wakeup_status() which can be unreliable
            // after light sleep on ESP32-S2 (timer wakes can mask the
            // ext1 cause, and the status register may not populate).
            if (lightSleepWakePending) {
                lightSleepWakePending = false;

                // D11 sleep toggle: requires 2 s hold, so must still be LOW
                if (digitalRead(SLEEP_TOGGLE_BTN) == LOW) {
                    unsigned long holdStart = millis();
                    bool toggled = false;
                    while (digitalRead(SLEEP_TOGGLE_BTN) == LOW) {
                        if (millis() - holdStart >= SLEEP_TOGGLE_HOLD_MS) {
                            handleSleepToggle();
                            toggled = true;
                            while (digitalRead(SLEEP_TOGGLE_BTN) == LOW) delay(20);
                            break;
                        }
                        delay(20);
                    }
                    if (toggled && deepSleepEnabled) {
                        enterDeepSleepMs(BATTERY_USB_POLL_INTERVAL_MS);
                        return;
                    }
                }
                // MODE / CAROUSEL: check GPIO first (button still held),
                // then fall back to ext1 status (quick press-and-release).
                else if (digitalRead(MODE_BTN) == LOW) {
                    while (digitalRead(MODE_BTN) == LOW) delay(20);
                    delay(50);
                    applyButtonAction(MODE_BTN);
                } else if (digitalRead(CAROUSEL_BTN) == LOW) {
                    while (digitalRead(CAROUSEL_BTN) == LOW) delay(20);
                    delay(50);
                    applyButtonAction(CAROUSEL_BTN);
                } else {
                    // Button already released — use ext1 status as fallback
                    esp_sleep_wakeup_cause_t lightWake = esp_sleep_get_wakeup_cause();
                    if (lightWake == ESP_SLEEP_WAKEUP_EXT1) {
                        uint64_t ws = esp_sleep_get_ext1_wakeup_status();
                        // D11 released = ignore (requires hold); only handle short-press buttons
                        if (ws & (1ULL << MODE_BTN)) {
                            delay(50);
                            applyButtonAction(MODE_BTN);
                        } else if (ws & (1ULL << CAROUSEL_BTN)) {
                            delay(50);
                            applyButtonAction(CAROUSEL_BTN);
                        }
                    }
                }
            }

            // Also handle residual button wake from deep→light transition
            if (buttonWakeGPIO != 0 && buttonWakeGPIO != SLEEP_TOGGLE_BTN) {
                applyButtonAction(buttonWakeGPIO);
                buttonWakeGPIO = 0;
            } else {
                buttonWakeGPIO = 0;
            }

            // ── Battery warnings (millis-based, works across light sleep) ──
            static unsigned long lastCriticalWarnMs = 0;
            bool showWarn50 = false;
            bool showCriticalWarn = false;

            if (battPct <= BATTERY_CRITICAL_PERCENT) {
                if (lastCriticalWarnMs == 0 || now - lastCriticalWarnMs >= BATTERY_CRITICAL_INTERVAL_MS) {
                    showCriticalWarn = true;
                    lastCriticalWarnMs = now;
                }
            } else {
                lastCriticalWarnMs = 0;
            }

            if (battPct <= BATTERY_WARN_50_PERCENT && battPct > BATTERY_CRITICAL_PERCENT) {
                if (!batteryWarn50Shown) { showWarn50 = true; batteryWarn50Shown = true; }
            } else if (battPct > BATTERY_WARN_50_PERCENT) {
                batteryWarn50Shown = false;
            }

            if (showCriticalWarn) {
                Serial.printf("Critical battery warning: %.2fV (%u%%)\n", battV, battPct);
                showBatteryWarningMessage("Battery is at 10%,", "please recharge!", BATTERY_CRITICAL_DURATION_MS, true);
            } else if (showWarn50) {
                Serial.printf("Battery 50%% warning: %.2fV (%u%%)\n", battV, battPct);
                showBatteryWarningMessage("Battery at 50%", "", BATTERY_WARN_50_DURATION_MS, false);
            }

            // ── Sample + display on schedule (millis survives light sleep) ──
            if (now - lastSampleMs >= BATTERY_SAMPLE_INTERVAL_MS) {
                lastSampleMs = now;

                if (scd30Ready) {
                    unsigned long waitStart = millis();
                    while (!scd30.dataReady()) {
                        if (millis() - waitStart > 5000) {
                            Serial.println("Timeout waiting for SCD-30 data");
                            break;
                        }
                        delay(50);
                    }

                    if (scd30.read()) {
                        float co2   = scd30.CO2;
                        float tempC = scd30.temperature;
                        float tempF = tempC * 9.0f / 5.0f + 32.0f;
                        float rh    = scd30.relative_humidity;

                        if (co2 > 0.0f) {
                            Serial.printf("CO2: %.0f ppm | Temp: %.1f F | RH: %.1f %% | Batt: %.2fV (%u%%)\n",
                                           co2, tempF, rh, battV, battPct);

                            pushSample(co2, tempF, rh);
                            lastCO2 = co2; lastTempF = tempF; lastRH = rh;
                            hasReading = true;

                            bool firstDisplay = (lastDisplayMs == 0);
                            if (firstDisplay || now - lastDisplayMs >= BATTERY_DISPLAY_INTERVAL_MS) {
                                updateDisplay(co2, tempF, rh);
                                lastDisplayMs = millis();
                                if (carouselModeEnabled) advanceDisplayMode();
                            }
                        }
                    } else {
                        Serial.println("Failed to read SCD-30");
                    }
                }
            }

            // ── Light sleep until next event ──
            uint32_t elapsed = millis() - lastSampleMs;
            uint32_t remaining = (elapsed < BATTERY_SAMPLE_INTERVAL_MS)
                                 ? (BATTERY_SAMPLE_INTERVAL_MS - elapsed) : 0;
            uint32_t sleepFor = remaining;
            if (sleepFor > BATTERY_USB_POLL_INTERVAL_MS) sleepFor = BATTERY_USB_POLL_INTERVAL_MS;
            if (sleepFor < 1000) sleepFor = 1000;

            enterLightSleepMs(sleepFor);
            return;
        }
    }

    uint32_t sampleIntervalMs = currentSampleIntervalMs();
    uint32_t displayIntervalMs = currentDisplayIntervalMs();
    bool shouldSample = (now - lastSampleMs >= sampleIntervalMs);

    if (shouldSample) {
        lastSampleMs = now;

        // Wait for sensor data (up to 5 s)
        unsigned long waitStart = millis();
        while (!scd30.dataReady()) {
            if (millis() - waitStart > 5000) {
                Serial.println("Timeout waiting for SCD-30 data");
                break;
            }
            delay(50);
        }

        if (scd30.read()) {
            float co2   = scd30.CO2;
            float tempC = scd30.temperature;
            float tempF = tempC * 9.0f / 5.0f + 32.0f;
            float rh    = scd30.relative_humidity;

            if (co2 <= 0.0f) {
                Serial.println("Ignoring invalid CO2 reading (0 ppm)");
            } else {
                {
                    float uBattV = readBatteryVoltage();
                    uint8_t uBattPct = batteryPercentFromVoltage(uBattV);
                    handleBatteryTierTransition(uBattPct);
                    Serial.printf("CO2: %.0f ppm | Temp: %.1f F | RH: %.1f %% | Batt: %.2fV (%u%%)\n",
                                   co2, tempF, rh, uBattV, uBattPct);
                }

                portENTER_CRITICAL(&stateMux);
                displayUpdateInProgress = true;
                portEXIT_CRITICAL(&stateMux);

                pushSample(co2, tempF, rh);
                bool firstDisplay = !hasReading;
                lastCO2 = co2; lastTempF = tempF; lastRH = rh;
                hasReading = true;
                flashSampleIndicator();

                bool shouldDisplay = firstDisplay || (millis() - lastDisplayMs >= displayIntervalMs);
                if (shouldDisplay) {
                    updateDisplay(co2, tempF, rh);
                    lastDisplayMs = millis();
                    if (carouselModeEnabled) {
                        advanceDisplayMode();
                    }
                }

                portENTER_CRITICAL(&stateMux);
                displayUpdateInProgress = false;
                portEXIT_CRITICAL(&stateMux);

                applyPendingModeCycleIfAny();
                applyPendingCarouselToggleIfAny();
            }
        } else {
            Serial.println("Failed to read SCD-30");
        }
    }

    applyPendingModeCycleIfAny();
    applyPendingCarouselToggleIfAny();

    // Keep loop responsive for power-mode transitions and deferred toggles
    delay(100);
}
