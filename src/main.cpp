#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SCD30.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_ThinkInk.h>
#include <Adafruit_LIS3DH.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <USB.h>
#include <esp_sleep.h>
#include <Preferences.h>
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

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
#define CYCLE_INTERVAL_MS  20000   // 20 seconds between readings
#define FLASH_DURATION_MS    500   // neopixel flash length
#define STARTUP_FLASH_MS     220
#define INVERT_FLASH_MS      150
#define GRAPH_WINDOW_MINUTES 15
#define GRAPH_WINDOW_SAMPLES ((GRAPH_WINDOW_MINUTES * 60 * 1000) / CYCLE_INTERVAL_MS)

#define USB_SAMPLE_INTERVAL_MS      15000
#define USB_DISPLAY_INTERVAL_MS     60000
#define BATTERY_SAMPLE_INTERVAL_MS  60000
#define BATTERY_DISPLAY_INTERVAL_MS 300000
#define BATTERY_USB_POLL_INTERVAL_MS 10000
#define POWER_CHECK_INTERVAL_MS      5000
#define USB_SAMPLE_FLASH_R             80
#define USB_SAMPLE_FLASH_G              0
#define USB_SAMPLE_FLASH_B            120
#define LOW_BATTERY_PERCENT            10
#define LOW_BATTERY_FLASH_INTERVAL_MS 900000UL
#define SCD30_I2C_ADDR                0x61
#define SCD30_CMD_STOP_MEASUREMENTS   0x0104
#define GRAPH_LINE_THICKNESS           1

#define BATTERY_DISPLAY_EVERY_N ((BATTERY_DISPLAY_INTERVAL_MS / BATTERY_SAMPLE_INTERVAL_MS) > 0 ? (BATTERY_DISPLAY_INTERVAL_MS / BATTERY_SAMPLE_INTERVAL_MS) : 1)

// ---------------------------------------------------------------------------
// History ring buffer  (30 min @ 20 s/sample = 90 samples)
// ---------------------------------------------------------------------------
#define HISTORY_LEN 90
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

// ---------------------------------------------------------------------------
// Display colour-scheme toggle  (D15 long-press)
// ---------------------------------------------------------------------------
#define INVERT_BTN   15           // D15 button
#define MODE_BTN     12           // D12 button
#define CAROUSEL_BTN 14           // D14 button
#define LONG_PRESS_MS 2000
static bool     inverted       = false;
static bool     graphHeavyLayout = true;

enum DisplayMode : uint8_t {
    DISPLAY_MODE_COMBINED = 0,
    DISPLAY_MODE_CO2_ONLY,
    DISPLAY_MODE_TEMP_ONLY,
    DISPLAY_MODE_RH_ONLY,
    DISPLAY_MODE_TEMP_RH,
    DISPLAY_MODE_COUNT
};

static uint8_t currentDisplayMode = DISPLAY_MODE_COMBINED;
static bool carouselModeEnabled = false;

// Last displayed values (for immediate redraw on invert toggle)
static float lastCO2   = 0;
static float lastTempF = 0;
static float lastRH    = 0;
static bool  hasReading = false;
static bool  displayUpdateInProgress = false;
static bool  invertTogglePending = false;
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
RTC_DATA_ATTR static uint32_t batterySampleCycles = 0;
RTC_DATA_ATTR static uint32_t lowBatteryElapsedMs = 0;
RTC_DATA_ATTR static uint32_t batterySampleElapsedMs = BATTERY_SAMPLE_INTERVAL_MS;
static bool wokeFromDeepSleep = false;
static Preferences prefs;
static bool prefsReady = false;

#define PREFS_NAMESPACE "magco2"
#define PREF_KEY_INVERT "invert"

static uint16_t fgColor() { return inverted ? EPD_WHITE : EPD_BLACK; }
static uint16_t bgColor() { return inverted ? EPD_BLACK : EPD_WHITE; }

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
    if (!prefsReady) return;
    inverted = prefs.getBool(PREF_KEY_INVERT, false);
    graphHeavyLayout = true;
    Serial.printf("Loaded prefs: invert=%s layout=GRAPH_HEAVY\n", inverted ? "ON" : "OFF");
}

static void saveVisualPreferences() {
    if (!prefsReady) return;
    prefs.putBool(PREF_KEY_INVERT, inverted);
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

static void playInvertToggleTone() {
    playToneOnce(1318, 80);
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
#if CONFIG_TINYUSB_ENABLED
    bool usbEnum = (bool)USB;
#else
    bool usbEnum = false;
#endif

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

    if (usbEnum) {
        usbByHeuristic = true;
        usbHeuristicScore = 3;
    }

    return usbEnum || usbByHeuristic;
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

static void flashNeopixelsColor(uint8_t red, uint8_t green, uint8_t blue, uint16_t ms);

static void flashSampleIndicator() {
    if (usbPowerPresent) {
        flashNeopixelsColor(USB_SAMPLE_FLASH_R, USB_SAMPLE_FLASH_G, USB_SAMPLE_FLASH_B, FLASH_DURATION_MS);
    }
}

static void enterDeepSleepMs(uint32_t sleepMs) {
    bool stopOk = stopScd30Measurements();
    if (!stopOk) {
        Serial.println("Warning: failed to send SCD30 stop command");
    } else {
        Serial.println("SCD30 measurement stopped before deep sleep");
    }

    disableRadios();
    digitalWrite(NEOPIXEL_POWER, HIGH);
    digitalWrite(SPEAKER_SHUTDOWN, LOW);

    Serial.printf("Battery mode deep sleep for %lu ms\n", (unsigned long)sleepMs);
    Serial.flush();
    esp_sleep_enable_timer_wakeup((uint64_t)sleepMs * 1000ULL);
    esp_deep_sleep_start();
}

// Forward declaration for toggle redraw logic
void updateDisplay(float co2, float tempF, float rh);
static void flashNeopixelsColor(uint8_t red, uint8_t green, uint8_t blue, uint16_t ms);

static const char *displayModeName(uint8_t mode) {
    switch (mode) {
        case DISPLAY_MODE_COMBINED: return "COMBINED";
        case DISPLAY_MODE_CO2_ONLY: return "CO2";
        case DISPLAY_MODE_TEMP_ONLY: return "TEMP";
        case DISPLAY_MODE_RH_ONLY: return "HUMIDITY";
        case DISPLAY_MODE_TEMP_RH: return "TEMP_RH";
        default: return "UNKNOWN";
    }
}

static void advanceDisplayMode() {
    currentDisplayMode = (uint8_t)((currentDisplayMode + 1) % DISPLAY_MODE_COUNT);
}

static void handleInvertToggleRequest() {
    bool canApplyNow;

    portENTER_CRITICAL(&stateMux);
    canApplyNow = !displayUpdateInProgress;
    if (!canApplyNow) {
        invertTogglePending = true;
        portEXIT_CRITICAL(&stateMux);
        return;
    }
    displayUpdateInProgress = true;
    portEXIT_CRITICAL(&stateMux);

    inverted = !inverted;
    saveVisualPreferences();
    Serial.printf("Display inverted: %s\n", inverted ? "ON" : "OFF");
    flashNeopixelsColor(24, 24, 24, INVERT_FLASH_MS);
    playInvertToggleTone();

    if (hasReading) {
        updateDisplay(lastCO2, lastTempF, lastRH);
    }

    portENTER_CRITICAL(&stateMux);
    displayUpdateInProgress = false;
    portEXIT_CRITICAL(&stateMux);
}

static void applyPendingInvertToggleIfAny() {
    bool shouldApply = false;
    portENTER_CRITICAL(&stateMux);
    if (invertTogglePending && !displayUpdateInProgress) {
        invertTogglePending = false;
        displayUpdateInProgress = true;
        shouldApply = true;
    }
    portEXIT_CRITICAL(&stateMux);

    if (!shouldApply) return;

    inverted = !inverted;
    saveVisualPreferences();
    Serial.printf("Display inverted (deferred): %s\n", inverted ? "ON" : "OFF");
    flashNeopixelsColor(24, 24, 24, INVERT_FLASH_MS);
    playInvertToggleTone();
    if (hasReading) {
        updateDisplay(lastCO2, lastTempF, lastRH);
    }

    portENTER_CRITICAL(&stateMux);
    displayUpdateInProgress = false;
    portEXIT_CRITICAL(&stateMux);
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
    flashNeopixelsColor(24, 24, 24, INVERT_FLASH_MS);
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
    flashNeopixelsColor(24, 24, 24, INVERT_FLASH_MS);
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
        flashNeopixelsColor(140, 0, 110, INVERT_FLASH_MS);
    } else {
        flashNeopixelsColor(255, 0, 0, INVERT_FLASH_MS);
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
        flashNeopixelsColor(140, 0, 110, INVERT_FLASH_MS);
    } else {
        flashNeopixelsColor(255, 0, 0, INVERT_FLASH_MS);
    }
    playModeToggleTone();

    if (hasReading) {
        updateDisplay(lastCO2, lastTempF, lastRH);
    }

    portENTER_CRITICAL(&stateMux);
    displayUpdateInProgress = false;
    portEXIT_CRITICAL(&stateMux);
}

static void invertButtonTask(void *param) {
    (void)param;
    bool pressedPrev = false;
    bool longPressFired = false;
    unsigned long downAt = 0;

    while (true) {
        bool pressed = (digitalRead(INVERT_BTN) == LOW);

        if (pressed && !pressedPrev) {
            downAt = millis();
            longPressFired = false;
        }

        if (pressed && !longPressFired && (millis() - downAt >= LONG_PRESS_MS)) {
            longPressFired = true;
            handleInvertToggleRequest();
        }

        if (!pressed) {
            longPressFired = false;
        }

        pressedPrev = pressed;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
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

    while (true) {
        bool pressed = (digitalRead(CAROUSEL_BTN) == LOW);

        if (pressed && !pressedPrev) {
            downAt = millis();
        }

        if (!pressed && pressedPrev) {
            if (millis() - downAt >= 40) {
                handleCarouselToggleRequest();
            }
        }

        pressedPrev = pressed;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ---------------------------------------------------------------------------
// Globals  (board-defined pins from MagTag variant header)
// ---------------------------------------------------------------------------
Adafruit_SCD30  scd30;
Adafruit_NeoPixel pixels(4, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
ThinkInk_290_Grayscale4_EAAMFGN display(EPD_DC, EPD_RESET, EPD_CS, -1, -1);
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

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
    flashNeopixelsColor(255, 0, 0, FLASH_DURATION_MS);
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

    // Plot line segments, newest sample pinned to the right edge
    // and the graph shifts left as new samples arrive.
    auto yForVal = [&](float v) -> int16_t {
        float frac = (v - lo) / (hi - lo);               // 0..1
        return gy + gh - 1 - (int16_t)(frac * (gh - 1)); // top = high
    };

    // x positions are based on sample age over fixed 15-minute window,
    // so the graph remains truly time-based across variable sample intervals.
    for (int i = 1; i < win; i++) {
        unsigned long age0 = nowMs - histGetTs(start + i - 1);
        unsigned long age1 = nowMs - histGetTs(start + i);
        int16_t x0 = gx + gw - 1 - (int32_t)age0 * (gw - 1) / (int32_t)windowMs;
        int16_t x1 = gx + gw - 1 - (int32_t)age1 * (gw - 1) / (int32_t)windowMs;
        int16_t y0 = yForVal(histGet(buf, start + i - 1));
        int16_t y1 = yForVal(histGet(buf, start + i));
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
// Layout constants  (screen is 296 x 128)
#define TEXT_RIGHT_BALANCED   140
#define GRAPH_X_BALANCED      152
#define GRAPH_W_BALANCED      136

#define TEXT_RIGHT_GRAPH_HEAVY 88
#define GRAPH_X_GRAPH_HEAVY    100
#define GRAPH_W_GRAPH_HEAVY    188

// CO2 graph is taller to match the larger text
#define GRAPH_H_CO2  40
#define GRAPH_H_STD  30           // temp & RH graphs
#define GRAPH_GAP    6
// Total: 40 + 30 + 30 + 2*6 = 112.  Centred: (128-112)/2 = 8
#define GRAPH_Y0     8

// Screen-edge content padding for enclosure bezel compensation
#define SCREEN_PAD_LEFT   0
#define SCREEN_PAD_RIGHT  0
#define SCREEN_PAD_TOP    0
#define SCREEN_PAD_BOTTOM 0

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
    if (inverted) display.fillScreen(EPD_BLACK);
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
        if (currentDisplayMode == DISPLAY_MODE_TEMP_ONLY) {
            metricBuf = histTempF;
            currentVal = tempF;
            decimals = 1;
        } else if (currentDisplayMode == DISPLAY_MODE_RH_ONLY) {
            metricBuf = histRH;
            currentVal = rh;
            decimals = 1;
        }

        float lo = currentVal, hi = currentVal;
        getWindowMinMax(metricBuf, n, &lo, &hi);

        int16_t textRight = contentLeft + TEXT_RIGHT_GRAPH_HEAVY;
        int16_t graphX = contentLeft + GRAPH_X_GRAPH_HEAVY;
        int16_t graphW = contentRight - graphX + 1;
        if (graphW < 24) graphW = 24;
        if (textRight >= graphX) textRight = graphX - 4;
        int16_t yCurrent = contentTop + 6;
        int16_t yHigh = contentTop + (contentH / 2) - 8;
        int16_t yLow = contentBottom - 20;

        display.setTextSize(3);
        if (decimals == 0) snprintf(buf, sizeof(buf), "%d", (int)currentVal);
        else snprintf(buf, sizeof(buf), "%.1f", currentVal);
        printRightAligned(textRight, yCurrent, buf);

        display.setTextSize(2);
        if (decimals == 0) snprintf(buf, sizeof(buf), "%d", (int)hi);
        else snprintf(buf, sizeof(buf), "%.1f", hi);
        printRightAligned(textRight, yHigh, buf);

        if (decimals == 0) snprintf(buf, sizeof(buf), "%d", (int)lo);
        else snprintf(buf, sizeof(buf), "%.1f", lo);
        printRightAligned(textRight, yLow, buf);

        drawGraph(metricBuf, n, graphX, contentTop + 2, graphW, contentH - 4);
    } else {
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
    }

    display.display();
}

// ---------------------------------------------------------------------------
// Show a single-line status message on the e-ink display.
// ---------------------------------------------------------------------------
void showStatus(const char *msg) {
    display.clearBuffer();
    display.setTextColor(fgColor());
    if (inverted) display.fillScreen(EPD_BLACK);
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
    if (inverted) display.fillScreen(EPD_BLACK);
    display.drawBitmap(0, 0, STARTUP_IMAGE_BITMAP, STARTUP_IMAGE_WIDTH, STARTUP_IMAGE_HEIGHT, fgColor());
    display.display();
#else
    showStatus("Started");
#endif
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("MagTag CO2 Monitor starting...");
    wokeFromDeepSleep = (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED);

    prefsReady = prefs.begin(PREFS_NAMESPACE, false);
    if (!prefsReady) {
        Serial.println("WARNING: failed to open preferences namespace");
    }
    loadVisualPreferences();
    disableRadios();

    // ── Neopixels ──
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
    pixels.begin();
    pixels.setBrightness(50);
    pixels.clear();
    pixels.show();
    if (!wokeFromDeepSleep) {
        flashNeopixelsColor(0, 180, 0, STARTUP_FLASH_MS);
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

    // ── Invert button (D15) ──
    pinMode(INVERT_BTN, INPUT_PULLUP);
    // ── Mode cycle button (D12) ──
    pinMode(MODE_BTN, INPUT_PULLUP);
    // ── Carousel toggle button (D14) ──
    pinMode(CAROUSEL_BTN, INPUT_PULLUP);

    // ── LIS3DH accelerometer (on-board) ──
    if (!lis.begin(0x19)) {
        Serial.println("WARNING: LIS3DH not found");
    } else {
        Serial.println("LIS3DH initialised");
    }

    // ── E-ink display ──
    display.begin(THINKINK_MONO);
    if (!wokeFromDeepSleep) {
        showStartupImage();
        Serial.println("Display: Startup image");
    }

#if CONFIG_TINYUSB_ENABLED
    USB.begin();
#endif
    usbPowerPresent = detectUsbPowerPresent();
    lastSampleMs = millis();
    lastDisplayMs = millis();
    Serial.printf("Power mode: %s\n", usbPowerPresent ? "USB" : "Battery");

    // ── D15 long-press listener task (runs continuously) ──
    xTaskCreate(invertButtonTask, "invert_btn", 4096, nullptr, 1, nullptr);
    // ── D12 mode-cycle button task ──
    xTaskCreate(modeButtonTask, "mode_btn", 4096, nullptr, 1, nullptr);
    // ── D14 carousel toggle button task ──
    xTaskCreate(carouselButtonTask, "carousel_btn", 4096, nullptr, 1, nullptr);

    if (!wokeFromDeepSleep) {
        delay(5000);
    }

    // ── I2C + SCD-30 sensor ──
    Wire.begin(SDA, SCL);

    if (!scd30.begin()) {
        Serial.println("ERROR: SCD-30 not found – check wiring");
        showStatus("Sensor not found!");
        while (true) { delay(1000); }
    }

    Serial.println("SCD-30 initialised");
}

// ---------------------------------------------------------------------------
// Main loop – read → flash → display → sleep, every 20 s.
//   D15 invert is handled by a dedicated FreeRTOS task.
// ---------------------------------------------------------------------------
void loop() {
    unsigned long now = millis();

    bool usbNow = detectUsbPowerPresent();
    if (usbNow != usbPowerPresent) {
        usbPowerPresent = usbNow;
        Serial.printf("Power mode changed: %s\n", usbPowerPresent ? "USB" : "Battery");
        if (usbPowerPresent) {
            flashNeopixelsColor(0, 0, 200, 180);     // USB connected: blue
            playUsbConnectedJingle();
        } else {
            flashNeopixelsColor(220, 90, 0, 180);    // USB disconnected: orange
            playUsbDisconnectedJingle();
        }
    }

    if (!usbPowerPresent) {
        const uint32_t sleepPollMs = BATTERY_USB_POLL_INTERVAL_MS;

        float battV = readBatteryVoltage();
        uint8_t battPct = batteryPercentFromVoltage(battV);

        if (battPct < LOW_BATTERY_PERCENT) {
            lowBatteryElapsedMs += sleepPollMs;
            if (lowBatteryElapsedMs >= LOW_BATTERY_FLASH_INTERVAL_MS) {
                Serial.printf("Low battery warning: %.2fV (%u%%)\n", battV, battPct);
                flashNeopixelsRed();
                lowBatteryElapsedMs = 0;
            }
        } else {
            lowBatteryElapsedMs = 0;
        }

        if (batterySampleElapsedMs < BATTERY_SAMPLE_INTERVAL_MS) {
            batterySampleElapsedMs += sleepPollMs;
            if (batterySampleElapsedMs > BATTERY_SAMPLE_INTERVAL_MS) {
                batterySampleElapsedMs = BATTERY_SAMPLE_INTERVAL_MS;
            }

            enterDeepSleepMs(sleepPollMs);
            return;
        }

        batterySampleElapsedMs = 0;

        bool sampled = false;

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
                Serial.printf("CO2: %.0f ppm | Temp: %.1f F | RH: %.1f %%\n", co2, tempF, rh);

                portENTER_CRITICAL(&stateMux);
                displayUpdateInProgress = true;
                portEXIT_CRITICAL(&stateMux);

                pushSample(co2, tempF, rh);
                bool firstBatteryDisplay = (batterySampleCycles == 0);
                batterySampleCycles++;
                bool periodicDisplay = ((batterySampleCycles % BATTERY_DISPLAY_EVERY_N) == 0);

                lastCO2 = co2;
                lastTempF = tempF;
                lastRH = rh;
                hasReading = true;

                if (firstBatteryDisplay || periodicDisplay) {
                    updateDisplay(co2, tempF, rh);
                    lastDisplayMs = millis();
                    if (carouselModeEnabled) {
                        advanceDisplayMode();
                    }
                }

                portENTER_CRITICAL(&stateMux);
                displayUpdateInProgress = false;
                portEXIT_CRITICAL(&stateMux);

                applyPendingInvertToggleIfAny();
                applyPendingModeCycleIfAny();
                applyPendingCarouselToggleIfAny();
                sampled = true;
            }
        } else {
            Serial.println("Failed to read SCD-30");
        }

        if (!sampled) {
            applyPendingInvertToggleIfAny();
            applyPendingModeCycleIfAny();
            applyPendingCarouselToggleIfAny();
        }

        enterDeepSleepMs(sleepPollMs);
        return;
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
                Serial.printf("CO2: %.0f ppm | Temp: %.1f F | RH: %.1f %%\n",
                               co2, tempF, rh);

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

                applyPendingInvertToggleIfAny();
                applyPendingModeCycleIfAny();
                applyPendingCarouselToggleIfAny();
            }
        } else {
            Serial.println("Failed to read SCD-30");
        }
    }

    applyPendingInvertToggleIfAny();
    applyPendingModeCycleIfAny();
    applyPendingCarouselToggleIfAny();

    // Keep loop responsive for power-mode transitions and deferred toggles
    delay(100);
}
