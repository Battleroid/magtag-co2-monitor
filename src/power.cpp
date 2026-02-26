#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <WiFi.h>
#ifdef CONFIG_BT_ENABLED
#include <esp_bt.h>
#endif

#include "config.h"
#include "state.h"
#include "power.h"
#include "sensor.h"
#include "leds.h"

float readBatteryVoltage() {
    // Use factory-calibrated millivolt reading instead of raw ADC conversion.
    // The MagTag has a 2:1 resistor divider on the battery pin, so multiply
    // the measured voltage by 2.  analogReadMilliVolts() accounts for the
    // actual ADC reference and per-chip eFuse calibration, giving ~50 mV
    // better accuracy than the naive (raw / 4095) * 3.3 formula.
    return analogReadMilliVolts(BATT_MONITOR) * 2.0f / 1000.0f;
}

uint8_t batteryPercentFromVoltage(float voltage) {
    const float fullV = 4.20f;
    const float emptyV = 3.30f;
    float pct = (voltage - emptyV) * 100.0f / (fullV - emptyV);
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return (uint8_t)pct;
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

bool detectUsbPowerPresent() {
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

void loadVisualPreferences() {
    graphHeavyLayout = true;
    Serial.println("Loaded prefs: layout=GRAPH_HEAVY");
}

void disableRadios() {
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
#ifdef CONFIG_BT_ENABLED
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        esp_bt_controller_disable();
    }
#endif
}

uint32_t currentSampleIntervalMs() {
    return usbPowerPresent ? USB_SAMPLE_INTERVAL_MS : BATTERY_SAMPLE_INTERVAL_MS;
}

uint32_t currentDisplayIntervalMs() {
    return usbPowerPresent ? USB_DISPLAY_INTERVAL_MS : BATTERY_DISPLAY_INTERVAL_MS;
}

void enterDeepSleepMs(uint32_t sleepMs) {
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

    // Stop SCD30 — but leave it running if it's in the pre-sample
    // warmup phase (rtcScd30WarmingUp).  The sensor continues on its
    // own clock during deep sleep and will be ready at sample time.
    if (scd30Ready && !rtcScd30WarmingUp) {
        bool stopOk = sensorStop();
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

    // Button wake: D14 (carousel), D12 (mode), D11 (sleep toggle), D15 (stats) — active-low
    uint64_t buttonMask = (1ULL << CAROUSEL_BTN) | (1ULL << MODE_BTN) | (1ULL << SLEEP_TOGGLE_BTN) | (1ULL << STATS_BTN);
    esp_sleep_enable_ext1_wakeup(buttonMask, ESP_EXT1_WAKEUP_ANY_LOW);

    // Keep RTC peripherals powered so internal pull-ups stay active
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    // Explicitly enable RTC-domain pull-ups on button GPIOs to prevent
    // floating inputs from triggering spurious ext1 wakes during sleep.
    const gpio_num_t btnPins[] = { (gpio_num_t)CAROUSEL_BTN, (gpio_num_t)MODE_BTN, (gpio_num_t)SLEEP_TOGGLE_BTN, (gpio_num_t)STATS_BTN };
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

void enterLightSleepMs(uint32_t sleepMs) {
    // ── Power down peripherals ──
    // NOTE: The SCD30 is managed by the caller.  During the warmup
    // phase (started SCD30_WARMUP_MS before sample time) it remains
    // powered and continues continuous measurement on its own clock
    // while the ESP32 sleeps.  Between samples the sensor is stopped
    // to save ~19 mA.

    // E-ink display: software power-down + hold RESET low
    display.powerDown();
    digitalWrite(EPD_RESET, LOW);

    // NeoPixels and speaker amp off
    digitalWrite(NEOPIXEL_POWER, HIGH);
    digitalWrite(SPEAKER_SHUTDOWN, LOW);

    // --- Wake sources ---
    esp_sleep_enable_timer_wakeup((uint64_t)sleepMs * 1000ULL);

    // Button wake: D14 (carousel), D12 (mode), D11 (sleep toggle), D15 (stats) — active-low
    uint64_t buttonMask = (1ULL << CAROUSEL_BTN) | (1ULL << MODE_BTN) | (1ULL << SLEEP_TOGGLE_BTN) | (1ULL << STATS_BTN);
    esp_sleep_enable_ext1_wakeup(buttonMask, ESP_EXT1_WAKEUP_ANY_LOW);

    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    const gpio_num_t btnPins[] = { (gpio_num_t)CAROUSEL_BTN, (gpio_num_t)MODE_BTN, (gpio_num_t)SLEEP_TOGGLE_BTN, (gpio_num_t)STATS_BTN };
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

    // Release display from hardware reset so the EPD library can talk to
    // it on the next updateDisplay() call (powerUp() handles the rest).
    digitalWrite(EPD_RESET, HIGH);

    // SCD30 is managed by the caller — it may or may not be running
    // depending on the warmup schedule.  I2C bus is restored above.

    lightSleepWakePending = true;
    Serial.printf("Woke from light sleep, cause: %d\n", (int)esp_sleep_get_wakeup_cause());
}
