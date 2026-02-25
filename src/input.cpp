#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"
#include "state.h"
#include "input.h"
#include "display.h"
#include "leds.h"
#include "audio.h"
#include "power.h"

const char *displayModeName(uint8_t mode) {
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

void advanceDisplayMode() {
    currentDisplayMode = (uint8_t)((currentDisplayMode + 1) % DISPLAY_MODE_COUNT);
}

void advanceDeepDisplayMode() {
    currentDeepDisplayMode = (uint8_t)((currentDeepDisplayMode + 1) % DEEP_DISPLAY_MODE_COUNT);
}

const char *displayModeNameDeep(uint8_t mode) {
    switch (mode) {
        case DEEP_DISPLAY_MODE_COMBINED: return "DEEP_COMBINED";
        case DEEP_DISPLAY_MODE_CO2_ONLY: return "DEEP_CO2";
        case DEEP_DISPLAY_MODE_TEMP_ONLY: return "DEEP_TEMP";
        case DEEP_DISPLAY_MODE_RH_ONLY: return "DEEP_RH";
        default: return "UNKNOWN";
    }
}

void applyButtonAction(uint8_t gpio) {
    if (gpio == MODE_BTN) {
        if (deepSleepEnabled && !usbPowerPresent) {
            advanceDeepDisplayMode();
            Serial.printf("Button -> deep mode: %s\n", displayModeNameDeep(currentDeepDisplayMode));
        } else {
            advanceDisplayMode();
            Serial.printf("Button -> mode: %s\n", displayModeName(currentDisplayMode));
        }
        flashNeopixelsColor(MODE_TOGGLE_FLASH_R, MODE_TOGGLE_FLASH_G, MODE_TOGGLE_FLASH_B, INVERT_FLASH_MS);
        playModeToggleTone();
    } else if (gpio == CAROUSEL_BTN) {
        carouselModeEnabled = !carouselModeEnabled;
        Serial.printf("Button -> carousel: %s\n", carouselModeEnabled ? "ON" : "OFF");
        if (carouselModeEnabled) {
            flashNeopixelsColor(CAROUSEL_ON_FLASH_R, CAROUSEL_ON_FLASH_G, CAROUSEL_ON_FLASH_B, INVERT_FLASH_MS);
        } else {
            flashNeopixelsColor(CAROUSEL_OFF_FLASH_R, CAROUSEL_OFF_FLASH_G, CAROUSEL_OFF_FLASH_B, INVERT_FLASH_MS);
        }
        playModeToggleTone();
    } else if (gpio == STATS_BTN) {
        Serial.println("Button -> stats");
        handleStatsButtonPress();
        return;  // handleStatsButtonPress already restores display
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

void delayWithButtonPolling(uint32_t waitMs) {
    const uint8_t btnPins[] = { MODE_BTN, CAROUSEL_BTN, SLEEP_TOGGLE_BTN, STATS_BTN };
    const int btnCount = sizeof(btnPins) / sizeof(btnPins[0]);
    bool prevState[4] = { false, false, false, false };
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

void handleModeCycleRequest() {
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

void applyPendingModeCycleIfAny() {
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

void handleCarouselToggleRequest() {
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

void applyPendingCarouselToggleIfAny() {
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

// ---------------------------------------------------------------------------
// Toggle deep/light sleep mode: flash LEDs, beep, show status, redraw.
// ---------------------------------------------------------------------------
void handleSleepToggle() {
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
// Handle a D15 stats button press: flash LED, show stats, wait, then
// restore the previous display content.
// ---------------------------------------------------------------------------
void handleStatsButtonPress() {
    flashNeopixelsColor(STATS_FLASH_R, STATS_FLASH_G, STATS_FLASH_B, INVERT_FLASH_MS);
    playModeToggleTone();

    showStatsScreen();
    delay(STATS_DISPLAY_MS);

    // Restore previous display
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
// FreeRTOS button tasks (USB mode only)
// ---------------------------------------------------------------------------

void modeButtonTask(void *param) {
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

void carouselButtonTask(void *param) {
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
void sleepToggleButtonTask(void *param) {
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

// ---------------------------------------------------------------------------
// D15 stats button task: short press shows battery/config stats overlay.
// Runs only in USB mode as a FreeRTOS task.
// ---------------------------------------------------------------------------
void statsButtonTask(void *param) {
    (void)param;
    bool pressedPrev = false;
    unsigned long downAt = 0;

    while (true) {
        bool pressed = (digitalRead(STATS_BTN) == LOW);

        if (pressed && !pressedPrev) {
            downAt = millis();
        }

        if (!pressed && pressedPrev) {
            if (millis() - downAt >= 40) {
                handleStatsButtonPress();
            }
        }

        pressedPrev = pressed;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
