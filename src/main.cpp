#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <USB.h>
#include <esp_sleep.h>

#include "config.h"
#include "state.h"
#include "sensor.h"
#include "display.h"
#include "power.h"
#include "leds.h"
#include "audio.h"
#include "input.h"

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
    pinMode(STATS_BTN, INPUT_PULLUP);

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
        else if (wakeStatus & (1ULL << STATS_BTN))     buttonWakeGPIO = STATS_BTN;

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
        xTaskCreate(statsButtonTask, "stats_btn", 8192, nullptr, 1, &statsButtonTaskHandle);
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
        if (!sensorInit()) {
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
// Main loop – read -> flash -> display -> sleep, every 20 s.
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
            if (statsButtonTaskHandle == nullptr)
                xTaskCreate(statsButtonTask, "stats_btn", 8192, nullptr, 1, &statsButtonTaskHandle);
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
            if (statsButtonTaskHandle != nullptr) {
                vTaskDelete(statsButtonTaskHandle);
                statsButtonTaskHandle = nullptr;
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
            while (!sensorReady()) {
                if (millis() - waitStart > 5000) {
                    Serial.println("Timeout waiting for SCD-30 data");
                    break;
                }
                for (uint8_t pin : {MODE_BTN, CAROUSEL_BTN, STATS_BTN}) {
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

            float co2, tempF, rh;
            if (sensorRead(&co2, &tempF, &rh)) {
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
                } else if (digitalRead(STATS_BTN) == LOW) {
                    while (digitalRead(STATS_BTN) == LOW) delay(20);
                    delay(50);
                    applyButtonAction(STATS_BTN);
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
                        } else if (ws & (1ULL << STATS_BTN)) {
                            delay(50);
                            applyButtonAction(STATS_BTN);
                        }
                    }
                }
            }

            // Also handle residual button wake from deep->light transition
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
                    sensorWaitForData(5000);

                    float co2, tempF, rh;
                    if (sensorRead(&co2, &tempF, &rh)) {
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

    // =====================================================================
    // USB ALWAYS-ON PATH — full display with graphs, FreeRTOS button tasks
    // =====================================================================
    uint32_t sampleIntervalMs = currentSampleIntervalMs();
    uint32_t displayIntervalMs = currentDisplayIntervalMs();
    bool shouldSample = (now - lastSampleMs >= sampleIntervalMs);

    if (shouldSample) {
        lastSampleMs = now;

        // Wait for sensor data (up to 5 s)
        sensorWaitForData(5000);

        float co2, tempF, rh;
        if (sensorRead(&co2, &tempF, &rh)) {
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
