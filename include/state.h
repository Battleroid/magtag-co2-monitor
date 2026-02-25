#pragma once

#include <Arduino.h>
#include <Adafruit_SCD30.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_ThinkInk.h>
#include <Adafruit_LIS3DH.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"
#include "types.h"

// ---------------------------------------------------------------------------
// History ring buffer
//   Size is derived from graph window and fastest configured sample interval,
//   so changing sample cadence (USB/battery) or window duration stays in sync.
// ---------------------------------------------------------------------------
#define FASTEST_SAMPLE_INTERVAL_MS ((USB_SAMPLE_INTERVAL_MS < BATTERY_SAMPLE_INTERVAL_MS) ? USB_SAMPLE_INTERVAL_MS : BATTERY_SAMPLE_INTERVAL_MS)
#define HISTORY_LEN ((((GRAPH_WINDOW_MINUTES * 60UL * 1000UL) + FASTEST_SAMPLE_INTERVAL_MS - 1) / FASTEST_SAMPLE_INTERVAL_MS) + 1)

extern float histCO2[HISTORY_LEN];
extern float histTempF[HISTORY_LEN];
extern float histRH[HISTORY_LEN];
extern unsigned long histTs[HISTORY_LEN];
extern int   histCount;
extern int   histHead;

void pushSample(float co2, float tempF, float rh);
int  histSamples();
float histGet(const float *buf, int i);
unsigned long histGetTs(int i);

// ---------------------------------------------------------------------------
// Hardware objects
// ---------------------------------------------------------------------------
extern Adafruit_SCD30  scd30;
extern Adafruit_NeoPixel pixels;
extern ThinkInk_290_Grayscale4_EAAMFGN display;
extern Adafruit_LIS3DH lis;

// ---------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------
extern bool     graphHeavyLayout;
extern uint8_t  currentDisplayMode;
extern bool     carouselModeEnabled;
extern uint8_t  currentDeepDisplayMode;
extern bool     deepSleepEnabled;
extern bool     lightSleepWakePending;

extern float    lastCO2;
extern float    lastTempF;
extern float    lastRH;
extern bool     hasReading;
extern bool     displayUpdateInProgress;
extern bool     modeCyclePending;
extern bool     carouselTogglePending;
extern portMUX_TYPE stateMux;

extern bool     usbPowerPresent;
extern unsigned long lastSampleMs;
extern unsigned long lastDisplayMs;
extern bool     batteryFilterInit;
extern float    batteryVoltageFiltered;
extern unsigned long lastPowerCheckMs;
extern int      usbHeuristicScore;
extern bool     usbByHeuristic;
extern volatile bool usbMounted;

extern TaskHandle_t modeButtonTaskHandle;
extern TaskHandle_t carouselButtonTaskHandle;
extern TaskHandle_t sleepToggleTaskHandle;
extern TaskHandle_t statsButtonTaskHandle;

// ---------------------------------------------------------------------------
// RTC-persisted state (survives deep sleep)
// ---------------------------------------------------------------------------
extern RTC_DATA_ATTR uint32_t batterySampleCycles;
extern RTC_DATA_ATTR uint32_t batterySampleElapsedMs;
extern RTC_DATA_ATTR bool     batteryWarn50Shown;
extern RTC_DATA_ATTR uint32_t batteryCriticalElapsedMs;
extern RTC_DATA_ATTR bool     rtcHasUsbPowerState;
extern RTC_DATA_ATTR bool     rtcLastUsbPowerPresent;
extern RTC_DATA_ATTR uint8_t  rtcLastBatteryLedTier;
extern RTC_DATA_ATTR bool     rtcBatteryTierFlashPending;
extern RTC_DATA_ATTR uint8_t  rtcPendingBatteryLedTier;
extern RTC_DATA_ATTR bool     rtcNotified4to3;
extern RTC_DATA_ATTR bool     rtcNotified2to1;

extern RTC_DATA_ATTR uint8_t  rtcDisplayMode;
extern RTC_DATA_ATTR bool     rtcCarouselEnabled;
extern RTC_DATA_ATTR float    rtcLastCO2;
extern RTC_DATA_ATTR float    rtcLastTempF;
extern RTC_DATA_ATTR float    rtcLastRH;
extern RTC_DATA_ATTR bool     rtcHasReading;
extern RTC_DATA_ATTR bool     rtcDeepSleepEnabled;
extern RTC_DATA_ATTR uint8_t  rtcDeepDisplayMode;

extern bool     wokeFromDeepSleep;
extern uint8_t  buttonWakeGPIO;
extern bool     scd30Ready;

// ---------------------------------------------------------------------------
// Inline helpers
// ---------------------------------------------------------------------------
inline uint16_t fgColor() { return EPD_BLACK; }
inline uint16_t bgColor() { return EPD_WHITE; }
