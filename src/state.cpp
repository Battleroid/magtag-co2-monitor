#include "state.h"

// ---------------------------------------------------------------------------
// History ring buffer
// ---------------------------------------------------------------------------
float histCO2[HISTORY_LEN];
float histTempF[HISTORY_LEN];
float histRH[HISTORY_LEN];
unsigned long histTs[HISTORY_LEN];
int   histCount = 0;
int   histHead  = 0;

void pushSample(float co2, float tempF, float rh) {
    histCO2[histHead]   = co2;
    histTempF[histHead] = tempF;
    histRH[histHead]    = rh;
    histTs[histHead]    = millis();
    histHead = (histHead + 1) % HISTORY_LEN;
    histCount++;
}

int histSamples() { return (histCount < HISTORY_LEN) ? histCount : HISTORY_LEN; }

float histGet(const float *buf, int i) {
    int n = histSamples();
    int idx = (histHead - n + i + HISTORY_LEN) % HISTORY_LEN;
    return buf[idx];
}

unsigned long histGetTs(int i) {
    int n = histSamples();
    int idx = (histHead - n + i + HISTORY_LEN) % HISTORY_LEN;
    return histTs[idx];
}

// ---------------------------------------------------------------------------
// Hardware objects  (board-defined pins from MagTag variant header)
// ---------------------------------------------------------------------------
Adafruit_SCD30  scd30;
Adafruit_NeoPixel pixels(4, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
ThinkInk_290_Grayscale4_EAAMFGN display(EPD_DC, EPD_RESET, EPD_CS, -1, -1);
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

// ---------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------
bool     graphHeavyLayout = true;
uint8_t  currentDisplayMode = DISPLAY_MODE_COMBINED;
bool     carouselModeEnabled = false;
uint8_t  currentDeepDisplayMode = DEEP_DISPLAY_MODE_COMBINED;
bool     deepSleepEnabled = false;
bool     lightSleepWakePending = false;

float    lastCO2   = 0;
float    lastTempF = 0;
float    lastRH    = 0;
bool     hasReading = false;
bool     displayUpdateInProgress = false;
bool     modeCyclePending = false;
bool     carouselTogglePending = false;
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

bool     usbPowerPresent = false;
unsigned long lastSampleMs = 0;
unsigned long lastDisplayMs = 0;
bool     batteryFilterInit = false;
float    batteryVoltageFiltered = 0.0f;
unsigned long lastPowerCheckMs = 0;
int      usbHeuristicScore = 0;
bool     usbByHeuristic = false;
volatile bool usbMounted = false;

TaskHandle_t modeButtonTaskHandle     = nullptr;
TaskHandle_t carouselButtonTaskHandle  = nullptr;
TaskHandle_t sleepToggleTaskHandle     = nullptr;
TaskHandle_t statsButtonTaskHandle     = nullptr;

// ---------------------------------------------------------------------------
// RTC-persisted state (survives deep sleep)
// ---------------------------------------------------------------------------
RTC_DATA_ATTR uint32_t batterySampleCycles = 0;
RTC_DATA_ATTR uint32_t batterySampleElapsedMs = BATTERY_SAMPLE_INTERVAL_MS;
RTC_DATA_ATTR bool     batteryWarn50Shown = false;
RTC_DATA_ATTR uint32_t batteryCriticalElapsedMs = 0;
RTC_DATA_ATTR bool     rtcHasUsbPowerState = false;
RTC_DATA_ATTR bool     rtcLastUsbPowerPresent = false;
RTC_DATA_ATTR uint8_t  rtcLastBatteryLedTier = 255;
RTC_DATA_ATTR bool     rtcBatteryTierFlashPending = false;
RTC_DATA_ATTR uint8_t  rtcPendingBatteryLedTier = 0;
RTC_DATA_ATTR bool     rtcNotified4to3 = false;
RTC_DATA_ATTR bool     rtcNotified2to1 = false;

RTC_DATA_ATTR uint8_t  rtcDisplayMode       = DISPLAY_MODE_COMBINED;
RTC_DATA_ATTR bool     rtcCarouselEnabled    = false;
RTC_DATA_ATTR float    rtcLastCO2            = 0;
RTC_DATA_ATTR float    rtcLastTempF          = 0;
RTC_DATA_ATTR float    rtcLastRH             = 0;
RTC_DATA_ATTR bool     rtcHasReading         = false;
RTC_DATA_ATTR bool     rtcDeepSleepEnabled   = false;
RTC_DATA_ATTR uint8_t  rtcDeepDisplayMode    = DEEP_DISPLAY_MODE_COMBINED;
RTC_DATA_ATTR bool     rtcScd30WarmingUp      = false;

bool     wokeFromDeepSleep = false;
uint8_t  buttonWakeGPIO = 0;
bool     scd30Ready = false;
