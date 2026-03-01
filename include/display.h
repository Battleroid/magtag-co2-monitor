#pragma once

#include <Arduino.h>

void updateDisplay(float co2, float tempF, float rh);
void updateDisplayDeepSleep(float co2, float tempF, float rh);
void showStatus(const char *msg);
void showBatteryWarningMessage(const char *line1, const char *line2, uint32_t durationMs, bool blinkDimRed);
void showStatsScreen();
void showSleepModeStatus();
void showStartupImage();
void showBuildInfoScreen();
