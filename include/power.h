#pragma once

#include <Arduino.h>

float readBatteryVoltage();
uint8_t batteryPercentFromVoltage(float voltage);
bool detectUsbPowerPresent();
void enterDeepSleepMs(uint32_t sleepMs);
void enterLightSleepMs(uint32_t sleepMs);
void disableRadios();
void loadVisualPreferences();
uint32_t currentSampleIntervalMs();
uint32_t currentDisplayIntervalMs();
