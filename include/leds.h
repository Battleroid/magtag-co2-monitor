#pragma once

#include <Arduino.h>

void flashNeopixelsColor(uint8_t red, uint8_t green, uint8_t blue, uint16_t ms);
void flashNeopixelsRed();
void flashCenterNeopixels(uint8_t r, uint8_t g, uint8_t b, uint16_t ms);
void flashSampleIndicator();

uint8_t batteryLedTierFromPercent(uint8_t batteryPercent);
void batteryLedTierColor(uint8_t tier, uint8_t *red, uint8_t *green, uint8_t *blue);
void showBatteryLedFrame(uint8_t litCount, uint8_t red, uint8_t green, uint8_t blue);
void showBatteryStatusColorFrame(uint8_t litCount);
void flashBatteryLedTier(uint8_t tier, uint8_t flashes, uint16_t onMs, uint16_t offMs);
void handleBatteryTierTransition(uint8_t batteryPercent);
void resetBatteryTierNotificationsForUsb();
void flashPendingBatteryTierIfAny();
void playBatteryStartupAnimation(uint8_t batteryPercent);
