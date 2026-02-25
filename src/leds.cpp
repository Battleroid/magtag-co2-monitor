#include <Arduino.h>
#include "config.h"
#include "state.h"
#include "leds.h"

void flashNeopixelsColor(uint8_t red, uint8_t green, uint8_t blue, uint16_t ms) {
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

void flashNeopixelsRed() {
    flashNeopixelsColor(GENERIC_RED_FLASH_R, GENERIC_RED_FLASH_G, GENERIC_RED_FLASH_B, FLASH_DURATION_MS);
}

void flashCenterNeopixels(uint8_t r, uint8_t g, uint8_t b, uint16_t ms) {
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

void flashSampleIndicator() {
    if (usbPowerPresent) {
        flashNeopixelsColor(USB_SAMPLE_FLASH_R, USB_SAMPLE_FLASH_G, USB_SAMPLE_FLASH_B, FLASH_DURATION_MS);
    }
}

uint8_t batteryLedTierFromPercent(uint8_t batteryPercent) {
    if (batteryPercent >= BATTERY_LEVEL_4_MIN_PERCENT) return 4;
    if (batteryPercent >= BATTERY_LEVEL_3_MIN_PERCENT) return 3;
    if (batteryPercent >= BATTERY_LEVEL_2_MIN_PERCENT) return 2;
    return 1;
}

void batteryLedTierColor(uint8_t tier, uint8_t *red, uint8_t *green, uint8_t *blue) {
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

void showBatteryLedFrame(uint8_t litCount, uint8_t red, uint8_t green, uint8_t blue) {
    const uint8_t batteryPixelOrder[4] = {3, 2, 1, 0};
    if (litCount > 4) litCount = 4;

    pixels.clear();
    for (uint8_t i = 0; i < litCount; i++) {
        pixels.setPixelColor(batteryPixelOrder[i], pixels.Color(red, green, blue));
    }
    pixels.show();
}

void showBatteryStatusColorFrame(uint8_t litCount) {
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

void flashBatteryLedTier(uint8_t tier, uint8_t flashes, uint16_t onMs, uint16_t offMs) {
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

void handleBatteryTierTransition(uint8_t batteryPercent) {
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

void resetBatteryTierNotificationsForUsb() {
    rtcLastBatteryLedTier = 255;
    rtcBatteryTierFlashPending = false;
    rtcPendingBatteryLedTier = 0;
    rtcNotified4to3 = false;
    rtcNotified2to1 = false;
}

void flashPendingBatteryTierIfAny() {
    if (!usbPowerPresent && rtcBatteryTierFlashPending) {
        flashBatteryLedTier(rtcPendingBatteryLedTier, 1, BATTERY_LEVEL_FLASH_ON_MS, BATTERY_LEVEL_FLASH_OFF_MS);
        rtcBatteryTierFlashPending = false;
    }
}

void playBatteryStartupAnimation(uint8_t batteryPercent) {
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
