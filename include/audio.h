#pragma once

#include <Arduino.h>

void playToneOnce(uint16_t freq, uint16_t durationMs, uint8_t volumePercent = 100);
void playStartupJingle();
void playLayoutToggleTone();
void playModeToggleTone();
void playUsbConnectedJingle();
void playUsbDisconnectedJingle();
