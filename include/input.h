#pragma once

#include <Arduino.h>

void applyButtonAction(uint8_t gpio);
void delayWithButtonPolling(uint32_t waitMs);
void handleModeCycleRequest();
void applyPendingModeCycleIfAny();
void handleCarouselToggleRequest();
void applyPendingCarouselToggleIfAny();
void handleSleepToggle();
void handleStatsButtonPress();
void advanceDisplayMode();
void advanceDeepDisplayMode();
const char *displayModeName(uint8_t mode);
const char *displayModeNameDeep(uint8_t mode);

// FreeRTOS button tasks (USB mode)
void modeButtonTask(void *param);
void carouselButtonTask(void *param);
void sleepToggleButtonTask(void *param);
void statsButtonTask(void *param);
