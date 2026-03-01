#pragma once

#include <Arduino.h>

bool sensorInit();
bool sensorReady();
bool sensorRead(float *co2, float *tempF, float *rh);
bool sensorStop();
void sensorWaitForData(uint32_t timeoutMs);
