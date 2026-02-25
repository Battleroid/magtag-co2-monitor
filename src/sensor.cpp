#include <Wire.h>
#include "config.h"
#include "state.h"
#include "sensor.h"

bool sensorInit() {
    return scd30.begin();
}

bool sensorReady() {
    return scd30.dataReady();
}

bool sensorRead(float *co2, float *tempF, float *rh) {
    if (!scd30.read()) return false;
    *co2   = scd30.CO2;
    float tempC = scd30.temperature;
    *tempF = tempC * 9.0f / 5.0f + 32.0f;
    *rh    = scd30.relative_humidity;
    return true;
}

bool sensorStop() {
    Wire.beginTransmission(SCD30_I2C_ADDR);
    Wire.write((uint8_t)((SCD30_CMD_STOP_MEASUREMENTS >> 8) & 0xFF));
    Wire.write((uint8_t)(SCD30_CMD_STOP_MEASUREMENTS & 0xFF));
    uint8_t err = Wire.endTransmission(true);
    delay(4);
    return (err == 0);
}

void sensorWaitForData(uint32_t timeoutMs) {
    unsigned long waitStart = millis();
    while (!scd30.dataReady()) {
        if (millis() - waitStart > timeoutMs) {
            Serial.println("Timeout waiting for SCD-30 data");
            break;
        }
        delay(50);
    }
}
