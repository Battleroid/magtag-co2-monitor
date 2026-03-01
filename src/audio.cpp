#include <Arduino.h>
#include "config.h"
#include "audio.h"

void playToneOnce(uint16_t freq, uint16_t durationMs, uint8_t volumePercent) {
    if (volumePercent > 100) volumePercent = 100;

    digitalWrite(SPEAKER_SHUTDOWN, HIGH);
    delay(8);

    if (volumePercent >= 100) {
        tone(A0, freq);
        delay(durationMs);
    } else {
        const uint8_t gatePeriodMs = 2;
        uint8_t onMs = (gatePeriodMs * volumePercent) / 100;
        if (onMs < 1 && volumePercent > 0) onMs = 1;
        uint8_t offMs = gatePeriodMs - onMs;

        unsigned long tStart = millis();
        while (millis() - tStart < durationMs) {
            if (onMs > 0) {
                tone(A0, freq);
                delay(onMs);
            }
            if (offMs > 0) {
                noTone(A0);
                delay(offMs);
            }
        }
    }

    noTone(A0);
    digitalWrite(SPEAKER_SHUTDOWN, LOW);
    delay(10);
}

void playStartupJingle() {
    playToneOnce(880, 110);
    delay(20);
    playToneOnce(1175, 110);
    delay(20);
    playToneOnce(1568, 170);
}

void playLayoutToggleTone() {
    playToneOnce(988, 90);
}

void playModeToggleTone() {
    playToneOnce(1318, 80);
}

void playUsbConnectedJingle() {
    playToneOnce(1047, 70);
    playToneOnce(1318, 80);
    playToneOnce(1568, 100);
}

void playUsbDisconnectedJingle() {
    playToneOnce(1568, 70);
    playToneOnce(1318, 80);
    playToneOnce(1047, 110);
}
