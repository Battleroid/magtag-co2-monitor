#include <Arduino.h>
#include "config.h"
#include "state.h"
#include "display.h"
#include "leds.h"
#include "power.h"

#if __has_include("startup_image.h")
#include "startup_image.h"
#define HAS_STARTUP_IMAGE 1
#else
#define HAS_STARTUP_IMAGE 0
#endif

// ---------------------------------------------------------------------------
// Helper: right-align text at a given x-right edge, y position.
//   Measures the string width with the current text size, then positions
//   the cursor so the text ends at xRight.
// ---------------------------------------------------------------------------
static void printRightAligned(int16_t xRight, int16_t y, const char *str) {
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(str, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(xRight - (int16_t)w, y);
    display.print(str);
}

// ---------------------------------------------------------------------------
// Draw a tiny line graph.
//   buf/getter: history data, n: number of samples,
//   gx,gy,gw,gh: graph bounding box.
// ---------------------------------------------------------------------------
static void drawGraph(const float *buf, int n,
                      int16_t gx, int16_t gy, int16_t gw, int16_t gh) {
    if (n < 2) return;

    const unsigned long windowMs = GRAPH_WINDOW_MINUTES * 60UL * 1000UL;
    unsigned long nowMs = millis();

    // Determine oldest sample still within the time window.
    int start = n - 1;
    for (int i = n - 1; i >= 0; --i) {
        if (nowMs - histGetTs(i) <= windowMs) {
            start = i;
        } else {
            break;
        }
    }

    int win = n - start;
    if (win < 2) return;

    // Find min/max for auto-scaling
    float lo = histGet(buf, start), hi = lo;
    for (int i = start + 1; i < n; i++) {
        float v = histGet(buf, i);
        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }
    // Ensure there's at least a small range so flat lines centre
    if (hi - lo < 0.1f) { lo -= 0.5f; hi += 0.5f; }

    uint16_t fg = fgColor();
    const int16_t graphBottom = gy + gh - 1;

    // Plot line segments, newest sample pinned to the right edge
    // and the graph shifts left as new samples arrive.
    auto yForVal = [&](float v) -> int16_t {
        float frac = (v - lo) / (hi - lo);               // 0..1
        return gy + gh - 1 - (int16_t)(frac * (gh - 1)); // top = high
    };

#if GRAPH_FILL_MODE != 0
    auto fillUnderSegment = [&](int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
        const uint16_t shade = GRAPH_FILL_SHADE;
        const int16_t graphRight = gx + gw - 1;

        if (x0 == x1) {
            if (x0 < gx || x0 > graphRight) return;
            if (y0 < gy) y0 = gy;
            if (y0 > graphBottom) y0 = graphBottom;
#if GRAPH_FILL_MODE == 1
            display.drawLine(x0, y0, x0, graphBottom, shade);
#elif GRAPH_FILL_MODE == 2
            for (int16_t py = graphBottom - ((graphBottom - y0) & 1 ? 0 : 1); py >= y0; py -= 2) {
                if ((x0 + (graphBottom - py)) & 1) display.drawPixel(x0, py, shade);
            }
#elif GRAPH_FILL_MODE == 3
            if (((x0 - gx) % GRAPH_FILL_VLINE_SPACING) == 0)
                display.drawLine(x0, y0, x0, graphBottom, shade);
#elif GRAPH_FILL_MODE == 4
            if (x0 & 1) {
                for (int16_t py = graphBottom; py >= y0; py -= 2) {
                    display.drawPixel(x0, py, shade);
                }
            }
#endif
            return;
        }

        int16_t xStart = (x0 < x1) ? x0 : x1;
        int16_t xEnd = (x0 < x1) ? x1 : x0;
        if (xEnd < gx || xStart > graphRight) return;
        if (xStart < gx) xStart = gx;
        if (xEnd > graphRight) xEnd = graphRight;

        float dx = (float)(x1 - x0);
        for (int16_t x = xStart; x <= xEnd; ++x) {
            float t = (dx == 0.0f) ? 0.0f : ((float)(x - x0) / dx);
            int16_t y = (int16_t)(y0 + (y1 - y0) * t);
            if (y < gy) y = gy;
            if (y > graphBottom) y = graphBottom;
#if GRAPH_FILL_MODE == 1
            display.drawLine(x, y, x, graphBottom, shade);
#elif GRAPH_FILL_MODE == 2
            for (int16_t py = graphBottom; py >= y; --py) {
                if ((x + (graphBottom - py)) & 1) display.drawPixel(x, py, shade);
            }
#elif GRAPH_FILL_MODE == 3
            if (((x - gx) % GRAPH_FILL_VLINE_SPACING) == 0)
                display.drawLine(x, y, x, graphBottom, shade);
#elif GRAPH_FILL_MODE == 4
            if (x & 1) {
                for (int16_t py = graphBottom; py >= y; py -= 2) {
                    display.drawPixel(x, py, shade);
                }
            }
#endif
        }
    };
#endif

    // x positions are based on sample age over GRAPH_WINDOW_MINUTES,
    // so the graph remains truly time-based across variable sample intervals.
    for (int i = 1; i < win; i++) {
        unsigned long age0 = nowMs - histGetTs(start + i - 1);
        unsigned long age1 = nowMs - histGetTs(start + i);
        int16_t x0 = gx + gw - 1 - (int32_t)age0 * (gw - 1) / (int32_t)windowMs;
        int16_t x1 = gx + gw - 1 - (int32_t)age1 * (gw - 1) / (int32_t)windowMs;
        float v0 = histGet(buf, start + i - 1);
        float v1 = histGet(buf, start + i);
        int16_t y0 = yForVal(v0);
        int16_t y1 = yForVal(v1);
#if GRAPH_FILL_MODE != 0
    #if GRAPH_FILL_RISING_ONLY
        if (v1 > v0) {
            fillUnderSegment(x0, y0, x1, y1);
        }
    #else
        fillUnderSegment(x0, y0, x1, y1);
    #endif
#endif
        if (GRAPH_LINE_THICKNESS <= 1) {
            display.drawLine(x0, y0, x1, y1, fg);
        } else {
            int16_t half = GRAPH_LINE_THICKNESS / 2;
            for (int16_t off = -half; off <= half; ++off) {
                display.drawLine(x0, y0 + off, x1, y1 + off, fg);
            }
        }
    }

    // Extend the last sample to the right edge so the graph doesn't show
    // a gap when millis() has advanced past the most recent sample
    // (e.g. after viewing the stats screen or between sleep cycles).
    int lastIdx = start + win - 1;
    unsigned long lastAge = nowMs - histGetTs(lastIdx);
    if (lastAge > 0) {
        int16_t lastX = gx + gw - 1 - (int32_t)lastAge * (gw - 1) / (int32_t)windowMs;
        int16_t rightEdge = gx + gw - 1;
        if (lastX < rightEdge - 1) {
            int16_t lastY = yForVal(histGet(buf, lastIdx));
#if GRAPH_FILL_MODE != 0
            fillUnderSegment(lastX, lastY, rightEdge, lastY);
#endif
            if (GRAPH_LINE_THICKNESS <= 1) {
                display.drawLine(lastX, lastY, rightEdge, lastY, fg);
            } else {
                int16_t half = GRAPH_LINE_THICKNESS / 2;
                for (int16_t off = -half; off <= half; ++off) {
                    display.drawLine(lastX, lastY + off, rightEdge, lastY + off, fg);
                }
            }
        }
    }
}

static bool getWindowMinMax(const float *buf, int n, float *outLo, float *outHi) {
    if (n < 1) return false;

    const unsigned long windowMs = GRAPH_WINDOW_MINUTES * 60UL * 1000UL;
    unsigned long nowMs = millis();
    int start = n - 1;
    for (int i = n - 1; i >= 0; --i) {
        if (nowMs - histGetTs(i) <= windowMs) start = i;
        else break;
    }

    float lo = histGet(buf, start);
    float hi = lo;
    for (int i = start + 1; i < n; ++i) {
        float v = histGet(buf, i);
        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }

    *outLo = lo;
    *outHi = hi;
    return true;
}

// ---------------------------------------------------------------------------
// Redraw the e-ink display.
//   Left side:  values right-aligned, vertically centred to each graph row.
//   Right side: three line graphs (CO2 / Temp / RH) over last 30 min.
// ---------------------------------------------------------------------------
void updateDisplay(float co2, float tempF, float rh) {
    uint16_t fg = fgColor();
    int16_t contentLeft = SCREEN_PAD_LEFT;
    int16_t contentTop = SCREEN_PAD_TOP;
    int16_t contentRight = display.width() - 1 - SCREEN_PAD_RIGHT;
    int16_t contentBottom = display.height() - 1 - SCREEN_PAD_BOTTOM;
    int16_t contentW = contentRight - contentLeft + 1;
    int16_t contentH = contentBottom - contentTop + 1;
    if (contentW < 32 || contentH < 32) return;

    display.clearBuffer();
    display.setTextColor(fg);

    char buf[32];
    int n = histSamples();

    if (currentDisplayMode == DISPLAY_MODE_COMBINED) {
        int16_t baseTextRight = graphHeavyLayout ? TEXT_RIGHT_GRAPH_HEAVY : TEXT_RIGHT_BALANCED;
        int16_t baseGraphX = graphHeavyLayout ? GRAPH_X_GRAPH_HEAVY : GRAPH_X_BALANCED;
        int16_t textRight = contentLeft + baseTextRight;
        int16_t graphX = contentLeft + baseGraphX;
        int16_t graphW = contentRight - graphX + 1;
        if (graphW < 24) graphW = 24;
        if (textRight >= graphX) textRight = graphX - 4;

        int16_t totalGraphH = GRAPH_H_CO2 + GRAPH_H_STD + GRAPH_H_STD + GRAPH_GAP + GRAPH_GAP;
        int16_t gyCO2  = contentTop + ((contentH > totalGraphH) ? (contentH - totalGraphH) / 2 : 0);
        int16_t gyTemp = gyCO2  + GRAPH_H_CO2 + GRAPH_GAP;
        int16_t gyRH   = gyTemp + GRAPH_H_STD + GRAPH_GAP;

        display.setTextSize(3);
        snprintf(buf, sizeof(buf), "%d", (int)co2);
        printRightAligned(textRight, gyCO2 + GRAPH_H_CO2 / 2 - 12, buf);

        display.setTextSize(2);
        snprintf(buf, sizeof(buf), "%.1f F", tempF);
        printRightAligned(textRight, gyTemp + GRAPH_H_STD / 2 - 8, buf);

        snprintf(buf, sizeof(buf), "%.1f %%", rh);
        printRightAligned(textRight, gyRH + GRAPH_H_STD / 2 - 8, buf);

        drawGraph(histCO2,   n, graphX, gyCO2,  graphW, GRAPH_H_CO2);
        drawGraph(histTempF, n, graphX, gyTemp, graphW, GRAPH_H_STD);
        drawGraph(histRH,    n, graphX, gyRH,   graphW, GRAPH_H_STD);
    } else if (currentDisplayMode == DISPLAY_MODE_CO2_ONLY ||
               currentDisplayMode == DISPLAY_MODE_TEMP_ONLY ||
               currentDisplayMode == DISPLAY_MODE_RH_ONLY) {
        const float *metricBuf = histCO2;
        float currentVal = co2;
        int decimals = 0;
        const char *suffix = "";
        if (currentDisplayMode == DISPLAY_MODE_TEMP_ONLY) {
            metricBuf = histTempF;
            currentVal = tempF;
            decimals = 1;
            suffix = "F";
        } else if (currentDisplayMode == DISPLAY_MODE_RH_ONLY) {
            metricBuf = histRH;
            currentVal = rh;
            decimals = 1;
            suffix = "%";
        }

        float lo = currentVal, hi = currentVal;
        getWindowMinMax(metricBuf, n, &lo, &hi);

        int16_t textRight = contentLeft + TEXT_RIGHT_GRAPH_HEAVY;
        int16_t graphX = contentLeft + GRAPH_X_GRAPH_HEAVY;
        int16_t graphW = contentRight - graphX + 1;
        if (graphW < 24) graphW = 24;
        if (textRight >= graphX) textRight = graphX - 4;

        int16_t totalGraphH = GRAPH_H_CO2 + GRAPH_H_STD + GRAPH_H_STD + GRAPH_GAP + GRAPH_GAP;
        int16_t gyCO2  = contentTop + ((contentH > totalGraphH) ? (contentH - totalGraphH) / 2 : 0);
        int16_t gyTemp = gyCO2 + GRAPH_H_CO2 + GRAPH_GAP;
        int16_t gyRH   = gyTemp + GRAPH_H_STD + GRAPH_GAP;

        int16_t yCurrent = gyCO2 + GRAPH_H_CO2 / 2 - 12;
        int16_t yHigh = gyTemp + GRAPH_H_STD / 2 - 8;
        int16_t yLow = gyRH + GRAPH_H_STD / 2 - 8;

        if (decimals == 0) snprintf(buf, sizeof(buf), "%d", (int)currentVal);
        else snprintf(buf, sizeof(buf), "%.1f", currentVal);
        {
            // Try size 3 for number; fall back to size 2 if it won't fit
            int16_t x1, y1;
            uint16_t numW, numH, sfxW = 0;
            int numSize = 3;
            display.setTextSize(3);
            display.getTextBounds(buf, 0, 0, &x1, &y1, &numW, &numH);
            if (suffix[0] != '\0') {
                display.setTextSize(2);
                uint16_t sfxH;
                display.getTextBounds(suffix, 0, 0, &x1, &y1, &sfxW, &sfxH);
            }
            if ((int16_t)numW + (int16_t)sfxW > textRight) {
                numSize = 2;
                display.setTextSize(2);
                display.getTextBounds(buf, 0, 0, &x1, &y1, &numW, &numH);
            }
            int16_t totalW = (int16_t)numW + (int16_t)sfxW;
            int16_t xStart = textRight - totalW;
            display.setTextSize(numSize);
            display.setCursor(xStart, yCurrent);
            display.print(buf);
            if (suffix[0] != '\0') {
                display.setTextSize(2);
                // Align baselines: size 3 = 21px, size 2 = 14px
                int16_t yOff = (numSize == 3) ? (21 - 14) : 0;
                display.setCursor(xStart + (int16_t)numW, yCurrent + yOff);
                display.print(suffix);
            }
        }

        display.setTextSize(2);
        if (decimals == 0) snprintf(buf, sizeof(buf), "%d%s", (int)hi, suffix);
        else snprintf(buf, sizeof(buf), "%.1f%s", hi, suffix);
        printRightAligned(textRight, yHigh, buf);

        if (decimals == 0) snprintf(buf, sizeof(buf), "%d%s", (int)lo, suffix);
        else snprintf(buf, sizeof(buf), "%.1f%s", lo, suffix);
        printRightAligned(textRight, yLow, buf);

        drawGraph(metricBuf, n, graphX, contentTop + 2, graphW, contentH - 4);
    } else if (currentDisplayMode == DISPLAY_MODE_TEMP_RH) {
        int16_t textRight = contentLeft + TEXT_RIGHT_GRAPH_HEAVY;
        int16_t graphX = contentLeft + GRAPH_X_GRAPH_HEAVY;
        int16_t graphW = contentRight - graphX + 1;
        if (graphW < 24) graphW = 24;
        if (textRight >= graphX) textRight = graphX - 4;
        int16_t graphGap = 8;
        int16_t halfH = (contentH - graphGap) / 2;
        int16_t upperY = contentTop;
        int16_t lowerY = upperY + halfH + graphGap;

        display.setTextSize(2);
        snprintf(buf, sizeof(buf), "%.1f F", tempF);
        printRightAligned(textRight, upperY + (halfH / 2) - 8, buf);
        snprintf(buf, sizeof(buf), "%.1f %%", rh);
        printRightAligned(textRight, lowerY + (halfH / 2) - 8, buf);

        drawGraph(histTempF, n, graphX, upperY, graphW, halfH);
        drawGraph(histRH,    n, graphX, lowerY, graphW, halfH);
    } else {
        // DISPLAY_MODE_TEXT_ONLY: CO2 left half, temp+RH right half (no graphs)
        int16_t screenW = display.width();
        int16_t screenH = display.height();
        int16_t halfW = screenW / 2;

        // Left half: CO2 + "ppm"
        display.setTextSize(3);
        snprintf(buf, sizeof(buf), "%d", (int)co2);
        int16_t bx, by; uint16_t bw, bh;
        display.getTextBounds(buf, 0, 0, &bx, &by, &bw, &bh);
        uint16_t co2NumW = bw, co2NumH = bh;

        display.setTextSize(2);
        uint16_t ppmW, ppmH;
        display.getTextBounds("ppm", 0, 0, &bx, &by, &ppmW, &ppmH);

        int16_t gap = 4;
        int16_t totalCO2H = (int16_t)co2NumH + gap + (int16_t)ppmH;
        int16_t co2StartY = (screenH - totalCO2H) / 2;

        display.setTextSize(3);
        display.setCursor((halfW - (int16_t)co2NumW) / 2, co2StartY);
        display.print(buf);

        display.setTextSize(2);
        display.setCursor((halfW - (int16_t)ppmW) / 2, co2StartY + (int16_t)co2NumH + gap);
        display.print("ppm");

        // Right half: temp + humidity
        display.setTextSize(2);
        snprintf(buf, sizeof(buf), "%.1f F", tempF);
        display.getTextBounds(buf, 0, 0, &bx, &by, &bw, &bh);
        uint16_t tempW = bw, tempH = bh;

        char rhBuf[32];
        snprintf(rhBuf, sizeof(rhBuf), "%.1f %%", rh);
        uint16_t rhW, rhH;
        display.getTextBounds(rhBuf, 0, 0, &bx, &by, &rhW, &rhH);

        uint16_t maxW = (tempW > rhW) ? tempW : rhW;
        int16_t rightGap = 8;
        int16_t totalRightH = (int16_t)tempH + rightGap + (int16_t)rhH;
        int16_t rightStartY = (screenH - totalRightH) / 2;
        int16_t rightStartX = halfW + (halfW - (int16_t)maxW) / 2;

        display.setCursor(rightStartX, rightStartY);
        display.print(buf);
        display.setCursor(rightStartX, rightStartY + (int16_t)tempH + rightGap);
        display.print(rhBuf);
    }

    display.display();
    flashPendingBatteryTierIfAny();
}

// ---------------------------------------------------------------------------
// Redraw the e-ink display in deep-sleep compact modes (no graphs).
//   Combined: CO2+ppm on left half, temp+RH on right half.
//   Single:   One metric centered with suffix below.
// ---------------------------------------------------------------------------
void updateDisplayDeepSleep(float co2, float tempF, float rh) {
    uint16_t fg = fgColor();
    display.clearBuffer();
    display.fillScreen(bgColor());
    display.setTextColor(fg);
    display.setTextWrap(false);

    char buf[32];
    int16_t x1, y1;
    uint16_t w, h;
    int16_t screenW = display.width();
    int16_t screenH = display.height();

    if (currentDeepDisplayMode == DEEP_DISPLAY_MODE_COMBINED) {
        // ── Left half: CO2 number + "ppm" suffix, centered ──
        int16_t halfW = screenW / 2;

        display.setTextSize(3);
        snprintf(buf, sizeof(buf), "%d", (int)co2);
        display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
        uint16_t co2NumW = w, co2NumH = h;

        display.setTextSize(2);
        uint16_t ppmW, ppmH;
        display.getTextBounds("ppm", 0, 0, &x1, &y1, &ppmW, &ppmH);

        int16_t gap = 4;
        int16_t totalCO2H = (int16_t)co2NumH + gap + (int16_t)ppmH;
        int16_t co2StartY = (screenH - totalCO2H) / 2;

        display.setTextSize(3);
        display.setCursor((halfW - (int16_t)co2NumW) / 2, co2StartY);
        display.print(buf);

        display.setTextSize(2);
        display.setCursor((halfW - (int16_t)ppmW) / 2, co2StartY + (int16_t)co2NumH + gap);
        display.print("ppm");

        // ── Right half: temp + humidity, left-aligned group centered ──
        display.setTextSize(2);
        snprintf(buf, sizeof(buf), "%.1f F", tempF);
        display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
        uint16_t tempW = w, tempH = h;

        char rhBuf[32];
        snprintf(rhBuf, sizeof(rhBuf), "%.1f %%", rh);
        uint16_t rhW, rhH;
        display.getTextBounds(rhBuf, 0, 0, &x1, &y1, &rhW, &rhH);

        uint16_t maxW = (tempW > rhW) ? tempW : rhW;
        int16_t rightGap = 8;
        int16_t totalRightH = (int16_t)tempH + rightGap + (int16_t)rhH;
        int16_t rightStartY = (screenH - totalRightH) / 2;
        int16_t rightStartX = halfW + (halfW - (int16_t)maxW) / 2;

        display.setCursor(rightStartX, rightStartY);
        display.print(buf);

        display.setCursor(rightStartX, rightStartY + (int16_t)tempH + rightGap);
        display.print(rhBuf);

    } else {
        // ── Single metric centered with suffix below ──
        float val;
        const char *suffix;
        int decimals;

        if (currentDeepDisplayMode == DEEP_DISPLAY_MODE_CO2_ONLY) {
            val = co2; suffix = "ppm"; decimals = 0;
        } else if (currentDeepDisplayMode == DEEP_DISPLAY_MODE_TEMP_ONLY) {
            val = tempF; suffix = "F"; decimals = 1;
        } else {
            val = rh; suffix = "%"; decimals = 1;
        }

        if (decimals == 0) snprintf(buf, sizeof(buf), "%d", (int)val);
        else                snprintf(buf, sizeof(buf), "%.1f", val);

        display.setTextSize(3);
        display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
        uint16_t numW = w, numH = h;

        display.setTextSize(2);
        uint16_t sfxW, sfxH;
        display.getTextBounds(suffix, 0, 0, &x1, &y1, &sfxW, &sfxH);

        int16_t gap = 4;
        int16_t totalH = (int16_t)numH + gap + (int16_t)sfxH;
        int16_t startY = (screenH - totalH) / 2;

        display.setTextSize(3);
        display.setCursor((screenW - (int16_t)numW) / 2, startY);
        display.print(buf);

        display.setTextSize(2);
        display.setCursor((screenW - (int16_t)sfxW) / 2, startY + (int16_t)numH + gap);
        display.print(suffix);
    }

    display.display();
    flashPendingBatteryTierIfAny();
}

// ---------------------------------------------------------------------------
// Show a single-line status message on the e-ink display.
// ---------------------------------------------------------------------------
void showStatus(const char *msg) {
    display.clearBuffer();
    display.setTextColor(fgColor());
    display.setTextSize(2);
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);
    int16_t contentW = display.width() - SCREEN_PAD_LEFT - SCREEN_PAD_RIGHT;
    int16_t contentH = display.height() - SCREEN_PAD_TOP - SCREEN_PAD_BOTTOM;
    int16_t x = SCREEN_PAD_LEFT + (contentW - (int16_t)w) / 2;
    int16_t y = SCREEN_PAD_TOP + (contentH - (int16_t)h) / 2;
    display.setCursor(x, y);
    display.print(msg);
    display.display();
}

void showBatteryWarningMessage(const char *line1, const char *line2, uint32_t durationMs, bool blinkDimRed) {
    display.clearBuffer();
    display.fillScreen(bgColor());
    display.setTextColor(fgColor());
    display.setTextWrap(false);

    int16_t x1, y1;
    uint16_t w1, h1;
    display.setTextSize(2);
    display.getTextBounds(line1, 0, 0, &x1, &y1, &w1, &h1);
    int16_t yTop = (display.height() / 2) - 20;
    int16_t line1X = (display.width() - (int16_t)w1) / 2;
    display.setCursor(line1X < 0 ? 0 : line1X, yTop);
    display.print(line1);

    if (line2 && line2[0] != '\0') {
        int16_t x2, y2;
        uint16_t w2, h2;
        display.setTextSize(2);
        display.getTextBounds(line2, 0, 0, &x2, &y2, &w2, &h2);
        int16_t line2X = (display.width() - (int16_t)w2) / 2;
        display.setCursor(line2X < 0 ? 0 : line2X, yTop + 28);
        display.print(line2);
    }

    display.display();

    unsigned long startMs = millis();
    unsigned long lastBlinkMs = startMs;
    bool ledOn = false;

    while (millis() - startMs < durationMs) {
        if (blinkDimRed && (millis() - lastBlinkMs >= BATTERY_CRITICAL_BLINK_MS)) {
            ledOn = !ledOn;
            pixels.clear();
            if (ledOn) {
                for (uint8_t i = 0; i < pixels.numPixels(); ++i) {
                    pixels.setPixelColor(i, pixels.Color(BATTERY_CRITICAL_LED_R, BATTERY_CRITICAL_LED_G, BATTERY_CRITICAL_LED_B));
                }
            }
            pixels.show();
            lastBlinkMs = millis();
        }
        delay(50);
    }

    pixels.clear();
    pixels.show();

    if (hasReading) {
        updateDisplay(lastCO2, lastTempF, lastRH);
        lastDisplayMs = millis();
    }
}

void showSleepModeStatus() {
    display.clearBuffer();
    display.fillScreen(bgColor());
    display.setTextColor(fgColor());
    display.setTextWrap(false);

    const char *line1 = "Deep Sleep";
    const char *line2 = deepSleepEnabled ? "On" : "Off";

    int16_t x1, y1;
    uint16_t w1, h1, w2, h2;

    display.setTextSize(2);
    display.getTextBounds(line1, 0, 0, &x1, &y1, &w1, &h1);
    display.getTextBounds(line2, 0, 0, &x1, &y1, &w2, &h2);

    int16_t gap = 8;
    int16_t totalH = (int16_t)h1 + gap + (int16_t)h2;
    int16_t startY = (display.height() - totalH) / 2;

    display.setCursor((display.width() - (int16_t)w1) / 2, startY);
    display.print(line1);

    display.setCursor((display.width() - (int16_t)w2) / 2, startY + (int16_t)h1 + gap);
    display.print(line2);

    display.display();
}

// ---------------------------------------------------------------------------
// Format a duration in milliseconds as a human-readable interval string.
// E.g. 15000 -> "15s", 300000 -> "5m", 3600000 -> "1h".
// ---------------------------------------------------------------------------
static void formatDuration(uint32_t ms, char *buf, size_t bufLen) {
    if (ms < 60000UL) {
        snprintf(buf, bufLen, "%lus", (unsigned long)(ms / 1000UL));
    } else if (ms < 3600000UL) {
        uint32_t m = ms / 60000UL;
        uint32_t s = (ms % 60000UL) / 1000UL;
        if (s > 0) snprintf(buf, bufLen, "%lum%lus", (unsigned long)m, (unsigned long)s);
        else       snprintf(buf, bufLen, "%lum", (unsigned long)m);
    } else {
        uint32_t h = ms / 3600000UL;
        uint32_t m = (ms % 3600000UL) / 60000UL;
        if (m > 0) snprintf(buf, bufLen, "%luh%lum", (unsigned long)h, (unsigned long)m);
        else       snprintf(buf, bufLen, "%luh", (unsigned long)h);
    }
}

// ---------------------------------------------------------------------------
// Estimate remaining battery time based on current voltage/percent and
// approximate average current draw for the active power mode.
//
// These are rough ballpark numbers for the MagTag with SCD30:
//   USB mode (always-on):    ~80 mA average (CPU + sensor + display + USB)
//   Light sleep (battery):   ~5 mA average  (mostly sleeping, brief wakes)
//   Deep sleep (battery):    ~3 mA average  (mostly off, brief sample wakes)
//
// Battery capacity configured via BATTERY_CAPACITY_MAH define.
// ---------------------------------------------------------------------------
static void estimateRemainingTime(uint8_t battPct, char *buf, size_t bufLen) {
    // Usable capacity (mAh) linearly scaled by battery percent
    const float capacityMah = (float)BATTERY_CAPACITY_MAH;
    float remainingMah = capacityMah * (battPct / 100.0f);

    float avgCurrentMa;
    if (usbPowerPresent) {
        // On USB power — effectively unlimited, don't estimate
        snprintf(buf, bufLen, "USB powered");
        return;
    } else if (deepSleepEnabled) {
        avgCurrentMa = 3.0f;
    } else {
        avgCurrentMa = 5.0f;
    }

    float hoursLeft = remainingMah / avgCurrentMa;
    uint32_t totalMinutes = (uint32_t)(hoursLeft * 60.0f);

    if (totalMinutes >= 1440) {
        uint32_t days = totalMinutes / 1440;
        uint32_t hours = (totalMinutes % 1440) / 60;
        snprintf(buf, bufLen, "~%lud %luh", (unsigned long)days, (unsigned long)hours);
    } else if (totalMinutes >= 60) {
        uint32_t hours = totalMinutes / 60;
        uint32_t mins = totalMinutes % 60;
        snprintf(buf, bufLen, "~%luh %lum", (unsigned long)hours, (unsigned long)mins);
    } else {
        snprintf(buf, bufLen, "~%lum", (unsigned long)totalMinutes);
    }
}

// ---------------------------------------------------------------------------
// Show battery and system configuration stats on the e-ink display.
// Called when D15 is pressed; displays for STATS_DISPLAY_MS then restores.
// ---------------------------------------------------------------------------
void showStatsScreen() {
    float battV = readBatteryVoltage();
    uint8_t battPct = batteryPercentFromVoltage(battV);

    char remainBuf[24];
    estimateRemainingTime(battPct, remainBuf, sizeof(remainBuf));

    // Format interval strings
    char usbSample[16], usbDisplay[16];
    char batSample[16], batDisplay[16];
    char graphWin[16];

    formatDuration(USB_SAMPLE_INTERVAL_MS, usbSample, sizeof(usbSample));
    formatDuration(USB_DISPLAY_INTERVAL_MS, usbDisplay, sizeof(usbDisplay));
    formatDuration(BATTERY_SAMPLE_INTERVAL_MS, batSample, sizeof(batSample));
    formatDuration(BATTERY_DISPLAY_INTERVAL_MS, batDisplay, sizeof(batDisplay));
    snprintf(graphWin, sizeof(graphWin), "%dm", GRAPH_WINDOW_MINUTES);

    const char *powerStatus;
    if (usbPowerPresent) {
        powerStatus = battV < 3.0f ? "USB (no batt?)" : "USB";
    } else {
        powerStatus = deepSleepEnabled ? "Batt (deep sleep)" : "Batt (light sleep)";
    }

    display.clearBuffer();
    display.fillScreen(bgColor());
    display.setTextColor(fgColor());
    display.setTextWrap(false);

    // ── Left column: battery info ──
    int16_t x = SCREEN_PAD_LEFT + 4;
    int16_t y = SCREEN_PAD_TOP + 2;

    display.setTextSize(1);
    display.setCursor(x, y);
    display.print("BATTERY");
    y += 14;

    display.setTextSize(2);
    char voltLine[24];
    snprintf(voltLine, sizeof(voltLine), "%.2fV  %u%%", battV, battPct);
    display.setCursor(x, y);
    display.print(voltLine);
    y += 22;

    display.setTextSize(1);
    display.setCursor(x, y);
    display.printf("Remaining: %s", remainBuf);
    y += 12;

    display.setCursor(x, y);
    display.printf("Capacity: %u mAh", (unsigned)BATTERY_CAPACITY_MAH);
    y += 12;

    display.setCursor(x, y);
    display.printf("Power: %s", powerStatus);
    y += 18;

    // ── Left column: configuration ──
    display.setCursor(x, y);
    display.print("CONFIG");
    y += 14;

    display.setCursor(x, y);
    display.printf("Graph window: %s", graphWin);
    y += 12;

    display.setCursor(x, y);
    display.printf("USB:  sample %s / disp %s", usbSample, usbDisplay);
    y += 12;

    display.setCursor(x, y);
    display.printf("Batt: sample %s / disp %s", batSample, batDisplay);

    display.display();
}

void showStartupImage() {
#if HAS_STARTUP_IMAGE
    display.clearBuffer();
    display.drawBitmap(0, 0, startupImageBitmap, STARTUPIMAGE_WIDTH, STARTUPIMAGE_HEIGHT, fgColor());
    display.display();
#else
    showStatus("Started");
#endif
}

void showBuildInfoScreen() {
    char lineVersion[32];
    char lineHash[40];
    char lineEpoch[40];
    char lineState[40];

    snprintf(lineVersion, sizeof(lineVersion), "Build: %s", BUILD_VERSION_STR);
    snprintf(lineHash, sizeof(lineHash), "Hash: %s", BUILD_HASH_STR);
    snprintf(lineEpoch, sizeof(lineEpoch), "Epoch: %lu", (unsigned long)BUILD_EPOCH_UNIX);
    snprintf(lineState, sizeof(lineState), "State: %s", BUILD_DIRTY_STR);

    display.clearBuffer();
    display.fillScreen(bgColor());
    display.setTextColor(fgColor());
    display.setTextWrap(false);

    const int16_t baseX = SCREEN_PAD_LEFT + 8;
    const int16_t baseY = SCREEN_PAD_TOP;

    display.setTextSize(2);
    display.setCursor(baseX, baseY + 16);
    display.print("Build Info");

    display.setTextSize(1);
    display.setCursor(baseX, baseY + 52);
    display.print(lineVersion);
    display.setCursor(baseX, baseY + 72);
    display.print(lineHash);
    display.setCursor(baseX, baseY + 92);
    display.print(lineEpoch);
    display.setCursor(baseX, baseY + 112);
    display.print(lineState);
    display.display();

    delay(STARTUP_BUILD_INFO_MS);
}
