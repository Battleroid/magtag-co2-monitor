#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SCD30.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_ThinkInk.h>
#include <Adafruit_LIS3DH.h>

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
#define CYCLE_INTERVAL_MS  20000   // 20 seconds between readings
#define FLASH_DURATION_MS    500   // neopixel flash length

// ---------------------------------------------------------------------------
// History ring buffer  (30 min @ 20 s/sample = 90 samples)
// ---------------------------------------------------------------------------
#define HISTORY_LEN 90
static float histCO2[HISTORY_LEN];
static float histTempF[HISTORY_LEN];
static float histRH[HISTORY_LEN];
static int   histCount = 0;        // total samples ever recorded
static int   histHead  = 0;        // next write index

static void pushSample(float co2, float tempF, float rh) {
    histCO2[histHead]   = co2;
    histTempF[histHead] = tempF;
    histRH[histHead]    = rh;
    histHead = (histHead + 1) % HISTORY_LEN;
    histCount++;
}

// Return the number of valid samples currently in the buffer.
static int histSamples() { return (histCount < HISTORY_LEN) ? histCount : HISTORY_LEN; }

// Get the i-th oldest valid sample (0 = oldest).
static float histGet(const float *buf, int i) {
    int n = histSamples();
    int idx = (histHead - n + i + HISTORY_LEN) % HISTORY_LEN;
    return buf[idx];
}

// ---------------------------------------------------------------------------
// Display colour-scheme toggle  (D15 long-press)
// ---------------------------------------------------------------------------
#define INVERT_BTN   15           // D15 button
#define LONG_PRESS_MS 2000
static bool     inverted       = false;
static bool     btnWasPressed  = false;
static unsigned long btnDownAt = 0;

// Last displayed values (for immediate redraw on invert toggle)
static float lastCO2   = 0;
static float lastTempF = 0;
static float lastRH    = 0;
static bool  hasReading = false;

static uint16_t fgColor() { return inverted ? EPD_WHITE : EPD_BLACK; }
static uint16_t bgColor() { return inverted ? EPD_BLACK : EPD_WHITE; }

// Forward declaration so checkInvertButton can call it
void updateDisplay(float co2, float tempF, float rh);

static void checkInvertButton() {
    bool pressed = (digitalRead(INVERT_BTN) == LOW);
    if (pressed && !btnWasPressed) {
        btnDownAt = millis();               // rising edge
    }
    if (!pressed && btnWasPressed) {        // released
        if (millis() - btnDownAt >= LONG_PRESS_MS) {
            inverted = !inverted;
            Serial.printf("Display inverted: %s\n", inverted ? "ON" : "OFF");
            if (hasReading) {
                updateDisplay(lastCO2, lastTempF, lastRH);
            }
        }
    }
    btnWasPressed = pressed;
}

// ---------------------------------------------------------------------------
// Globals  (board-defined pins from MagTag variant header)
// ---------------------------------------------------------------------------
Adafruit_SCD30  scd30;
Adafruit_NeoPixel pixels(4, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
ThinkInk_290_Grayscale4_EAAMFGN display(EPD_DC, EPD_RESET, EPD_CS, -1, -1);
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

// ---------------------------------------------------------------------------
// Flash all four neopixels red for FLASH_DURATION_MS, then turn them off.
// ---------------------------------------------------------------------------
void flashNeopixelsRed() {
    digitalWrite(NEOPIXEL_POWER, LOW);
    delay(10);
    for (int i = 0; i < 4; i++)
        pixels.setPixelColor(i, pixels.Color(255, 0, 0));
    pixels.show();
    delay(FLASH_DURATION_MS);
    pixels.clear();
    pixels.show();
    digitalWrite(NEOPIXEL_POWER, HIGH);
}

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

    // Find min/max for auto-scaling
    float lo = histGet(buf, 0), hi = lo;
    for (int i = 1; i < n; i++) {
        float v = histGet(buf, i);
        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }
    // Ensure there's at least a small range so flat lines centre
    if (hi - lo < 0.1f) { lo -= 0.5f; hi += 0.5f; }

    uint16_t fg = fgColor();

    // Plot line segments, newest sample pinned to the right edge
    // and the graph shifts left as new samples arrive.
    auto yForVal = [&](float v) -> int16_t {
        float frac = (v - lo) / (hi - lo);               // 0..1
        return gy + gh - 1 - (int16_t)(frac * (gh - 1)); // top = high
    };

    // x positions: newest (index n-1) at right edge, oldest at left
    // spread across the full HISTORY_LEN width so the graph scrolls
    // left as the buffer fills up.
    for (int i = 1; i < n; i++) {
        int16_t x0 = gx + gw - 1 - (int32_t)(n - i)     * (gw - 1) / (HISTORY_LEN - 1);
        int16_t x1 = gx + gw - 1 - (int32_t)(n - 1 - i) * (gw - 1) / (HISTORY_LEN - 1);
        int16_t y0 = yForVal(histGet(buf, i - 1));
        int16_t y1 = yForVal(histGet(buf, i));
        display.drawLine(x0, y0, x1, y1, fg);
    }
}

// ---------------------------------------------------------------------------
// Redraw the e-ink display.
//   Left side:  values right-aligned, vertically centred to each graph row.
//   Right side: three line graphs (CO2 / Temp / RH) over last 30 min.
// ---------------------------------------------------------------------------
// Layout constants  (screen is 296 x 128)
#define TEXT_RIGHT   140          // right edge for value text
#define GRAPH_X      152          // left edge of graphs
#define GRAPH_W      136          // graph width  (296 - 152 - 8 padding)

// CO2 graph is taller to match the larger text
#define GRAPH_H_CO2  40
#define GRAPH_H_STD  30           // temp & RH graphs
#define GRAPH_GAP    6
// Total: 40 + 30 + 30 + 2*6 = 112.  Centred: (128-112)/2 = 8
#define GRAPH_Y0     8

void updateDisplay(float co2, float tempF, float rh) {
    uint16_t fg = fgColor();

    display.clearBuffer();
    if (inverted) display.fillScreen(EPD_BLACK);
    display.setTextColor(fg);

    char buf[32];

    // Graph y positions
    int16_t gyCO2  = GRAPH_Y0;
    int16_t gyTemp = gyCO2  + GRAPH_H_CO2 + GRAPH_GAP;
    int16_t gyRH   = gyTemp + GRAPH_H_STD + GRAPH_GAP;

    // ── CO2 ── text centred on CO2 graph row (textSize 3 = 24px high)
    display.setTextSize(3);
    snprintf(buf, sizeof(buf), "%d", (int)co2);
    printRightAligned(TEXT_RIGHT, gyCO2 + GRAPH_H_CO2 / 2 - 12, buf);

    // ── Temperature ── text centred on temp graph row (textSize 2 = 16px high)
    display.setTextSize(2);
    snprintf(buf, sizeof(buf), "%.1f F", tempF);
    printRightAligned(TEXT_RIGHT, gyTemp + GRAPH_H_STD / 2 - 8, buf);

    // ── Humidity ── text centred on RH graph row
    snprintf(buf, sizeof(buf), "%.1f %%", rh);
    printRightAligned(TEXT_RIGHT, gyRH + GRAPH_H_STD / 2 - 8, buf);

    // ── Graphs ──
    int n = histSamples();
    drawGraph(histCO2,   n, GRAPH_X, gyCO2,  GRAPH_W, GRAPH_H_CO2);
    drawGraph(histTempF, n, GRAPH_X, gyTemp,  GRAPH_W, GRAPH_H_STD);
    drawGraph(histRH,    n, GRAPH_X, gyRH,    GRAPH_W, GRAPH_H_STD);

    display.display();
}

// ---------------------------------------------------------------------------
// Show a single-line status message on the e-ink display.
// ---------------------------------------------------------------------------
void showStatus(const char *msg) {
    display.clearBuffer();
    display.setTextColor(fgColor());
    if (inverted) display.fillScreen(EPD_BLACK);
    display.setTextSize(2);
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);
    int16_t x = (display.width() - (int16_t)w) / 2;
    int16_t y = (display.height() - (int16_t)h) / 2;
    display.setCursor(x, y);
    display.print(msg);
    display.display();
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("MagTag CO2 Monitor starting...");

    // ── Neopixels ──
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
    pixels.begin();
    pixels.setBrightness(50);
    pixels.clear();
    pixels.show();

    // ── Invert button (D15) ──
    pinMode(INVERT_BTN, INPUT_PULLUP);

    // ── LIS3DH accelerometer (on-board) ──
    if (!lis.begin(0x19)) {
        Serial.println("WARNING: LIS3DH not found");
    } else {
        Serial.println("LIS3DH initialised");
    }

    // ── E-ink display ──
    display.begin(THINKINK_MONO);
    showStatus("Started");
    Serial.println("Display: Started");

    delay(5000);

    // ── I2C + SCD-30 sensor ──
    Wire.begin(SDA, SCL);

    if (!scd30.begin()) {
        Serial.println("ERROR: SCD-30 not found – check wiring");
        showStatus("Sensor not found!");
        while (true) { delay(1000); }
    }

    Serial.println("SCD-30 initialised");
}

// ---------------------------------------------------------------------------
// Main loop – read → flash → display → sleep, every 20 s.
//   Also polls the D15 button for the invert toggle.
// ---------------------------------------------------------------------------
void loop() {
    // Poll invert button throughout the wait
    unsigned long loopStart = millis();

    // Wait for sensor data (up to 5 s)
    while (!scd30.dataReady()) {
        checkInvertButton();
        if (millis() - loopStart > 5000) {
            Serial.println("Timeout waiting for SCD-30 data");
            break;
        }
        delay(50);
    }

    if (scd30.read()) {
        float co2   = scd30.CO2;
        float tempC = scd30.temperature;
        float tempF = tempC * 9.0f / 5.0f + 32.0f;
        float rh    = scd30.relative_humidity;

        Serial.printf("CO2: %.0f ppm | Temp: %.1f F | RH: %.1f %%\n",
                       co2, tempF, rh);

        pushSample(co2, tempF, rh);
        lastCO2 = co2; lastTempF = tempF; lastRH = rh;
        hasReading = true;
        flashNeopixelsRed();
        updateDisplay(co2, tempF, rh);
    } else {
        Serial.println("Failed to read SCD-30");
    }

    // Idle for the remainder of the cycle, polling the button
    unsigned long elapsed = millis() - loopStart;
    unsigned long remaining = (elapsed < CYCLE_INTERVAL_MS) ? CYCLE_INTERVAL_MS - elapsed : 0;
    unsigned long waitEnd = millis() + remaining;
    while (millis() < waitEnd) {
        checkInvertButton();
        delay(50);
    }
}
