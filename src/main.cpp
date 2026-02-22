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
// Globals  (using board-defined pins: PIN_NEOPIXEL, NEOPIXEL_POWER,
//           EPD_DC, EPD_RESET, EPD_CS from the MagTag variant header)
// ---------------------------------------------------------------------------
Adafruit_SCD30  scd30;
Adafruit_NeoPixel pixels(4, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
ThinkInk_290_Grayscale4_EAAMFGN display(EPD_DC, EPD_RESET, EPD_CS, -1, -1);
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

// ---------------------------------------------------------------------------
// Flash all four neopixels red for FLASH_DURATION_MS, then turn them off.
// ---------------------------------------------------------------------------
void flashNeopixelsRed() {
    // Power on the neopixels (active-LOW enable on MagTag)
    digitalWrite(NEOPIXEL_POWER, LOW);
    delay(10);

    for (int i = 0; i < 4; i++) {
        pixels.setPixelColor(i, pixels.Color(255, 0, 0));
    }
    pixels.show();

    delay(FLASH_DURATION_MS);

    pixels.clear();
    pixels.show();

    // Power off neopixels to save energy
    digitalWrite(NEOPIXEL_POWER, HIGH);
}

// ---------------------------------------------------------------------------
// Redraw the e-ink display with current sensor values.
//   Display is 296 x 128 in landscape orientation.
// ---------------------------------------------------------------------------
void updateDisplay(float co2, float temperature, float humidity) {
    display.clearBuffer();
    display.setTextColor(EPD_BLACK);

    // ── Title ──
    display.setTextSize(2);
    display.setCursor(10, 8);
    display.print("CO2 Monitor");

    // Separator line
    display.drawFastHLine(0, 28, 296, EPD_BLACK);

    // ── CO2 (large) ──
    display.setTextSize(4);
    display.setCursor(10, 38);
    display.print((int)co2);

    display.setTextSize(2);
    display.setCursor(display.getCursorX() + 4, 50);
    display.print("ppm");

    // ── Temperature ──
    display.setTextSize(2);
    display.setCursor(10, 82);
    display.print("Temp: ");
    display.print(temperature, 1);
    display.print(" C");

    // ── Humidity ──
    display.setCursor(10, 106);
    display.print("RH:   ");
    display.print(humidity, 1);
    display.print(" %");

    display.display();
}

// ---------------------------------------------------------------------------
// Show a single-line status message on the e-ink display.
// ---------------------------------------------------------------------------
void showStatus(const char *msg) {
    display.clearBuffer();
    display.setTextColor(EPD_BLACK);
    display.setTextSize(2);
    display.setCursor(10, 56);
    display.print(msg);
    display.display();
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    // Brief pause so USB-CDC serial has time to enumerate (not blocking)
    delay(2000);

    Serial.println("MagTag CO2 Monitor starting...");

    // ── Neopixels ──
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);   // off initially
    pixels.begin();
    pixels.setBrightness(50);
    pixels.clear();
    pixels.show();

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

    // Wait 5 seconds before beginning sensor loop
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
// ---------------------------------------------------------------------------
void loop() {
    // Poll until the sensor has a fresh reading (timeout after 5 s)
    unsigned long t0 = millis();
    while (!scd30.dataReady()) {
        if (millis() - t0 > 5000) {
            Serial.println("Timeout waiting for SCD-30 data");
            break;
        }
        delay(100);
    }

    if (scd30.read()) {
        float co2  = scd30.CO2;
        float temp = scd30.temperature;
        float hum  = scd30.relative_humidity;

        Serial.printf("CO2: %.0f ppm | Temp: %.1f C | RH: %.1f %%\n",
                       co2, temp, hum);

        flashNeopixelsRed();
        updateDisplay(co2, temp, hum);
    } else {
        Serial.println("Failed to read SCD-30");
    }

    delay(CYCLE_INTERVAL_MS);
}
