# MagTag CO2 Monitor

CO2 / temperature / humidity monitor for Adafruit MagTag (ESP32-S2 + 2.9" grayscale e-ink), built with PlatformIO.

## Quick start (Linux)

From the repository root:

```bash
chmod +x setup_linux.sh
./setup_linux.sh
source .venv/bin/activate
make build
make upload
make monitor
```

The setup script installs a local Python virtual environment in `.venv`, then installs:
- `platformio` (build/upload toolchain)
- `pillow` (used by `tools/image_to_epd_bitmap.py`)

## Repository setup for a new user

1. Clone the repo.
2. Run `./setup_linux.sh`.
3. Activate the virtual environment each shell session:
   - `source .venv/bin/activate`
4. Build and upload:
   - `make build`
   - `make upload`

If serial upload/monitor reports permission errors on Linux, add your user to `dialout`:

```bash
sudo usermod -aG dialout "$USER"
```

Then log out and back in.

## Common commands

- Build firmware: `make build`
- Upload firmware: `make upload`
- Serial monitor (115200): `make monitor`
- Build + upload + monitor: `make all`
- Clean build artifacts: `make clean`
- Full clean (`.pio`): `make distclean`
- Convert all images in `assets/images/`: `make images`

## Customizable options

Most runtime behavior is configured via `#define` values in [src/main.cpp](src/main.cpp).

### Timing & intervals

| Definition | Default | Acceptable values |
| --- | --- | --- |
| `CYCLE_INTERVAL_MS` | `20000` | Positive integer milliseconds |
| `USB_SAMPLE_INTERVAL_MS` | `15000` | Positive integer milliseconds |
| `USB_DISPLAY_INTERVAL_MS` | `30000` | Positive integer milliseconds |
| `BATTERY_SAMPLE_INTERVAL_MS` | `60000` | Positive integer milliseconds |
| `BATTERY_DISPLAY_INTERVAL_MS` | `300000` | Positive integer milliseconds |
| `BATTERY_USB_POLL_INTERVAL_MS` | `10000` | Positive integer milliseconds |
| `POWER_CHECK_INTERVAL_MS` | `5000` | Positive integer milliseconds |
| `FLASH_DURATION_MS` | `500` | Positive integer milliseconds |
| `STARTUP_FLASH_MS` | `220` | Positive integer milliseconds |
| `INVERT_FLASH_MS` | `150` | Positive integer milliseconds |

### Graph & layout

| Definition | Default | Acceptable values |
| --- | --- | --- |
| `GRAPH_WINDOW_MINUTES` | `15` | Positive integer minutes (`>= 1`) |
| `GRAPH_LINE_THICKNESS` | `1` | Integer pixel thickness (`>= 1`) |
| `GRAPH_UNDERLINE_LIGHT_FILL` | `1` | `0` (off), `1` (on) |
| `GRAPH_UNDERLINE_FILL_SHADE` | `EPD_DARK` | `EPD_WHITE`, `EPD_LIGHT`, `EPD_DARK`, `EPD_GRAY` |
| `GRAPH_UNDERLINE_FILL_RISING_ONLY` | `1` | `0` (fill all segments), `1` (fill only rising segments) |
| `TEXT_RIGHT_BALANCED` | `140` | Pixel X coordinate within display width |
| `GRAPH_X_BALANCED` | `152` | Pixel X coordinate within display width |
| `GRAPH_W_BALANCED` | `136` | Positive pixel width |
| `TEXT_RIGHT_GRAPH_HEAVY` | `108` | Pixel X coordinate within display width |
| `GRAPH_X_GRAPH_HEAVY` | `120` | Pixel X coordinate within display width |
| `GRAPH_W_GRAPH_HEAVY` | `168` | Positive pixel width |
| `GRAPH_H_CO2` | `40` | Positive pixel height |
| `GRAPH_H_STD` | `30` | Positive pixel height |
| `GRAPH_GAP` | `6` | Non-negative pixel spacing |
| `SCREEN_PAD_LEFT` | `0` | Non-negative pixels |
| `SCREEN_PAD_RIGHT` | `10` | Non-negative pixels |
| `SCREEN_PAD_TOP` | `0` | Non-negative pixels |
| `SCREEN_PAD_BOTTOM` | `0` | Non-negative pixels |

### Button mapping

| Definition | Default | Acceptable values |
| --- | --- | --- |
| `MODE_BTN` | `12` | Valid ESP32-S2 GPIO used by a button |
| `CAROUSEL_BTN` | `14` | Valid ESP32-S2 GPIO used by a button |

### NeoPixel colors

| Definition | Default | Acceptable values |
| --- | --- | --- |
| `USB_SAMPLE_FLASH_R` | `15` | Integer `0..255` |
| `USB_SAMPLE_FLASH_G` | `15` | Integer `0..255` |
| `USB_SAMPLE_FLASH_B` | `15` | Integer `0..255` |
| `STARTUP_FLASH_R` | `0` | Integer `0..255` |
| `STARTUP_FLASH_G` | `180` | Integer `0..255` |
| `STARTUP_FLASH_B` | `0` | Integer `0..255` |
| `USB_CONNECTED_FLASH_R` | `0` | Integer `0..255` |
| `USB_CONNECTED_FLASH_G` | `0` | Integer `0..255` |
| `USB_CONNECTED_FLASH_B` | `200` | Integer `0..255` |
| `USB_DISCONNECTED_FLASH_R` | `220` | Integer `0..255` |
| `USB_DISCONNECTED_FLASH_G` | `90` | Integer `0..255` |
| `USB_DISCONNECTED_FLASH_B` | `0` | Integer `0..255` |
| `MODE_TOGGLE_FLASH_R` | `24` | Integer `0..255` |
| `MODE_TOGGLE_FLASH_G` | `24` | Integer `0..255` |
| `MODE_TOGGLE_FLASH_B` | `24` | Integer `0..255` |
| `CAROUSEL_ON_FLASH_R` | `140` | Integer `0..255` |
| `CAROUSEL_ON_FLASH_G` | `0` | Integer `0..255` |
| `CAROUSEL_ON_FLASH_B` | `110` | Integer `0..255` |
| `CAROUSEL_OFF_FLASH_R` | `255` | Integer `0..255` |
| `CAROUSEL_OFF_FLASH_G` | `0` | Integer `0..255` |
| `CAROUSEL_OFF_FLASH_B` | `0` | Integer `0..255` |
| `GENERIC_RED_FLASH_R` | `255` | Integer `0..255` |
| `GENERIC_RED_FLASH_G` | `0` | Integer `0..255` |
| `GENERIC_RED_FLASH_B` | `0` | Integer `0..255` |

### Battery behavior

| Definition | Default | Acceptable values |
| --- | --- | --- |
| `LOW_BATTERY_PERCENT` | `10` | Integer percentage `0..100` |
| `LOW_BATTERY_FLASH_INTERVAL_MS` | `900000UL` | Positive integer milliseconds |
| `BATTERY_WARN_50_PERCENT` | `50` | Integer percentage `0..100` |
| `BATTERY_WARN_50_DURATION_MS` | `30000UL` | Positive integer milliseconds |
| `BATTERY_CRITICAL_PERCENT` | `10` | Integer percentage `0..100` |
| `BATTERY_CRITICAL_INTERVAL_MS` | `1800000UL` | Positive integer milliseconds |
| `BATTERY_CRITICAL_DURATION_MS` | `60000UL` | Positive integer milliseconds |
| `BATTERY_CRITICAL_BLINK_MS` | `600UL` | Positive integer milliseconds |
| `BATTERY_CRITICAL_LED_R` | `30` | Integer `0..255` |
| `BATTERY_CRITICAL_LED_G` | `0` | Integer `0..255` |
| `BATTERY_CRITICAL_LED_B` | `0` | Integer `0..255` |
| `BATTERY_LEVEL_4_MIN_PERCENT` | `81` | Integer percentage `0..100` |
| `BATTERY_LEVEL_3_MIN_PERCENT` | `50` | Integer percentage `0..100` |
| `BATTERY_LEVEL_2_MIN_PERCENT` | `30` | Integer percentage `0..100` |
| `BATTERY_LEVEL_4_LED_R` | `0` | Integer `0..255` |
| `BATTERY_LEVEL_4_LED_G` | `180` | Integer `0..255` |
| `BATTERY_LEVEL_4_LED_B` | `0` | Integer `0..255` |
| `BATTERY_LEVEL_3_LED_R` | `220` | Integer `0..255` |
| `BATTERY_LEVEL_3_LED_G` | `180` | Integer `0..255` |
| `BATTERY_LEVEL_3_LED_B` | `0` | Integer `0..255` |
| `BATTERY_LEVEL_2_LED_R` | `255` | Integer `0..255` |
| `BATTERY_LEVEL_2_LED_G` | `90` | Integer `0..255` |
| `BATTERY_LEVEL_2_LED_B` | `0` | Integer `0..255` |
| `BATTERY_LEVEL_1_LED_R` | `255` | Integer `0..255` |
| `BATTERY_LEVEL_1_LED_G` | `0` | Integer `0..255` |
| `BATTERY_LEVEL_1_LED_B` | `0` | Integer `0..255` |
| `BATTERY_LEVEL_FLASH_ON_MS` | `180` | Positive integer milliseconds |
| `BATTERY_LEVEL_FLASH_OFF_MS` | `120` | Positive integer milliseconds |
| `BATTERY_STARTUP_SWEEP_MS` | `90` | Positive integer milliseconds |
| `BATTERY_STARTUP_LEVEL_FLASHES` | `3` | Positive integer count |

### Sensor & history

| Definition | Default | Acceptable values |
| --- | --- | --- |
| `SCD30_I2C_ADDR` | `0x61` | Valid 7-bit I2C address for the sensor |
| `SCD30_CMD_STOP_MEASUREMENTS` | `0x0104` | Sensor protocol command constant |
| `SCD30_WARMUP_MS` | `25000` | Positive integer milliseconds |
| `HISTORY_LEN` | `90` | Positive integer sample slots |

## Startup image workflow

Generate image headers from files in `assets/images/`:

```bash
make images
```

Or convert one image manually:

```bash
make image IMAGE_INPUT=assets/images/startup_image.png IMAGE_OUTPUT=src/startup_image.h IMAGE_SYMBOL=startupImage
```

## PlatformIO environment

Board/environment is configured in [platformio.ini](platformio.ini):
- `env:magtag`
- board: `adafruit_magtag29_esp32s2`
- framework: `arduino`

## Notes

- The project now uses fixed black-on-white display rendering (inversion mode removed).
- The Makefile prefers local `.venv` tools automatically when available.
