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

### Graph rendering

- [src/main.cpp](src/main.cpp#L45) `GRAPH_LINE_THICKNESS`
- [src/main.cpp](src/main.cpp#L46) `GRAPH_UNDERLINE_LIGHT_FILL` (master toggle)
- [src/main.cpp](src/main.cpp#L47) `GRAPH_UNDERLINE_FILL_SHADE`
- [src/main.cpp](src/main.cpp#L48) `GRAPH_UNDERLINE_FILL_RISING_ONLY`
- [src/main.cpp](src/main.cpp#L39) `GRAPH_WINDOW_MINUTES`

### Sampling & display cadence

- [src/main.cpp](src/main.cpp#L27) `CYCLE_INTERVAL_MS`
- [src/main.cpp](src/main.cpp#L28) `USB_SAMPLE_INTERVAL_MS`
- [src/main.cpp](src/main.cpp#L29) `USB_DISPLAY_INTERVAL_MS`
- [src/main.cpp](src/main.cpp#L30) `BATTERY_SAMPLE_INTERVAL_MS`
- [src/main.cpp](src/main.cpp#L31) `BATTERY_DISPLAY_INTERVAL_MS`

### Button mapping

- [src/main.cpp](src/main.cpp#L73) `MODE_BTN`
- [src/main.cpp](src/main.cpp#L74) `CAROUSEL_BTN`

### Battery warning thresholds

- [src/main.cpp](src/main.cpp#L117) `BATTERY_WARN_50_PERCENT`
- [src/main.cpp](src/main.cpp#L120) `BATTERY_CRITICAL_PERCENT`

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
