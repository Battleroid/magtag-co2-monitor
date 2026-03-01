# Copilot Instructions — MagTag CO2 Monitor

## Project overview

ESP32-S2 (Adafruit MagTag 2.9") CO2 monitor built with PlatformIO + Arduino.
Source is in `src/` and `include/`. Build system is PlatformIO wrapped by a
Makefile.

## Running terminal commands reliably

PlatformIO builds take ~18-25 seconds. The VS Code terminal integration has
chronic issues with commands being interrupted by signals (SIGINT / ^C),
output being swallowed, or pipe chains (`| tail`) causing premature
termination.

### Preferred: use a subagent for builds

The most reliable way to run a build is to delegate it to a subagent:

```
runSubagent(
  description: "Build PlatformIO project",
  prompt: "Run `cd /home/cweed/git/magtag-co2-mon && pio run 2>&1 | tail -20`
           and report whether the build succeeded or failed, including any
           error messages."
)
```

This avoids all terminal signal/buffering issues.

### Fallback: background build with deferred output check

If you must use `run_in_terminal` directly:

1. **Launch** the build as a background job writing to a temp file:
   ```
   cd /home/cweed/git/magtag-co2-mon; nohup pio run > /tmp/pio_build.txt 2>&1 &
   echo "STARTED"
   ```
   Use `isBackground: false` with a short timeout (~10 s) — it returns after
   the `echo`.

2. **Wait**, then **check** with a *separate* terminal call (~30 s later):
   ```
   grep -E 'SUCCESS|FAILED|error:' /tmp/pio_build.txt | head -10; echo DONE
   ```

### What NOT to do

- **Do not** run `pio run` as a foreground blocking call — it will almost
  always get interrupted by ^C before it finishes.
- **Do not** chain `pio run 2>&1 | tail -N` in a single foreground call — the
  pipe makes the build susceptible to SIGPIPE / early termination.
- **Do not** run multiple terminal commands in parallel — they share
  state and interfere with each other.
- **Do not** try more than twice with `run_in_terminal` before switching to
  the subagent approach.

## Build commands

| Task              | Command                                |
|-------------------|----------------------------------------|
| Build firmware    | `make build` or `pio run`              |
| Upload to board   | `make upload`                          |
| Serial monitor    | `make monitor`                         |
| Convert images    | `make images`                          |
| Clean build       | `make clean`                           |

## Key file locations

| Path               | Purpose                                 |
|--------------------|-----------------------------------------|
| `src/main.cpp`     | Setup/loop, power-mode branching        |
| `src/power.cpp`    | USB detection, sleep entry/exit         |
| `src/sensor.cpp`   | SCD-30 I2C communication               |
| `src/display.cpp`  | E-ink rendering, graph drawing          |
| `src/leds.cpp`     | NeoPixel effects                        |
| `src/state.cpp`    | Global state, history ring buffer       |
| `src/input.cpp`    | Button task handlers (FreeRTOS)         |
| `src/audio.cpp`    | Speaker/jingle routines                 |
| `include/config.h` | All tunable constants (timing, pins, colors) |
| `include/state.h`  | Extern declarations, RTC-persisted vars |
| `include/types.h`  | Enum / type definitions                 |
| `platformio.ini`   | Build configuration                     |
| `Makefile`         | Developer-facing build/upload/image targets |

## Hardware notes

- **Board**: Adafruit MagTag 2.9" (ESP32-S2, native USB CDC, no UART)
- **Sensor**: Sensirion SCD-30 via I2C (address 0x61, ~2 s measurement
  interval, ~25 s cold-start warm-up)
- **Display**: 2.9" grayscale e-ink (ThinkInk 290 Grayscale4)
- **LEDs**: 4× NeoPixel (active-high power enable on `NEOPIXEL_POWER`)
- **Buttons**: D12 (mode), D14 (carousel), D11 (sleep toggle, 2 s hold),
  D15 (stats) — all active-low with internal pull-ups
- **Serial**: ESP32-S2 USB CDC — output is silently dropped when no host
  serial monitor is attached. Always add `Serial.flush()` after important
  prints.

## Architecture

Three execution paths in `loop()`:

1. **USB always-on** — 15 s sampling, FreeRTOS button tasks, full graphs
2. **Light sleep (battery, deep sleep disabled)** — 60 s sampling, SCD-30
   stays running during sleep, RAM preserved, ext1 button wake
3. **Deep sleep (battery, deep sleep enabled)** — 60 s sampling, RTC state
   persistence, SCD-30 stopped between cycles, compact display (no graphs)
