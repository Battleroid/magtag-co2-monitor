.PHONY: build upload monitor clean all install

PLATFORMIO := platformio

# Build firmware
build:
	@mkdir -p .pio/build/magtag
	$(PLATFORMIO) run

# Build and upload to board
# NOTE: On first flash, you may need to enter the ROM bootloader by
# holding BOOT, pressing RESET, then releasing BOOT.
upload:
	@mkdir -p .pio/build/magtag
	$(PLATFORMIO) run --target upload

# Open serial monitor (115200 baud)
monitor:
	$(PLATFORMIO) device monitor

# Build, upload, and open serial monitor
all: upload monitor

# Remove build artifacts
clean:
	$(PLATFORMIO) run --target clean

# Full clean (removes .pio directory entirely)
distclean:
	rm -rf .pio

# Install PlatformIO CLI (requires pip)
install:
	pip install platformio
