.PHONY: build upload monitor clean all install image

PLATFORMIO := platformio
PYTHON := python3
IMAGE_INPUT ?= smile.png
IMAGE_OUTPUT ?= src/startup_image.h
IMAGE_SYMBOL ?= startupImage

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

# Convert an image for startup bitmap rendering on MagTag e-ink.
# Example: make image IMAGE_INPUT=smile.png
image:
	$(PYTHON) tools/image_to_epd_bitmap.py \
		--input "$(IMAGE_INPUT)" \
		--output "$(IMAGE_OUTPUT)" \
		--symbol "$(IMAGE_SYMBOL)" \
		--width 296 \
		--height 128

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
