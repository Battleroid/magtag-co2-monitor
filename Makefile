.PHONY: build upload monitor clean all install image images

PLATFORMIO := platformio
PYTHON := python3
IMAGE_WIDTH := 296
IMAGE_HEIGHT := 128

# Discover all source images and derive header targets
IMAGE_SRCS := $(wildcard assets/images/*.png assets/images/*.jpg assets/images/*.jpeg assets/images/*.bmp)
IMAGE_STEMS := $(basename $(notdir $(IMAGE_SRCS)))
IMAGE_HDRS := $(addprefix src/,$(addsuffix .h,$(IMAGE_STEMS)))

# Convert snake_case filename to camelCase symbol name
# e.g. startup_image -> startupImage
define to_camel
$(shell echo '$(1)' | sed -E 's/_([a-z])/\U\1/g')
endef

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

# Convert ALL images in assets/images/ to src/ headers.
images: $(IMAGE_HDRS)

# Pattern rule: generate src/<name>.h from any matching asset image.
# Searches for the first file in assets/images/ whose basename (minus extension) matches the stem.
.SECONDEXPANSION:
src/%.h: $$(firstword $$(wildcard assets/images/%.png assets/images/%.jpg assets/images/%.jpeg assets/images/%.bmp))
	$(PYTHON) tools/image_to_epd_bitmap.py \
		--input "$<" \
		--output "$@" \
		--symbol "$(call to_camel,$*)" \
		--width $(IMAGE_WIDTH) \
		--height $(IMAGE_HEIGHT)

# Convert a single image (backward-compatible).
# Example: make image IMAGE_INPUT=assets/images/smile.png IMAGE_OUTPUT=src/smile.h IMAGE_SYMBOL=smile
IMAGE_INPUT ?= assets/images/startup_image.png
IMAGE_OUTPUT ?= src/startup_image.h
IMAGE_SYMBOL ?= startupImage
image:
	$(PYTHON) tools/image_to_epd_bitmap.py \
		--input "$(IMAGE_INPUT)" \
		--output "$(IMAGE_OUTPUT)" \
		--symbol "$(IMAGE_SYMBOL)" \
		--width $(IMAGE_WIDTH) \
		--height $(IMAGE_HEIGHT)

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
