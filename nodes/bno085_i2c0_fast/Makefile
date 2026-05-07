# Device tree overlay: I2C0 @ 400 kHz for BNO085 / shared sensors on bus 0
#
# Mirrors power_control/sys_heartbeat: compile overlay, install to /boot/overlay-user,
# register via orangepiEnv/manage_overlays.sh user_overlays.

OVERLAY_NAME := argo-bno085-i2c0-fast
DTS_FILE := $(OVERLAY_NAME).dts
# Plugin overlays built with -@ match Orange Pi user overlay naming (see ph4 heartbeat)
DTBO_FILE := $(OVERLAY_NAME)-overlay.dtbo
PREPROCESSED := $(OVERLAY_NAME).dts.preprocessed

OVERLAY_INSTALL_DIR := /boot/overlay-user
MANAGE_OVERLAYS_SCRIPT := ../../orangepiEnv/manage_overlays.sh

KDIR := /lib/modules/$(shell uname -r)/build

.PHONY: all clean install uninstall help

all: $(DTBO_FILE)

# Preprocess then compile so dtcb accepts -@ (__symbols__) like other Argo overlays
$(DTBO_FILE): $(DTS_FILE)
	@if [ ! -d "$(KDIR)" ]; then \
		echo "Kernel headers missing at $(KDIR); building overlay without cpp includes..."; \
		dtc -I dts -O dtb -o $(DTBO_FILE) -@ $(DTS_FILE); \
	else \
		cpp -nostdinc -undef -x assembler-with-cpp -I "$(KDIR)/include" $(DTS_FILE) > $(PREPROCESSED); \
		dtc -O dtb -o $(DTBO_FILE) -@ $(PREPROCESSED); \
	fi
	@echo "Built $(DTBO_FILE)"

clean:
	rm -f $(DTBO_FILE) $(PREPROCESSED)

help:
	@echo "Targets:"
	@echo "  all       - Build $(DTBO_FILE)"
	@echo "  install   - Copy to $(OVERLAY_INSTALL_DIR) and add to user_overlays"
	@echo "  uninstall - Remove dtbo and user_overlays entry"
	@echo "Reboot after install. Ensure all devices on i2c-0 support Fast mode (400 kHz)."

install: $(DTBO_FILE)
	@echo "Installing $(DTBO_FILE) to $(OVERLAY_INSTALL_DIR)..."
	sudo mkdir -p $(OVERLAY_INSTALL_DIR)
	sudo cp $(DTBO_FILE) $(OVERLAY_INSTALL_DIR)/
	sudo bash $(MANAGE_OVERLAYS_SCRIPT) add $(OVERLAY_NAME)
	@echo "Done. Reboot to apply. Verify with: dmesg | grep i2c-0"

uninstall:
	sudo rm -f $(OVERLAY_INSTALL_DIR)/$(DTBO_FILE)
	sudo bash $(MANAGE_OVERLAYS_SCRIPT) remove $(OVERLAY_NAME)
	@echo "Removed from boot config. Reboot to restore default I2C clock."
