# Makefile for Argo PH4 Green LED Heartbeat Overlay

# This Makefile compiles and installs the device tree overlay for the
# green LED on PH4, configuring it as a system heartbeat indicator.
# It uses the centralized overlay management script.

# Overlay name, used for filenames and installation
OVERLAY_NAME = argo-ph4-led-overlay

# Source and target files
DTS_FILE = $(OVERLAY_NAME).dts
DTS_PREPROCESSED_FILE = $(OVERLAY_NAME)-overlay.dts.preprocessed
DTBO_FILE = $(OVERLAY_NAME)-overlay.dtbo

# Kernel source directory for includes, needed by the preprocessor
KDIR := /lib/modules/$(shell uname -r)/build

# Installation directory for device tree overlays
OVERLAY_INSTALL_DIR = /boot/overlay-user

# Path to the centralized overlay management script
MANAGE_OVERLAYS_SCRIPT = ../../orangepiEnv/manage_overlays.sh

.PHONY: all clean install uninstall

all: $(DTBO_FILE)

# Preprocess and compile the .dts file to a .dtbo file
$(DTBO_FILE): $(DTS_FILE)
	@echo "Preprocessing Device Tree Overlay: $(DTS_FILE)"
	cpp -nostdinc -undef -x assembler-with-cpp -I $(KDIR)/include $(DTS_FILE) > $(DTS_PREPROCESSED_FILE)
	@echo "Compiling preprocessed overlay..."
	dtc -O dtb -o $(DTBO_FILE) -@ $(DTS_PREPROCESSED_FILE)
	@echo "Successfully compiled $(DTBO_FILE)"

# Install the overlay and update orangepiEnv.txt
install: $(DTBO_FILE) install_service
	@echo "Installing overlay..."
	sudo cp $(DTBO_FILE) $(OVERLAY_INSTALL_DIR)/
	@echo "Overlay file copied to $(OVERLAY_INSTALL_DIR)"
	@echo "Adding overlay to orangepiEnv.txt..."
	sudo bash $(MANAGE_OVERLAYS_SCRIPT) add $(OVERLAY_NAME)
	@echo "Installation complete. Please reboot for changes to take effect."

# Install the permissions service
install_service:
	@echo "--- Installing permissions service ---"
	sudo cp $(PWD)/argo-ph4-led-postinit.sh /usr/local/bin/
	sudo cp $(PWD)/argo-ph4-led-perms.service /etc/systemd/system/
	sudo systemctl daemon-reload
	sudo systemctl enable argo-ph4-led-perms.service
	@echo "Permissions service installed and enabled."

# Uninstall the overlay and update orangepiEnv.txt
uninstall: uninstall_service
	@echo "Uninstalling overlay..."
	sudo rm -f $(OVERLAY_INSTALL_DIR)/$(DTBO_FILE)
	@echo "Overlay file removed from $(OVERLAY_INSTALL_DIR)"
	@echo "Removing overlay from orangepiEnv.txt..."
	sudo bash $(MANAGE_OVERLAYS_SCRIPT) remove $(OVERLAY_NAME)
	@echo "Uninstallation complete. Please reboot for changes to take effect."

uninstall_service:
	@echo "--- Uninstalling permissions service ---"
	sudo systemctl disable argo-ph4-led-perms.service
	sudo rm -f /etc/systemd/system/argo-ph4-led-perms.service
	sudo rm -f /usr/local/bin/argo-ph4-led-postinit.sh
	sudo systemctl daemon-reload
	@echo "Permissions service uninstalled."

# Clean compiled and intermediate files
clean:
	@echo "Cleaning up compiled files..."
	rm -f $(DTBO_FILE) $(DTS_PREPROCESSED_FILE)
	@echo "Cleanup complete."
