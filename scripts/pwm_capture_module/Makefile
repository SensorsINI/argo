# builds radio-servo kernel module for argo and handles system installation

MOD=argo_radio_servo_module
OVERLAY=argo_radio_servo_overlay
SYS=/sys/kernel/argo_radio_servo
obj-m := $(MOD).o       
 
# get build date in readable format with no spaces
BUILD_DATE_CMD = $(shell date +%Y-%m-%d)
EXTRA_CFLAGS += -DBUILD_DATE='\"$(BUILD_DATE_CMD)\"' 
KDIR := /lib/modules/$(shell uname -r)/build
MDIR := /lib/modules/$(shell uname -r)/kernel/drivers/misc/argo
PWD := $(shell pwd)

.PHONY: all clean overlay install_overlay insmod rmmod watch test_pwm install_module install_rules post_insmod_setup # Ensure all targets are listed here

# all target does everything needed to build the module and overlay
all: module overlay install_module install_overlay install_boot_env install_rules install_permissions_service

module:
	# bulding kernel module for PWM input and output
	@echo $(EXTRA_CFLAGS)
	$(MAKE) -C $(KDIR) M=$(PWD) EXTRA_CFLAGS="$(EXTRA_CFLAGS)"

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

overlay:
	# make the kernel overlay for the PWM input pins
	cpp -nostdinc -undef -x assembler-with-cpp -I ${KDIR}/include $(OVERLAY).dts $(OVERLAY).dts.cpp \
	&& dtc -I dts -O dtb -o $(OVERLAY).dtbo $(OVERLAY).dts.cpp

install_overlay: overlay
	# installing overlay to boot environment
	sudo cp $(OVERLAY).dtbo /boot/overlay-user

install_boot_env:
	# only copy if orangepiEnv.txt is diffferent than the existing one
	@if ! cmp -s orangepiEnv.txt /boot/orangepiEnv.txt; then \
		echo "orangepiEnv.txt has changed, copying to /boot..."; \
		echo "Installing boot environment variables for Orange Pi..."; \
		sudo cp orangepiEnv.txt /boot/; \
		echo "Reboot to apply changes."; \
	else \
		echo "No changes in orangepiEnv.txt, skipping copy."; \
	fi

# following rules not used, instead permissions service that is more robust
# install_rules:
# 	# Copy the udev rules file to the appropriate directory
# 	sudo cp 99-argo-radio-servo.rules /etc/udev/rules.d/
# 	# Reload udev rules
# 	sudo udevadm control --reload-rules
# 	# Trigger udev to apply the new rules
# 	sudo udevadm trigger --subsystem-match=kobject # Target kobject subsystem specifically

install_permissions_service:
	# Copy the postinit script
	sudo cp $(PWD)/argo-radio-servo-postinit.sh /usr/local/bin
	# Copy the systemd service file
	sudo cp $(PWD)/argo-radio-servo-perms.service /etc/systemd/system
	# Reload systemd manager configuration to recognize the new service
	sudo systemctl daemon-reload
	# Enable the service to start on boot
	sudo systemctl enable argo-radio-servo-perms.service

install_module: module
	@echo "--- Installing kernel module for boot-time loading ---"
	@echo "Creating target directory: $(MDIR)"
	sudo mkdir -p "$(MDIR)"
	@echo "Copying module to $(MDIR)..."
	sudo cp "$(PWD)/$(MOD).ko" "$(MDIR)"
	@echo "Updating module dependencies..."
	sudo depmod -a
	@echo "Configuring module to load on boot..."
	echo "$(MOD)" | sudo tee /etc/modules-load.d/argo-radio-servo.conf > /dev/null
	@echo "--- Installation Complete ---"
	@echo "Module has been installed. You can now manage it with modprobe."
	@echo "To test, run: 'sudo modprobe $(MOD)'"
	@echo "To check status, run: 'lsmod | grep $(MOD)'"
	@echo "The module is configured to load automatically on next boot."

# The post_insmod_setup target is now only for manual testing, not for boot-time persistence.
# Udev rules handle boot-time permissions.
post_insmod_setup:
	@echo "Waiting for sysfs files to appear for manual setup..."
	sleep 0.5
	@echo "Setting sysfs file permissions for group dialout (manual run only)..."
	sudo chgrp dialout $(SYS)/radio_rudder_pw_us || true
	sudo chgrp dialout $(SYS)/radio_sail_pw_us || true
	sudo chgrp dialout $(SYS)/servo_rudder_pw_us || true
	sudo chgrp dialout $(SYS)/servo_sail_pw_us || true
	sudo chmod g+w $(SYS)/servo_rudder_pw_us || true
	sudo chmod g+w $(SYS)/servo_sail_pw_us || true
	@echo "Sysfs file group ownership and permissions updated (manual run only). Udev handles boot-time."
	ls -l $(SYS)

blacklist_ethernet:
	echo "blacklist sunxi_gmac" | sudo tee /etc/modprobe.d/blacklist-sunxi-gmac.conf > /dev/null
	@echo "Blacklisted sunxi-gmac module to prevent ethernet conflict with PWM pins."

insmod: module
	-sudo rmmod $(MOD) 2>/dev/null && echo "Removed module from running kernel" # Attempt to remove, ignore errors if not loaded
	@echo "Unexporting PWM2 and PWM4 to ensure they are available for the module..."
	-echo 2 | sudo tee /sys/class/pwm/pwmchip0/unexport > /dev/null
	-echo 4 | sudo tee /sys/class/pwm/pwmchip0/unexport > /dev/null
	sudo insmod $(MOD).ko
	# Call here to post_insmod_setup here is for testing. Udev will handle permissions normally
	dmesg | tail -n 20

rmmod:
	rmmod $(MOD)

watch:
	watch -n .2 "cat $(SYS)/radio_sail_pw_us;cat $(SYS)/radio_rudder_pw_us"

# New target to test PWM functions (inputs and outputs)
test_pwm:
	@echo "Starting PWM test script. Press Ctrl+C to stop."
	./test_pwm_loop.sh
