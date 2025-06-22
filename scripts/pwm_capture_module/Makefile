MOD=argo_radio_servo_module
OVERLAY=argo_radio_servo_overlay
SYS=/sys/kernel/argo_radio_servo
obj-m := $(MOD).o       
 
# Define GIT_HASH to be passed to the C code.
# This will be picked up by the kernel build system.
# The `|| echo "not-a-repo"` part handles cases where this is not a git repository.
GIT_HASH_CMD = $(shell git rev-parse --short HEAD 2>/dev/null || echo "not-a-repo")
GIT_COMMIT_DATE_CMD = $(shell git show -s --format=%ci HEAD 2>/dev/null || echo "unknown")
BUILD_DATE_CMD = $(shell date +%Y-%m-%dT%H:%M:%S%z)
EXTRA_CFLAGS += -DGIT_HASH='"$(GIT_HASH_CMD)"'
EXTRA_CFLAGS += -DGIT_COMMIT_DATE='"$(GIT_COMMIT_DATE_CMD)"'
EXTRA_CFLAGS += -DBUILD_DATE='"$(BUILD_DATE_CMD)"' 
KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

.PHONY: all clean overlay install_overlay insmod rmmod watch test_pwm install_module install_rules post_insmod_setup # Ensure all targets are listed here

# all target does everything needed to build the module and overlay
all: module overlay install_module install_overlay install_boot_env install_rules install_permissions_service 

module:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

overlay:
	cpp -nostdinc -undef -x assembler-with-cpp -I ${KDIR}/include $(OVERLAY).dts $(OVERLAY).dts.cpp \
	&& dtc -I dts -O dtb -o $(OVERLAY).dtbo $(OVERLAY).dts.cpp

install_overlay:
	cp $(OVERLAY).dtbo /boot/overlay-user

install_boot_env:
	# only copy if orangepiEnv.txt is diffferent than the existing one
	if ! cmp -s orangepiEnv.txt /boot/orangepiEnv.txt ; then
		echo "orangepiEnv.txt has changed, copying to /boot..."
		echo "Installing boot environment variables for Orange Pi..."
		# Copy the environment file to the boot directory
		# This file should contain the necessary boot parameters for the Orange Pi
		sudo cp orangepiEnv.txt /boot/
		echo "reboot to apply changes"
	else
		echo "No changes in orangepiEnv.txt, skipping copy."
	fi

install_rules:
	# Copy the udev rules file to the appropriate directory
	sudo cp 99-argo-radio-servo.rules /etc/udev/rules.d/
	# Reload udev rules
	sudo udevadm control --reload-rules
	# Trigger udev to apply the new rules
	sudo udevadm trigger --subsystem-match=kobject # Target kobject subsystem specifically

install_permissions_service:
	# Copy the postinit script
	sudo cp $(PWD)/argo-radio-servo-postinit.sh /usr/local/bin
	# Copy the systemd service file FIRST
	sudo cp $(PWD)/argo-radio-servo-perms.service /etc/systemd/system
	# Force systemd to re-scan for new unit files and clear its cache
	# This is a bit aggressive but helps with stubborn caching issues
	sudo systemctl daemon-reload --no-pager
	# Add a small delay to allow systemd to process
	sleep 3
	# Check if systemd now sees the unit file before attempting to enable
	@systemctl status argo-radio-servo-perms.service --no-pager 2>/dev/null || echo "DEBUG: systemd does not yet see the unit file. This is the problem."
	# Then enable the service
	sudo systemctl enable argo-radio-servo-perms.service

install_module: module
	@echo "--- Debugging install_module ---"
	@echo "Source .ko file expected at: $(PWD)/$(MOD).ko"
	@ls -l "$(PWD)/$(MOD).ko" || echo "ERROR: Source .ko file not found at $(PWD)/$(MOD).ko"
	@echo "Attempting to create target directory: $(KDIR)/drivers/misc/argo/"
	sudo mkdir -p "$(KDIR)/drivers/misc/argo/"
	@echo "Checking target directory after mkdir -p:"
	@ls -ld "$(KDIR)/drivers/misc/argo/" || echo "FATAL ERROR: Target directory still not found after mkdir -p. Permissions issue or fundamental path problem."
	@echo "Attempting to copy module..."
	sudo cp "$(PWD)/$(MOD).ko" "$(KDIR)/drivers/misc/argo/"
	@echo "Copy command executed. Checking target directory after copy:"
	@ls -l "$(KDIR)/drivers/misc/argo/$(MOD).ko" || echo "ERROR: Module not found in target after copy. Copy command likely failed."
	sudo depmod -a
	echo "$(MOD)" | sudo tee /etc/modules-load.d/argo-radio-servo.conf > /dev/null
	@echo "--- Debugging install_module complete ---"

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
	@echo "blacklist sunxi_gmac" | sudo tee /etc/modprobe.d/blacklist-sunxi-gmac.conf > /dev/null
	@echo "Blacklisted sunxi-gmac module to prevent ethernet conflict with PWM pins."

insmod: module
	-sudo rmmod $(MOD) # Attempt to remove, ignore errors if not loaded
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
