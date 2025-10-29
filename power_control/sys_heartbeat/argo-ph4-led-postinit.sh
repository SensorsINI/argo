#!/bin/bash
# ~/argo/power_control/sys_heartbeat/argo-ph4-led-postinit.sh

# This script is run by a systemd service at boot to set the correct
# permissions on the sysfs files for the kernel-managed PH4 Green LED.
# This allows the argo_power_control.py script, running as a non-root user,
# to take control of the LED.

set -e

LED_SYSFS_PATH="/sys/class/leds/argo:green:heartbeat"
TARGET_GROUP="orangepi"

# Wait for the sysfs entry to be created by the kernel
# It can take a few moments after the overlay is loaded.
WAIT_SECONDS=10
for (( i=0; i<WAIT_SECONDS; i++ )); do
    if [ -d "$LED_SYSFS_PATH" ]; then
        echo "Found LED sysfs directory: $LED_SYSFS_PATH"
        break
    fi
    sleep 1
done

if [ ! -d "$LED_SYSFS_PATH" ]; then
    echo "Error: LED sysfs directory not found after $WAIT_SECONDS seconds." >&2
    exit 1
fi

# Set group ownership on the brightness and trigger files
chgrp "$TARGET_GROUP" "$LED_SYSFS_PATH/brightness"
chgrp "$TARGET_GROUP" "$LED_SYSFS_PATH/trigger"
echo "Set group ownership to '$TARGET_GROUP' for LED control files."

# Set group write permissions
chmod g+w "$LED_SYSFS_PATH/brightness"
chmod g+w "$LED_SYSFS_PATH/trigger"
echo "Set group write permissions for LED control files."

echo "Permissions set for argo:green:heartbeat LED."
ls -l "$LED_SYSFS_PATH"



