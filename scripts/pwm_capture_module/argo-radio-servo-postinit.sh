#!/bin/bash
# Script to set permissions and group after argo_radio_servo_module loads

SYS_PATH="/sys/kernel/argo_radio_servo"
TARGET_GROUP="dialout" # Or any other group you want

# Wait for the sysfs directory to appear
# This loop ensures the module has created the files
while [ ! -d "$SYS_PATH" ]; do
    sleep 0.1
done

# Set group and permissions for the files
# Using '|| true' to prevent script from failing if a file temporarily doesn't exist
chown root:$TARGET_GROUP "$SYS_PATH"/radio_rudder_pw_us || true
chown root:$TARGET_GROUP "$SYS_PATH"/radio_sail_pw_us || true
chown root:$TARGET_GROUP "$SYS_PATH"/servo_rudder_pw_us || true
chown root:$TARGET_GROUP "$SYS_PATH"/servo_sail_pw_us || true

chmod 0664 "$SYS_PATH"/radio_rudder_pw_us || true # rw-rw-r--
chmod 0664 "$SYS_PATH"/radio_sail_pw_us || true   # rw-rw-r--
chmod 0664 "$SYS_PATH"/servo_rudder_pw_us || true # rw-rw-r--
chmod 0664 "$SYS_PATH"/servo_sail_pw_us || true   # rw-rw-r--

echo "Argo Radio Servo: Permissions and group set by post-init script." | tee /dev/kmsg /dev/console