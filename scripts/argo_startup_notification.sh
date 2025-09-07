#!/bin/bash
# Argo Startup Notification Script
# Sends GUI notification when Argo system starts

SCRIPT_DIR="/home/orangepi/argo/scripts"
NOTIFICATION_SCRIPT="$SCRIPT_DIR/storage_notifications.py"

# Wait a bit for the system to fully boot and display to be ready
sleep 10

# Check if we're in a GUI environment
if [[ -n "$DISPLAY" ]] || [[ -n "$WAYLAND_DISPLAY" ]]; then
    # Send startup notification
    python3 "$NOTIFICATION_SCRIPT" startup
    
    # Also check for any immediate storage warnings
    python3 "$NOTIFICATION_SCRIPT" check
else
    echo "No GUI environment detected, skipping startup notification"
fi
