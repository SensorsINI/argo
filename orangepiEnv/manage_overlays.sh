#!/bin/bash
# orangepiEnv/manage_overlays.sh

# This script manages the user_overlays entry in orangepiEnv.txt to avoid conflicts
# between different Argo components that need to add device tree overlays.
#
# It is called by the Makefiles of various hardware-interfacing components
# to centralize the modification of the boot configuration.
#
# Current Argo components using this script:
# - nodes/pwm_capture_module/Makefile
# - power_control/sys_heartbeat/Makefile
#
# Usage:
# sudo bash manage_overlays.sh add <overlay_name>
# sudo bash manage_overlays.sh remove <overlay_name>
#
# It ensures that orangepiEnv.txt exists and that the user_overlays line is
# correctly formatted. Overlays are space-separated.

set -e

ACTION=$1
OVERLAY=$2
ENV_FILE_DIR="/boot"
ENV_FILE_NAME="orangepiEnv.txt"
ENV_FILE_PATH="$ENV_FILE_DIR/$ENV_FILE_NAME"
OVERLAY_VAR="user_overlays" # Target the 'user_overlays' variable for custom Argo overlays

# --- For local testing when /boot is not available ---
# If we're not running as root, assume local test and use a local file
if [ "$(id -u)" != "0" ]; then
    ENV_FILE_DIR="."
    ENV_FILE_PATH="./$ENV_FILE_NAME"
    echo "Running in local/test mode. Using $ENV_FILE_PATH"
fi
# --- End local testing ---

if [ -z "$ACTION" ] || [ -z "$OVERLAY" ]; then
    echo "Usage: $0 <add|remove> <overlay_name>"
    exit 1
fi

# 1. Ensure the orangepiEnv.txt file exists
if [ ! -f "$ENV_FILE_PATH" ]; then
    echo "Creating $ENV_FILE_PATH..."
    touch "$ENV_FILE_PATH"
fi

# 2. Read existing overlays
current_overlays=""
if grep -q "^${OVERLAY_VAR}=" "$ENV_FILE_PATH"; then
    current_overlays=$(grep "^${OVERLAY_VAR}=" "$ENV_FILE_PATH" | sed "s/^${OVERLAY_VAR}=//")
fi

# Convert to an array for easier manipulation
IFS=' ' read -r -a overlays_array <<< "$current_overlays"

# 3. Perform action
if [ "$ACTION" == "add" ]; then
    # Add the overlay if it's not already present
    if [[ ! " ${overlays_array[@]} " =~ " ${OVERLAY} " ]]; then
        overlays_array+=("$OVERLAY")
        echo "Added overlay: $OVERLAY"
    else
        echo "Overlay '$OVERLAY' already present."
    fi
elif [ "$ACTION" == "remove" ]; then
    # Remove the overlay if it exists
    new_overlays=()
    removed=false
    for item in "${overlays_array[@]}"; do
        if [ "$item" != "$OVERLAY" ]; then
            new_overlays+=("$item")
        else
            removed=true
        fi
    done
    overlays_array=("${new_overlays[@]}")
    if [ "$removed" = true ]; then
        echo "Removed overlay: $OVERLAY"
    else
        echo "Overlay '$OVERLAY' not found."
    fi
else
    echo "Invalid action: $ACTION. Use 'add' or 'remove'."
    exit 1
fi

# 4. Write the updated overlays back to the file
# Create the new line content
new_line_content=$(IFS=" "; echo "${overlays_array[*]}")

# Create a temporary file
tmp_file=$(mktemp)

# Remove the old overlays line if it exists
if grep -q "^${OVERLAY_VAR}=" "$ENV_FILE_PATH"; then
    grep -v "^${OVERLAY_VAR}=" "$ENV_FILE_PATH" > "$tmp_file"
else
    cat "$ENV_FILE_PATH" > "$tmp_file"
fi

# Add the new overlays line, but only if it's not empty
if [ -n "$new_line_content" ]; then
    echo "${OVERLAY_VAR}=${new_line_content}" >> "$tmp_file"
    echo "Updated ${OVERLAY_VAR}: ${new_line_content}"
else
    echo "No overlays remaining in ${OVERLAY_VAR}."
fi

# Replace the original file
# Need to use sudo if we are modifying /boot
if [ "$(id -u)" == "0" ]; then
    mv "$tmp_file" "$ENV_FILE_PATH"
else
    # In local mode, just move it. Sudo might not be available or needed.
    # The calling Makefile will use sudo for the whole script if needed.
    mv "$tmp_file" "$ENV_FILE_PATH"
fi

echo "Successfully updated $ENV_FILE_PATH."
