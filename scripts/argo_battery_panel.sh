#!/bin/bash
# Argo Battery Status Panel Launcher
# Launches the battery status panel script with proper ROS2 environment

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Launch the battery status panel
exec python3 "$SCRIPT_DIR/battery_status_panel.py" "$@"

