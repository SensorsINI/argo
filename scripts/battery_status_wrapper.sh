#!/bin/bash
# Battery Status Wrapper for XFCE4 Panel
# Sources ROS2 environment and runs simple battery status display

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Run the simple battery status script
exec python3 "$SCRIPT_DIR/battery_status_simple.py"

