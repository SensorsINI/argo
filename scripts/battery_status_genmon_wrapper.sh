#!/bin/bash
# Battery Status Genmon Wrapper for XFCE4 Generic Monitor Plugin
# Sources ROS2 environment and runs battery status genmon script

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Run the battery status genmon script
exec python3 "$SCRIPT_DIR/battery_status_genmon.py"
