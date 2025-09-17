#!/bin/bash
# Argo Desktop Status Launcher
# Starts the desktop status monitor with proper environment

# Change to argo directory
cd "$(dirname "$0")/.."

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Start the desktop status monitor
python3 launch/argo_desktop_status.py
