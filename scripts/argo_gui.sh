#!/bin/bash
# Argo GUI Launcher Script
# Launches the Argo CLI GUI with proper environment setup
# Requires sudo privileges to control systemd services

# Check if running with sudo
if [ "$EUID" -ne 0 ]; then
    echo "Error: This GUI requires sudo privileges to control systemd services"
    echo "Please run with: sudo ./scripts/argo_gui.sh"
    exit 1
fi

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARGO_DIR="$(dirname "$SCRIPT_DIR")"

# Change to argo directory
cd "$ARGO_DIR"

# Source ROS2 environment
source /opt/ros/humble/setup.bash 2>/dev/null || {
    echo "Warning: Could not source ROS2 environment"
    echo "Make sure ROS2 is installed and sourced"
}

# Launch the GUI
python3 scripts/argo_gui.py "$@"
