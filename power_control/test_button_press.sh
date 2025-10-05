#!/bin/bash
# Test script for simulating button presses via CLI

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Parse command line arguments
if [ "$1" = "double" ] || [ "$1" = "double-tap" ]; then
    echo "🔵 Simulating DOUBLE TAP - Toggle Argo service"
    bash -c "source /opt/ros/humble/setup.bash && python3 '$SCRIPT_DIR/argo_power_control.py' --simulate-double-tap"
elif [ "$1" = "triple" ] || [ "$1" = "triple-tap" ]; then
    echo "🟣 Simulating TRIPLE TAP - Toggle recording"
    bash -c "source /opt/ros/humble/setup.bash && python3 '$SCRIPT_DIR/argo_power_control.py' --simulate-triple-tap"
else
    echo "Usage: $0 {double|triple}"
    echo ""
    echo "Commands:"
    echo "  double  - Simulate double tap (toggle Argo launch service)"
    echo "  triple  - Simulate triple tap (toggle recording)"
    echo ""
    echo "Examples:"
    echo "  $0 double    # Toggle Argo service"
    echo "  $0 triple    # Toggle recording"
    exit 1
fi


