#!/bin/bash
# BNO085 Driver Launcher Script
# This script launches both the C++ driver and Python bridge for the BNO085 IMU system

set -e  # Exit on any error

# Determine Argo repository directory dynamically
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
ARGO_DIR="$(dirname "$SCRIPT_DIR")"

# Change to the correct directory
cd "$ARGO_DIR/nodes"

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source the custom workspace
source "$ARGO_DIR/nodes/argo_bno08x_driver_workspace/install/setup.bash"

# Export environment variables for child processes
export ROS_DOMAIN_ID=0
export AMENT_PREFIX_PATH
export CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH
export PATH
export PYTHONPATH
export ROS_DISTRO
export ROS_LOCALHOST_ONLY
export ROS_PYTHON_VERSION
export ROS_VERSION

# Start the C++ driver in background
echo "Starting BNO08x C++ driver..."
ros2 run bno08x_driver bno08x_driver --ros-args --params-file "$ARGO_DIR/nodes/vendor/bno085_i2c_5hz.yaml" &
DRIVER_PID=$!

# Wait a moment for driver to initialize
sleep 3

# Start the Python bridge
echo "Starting BNO085 Python bridge..."
python3 "$ARGO_DIR/nodes/bno085.py" bridge &
BRIDGE_PID=$!

# Function to cleanup on exit
cleanup() {
    echo "Shutting down BNO085 system..."
    kill $DRIVER_PID 2>/dev/null || true
    kill $BRIDGE_PID 2>/dev/null || true
    wait $DRIVER_PID 2>/dev/null || true
    wait $BRIDGE_PID 2>/dev/null || true
    exit 0
}

# Set up signal handlers
trap cleanup SIGTERM SIGINT

# Wait for both processes
wait $DRIVER_PID $BRIDGE_PID
