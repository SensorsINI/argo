#!/bin/bash
# BNO085 Driver Launcher Script
# This script launches both the C++ driver and Python bridge for the BNO085 IMU system
#
# If the C++ driver crashes while the bridge stays up, we must exit the launcher so
# systemd sees the service as failed and Restart=always can restart the whole unit.
# (Previously: "wait $DRIVER_PID $BRIDGE_PID" waited for BOTH; driver death left bash
#  stuck forever with no automatic recovery.)

# Do not use set -u: ROS setup.bash references vars (e.g. AMENT_TRACE_SETUP_FILES) before they exist.
set -eo pipefail

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

BNO085_PARAMS="$ARGO_DIR/nodes/vendor/bno085_i2c_argo.yaml"

# Start the C++ driver in background
echo "Starting BNO08x C++ driver..."
ros2 run bno08x_driver bno08x_driver --ros-args --params-file "$BNO085_PARAMS" &
DRIVER_PID=$!

# Wait a moment for driver to initialize
sleep 3

if ! kill -0 "$DRIVER_PID" 2>/dev/null; then
    echo "BNO08x driver exited during startup wait; exiting for systemd restart."
    exit 1
fi

# Start the Python bridge
echo "Starting BNO085 Python bridge..."
python3 "$ARGO_DIR/nodes/bno085.py" bridge &
BRIDGE_PID=$!

# Function to cleanup on exit
cleanup() {
    echo "Shutting down BNO085 system..."
    kill "$DRIVER_PID" 2>/dev/null || true
    kill "$BRIDGE_PID" 2>/dev/null || true
    wait "$DRIVER_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" 2>/dev/null || true
}

# Set up signal handlers — intentional stop: exit 0 so we don't force-restart loop
trap 'cleanup; exit 0' SIGTERM SIGINT

# When either child exits, kill the other and exit non-zero so systemd restarts (Restart=always).
# wait -n: wait for any one job (bash 4.3+).
echo "Monitoring driver (pid $DRIVER_PID) and bridge (pid $BRIDGE_PID)..."
set +e
wait -n "$DRIVER_PID" "$BRIDGE_PID"
wait_status=$?
set -e
echo "BNO085 subprocess exited (wait status $wait_status); tearing down for systemd restart."
cleanup
exit 1
