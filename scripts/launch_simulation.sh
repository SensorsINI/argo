#!/bin/bash
#
# This script launches the Argo simulation environment using the ROS2 launch system.
# It ensures that all simulation nodes are started and managed correctly.
#

# Set script to exit immediately if a command exits with a non-zero status.
set -e

# --- Source ROS2 Environment ---
# This is critical for ensuring that 'ros2' commands are available.
source /opt/ros/humble/setup.bash

# --- Get Project Directory ---
# This finds the root of the 'argo' project directory.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
ARGO_DIR="$(dirname "$SCRIPT_DIR")"

# --- Launch the Simulation ---
# We use 'ros2 launch' to start the simulation_launch.py file.
# The launch file defines all the nodes needed for the simulation.
echo "🚢 Launching Argo Simulation Environment..."
ros2 launch "$ARGO_DIR/launch/simulation_launch.py"
