#!/bin/bash
# Launch simulation with output logging to file
# Usage: ./scripts/launch_simulation.sh [local|remote]

# Set default mode
MODE=${1:-local}

# Create logs directory if it doesn't exist
LOG_DIR="$HOME/argo/logs"
mkdir -p "$LOG_DIR"

# Generate log filename with timestamp
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/simulation_${MODE}_${TIMESTAMP}.log"

# Change to Argo directory
cd "$HOME/argo" || exit 1

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Check for foxglove-bridge before starting simulation
echo "🔍 Checking prerequisites for simulation..."
if ! dpkg -l | grep -q "ros-humble-foxglove-bridge"; then
    echo ""
    echo "❌ ERROR: foxglove-bridge is not installed!"
    echo ""
    echo "   The simulation requires foxglove-bridge for visualization in Foxglove Studio."
    echo ""
    echo "   To install it, run:"
    echo "     make install-foxglove-bridge"
    echo ""
    echo "   Or install all dependencies:"
    echo "     make install-deps"
    echo ""
    exit 1
fi

# Verify foxglove_bridge can be run
if ! ros2 pkg list | grep -q "^foxglove_bridge$"; then
    echo ""
    echo "❌ ERROR: foxglove_bridge package not found!"
    echo ""
    echo "   The package may be installed but not properly sourced."
    echo "   Try running: source /opt/ros/humble/setup.bash"
    echo "   Then verify with: ros2 pkg list | grep foxglove"
    echo ""
    exit 1
fi

echo "✅ Prerequisites check passed"
echo ""

echo "🚢 Starting Argo simulation (${MODE} mode)..."
echo "📝 Logging output to: $LOG_FILE"
echo "💡 To view log: tail -f $LOG_FILE"
echo "💡 To grep log: grep 'pattern' $LOG_FILE"
echo ""

# Trap Ctrl+C to show log location
# trap 'echo ""; echo "📝 Full log saved to: $LOG_FILE"; echo "💡 Grep with: grep \"pattern\" $LOG_FILE"' INT TERM

# Launch simulation and log everything
# Use lifecycle manager which includes proper validation and error handling
if [ "$MODE" = "local" ]; then
    python3 launch/argo_lifecycle_manager.py simulate_local 2>&1 | tee "$LOG_FILE"
else
    python3 launch/argo_lifecycle_manager.py simulate_remote 2>&1 | tee "$LOG_FILE"
fi

# Capture the exit code of the simulation command (before the pipe to tee)
# PIPESTATUS is an array variable where PIPESTATUS[0] = exit status of the first command in a pipeline
EXIT_CODE=${PIPESTATUS[0]}

# Show log location on exit
echo ""
echo "📝 Full log saved to: $LOG_FILE"
echo "💡 Grep with: grep \"pattern\" $LOG_FILE"

exit $EXIT_CODE
