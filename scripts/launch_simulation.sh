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

echo "🚢 Starting Argo simulation (${MODE} mode)..."
echo "📝 Logging output to: $LOG_FILE"
echo "💡 To view log: tail -f $LOG_FILE"
echo "💡 To grep log: grep 'pattern' $LOG_FILE"
echo ""

# Trap Ctrl+C to show log location
# trap 'echo ""; echo "📝 Full log saved to: $LOG_FILE"; echo "💡 Grep with: grep \"pattern\" $LOG_FILE"' INT TERM

# Launch simulation and log everything
if [ "$MODE" = "local" ]; then
    ros2 launch launch/argo_launch.py mode:=simulation 2>&1 | tee "$LOG_FILE"
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
