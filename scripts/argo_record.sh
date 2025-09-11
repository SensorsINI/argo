#!/bin/bash

# Argo ROS2 Bag Recording Script
# Handles rosbag recording with start/end reporting

set -e  # Exit on any error

# Configuration
BAGFILES_DIR="/home/orangepi/bagfiles"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_NAME="argo_${TIMESTAMP}"
BAG_PATH="${BAGFILES_DIR}/${BAG_NAME}"

# Function to log messages with timestamp
log_message() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a /tmp/argo_record.log
}

# Function to handle cleanup on exit
cleanup() {
    log_message "🛑 Recording stopped. Final bag location: ${BAG_PATH}"
    log_message "📁 Bag directory contents:"
    if [ -d "${BAG_PATH}" ]; then
        ls -la "${BAG_PATH}" | tee -a /tmp/argo_record.log
        log_message "💾 Bag size: $(du -sh "${BAG_PATH}" 2>/dev/null | cut -f1 || echo 'Unknown')"
    else
        log_message "⚠️  Warning: Bag directory not found at ${BAG_PATH}"
    fi
    exit 0
}

# Set up signal handlers for graceful shutdown
trap cleanup SIGTERM SIGINT EXIT

# Ensure bagfiles directory exists
mkdir -p "${BAGFILES_DIR}"

# Log start information
log_message "🚀 Starting Argo ROS2 bag recording..."
log_message "📁 Working directory: ${BAGFILES_DIR}"
log_message "📦 Bag name: ${BAG_NAME}"
log_message "📍 Full path: ${BAG_PATH}"
log_message "🕐 Start time: $(date)"

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Start recording
log_message "🎬 Executing: ros2 bag record -a -o ${BAG_NAME}"
ros2 bag record -a -o "${BAG_NAME}"

# If we reach here, recording completed normally
log_message "✅ Recording completed successfully"

