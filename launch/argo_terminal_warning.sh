#!/bin/bash
# Argo Terminal Warning Script
# Shows storage status when opening new terminals

# Only show warning if we're in the argo directory or if ROS2 is sourced
if [[ "$PWD" == *"argo"* ]] || [[ -n "$ROS_DISTRO" ]]; then
    # Determine repo directory from this script's location
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
    # Check if storage_monitor.py exists and is executable
    if [[ -f "$REPO_DIR/launch/storage_monitor.py" ]]; then
        echo "🚢 Argo Autonomous Sailboat System"
        python3 "$REPO_DIR/launch/storage_monitor.py" --check
        echo ""
    fi
fi
