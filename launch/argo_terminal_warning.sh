#!/bin/bash
# Argo Terminal Warning Script
# Shows storage status when opening new terminals

# Only show warning if we're in the argo directory or if ROS2 is sourced
if [[ "$PWD" == *"argo"* ]] || [[ -n "$ROS_DISTRO" ]]; then
    # Check if storage_monitor.py exists and is executable
    if [[ -f "/home/orangepi/argo/launch/storage_monitor.py" ]]; then
        echo "🚢 Argo Autonomous Sailboat System"
        python3 /home/orangepi/argo/launch/storage_monitor.py --check
        echo ""
    fi
fi
