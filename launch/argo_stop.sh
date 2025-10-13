#!/bin/bash
# Argo Stop Script - Stop Argo nodes via systemd service

echo "🛑 Stopping Argo ROS2 nodes via systemd service..."

# Try to stop the service
if systemctl --user stop argo_launch.service 2>/dev/null; then
    echo "✅ Argo launch service stopped successfully (user service)"
elif sudo systemctl stop argo_launch.service 2>/dev/null; then
    echo "✅ Argo launch service stopped successfully (system service)"
else
    echo "⚠️  Failed to stop argo_launch service via systemctl"
    echo "📋 Trying direct lifecycle manager approach..."
    
    # Fallback to direct lifecycle manager
    cd "$(dirname "$0")/.."
    source /opt/ros/humble/setup.bash
    python3 launch/argo_lifecycle_manager.py stop
    exit $?
fi

# Verify the service is stopped
if systemctl --user is-active --quiet argo_launch.service 2>/dev/null || systemctl is-active --quiet argo_launch.service 2>/dev/null; then
    echo "⚠️  Service still appears to be running"
    echo "📋 Checking service status..."
    if systemctl --user status argo_launch.service --no-pager -l 2>/dev/null; then
        systemctl --user status argo_launch.service --no-pager -l
    else
        systemctl status argo_launch.service --no-pager -l
    fi
else
    echo "✅ Argo launch service confirmed stopped"
fi
