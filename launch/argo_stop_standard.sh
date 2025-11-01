#!/bin/bash
# Argo Stop Script - Stop Argo nodes via standard ROS2 launch system

echo "🛑 Stopping Argo ROS2 nodes..."

# Determine service name (support both old and new)
SERVICE_NAME="argo_launch_standard.service"

# Try to stop the service
if sudo systemctl stop "$SERVICE_NAME" 2>/dev/null; then
    echo "✅ Argo launch service stopped successfully"
else
    echo "⚠️  Failed to stop $SERVICE_NAME via systemctl"
    echo "📋 Trying direct node termination..."
    
    # Fallback: kill ROS2 launch processes
    pkill -f "ros2 launch.*argo_launch.py" || true
    pkill -f "argo_health_monitor.py" || true
    
    # Also kill individual nodes
    ros2 node list 2>/dev/null | while read node; do
        if [[ "$node" == *"node"* ]]; then
            # Extract node name and try to kill process
            pkill -f "$node" || true
        fi
    done
    
    echo "✅ Terminated ROS2 launch processes"
fi

# Verify the service is stopped
if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
    echo "⚠️  Service still appears to be running"
    echo "📋 Checking service status..."
    sudo systemctl status "$SERVICE_NAME" --no-pager -l
else
    echo "✅ Argo launch service confirmed stopped"
fi

