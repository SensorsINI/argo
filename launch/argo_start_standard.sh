#!/bin/bash
# Argo Start Script - Start Argo nodes via standard ROS2 launch system

echo "🚀 Starting Argo ROS2 nodes via standard ROS2 launch..."

# Determine service name (support both old and new)
SERVICE_NAME="argo_launch_standard.service"

# Check if service is already running
if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
    echo "⚠️  Argo launch service is already running"
    systemctl status "$SERVICE_NAME" --no-pager -l
    exit 0
fi

# Try to start the service
if sudo systemctl start "$SERVICE_NAME" 2>/dev/null; then
    echo "✅ Argo launch service started successfully"
else
    echo "❌ Failed to start $SERVICE_NAME"
    echo "📋 Trying direct ROS2 launch approach..."
    
    # Fallback to direct ROS2 launch
    cd "$(dirname "$0")/.."
    source /opt/ros/humble/setup.bash
    ros2 launch launch/argo_launch.py &
    HEALTH_MONITOR_PID=$!
    python3 launch/argo_health_monitor.py &
    echo "✅ Launched via ros2 launch (PID: $HEALTH_MONITOR_PID)"
    exit 0
fi

echo "📊 Monitoring service status..."

# Wait for service to start and nodes to initialize
echo "⏳ Waiting 20s for nodes to initialize..."
sleep 20

# Check if service is running
if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
    echo "✅ Argo launch service is active"
    echo "🔍 Checking node status..."
    
    # Use the standard status reporter
    cd "$(dirname "$0")/.."
    source /opt/ros/humble/setup.bash
    python3 launch/argo_lifecycle_manager.py status
else
    echo "❌ Argo launch service failed to start"
    echo "📋 Checking service logs..."
    sudo journalctl -u "$SERVICE_NAME" --no-pager -n 20
    exit 1
fi
