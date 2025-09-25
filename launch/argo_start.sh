#!/bin/bash
# Argo Start Script - Start Argo nodes via systemd service for robustness

echo "🚀 Starting Argo ROS2 nodes via systemd service..."

# Check if we can run systemctl without sudo
if systemctl --user is-active --quiet argo-launch.service 2>/dev/null; then
    echo "⚠️  Argo launch service is already running"
    systemctl --user status argo-launch.service --no-pager -l
    exit 0
fi

# Try to start the service
if systemctl --user start argo-launch.service 2>/dev/null; then
    echo "✅ Argo launch service started successfully"
elif sudo systemctl start argo-launch.service 2>/dev/null; then
    echo "✅ Argo launch service started successfully (with sudo)"
else
    echo "❌ Failed to start argo-launch service"
    echo "📋 Trying direct lifecycle manager approach..."
    
    # Fallback to direct lifecycle manager
    cd "$(dirname "$0")/.."
    source /opt/ros/humble/setup.bash
    python3 launch/argo_lifecycle_manager.py run
    exit $?
fi

echo "📊 Monitoring service status..."

# Wait for service to start and nodes to initialize (allow time for failures)
echo "⏳ Waiting for nodes to initialize..."
sleep 10

# Check if service is running
if systemctl --user is-active --quiet argo-launch.service 2>/dev/null || systemctl is-active --quiet argo-launch.service 2>/dev/null; then
    echo "✅ Argo launch service is active"
    echo "🔍 Checking node status..."
    
    # Use the lifecycle manager to check node status
    cd "$(dirname "$0")/.."
    source /opt/ros/humble/setup.bash
    python3 launch/argo_lifecycle_manager.py status
else
    echo "❌ Argo launch service failed to start"
    echo "📋 Checking service logs..."
    if systemctl --user status argo-launch.service --no-pager -l 2>/dev/null; then
        systemctl --user status argo-launch.service --no-pager -l
    else
        sudo journalctl -u argo-launch.service --no-pager -n 20
    fi
    exit 1
fi

