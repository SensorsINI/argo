#!/bin/bash
# Argo Restart Script - Restart Argo nodes via systemd service

echo "🔄 Restarting Argo ROS2 nodes via systemd service..."
sudo systemctl restart argo_launch.service

if [ $? -eq 0 ]; then
    echo "✅ Argo launch service restarted successfully"
    echo "📊 Monitoring service status..."
    
    # Wait for service to start and nodes to initialize
    echo "⏳ Waiting for nodes to initialize..."
    sleep 5
    
    # Check if service is running
    if systemctl is-active --quiet argo_launch.service; then
        echo "✅ Argo launch service is active"
        echo "🔍 Checking node status..."
        
        # Use the lifecycle manager to check node status
        cd "$(dirname "$0")/.."
        source /opt/ros/humble/setup.bash
        python3 launch/argo_lifecycle_manager.py status
    else
        echo "❌ Argo launch service failed to restart"
        echo "📋 Checking service logs..."
        sudo journalctl -u argo_launch.service --no-pager -n 20
        exit 1
    fi
else
    echo "❌ Failed to restart argo_launch service"
    exit 1
fi

