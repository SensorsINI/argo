#!/bin/bash
# Quick diagnostic script to check if foxglove_bridge is running and listening on port 8765

echo "🔍 Checking Foxglove Bridge Status..."
echo ""

# Check if foxglove_bridge process is running
if pgrep -f foxglove_bridge > /dev/null; then
    echo "✅ foxglove_bridge process is running"
    ps aux | grep -E 'foxglove_bridge' | grep -v grep | head -1
else
    echo "❌ foxglove_bridge process is NOT running"
fi

echo ""

# Check if port 8765 is listening
if command -v netstat > /dev/null; then
    PORT_CHECK=$(netstat -tlnp 2>/dev/null | grep ':8765 ' || true)
elif command -v ss > /dev/null; then
    PORT_CHECK=$(ss -tlnp 2>/dev/null | grep ':8765 ' || true)
else
    PORT_CHECK=""
fi

if [ -n "$PORT_CHECK" ]; then
    echo "✅ Port 8765 is listening:"
    echo "$PORT_CHECK"
else
    echo "❌ Port 8765 is NOT listening"
    echo "   foxglove_bridge may not have started successfully"
fi

echo ""
echo "📋 ROS2 Node Check:"
bash -c 'source /opt/ros/humble/setup.bash 2>/dev/null && ros2 node list 2>/dev/null | grep -i foxglove || echo "   No foxglove nodes found in ROS2"' || echo "   Could not check ROS2 nodes (ROS2 environment not available)"

echo ""
echo "💡 To start foxglove_bridge manually:"
echo "   ros2 run foxglove_bridge foxglove_bridge"
echo ""
echo "💡 To check if it's installed:"
echo "   ros2 pkg executables foxglove_bridge"
