#!/bin/bash
# Diagnose topic schema conflicts for Foxglove
# This script helps identify schema mismatches that cause Foxglove errors

echo "=== Topic Schema Diagnostic Tool ==="
echo ""

# Check if ROS2 is running
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ROS2 not found in PATH"
    exit 1
fi

# Topic to check
TOPIC="/rudder_sail_radio"

echo "Checking topic: $TOPIC"
echo ""

# Check if topic exists
if ! ros2 topic list | grep -q "^${TOPIC}$"; then
    echo "❌ Topic $TOPIC does not exist"
    echo ""
    echo "Topics that might be similar:"
    ros2 topic list | grep -i "rudder\|sail\|radio" || echo "  (none found)"
    exit 1
fi

echo "✅ Topic exists"
echo ""

# Get topic info with verbose output
echo "=== Topic Information ==="
ros2 topic info $TOPIC --verbose
echo ""

# Get topic type
TOPIC_TYPE=$(ros2 topic type $TOPIC)
echo "=== Topic Type ==="
echo "Type: $TOPIC_TYPE"
echo ""

# Check for multiple types (schema conflict)
TYPE_COUNT=$(ros2 topic info $TOPIC | grep -c "Type:")
if [ "$TYPE_COUNT" -gt 1 ]; then
    echo "⚠️  SCHEMA CONFLICT DETECTED!"
    echo "   Multiple message types found for the same topic"
    echo ""
    echo "Conflicting types:"
    ros2 topic info $TOPIC --verbose | grep "Topic type:" | sed 's/^/   - /'
    echo ""
    echo "This will cause Foxglove to show schema errors."
    echo ""
    
    # Show publishers
    echo "=== Publishers (causing conflict) ==="
    ros2 topic info $TOPIC --verbose | grep -A 10 "Publisher" | head -20
    echo ""
    
    # Show subscribers
    echo "=== Subscribers ==="
    ros2 topic info $TOPIC --verbose | grep -A 10 "Subscription" | head -20
    echo ""
    
    echo "=== RECOMMENDED FIX ==="
    echo "1. Ensure only ONE publisher uses the correct message type:"
    echo "   geometry_msgs/msg/Vector3"
    echo ""
    echo "2. For /rudder_sail_radio, expected type is:"
    echo "   geometry_msgs/msg/Vector3"
    echo ""
    echo "3. If foxglove_bridge is publishing, it may need configuration"
    echo "   or the topic name might be wrong in Foxglove panels"
    echo ""
else
    echo "✅ No schema conflict - single message type"
    echo ""
    if [ "$TOPIC_TYPE" = "geometry_msgs/msg/Vector3" ]; then
        echo "✅ Correct message type for /rudder_sail_radio"
    else
        echo "⚠️  Unexpected message type: $TOPIC_TYPE"
        echo "   Expected: geometry_msgs/msg/Vector3"
    fi
fi

echo ""
echo "=== All ROS2 Nodes ==="
ros2 node list
echo ""

echo "=== Checking foxglove_bridge ==="
if ros2 node list | grep -q "foxglove_bridge"; then
    echo "foxglove_bridge is running"
    echo ""
    echo "foxglove_bridge topics:"
    ros2 node info /foxglove_bridge 2>/dev/null | grep -E "(Publisher|Subscription)" | head -10
else
    echo "foxglove_bridge is not running"
fi

