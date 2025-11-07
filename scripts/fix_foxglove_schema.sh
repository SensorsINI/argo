#!/bin/bash
# Fix Foxglove schema conflicts by identifying and stopping conflicting publishers

echo "=== Foxglove Schema Conflict Fixer ==="
echo ""

TOPIC="/rudder_sail_radio"
EXPECTED_TYPE="geometry_msgs/msg/Vector3"

echo "Target topic: $TOPIC"
echo "Expected type: $EXPECTED_TYPE"
echo ""

# Check current state
echo "=== Current Topic State ==="
if ros2 topic list | grep -q "^${TOPIC}$"; then
    CURRENT_TYPE=$(ros2 topic type $TOPIC)
    echo "Current type: $CURRENT_TYPE"
    
    # Get all publishers
    PUBLISHERS=$(ros2 topic info $TOPIC --verbose 2>&1 | grep "Node name:" | grep -v Subscription | awk '{print $3}')
    
    if [ -z "$PUBLISHERS" ]; then
        echo "No publishers found (topic exists but no one is publishing)"
        echo ""
        echo "This is normal - the topic will be created when you publish from Foxglove"
        exit 0
    fi
    
    echo ""
    echo "Found publishers:"
    for pub in $PUBLISHERS; do
        PUB_TYPE=$(ros2 topic info $TOPIC --verbose 2>&1 | grep -A 5 "Node name: $pub" | grep "Topic type:" | head -1 | awk '{print $3 " " $4 " " $5}')
        echo "  - $pub (type: $PUB_TYPE)"
        
        if [ "$PUB_TYPE" != "$EXPECTED_TYPE" ]; then
            echo "    ⚠️  WRONG TYPE - This is causing the conflict!"
        fi
    done
    
else
    echo "Topic does not exist yet"
    echo "This is fine - it will be created when you publish from Foxglove"
    exit 0
fi

echo ""
echo "=== Solution ==="
echo "If foxglove_bridge is publishing the wrong type:"
echo "1. Check your Foxglove panels configuration"
echo "2. Ensure Joystick panel publishes to /joy (not /rudder_sail_radio)"
echo "3. Ensure Teleop panel (if used) uses Vector3 type"
echo ""
echo "The simulator bridge should only SUBSCRIBE to /rudder_sail_radio"
echo "It should never PUBLISH to it."

