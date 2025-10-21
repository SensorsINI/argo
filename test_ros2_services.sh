#!/bin/bash
# Test script for ROS2 service integration

set -e

echo "=========================================="
echo "ROS2 Service Integration Test"
echo "=========================================="
echo ""

# Source ROS2 environment
source /opt/ros/humble/setup.bash

echo "1. Testing if ROS2 services are available..."
echo ""

# Check for lifecycle services
echo "Checking lifecycle services:"
ros2 service list | grep "/argo/lifecycle/" || echo "  ⚠️  No lifecycle services found (lifecycle manager not running)"
echo ""

# Check for power control services
echo "Checking power control services:"
ros2 service list | grep "/argo/power/" || echo "  ⚠️  No power control services found (power control not running)"
echo ""

# Check for topics
echo "2. Testing if ROS2 topics are available..."
echo ""

echo "Checking lifecycle topics:"
ros2 topic list | grep "/argo/lifecycle/" || echo "  ⚠️  No lifecycle topics found"
echo ""

echo "Checking power control topics:"
ros2 topic list | grep "/argo/power/" || echo "  ⚠️  No power control topics found"
echo ""

# Test service calls (only if services exist)
echo "3. Testing service calls..."
echo ""

if ros2 service list | grep -q "/argo/lifecycle/status"; then
    echo "Testing lifecycle status service:"
    ros2 service call /argo/lifecycle/status std_srvs/srv/Trigger --once || echo "  ⚠️  Service call failed"
    echo ""
else
    echo "  ℹ️  Lifecycle manager not running - start with:"
    echo "     python3 launch/argo_lifecycle_manager.py run"
    echo ""
fi

if ros2 service list | grep -q "/argo/power/toggle_recording"; then
    echo "  ℹ️  Power control services available"
    echo "  ⚠️  Skipping power control tests (require physical hardware)"
    echo ""
else
    echo "  ℹ️  Power control not running - start with:"
    echo "     sudo bash -c 'source /opt/ros/humble/setup.bash && python3 power_control/argo_power_control.py'"
    echo ""
fi

echo "=========================================="
echo "Test Summary"
echo "=========================================="
echo ""
echo "To fully test the services:"
echo "1. Start lifecycle manager: python3 launch/argo_lifecycle_manager.py run"
echo "2. Start power control: sudo bash -c 'source /opt/ros/humble/setup.bash && python3 power_control/argo_power_control.py'"
echo "3. Run this test script again"
echo ""
echo "Available service commands:"
echo "  ros2 service call /argo/lifecycle/status std_srvs/srv/Trigger"
echo "  ros2 service call /argo/lifecycle/start std_srvs/srv/Trigger"
echo "  ros2 service call /argo/lifecycle/stop std_srvs/srv/Trigger"
echo "  ros2 service call /argo/power/toggle_recording std_srvs/srv/Trigger"
echo ""
echo "Available topic commands:"
echo "  ros2 topic echo /argo/lifecycle/status"
echo "  ros2 topic echo /argo/power/status"
echo "  ros2 topic echo /argo/power/button_events"
echo ""

