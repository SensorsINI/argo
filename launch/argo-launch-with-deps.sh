#!/bin/bash
# Argo Launch Script with Dependencies
# Ensures kernel module is loaded before starting ROS2 nodes

set -e

echo "🚢 Starting Argo Launch with Dependencies..."

# Check if kernel module is loaded
if ! lsmod | grep -q argo_radio_servo_module; then
    echo "📦 Loading argo_radio_servo_module kernel module..."
    cd /home/orangepi/argo/nodes/pwm_capture_module
    sudo insmod argo_radio_servo_module.ko
    echo "✅ Kernel module loaded"
fi

# Check if sysfs interface exists
if [ ! -d "/sys/kernel/argo_radio_servo" ]; then
    echo "❌ Error: /sys/kernel/argo_radio_servo not found"
    echo "   Kernel module may not be properly loaded"
    exit 1
fi

echo "✅ All dependencies ready, starting ROS2 launch..."

# Start the actual ROS2 launch
cd /home/orangepi/argo
source /opt/ros/humble/setup.bash
exec python3 launch/argo_lifecycle_manager.py continuous




