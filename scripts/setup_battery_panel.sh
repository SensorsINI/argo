#!/bin/bash
# Setup script for Argo Battery Status Panel in XFCE4
# This script helps configure the battery status display in the XFCE4 panel

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "🔋 Setting up Argo Battery Status Panel for XFCE4..."

# Check if we're in the right environment
if ! command -v xfce4-panel &> /dev/null; then
    echo "❌ Error: XFCE4 panel not found. This script is designed for XFCE4 desktop environment."
    exit 1
fi

# Check if ROS2 is available
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "❌ Error: ROS2 Humble not found at /opt/ros/humble/setup.bash"
    exit 1
fi

# Check if argo_battery_water node is running
if ! ros2 service list 2>/dev/null | grep -q "/battery_status"; then
    echo "⚠️  Warning: Battery service not available. Make sure the argo_battery_water node is running."
    echo "   You can start it with: python3 $PROJECT_ROOT/nodes/argo_battery_water.py"
fi

echo "✅ Environment checks passed"

# Test the battery status display
echo "🧪 Testing battery status display..."
if source /opt/ros/humble/setup.bash && python3 "$SCRIPT_DIR/battery_status_simple.py"; then
    echo "✅ Battery status display test successful"
else
    echo "❌ Battery status display test failed"
    exit 1
fi

echo ""
echo "🎯 Setup Instructions for XFCE4 Panel:"
echo ""
echo "Now that the Generic Monitor plugin is installed, here are the setup options:"
echo ""
echo "📋 Option 1: Generic Monitor Plugin (Recommended)"
echo ""
echo "1. Right-click on the XFCE4 panel and select 'Panel' → 'Add New Items...'"
echo ""
echo "2. In the 'Add New Items' dialog, select 'Generic Monitor' and click 'Add'"
echo ""
echo "3. Right-click on the newly added Generic Monitor and select 'Properties'"
echo ""
echo "4. In the Generic Monitor properties:"
echo "   - Command: $SCRIPT_DIR/battery_status_genmon_wrapper.sh"
echo "   - Update interval: 10 seconds (or your preference)"
echo "   - Label: Leave empty or set to 'Battery'"
echo ""
echo "5. Click 'Close' to save the configuration"
echo ""
echo "📋 Option 2: Terminal Command"
echo ""
echo "You can always check battery status manually:"
echo "   $SCRIPT_DIR/battery_status_wrapper.sh"
echo ""
echo "🔧 Troubleshooting:"
echo ""
echo "If the battery status doesn't appear:"
echo "1. Check that the argo_battery_water node is running:"
echo "   ros2 service list | grep battery_status"
echo ""
echo "2. Test the genmon script output:"
echo "   $SCRIPT_DIR/battery_status_genmon_wrapper.sh"
echo ""
echo "3. Test the simple status display:"
echo "   $SCRIPT_DIR/battery_status_wrapper.sh"
echo ""
echo "4. Check ROS2 environment is sourced:"
echo "   source /opt/ros/humble/setup.bash"
echo ""
echo "5. Verify the argo_battery_water node is healthy:"
echo "   ros2 service call /battery_status std_srvs/srv/Trigger"
echo ""
echo "✨ Battery status panel setup complete!"
echo ""
echo "The battery status will show:"
echo "🔋 7.2V (45%) 🔌⚡  - Normal battery with charging and AC power"
echo "🔴 6.8V (15%) 🔋    - Low battery warning"
echo "💧 7.1V (42%) 🔌    - Saltwater intrusion detected"
echo "💦 7.3V (48%) ⚡    - High humidity detected"
echo ""

