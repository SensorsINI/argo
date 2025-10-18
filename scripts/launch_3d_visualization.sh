#!/bin/bash
# Launch 3D Visualization for Argo
# =================================
# This script starts the transform publisher and boat visualization nodes
# needed for 3D visualization in Foxglove alongside the main Argo system.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "🚢 Starting Argo 3D Visualization System..."

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Change to project directory
cd "$PROJECT_ROOT"

# Function to start a node in background
start_node() {
    local node_script="$1"
    local node_name="$2"
    
    if [ -f "$node_script" ]; then
        echo "Starting $node_name..."
        python3 "$node_script" &
        echo "  ✅ $node_name started (PID: $!)"
    else
        echo "  ❌ Error: $node_script not found"
        return 1
    fi
}

# Start the sailing area publisher (if not already running)
if ! pgrep -f "sailing_area_publisher.py" > /dev/null; then
    start_node "nodes/sailing_area_publisher.py" "Sailing Area Publisher"
else
    echo "  ℹ️  Sailing Area Publisher already running"
fi

# Start the transform publisher
start_node "nodes/argo_transform_publisher.py" "Transform Publisher"

# Start the boat visualization node
start_node "nodes/argo_boat_visualization.py" "Boat Visualization"

echo ""
echo "🎯 3D Visualization nodes started!"
echo ""
echo "📋 Next steps:"
echo "   1. Start the main Argo system: ./launch/argo_start.sh"
echo "   2. Open Foxglove Studio"
echo "   3. Import layout: foxglove/argo_3d_map.json"
echo "   4. Connect to rosbridge on port 9090"
echo ""
echo "🔍 To check running nodes:"
echo "   ros2 node list"
echo ""
echo "📊 To view visualization topics:"
echo "   ros2 topic list | grep -E '(visualization|sailing|tf)'"
echo ""
echo "🛑 To stop visualization nodes:"
echo "   pkill -f 'argo_transform_publisher.py'"
echo "   pkill -f 'argo_boat_visualization.py'"
echo ""

# Wait for user input to stop
echo "Press Ctrl+C to stop all visualization nodes..."
wait

